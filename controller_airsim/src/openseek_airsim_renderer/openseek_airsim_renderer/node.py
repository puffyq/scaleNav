import math
import sys
import threading
import time
from array import array

import numpy as np
import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

from .coordinates import matrix_to_quaternion, ros_pose_to_airsim
from .rpc import MessagePackRpcClient


class AirSimRendererNode(Node):
    def __init__(self):
        super().__init__("openseek_airsim_renderer")
        self.rpc_ip = self.declare_parameter("rpc_ip", "127.0.0.1").value
        self.rpc_port = self.declare_parameter("rpc_port", 41451).value
        self.rpc_timeout = self.declare_parameter("rpc_timeout", 2.0).value
        self.vehicle_name = self.declare_parameter("vehicle_name", "").value
        self.camera_name = self.declare_parameter("camera_name", "0").value
        self.odom_topic = self.declare_parameter("odom_topic", "/sim/odom").value
        color_topic = self.declare_parameter(
            "color_topic", "/camera/color/image"
        ).value
        depth_topic = self.declare_parameter(
            "depth_topic", "/camera/depth/image"
        ).value
        color_info_topic = self.declare_parameter(
            "color_info_topic", "/camera/color/camera_info"
        ).value
        depth_info_topic = self.declare_parameter(
            "depth_info_topic", "/camera/depth/camera_info"
        ).value
        collision_topic = self.declare_parameter(
            "collision_topic", "/sim/collision"
        ).value
        self.body_frame = self.declare_parameter("body_frame", "base_link").value
        self.camera_frame = self.declare_parameter(
            "camera_frame", "camera_optical"
        ).value
        self.rgb_encoding = self.declare_parameter("rgb_encoding", "bgr8").value
        self.pose_rate = self.declare_parameter("pose_rate", 60.0).value
        self.render_rate = self.declare_parameter("render_rate", 20.0).value
        self.startup_warmup_frames = self.declare_parameter(
            "startup_warmup_frames", 5
        ).value
        self.horizontal_fov = self.declare_parameter(
            "horizontal_fov_degrees", 90.0
        ).value
        self.ignore_collision = self.declare_parameter(
            "ignore_collision", True
        ).value
        self.pause_simulation = self.declare_parameter(
            "pause_simulation", False
        ).value
        self.airsim_origin_enu = self._vector_parameter(
            "airsim_origin_enu", [0.0, 0.0, 0.0]
        )
        self.camera_translation_flu = self._vector_parameter(
            "camera_translation_flu", [0.0, 0.0, 0.0]
        )
        self._validate_parameters()
        if int(self.startup_warmup_frames) < 0:
            raise ValueError("startup_warmup_frames must be non-negative")
        self.startup_warmup_frames = int(self.startup_warmup_frames)
        self.warmup_frames_remaining = self.startup_warmup_frames

        self.client = MessagePackRpcClient(
            self.rpc_ip, self.rpc_port, self.rpc_timeout
        )
        self.pose_client = MessagePackRpcClient(
            self.rpc_ip, self.rpc_port, self.rpc_timeout
        )
        self.connected = False
        self.pose_connected = False
        self.latest_odom = None
        self.last_pose_odom = None
        self.odom_lock = threading.Lock()
        self.pose_render_lock = threading.Lock()
        self.last_warning = {}
        self.pose_stop = threading.Event()
        self.odom_callback_group = MutuallyExclusiveCallbackGroup()
        self.render_callback_group = MutuallyExclusiveCallbackGroup()

        self.color_pub = self.create_publisher(
            Image, color_topic, qos_profile_sensor_data
        )
        self.depth_pub = self.create_publisher(
            Image, depth_topic, qos_profile_sensor_data
        )
        self.color_info_pub = self.create_publisher(
            CameraInfo, color_info_topic, qos_profile_sensor_data
        )
        self.depth_info_pub = self.create_publisher(
            CameraInfo, depth_info_topic, qos_profile_sensor_data
        )
        self.collision_pub = self.create_publisher(Bool, collision_topic, 10)
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self._on_odometry,
            qos_profile_sensor_data,
            callback_group=self.odom_callback_group,
        )
        self.static_tf = StaticTransformBroadcaster(self)
        self._publish_static_camera_transform()
        self.timer = self.create_timer(
            1.0 / self.render_rate,
            self._render,
            callback_group=self.render_callback_group,
        )
        self.pose_thread = threading.Thread(
            target=self._pose_loop,
            name="airsim_pose_publisher",
            daemon=True,
        )
        self.pose_thread.start()

        self.get_logger().info(
            f"AirSim renderer ready: odom={self.odom_topic}, "
            f"RPC={self.rpc_ip}:{self.rpc_port}, pose={self.pose_rate:.0f} Hz, "
            f"RGB-D={self.render_rate:.0f} Hz"
        )

    def destroy_node(self):
        self.pose_stop.set()
        self.pose_thread.join(timeout=self.rpc_timeout + 1.0)
        if self.connected and self.pause_simulation:
            try:
                self.client.call("simPause", False)
            except Exception:
                pass
        self.pose_client.close()
        self.client.close()
        return super().destroy_node()

    def _vector_parameter(self, name, default):
        value = self.declare_parameter(name, default).value
        if len(value) != 3 or not np.all(np.isfinite(value)):
            raise ValueError(f"{name} must contain three finite values")
        return np.asarray(value, dtype=np.float64)

    def _validate_parameters(self):
        if not 1 <= self.rpc_port <= 65535 or self.rpc_timeout <= 0.0:
            raise ValueError("invalid AirSim RPC parameters")
        if (
            self.pose_rate <= 0.0
            or self.render_rate <= 0.0
            or not 0.0 < self.horizontal_fov < 180.0
        ):
            raise ValueError("invalid pose/render rate or camera FOV")
        if self.rgb_encoding not in ("bgr8", "rgb8"):
            raise ValueError("rgb_encoding must be bgr8 or rgb8")

    def _on_odometry(self, message):
        with self.odom_lock:
            self.latest_odom = message

    def _connect(self):
        if self.connected:
            return True
        try:
            if not self.client.call("ping"):
                raise ConnectionError("AirSim ping returned false")
            if self.pause_simulation:
                self.client.call("simPause", True)
            self.connected = True
            self.warmup_frames_remaining = self.startup_warmup_frames
            self.get_logger().info("Connected to AirSim RPC server")
            return True
        except Exception as error:
            self._warn_throttled("rpc", f"Waiting for AirSim RPC: {error}")
            return False

    def _pose_loop(self):
        period = 1.0 / self.pose_rate
        deadline = time.monotonic()
        while not self.pose_stop.is_set():
            with self.odom_lock:
                odom = self.latest_odom
            if odom is not None:
                try:
                    pose = ros_pose_to_airsim(
                        odom.pose.pose, self.airsim_origin_enu
                    )
                    with self.pose_render_lock:
                        self.pose_client.call(
                            "simSetVehiclePose",
                            pose,
                            self.ignore_collision,
                            self.vehicle_name,
                        )
                        with self.odom_lock:
                            self.last_pose_odom = odom
                    if not self.pose_connected:
                        self.pose_connected = True
                        self.get_logger().info("AirSim high-rate pose publisher connected")
                except ValueError as error:
                    self._warn_throttled(
                        "pose", f"Rejected controller pose: {error}"
                    )
                except Exception as error:
                    self.pose_connected = False
                    self.pose_client.close()
                    self._warn_throttled(
                        "pose_rpc", f"AirSim pose RPC failed: {error}"
                    )

            deadline += period
            delay = deadline - time.monotonic()
            if delay < -period:
                deadline = time.monotonic()
                delay = 0.0
            self.pose_stop.wait(max(0.0, delay))

    def _render(self):
        if not self._connect():
            return

        try:
            requests = [
                [self.camera_name, 0, False, False],
                [self.camera_name, 1, True, False],
            ]
            # Keep the AirSim vehicle pose fixed for the complete camera
            # capture. The returned image is stamped with this exact odometry
            # sample and EPIC later retrieves the same sample from its history.
            with self.pose_render_lock:
                with self.odom_lock:
                    odom = self.last_pose_odom
                if odom is None:
                    self._warn_throttled(
                        "odom", f"Waiting for a published pose from {self.odom_topic}"
                    )
                    return
                responses = self.client.call(
                    "simGetImages", requests, self.vehicle_name, False
                )
            if len(responses) != 2:
                raise RuntimeError("AirSim returned an incomplete RGB-D response")

            if self.warmup_frames_remaining > 0:
                depth = np.asarray(responses[1][1], dtype=np.float32)
                finite_depth = depth[np.isfinite(depth)]
                completed = (
                    self.startup_warmup_frames - self.warmup_frames_remaining + 1
                )
                if finite_depth.size:
                    p01, p50, p99 = np.percentile(finite_depth, [1.0, 50.0, 99.0])
                    self.get_logger().info(
                        f"Discarding AirSim RGB-D warmup frame "
                        f"{completed}/{self.startup_warmup_frames}: "
                        f"depth p01/p50/p99={p01:.3f}/{p50:.3f}/{p99:.3f} m"
                    )
                self.warmup_frames_remaining -= 1
                if self.warmup_frames_remaining == 0:
                    self.get_logger().info(
                        "AirSim RGB-D warmup complete; publishing sensor frames"
                    )
                return

            self._publish_color(responses[0], odom.header.stamp)
            self._publish_depth(responses[1], odom.header.stamp)
            collision = self.client.call("simGetCollisionInfo", self.vehicle_name)
            self.collision_pub.publish(Bool(data=bool(collision[0])))
        except ValueError as error:
            self._warn_throttled("pose", f"Rejected controller pose: {error}")
        except Exception as error:
            self.connected = False
            self.client.close()
            self._warn_throttled("render", f"AirSim render RPC failed: {error}")

    def _publish_color(self, response, stamp):
        width, height, image_type = int(response[9]), int(response[10]), int(response[11])
        if width <= 0 or height <= 0 or image_type != 0:
            raise RuntimeError("invalid AirSim Scene metadata")
        pixels = response[0]
        data = pixels if isinstance(pixels, bytes) else bytes(pixels)
        if len(data) != width * height * 3:
            raise RuntimeError("Scene image is not uncompressed 3-channel data")

        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = self.camera_frame
        message.width = width
        message.height = height
        message.encoding = self.rgb_encoding
        message.is_bigendian = False
        message.step = width * 3
        message.data = data
        self.color_pub.publish(message)
        self.color_info_pub.publish(self._camera_info(stamp, width, height))

    def _publish_depth(self, response, stamp):
        width, height, image_type = int(response[9]), int(response[10]), int(response[11])
        values = response[1]
        if width <= 0 or height <= 0 or image_type != 1:
            raise RuntimeError("invalid AirSim DepthPlanar metadata")
        if len(values) != width * height:
            raise RuntimeError("DepthPlanar float buffer has the wrong size")
        buffer = array("f", values)
        if sys.byteorder != "little":
            buffer.byteswap()

        message = Image()
        message.header.stamp = stamp
        message.header.frame_id = self.camera_frame
        message.width = width
        message.height = height
        message.encoding = "32FC1"
        message.is_bigendian = False
        message.step = width * 4
        message.data = buffer.tobytes()
        self.depth_pub.publish(message)
        self.depth_info_pub.publish(self._camera_info(stamp, width, height))

    def _camera_info(self, stamp, width, height):
        focal_length = (width / 2.0) / math.tan(math.radians(self.horizontal_fov / 2.0))
        message = CameraInfo()
        message.header.stamp = stamp
        message.header.frame_id = self.camera_frame
        message.width = width
        message.height = height
        message.distortion_model = "plumb_bob"
        message.d = [0.0] * 5
        message.k = [focal_length, 0.0, width / 2.0, 0.0, focal_length, height / 2.0, 0.0, 0.0, 1.0]
        message.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        message.p = [focal_length, 0.0, width / 2.0, 0.0, 0.0, focal_length, height / 2.0, 0.0, 0.0, 0.0, 1.0, 0.0]
        return message

    def _publish_static_camera_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.body_frame
        transform.child_frame_id = self.camera_frame
        transform.transform.translation.x = float(self.camera_translation_flu[0])
        transform.transform.translation.y = float(self.camera_translation_flu[1])
        transform.transform.translation.z = float(self.camera_translation_flu[2])
        body_from_optical = np.asarray(
            [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]]
        )
        w, x, y, z = matrix_to_quaternion(body_from_optical)
        transform.transform.rotation.w = float(w)
        transform.transform.rotation.x = float(x)
        transform.transform.rotation.y = float(y)
        transform.transform.rotation.z = float(z)
        self.static_tf.sendTransform(transform)

    def _warn_throttled(self, key, message):
        current = time.monotonic()
        if current - self.last_warning.get(key, -math.inf) >= 5.0:
            self.get_logger().warning(message)
            self.last_warning[key] = current


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = None
    try:
        node = AirSimRendererNode()
        executor = MultiThreadedExecutor(num_threads=2)
        executor.add_node(node)
        executor.spin()
    finally:
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
