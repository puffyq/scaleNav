#!/usr/bin/env python3
"""Exercise empty depth frames against the real graph node on an isolated ROS domain."""

import argparse
from array import array
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image, PointCloud2, PointField
from std_msgs.msg import String
from visualization_msgs.msg import MarkerArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--node-binary", type=Path, required=True)
    parser.add_argument("--domain-id", type=int, default=173)
    parser.add_argument("--semantic", action="store_true")
    parser.add_argument("--free-rays", action="store_true")
    parser.add_argument("--gcn-model", type=Path)
    args = parser.parse_args()
    os.environ["ROS_DOMAIN_ID"] = str(args.domain_id)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    with tempfile.TemporaryDirectory(prefix="scalenav-empty-cloud-") as directory:
        os.environ["ROS_LOG_DIR"] = directory
        rclpy.init()
        node = rclpy.create_node("empty_cloud_regression")
        odom_pub = node.create_publisher(Odometry, "/sim/odom", 20)
        cloud_pub = node.create_publisher(PointCloud2, "/depth/points", qos_profile_sensor_data)
        free_pub = node.create_publisher(PointCloud2, "/depth/free_rays", qos_profile_sensor_data)
        goal_pub = node.create_publisher(PoseStamped, "/goal", 10)
        semantic_pub = node.create_publisher(Image, "/scalenav/text_heatmap_raw", 10)
        timings = []
        goals = []
        skeleton = []
        semantic_points = []
        semantic_counts = []

        def collect_graph(message):
            for marker in message.markers:
                if marker.ns == "scalenav_skeleton_nodes":
                    skeleton[:] = marker.points
                elif marker.ns == "scalenav_current_semantic_points":
                    semantic_points[:] = marker.points
                    semantic_counts.append(len(marker.points))

        node.create_subscription(MarkerArray, "/scalenav/graph", collect_graph, 10)

        def collect_cloud_timing(message):
            if '"module":"cloud"' not in message.data and '"switch_reason"' not in message.data:
                return
            try:
                record = json.loads(message.data)
            except json.JSONDecodeError:
                return
            if record.get("module") == "cloud":
                timings.append(record)

        node.create_subscription(String, "/scalenav/timing", collect_cloud_timing, 100)
        node.create_subscription(PoseStamped, "/scalenav/local_goal",
                                 lambda message: goals.append(message.pose.position.y), 100)
        command = [str(args.node_binary.resolve()), "--ros-args", "-p", "graph_fixed_layer:=true",
                   "-p", "graph_layer_z:=60.6", "-p", "wait_for_initial_semantic:=false",
                   "-p", "map_history_radius_m:=40.0",
                   "-p", "semantic_heatmap_topic:=/scalenav/text_heatmap_raw"]
        if args.gcn_model:
            command += ["-p", "gcn_frontier_column_topic:=/scalenav/gcn_frontier_column",
                        "-p", "gcn_frontier_required:=true"]
        with open(Path(directory) / "node.log", "w+") as log:
            process = subprocess.Popen(command, cwd=directory, stdout=log, stderr=log)
            policy = None
            try:
                if args.gcn_model:
                    policy_script = Path(__file__).resolve().parents[3] / "scalenav/gcn_frontier_policy_ros2.py"
                    policy_env = os.environ.copy()
                    policy_env["PYTHONPATH"] = os.pathsep.join(
                        [str(policy_script.parents[3] / "train_gcn"),
                         policy_env.get("PYTHONPATH", "")])
                    policy = subprocess.Popen(
                        [sys.executable, str(policy_script), "--model", str(args.gcn_model.resolve()),
                         "--device", "cpu", "--mission-goal-topic", "/goal"],
                        cwd=directory, stdout=log, stderr=log, env=policy_env)
                started = time.monotonic()
                goal_sent = False
                last_semantic_time = -1.0
                while time.monotonic() - started < 22.0:
                    assert process.poll() is None, "graph node exited"
                    if policy:
                        assert policy.poll() is None, "GCN policy exited"
                    elapsed = time.monotonic() - started
                    odom = Odometry()
                    odom.header.stamp = node.get_clock().now().to_msg()
                    odom.header.frame_id = "world_enu"
                    odom.child_frame_id = "base_link"
                    odom.pose.pose.position.y = min(80.0, max(0.0, elapsed - 5.0) * 5.0)
                    odom.pose.pose.position.z = 60.6
                    odom.twist.twist.linear.y = 5.0 if 5.0 < elapsed < 21.0 else 0.0
                    if args.free_rays and elapsed >= 5.0:
                        assert goals, "no local goal while the vehicle was stationary"
                    odom.pose.pose.orientation.z = math.sqrt(0.5)
                    odom.pose.pose.orientation.w = math.sqrt(0.5)
                    odom_pub.publish(odom)
                    cloud = PointCloud2()
                    cloud.header.stamp = odom.header.stamp
                    cloud.header.frame_id = "base_link"
                    cloud.height = 1
                    cloud.point_step = 12
                    cloud.fields = [PointField(name=name, offset=offset,
                                               datatype=PointField.FLOAT32, count=1)
                                    for name, offset in (("x", 0), ("y", 4), ("z", 8))]
                    cloud_pub.publish(cloud)
                    if args.free_rays:
                        rays = PointCloud2()
                        rays.header = cloud.header
                        rays.height = 1
                        rays.fields = cloud.fields
                        rays.point_step = 12
                        coordinates = [coordinate for lateral in range(-18, 19, 2)
                                       for vertical in (-2.0, 0.0, 2.0)
                                       for coordinate in (20.5, float(lateral), vertical - 0.1)]
                        rays.width = len(coordinates) // 3
                        rays.row_step = rays.width * rays.point_step
                        rays.data = array("f", coordinates).tobytes()
                        free_pub.publish(rays)
                    if args.semantic and elapsed - last_semantic_time >= 0.25:
                        heatmap = Image()
                        heatmap.header = odom.header
                        heatmap.height = 96
                        heatmap.width = 160
                        heatmap.encoding = "32FC1"
                        heatmap.step = 160 * 4
                        heatmap.data = array("f", [0.05 if 64 <= column < 96 else 0.3
                                                   for _ in range(96)
                                                   for column in range(160)]).tobytes()
                        semantic_pub.publish(heatmap)
                        last_semantic_time = elapsed
                    expected_goal_subscribers = 2 if policy else 1
                    if (elapsed >= 1.0 and not goal_sent and
                            goal_pub.get_subscription_count() >= expected_goal_subscribers):
                        goal = PoseStamped()
                        goal.header = odom.header
                        goal.pose.position.y = 140.0
                        goal.pose.position.z = 60.6
                        goal.pose.orientation.w = 1.0
                        goal_pub.publish(goal)
                        goal_sent = True
                    rclpy.spin_once(node, timeout_sec=0.02)
                    time.sleep(0.03)
                cloud_updates = [item for item in timings if item.get("module") == "cloud"]
                assert len(cloud_updates) > 20, f"only {len(cloud_updates)} empty cloud updates"
                assert all(item["input_points"] == 0 and item["map_points"] == 0
                           for item in cloud_updates), "free rays became obstacles"
                if not args.free_rays:
                    assert not skeleton, "graph created without geometric observations or free rays"
                    assert not goals, "local goal published without a geometric graph"
                    print(f"PASS: {len(cloud_updates)} empty frames processed; "
                          "no route created without free-space evidence")
                    return
                assert goals and goals[-1] > 80.0, f"frontier stopped advancing: {goals[-5:]}"
                assert any(point.y > 80.0 for point in skeleton), "no graph nodes ahead of final odom"
                forward_nodes = [point for point in skeleton if 80.0 < point.y < 102.0]
                lateral_width = (max(point.x for point in forward_nodes) -
                                 min(point.x for point in forward_nodes)) if forward_nodes else 0.0
                if args.free_rays:
                    assert lateral_width > 20.0, f"graph still a narrow strip: {lateral_width:.2f} m"
                    assert any(point.x < -8.0 for point in forward_nodes)
                    assert any(point.x > 8.0 for point in forward_nodes)
                if args.semantic:
                    assert not semantic_points, "virtual semantic frontier points are still published"
                log.flush()
                log.seek(0)
                output = log.read()
                assert "[background incremental]" in output, "topology never refreshed"
                assert output.count("[ScaleNav free-space]") <= 2, "bootstrap goals override graph goals"
                if args.semantic:
                    assert "virtual_frontiers=0" in output, "virtual semantic frontiers were not disabled"
                    assert "measured=0 virtual=0" in output, "fixed-depth semantic points still exist"
                if policy:
                    assert "[ScaleNav GCN] frontier column=" in output, "no GCN direction received"
                print(f"PASS: {len(cloud_updates)} empty frames processed; "
                      f"{len(skeleton)} graph nodes; {len(semantic_points)} semantic points; "
                      f"forward graph width={lateral_width:.2f} m; final local goal y={goals[-1]:.2f}")
            except Exception:
                log.flush()
                log.seek(0)
                print(log.read()[-6000:])
                raise
            finally:
                if policy and policy.poll() is None:
                    policy.send_signal(signal.SIGINT)
                    try:
                        policy.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        policy.kill()
                        policy.wait()
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait()
                node.destroy_node()
                rclpy.shutdown()


if __name__ == "__main__":
    main()
