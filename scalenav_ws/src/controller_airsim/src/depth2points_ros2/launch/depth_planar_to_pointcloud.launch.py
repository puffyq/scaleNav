from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="depth2points_ros2",
            executable="depth_planar_to_pointcloud_node",
            name="depth_planar_to_pointcloud",
            output="screen",
            parameters=[
                PathJoinSubstitution([
                    FindPackageShare("depth2points_ros2"),
                    "config",
                    "depth_planar_to_pointcloud.yaml",
                ])
            ],
        )
    ])
