from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from pathlib import Path


def generate_launch_description():
    package_share = get_package_share_directory("scalenav_log")
    if "/install/" in package_share:
        workspace_root = Path(package_share.split("/install/", 1)[0])
        default_output_dir = str(workspace_root.parent / "log_scalenav")
    else:
        default_output_dir = "~/scalenav_logs"
    return LaunchDescription([
        DeclareLaunchArgument(
            "output_dir",
            default_value=default_output_dir,
            description="Root directory for automatically recorded sessions",
        ),
        Node(
            package="scalenav_log",
            executable="scalenav_log_node",
            name="scalenav_log_node",
            output="screen",
            parameters=[{"output_dir": LaunchConfiguration("output_dir")}],
        ),
    ])
