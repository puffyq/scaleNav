from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    names = (
        ("depth_topic", "/camera/depth/image"),
        ("camera_info_topic", "/camera/depth/camera_info"),
        ("pointcloud_topic", "/depth/points"),
        ("free_ray_topic", "/depth/free_rays"),
        ("output_frame", "base_link"),
        ("fx", "0.0"),
        ("fy", "0.0"),
        ("cx", "0.0"),
        ("cy", "0.0"),
        ("horizontal_fov_deg", "90.0"),
        ("vertical_fov_deg", "60.0"),
    )
    return LaunchDescription([
        *(DeclareLaunchArgument(name, default_value=value) for name, value in names),
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
                ]),
                {name: LaunchConfiguration(name) for name, _ in names},
            ],
        )
    ])
