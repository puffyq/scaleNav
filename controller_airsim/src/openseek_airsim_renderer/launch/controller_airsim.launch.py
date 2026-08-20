from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("openseek_airsim_renderer")
    controller_share = FindPackageShare("openseek_uav_sim")
    controller_config = LaunchConfiguration("controller_config")
    renderer_config = LaunchConfiguration("renderer_config")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller_config",
                default_value=PathJoinSubstitution(
                    [controller_share, "config", "uav_sim.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "renderer_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "renderer.yaml"]
                ),
            ),
            Node(
                package="openseek_uav_sim",
                executable="uav_sim_node",
                name="openseek_uav_sim",
                output="screen",
                parameters=[controller_config],
            ),
            Node(
                package="openseek_airsim_renderer",
                executable="airsim_renderer_node",
                name="openseek_airsim_renderer",
                output="screen",
                parameters=[renderer_config],
            ),
        ]
    )
