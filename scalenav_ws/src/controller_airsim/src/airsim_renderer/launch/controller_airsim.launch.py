from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    package_share = FindPackageShare("airsim_renderer")
    controller_share = FindPackageShare("uav_sim")
    controller_config = LaunchConfiguration("controller_config")
    maximum_linear_speed = LaunchConfiguration("maximum_linear_speed")
    renderer_config = LaunchConfiguration("renderer_config")
    ignore_collision = LaunchConfiguration("ignore_collision")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "controller_config",
                default_value=PathJoinSubstitution(
                    [controller_share, "config", "uav_sim.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "maximum_linear_speed",
                default_value="6.0",
                description="Physical speed cap; start.sh keeps this equal to YOPO's trajectory cap",
            ),
            DeclareLaunchArgument(
                "renderer_config",
                default_value=PathJoinSubstitution(
                    [package_share, "config", "renderer.yaml"]
                ),
            ),
            DeclareLaunchArgument(
                "ignore_collision",
                default_value="true",
                description="Allow AirSim pose updates to pass through collisions",
            ),
            Node(
                package="uav_sim",
                executable="uav_sim_node",
                name="uav_sim",
                output="screen",
                parameters=[
                    controller_config,
                    {
                        "maximum_linear_speed": ParameterValue(
                            maximum_linear_speed, value_type=float
                        )
                    },
                ],
            ),
            Node(
                package="airsim_renderer",
                executable="airsim_renderer_node",
                name="airsim_renderer",
                output="screen",
                parameters=[renderer_config, {"ignore_collision": ignore_collision}],
            ),
        ]
    )
