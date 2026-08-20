from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = LaunchConfiguration("config")
    ros_ip = LaunchConfiguration("ros_ip")
    ros_tcp_port = LaunchConfiguration("ros_tcp_port")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "config",
                default_value=PathJoinSubstitution(
                    [FindPackageShare("openseek_uav_sim"), "config", "uav_sim.yaml"]
                ),
                description="Optional UAV simulator YAML file.",
            ),
            DeclareLaunchArgument("ros_ip", default_value="0.0.0.0"),
            DeclareLaunchArgument("ros_tcp_port", default_value="10000"),
            Node(
                package="ros_tcp_endpoint",
                executable="default_server_endpoint",
                name="unity_endpoint",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"ROS_IP": ros_ip},
                    {"ROS_TCP_PORT": ParameterValue(ros_tcp_port, value_type=int)},
                ],
            ),
            Node(
                package="openseek_uav_sim",
                executable="uav_sim_node",
                name="openseek_uav_sim",
                output="screen",
                emulate_tty=True,
                parameters=[config],
            ),
        ]
    )
