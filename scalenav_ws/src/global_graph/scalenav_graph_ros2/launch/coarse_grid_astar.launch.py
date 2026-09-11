from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("cloud_topic", default_value="/depth/points"),
        DeclareLaunchArgument("odom_topic", default_value="/sim/odom"),
        DeclareLaunchArgument("goal_topic", default_value="/goal_pose"),
        DeclareLaunchArgument("next_goal_topic", default_value="/scalenav/local_goal"),
        DeclareLaunchArgument("frame_id", default_value="world_enu"),
        DeclareLaunchArgument("resolution_m", default_value="0.5"),
        DeclareLaunchArgument("inflate_radius_m", default_value="0.61"),
        DeclareLaunchArgument("origin_x_m", default_value="-50.0"),
        DeclareLaunchArgument("origin_y_m", default_value="-10.0"),
        DeclareLaunchArgument("origin_z_m", default_value="0.0"),
        DeclareLaunchArgument("size_x_m", default_value="100.0"),
        DeclareLaunchArgument("size_y_m", default_value="170.0"),
        DeclareLaunchArgument("size_z_m", default_value="10.0"),
        DeclareLaunchArgument("local_goal_lookahead_m", default_value="15.0"),
        DeclareLaunchArgument("local_radius_m", default_value="20.0"),
        DeclareLaunchArgument("plan_period_ms", default_value="100"),
        Node(
            package="scalenav_graph_ros2",
            executable="coarse_grid_astar_node",
            name="coarse_grid_astar",
            output="screen",
            parameters=[{
                "cloud_topic": LaunchConfiguration("cloud_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "goal_topic": LaunchConfiguration("goal_topic"),
                "next_goal_topic": LaunchConfiguration("next_goal_topic"),
                "frame_id": LaunchConfiguration("frame_id"),
                "resolution_m": LaunchConfiguration("resolution_m"),
                "inflate_radius_m": LaunchConfiguration("inflate_radius_m"),
                "origin_x_m": LaunchConfiguration("origin_x_m"),
                "origin_y_m": LaunchConfiguration("origin_y_m"),
                "origin_z_m": LaunchConfiguration("origin_z_m"),
                "size_x_m": LaunchConfiguration("size_x_m"),
                "size_y_m": LaunchConfiguration("size_y_m"),
                "size_z_m": LaunchConfiguration("size_z_m"),
                "local_goal_lookahead_m": LaunchConfiguration("local_goal_lookahead_m"),
                "local_radius_m": LaunchConfiguration("local_radius_m"),
                "plan_period_ms": LaunchConfiguration("plan_period_ms"),
            }],
        ),
    ])
