from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("openseek_frgraph_ros2"),
        "config",
        "depth_planar_to_pointcloud.yaml",
    ])
    return LaunchDescription([
        Node(
            package="openseek_frgraph_ros2",
            executable="depth_planar_to_pointcloud_node",
            name="depth_planar_to_pointcloud",
            output="screen",
            parameters=[config],
        ),
        Node(
            package="openseek_frgraph_ros2",
            executable="frgraph_planner_manager_node",
            name="frgraph_planner_manager",
            output="screen",
            remappings=[("/velodyne_points", "/frgraph/points")],
            parameters=[{
                "size_of_cropped_pointcloud": 20.0,
                "planner/collision_check_radius": 0.20,
                "odom_topic": "/sim/odom",
                "goal_topic": "/goal",
                "goal_alias_topic": "/goal_pose",
                "use_tf_odom": False,
                "lidar/min_elev_angle": -30.9637565,
                "lidar/max_elev_angle": 30.9637565,
                "gap_extractor/3D/range_map_width": 1600,
                "gap_extractor/3D/range_map_height": 96,
                "gap_extractor/3D/map_size": 20,
                "trajectory/num_of_yaw_samples": 18,
                "trajectory/num_of_roll_samples": 5,
                "trajectory/num_of_pitch_samples": 5,
            }],
        ),
    ])
