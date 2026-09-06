from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("scalenav_graph_ros2"), "config", "config.yaml"])),
        DeclareLaunchArgument("cloud_topic", default_value="/depth/points"),
        DeclareLaunchArgument("free_ray_topic", default_value="/depth/free_rays"),
        DeclareLaunchArgument("semantic_heatmap_topic", default_value="/scalenav/text_heatmap_raw"),
        DeclareLaunchArgument("semantic_depth_topic", default_value="/camera/depth/image"),
        DeclareLaunchArgument("odom_topic", default_value="/sim/odom"),
        DeclareLaunchArgument("goal_topic", default_value="/goal"),
        DeclareLaunchArgument("next_goal_topic", default_value="/scalenav/local_goal"),
        DeclareLaunchArgument("frontier_goal_topic", default_value="/scalenav/frontier_goal"),
        DeclareLaunchArgument("gcn_frontier_column_topic", default_value=""),
        DeclareLaunchArgument("gcn_frontier_required", default_value="false"),
        DeclareLaunchArgument("gcn_frontier_timeout_ms", default_value="1000.0"),
        DeclareLaunchArgument("gcn_frontier_direction_weight", default_value="20.0"),
        DeclareLaunchArgument("timing_topic", default_value="/scalenav/timing"),
        DeclareLaunchArgument("next_goal_frame", default_value="world_enu"),
        DeclareLaunchArgument("visualization_frame", default_value="odom"),
        DeclareLaunchArgument("odom_twist_frame", default_value="world"),
        DeclareLaunchArgument(
            "flight_statistics_file",
            default_value="scalenav_flight_statistics.csv"),
        DeclareLaunchArgument(
            "graph_log_file",
            default_value="scalenav_graph_snapshots.jsonl"),
        DeclareLaunchArgument("trajectory_speed_color_max_mps", default_value="6.0"),
        DeclareLaunchArgument("trajectory_max_points", default_value="50000"),
        DeclareLaunchArgument("graph_fixed_layer", default_value="true"),
        DeclareLaunchArgument("graph_layer_z", default_value="1.6"),
        DeclareLaunchArgument("reuse_graph_on_goal", default_value="true"),
        DeclareLaunchArgument("reuse_previous_route", default_value="false"),
        DeclareLaunchArgument("map_margin", default_value="50.0"),
        DeclareLaunchArgument("map_voxel_size", default_value="0.1"),
        # Keep observed static obstacles while semantic frontier edges remain
        # eligible.  A current-frame-only KD-tree can forget an occluded wall
        # and immediately recreate an ordinary-semantic edge that was just
        # removed as blocked.
        DeclareLaunchArgument("map_history_radius_m", default_value="40.0"),
        DeclareLaunchArgument("map_max_points", default_value="100000"),
        DeclareLaunchArgument("map_prune_distance_m", default_value="0.5"),
        DeclareLaunchArgument("update_period_ms", default_value="100"),
        DeclareLaunchArgument("skeleton_rebuild_period_ms", default_value="100.0"),
        DeclareLaunchArgument("local_goal_min_advance_m", default_value="0.75"),
        DeclareLaunchArgument("local_goal_lookahead_m", default_value="15.0"),
        DeclareLaunchArgument("frontier_goal_margin_m", default_value="3.5"),
        DeclareLaunchArgument("max_update_region_num", default_value="0"),
        # Compatibility arguments; the planner publishes every update tick.
        DeclareLaunchArgument("route_plan_period_ms", default_value="100"),
        DeclareLaunchArgument("local_goal_reserve_m", default_value="0.0"),
        DeclareLaunchArgument("local_graph_radius_m", default_value="45.0"),
        DeclareLaunchArgument("local_sliding_graph", default_value="false"),
        DeclareLaunchArgument("local_sliding_graph_radius_m", default_value="40.0"),
        DeclareLaunchArgument("use_edge_witness_path", default_value="false"),
        DeclareLaunchArgument("goal_path_cost_weight", default_value="1.0"),
        DeclareLaunchArgument("frontier_goal_distance_weight", default_value="2.0"),
        DeclareLaunchArgument("frontier_direction_loss_weight", default_value="0.35"),
        DeclareLaunchArgument("frontier_semantic_score_weight", default_value="1.0"),
        DeclareLaunchArgument("frontier_semantic_detour_budget_m", default_value="45.0"),
        DeclareLaunchArgument("frontier_semantic_frame_budget_m", default_value="12.0"),
        DeclareLaunchArgument("frontier_semantic_noise_floor", default_value="0.08"),
        DeclareLaunchArgument("semantic_cost_weight", default_value="2.0"),
        DeclareLaunchArgument("semantic_route_switch_risk_margin", default_value="0.08"),
        DeclareLaunchArgument("semantic_route_switch_cost_ratio", default_value="0.90"),
        DeclareLaunchArgument("semantic_opportunity_persistence_frames", default_value="2"),
        DeclareLaunchArgument("semantic_opportunity_switch_margin_m", default_value="3.0"),
        DeclareLaunchArgument("semantic_opportunity_cooldown_s", default_value="0.8"),
        DeclareLaunchArgument(
            "semantic_opportunity_direction_tolerance_deg", default_value="30.0"),
        DeclareLaunchArgument("semantic_route_influence_m", default_value="8.0"),
        DeclareLaunchArgument("semantic_visualization_max_score", default_value="0.4"),
        DeclareLaunchArgument("semantic_baseline_quantile", default_value="0.25"),
        DeclareLaunchArgument("semantic_point_influence_m", default_value="8.0"),
        DeclareLaunchArgument("semantic_edge_candidate_limit", default_value="8"),
        DeclareLaunchArgument("bubble_astar_safe_distance", default_value="0.61"),
        DeclareLaunchArgument("bubble_astar_clearance_tolerance", default_value="0.20"),
        DeclareLaunchArgument("bubble_topo/clearance_cost_weight", default_value="2.0"),
        DeclareLaunchArgument("bubble_topo/clearance_target_m", default_value="1.2"),
        DeclareLaunchArgument("previous_path_cost_factor", default_value="1.0"),
        DeclareLaunchArgument("route_remap_distance_m", default_value="2.0"),
        DeclareLaunchArgument("route_reuse_horizon_m", default_value="10.0"),
        DeclareLaunchArgument("route_reuse_lateral_distance_m", default_value="1.5"),
        DeclareLaunchArgument("local_goal_hold_timeout_ms", default_value="400.0"),
        DeclareLaunchArgument("stuck_replan_timeout_ms", default_value="2000.0"),
        DeclareLaunchArgument("stuck_replan_speed_mps", default_value="0.20"),
        DeclareLaunchArgument("stuck_replan_min_goal_distance_m", default_value="3.0"),
        DeclareLaunchArgument("stuck_replan_cooldown_s", default_value="5.0"),
        DeclareLaunchArgument("frontier_extension_search_period_ms", default_value="1000.0"),
        DeclareLaunchArgument("goal_connect_distance_m", default_value="6.0"),
        DeclareLaunchArgument("goal_connect_timeout_ms", default_value="20.0"),
        DeclareLaunchArgument("odom_reconnect_distance_m", default_value="1.0"),
        DeclareLaunchArgument("odom_reconnect_yaw_deg", default_value="20.0"),
        DeclareLaunchArgument("odom_fallback_radius_m", default_value="15.0"),
        DeclareLaunchArgument("odom_fallback_candidates", default_value="8"),
        DeclareLaunchArgument("odom_connect_timeout_ms", default_value="3.0"),
        DeclareLaunchArgument("semantic_pose_tolerance_ms", default_value="250.0"),
        DeclareLaunchArgument("semantic_depth_tolerance_ms", default_value="50.0"),
        DeclareLaunchArgument("semantic_depth_max_m", default_value="20.0"),
        DeclareLaunchArgument("semantic_max_age_ms", default_value="1500.0"),
        DeclareLaunchArgument("semantic_risk_memory_ms", default_value="5000.0"),
        DeclareLaunchArgument("semantic_risk_accumulation_alpha", default_value="0.25"),
        DeclareLaunchArgument("wait_for_initial_semantic", default_value="true"),
        DeclareLaunchArgument("initial_semantic_wait_timeout_ms", default_value="5000.0"),
        DeclareLaunchArgument("semantic_virtual_depth_m", default_value="35.0"),
        DeclareLaunchArgument("semantic_points_enabled", default_value="true"),
        DeclareLaunchArgument("semantic_point_min_score", default_value="0.20"),
        DeclareLaunchArgument("semantic_point_separation_m", default_value="1.5"),
        DeclareLaunchArgument("semantic_point_radius_m", default_value="0.75"),
        DeclareLaunchArgument("semantic_point_max_nodes", default_value="16"),
        DeclareLaunchArgument("virtual_semantic_prune_enabled", default_value="true"),
        DeclareLaunchArgument("virtual_semantic_backtrack_margin_m", default_value="12.0"),
        DeclareLaunchArgument("virtual_semantic_max_nodes", default_value="512"),
        DeclareLaunchArgument("semantic_label_max_nodes", default_value="16"),
        DeclareLaunchArgument("semantic_camera_translation_flu.x", default_value="0.5"),
        DeclareLaunchArgument("semantic_camera_translation_flu.y", default_value="0.0"),
        DeclareLaunchArgument("semantic_camera_translation_flu.z", default_value="-0.1"),
        DeclareLaunchArgument("semantic_horizontal_fov_deg", default_value="90.0"),
        DeclareLaunchArgument("semantic_vertical_fov_deg", default_value="60.0"),
        DeclareLaunchArgument("semantic_patch_cols", default_value="5"),
        DeclareLaunchArgument("semantic_patch_rows", default_value="3"),
        Node(
            package="scalenav_graph_ros2",
            executable="scalenav_graph_node",
            name="scalenav_graph",
            output="screen",
            parameters=[LaunchConfiguration("config_file"), {
                "cloud_topic": LaunchConfiguration("cloud_topic"),
                "free_ray_topic": LaunchConfiguration("free_ray_topic"),
                "semantic_heatmap_topic": LaunchConfiguration("semantic_heatmap_topic"),
                "semantic_depth_topic": LaunchConfiguration("semantic_depth_topic"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "goal_topic": LaunchConfiguration("goal_topic"),
                "next_goal_topic": LaunchConfiguration("next_goal_topic"),
                "frontier_goal_topic": LaunchConfiguration("frontier_goal_topic"),
                "gcn_frontier_column_topic": LaunchConfiguration(
                    "gcn_frontier_column_topic"),
                "gcn_frontier_required": LaunchConfiguration("gcn_frontier_required"),
                "gcn_frontier_timeout_ms": LaunchConfiguration("gcn_frontier_timeout_ms"),
                "gcn_frontier_direction_weight": LaunchConfiguration(
                    "gcn_frontier_direction_weight"),
                "timing_topic": LaunchConfiguration("timing_topic"),
                "next_goal_frame": LaunchConfiguration("next_goal_frame"),
                "visualization_frame": LaunchConfiguration("visualization_frame"),
                "odom_twist_frame": LaunchConfiguration("odom_twist_frame"),
                "flight_statistics_file": LaunchConfiguration("flight_statistics_file"),
                "graph_log_file": LaunchConfiguration("graph_log_file"),
                "trajectory_speed_color_max_mps": LaunchConfiguration(
                    "trajectory_speed_color_max_mps"),
                "trajectory_max_points": LaunchConfiguration("trajectory_max_points"),
                "graph_fixed_layer": LaunchConfiguration("graph_fixed_layer"),
                "graph_layer_z": LaunchConfiguration("graph_layer_z"),
                "reuse_graph_on_goal": LaunchConfiguration("reuse_graph_on_goal"),
                "reuse_previous_route": LaunchConfiguration("reuse_previous_route"),
                "map_margin": LaunchConfiguration("map_margin"),
                "map_voxel_size": LaunchConfiguration("map_voxel_size"),
                "map_history_radius_m": LaunchConfiguration("map_history_radius_m"),
                "map_max_points": LaunchConfiguration("map_max_points"),
                "map_prune_distance_m": LaunchConfiguration("map_prune_distance_m"),
                "update_period_ms": LaunchConfiguration("update_period_ms"),
                "skeleton_rebuild_period_ms": LaunchConfiguration("skeleton_rebuild_period_ms"),
                "local_goal_min_advance_m": LaunchConfiguration("local_goal_min_advance_m"),
                "local_goal_lookahead_m": LaunchConfiguration("local_goal_lookahead_m"),
                "frontier_goal_margin_m": LaunchConfiguration("frontier_goal_margin_m"),
                "max_update_region_num": LaunchConfiguration("max_update_region_num"),
                "route_plan_period_ms": LaunchConfiguration("route_plan_period_ms"),
                "local_goal_reserve_m": LaunchConfiguration("local_goal_reserve_m"),
                "local_graph_radius_m": LaunchConfiguration("local_graph_radius_m"),
                "local_sliding_graph": LaunchConfiguration("local_sliding_graph"),
                "local_sliding_graph_radius_m": LaunchConfiguration(
                    "local_sliding_graph_radius_m"),
                "use_edge_witness_path": LaunchConfiguration("use_edge_witness_path"),
                "goal_path_cost_weight": LaunchConfiguration("goal_path_cost_weight"),
                "frontier_goal_distance_weight": LaunchConfiguration(
                    "frontier_goal_distance_weight"),
                "frontier_direction_loss_weight": LaunchConfiguration(
                    "frontier_direction_loss_weight"),
                "frontier_semantic_score_weight": LaunchConfiguration(
                    "frontier_semantic_score_weight"),
                "frontier_semantic_detour_budget_m": LaunchConfiguration(
                    "frontier_semantic_detour_budget_m"),
                "frontier_semantic_frame_budget_m": LaunchConfiguration(
                    "frontier_semantic_frame_budget_m"),
                "frontier_semantic_noise_floor": LaunchConfiguration(
                    "frontier_semantic_noise_floor"),
                "semantic_cost_weight": LaunchConfiguration("semantic_cost_weight"),
                "semantic_route_switch_risk_margin": LaunchConfiguration(
                    "semantic_route_switch_risk_margin"),
                "semantic_route_switch_cost_ratio": LaunchConfiguration(
                    "semantic_route_switch_cost_ratio"),
                "semantic_opportunity_persistence_frames": LaunchConfiguration(
                    "semantic_opportunity_persistence_frames"),
                "semantic_opportunity_switch_margin_m": LaunchConfiguration(
                    "semantic_opportunity_switch_margin_m"),
                "semantic_opportunity_cooldown_s": LaunchConfiguration(
                    "semantic_opportunity_cooldown_s"),
                "semantic_opportunity_direction_tolerance_deg": LaunchConfiguration(
                    "semantic_opportunity_direction_tolerance_deg"),
                "semantic_route_influence_m": LaunchConfiguration(
                    "semantic_route_influence_m"),
                "semantic_visualization_max_score": LaunchConfiguration(
                    "semantic_visualization_max_score"),
                "semantic_baseline_quantile": LaunchConfiguration(
                    "semantic_baseline_quantile"),
                "bubble_topo/semantic_point_influence_m": LaunchConfiguration(
                    "semantic_point_influence_m"),
                "bubble_topo/semantic_edge_candidate_limit": LaunchConfiguration(
                    "semantic_edge_candidate_limit"),
                "bubble_astar/safe_distance": LaunchConfiguration(
                    "bubble_astar_safe_distance"),
                "bubble_astar/clearance_tolerance": LaunchConfiguration(
                    "bubble_astar_clearance_tolerance"),
                "bubble_topo/clearance_cost_weight": LaunchConfiguration(
                    "bubble_topo/clearance_cost_weight"),
                "bubble_topo/clearance_target_m": LaunchConfiguration(
                    "bubble_topo/clearance_target_m"),
                "previous_path_cost_factor": LaunchConfiguration("previous_path_cost_factor"),
                "route_remap_distance_m": LaunchConfiguration("route_remap_distance_m"),
                "route_reuse_horizon_m": LaunchConfiguration("route_reuse_horizon_m"),
                "route_reuse_lateral_distance_m": LaunchConfiguration(
                    "route_reuse_lateral_distance_m"),
                "local_goal_hold_timeout_ms": LaunchConfiguration(
                    "local_goal_hold_timeout_ms"),
                "stuck_replan_timeout_ms": LaunchConfiguration("stuck_replan_timeout_ms"),
                "stuck_replan_speed_mps": LaunchConfiguration("stuck_replan_speed_mps"),
                "stuck_replan_min_goal_distance_m": LaunchConfiguration(
                    "stuck_replan_min_goal_distance_m"),
                "stuck_replan_cooldown_s": LaunchConfiguration("stuck_replan_cooldown_s"),
                "frontier_extension_search_period_ms": LaunchConfiguration(
                    "frontier_extension_search_period_ms"),
                "goal_connect_distance_m": LaunchConfiguration("goal_connect_distance_m"),
                "goal_connect_timeout_ms": LaunchConfiguration("goal_connect_timeout_ms"),
                "odom_reconnect_distance_m": LaunchConfiguration("odom_reconnect_distance_m"),
                "odom_reconnect_yaw_deg": LaunchConfiguration("odom_reconnect_yaw_deg"),
                "odom_fallback_radius_m": LaunchConfiguration("odom_fallback_radius_m"),
                "odom_fallback_candidates": LaunchConfiguration("odom_fallback_candidates"),
                "odom_connect_timeout_ms": LaunchConfiguration("odom_connect_timeout_ms"),
                "semantic_pose_tolerance_ms": LaunchConfiguration("semantic_pose_tolerance_ms"),
                "semantic_depth_tolerance_ms": LaunchConfiguration(
                    "semantic_depth_tolerance_ms"),
                "semantic_depth_max_m": LaunchConfiguration("semantic_depth_max_m"),
                "semantic_max_age_ms": LaunchConfiguration("semantic_max_age_ms"),
                "semantic_risk_memory_ms": LaunchConfiguration("semantic_risk_memory_ms"),
                "bubble_topo/semantic_risk_memory_ms": LaunchConfiguration(
                    "semantic_risk_memory_ms"),
                "bubble_topo/semantic_risk_accumulation_alpha": LaunchConfiguration(
                    "semantic_risk_accumulation_alpha"),
                "wait_for_initial_semantic": LaunchConfiguration("wait_for_initial_semantic"),
                "initial_semantic_wait_timeout_ms": LaunchConfiguration(
                    "initial_semantic_wait_timeout_ms"),
                "semantic_virtual_depth_m": LaunchConfiguration("semantic_virtual_depth_m"),
                "semantic_points_enabled": LaunchConfiguration("semantic_points_enabled"),
                "semantic_point_min_score": LaunchConfiguration("semantic_point_min_score"),
                "semantic_point_separation_m": LaunchConfiguration(
                    "semantic_point_separation_m"),
                "semantic_point_radius_m": LaunchConfiguration("semantic_point_radius_m"),
                "semantic_point_max_nodes": LaunchConfiguration("semantic_point_max_nodes"),
                "virtual_semantic_prune_enabled": LaunchConfiguration(
                    "virtual_semantic_prune_enabled"),
                "virtual_semantic_backtrack_margin_m": LaunchConfiguration(
                    "virtual_semantic_backtrack_margin_m"),
                "virtual_semantic_max_nodes": LaunchConfiguration(
                    "virtual_semantic_max_nodes"),
                "semantic_label_max_nodes": LaunchConfiguration(
                    "semantic_label_max_nodes"),
                "semantic_camera_translation_flu.x": LaunchConfiguration(
                    "semantic_camera_translation_flu.x"),
                "semantic_camera_translation_flu.y": LaunchConfiguration(
                    "semantic_camera_translation_flu.y"),
                "semantic_camera_translation_flu.z": LaunchConfiguration(
                    "semantic_camera_translation_flu.z"),
                "semantic_horizontal_fov_deg": LaunchConfiguration(
                    "semantic_horizontal_fov_deg"),
                "semantic_vertical_fov_deg": LaunchConfiguration(
                    "semantic_vertical_fov_deg"),
                "semantic_patch_cols": LaunchConfiguration("semantic_patch_cols"),
                "semantic_patch_rows": LaunchConfiguration("semantic_patch_rows"),
            }],
        ),
    ])
