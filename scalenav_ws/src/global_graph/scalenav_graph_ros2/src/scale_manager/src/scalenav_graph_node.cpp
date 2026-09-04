#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <cstdio>
#include <deque>
#include <functional>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <limits>
#include <memory>
#include <map>
#include <mutex>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <std_msgs/msg/string.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <csignal>
#include <execinfo.h>
#include <unistd.h>

#include "scalenav_graph_ros2/route_memory.hpp"
#include "pointcloud_topo/graph.h"

namespace {

void crashSignalHandler(int signal)
{
  void *frames[64];
  const int frame_count = ::backtrace(frames, 64);
  static constexpr char message[] = "\nScaleNav fatal signal stack:\n";
  const auto ignored = ::write(STDERR_FILENO, message, sizeof(message) - 1);
  (void)ignored;
  ::backtrace_symbols_fd(frames, frame_count, STDERR_FILENO);
  std::signal(signal, SIG_DFL);
  std::raise(signal);
}

geometry_msgs::msg::Point toPoint(const Eigen::Vector3f &value)
{
  geometry_msgs::msg::Point point;
  point.x = value.x();
  point.y = value.y();
  point.z = value.z();
  return point;
}

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d &value)
{
  geometry_msgs::msg::Point point;
  point.x = value.x();
  point.y = value.y();
  point.z = value.z();
  return point;
}

Eigen::Vector3f projectPlanningPoint(
  const Eigen::Vector3f &value, bool fixed_layer, double layer_z)
{
  Eigen::Vector3f projected = value;
  if (fixed_layer) projected.z() = static_cast<float>(layer_z);
  return projected;
}

// Polynomial fitting must receive sufficiently dense samples. Keep the A*
// node path unchanged and only interpolate the fitting inputs.
std::vector<Eigen::Vector3f> densifyPolynomialInputs(
  const std::vector<Eigen::Vector3f> &path, float maximum_spacing_m)
{
  if (path.size() < 2 || !std::isfinite(maximum_spacing_m) || maximum_spacing_m <= 0.0F) {
    return path;
  }
  std::vector<Eigen::Vector3f> dense;
  dense.reserve(path.size());
  dense.push_back(path.front());
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector3f &from = path[i - 1];
    const Eigen::Vector3f &to = path[i];
    const float length = (to - from).norm();
    if (!std::isfinite(length) || length <= 1.0e-3F) {
      if ((dense.back() - to).norm() > 1.0e-3F) dense.push_back(to);
      continue;
    }
    const int segments = std::max(1, static_cast<int>(std::ceil(
      length / maximum_spacing_m)));
    for (int segment = 1; segment <= segments; ++segment) {
      const float t = static_cast<float>(segment) / static_cast<float>(segments);
      dense.push_back(from + t * (to - from));
    }
  }
  return dense;
}

struct Rgb
{
  float r;
  float g;
  float b;
};

// Paper/RViz palette: cool structural colors, warm colors only for decisions
// and risk. Values are exact sRGB hex conversions.
constexpr Rgb kTopology{0.396078F, 0.474510F, 0.521569F};       // #657985
constexpr Rgb kCandidate{0.709804F, 0.756863F, 0.784314F};      // #B5C1C8
constexpr Rgb kSelectedPath{0.0F, 0.486275F, 0.513725F};        // #007C83
constexpr Rgb kPolynomialPath{0.960784F, 0.647059F, 0.101961F}; // #F5A51A
constexpr Rgb kUav{0.141176F, 0.203922F, 0.239216F};            // #24343D
constexpr Rgb kMissionGoal{0.192157F, 0.368627F, 0.470588F};    // #315E78
constexpr Rgb kFrontierGoal{0.835294F, 0.521569F, 0.141176F};   // #D58524
constexpr Rgb kLocalGoal{0.552941F, 0.376471F, 0.568627F};      // #8D6091
constexpr Rgb kRiskLow{0.929412F, 0.952941F, 0.941176F};        // #EDF3F0
constexpr Rgb kRiskMedium{0.850980F, 0.678431F, 0.239216F};     // #D9AD3D
constexpr Rgb kRiskHigh{0.819608F, 0.305882F, 0.274510F};       // #D14E46
constexpr Rgb kSemanticBest{0.180392F, 0.760784F, 0.325490F};  // #2EC254

void setColor(std_msgs::msg::ColorRGBA &color, const Rgb &rgb, float alpha = 1.0F)
{
  color.r = rgb.r;
  color.g = rgb.g;
  color.b = rgb.b;
  color.a = alpha;
}

}  // namespace

class ScaleNavGraphNode final : public rclcpp::Node {
 public:
  ScaleNavGraphNode()
  : Node("scalenav_graph_node"),
    map_(std::make_shared<fast_planner::LIOInterface>()),
    astar_(std::make_shared<ParallelBubbleAstar>()),
    topo_(std::make_shared<TopoGraph>())
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/depth/points");
    free_ray_topic_ = declare_parameter<std::string>("free_ray_topic", "/depth/free_rays");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/sim/odom");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal");
    next_goal_topic_ = declare_parameter<std::string>("next_goal_topic", "/scalenav/local_goal");
    clearance_topic_ = declare_parameter<std::string>("clearance_topic", "/scalenav/clearance");
    timing_topic_ = declare_parameter<std::string>("timing_topic", "/scalenav/timing");
    next_goal_frame_ = declare_parameter<std::string>("next_goal_frame", "world_enu");
    visualization_frame_ = declare_parameter<std::string>("visualization_frame", "odom");
    odom_twist_frame_ = declare_parameter<std::string>("odom_twist_frame", "world");
    flight_statistics_file_ = declare_parameter<std::string>(
      "flight_statistics_file", "scalenav_flight_statistics.csv");
    graph_log_file_ = declare_parameter<std::string>(
      "graph_log_file", "scalenav_graph_snapshots.jsonl");
    trajectory_speed_color_max_mps_ = declare_parameter<double>(
      "trajectory_speed_color_max_mps", 6.0);
    trajectory_max_points_ = static_cast<std::size_t>(std::max(
      1000L, static_cast<long>(declare_parameter<int>("trajectory_max_points", 50000))));
    graph_fixed_layer_ = declare_parameter<bool>("graph_fixed_layer", true);
    graph_layer_z_ = declare_parameter<double>("graph_layer_z", 1.6);
    reuse_graph_on_goal_ = declare_parameter<bool>("reuse_graph_on_goal", true);
    // Compatibility-only parameters. Route planning is always fresh A* from
    // the current odometry node; no previous-route mode is supported.
    (void)declare_parameter<bool>("reuse_previous_route", false);
    map_margin_ = declare_parameter<double>("map_margin", 20.0);
    map_voxel_size_ = declare_parameter<double>("map_voxel_size", 0.1);
    map_history_radius_m_ = declare_parameter<double>("map_history_radius_m", 0.0);
    map_max_points_ = declare_parameter<int>("map_max_points", 20000);
    map_prune_distance_m_ = declare_parameter<double>("map_prune_distance_m", 0.5);
    update_period_ms_ = declare_parameter<int>("update_period_ms", 100);
    diagnostic_log_period_ms_ = std::max(
      250, static_cast<int>(declare_parameter<int>("diagnostic_log_period_ms", 2000)));
    skeleton_rebuild_period_ms_ = declare_parameter<double>("skeleton_rebuild_period_ms", 100.0);
    local_goal_min_advance_m_ = declare_parameter<double>("local_goal_min_advance_m", 0.75);
    local_goal_lookahead_m_ = declare_parameter<double>("local_goal_lookahead_m", 15.0);
    frontier_replan_progress_ratio_ = std::clamp(
      declare_parameter<double>("frontier_replan_progress_ratio", 0.40), 0.0, 1.0);
    // Kept as launch-API compatibility knobs.  Planning is intentionally
    // performed on every update tick; throttling it here made the graph and
    // subgoal stale while the vehicle was moving.
    route_plan_period_ms_ = declare_parameter<int>("route_plan_period_ms", 100);
    local_goal_reserve_m_ = declare_parameter<double>("local_goal_reserve_m", 0.0);
    local_graph_radius_m_ = declare_parameter<double>("local_graph_radius_m", 45.0);
    frontier_goal_margin_m_ = declare_parameter<double>("frontier_goal_margin_m", 3.5);
    frontier_progress_loss_weight_ = declare_parameter<double>(
      "frontier_progress_loss_weight", 0.5);
    frontier_direction_loss_weight_ = declare_parameter<double>(
      "frontier_direction_loss_weight", 0.35);
    frontier_fov_loss_weight_ = declare_parameter<double>(
      "frontier_fov_loss_weight", 0.2);
    frontier_smoothness_loss_weight_ = declare_parameter<double>(
      "frontier_smoothness_loss_weight", 0.35);
    // Downstream trajectory fitting consumes the A* node centers directly.
    // Keep the parameter for launch-file compatibility, but do not expose
    // edge witness geometry as the optimization path.
    use_edge_witness_path_ = declare_parameter<bool>("use_edge_witness_path", false);
    // ScaleNav already stores collision-checked witness paths on each edge.  A
    // second clearance raycast over every published segment is not part of
    // the original planner and can consume most of the route-update period.
    goal_path_cost_weight_ = declare_parameter<double>("goal_path_cost_weight", 1.0);
    frontier_goal_distance_weight_ = std::clamp(declare_parameter<double>(
      "frontier_goal_distance_weight", 2.0), 0.0, 10.0);
    // Compatibility only: semantic endpoint scores are now converted to
    // equivalent metres before the common objective normalization.
    (void)declare_parameter<double>("frontier_semantic_score_weight", 1.0);
    frontier_semantic_detour_budget_m_ = std::max(0.0, declare_parameter<double>(
      "frontier_semantic_detour_budget_m", 45.0));
    frontier_semantic_frame_budget_m_ = std::max(0.0, declare_parameter<double>(
      "frontier_semantic_frame_budget_m", 12.0));
    frontier_semantic_noise_floor_ = std::clamp(declare_parameter<double>(
      "frontier_semantic_noise_floor", 0.08), 1.0e-3, 1.0);
    semantic_cost_weight_ = declare_parameter<double>("semantic_cost_weight", 2.0);
    (void)declare_parameter<double>("semantic_route_switch_risk_margin", 0.08);
    (void)declare_parameter<double>("semantic_route_switch_cost_ratio", 0.90);
    semantic_opportunity_persistence_frames_ = static_cast<int>(std::max<std::int64_t>(
      1, declare_parameter<int>("semantic_opportunity_persistence_frames", 2)));
    semantic_opportunity_switch_margin_m_ = std::max(0.0,
      declare_parameter<double>("semantic_opportunity_switch_margin_m", 3.0));
    semantic_opportunity_cooldown_s_ = std::max(0.0,
      declare_parameter<double>("semantic_opportunity_cooldown_s", 0.8));
    semantic_opportunity_direction_tolerance_deg_ = std::clamp(
      declare_parameter<double>("semantic_opportunity_direction_tolerance_deg", 30.0),
      1.0, 90.0);
    semantic_route_influence_m_ = declare_parameter<double>(
      "semantic_route_influence_m", 8.0);
    semantic_visualization_max_score_ = declare_parameter<double>(
      "semantic_visualization_max_score", 0.4);
    semantic_baseline_quantile_ = declare_parameter<double>(
      "semantic_baseline_quantile", 0.25);
    (void)declare_parameter<double>("previous_path_cost_factor", 1.0);
    (void)declare_parameter<double>("route_remap_distance_m", 1.25);
    (void)declare_parameter<double>("route_reuse_horizon_m", 10.0);
    (void)declare_parameter<double>("route_reuse_lateral_distance_m", 1.5);
    local_goal_hold_timeout_ms_ = declare_parameter<double>(
      "local_goal_hold_timeout_ms", 400.0);
    (void)declare_parameter<double>("frontier_extension_search_period_ms", 1000.0);
    goal_connect_distance_m_ = declare_parameter<double>("goal_connect_distance_m", 6.0);
    goal_connect_timeout_ms_ = declare_parameter<double>("goal_connect_timeout_ms", 20.0);
    odom_reconnect_distance_m_ = declare_parameter<double>("odom_reconnect_distance_m", 1.0);
    odom_reconnect_yaw_deg_ = declare_parameter<double>("odom_reconnect_yaw_deg", 20.0);
    odom_fallback_radius_m_ = declare_parameter<double>("odom_fallback_radius_m", 15.0);
    odom_fallback_candidates_ = declare_parameter<int>("odom_fallback_candidates", 8);
    odom_connect_timeout_ms_ = declare_parameter<double>("odom_connect_timeout_ms", 3.0);
    cloud_pose_tolerance_ms_ = declare_parameter<double>("cloud_pose_tolerance_ms", 50.0);
    semantic_heatmap_topic_ = declare_parameter<std::string>(
      "semantic_heatmap_topic", "/scalenav/text_heatmap_raw");
    semantic_depth_topic_ = declare_parameter<std::string>(
      "semantic_depth_topic", "/camera/depth/image");
    semantic_pose_tolerance_ms_ = declare_parameter<double>(
      "semantic_pose_tolerance_ms", 250.0);
    semantic_depth_tolerance_ms_ = std::max(0.0, declare_parameter<double>(
      "semantic_depth_tolerance_ms", 50.0));
    semantic_depth_max_m_ = std::max(0.1, declare_parameter<double>(
      "semantic_depth_max_m", 20.0));
    semantic_max_age_ms_ = declare_parameter<double>("semantic_max_age_ms", 1500.0);
    semantic_risk_memory_ms_ = std::max(semantic_max_age_ms_, declare_parameter<double>(
      "semantic_risk_memory_ms", 5000.0));
    semantic_risk_accumulation_alpha_ = std::clamp(
      declare_parameter<double>("semantic_risk_accumulation_alpha", 0.25), 0.0, 1.0);
    wait_for_initial_semantic_ = declare_parameter<bool>(
      "wait_for_initial_semantic", true);
    initial_semantic_wait_timeout_ms_ = std::max(0.0, declare_parameter<double>(
      "initial_semantic_wait_timeout_ms", 5000.0));
    semantic_camera_tx_ = declare_parameter<double>("semantic_camera_translation_flu.x", 0.5);
    semantic_camera_ty_ = declare_parameter<double>("semantic_camera_translation_flu.y", 0.0);
    semantic_camera_tz_ = declare_parameter<double>("semantic_camera_translation_flu.z", -0.1);
    semantic_horizontal_fov_deg_ = declare_parameter<double>("semantic_horizontal_fov_deg", 90.0);
    semantic_vertical_fov_deg_ = declare_parameter<double>("semantic_vertical_fov_deg", 60.0);
    semantic_patch_cols_ = std::clamp(
      static_cast<int>(declare_parameter<int>("semantic_patch_cols", 5)), 1, 32);
    semantic_patch_rows_ = std::clamp(
      static_cast<int>(declare_parameter<int>("semantic_patch_rows", 3)), 1, 32);
    semantic_virtual_depth_m_ = declare_parameter<double>("semantic_virtual_depth_m", 30.0);
    semantic_points_enabled_ = declare_parameter<bool>("semantic_points_enabled", true);
    // PEARL scores are calibrated contrast scores, not obstacle clearance.
    // Retain weak but useful semantic evidence; geometric collision checks
    // remain authoritative for executable paths.
    semantic_point_min_score_ = declare_parameter<double>("semantic_point_min_score", 0.20);
    semantic_point_separation_m_ = declare_parameter<double>(
      "semantic_point_separation_m", 1.5);
    semantic_point_radius_m_ = declare_parameter<double>("semantic_point_radius_m", 0.75);
    semantic_point_max_nodes_ = declare_parameter<int>("semantic_point_max_nodes", 16);
    virtual_semantic_prune_enabled_ = declare_parameter<bool>(
      "virtual_semantic_prune_enabled", true);
    virtual_semantic_backtrack_margin_m_ = declare_parameter<double>(
      "virtual_semantic_backtrack_margin_m", 12.0);
    virtual_semantic_max_nodes_ = declare_parameter<int>(
      "virtual_semantic_max_nodes", 512);
    semantic_label_max_nodes_ = declare_parameter<int>("semantic_label_max_nodes", 16);

    cloud_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    semantic_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    state_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);
    planner_callback_group_ = create_callback_group(
      rclcpp::CallbackGroupType::MutuallyExclusive);

    declare_parameter<double>("bubble_topo/min_x", 1.11);
    declare_parameter<double>("bubble_topo/min_y", 1.11);
    declare_parameter<double>("bubble_topo/min_z", 0.51);
    declare_parameter<double>("bubble_topo/init_region_size_x", 3.3);
    declare_parameter<double>("bubble_topo/init_region_size_y", 3.3);
    declare_parameter<double>("bubble_topo/init_region_size_z", 2.0);
    declare_parameter<double>("bubble_topo/bubble_min_radius", 0.65);
    declare_parameter<double>("bubble_topo/frontier_bubble_min_radius", 0.65);
    declare_parameter<double>("bubble_topo/cube_discrete_size", 0.40);
    declare_parameter<double>("bubble_topo/semantic_node_match_distance", 2.5);
    declare_parameter<int>("bubble_topo/semantic_point_connection_candidates", 4);
    declare_parameter<int>("bubble_topo/semantic_point_max_connections", 2);
    declare_parameter<int>("bubble_topo/semantic_edge_candidate_limit", 8);
    declare_parameter<double>(
      "bubble_topo/semantic_risk_memory_ms", semantic_risk_memory_ms_);
    declare_parameter<double>(
      "bubble_topo/semantic_risk_accumulation_alpha", semantic_risk_accumulation_alpha_);
    clearance_cost_weight_ = declare_parameter<double>(
      "bubble_topo/clearance_cost_weight", 2.0);
    clearance_target_m_ = declare_parameter<double>(
      "bubble_topo/clearance_target_m", 1.2);
    semantic_point_influence_m_ = declare_parameter<double>(
      "bubble_topo/semantic_point_influence_m", 8.0);
    declare_parameter<bool>("bubble_topo/planar_graph", graph_fixed_layer_);
    declare_parameter<double>("bubble_topo/planar_z", graph_layer_z_);
    declare_parameter<int>("max_update_region_num", 0);
    // Odom reconnection is on the online update path. Keep the ScaleNav local
    // connection search bounded; frontier_goal-to-mission_goal uses its separate budget.
    declare_parameter<double>("parallel_astar/update_connection_timeout", 0.003);
    declare_parameter<double>("parallel_astar/insert_node_timeout", 0.02);
    declare_parameter<double>("bubble_astar/resolution_astar", 0.30);
    declare_parameter<double>("bubble_astar/lambda_heu", 1.0);
    declare_parameter<double>("bubble_astar/safe_distance", 0.61);
    declare_parameter<bool>("bubble_astar/planar_search", graph_fixed_layer_);
    declare_parameter<double>("bubble_astar/planar_z", graph_layer_z_);
    declare_parameter<int>("bubble_astar/allocate_num", 100000);
    declare_parameter<bool>("bubble_astar/debug", false);

    map_->configureStorage(
      static_cast<float>(map_voxel_size_), static_cast<float>(map_history_radius_m_),
      static_cast<std::size_t>(std::max(map_max_points_, 1000)),
      static_cast<float>(map_prune_distance_m_));
    if (graph_fixed_layer_) {
      map_->setGraphObstacleMinZ(static_cast<float>(graph_layer_z_ - 1.0));
    }

    RCLCPP_INFO(
      get_logger(),
      "ScaleNav Bubble/TopoGraph volume=3D; local goal layer=%s z=%.2f; "
      "obstacle_min_z=%.2f; planner_tick=%.2f Hz lookahead=%.2f m "
      "local_graph_radius=%.1f m",
      graph_fixed_layer_ ? "fixed" : "3D", graph_layer_z_,
      graph_fixed_layer_ ? graph_layer_z_ - 1.0 : std::numeric_limits<double>::quiet_NaN(),
      1000.0 / static_cast<double>(std::max(1, update_period_ms_)),
      local_goal_lookahead_m_, local_graph_radius_m_);
    RCLCPP_INFO(
      get_logger(),
      "ScaleNav config: clearance_target=%.2f m clearance_weight=%.2f "
      "geometry_map=%s route_mode=SEGMENT_HOLD_REPLAN frontier_goal_distance_weight=%.2f "
      "frontier_direction_loss_weight=%.2f "
      "semantic_detour_budget=%.1f m semantic_frame_budget=%.1f m semantic_noise_floor=%.3f "
      "semantic_opportunity=%d frames/%.1f m/%.1f s "
      "semantic_windows=%.0f/%.0f ms(frontier/risk) "
      "semantic_risk_accumulation_alpha=%.2f "
      "semantic_radius=%.2f m "
      "semantic_visual_max=%.2f pearl_risk_threshold=%.2f baseline_q=%.2f "
      "semantic_edge_candidate_limit=%d diagnostic_period=%d ms",
      clearance_target_m_, clearance_cost_weight_,
      map_history_radius_m_ <= 0.0 ? "CURRENT_FRAME" : "SLIDING_WINDOW",
      frontier_goal_distance_weight_,
      frontier_direction_loss_weight_,
      frontier_semantic_detour_budget_m_, frontier_semantic_frame_budget_m_,
      frontier_semantic_noise_floor_, semantic_opportunity_persistence_frames_,
      semantic_opportunity_switch_margin_m_, semantic_opportunity_cooldown_s_,
      semantic_max_age_ms_, semantic_risk_memory_ms_,
      semantic_risk_accumulation_alpha_,
      semantic_point_radius_m_, semantic_visualization_max_score_,
      semantic_point_min_score_,
      semantic_baseline_quantile_,
      static_cast<int>(get_parameter("bubble_topo/semantic_edge_candidate_limit").as_int()),
      diagnostic_log_period_ms_);
    RCLCPP_INFO(get_logger(),
      "ScaleNav semantic projection: enabled=%d max_nodes=%d separation=%.2f m virtual_depth=%.2f m",
      static_cast<int>(semantic_points_enabled_), semantic_point_max_nodes_,
      semantic_point_separation_m_, semantic_virtual_depth_m_);
    RCLCPP_INFO(get_logger(), "ScaleNav graph snapshots: file=%s period=%d ms",
      graph_log_file_.c_str(), diagnostic_log_period_ms_);

    graph_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/scalenav/graph", 1);
    bubble_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/scalenav/bubbles", 1);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/scalenav/path", 1);
    flight_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/scalenav/flight", 1);
    next_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(next_goal_topic_, 10);
    clearance_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(clearance_topic_, 10);
    timing_pub_ = create_publisher<std_msgs::msg::String>(timing_topic_, 100);

    rclcpp::SubscriptionOptions cloud_options;
    cloud_options.callback_group = cloud_callback_group_;
    rclcpp::SubscriptionOptions semantic_options;
    semantic_options.callback_group = semantic_callback_group_;
    rclcpp::SubscriptionOptions state_options;
    state_options.callback_group = state_callback_group_;
    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { onCloud(message); },
      cloud_options);
    free_ray_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      free_ray_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { onFreeRays(message); },
      cloud_options);
    auto semantic_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    semantic_heatmap_sub_ = create_subscription<sensor_msgs::msg::Image>(
      semantic_heatmap_topic_, semantic_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        onSemanticHeatmap(message);
      }, semantic_options);
    semantic_depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
      semantic_depth_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        onSemanticDepth(message);
      }, semantic_options);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { onOdom(message); },
      state_options);
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) { onGoal(message); },
      state_options);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(1, update_period_ms_)),
      [this]() { update(); }, planner_callback_group_);
    flight_timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(50, update_period_ms_)),
      [this]() { publishFlightTelemetry(); }, state_callback_group_);
  }

  ~ScaleNavGraphNode() override
  {
    shutting_down_.store(true);
    writeFlightStatistics(true);
    if (rebuild_thread_.joinable()) {
      // A ROS node can be released by the rebuild worker when its temporary
      // ROS handles are destroyed during shutdown.  Joining the current
      // thread throws std::system_error(Resource deadlock avoided).
      if (rebuild_thread_.get_id() == std::this_thread::get_id()) {
        rebuild_thread_.detach();
      } else {
        rebuild_thread_.join();
      }
    }
  }

 private:
  struct TimedPose
  {
    std::int64_t stamp_ns = 0;
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
  };

  struct TrajectorySample
  {
    double time_s = 0.0;
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Vector3f velocity = Eigen::Vector3f::Zero();
  };

  struct SemanticFrame
  {
    std::int64_t stamp_ns = 0;
    Eigen::Vector3f origin = Eigen::Vector3f::Zero();
    std::vector<Eigen::Vector3f> points_world;
    std::vector<float> scores;
    std::vector<float> confidences;
    // True for the fixed-depth planning anchor; false for a depth-measured
    // projection on the observed surface.
    std::vector<std::uint8_t> is_virtual;
    std::vector<std::int8_t> columns;
  };

  struct SemanticDepthFrame
  {
    std::int64_t stamp_ns = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::vector<float> depth_m;
  };

  // The route currently accepted for execution. Keep the graph path and the
  // optional execution shortcut path separate: shortcut chords are checked
  // for geometric safety, but are not topology edges and must not be used for
  // graph connectivity validation.
  struct AcceptedRouteState
  {
    bool valid = false;
    Eigen::Vector3f frontier_goal = Eigen::Vector3f::Zero();
    std::uint64_t frontier_goal_id = 0;
    float frontier_goal_initial_route_length_m =
      std::numeric_limits<float>::quiet_NaN();
    float frontier_goal_progress_m = 0.0F;
    float frontier_goal_progress_t = 0.0F;
    std::vector<TopoNode::Ptr> topology_path;
    std::vector<TopoNode::Ptr> execution_path;
    std::vector<Eigen::Vector3f> witness_path;

    void clear()
    {
      valid = false;
      frontier_goal.setZero();
      frontier_goal_id = 0;
      frontier_goal_initial_route_length_m = std::numeric_limits<float>::quiet_NaN();
      frontier_goal_progress_m = 0.0F;
      frontier_goal_progress_t = 0.0F;
      topology_path.clear();
      execution_path.clear();
      witness_path.clear();
    }
  };

  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr &message)
  {
    const Eigen::Vector3f next_position(
      static_cast<float>(message->pose.pose.position.x),
      static_cast<float>(message->pose.pose.position.y),
      static_cast<float>(message->pose.pose.position.z));
    Eigen::Quaternionf next_orientation(
      static_cast<float>(message->pose.pose.orientation.w),
      static_cast<float>(message->pose.pose.orientation.x),
      static_cast<float>(message->pose.pose.orientation.y),
      static_cast<float>(message->pose.pose.orientation.z));
    next_orientation.normalize();
    position_ = next_position;
    orientation_ = next_orientation;
    const auto &linear_velocity = message->twist.twist.linear;
    const Eigen::Vector3f measured_velocity(
      static_cast<float>(linear_velocity.x), static_cast<float>(linear_velocity.y),
      static_cast<float>(linear_velocity.z));
    const Eigen::Vector3f world_velocity = odom_twist_frame_ == "body" ?
      next_orientation * measured_velocity : measured_velocity;
    world_velocity_ = world_velocity;
    speed_mps_ = world_velocity.norm();
    const double sample_time = message->header.stamp.sec != 0 ||
      message->header.stamp.nanosec != 0 ?
      static_cast<double>(stampNanoseconds(message->header.stamp)) * 1.0e-9 :
      now().seconds();
    updateFlightStatistics(next_position, world_velocity, sample_time);
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      odom_history_.push_back(TimedPose{
        stampNanoseconds(message->header.stamp), next_position, next_orientation});
      while (odom_history_.size() > max_odom_history_size_) odom_history_.pop_front();
    }
    if (graph_fixed_layer_ && !graph_layer_initialized_) {
      graph_layer_z_ = position_.z();
      graph_layer_initialized_ = true;
      if (map_) map_->setGraphObstacleMinZ(static_cast<float>(graph_layer_z_ - 1.0));
      RCLCPP_INFO(get_logger(), "ScaleNav graph fixed layer z=%.2f obstacle_min_z=%.2f",
        graph_layer_z_, graph_layer_z_ - 1.0);
    }
    have_odom_ = true;
  }

  void updateFlightStatistics(const Eigen::Vector3f &position,
                              const Eigen::Vector3f &velocity,
                              double sample_time)
  {
    if (!position.allFinite() || !velocity.allFinite() || !std::isfinite(sample_time)) return;
    if (!flight_trajectory_.empty()) {
      const auto &previous = flight_trajectory_.back();
      const double dt = sample_time - previous.time_s;
      if (dt > 1e-3 && dt < 2.0) {
        const float distance = (position - previous.position).norm();
        flight_path_length_m_ += distance;
        flight_speed_integral_ += static_cast<double>(velocity.norm()) * dt;
        flight_duration_s_ += dt;
        const Eigen::Vector3f raw_acceleration = (velocity - previous.velocity) /
          static_cast<float>(dt);
        const Eigen::Vector3f acceleration = 0.2F * raw_acceleration +
          0.8F * flight_acceleration_;
        if (have_flight_acceleration_) {
          const Eigen::Vector3f jerk = (acceleration - flight_acceleration_) /
            static_cast<float>(dt);
          const double jerk_norm = jerk.norm();
          flight_max_jerk_mps3_ = std::max(flight_max_jerk_mps3_, jerk_norm);
          flight_jerk_squared_integral_ += jerk_norm * jerk_norm * dt;
        }
        flight_acceleration_ = acceleration;
        have_flight_acceleration_ = true;
        flight_max_acceleration_mps2_ = std::max(
          flight_max_acceleration_mps2_, static_cast<double>(acceleration.norm()));
        flight_max_speed_mps_ = std::max(
          flight_max_speed_mps_, static_cast<double>(velocity.norm()));
      }
    } else {
      flight_start_time_s_ = sample_time;
    }
    flight_trajectory_.push_back(TrajectorySample{sample_time, position, velocity});
    while (flight_trajectory_.size() > trajectory_max_points_) flight_trajectory_.pop_front();
  }

  static std_msgs::msg::ColorRGBA speedColor(float speed, double maximum_speed)
  {
    std_msgs::msg::ColorRGBA color;
    const float t = std::clamp(speed / static_cast<float>(std::max(0.1, maximum_speed)), 0.0F, 1.0F);
    // Perceptually distinct anchors: stopped blue, cruise cyan/yellow, fast red.
    constexpr Rgb anchors[5] = {
      {0.121569F, 0.466667F, 0.705882F},
      {0.090196F, 0.745098F, 0.811765F},
      {0.368627F, 0.788235F, 0.384314F},
      {1.0F, 0.756863F, 0.027451F},
      {0.839216F, 0.152941F, 0.156863F}};
    const float scaled = t * 4.0F;
    const int index = std::min(3, static_cast<int>(scaled));
    const float local = scaled - static_cast<float>(index);
    color.r = anchors[index].r * (1.0F - local) + anchors[index + 1].r * local;
    color.g = anchors[index].g * (1.0F - local) + anchors[index + 1].g * local;
    color.b = anchors[index].b * (1.0F - local) + anchors[index + 1].b * local;
    color.a = 1.0F;
    return color;
  }

  void publishFlightTelemetry()
  {
    constexpr std::size_t max_visualization_points = 2000;
    std::vector<std::size_t> sample_indices;
    if (!flight_trajectory_.empty()) {
      const std::size_t point_count = flight_trajectory_.size();
      const std::size_t stride = point_count > max_visualization_points ?
        std::max<std::size_t>(1, (point_count - 1 + max_visualization_points - 2) /
          (max_visualization_points - 1)) : 1;
      sample_indices.reserve(std::min(point_count, max_visualization_points));
      for (std::size_t index = 0; index < point_count; index += stride) {
        sample_indices.push_back(index);
      }
      if (sample_indices.back() != point_count - 1) sample_indices.push_back(point_count - 1);
    }
    visualization_msgs::msg::MarkerArray message;
    visualization_msgs::msg::Marker trajectory;
    trajectory.header.frame_id = visualization_frame_;
    trajectory.header.stamp = now();
    trajectory.ns = "scalenav_flight_trajectory";
    trajectory.id = 0;
    trajectory.type = visualization_msgs::msg::Marker::LINE_STRIP;
    trajectory.action = visualization_msgs::msg::Marker::ADD;
    trajectory.scale.x = 0.10;
    trajectory.color.a = 1.0;
    for (const std::size_t index : sample_indices) {
      const auto &sample = flight_trajectory_[index];
      trajectory.points.push_back(toPoint(sample.position));
      trajectory.colors.push_back(speedColor(
        sample.velocity.norm(), trajectory_speed_color_max_mps_));
    }
    message.markers.push_back(std::move(trajectory));

    const double rms_jerk = flight_duration_s_ > 1e-6 ?
      std::sqrt(flight_jerk_squared_integral_ / flight_duration_s_) : 0.0;
    const double average_speed = flight_duration_s_ > 1e-6 ?
      flight_speed_integral_ / flight_duration_s_ : 0.0;
    visualization_msgs::msg::Marker vehicle;
    vehicle.header.frame_id = visualization_frame_;
    vehicle.header.stamp = message.markers.front().header.stamp;
    vehicle.ns = "scalenav_flight_vehicle";
    vehicle.id = 2;
    vehicle.type = visualization_msgs::msg::Marker::ARROW;
    vehicle.action = visualization_msgs::msg::Marker::ADD;
    vehicle.pose.position = toPoint(position_);
    vehicle.pose.orientation.x = orientation_.x();
    vehicle.pose.orientation.y = orientation_.y();
    vehicle.pose.orientation.z = orientation_.z();
    vehicle.pose.orientation.w = orientation_.w();
    vehicle.scale.x = 1.25;
    vehicle.scale.y = 0.30;
    vehicle.scale.z = 0.30;
    vehicle.color = speedColor(speed_mps_, trajectory_speed_color_max_mps_);
    message.markers.push_back(std::move(vehicle));
    flight_pub_->publish(message);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "[ScaleNav flight] path=%.2f m duration=%.2f s speed=%.2f/%.2f m/s "
      "acc_max=%.2f m/s^2 jerk_rms=%.2f jerk_max=%.2f m/s^3",
      flight_path_length_m_, flight_duration_s_, static_cast<double>(speed_mps_),
      flight_max_speed_mps_, flight_max_acceleration_mps2_, rms_jerk,
      flight_max_jerk_mps3_);
    writeFlightStatistics(false);
  }

  void writeFlightStatistics(bool final)
  {
    const auto steady_now = std::chrono::steady_clock::now();
    if (!final && have_flight_statistics_write_time_ &&
        std::chrono::duration<double>(steady_now - last_flight_statistics_write_).count() < 5.0) {
      return;
    }
    last_flight_statistics_write_ = steady_now;
    have_flight_statistics_write_time_ = true;
    const double rms_jerk = flight_duration_s_ > 1e-6 ?
      std::sqrt(flight_jerk_squared_integral_ / flight_duration_s_) : 0.0;
    const double average_speed = flight_duration_s_ > 1e-6 ?
      flight_speed_integral_ / flight_duration_s_ : 0.0;
    const auto path = std::filesystem::path(flight_statistics_file_);
    std::error_code error;
    std::filesystem::create_directories(path.parent_path(), error);
    std::ofstream output(path, std::ios::app);
    if (!output) return;
    if (output.tellp() == std::streampos(0)) {
      output << "wall_time,source,final,path_m,duration_s,current_speed_mps,"
        "average_speed_mps,max_speed_mps,max_acceleration_mps2,jerk_rms_mps3,"
        "max_jerk_mps3\n";
    }
    const auto wall_time = std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    output << std::fixed << std::setprecision(6)
      << wall_time << ",scalenav," << (final ? 1 : 0) << ","
      << flight_path_length_m_ << "," << flight_duration_s_ << ","
      << speed_mps_ << "," << average_speed << "," << flight_max_speed_mps_ << ","
      << flight_max_acceleration_mps2_ << "," << rms_jerk << ","
      << flight_max_jerk_mps3_ << "\n";
  }

  void writeGraphSnapshot(const TopoGraph::Ptr &topo,
                          const std::vector<TopoNode::Ptr> &path_nodes,
                          bool found)
  {
    if (!topo) return;
    const auto steady_now = std::chrono::steady_clock::now();
    if (have_graph_log_time_ &&
        std::chrono::duration<double, std::milli>(
          steady_now - last_graph_log_time_).count() < diagnostic_log_period_ms_) {
      return;
    }
    last_graph_log_time_ = steady_now;
    have_graph_log_time_ = true;

    std::vector<TopoNode::Ptr> nodes;
    std::unordered_map<TopoNode::Ptr, std::size_t> node_index;
    auto add_node = [&](const TopoNode::Ptr &node) {
      if (!node || node_index.count(node)) return;
      node_index.emplace(node, nodes.size());
      nodes.push_back(node);
    };
    for (const auto &entry : topo->reg_map_idx2ptr_) {
      if (!entry.second) continue;
      for (const auto &node : entry.second->topo_nodes_) add_node(node);
    }
    add_node(topo->odom_node_);
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      const auto &node = nodes[i];
      if (!node) continue;
      for (const auto &neighbor : node->neighbors_) add_node(neighbor);
    }

    const auto role_name = [](TopoNodeRole role) {
      switch (role) {
        case TopoNodeRole::Semantic: return "semantic";
        case TopoNodeRole::Odom: return "odom";
        default: return "geometric";
      }
    };
    const auto geometry_name = [](TopoGeometryState state) {
      return state == TopoGeometryState::Unknown ? "unknown" : "verified";
    };
    const auto json_number = [](double value) {
      return std::isfinite(value) ? std::to_string(value) : "null";
    };
    const auto wall_time = std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    std::size_t edge_count = 0;
    std::size_t directed_edge_count = 0;
    std::size_t asymmetric_edge_count = 0;
    std::size_t dangling_neighbor_count = 0;
    std::size_t duplicate_pair_count = 0;
    std::map<std::size_t, std::size_t> degree_histogram;
    for (const auto &node : nodes) {
      if (!node) continue;
      degree_histogram[node->neighbors_.size()]++;
      directed_edge_count += node->neighbors_.size();
      for (const auto &neighbor : node->neighbors_) {
        if (!node_index.count(neighbor)) {
          ++dangling_neighbor_count;
          continue;
        }
        if (neighbor->neighbors_.count(node) == 0) ++asymmetric_edge_count;
        if (node_index.count(neighbor) &&
            std::less<const TopoNode *>{}(node.get(), neighbor.get())) {
          ++edge_count;
        }
      }
    }
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (!nodes[i] || nodes[i]->role_ == TopoNodeRole::Odom) continue;
      for (std::size_t j = i + 1; j < nodes.size(); ++j) {
        if (!nodes[j] || nodes[j]->role_ == TopoNodeRole::Odom) continue;
        if ((nodes[i]->center_ - nodes[j]->center_).norm() < 0.25F) ++duplicate_pair_count;
      }
    }
    std::size_t zero_degree_nodes = 0;
    std::size_t forward_zero_degree_nodes = 0;
    std::size_t route_anchor_count = 0;
    Eigen::Vector3f forward = orientation_ * Eigen::Vector3f::UnitX();
    forward.z() = 0.0F;
    if (forward.squaredNorm() > 1e-6F) forward.normalize();
    for (const auto &node : nodes) {
      if (node && node->is_route_anchor_) ++route_anchor_count;
      if (node && node->role_ != TopoNodeRole::Odom && node->neighbors_.empty()) {
        ++zero_degree_nodes;
        if (forward.squaredNorm() > 1e-6F) {
          const Eigen::Vector3f delta = node->center_ - position_;
          if (delta.norm() <= 30.0F && delta.dot(forward) > 0.0F) {
            ++forward_zero_degree_nodes;
          }
        }
      }
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "ScaleNav graph stats: nodes=%zu edges=%zu directed=%zu degree0=%zu "
      "route_anchors=%zu asymmetric=%zu dangling=%zu duplicate<0.25m=%zu",
      nodes.size(), edge_count, directed_edge_count, zero_degree_nodes,
      route_anchor_count, asymmetric_edge_count, dangling_neighbor_count,
      duplicate_pair_count);
    const auto update_timing = topo->getLastUpdateTiming();
    const auto path = std::filesystem::path(graph_log_file_);
    std::error_code error;
    std::filesystem::create_directories(path.parent_path(), error);
    std::ofstream output(path, std::ios::app);
    if (!output) return;

    output << "{\"event\":\"graph_snapshot\",\"wall_time\":"
      << std::fixed << std::setprecision(6) << wall_time
      << ",\"stamp_ns\":" << now().nanoseconds()
      << ",\"position\":[" << position_.x() << "," << position_.y() << ","
      << position_.z() << "],\"mission_goal\":[" << goal_.x() << "," << goal_.y() << ","
      << goal_.z() << "],\"frontier_goal\":[" << accepted_route_.frontier_goal.x() << ","
      << accepted_route_.frontier_goal.y() << "," << accepted_route_.frontier_goal.z() << "],\"local_goal\":["
      << previous_local_goal_.x() << "," << previous_local_goal_.y() << ","
      << previous_local_goal_.z()
      << "],\"local_goal_valid\":" << (have_previous_local_goal_ ? 1 : 0)
      << ",\"found\":" << (found ? 1 : 0)
      << ",\"node_count\":" << nodes.size() << ",\"edge_count\":" << edge_count
      << ",\"directed_edge_count\":" << directed_edge_count
      << ",\"asymmetric_edge_count\":" << asymmetric_edge_count
      << ",\"dangling_neighbor_count\":" << dangling_neighbor_count
      << ",\"duplicate_pair_count_25cm\":" << duplicate_pair_count
      << ",\"zero_degree_nodes\":" << zero_degree_nodes
      << ",\"forward_zero_degree_nodes_30m\":" << forward_zero_degree_nodes
      << ",\"route_anchor_count\":" << route_anchor_count
      << ",\"last_update\":{\"regions\":" << update_timing.regions
      << ",\"total_ms\":" << update_timing.total_ms
      << ",\"prepare_ms\":" << update_timing.prepare_ms
      << ",\"parallel_wall_ms\":" << update_timing.parallel_wall_ms
      << ",\"bubble_cpu_ms\":" << update_timing.bubble_cpu_ms
      << ",\"cluster_cpu_ms\":" << update_timing.cluster_cpu_ms
      << ",\"diff_ms\":" << update_timing.diff_ms
      << ",\"remove_ms\":" << update_timing.remove_ms
      << ",\"reconnect_ms\":" << update_timing.reconnect_ms
      << ",\"insert_ms\":" << update_timing.insert_ms
      << ",\"occupied_regions\":" << update_timing.occupied_regions
      << ",\"free_regions\":" << update_timing.free_regions
      << ",\"bubbles_3d\":" << update_timing.bubbles
      << ",\"bubbles_planar\":" << update_timing.planar_bubbles
      << ",\"new_nodes\":" << update_timing.new_nodes
      << ",\"remained_nodes\":" << update_timing.remained_nodes
      << ",\"inserted_nodes\":" << update_timing.inserted_nodes
      << ",\"insert_candidate_edges\":" << update_timing.insert_candidate_edges
      << ",\"insert_success_edges\":" << update_timing.insert_success_edges
      << ",\"insert_timeout_edges\":" << update_timing.insert_timeout_edges
      << ",\"insert_no_path_edges\":" << update_timing.insert_no_path_edges
      << ",\"insert_start_fail_edges\":" << update_timing.insert_start_fail_edges
      << ",\"insert_end_fail_edges\":" << update_timing.insert_end_fail_edges
      << ",\"insert_collision_reject_edges\":" << update_timing.insert_collision_reject_edges
      << ",\"existing_edges_checked\":" << update_timing.existing_edges_checked
      << ",\"existing_edges_kept\":" << update_timing.existing_edges_kept
      << ",\"existing_edges_repaired\":" << update_timing.existing_edges_repaired
      << ",\"existing_edges_removed\":" << update_timing.existing_edges_removed
      << ",\"existing_edges_soft_retry\":" << update_timing.existing_edges_soft_retry
      << ",\"existing_edges_cooldown_skipped\":" << update_timing.existing_edges_cooldown_skipped
      << ",\"removed_nodes\":" << update_timing.removed_nodes
      << ",\"deferred_nodes\":" << update_timing.deferred_nodes
      << ",\"duplicate_nodes_merged\":" << update_timing.duplicate_nodes_merged
      << ",\"half_edges_removed\":" << update_timing.half_edges_removed
      << "}"
      << ",\"degree_histogram\":{";
    bool first_degree = true;
    for (const auto &[degree, count] : degree_histogram) {
      if (!first_degree) output << ',';
      first_degree = false;
      output << '\"' << degree << "\":" << count;
    }
    output << "}"
      << ",\"nodes\":[";
    for (std::size_t i = 0; i < nodes.size(); ++i) {
      if (i) output << ',';
      const auto &node = nodes[i];
      output << "{\"id\":" << i
        << ",\"persistent_id\":" << node->persistent_id_
        << ",\"role\":\"" << role_name(node->role_)
        << "\",\"geometry_state\":\"" << geometry_name(node->geometry_state_)
        << "\",\"center\":[" << node->center_.x() << "," << node->center_.y()
        << "," << node->center_.z() << "]"
        << ",\"radius\":" << json_number(node->bubble_radius_)
        << ",\"semantic_score\":" << json_number(node->semantic_score_)
        << ",\"semantic_confidence\":" << json_number(node->semantic_confidence_)
        << ",\"semantic_observations\":" << node->semantic_observations_
        << ",\"is_virtual_semantic\":" <<
          (node->is_virtual_semantic_ ? "true" : "false")
        << ",\"geometry_miss_count\":" << static_cast<unsigned>(node->geometry_miss_count_)
        << ",\"route_anchor\":" << (node->is_route_anchor_ ? 1 : 0)
        << ",\"degree\":" << node->neighbors_.size() << ",\"neighbors\":[";
      bool first_neighbor = true;
      for (const auto &neighbor : node->neighbors_) {
        const auto it = node_index.find(neighbor);
        if (it == node_index.end()) continue;
        if (!first_neighbor) output << ',';
        first_neighbor = false;
        output << it->second;
      }
      output << "]}";
    }
    output << "],\"path\":[";
    for (std::size_t i = 0; i < path_nodes.size(); ++i) {
      if (i) output << ',';
      const auto it = node_index.find(path_nodes[i]);
      output << (it == node_index.end() ? -1 : static_cast<long long>(it->second));
    }
    output << "]}\n";
  }

  void onSemanticDepth(const sensor_msgs::msg::Image::ConstSharedPtr &message)
  {
    if (message->encoding != "32FC1" || message->is_bigendian ||
        message->width == 0 || message->height == 0 ||
        message->step < message->width * sizeof(float) ||
        message->data.size() < message->step * message->height) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Ignoring semantic depth with invalid contract: encoding=%s size=%ux%u step=%u",
        message->encoding.c_str(), message->width, message->height, message->step);
      return;
    }
    SemanticDepthFrame frame;
    frame.stamp_ns = stampNanoseconds(message->header.stamp);
    frame.width = message->width;
    frame.height = message->height;
    frame.depth_m.resize(static_cast<std::size_t>(frame.width) * frame.height);
    for (std::uint32_t v = 0; v < frame.height; ++v) {
      const auto *row = reinterpret_cast<const float *>(
        message->data.data() + static_cast<std::size_t>(v) * message->step);
      std::copy_n(row, frame.width,
        frame.depth_m.begin() + static_cast<std::size_t>(v) * frame.width);
    }
    std::lock_guard<std::mutex> lock(semantic_mutex_);
    semantic_depth_history_.push_back(std::move(frame));
    while (semantic_depth_history_.size() > max_semantic_depth_history_size_) {
      semantic_depth_history_.pop_front();
    }
  }

  std::optional<SemanticDepthFrame> semanticDepthForStamp(std::int64_t stamp_ns)
  {
    std::lock_guard<std::mutex> lock(semantic_mutex_);
    if (semantic_depth_history_.empty()) return std::nullopt;
    // Several simulator image topics omit header timestamps. In that case the
    // newest depth frame is the only meaningful synchronization target.
    if (stamp_ns == 0) return semantic_depth_history_.back();
    auto closest = semantic_depth_history_.begin();
    std::int64_t delta = std::llabs(closest->stamp_ns - stamp_ns);
    for (auto it = std::next(closest); it != semantic_depth_history_.end(); ++it) {
      const std::int64_t candidate_delta = std::llabs(it->stamp_ns - stamp_ns);
      if (candidate_delta < delta) {
        closest = it;
        delta = candidate_delta;
      }
    }
    if (static_cast<double>(delta) / 1.0e6 > semantic_depth_tolerance_ms_) {
      return std::nullopt;
    }
    return *closest;
  }

  void onSemanticHeatmap(const sensor_msgs::msg::Image::ConstSharedPtr &message)
  {
    if (message->encoding != "32FC1" || message->is_bigendian ||
        message->width == 0 || message->height == 0 ||
        message->step < message->width * sizeof(float) ||
        message->data.size() < message->step * message->height) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring semantic heatmap with invalid contract: encoding=%s size=%ux%u step=%u",
        message->encoding.c_str(), message->width, message->height, message->step);
      return;
    }

    TimedPose capture_pose;
    double pose_delta_ms = 0.0;
    if (!poseForCloud(message->header.stamp, capture_pose, pose_delta_ms) ||
        pose_delta_ms > semantic_pose_tolerance_ms_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Ignoring semantic heatmap without synchronized odometry: delta=%.1f ms tolerance=%.1f ms",
        pose_delta_ms, semantic_pose_tolerance_ms_);
      return;
    }

    const std::int64_t message_stamp_ns = stampNanoseconds(message->header.stamp);
    // A zero image timestamp is valid in the simulator, but cannot be used as
    // a semantic-memory age. Anchor the frame to the synchronized odometry
    // sample instead (and fall back to node time only if that sample is also
    // unstamped).
    const std::int64_t stamp_ns = message_stamp_ns != 0 ? message_stamp_ns :
      (capture_pose.stamp_ns != 0 ? capture_pose.stamp_ns : get_clock()->now().nanoseconds());
    const auto depth_frame = semanticDepthForStamp(message_stamp_ns);
    const double depth_delta_ms = depth_frame ?
      ((message_stamp_ns == 0 || depth_frame->stamp_ns == 0) ? 0.0 :
       static_cast<double>(std::llabs(depth_frame->stamp_ns - message_stamp_ns)) / 1.0e6) :
      std::numeric_limits<double>::quiet_NaN();
    std::size_t semantic_point_count = 0;
    std::size_t semantic_measured_count = 0;
    std::size_t semantic_virtual_count = 0;
    float semantic_frame_baseline = 0.0F;
    float semantic_raw_min = 0.0F;
    float semantic_raw_max = 0.0F;
    float semantic_risk_min = 0.0F;
    float semantic_risk_max = 0.0F;
    std::string semantic_columns_summary;
    const Eigen::Vector3f camera_translation(
      static_cast<float>(semantic_camera_tx_), static_cast<float>(semantic_camera_ty_),
      static_cast<float>(semantic_camera_tz_));
    // The published heatmap is an upsampled image. Aggregate each model patch
    // by its finite-pixel mean, then keep only the middle row for planar
    // forward flight: one candidate per horizontal column.
    constexpr std::size_t patch_cols = 5U;
    constexpr std::size_t patch_rows = 3U;
    const std::size_t patch_count = patch_cols * patch_rows;
    std::vector<float> patch_sums(patch_count, 0.0F);
    std::vector<std::uint32_t> patch_pixel_counts(patch_count, 0U);
    std::vector<std::uint8_t> patch_valid(patch_count, 0U);
    std::vector<std::uint32_t> patch_center_u(patch_count, 0U);
    std::vector<std::uint32_t> patch_center_v(patch_count, 0U);
    for (std::uint32_t v = 0; v < message->height; ++v) {
      const auto *heatmap_row = reinterpret_cast<const float *>(
        message->data.data() + static_cast<std::size_t>(v) * message->step);
      for (std::uint32_t u = 0; u < message->width; ++u) {
        const float semantic = std::clamp(heatmap_row[u], 0.0F, 1.0F);
        if (!std::isfinite(semantic)) continue;
        const std::size_t patch_u = std::min(
          patch_cols - 1,
          static_cast<std::size_t>(u) * patch_cols /
            static_cast<std::size_t>(message->width));
        const std::size_t patch_v = std::min(
          patch_rows - 1,
          static_cast<std::size_t>(v) * patch_rows /
            static_cast<std::size_t>(message->height));
        const std::size_t patch_index = patch_v * patch_cols + patch_u;
        patch_sums[patch_index] += semantic;
        ++patch_pixel_counts[patch_index];
        patch_valid[patch_index] = 1U;
      }
    }
    {
      std::lock_guard<std::mutex> lock(semantic_mutex_);
      SemanticFrame frame;
      frame.stamp_ns = stamp_ns;
      frame.origin = capture_pose.position;
      // Keep both projections for every patch.  A measured surface point is
      // useful to the ordinary topology, while the fixed-depth counterpart
      // remains the semantic frontier anchor even when depth is valid.
      // Keep the five fixed-depth semantic frontier anchors independent from
      // measured surface projections used to annotate ordinary topology.
      // A valid depth sample must never replace or remove a virtual column.
      frame.points_world.reserve(patch_cols * 2U);
      frame.scores.reserve(patch_cols * 2U);
      frame.confidences.reserve(patch_cols * 2U);
      frame.is_virtual.reserve(patch_cols * 2U);
      frame.columns.reserve(patch_cols * 2U);
      std::vector<float> valid_patch_scores;
      valid_patch_scores.reserve(patch_count);
      for (std::size_t i = 0; i < patch_count; ++i) {
        if (patch_valid[i] && patch_pixel_counts[i] > 0U) {
          valid_patch_scores.push_back(
            patch_sums[i] / static_cast<float>(patch_pixel_counts[i]));
        }
      }
      float frame_baseline = 0.0F;
      if (!valid_patch_scores.empty()) {
        frame_baseline = semanticFrameBaseline(
          valid_patch_scores, static_cast<float>(semantic_baseline_quantile_));
      }
      float raw_min = std::numeric_limits<float>::infinity();
      float raw_max = 0.0F;
      float calibrated_min = std::numeric_limits<float>::infinity();
      float calibrated_max = 0.0F;
      std::vector<float> calibrated_scores(patch_count, 0.0F);
      for (std::size_t i = 0; i < patch_count; ++i) {
        if (patch_valid[i] && patch_pixel_counts[i] > 0U) {
          const float patch_mean =
            patch_sums[i] / static_cast<float>(patch_pixel_counts[i]);
          calibrated_scores[i] = calibrateSemanticScore(patch_mean, frame_baseline);
          const std::size_t patch_u = i % patch_cols;
          const std::size_t patch_v = i / patch_cols;
          const std::uint32_t u_begin = static_cast<std::uint32_t>(
            patch_u * static_cast<std::size_t>(message->width) / patch_cols);
          const std::uint32_t u_end = static_cast<std::uint32_t>(
            (patch_u + 1U) * static_cast<std::size_t>(message->width) / patch_cols);
          const std::uint32_t v_begin = static_cast<std::uint32_t>(
            patch_v * static_cast<std::size_t>(message->height) / patch_rows);
          const std::uint32_t v_end = static_cast<std::uint32_t>(
            (patch_v + 1U) * static_cast<std::size_t>(message->height) / patch_rows);
          patch_center_u[i] = std::min(
            message->width - 1U, u_begin + std::max(1U, u_end - u_begin) / 2U);
          patch_center_v[i] = std::min(
            message->height - 1U, v_begin + std::max(1U, v_end - v_begin) / 2U);
        }
      }
      const std::size_t middle_row = patch_rows / 2U;
      for (std::size_t i = 0; i < patch_count; ++i) {
        if (!patch_valid[i] || patch_pixel_counts[i] == 0U ||
            i / patch_cols != middle_row) continue;
        const float normalized_u =
          (static_cast<float>(patch_center_u[i]) + 0.5F) /
          static_cast<float>(message->width);
        const float normalized_v =
          (static_cast<float>(patch_center_v[i]) + 0.5F) /
          static_cast<float>(message->height);
        float measured_depth_m = 0.0F;
        bool measured_depth_valid = false;
        if (depth_frame) {
          const std::uint32_t depth_u = std::min(
            depth_frame->width - 1, static_cast<std::uint32_t>(normalized_u * depth_frame->width));
          const std::uint32_t depth_v = std::min(
            depth_frame->height - 1, static_cast<std::uint32_t>(normalized_v * depth_frame->height));
          const float sampled = depth_frame->depth_m[
            static_cast<std::size_t>(depth_v) * depth_frame->width + depth_u];
          if (std::isfinite(sampled) && sampled > 0.0F &&
              sampled < static_cast<float>(semantic_depth_max_m_ - 1.0e-4)) {
            measured_depth_m = sampled;
            measured_depth_valid = true;
          }
        }
        const float patch_mean = patch_sums[i] /
          static_cast<float>(patch_pixel_counts[i]);
        // Use the patch mean directly for relative frame decisions. The
        // baseline remains diagnostic only and must not reorder columns.
        const float calibrated_score = patch_mean;

        const float fov_radius = std::clamp(std::max(
          std::abs(2.0F * normalized_u - 1.0F),
          std::abs(2.0F * normalized_v - 1.0F)), 0.0F, 1.0F);
        const float fov_confidence = 1.0F - 0.35F * fov_radius * fov_radius;
        const std::size_t patch_col = i % patch_cols;
        raw_min = std::min(raw_min, patch_mean);
        raw_max = std::max(raw_max, patch_mean);
        calibrated_min = std::min(calibrated_min, calibrated_score);
        calibrated_max = std::max(calibrated_max, calibrated_score);

        const auto append_projection = [&](float projection_depth_m, bool is_virtual) {
          const Eigen::Vector3f body = is_virtual ?
            virtualSemanticPointFlu(
              normalized_u, normalized_v,
              static_cast<float>(semantic_horizontal_fov_deg_),
              static_cast<float>(semantic_vertical_fov_deg_),
              projection_depth_m, camera_translation) :
            semanticPointFluAtOpticalDepth(
              normalized_u, normalized_v,
              static_cast<float>(semantic_horizontal_fov_deg_),
              static_cast<float>(semantic_vertical_fov_deg_),
              projection_depth_m, camera_translation);
          const Eigen::Vector3f point_world =
            capture_pose.position + capture_pose.orientation * body;
          // Once projected to the fixed planning layer, image elevation is no
          // longer evidence against the semantic anchor.
          const float below_layer = static_cast<float>(graph_layer_z_) - point_world.z();
          const float ground_confidence = graph_fixed_layer_ ? 1.0F :
            (below_layer <= 0.5F ? 1.0F : std::clamp(
              1.0F - (below_layer - 0.5F) / 5.0F, 0.25F, 1.0F));
          frame.points_world.push_back(point_world);
          frame.scores.push_back(calibrated_score);
          frame.confidences.push_back(std::clamp(
            fov_confidence * ground_confidence, 0.05F, 1.0F));
          frame.is_virtual.push_back(is_virtual ? 1U : 0U);
          frame.columns.push_back(static_cast<std::int8_t>(patch_col));
        };

        // Every horizontal column always produces one fixed-depth virtual
        // frontier candidate. A measured projection is additionally emitted
        // for semantic annotation of the ordinary topology; it is not a
        // substitute for the virtual candidate.
        if (measured_depth_valid) {
          append_projection(measured_depth_m, false);
          ++semantic_measured_count;
        }
        append_projection(static_cast<float>(semantic_virtual_depth_m_), true);
        ++semantic_virtual_count;
      }
      semantic_point_count = frame.points_world.size();
      semantic_frame_baseline = frame_baseline;
      semantic_raw_min = std::isfinite(raw_min) ? raw_min : 0.0F;
      semantic_raw_max = raw_max;
      semantic_risk_min = std::isfinite(calibrated_min) ? calibrated_min : 0.0F;
      semantic_risk_max = calibrated_max;
      semantic_frame_ = std::move(frame);
      std::ostringstream columns_log;
      columns_log << "[";
      for (std::size_t i = 0; i < semantic_frame_->scores.size(); ++i) {
        if (i != 0) columns_log << ",";
        columns_log << static_cast<int>(semantic_frame_->columns[i]) << ":" <<
          std::fixed << std::setprecision(3) << semantic_frame_->scores[i] <<
          (semantic_frame_->is_virtual[i] != 0U ? ":V" : ":D");
      }
      columns_log << "]";
      semantic_columns_summary = columns_log.str();
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), diagnostic_log_period_ms_,
      "[ScaleNav semantic] image=%ux%u patches=%zux%zu projected_total=%zu "
      "virtual_depth=%.2f m "
      "measured=%zu virtual=%zu raw=%.3f..%.3f baseline=%.3f risk=%.3f..%.3f "
      "pose_sync=%.1f ms depth_sync=%.1f ms image_stamp_ns=%lld frame_stamp_ns=%lld",
      message->width, message->height, patch_cols, patch_rows,
      semantic_point_count, semantic_virtual_depth_m_, semantic_measured_count,
      semantic_virtual_count, semantic_raw_min,
      semantic_raw_max, semantic_frame_baseline, semantic_risk_min,
      semantic_risk_max, pose_delta_ms, depth_delta_ms,
      static_cast<long long>(message_stamp_ns), static_cast<long long>(stamp_ns));
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), diagnostic_log_period_ms_,
      "[ScaleNav semantic frame] middle_row_columns=%s", semantic_columns_summary.c_str());
  }

  void mergeSemanticMemory(const std::vector<TopoSemanticRecord> &records)
  {
    std::lock_guard<std::mutex> lock(semantic_memory_mutex_);
    for (const auto &record : records) {
      if (record.node_id == 0 || !record.center.allFinite() ||
          !std::isfinite(record.score) || !std::isfinite(record.confidence)) {
        continue;
      }
      auto it = semantic_memory_.find(record.node_id);
      if (it == semantic_memory_.end() ||
          record.stamp_ns >= it->second.stamp_ns) {
        semantic_memory_[record.node_id] = record;
      }
    }
  }

  std::vector<TopoSemanticRecord> semanticMemorySnapshot() const
  {
    std::lock_guard<std::mutex> lock(semantic_memory_mutex_);
    std::vector<TopoSemanticRecord> records;
    records.reserve(semantic_memory_.size());
    for (const auto &entry : semantic_memory_) records.push_back(entry.second);
    return records;
  }

  std::size_t persistentSemanticRecordCount() const
  {
    std::lock_guard<std::mutex> lock(semantic_memory_mutex_);
    return semantic_memory_.size();
  }

  void eraseSemanticMemory(const std::vector<std::uint64_t> &ids)
  {
    if (ids.empty()) return;
    std::lock_guard<std::mutex> lock(semantic_memory_mutex_);
    for (const auto id : ids) semantic_memory_.erase(id);
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &message)
  {
    std::lock_guard<std::mutex> topology_lock(topology_operation_mutex_);
    Eigen::Vector3f next_goal(
      static_cast<float>(message->pose.position.x),
      static_cast<float>(message->pose.position.y),
      static_cast<float>(message->pose.position.z));
    if (graph_fixed_layer_) {
      next_goal.z() = static_cast<float>(graph_layer_initialized_ ? graph_layer_z_ : next_goal.z());
    }
    if (have_goal_ && (next_goal - goal_).norm() < 1e-3F) return;
    vector<TopoSemanticRecord> semantic_memory;
    std::string goal_graph_mode = "REBUILD_GRAPH";
    bool bounds_expanded = false;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      if (topo_) mergeSemanticMemory(topo_->semanticMemorySnapshot());
      semantic_memory = semanticMemorySnapshot();
      goal_ = next_goal;
      have_goal_ = true;
      // A new mission goal starts a fresh route search. The measured graph may
      // still be retained, but no previous route is reused for planning.
      accepted_route_.clear();
      mission_direct_goal_latched_ = false;
      polynomial_guide_path_.clear();
      polynomial_curve_ = scalenav_graph::WitnessParametricCurve();
      polynomial_curve_valid_ = false;
      have_previous_local_goal_ = false;
      const bool can_reuse = reuse_graph_on_goal_ && graph_initialized_.load() &&
        skeleton_initialized_.load() && topo_ && astar_ &&
        topo_->lidar_map_interface_;
      if (can_reuse) {
        // TopoGraph creates regions lazily, but Bubble A* still rejects samples
        // outside LIOInterface::IsInMap(). Grow that search domain with every
        // mission goal while retaining all existing nodes, edges and semantics.
        bounds_expanded = expandMapBounds(
          topo_->lidar_map_interface_, position_, next_goal,
          std::max(map_margin_, map_history_radius_m_));
        expandMapBounds(
          map_, position_, next_goal,
          std::max(map_margin_, map_history_radius_m_));
        if (bounds_expanded) map_changed_ = true;
      }
      if (!can_reuse) {
        graph_initialized_ = false;
        skeleton_initialized_ = false;
        map_changed_ = true;
        have_graph_odom_ = false;
        astar_ = std::make_shared<ParallelBubbleAstar>();
        topo_ = std::make_shared<TopoGraph>();
        topo_->loadSemanticMemory(semantic_memory);
      }
      goal_graph_mode = can_reuse ?
        (bounds_expanded ? "REUSE_EXPANDED_GRAPH" : "REUSE_EXISTING_GRAPH") :
        "REBUILD_GRAPH";
      ++goal_generation_;
    }
    RCLCPP_INFO(get_logger(),
      "[ScaleNav goal] target=(%.2f,%.2f,%.2f) graph=%s reuse=%d bounds_expanded=%d "
      "semantic_memory=%zu",
      goal_.x(), goal_.y(), goal_.z(), goal_graph_mode.c_str(),
      static_cast<int>(reuse_graph_on_goal_), static_cast<int>(bounds_expanded),
      semantic_memory.size());
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message)
  {
    const auto callback_start = std::chrono::steady_clock::now();
    const auto decode_start = callback_start;
    pcl::PointCloud<fast_planner::PointType> cloud_body;
    pcl::fromROSMsg(*message, cloud_body);
    if (cloud_body.empty() || !have_odom_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[ScaleNav input] dropped cloud: points=%zu have_odom=%d",
        cloud_body.size(), static_cast<int>(have_odom_));
      return;
    }
    TimedPose capture_pose;
    double pose_sync_ms = 0.0;
    if (!poseForCloud(message->header.stamp, capture_pose, pose_sync_ms)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Dropping point cloud without synchronized odometry: stamp delta=%.1f ms, "
        "tolerance=%.1f ms",
        pose_sync_ms, cloud_pose_tolerance_ms_);
      return;
    }
    const double decode_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - decode_start).count();

    const auto transform_start = std::chrono::steady_clock::now();
    pcl::PointCloud<fast_planner::PointType> cloud_world;
    cloud_world.reserve(cloud_body.size());
    for (const auto &point : cloud_body.points) {
      const Eigen::Vector3f body(point.x, point.y, point.z);
      const Eigen::Vector3f world =
        capture_pose.position + capture_pose.orientation * body;
      cloud_world.push_back(fast_planner::PointType(world.x(), world.y(), world.z()));
    }
    const double transform_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - transform_start).count();

    const auto voxel_start = std::chrono::steady_clock::now();
    pcl::PointCloud<fast_planner::PointType>::Ptr voxel_cloud(
      new pcl::PointCloud<fast_planner::PointType>());
    pcl::VoxelGrid<fast_planner::PointType> voxel_filter;
    const float voxel_size = static_cast<float>(std::max(map_voxel_size_, 0.05));
    voxel_filter.setLeafSize(voxel_size, voxel_size, voxel_size);
    voxel_filter.setInputCloud(cloud_world.makeShared());
    voxel_filter.filter(*voxel_cloud);
    const double voxel_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - voxel_start).count();
    if (voxel_cloud->empty()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "[ScaleNav input] dropped cloud after voxel filtering: input=%zu leaf=%.3f m",
        cloud_body.size(), voxel_size);
      return;
    }

    const auto map_start = std::chrono::steady_clock::now();
    fast_planner::LIOInterface::Ptr active_map;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      active_map = map_;
    }
    const bool changed = active_map->updateCloudWorld(
      *voxel_cloud, capture_pose.position, capture_pose.orientation);
    if (changed) map_changed_.store(true);
    const double map_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - map_start).count();
    std::size_t occupied_hits = 0;
    std::size_t unused_free = 0;
    std::size_t unused_carved = 0;
    active_map->lastRayCarvingStats(occupied_hits, unused_free, unused_carved);
    cloud_count_++;
    have_cloud_ = true;
    const double total_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - callback_start).count();
    std::ostringstream timing_json;
    timing_json << std::fixed << std::setprecision(6)
      << "{\"module\":\"cloud\",\"stamp_ns\":" << now().nanoseconds()
      << ",\"decode_ms\":" << decode_ms
      << ",\"transform_ms\":" << transform_ms
      << ",\"voxel_ms\":" << voxel_ms
      << ",\"map_ms\":" << map_ms
      << ",\"total_ms\":" << total_ms
      << ",\"pose_sync_ms\":" << pose_sync_ms
      << ",\"input_points\":" << cloud_body.size()
      << ",\"voxel_points\":" << voxel_cloud->size()
      << ",\"map_points\":" << active_map->pointCount()
      << ",\"occupied_hits\":" << occupied_hits << '}';
    publishTiming(timing_json.str());
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), diagnostic_log_period_ms_,
      "[ScaleNav timing][cloud] decode=%.3f ms transform=%.3f ms map_update=%.3f ms "
      "voxel=%.3f ms total=%.3f ms pose_sync=%.3f ms input=%zu voxel_points=%zu "
      "map_points=%zu occupied_hits=%zu leaf=%.3f",
      decode_ms, transform_ms, map_ms, voxel_ms, total_ms, pose_sync_ms,
      cloud_body.size(), voxel_cloud->size(), active_map->pointCount(), occupied_hits,
      voxel_size);
  }

  void onFreeRays(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &)
  {
  }

  static std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time &stamp)
  {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(stamp.nanosec);
  }

  void publishTiming(const std::string &json)
  {
    std_msgs::msg::String message;
    message.data = json;
    timing_pub_->publish(message);
  }

  bool poseForCloud(const builtin_interfaces::msg::Time &stamp, TimedPose &pose,
                    double &delta_ms) const
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    if (odom_history_.empty()) {
      delta_ms = std::numeric_limits<double>::infinity();
      return false;
    }
    const std::int64_t target = stampNanoseconds(stamp);
    if (target == 0) {
      pose = odom_history_.back();
      delta_ms = 0.0;
      return true;
    }
    auto closest = odom_history_.begin();
    std::int64_t closest_delta = std::llabs(closest->stamp_ns - target);
    for (auto candidate = std::next(odom_history_.begin());
         candidate != odom_history_.end(); ++candidate) {
      const std::int64_t candidate_delta = std::llabs(candidate->stamp_ns - target);
      if (candidate_delta < closest_delta) {
        closest = candidate;
        closest_delta = candidate_delta;
      }
    }
    pose = *closest;
    delta_ms = static_cast<double>(closest_delta) / 1.0e6;
    return delta_ms <= cloud_pose_tolerance_ms_;
  }

  static void configureMapBounds(const fast_planner::LIOInterface::Ptr &map,
                                 const Eigen::Vector3f &position,
                                 const Eigen::Vector3f &goal,
                                 double margin)
  {
    const Eigen::Vector3f lower = position.cwiseMin(goal) -
      Eigen::Vector3f::Constant(static_cast<float>(margin));
    const Eigen::Vector3f upper = position.cwiseMax(goal) +
      Eigen::Vector3f::Constant(static_cast<float>(margin));
    map->configureBounds(lower, upper);
  }

  static bool expandMapBounds(const fast_planner::LIOInterface::Ptr &map,
                              const Eigen::Vector3f &position,
                              const Eigen::Vector3f &goal,
                              double margin)
  {
    if (!map) return false;
    const Eigen::Vector3f lower = position.cwiseMin(goal) -
      Eigen::Vector3f::Constant(static_cast<float>(margin));
    const Eigen::Vector3f upper = position.cwiseMax(goal) +
      Eigen::Vector3f::Constant(static_cast<float>(margin));
    return map->expandBounds(lower, upper);
  }

  void startSkeletonRebuild()
  {
    if (rebuild_running_.exchange(true)) return;
    if (rebuild_thread_.joinable()) {
      if (rebuild_thread_.get_id() == std::this_thread::get_id()) {
        rebuild_thread_.detach();
      } else {
        rebuild_thread_.join();
      }
    }

    fast_planner::LIOInterface::Ptr source_map;
    Eigen::Vector3f position;
    Eigen::Vector3f goal;
    Eigen::Quaternionf orientation;
    std::uint64_t generation;
    ParallelBubbleAstar::Ptr current_astar;
    TopoGraph::Ptr current_topo;
    bool incremental_update = false;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      source_map = map_;
      position = position_;
      goal = goal_;
      orientation = orientation_;
      generation = goal_generation_;
      current_astar = astar_;
      current_topo = topo_;
      incremental_update = graph_initialized_.load() && skeleton_initialized_.load() &&
        current_topo && current_astar;
    }
    const auto accumulated = source_map->accumulatedCloudSnapshot();
    const auto latest = source_map->latestCloudSnapshot();
    // Preserve the ray-carved free-space evidence when a new map instance is
    // created. Without this snapshot, the first skeleton build can only see
    // occupied returns and cannot generate Bubbles in clear corridors.
    const auto free_space = source_map->freeSpaceSnapshot();
    const auto semantic_memory = semanticMemorySnapshot();
    map_changed_.store(false);
    last_skeleton_rebuild_time_ = std::chrono::steady_clock::now();
    have_skeleton_rebuild_time_ = true;

    rebuild_thread_ = std::thread(
      [this, source_map, accumulated, latest, free_space, semantic_memory,
       position, goal, orientation, generation,
       current_astar, current_topo, incremental_update]() mutable {
        const auto total_start = std::chrono::steady_clock::now();
        try {
          // Build against an immutable map snapshot.  The live map is still
          // updated by the depth callback while this thread runs; sharing it
          // with Bubble generation means one update can query two different
          // KD-tree states and reject its own freshly generated nodes.
          auto next_map = std::make_shared<fast_planner::LIOInterface>();
          next_map->configureBounds(
            source_map->lp_->global_box_min_boundary_,
            source_map->lp_->global_box_max_boundary_);
          next_map->lp_->max_ray_length_ = source_map->lp_->max_ray_length_;
          next_map->lp_->fov_up = source_map->lp_->fov_up;
          next_map->lp_->fov_down = source_map->lp_->fov_down;
          next_map->lp_->fov_vp_up = source_map->lp_->fov_vp_up;
          next_map->lp_->fov_vp_down = source_map->lp_->fov_vp_down;
          next_map->configureStorage(
            static_cast<float>(map_voxel_size_), static_cast<float>(map_history_radius_m_),
            static_cast<std::size_t>(std::max(map_max_points_, 1000)),
            static_cast<float>(map_prune_distance_m_));
          if (graph_fixed_layer_) {
            next_map->setGraphObstacleMinZ(static_cast<float>(graph_layer_z_ - 1.0));
          }
          if (!incremental_update) {
            // Bubble generation remains 3D; only the derived topology is planar.
            configureMapBounds(
              next_map, position, goal, std::max(map_margin_, map_history_radius_m_));
          }
          const auto snapshot_start = std::chrono::steady_clock::now();
          next_map->loadSnapshot(
            accumulated, latest, free_space, position, orientation);
          const double snapshot_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - snapshot_start).count();
          const auto init_start = std::chrono::steady_clock::now();
          auto next_astar = std::make_shared<ParallelBubbleAstar>();
          auto next_topo = incremental_update ? current_topo :
            std::make_shared<TopoGraph>();
          ros::NodeHandle nh(shared_from_this());
          next_astar->init(nh, next_map);
          if (!incremental_update) {
            next_topo->init(nh, next_map, next_astar);
            next_topo->loadSemanticMemory(semantic_memory);
          }
          next_astar->planar_search_ = graph_fixed_layer_;
          next_astar->planar_z_ = static_cast<float>(graph_layer_z_);
          next_topo->planar_graph_ = graph_fixed_layer_;
          next_topo->planar_z_ = static_cast<float>(graph_layer_z_);
          const double init_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - init_start).count();

          const auto regions_start = std::chrono::steady_clock::now();
          double regions_ms = 0.0;
          double skeleton_ms = 0.0;
          double odom_ms = 0.0;
          TopoGraphUpdateTiming timing;
          {
            // ScaleNav's updateSkeleton is an in-place V_remove/V_remain/V_insert
            // diff. Keep readers out until that diff and odom reconnection are
            // complete; rebuilding a detached copy defeats the incremental
            // graph and allows readers to observe half-removed nodes.
            std::lock_guard<std::mutex> topology_lock(topology_operation_mutex_);
            if (incremental_update) {
              next_topo->lidar_map_interface_ = next_map;
              next_topo->parallel_bubble_astar_ = next_astar;
            }
            next_topo->setUpdateGoal(goal);
            next_topo->getRegionsToUpdate();
            regions_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - regions_start).count();
            const auto skeleton_start = std::chrono::steady_clock::now();
            next_topo->updateSkeleton();
            skeleton_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - skeleton_start).count();

            const auto odom_start = std::chrono::steady_clock::now();
            if (!incremental_update) {
              // The asynchronous rebuild captured this pose before doing the
              // map and topology work. It is suitable for a new graph, but it
              // must not overwrite the live odom node in a shared incremental
              // graph after the vehicle has moved. The planner timer owns
              // odom reconnection once the graph has been initialized.
              float yaw = std::atan2((orientation * Eigen::Vector3f::UnitX()).y(),
                                     (orientation * Eigen::Vector3f::UnitX()).x());
              Eigen::Vector3f rebuild_position = position;
              next_topo->updateOdomNode(rebuild_position, yaw);
              ensureOdomConnectivity(next_topo, rebuild_position);
            }
            odom_ms = std::chrono::duration<double, std::milli>(
              std::chrono::steady_clock::now() - odom_start).count();
            timing = next_topo->getLastUpdateTiming();
            mergeSemanticMemory(next_topo->semanticMemorySnapshot());
          }

          if (!incremental_update && (timing.bubbles == 0 || timing.new_nodes == 0)) {
            map_changed_.store(true);
            RCLCPP_WARN(get_logger(),
              "ScaleNav rebuild rejected: no real Bubble topology (points=%zu bubbles=%zu nodes=%zu)",
              accumulated.size(), timing.bubbles, timing.new_nodes);
            rebuild_running_.store(false);
            return;
          }

          if (!shutting_down_.load() && generation == goal_generation_.load()) {
            std::lock_guard<std::mutex> lock(graph_mutex_);
            if (!incremental_update) {
              map_ = next_map;
              astar_ = std::move(next_astar);
              topo_ = std::move(next_topo);
            } else {
              // Keep the live map fed by depth callbacks. Replacing it with the
              // rebuild snapshot rolled obstacles back and stalled route updates.
              astar_ = std::move(next_astar);
              astar_->lidar_map_interface_ = map_;
              topo_->parallel_bubble_astar_ = astar_;
              topo_->lidar_map_interface_ = map_;
            }
            graph_initialized_ = true;
            skeleton_initialized_ = true;
            ++skeleton_update_count_;
            ++topology_update_generation_;
          }
          const double total_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - total_start).count();
          std::ostringstream timing_json;
          timing_json << std::fixed << std::setprecision(6)
            << "{\"module\":\"background\",\"stamp_ns\":" << now().nanoseconds()
            << ",\"mode\":\"" << (incremental_update ? "incremental" : "initialize") << '"'
            << ",\"snapshot_ms\":" << snapshot_ms
            << ",\"init_ms\":" << init_ms
            << ",\"region_ms\":" << regions_ms
            << ",\"skeleton_ms\":" << skeleton_ms
            << ",\"odom_ms\":" << odom_ms
            << ",\"total_ms\":" << total_ms
            << ",\"points\":" << accumulated.size()
            << ",\"regions\":" << timing.regions
            << ",\"bubbles\":" << timing.bubbles
            << ",\"new_nodes\":" << timing.new_nodes
            << ",\"inserted_nodes\":" << timing.inserted_nodes
            << ",\"removed_nodes\":" << timing.removed_nodes
            << ",\"edge_candidates\":" << timing.insert_candidate_edges
            << ",\"edge_success\":" << timing.insert_success_edges
            << ",\"edge_timeout\":" << timing.insert_timeout_edges
            << ",\"edge_collision_reject\":" << timing.insert_collision_reject_edges
            << '}';
          publishTiming(timing_json.str());
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), diagnostic_log_period_ms_,
            "[ScaleNav timing][background %s] points=%zu snapshot_kdtree=%.3f ms "
            "init=%.3f ms regions=%zu occupied_regions=%zu "
            "free_regions=%zu region_select=%.3f ms skeleton=%.3f ms "
            "odom_connect=%.3f ms "
            "total=%.3f ms bubbles_3d=%zu bubbles_planar=%zu nodes=%zu "
            "remained=%zu inserted=%zu removed=%zu deferred=%zu planar_rejected=%zu "
            "edge_candidates=%zu edge_success=%zu edge_timeout=%zu "
            "edge_no_path=%zu edge_collision_reject=%zu "
            "exist_checked=%zu exist_kept=%zu exist_repaired=%zu "
            "exist_removed=%zu exist_soft_retry=%zu exist_cooldown_skip=%zu "
            "duplicate_merged=%zu half_edges_removed=%zu "
            "semantic_restored=%zu semantic_memory=%zu",
            incremental_update ? "incremental" : "initialize",
            accumulated.size(), snapshot_ms, init_ms,
            timing.regions, timing.occupied_regions, timing.free_regions,
            regions_ms, skeleton_ms, odom_ms, total_ms,
            timing.bubbles, timing.planar_bubbles, timing.new_nodes,
            timing.remained_nodes, timing.inserted_nodes, timing.removed_nodes,
            timing.deferred_nodes,
            timing.bubbles >= timing.planar_bubbles ?
              timing.bubbles - timing.planar_bubbles : 0UL,
            timing.insert_candidate_edges, timing.insert_success_edges,
            timing.insert_timeout_edges, timing.insert_no_path_edges,
            timing.insert_collision_reject_edges,
            timing.existing_edges_checked, timing.existing_edges_kept,
            timing.existing_edges_repaired, timing.existing_edges_removed,
            timing.existing_edges_soft_retry,
            timing.existing_edges_cooldown_skipped,
            timing.duplicate_nodes_merged, timing.half_edges_removed,
            timing.semantic_restored_nodes, timing.semantic_memory_records);
        } catch (const std::exception &error) {
          RCLCPP_ERROR(get_logger(), "ScaleNav background rebuild failed: %s", error.what());
          map_changed_.store(true);
        }
        rebuild_running_.store(false);
      });
  }

  void update()
  {
    if (!have_odom_ || !have_goal_ || !have_cloud_) return;
    const auto start = std::chrono::steady_clock::now();
    double odom_ms = 0.0;
    double astar_ms = 0.0;
    double publish_ms = 0.0;
    const auto now_steady = std::chrono::steady_clock::now();
    const bool rebuild_due = !have_skeleton_rebuild_time_ ||
      std::chrono::duration<double, std::milli>(
      now_steady - last_skeleton_rebuild_time_).count() >= skeleton_rebuild_period_ms_;
    if ((!skeleton_initialized_ || map_changed_.load()) && rebuild_due &&
        !rebuild_running_.load()) {
      startSkeletonRebuild();
    }

    TopoGraph::Ptr active_topo;
    ParallelBubbleAstar::Ptr active_astar;
    fast_planner::LIOInterface::Ptr active_map;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      active_topo = topo_;
      active_astar = astar_;
      active_map = map_;
      if (active_topo && active_map) {
        active_topo->lidar_map_interface_ = active_map;
        if (active_astar) {
          active_astar->lidar_map_interface_ = active_map;
          active_topo->parallel_bubble_astar_ = active_astar;
        }
      }
    }
    if (!graph_initialized_ || !active_topo || !active_topo->odom_node_) return;
    // Rebuild mutates the same TopoGraph. Wait for that diff to finish rather
    // than skipping the planner tick — a missed subgoal publish looks like
    // "planning stopped" to YOPO.
    std::lock_guard<std::mutex> topology_lock(topology_operation_mutex_);
    const bool semantic_frame_applied = updateTopoSemanticMemory(active_topo);
    // Semantic links are provisional and must follow the live lidar map.  A
    // new obstacle can invalidate an edge even when PEARL has not produced a
    // newer heatmap frame, so perform this check on every planner tick.
    const std::size_t removed_semantic_edges = active_topo->revalidateSemanticEdges();
    if (removed_semantic_edges > 0) {
      RCLCPP_WARN(get_logger(),
        "[ScaleNav semantic edge] removed=%zu blocked ordinary-semantic links",
        removed_semantic_edges);
      map_changed_.store(true);
    }
    bool accepted_route_forced_replan = false;
    const char *accepted_route_forced_reason = "NONE";
    bool accepted_route_stale_but_safe = false;
    // Capture continuity before any safety check can invalidate route state.
    // Velocity is the best description of the motion that a replacement
    // route would have to reverse; at low speed use the accepted route ahead.
    Eigen::Vector3f route_continuity_direction = world_velocity_;
    route_continuity_direction.z() = 0.0F;
    if (route_continuity_direction.norm() <= 0.5F &&
        accepted_route_.valid && accepted_route_.witness_path.size() >= 2) {
      Eigen::Vector3f ahead;
      if (scalenav_graph::routeLookaheadPoint(
            accepted_route_.witness_path, position_, 4.0F, ahead)) {
        route_continuity_direction = ahead - position_;
        route_continuity_direction.z() = 0.0F;
      }
    }
    if (route_continuity_direction.norm() <= 1e-3F) {
      route_continuity_direction = orientation_ * Eigen::Vector3f::UnitX();
      route_continuity_direction.z() = 0.0F;
    }
    // A route is executable only while every topology edge it references is
    // still present.  revalidateSemanticEdges() can detach an edge before the
    // planner reaches the accepted-route check; without this guard the
    // stale route would be held until the progress trigger and appear to
    // ignore the detach event.
    if (accepted_route_.valid && accepted_route_.topology_path.size() >= 2) {
      bool accepted_route_edge_missing = false;
      bool accepted_route_ordinary_semantic_failure = false;
      bool accepted_route_backbone_edge_missing = false;
      bool accepted_route_shortcut_blocked = false;
      std::uint64_t missing_from = 0;
      std::uint64_t missing_to = 0;
      for (std::size_t i = 1; i < accepted_route_.topology_path.size(); ++i) {
        const auto &from = accepted_route_.topology_path[i - 1];
        const auto &to = accepted_route_.topology_path[i];
        // The rolling odom vertex is replaced/reconnected as the vehicle
        // moves.  An accepted route may therefore retain the previous odom
        // pointer at its head even though the downstream segment is still
        // valid.  Do not treat that expected head remap as a detached edge;
        // all interior edges remain subject to the checks below.
        // The rolling odom vertex is intentionally disconnected and
        // reconnected later in this tick.  Its first edge is therefore not a
        // stable route-topology contract; reachability is checked after the
        // odom reconnect below.  Interior ordinary-semantic edges remain
        // subject to immediate detach validation.
        const bool stale_odom_head = i == 1 && from &&
          from->role_ == TopoNodeRole::Odom;
        if (stale_odom_head) continue;
        if (!from || !to) {
          accepted_route_edge_missing = true;
          if (from) missing_from = from->persistent_id_;
          if (to) missing_to = to->persistent_id_;
          break;
        }
        const bool ordinary_semantic_edge = isOrdinarySemanticLink(from, to);
        // Every stable interior edge belongs to the accepted A* node sequence.
        // If an ordinary edge disappears, holding the old node sequence would
        // publish a route that the current graph no longer contains. The odom
        // head remains the sole exception because it is reconnected below.
        const bool neighbor_present = from && to && from->neighbors_.count(to) > 0;
        const bool witness_present = from && to && [&]() {
          const auto it = from->paths_.find(to);
          return it != from->paths_.end() && it->second.size() >= 2;
        }();
        const bool weight_present = from && to && [&]() {
          const auto it = from->weight_.find(to);
          return it != from->weight_.end() && std::isfinite(it->second);
        }();
        if (!hasExecutableTopologyEdge(from, to)) {
          if (from) missing_from = from->persistent_id_;
          if (to) missing_to = to->persistent_id_;
          const char *failure = !from || !to ? "NULL_ENDPOINT" :
            !neighbor_present ? "NEIGHBOR_MISSING" :
            !witness_present ? "WITNESS_MISSING" :
            !weight_present ? "WEIGHT_MISSING" : "EDGE_NOT_EXECUTABLE";
          RCLCPP_WARN(
            get_logger(),
            "[ScaleNav route edge] topology failure %llu->%llu reason=%s "
            "kind=%s from_geom=%d to_geom=%d neighbor=%d witness=%d weight=%d "
            "action=%s from=(%.2f,%.2f,%.2f) to=(%.2f,%.2f,%.2f)",
            static_cast<unsigned long long>(missing_from),
            static_cast<unsigned long long>(missing_to), failure,
            ordinary_semantic_edge ? "ORDINARY_SEMANTIC" : "BACKBONE",
            from ? static_cast<int>(from->geometry_state_) : -1,
            to ? static_cast<int>(to->geometry_state_) : -1,
            static_cast<int>(neighbor_present), static_cast<int>(witness_present),
            static_cast<int>(weight_present), ordinary_semantic_edge ? "REPLAN" : "CHECK_WITNESS",
            from ? from->center_.x() : 0.0F,
            from ? from->center_.y() : 0.0F, from ? from->center_.z() : 0.0F,
            to ? to->center_.x() : 0.0F, to ? to->center_.y() : 0.0F,
            to ? to->center_.z() : 0.0F);
          accepted_route_edge_missing = true;
          if (ordinary_semantic_edge) {
            accepted_route_ordinary_semantic_failure = true;
          } else {
            accepted_route_backbone_edge_missing = true;
          }
          break;
        }
        if (ordinary_semantic_edge && active_topo->parallel_bubble_astar_) {
          std::vector<Eigen::Vector3f> direct{from->center_, to->center_};
          ParallelBubbleAstar::CollisionCheckInfo live_info;
          const bool live_safe = active_topo->parallel_bubble_astar_
            ->collisionCheck_shortenPath(direct, &live_info);
          const auto clearance_it = from->edge_clearance_.find(to);
          const double stored_clearance = clearance_it != from->edge_clearance_.end() ?
            static_cast<double>(clearance_it->second) : std::numeric_limits<double>::quiet_NaN();
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[ScaleNav route edge] semantic live check %llu->%llu safe=%d "
            "live_min_clearance=%.3f safe_distance=%.3f min_at=(%.2f,%.2f,%.2f) "
            "stored_clearance=%.3f from=(%.2f,%.2f,%.2f) to=(%.2f,%.2f,%.2f)",
            static_cast<unsigned long long>(from->persistent_id_),
            static_cast<unsigned long long>(to->persistent_id_),
            static_cast<int>(live_safe), live_info.minimum_clearance,
            active_topo->parallel_bubble_astar_->safe_distance_,
            live_info.minimum_clearance_point.x(),
            live_info.minimum_clearance_point.y(),
            live_info.minimum_clearance_point.z(), stored_clearance,
            from->center_.x(), from->center_.y(), from->center_.z(),
            to->center_.x(), to->center_.y(), to->center_.z());
          if (!live_safe) {
            accepted_route_edge_missing = true;
            missing_from = from->persistent_id_;
            missing_to = to->persistent_id_;
            RCLCPP_WARN(
              get_logger(),
              "[ScaleNav route edge] live collision failure %llu->%llu "
              "kind=ORDINARY_SEMANTIC from=(%.2f,%.2f,%.2f) to=(%.2f,%.2f,%.2f)",
              static_cast<unsigned long long>(missing_from),
              static_cast<unsigned long long>(missing_to), from->center_.x(),
              from->center_.y(), from->center_.z(), to->center_.x(),
              to->center_.y(), to->center_.z());
            break;
          }
        }
      }
      // Revalidate actual execution geometry when DP shortcutting removed
      // topology vertices. A shortcut chord has no neighbors_/paths_ entry,
      // so checking topology_path alone cannot detect a newly blocked chord.
      if (!accepted_route_ordinary_semantic_failure &&
          executionPathContainsShortcutChord(
            accepted_route_.topology_path, accepted_route_.execution_path) &&
          active_topo->parallel_bubble_astar_ &&
          accepted_route_.witness_path.size() >= 2) {
        auto remaining_execution = scalenav_graph::forwardRouteFromT(
          accepted_route_.witness_path,
          accepted_route_.frontier_goal_progress_t);
        if (remaining_execution.size() >= 2) {
          ParallelBubbleAstar::CollisionCheckInfo execution_info;
          const bool execution_safe = active_topo->parallel_bubble_astar_
            ->collisionCheck_shortenPath(remaining_execution, &execution_info);
          if (!execution_safe) {
            accepted_route_edge_missing = true;
            accepted_route_shortcut_blocked = true;
            const char *reason = execution_info.reason ==
                ParallelBubbleAstar::CollisionCheckInfo::CLEARANCE ? "CLEARANCE" :
              execution_info.reason == ParallelBubbleAstar::CollisionCheckInfo::BUBBLE_OVERLAP ?
                "BUBBLE_OVERLAP" : "INVALID_PATH";
            RCLCPP_WARN(
              get_logger(),
              "[ScaleNav route] shortcut execution suffix blocked; forcing fresh A* "
              "reason=%s points=%zu minimum_clearance=%.3f failed_index=%zu "
              "failed_point=(%.2f,%.2f,%.2f) clearance=%.3f safe_distance=%.3f",
              reason, remaining_execution.size(), execution_info.minimum_clearance,
              execution_info.failed_index, execution_info.failed_point.x(),
              execution_info.failed_point.y(), execution_info.failed_point.z(),
              execution_info.clearance,
              active_topo->parallel_bubble_astar_->safe_distance_);
          }
        }
      }
      // Sliding-window skeleton updates can replace a backbone edge while the
      // already accepted witness remains collision-free.  Keep executing that
      // complete segment in this case; only a semantic link failure or a live
      // witness collision is allowed to force an immediate replan.
      if (accepted_route_edge_missing && !accepted_route_ordinary_semantic_failure &&
          !accepted_route_shortcut_blocked && accepted_route_backbone_edge_missing &&
          active_topo->parallel_bubble_astar_ &&
          accepted_route_.witness_path.size() >= 2) {
        auto witness = scalenav_graph::forwardRouteFromT(
          accepted_route_.witness_path,
          accepted_route_.frontier_goal_progress_t);
        ParallelBubbleAstar::CollisionCheckInfo witness_info;
        const bool witness_safe = active_topo->parallel_bubble_astar_
          ->collisionCheck_shortenPath(witness, &witness_info);
        if (witness_safe) {
          accepted_route_stale_but_safe = true;
          accepted_route_edge_missing = false;
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[ScaleNav route] backbone edge rebuilt but accepted witness remains safe; "
            "holding route missing_edge=%llu->%llu min_clearance=%.3f",
            static_cast<unsigned long long>(missing_from),
            static_cast<unsigned long long>(missing_to),
            witness_info.minimum_clearance);
        } else {
          const char *witness_reason = witness_info.reason ==
              ParallelBubbleAstar::CollisionCheckInfo::CLEARANCE ? "CLEARANCE" :
            witness_info.reason == ParallelBubbleAstar::CollisionCheckInfo::BUBBLE_OVERLAP ?
              "BUBBLE_OVERLAP" :
            witness_info.reason == ParallelBubbleAstar::CollisionCheckInfo::INVALID_PATH ?
              "INVALID_PATH" : "NONE";
          RCLCPP_WARN(
            get_logger(),
            "[ScaleNav route] stale backbone witness unsafe; forcing replan "
            "missing_edge=%llu->%llu reason=%s witness_points=%zu "
            "minimum_clearance=%.3f failed_index=%zu failed_point=(%.2f,%.2f,%.2f) "
            "clearance=%.3f safe_distance=%.3f",
            static_cast<unsigned long long>(missing_from),
            static_cast<unsigned long long>(missing_to), witness_reason,
            witness.size(), witness_info.minimum_clearance,
            witness_info.failed_index, witness_info.failed_point.x(),
            witness_info.failed_point.y(), witness_info.failed_point.z(),
            witness_info.clearance,
            active_topo->parallel_bubble_astar_->safe_distance_);
        }
      }
      if (accepted_route_edge_missing) {
        RCLCPP_WARN(get_logger(),
          "[ScaleNav route] accepted route invalidated by detached/blocked edge "
          "%llu->%llu; forcing fresh A*",
          static_cast<unsigned long long>(missing_from),
          static_cast<unsigned long long>(missing_to));
        accepted_route_.clear();
        map_changed_.store(true);
        accepted_route_forced_replan = true;
        accepted_route_forced_reason = "ROUTE_TOPOLOGY_CHANGED";
      }
    }

    // Delay the first route publication until PEARL has supplied one valid,
    // synchronized frame. This keeps the first graph/path publication
    // semantically annotated instead of publishing a geometry-only route and
    // adding semantic points several ticks later. The gate is one-shot and
    // can be disabled for geometry-only runs.
    if (wait_for_initial_semantic_ && !initial_semantic_wait_complete_ && have_goal_) {
      bool semantic_ready = false;
      {
        std::lock_guard<std::mutex> semantic_lock(semantic_mutex_);
        if (semantic_frame_) {
          const double age_ms = static_cast<double>(std::llabs(
            get_clock()->now().nanoseconds() - semantic_frame_->stamp_ns)) / 1.0e6;
          semantic_ready = age_ms <= semantic_max_age_ms_;
        }
      }
      if (semantic_ready) {
        initial_semantic_wait_complete_ = true;
        RCLCPP_INFO(get_logger(),
          "[ScaleNav semantic] initial frame ready; publishing first annotated route");
      } else {
        if (!initial_semantic_wait_started_) {
          initial_semantic_wait_started_ = true;
          initial_semantic_wait_start_ = std::chrono::steady_clock::now();
        }
        const double waited_ms = std::chrono::duration<double, std::milli>(
          std::chrono::steady_clock::now() - initial_semantic_wait_start_).count();
        if (waited_ms < initial_semantic_wait_timeout_ms_) {
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
            "[ScaleNav semantic] waiting for initial PEARL frame before first route "
            "(%.0f/%.0f ms)", waited_ms, initial_semantic_wait_timeout_ms_);
          return;
        }
        initial_semantic_wait_complete_ = true;
        RCLCPP_WARN(get_logger(),
          "[ScaleNav semantic] initial-frame wait timed out after %.0f ms; "
          "continuing with geometry-only first route", waited_ms);
      }
    } else if (!wait_for_initial_semantic_) {
      initial_semantic_wait_complete_ = true;
    }

    // Keep graph-to-odometry connectivity current independently of the slower
    // route/subgoal planning clock.
    float yaw = std::atan2((orientation_ * Eigen::Vector3f::UnitX()).y(),
                           (orientation_ * Eigen::Vector3f::UnitX()).x());
    if (graph_odom_topo_ != active_topo && active_topo->odom_node_) {
      graph_odom_topo_ = active_topo;
      graph_odom_position_ = active_topo->odom_node_->center_;
      graph_odom_yaw_ = yaw;
      have_graph_odom_ = true;
    }
    const float yaw_delta = std::atan2(std::sin(yaw - graph_odom_yaw_),
                                      std::cos(yaw - graph_odom_yaw_));
    const bool graph_odom_disconnected = active_topo->odom_node_->neighbors_.empty();
    const bool graph_odom_center_stale =
      (position_ - active_topo->odom_node_->center_).norm() > odom_reconnect_distance_m_;
    if (!have_graph_odom_ || graph_odom_disconnected || graph_odom_center_stale ||
        (position_ - graph_odom_position_).norm() > odom_reconnect_distance_m_ ||
        std::abs(yaw_delta) > odom_reconnect_yaw_deg_ * static_cast<float>(M_PI / 180.0)) {
      const auto odom_start = std::chrono::steady_clock::now();
      Eigen::Vector3f graph_position = position_;
      active_topo->updateOdomNode(graph_position, yaw);
      ensureOdomConnectivity(active_topo, graph_position);
      odom_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - odom_start).count();
      graph_odom_position_ = position_;
      graph_odom_yaw_ = yaw;
      graph_odom_topo_ = active_topo;
      have_graph_odom_ = true;
    }

    // Keep Verified reachability separate from the final provisional semantic
    // edge. Unknown semantic endpoints remain valid frontier candidates, but
    // they cannot bridge disconnected Verified components in this hold check.
    AcceptedRouteConnectivity accepted_connectivity;
    if (accepted_route_.valid && !accepted_route_.topology_path.empty() &&
        active_topo->odom_node_) {
      accepted_connectivity = acceptedRouteConnectivity(
        active_topo->odom_node_, accepted_route_.topology_path);
    }
    bool accepted_route_reachable = accepted_connectivity.routeUsable() ||
      accepted_route_stale_but_safe;
    const bool accepted_route_head_attached =
      accepted_connectivity.accepted_head_edge_usable;
    if (accepted_route_.valid && !accepted_route_reachable) {
      // A rolling odom rebuild can temporarily lose the graph prefix to the
      // old Verified anchor while the already accepted geometric suffix is
      // still safe. Validate that suffix before throwing away the route.
      bool accepted_suffix_safe = false;
      if (accepted_connectivity.accepted_stable_edges_usable &&
          (!accepted_connectivity.has_terminal_unknown ||
           accepted_connectivity.terminal_unknown_edge_usable) &&
          active_topo->parallel_bubble_astar_ &&
          accepted_route_.witness_path.size() >= 2) {
        auto suffix = scalenav_graph::forwardRouteFromT(
          accepted_route_.witness_path, accepted_route_.frontier_goal_progress_t);
        if (suffix.size() >= 2) {
          ParallelBubbleAstar::CollisionCheckInfo suffix_info;
          accepted_suffix_safe = active_topo->parallel_bubble_astar_
            ->collisionCheck_shortenPath(suffix, &suffix_info);
          if (!accepted_suffix_safe) {
            RCLCPP_WARN(
              get_logger(),
              "[ScaleNav route] odom prefix disconnected and accepted suffix "
              "is unsafe; forcing fresh A* reason=%d failed_index=%zu clearance=%.3f",
              static_cast<int>(suffix_info.reason), suffix_info.failed_index,
              suffix_info.clearance);
          }
        }
      }
      if (accepted_suffix_safe) {
        accepted_route_stale_but_safe = true;
        accepted_route_reachable = true;
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[ScaleNav route] odom prefix temporarily disconnected but accepted "
          "witness suffix is safe; holding route frontier=%llu",
          static_cast<unsigned long long>(accepted_route_.frontier_goal_id));
      }
    }
    if (accepted_route_.valid && !accepted_route_reachable) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[ScaleNav route] accepted route is not executable from current odom "
        "(reachable=%d route_head=%d stable_edges=%d verified_prefix=%d terminal_unknown=%d "
        "terminal_edge_usable=%d verified_visited=%zu); forcing fresh "
        "odom-rooted A* frontier=%llu",
        static_cast<int>(accepted_route_reachable),
        static_cast<int>(accepted_connectivity.accepted_head_edge_usable),
        static_cast<int>(accepted_connectivity.accepted_stable_edges_usable),
        static_cast<int>(accepted_connectivity.verified_prefix_reachable),
        static_cast<int>(accepted_connectivity.has_terminal_unknown),
        static_cast<int>(accepted_connectivity.terminal_unknown_edge_usable),
        accepted_connectivity.verified_nodes_visited,
        static_cast<unsigned long long>(accepted_route_.frontier_goal_id));
      accepted_route_.clear();
      map_changed_.store(true);
      accepted_route_forced_replan = true;
      accepted_route_forced_reason =
        !accepted_connectivity.verified_prefix_reachable ?
        "ROUTE_UNREACHABLE" : "ROUTE_TOPOLOGY_CHANGED";
    }
    // ScaleNav publishes the rolling graph and local subgoal on every planner
    // tick.  The graph search itself is already bounded by local_graph_radius;
    // throttling this block to a multi-second period makes both the graph and
    // subgoal stale while the vehicle keeps moving.  Keep the configured
    // lookahead as a geometric distance instead of coupling it to speed and a
    // planning period (which can otherwise jump to 15--20 m at flight speed).
    const float effective_lookahead_m = static_cast<float>(
      std::max(0.0, local_goal_lookahead_m_));
    std::vector<TopoNode::Ptr> path_nodes;
    // Preserve the complete A* sequence separately; shortcut chords are
    // execution geometry, not topology edges.
    std::vector<TopoNode::Ptr> candidate_topology_path;
    std::unordered_set<std::pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash>
      last_path_edges;
    // Every planner tick starts a fresh mission-directed A* from odom. Keep
    // the historical-edge set empty so no previous route can bias the search.
    const bool route_aligned = false;
    const float route_lateral_error = std::numeric_limits<float>::infinity();
    const std::size_t geometrically_remembered_edges = 0;

    const auto astar_start = std::chrono::steady_clock::now();
    bool candidate_found = false;
    bool candidate_accepted = false;
    const char *route_switch_reason = "NONE";
    bool found = false;
    TopoGraphSearchStats incumbent_search_stats;
    TopoGraphSearchStats candidate_search_stats;
    Eigen::Vector3f layer_goal = goal_;
    if (graph_fixed_layer_) layer_goal.z() = static_cast<float>(graph_layer_z_);
    const float vehicle_to_goal = (position_ - layer_goal).norm();
    const bool goal_in_window = have_goal_ &&
      vehicle_to_goal <= static_cast<float>(local_graph_radius_m_);
    const float accepted_route_length = scalenav_graph::routeLength(accepted_route_.witness_path);
    if (accepted_route_.valid && accepted_route_length > 1e-3F) {
      float projected_t = 0.0F;
      bool progress_found = polynomial_curve_valid_ &&
        scalenav_graph::routeProgressTAlongCurve(
          polynomial_curve_, position_, accepted_route_.frontier_goal_progress_t,
          projected_t);
      // The progress trigger must remain functional even if a polynomial fit
      // is temporarily unavailable. Fall back to the exact accepted A* node
      // polyline; this affects only the replan trigger, never the route shape.
      if (!progress_found && accepted_route_.witness_path.size() >= 2) {
        const float projected_m = scalenav_graph::routeProgressAlongPath(
          accepted_route_.witness_path, position_);
        if (std::isfinite(projected_m)) {
          projected_t = std::clamp(projected_m / accepted_route_length, 0.0F, 1.0F);
          progress_found = true;
        }
      }
      if (progress_found) {
        accepted_route_.frontier_goal_progress_t = std::max(
          accepted_route_.frontier_goal_progress_t,
          std::clamp(projected_t, 0.0F, 1.0F));
        accepted_route_.frontier_goal_progress_m =
          accepted_route_.frontier_goal_progress_t * accepted_route_length;
      }
    }
    const float accepted_route_remaining =
      scalenav_graph::routeLength(scalenav_graph::forwardRouteFromT(
        accepted_route_.witness_path, accepted_route_.frontier_goal_progress_t));
    const bool accepted_witness_usable = accepted_route_.valid &&
      accepted_route_.witness_path.size() >= 2 &&
      accepted_route_.topology_path.size() >= 2;
    const bool frontier_progress_replan = accepted_witness_usable &&
      scalenav_graph::routeProgressReachedFraction(
        accepted_route_.frontier_goal_progress_m,
        accepted_route_.frontier_goal_initial_route_length_m,
        static_cast<float>(frontier_replan_progress_ratio_));
    const std::int64_t active_virtual_semantic_stamp_ns =
      activeVirtualSemanticStampNs();
    if (!mission_direct_goal_latched_ && have_goal_ &&
        scalenav_graph::missionGoalWithinDirectHorizon(
          vehicle_to_goal, static_cast<float>(goal_connect_distance_m_),
          effective_lookahead_m)) {
      mission_direct_goal_latched_ = true;
      RCLCPP_INFO(
        get_logger(),
        "[ScaleNav terminal] direct mission goal enabled at %.2f m; "
        "frontier A* extension disabled",
        static_cast<double>(vehicle_to_goal));
    }
    const bool mission_goal_direct = have_goal_ && mission_direct_goal_latched_;
    scalenav_graph::SemanticOpportunity semantic_opportunity;
    bool semantic_opportunity_observed = false;
    bool semantic_opportunity_persistent = false;
    bool semantic_opportunity_cooldown_ready = true;
    bool semantic_opportunity_waiting_for_progress = false;
    bool semantic_opportunity_replan = false;
    if (semantic_frame_applied) {
      std::optional<SemanticFrame> applied_frame;
      {
        std::lock_guard<std::mutex> lock(semantic_mutex_);
        if (semantic_frame_ &&
            semantic_frame_->stamp_ns != last_semantic_opportunity_evaluated_stamp_ns_) {
          applied_frame = semantic_frame_;
          last_semantic_opportunity_evaluated_stamp_ns_ = semantic_frame_->stamp_ns;
        }
      }
      if (applied_frame && accepted_witness_usable && !mission_goal_direct) {
        Eigen::Vector3f route_target = accepted_route_.frontier_goal;
        Eigen::Vector3f lookahead_target;
        if (scalenav_graph::routeLookaheadPoint(
              accepted_route_.witness_path, position_, effective_lookahead_m,
              lookahead_target)) {
          route_target = lookahead_target;
        }
        semantic_opportunity = scalenav_graph::evaluateSemanticOpportunity(
          applied_frame->points_world, applied_frame->scores,
          applied_frame->is_virtual, applied_frame->columns,
          applied_frame->origin, route_target,
          static_cast<float>(frontier_semantic_detour_budget_m_),
          static_cast<float>(frontier_semantic_noise_floor_));
        semantic_opportunity_observed = semantic_opportunity.valid;
        semantic_opportunity_persistent =
          scalenav_graph::updateSemanticOpportunityPersistence(
            semantic_opportunity,
            static_cast<float>(semantic_opportunity_switch_margin_m_),
            std::cos(static_cast<float>(semantic_opportunity_direction_tolerance_deg_) *
              static_cast<float>(M_PI / 180.0)),
            semantic_opportunity_persistence_frames_,
            pending_semantic_opportunity_direction_,
            pending_semantic_opportunity_frames_);
        if (have_semantic_opportunity_probe_time_) {
          const double elapsed_s = std::chrono::duration<double>(
            now_steady - last_semantic_opportunity_probe_time_).count();
          semantic_opportunity_cooldown_ready =
            elapsed_s >= semantic_opportunity_cooldown_s_;
        }
        if (!semantic_opportunity_cooldown_ready) {
          // A probe consumes the evidence that caused it. Do not carry a
          // previously accumulated persistence count through the cooldown;
          // otherwise the first frame after cooldown immediately retriggers.
          pending_semantic_opportunity_direction_.setZero();
          pending_semantic_opportunity_frames_ = 0;
          semantic_opportunity_persistent = false;
        }
        // A semantic opportunity is evidence for the next route decision, not
        // an immediate mid-segment steering command.  Let the accepted route
        // run until its configured progress boundary; otherwise alternating
        // heatmap frames can replace a route every 1-2 seconds before any
        // meaningful progress has been made.
        semantic_opportunity_waiting_for_progress = semantic_opportunity_persistent &&
          semantic_opportunity_cooldown_ready && !frontier_progress_replan;
        semantic_opportunity_replan = semantic_opportunity_persistent &&
          semantic_opportunity_cooldown_ready && frontier_progress_replan;
      }
    }
    const bool route_has_planning_horizon = accepted_witness_usable &&
      !frontier_progress_replan;
    const bool need_candidate_search = !accepted_witness_usable ||
      !accepted_route_reachable || frontier_progress_replan ||
      semantic_opportunity_replan;
    bool using_accepted_route = false;
    std::vector<TopoNode::Ptr> candidate_nodes;
    if (!need_candidate_search && accepted_witness_usable) {
      // Hold the complete route segment until its progress threshold is met.
      // No old and new paths are stitched together.
      path_nodes = accepted_route_.execution_path;
      found = true;
      using_accepted_route = true;
    } else {
      // Once the mission endpoint is inside the direct horizon, semantic
      // frontier ranking is no longer relevant. Keep A* available for a safe
      // topology path, but exclude virtual semantic endpoints from this
      // terminal search.
      const std::int64_t search_semantic_stamp_ns = mission_goal_direct ?
        -1 : active_virtual_semantic_stamp_ns;
      candidate_found = active_topo->goalDirectedSearch(
        active_topo->odom_node_, goal_, path_nodes, 0.0,
        static_cast<float>(goal_path_cost_weight_),
        1.0F,
        last_path_edges,
        static_cast<float>(semantic_cost_weight_),
        static_cast<float>(local_graph_radius_m_),
        &position_, 0.0F, goal_in_window,
        std::numeric_limits<float>::infinity(),
        effective_lookahead_m, &route_continuity_direction, 0.0F,
        0.0F, static_cast<float>(frontier_direction_loss_weight_), 0.0F, 0.0F,
        &candidate_search_stats, search_semantic_stamp_ns,
        static_cast<float>(frontier_goal_distance_weight_),
        static_cast<float>(frontier_semantic_detour_budget_m_),
        static_cast<float>(frontier_semantic_frame_budget_m_),
        static_cast<float>(frontier_semantic_noise_floor_));
      candidate_nodes.swap(path_nodes);
      if (candidate_found && candidate_nodes.size() < 2) {
        candidate_found = false;
        candidate_nodes.clear();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "ScaleNav rejected topology search result without node path");
      }
      if (candidate_found) {
        candidate_topology_path = candidate_nodes;
        path_nodes = candidate_topology_path;
        found = true;
        candidate_accepted = true;
        if (!path_nodes.empty()) {
          const auto &selected = path_nodes.back();
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), diagnostic_log_period_ms_,
            "[ScaleNav frontier selected] id=%llu type=%s column=%d score=%.3f "
            "confidence=%.3f geometry=%s center=(%.2f,%.2f,%.2f)",
            static_cast<unsigned long long>(selected->persistent_id_),
            isVirtualSemanticEndpoint(selected) ? "VIRTUAL_SEMANTIC" :
              (selected->geometry_state_ == TopoGeometryState::Verified ?
                "ORDINARY_VERIFIED" : "OTHER"),
            static_cast<int>(selected->semantic_column_), selected->semantic_score_,
            selected->semantic_confidence_,
            selected->geometry_state_ == TopoGeometryState::Verified ? "VERIFIED" : "UNKNOWN",
            selected->center_.x(), selected->center_.y(), selected->center_.z());
        }
        route_switch_reason = accepted_route_forced_replan ?
          accepted_route_forced_reason :
          (frontier_progress_replan ? "FRONTIER_PROGRESS" :
          (semantic_opportunity_replan ? "SEMANTIC_OPPORTUNITY" :
          (accepted_witness_usable ? "ROUTE_UNREACHABLE" : "INITIAL_ACCEPT")));
      } else if (accepted_witness_usable) {
        // A failed refresh does not splice a partial candidate into the old
        // route. Hold the previous complete segment until a new full search
        // succeeds (publish() will still reject it if its witness is unsafe).
        path_nodes = accepted_route_.execution_path;
        found = true;
        using_accepted_route = true;
        route_switch_reason = "REPLAN_FAILED_HOLD";
      } else {
        // No complete frontier route is available.  If the current direct
        // route was invalidated by an obstacle, still move to the safest
        // reachable measured neighbour instead of holding the blocked target.
        // This is a one-edge escape route built entirely from already
        // verified topology and cannot use a semantic detour.
        TopoNode::Ptr escape;
        const Eigen::Vector3f mission_delta = layer_goal - position_;
        const float mission_norm = mission_delta.norm();
        Eigen::Vector3f mission_dir = Eigen::Vector3f::UnitY();
        if (mission_norm > 1e-3F) mission_dir = mission_delta / mission_norm;
        float best_progress = -std::numeric_limits<float>::infinity();
        for (const auto &neighbor : active_topo->odom_node_->neighbors_) {
          if (!neighbor || neighbor->geometry_state_ != TopoGeometryState::Verified)
            continue;
          const Eigen::Vector3f delta = neighbor->center_ - position_;
          const float forward = delta.dot(mission_dir);
          if (forward <= 0.25F) continue;
          const auto edge = active_topo->odom_node_->paths_.find(neighbor);
          if (edge == active_topo->odom_node_->paths_.end() || edge->second.size() < 2)
            continue;
          auto direct = std::vector<Eigen::Vector3f>{active_topo->odom_node_->center_, neighbor->center_};
          if (!active_topo->parallel_bubble_astar_->collisionCheck_shortenPath(direct))
            continue;
          if (forward > best_progress) { best_progress = forward; escape = neighbor; }
        }
        if (escape) {
          path_nodes = {active_topo->odom_node_, escape};
          candidate_topology_path = path_nodes;
          found = true;
          candidate_accepted = true;
          route_switch_reason = "OBSTACLE_ESCAPE";
          RCLCPP_WARN(get_logger(),
            "[ScaleNav route] frontier search failed; selecting forward verified escape node id=%llu",
            static_cast<unsigned long long>(escape->persistent_id_));
        }
      }
    }
    std::size_t shortcut_count = 0;
    const std::size_t original_path_nodes = path_nodes.size();
    if (found && candidate_accepted && path_nodes.size() >= 3) {
      path_nodes = shortcutBubblePath(active_topo, path_nodes, shortcut_count);
      candidate_nodes = path_nodes;
      if (shortcut_count > 0) {
        RCLCPP_INFO(
          get_logger(),
          "[ScaleNav bubble shortcut] removed=%zu path_nodes=%zu->%zu",
          shortcut_count, original_path_nodes, path_nodes.size());
      }
    }
    // A fresh search replaces the complete route segment. Between progress
    // thresholds the previously planned segment is held intact; it is never
    // stitched to a new prefix or remapped through old topology nodes.
    auto route_points = [](const std::vector<TopoNode::Ptr> &nodes) {
      std::vector<Eigen::Vector3f> points;
      points.reserve(nodes.size());
      for (const auto &node : nodes) if (node) points.push_back(node->center_);
      return points;
    };
    const float frontier_objective_scale = std::max(
      1.0F, (layer_goal - position_).norm());
    struct RouteMetrics {
      float risk;
      float route_cost;
      float goal_distance;
      float objective;
      float progress;
    };
    const float local_semantic_radius_m = static_cast<float>(
      local_graph_radius_m_ + std::max(0.0, semantic_route_influence_m_));
    std::size_t local_inactive_virtual_semantic_nodes = 0;
    const auto local_semantic_nodes = active_topo->semanticRiskNodes(
      &position_, local_semantic_radius_m, active_virtual_semantic_stamp_ns,
      &local_inactive_virtual_semantic_nodes);
    const auto metrics_for = [&](const std::vector<TopoNode::Ptr> &nodes) {
      const auto points = route_points(nodes);
      RouteMetrics metrics{
        semanticRiskAlongRoute(active_topo, points), 0.0F,
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity(), 0.0F};
      if (points.empty()) return metrics;
      metrics.progress = (points.back() - position_).norm();
      for (std::size_t i = 1; i < nodes.size(); ++i) {
        const auto &from = nodes[i - 1];
        const auto &to = nodes[i];
        if (!from || !to) continue;
        metrics.route_cost += active_topo->routeEdgeCost(
          from, to, static_cast<float>(goal_path_cost_weight_),
          static_cast<float>(semantic_cost_weight_), false, 1.0F,
          &local_semantic_nodes);
      }
      metrics.goal_distance = (points.back() - layer_goal).norm();
      // The frontier selector adds a frame-relative semantic term for virtual
      // endpoints. Apply the same term to the incumbent so a probe compares
      // complete routes under one objective instead of receiving a free route
      // switch merely because it was triggered by a heatmap change.
      const auto &endpoint = nodes.back();
      if (endpoint && isVirtualSemanticEndpoint(endpoint)) {
        const auto accumulated_risk = [](const TopoNode::Ptr &node) {
          if (!node) return 0.0F;
          return node->semantic_risk_ > 0.0F ?
            std::clamp(node->semantic_risk_, 0.0F, 1.0F) :
            std::clamp(node->semantic_score_, 0.0F, 1.0F);
        };
        float frame_min = accumulated_risk(endpoint);
        float frame_max = frame_min;
        for (const auto &semantic_node : local_semantic_nodes) {
          if (!semantic_node || !isVirtualSemanticEndpoint(semantic_node) ||
              semantic_node->semantic_frame_stamp_ns_ != endpoint->semantic_frame_stamp_ns_)
            continue;
          const float score = accumulated_risk(semantic_node);
          frame_min = std::min(frame_min, score);
          frame_max = std::max(frame_max, score);
        }
        const float frame_range = std::max(0.0F, frame_max - frame_min);
        const float regret = std::clamp(
          (accumulated_risk(endpoint) - frame_min) /
            std::max(static_cast<float>(frontier_semantic_noise_floor_), frame_range),
          0.0F, 1.0F);
        metrics.route_cost += static_cast<float>(frontier_semantic_frame_budget_m_) * frame_min +
          static_cast<float>(frontier_semantic_detour_budget_m_) * regret;
      }
      // Keep route arbitration identical to frontier A*: semantic and
      // clearance costs remain in route_cost, while mission distance uses the
      // configured frontier weight and the same dimensionless scale.
      metrics.objective = (metrics.route_cost +
        static_cast<float>(frontier_goal_distance_weight_) * metrics.goal_distance) /
        frontier_objective_scale;
      return metrics;
    };
    RouteMetrics incumbent_metrics{
      0.0F, 0.0F, std::numeric_limits<float>::infinity(),
      std::numeric_limits<float>::infinity(), 0.0F};
    if (accepted_route_.valid && accepted_route_.topology_path.size() >= 2) {
      auto incumbent_nodes = accepted_route_.topology_path;
      // The rolling odom node is replaced as the vehicle moves. Compare from
      // the current odom while retaining the accepted downstream topology.
      if (incumbent_nodes.front() && incumbent_nodes.front()->role_ == TopoNodeRole::Odom &&
          active_topo->odom_node_) {
        incumbent_nodes.front() = active_topo->odom_node_;
      }
      incumbent_metrics = metrics_for(incumbent_nodes);
    }
    RouteMetrics candidate_metrics{
      0.0F, 0.0F, std::numeric_limits<float>::infinity(),
      std::numeric_limits<float>::infinity(), 0.0F};
    if (candidate_found) {
      const auto &candidate_metric_nodes = candidate_topology_path.empty() ?
        candidate_nodes : candidate_topology_path;
      candidate_metrics = metrics_for(candidate_metric_nodes);
    }
    const bool compared_route_metrics = std::isfinite(incumbent_metrics.objective);
    const float current_route_risk = incumbent_metrics.risk;
    const char *incumbent_result = compared_route_metrics ? "ACCEPTED_ROUTE" : "NONE";
    const double incumbent_goal_distance_log =
      std::isfinite(incumbent_metrics.goal_distance) ?
      static_cast<double>(incumbent_metrics.goal_distance) : -1.0;
    const double candidate_goal_distance_log =
      std::isfinite(candidate_metrics.goal_distance) ?
      static_cast<double>(candidate_metrics.goal_distance) : -1.0;
    const double incumbent_route_cost_log =
      std::isfinite(incumbent_metrics.route_cost) ?
      static_cast<double>(incumbent_metrics.route_cost) : -1.0;
    const double candidate_route_cost_log =
      std::isfinite(candidate_metrics.route_cost) ?
      static_cast<double>(candidate_metrics.route_cost) : -1.0;
    const double incumbent_objective_log =
      std::isfinite(incumbent_metrics.objective) ?
      static_cast<double>(incumbent_metrics.objective) : -1.0;
    const double candidate_objective_log =
      std::isfinite(candidate_metrics.objective) ?
      static_cast<double>(candidate_metrics.objective) : -1.0;
    if (found &&
        (path_nodes.size() < 2 || !active_topo->odom_node_ ||
         active_topo->odom_node_->neighbors_.empty())) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ScaleNav rejected route without a connected topology head: path_nodes=%zu odom_degree=%zu",
        path_nodes.size(), active_topo->odom_node_ ?
          active_topo->odom_node_->neighbors_.size() : 0U);
      path_nodes.clear();
      found = false;
    }
    std::size_t reused_path_edges = 0;
    for (std::size_t i = 1; i < path_nodes.size(); ++i) {
      if (last_path_edges.find({path_nodes[i - 1], path_nodes[i]}) != last_path_edges.end()) {
        ++reused_path_edges;
      }
    }
    astar_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - astar_start).count();
    // Keep the candidate frontier_goal local until publish() has accepted the
    // complete witness.  A topology path is not an incumbent by itself.
    const bool proposed_have_frontier_goal = found && !path_nodes.empty();
    // Once the vehicle enters the terminal horizon, the mission endpoint is
    // the frontier goal. Do not leave the previous exploration frontier
    // latched in RViz or route state while the local goal has already switched
    // to the mission endpoint.
    const Eigen::Vector3f proposed_frontier_goal = projectPlanningPoint(
      mission_goal_direct ? layer_goal :
      ((found && !path_nodes.empty()) ? path_nodes.back()->center_ :
       accepted_route_.frontier_goal),
      graph_fixed_layer_, graph_layer_z_);
    const std::uint64_t proposed_frontier_goal_id = mission_goal_direct ? 0ULL :
      ((found && !path_nodes.empty()) ? path_nodes.back()->persistent_id_ :
       accepted_route_.frontier_goal_id);
    if (semantic_opportunity_replan && candidate_found && accepted_route_.valid) {
      const float incumbent_loss_m = std::isfinite(incumbent_metrics.objective) ?
        incumbent_metrics.objective * frontier_objective_scale :
        std::numeric_limits<float>::infinity();
      const float candidate_loss_m = std::isfinite(candidate_metrics.objective) ?
        candidate_metrics.objective * frontier_objective_scale :
        std::numeric_limits<float>::infinity();
      const float improvement_m = incumbent_loss_m - candidate_loss_m;
      const bool same_endpoint = proposed_frontier_goal_id == accepted_route_.frontier_goal_id &&
        (proposed_frontier_goal - accepted_route_.frontier_goal).norm() <= 1e-3F;
      const bool switch_allowed = !same_endpoint && std::isfinite(improvement_m) &&
        improvement_m >= static_cast<float>(semantic_opportunity_switch_margin_m_);
      RCLCPP_INFO(
        get_logger(),
        "[ScaleNav semantic probe] incumbent=%.3f candidate=%.3f "
        "improvement=%.2f m margin=%.2f same_endpoint=%d switch=%d old_id=%llu new_id=%llu",
        static_cast<double>(incumbent_metrics.objective),
        static_cast<double>(candidate_metrics.objective),
        static_cast<double>(improvement_m), semantic_opportunity_switch_margin_m_,
        static_cast<int>(same_endpoint),
        static_cast<int>(switch_allowed),
        static_cast<unsigned long long>(accepted_route_.frontier_goal_id),
        static_cast<unsigned long long>(proposed_frontier_goal_id));
      if (!switch_allowed) {
        // The probe is diagnostic only when it does not beat the incumbent by
        // the configured hysteresis margin. Keep the complete accepted segment.
      path_nodes = accepted_route_.execution_path;
      found = true;
      using_accepted_route = true;
      candidate_accepted = false;
      route_switch_reason = "SEMANTIC_OPPORTUNITY_HOLD";
      }
    }
    // A semantic search is a consumed probe whether it switches, holds, or
    // fails. Without this reset, persistence stays above its threshold and
    // every subsequent heatmap frame launches another A* search.
    if (semantic_opportunity_replan) {
      last_semantic_opportunity_probe_time_ = now_steady;
      have_semantic_opportunity_probe_time_ = true;
      pending_semantic_opportunity_direction_.setZero();
      pending_semantic_opportunity_frames_ = 0;
    }
    const std::size_t route_memory_points = accepted_route_.witness_path.size();
    const auto publish_start = std::chrono::steady_clock::now();
    const float publish_progress_t = using_accepted_route ?
      accepted_route_.frontier_goal_progress_t : 0.0F;
    const auto stats = publish(
      active_topo, path_nodes, found,
      effective_lookahead_m, publish_progress_t, candidate_accepted);
    publish_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - publish_start).count();
    if (found && !stats.witness_collision_free) {
      // A topology search may reuse an edge witness that became stale while
      // the map was updated. Do not retain its frontier_goal as an executable
      // route; the next tick must start a fresh candidate search.  A failed
      // new candidate does not invalidate an already accepted route unless
      // the current accepted witness was independently blocked above.
      found = false;
      path_nodes.clear();
      if (using_accepted_route) accepted_route_.clear();
      // The candidate curve was provisional until the exact witness passed
      // the final collision check.
      polynomial_guide_path_.clear();
      polynomial_curve_ = scalenav_graph::WitnessParametricCurve();
      polynomial_curve_valid_ = false;
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[ScaleNav route rejected] final published witness failed collision check; "
        "frontier goal state cleared for fresh search");
    }
    if (found && stats.witness_collision_free && !using_accepted_route) {
      // Commit route state only after the exact witness that will be executed
      // has passed the publish-level collision check.
      const std::uint64_t old_frontier_goal_id = accepted_route_.frontier_goal_id;
      // The unshortened A* sequence is the only topology contract. The
      // shortcut sequence is used solely for node-center execution/trajectory
      // generation; its chords are not entries in neighbors_/paths_.
      accepted_route_.topology_path = candidate_topology_path.empty() ?
        path_nodes : candidate_topology_path;
      accepted_route_.execution_path = path_nodes;
      accepted_route_.witness_path = stats.witness_path;
      const bool frontier_goal_changed = !accepted_route_.valid ||
        proposed_frontier_goal_id != accepted_route_.frontier_goal_id ||
        (proposed_frontier_goal - accepted_route_.frontier_goal).norm() > 1e-3F;
      accepted_route_.frontier_goal = proposed_frontier_goal;
      accepted_route_.frontier_goal_id = proposed_frontier_goal_id;
      accepted_route_.valid = proposed_have_frontier_goal;
      if (frontier_goal_changed || candidate_accepted) {
        accepted_route_.frontier_goal_initial_route_length_m =
          scalenav_graph::routeLength(accepted_route_.witness_path);
        accepted_route_.frontier_goal_progress_m = 0.0F;
        accepted_route_.frontier_goal_progress_t = 0.0F;
      }
      RCLCPP_INFO(
        get_logger(),
        "[ScaleNav route plan] reason=%s old_frontier_goal_id=%llu new_frontier_goal_id=%llu "
        "route_aligned=%d route_lateral_error=%.2f m "
        "frontier_weight=%.2f objective_scale=%.2f incumbent_distance=%.2f "
        "candidate_distance=%.2f incumbent_objective=%.3f candidate_objective=%.3f "
        "route_length=%.2f route_remaining=%.2f",
        route_switch_reason,
        static_cast<unsigned long long>(old_frontier_goal_id),
        static_cast<unsigned long long>(accepted_route_.frontier_goal_id),
        static_cast<int>(route_aligned), static_cast<double>(route_lateral_error),
        frontier_goal_distance_weight_, frontier_objective_scale,
        incumbent_goal_distance_log, candidate_goal_distance_log,
        incumbent_objective_log, candidate_objective_log,
        static_cast<double>(scalenav_graph::routeLength(stats.witness_path)),
        static_cast<double>(scalenav_graph::routeLength(stats.witness_path)));
    }
    if (!found) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ScaleNav rolling route has no reachable real Bubble topology: "
        "odom_degree=%zu skeleton_nodes=%zu edges=%zu",
        active_topo->odom_node_->neighbors_.size(), stats.skeleton_nodes, stats.edges);
    }

    const auto end = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(end - start).count();
    const std::size_t persistent_semantic_records = persistentSemanticRecordCount();
    const std::size_t global_semantic_nodes =
      stats.semantic_nodes + stats.virtual_semantic_nodes;
    const std::size_t local_graph_nodes = active_topo->nodeCountWithinRadius(
      position_, static_cast<float>(local_graph_radius_m_));
    const std::size_t astar_searches = static_cast<std::size_t>(need_candidate_search);
    const std::size_t astar_expanded_nodes = candidate_search_stats.expanded_nodes;
    const std::size_t astar_edge_evaluations = candidate_search_stats.edge_evaluations;
    const std::size_t astar_semantic_nodes = candidate_search_stats.semantic_query_nodes;
    const std::size_t astar_inactive_virtual_semantic_nodes =
      candidate_search_stats.semantic_inactive_virtual_nodes_skipped;
    const std::size_t astar_semantic_checks = candidate_search_stats.semantic_candidate_checks;
    const bool astar_timed_out = candidate_search_stats.timed_out;
    const bool waiting_for_frontier_extension = false;
    const char *route_decision = candidate_accepted && found && stats.witness_collision_free ?
      "CANDIDATE_COMMITTED" :
      (using_accepted_route && found && stats.witness_collision_free ? "ROUTE_HELD" :
      (candidate_found && !stats.witness_collision_free ? "CANDIDATE_WITNESS_REJECTED" :
      (candidate_found ? "CANDIDATE_REJECTED" : "NO_CANDIDATE")));
    std::ostringstream timing_json;
    timing_json << std::fixed << std::setprecision(6)
      << "{\"module\":\"planner\",\"stamp_ns\":" << now().nanoseconds()
      << ",\"odom_ms\":" << odom_ms
      << ",\"astar_ms\":" << astar_ms
      << ",\"publish_ms\":" << publish_ms
      << ",\"total_ms\":" << ms
      << ",\"searched\":" << (need_candidate_search ? "true" : "false")
      << ",\"found\":" << (found ? "true" : "false")
      << ",\"candidate_found\":" << (candidate_found ? "true" : "false")
      << ",\"candidate_accepted\":" << (candidate_accepted ? "true" : "false")
      << ",\"astar_timed_out\":" << (astar_timed_out ? "true" : "false")
      << ",\"switch_reason\":\"" << route_switch_reason << '"'
      << ",\"route_decision\":\"" << route_decision << '"'
      << ",\"cloud_count\":" << cloud_count_
      << ",\"skeleton_updates\":" << skeleton_update_count_.load()
      << ",\"nodes\":" << stats.skeleton_nodes
      << ",\"ordinary_backbone_nodes\":" << stats.skeleton_nodes
      << ",\"unknown_virtual_nodes\":" << stats.virtual_semantic_nodes
      << ",\"odom_nodes\":" << (active_topo->odom_node_ ? 1 : 0)
      << ",\"local_graph_nodes\":" << local_graph_nodes
      << ",\"edges\":" << stats.edges
      << ",\"path_nodes\":" << path_nodes.size()
      << ",\"topology_path_nodes\":" << accepted_route_.topology_path.size()
      << ",\"execution_path_nodes\":" << accepted_route_.execution_path.size()
      << ",\"original_path_nodes\":" << original_path_nodes
      << ",\"bubble_shortcuts\":" << shortcut_count
      << ",\"witness_points\":" << stats.witness_points
      << ",\"semantic_risk_edges_checked\":" << stats.semantic_risk_edges_checked
      << ",\"semantic_risk_edges_rejected\":" << stats.semantic_risk_edges_rejected
      << ",\"astar_expanded_nodes\":" << astar_expanded_nodes
      << ",\"astar_edge_evaluations\":" << astar_edge_evaluations
      << ",\"astar_reverse_edges_skipped\":" << candidate_search_stats.reverse_edges_skipped
      << ",\"astar_semantic_risk_nodes\":" << candidate_search_stats.semantic_query_nodes
      << ",\"astar_inactive_frontier_semantic_nodes\":" <<
        candidate_search_stats.semantic_inactive_virtual_nodes_skipped
      << ",\"astar_semantic_frontier_candidates\":" <<
        candidate_search_stats.semantic_frontier_candidates
      << ",\"astar_ordinary_frontier_candidates\":" <<
        candidate_search_stats.ordinary_frontier_candidates
      << ",\"astar_semantic_frontier_edge_rejections\":" <<
        candidate_search_stats.semantic_frontier_edge_rejections
      << ",\"semantic_mixed_frames\":" << candidate_search_stats.semantic_mixed_frames
      << ",\"semantic_all_low_frames\":" << candidate_search_stats.semantic_all_low_frames
      << ",\"semantic_all_high_frames\":" << candidate_search_stats.semantic_all_high_frames
      << ",\"selected_semantic_frame_stamp_ns\":" <<
        candidate_search_stats.selected_semantic_frame_stamp_ns
      << ",\"selected_semantic_column\":" <<
        static_cast<int>(candidate_search_stats.selected_semantic_column)
      << ",\"selected_semantic_frame_min_risk\":" <<
        candidate_search_stats.selected_semantic_frame_min_risk
      << ",\"selected_semantic_frame_risk_range\":" <<
        candidate_search_stats.selected_semantic_frame_risk_range
      << ",\"selected_semantic_risk_regret\":" <<
        candidate_search_stats.selected_semantic_risk_regret
      << ",\"selected_semantic_cost_m\":" <<
        candidate_search_stats.selected_semantic_cost_m
      << ",\"astar_best_semantic_frontier_objective\":" <<
        candidate_search_stats.best_semantic_frontier_objective
      << ",\"astar_best_ordinary_frontier_objective\":" <<
        candidate_search_stats.best_ordinary_frontier_objective
      << ",\"astar_best_semantic_frontier_id\":" <<
        candidate_search_stats.best_semantic_frontier_id
      << ",\"astar_best_ordinary_frontier_id\":" <<
        candidate_search_stats.best_ordinary_frontier_id
      << ",\"semantic_frontier_ranking\":[";
    for (std::size_t index = 0;
         index < candidate_search_stats.semantic_frontier_ranking.size(); ++index) {
      if (index != 0) timing_json << ',';
      const auto &candidate = candidate_search_stats.semantic_frontier_ranking[index];
      timing_json << "{\"id\":" << candidate.node_id
        << ",\"frame_stamp_ns\":" << candidate.frame_stamp_ns
        << ",\"column\":" << static_cast<int>(candidate.column)
        << ",\"risk\":" << candidate.semantic_risk
        << ",\"astar_cost\":" << candidate.astar_cost
        << ",\"route_distance\":" << candidate.route_distance
        << ",\"mission_distance\":" << candidate.mission_distance
        << ",\"frame_min_risk\":" << candidate.frame_min_risk
        << ",\"frame_risk_range\":" << candidate.frame_risk_range
        << ",\"risk_regret\":" << candidate.risk_regret
        << ",\"semantic_cost_m\":" << candidate.semantic_cost_m
        << ",\"direction_cosine\":" << candidate.direction_cosine
        << ",\"backtrack_cost_m\":" << candidate.backtrack_cost_m
        << ",\"objective\":" << candidate.objective << '}';
    }
    const TopoNode::Ptr committed_frontier =
      found && !path_nodes.empty() ? path_nodes.back() : nullptr;
    timing_json << ']'
      << ",\"committed_frontier_id\":" <<
        (mission_goal_direct ? 0ULL :
          (committed_frontier ? committed_frontier->persistent_id_ : 0ULL))
      << ",\"committed_frontier_type\":\"" <<
        (mission_goal_direct ? "MISSION_GOAL" : (committed_frontier ?
          (isVirtualSemanticEndpoint(committed_frontier) ? "VIRTUAL_SEMANTIC" :
           (committed_frontier->geometry_state_ == TopoGeometryState::Verified ?
             "ORDINARY_VERIFIED" : "OTHER")) : "NONE")) << '"'
      << ",\"frontier_progress_replan\":"
      << (frontier_progress_replan ? "true" : "false")
      << ",\"frontier_replan_ratio\":" << frontier_replan_progress_ratio_
      << ",\"frontier_goal_distance_weight\":" << frontier_goal_distance_weight_
      << ",\"frontier_semantic_detour_budget_m\":" << frontier_semantic_detour_budget_m_
      << ",\"frontier_semantic_frame_budget_m\":" << frontier_semantic_frame_budget_m_
      << ",\"frontier_semantic_noise_floor\":" << frontier_semantic_noise_floor_
      << ",\"semantic_frontier_memory_ms\":" << semantic_max_age_ms_
      << ",\"semantic_risk_memory_ms\":" << semantic_risk_memory_ms_
      << ",\"semantic_risk_accumulation_alpha\":" <<
        semantic_risk_accumulation_alpha_
      << ",\"semantic_opportunity_observed\":" <<
        (semantic_opportunity_observed ? "true" : "false")
      << ",\"semantic_opportunity_persistent\":" <<
        (semantic_opportunity_persistent ? "true" : "false")
      << ",\"semantic_opportunity_cooldown_ready\":" <<
        (semantic_opportunity_cooldown_ready ? "true" : "false")
      << ",\"semantic_opportunity_waiting_for_progress\":"
      << (semantic_opportunity_waiting_for_progress ? "true" : "false")
      << ",\"semantic_opportunity_replan\":" <<
        (semantic_opportunity_replan ? "true" : "false")
      << ",\"semantic_opportunity_best_column\":" << semantic_opportunity.best_column
      << ",\"semantic_opportunity_route_column\":" << semantic_opportunity.route_column
      << ",\"semantic_opportunity_best_risk\":" << semantic_opportunity.best_risk
      << ",\"semantic_opportunity_route_risk\":" << semantic_opportunity.route_risk
      << ",\"semantic_opportunity_improvement_m\":" << semantic_opportunity.improvement_m
      << ",\"semantic_opportunity_pending_frames\":" << pending_semantic_opportunity_frames_
      << ",\"frontier_objective_scale\":" << frontier_objective_scale
      << ",\"incumbent_frontier_distance\":" << incumbent_goal_distance_log
      << ",\"candidate_frontier_distance\":" << candidate_goal_distance_log
      << ",\"incumbent_route_cost\":" << incumbent_route_cost_log
      << ",\"candidate_route_cost\":" << candidate_route_cost_log
      << ",\"incumbent_frontier_objective\":" << incumbent_objective_log
      << ",\"candidate_frontier_objective\":" << candidate_objective_log
      << ",\"mission_goal_direct\":" << (mission_goal_direct ? "true" : "false")
      << ",\"verified_prefix_reachable\":"
      << (accepted_connectivity.verified_prefix_reachable ? "true" : "false")
      << ",\"accepted_head_edge_usable\":"
      << (accepted_connectivity.accepted_head_edge_usable ? "true" : "false")
      << ",\"terminal_unknown\":"
      << (accepted_connectivity.has_terminal_unknown ? "true" : "false")
      << ",\"terminal_unknown_edge_usable\":"
      << (accepted_connectivity.terminal_unknown_edge_usable ? "true" : "false")
      << ",\"verified_nodes_visited\":" << accepted_connectivity.verified_nodes_visited
      << ",\"route_reachable\":" << (accepted_route_reachable ? "true" : "false")
      << ",\"accepted_route_stale_but_safe\":"
      << (accepted_route_stale_but_safe ? "true" : "false")
      << ",\"frontier_progress_t\":"
      << static_cast<double>(accepted_route_.frontier_goal_progress_t)
      << '}';
    publishTiming(timing_json.str());
    float path_semantic_risk_min = 0.0F;
    float path_semantic_risk_max = 0.0F;
    float path_semantic_risk_mean = 0.0F;
    std::string path_semantic_risks = "none";
    if (!stats.path_edge_semantic_risks.empty()) {
      path_semantic_risk_min = *std::min_element(
        stats.path_edge_semantic_risks.begin(), stats.path_edge_semantic_risks.end());
      path_semantic_risk_max = *std::max_element(
        stats.path_edge_semantic_risks.begin(), stats.path_edge_semantic_risks.end());
      for (const float risk : stats.path_edge_semantic_risks) {
        path_semantic_risk_mean += risk;
      }
      path_semantic_risk_mean /= static_cast<float>(stats.path_edge_semantic_risks.size());
      std::ostringstream risk_stream;
      risk_stream << std::fixed << std::setprecision(3);
      for (std::size_t i = 0; i < stats.path_edge_semantic_risks.size(); ++i) {
        if (i != 0) risk_stream << ',';
        if (i + 1 < path_nodes.size() && path_nodes[i] && path_nodes[i + 1]) {
          risk_stream << path_nodes[i]->persistent_id_ << "->" <<
            path_nodes[i + 1]->persistent_id_ << ':';
        } else {
          risk_stream << i << ':';
        }
        risk_stream << stats.path_edge_semantic_risks[i];
      }
      path_semantic_risks = risk_stream.str();
    }

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), diagnostic_log_period_ms_,
      "[ScaleNav timing][update] rebuild_running=%d odom_connect=%.3f ms "
      "astar=%.3f ms publish=%.3f ms total=%.3f ms cloud=%zu skeleton_updates=%zu "
      "bubbles=%zu nodes=%zu edges=%zu path_nodes=%zu topology_path_nodes=%zu "
      "execution_path_nodes=%zu witness_points=%zu->%zu "
      "original_path_nodes=%zu bubble_shortcuts=%zu "
      "route_memory_points=%zu "
      "persistent_semantic_records=%zu global_nodes=%zu global_edges=%zu "
      "global_semantic_nodes=%zu global_verified_semantic_nodes=%zu "
      "global_virtual_semantic_nodes=%zu local_graph_nodes=%zu local_semantic_nodes=%zu "
      "local_inactive_virtual_semantic_nodes=%zu local_semantic_radius=%.1f m "
      "astar_searches=%zu astar_expanded_nodes=%zu incumbent_expanded_nodes=%zu "
      "candidate_expanded_nodes=%zu astar_edge_evaluations=%zu "
      "astar_semantic_nodes=%zu astar_inactive_virtual_semantic_nodes=%zu "
      "astar_semantic_checks=%zu "
      "astar_candidate_frontier_goals=%zu astar_timed_out=%d "
      "geometry_source=%s "
      "remembered_edges=%zu/%zu geometric_edges=%zu route_mode=%s "
      "incumbent=%s frontier_goal_id=%llu candidate_found=%d candidate_accepted=%d "
      "switch_reason=%s route_decision=%s "
      "route_compare=%d incumbent_loss=%.2f candidate_loss=%.2f "
      "frontier_goal_distance_weight=%.2f frontier_objective_scale=%.2f "
      "semantic_detour_budget=%.1f semantic_frame_budget=%.1f semantic_noise_floor=%.3f "
      "semantic_opportunity=%d persistent=%d cooldown=%d replan=%d "
      "semantic_waiting_for_progress=%d "
      "semantic_columns=%d->%d improvement=%.2f m pending=%d "
      "incumbent_frontier_distance=%.2f candidate_frontier_distance=%.2f "
      "incumbent_route_cost=%.2f candidate_route_cost=%.2f "
      "incumbent_frontier_objective=%.3f candidate_frontier_objective=%.3f "
      "incumbent_risk=%.3f candidate_risk=%.3f "
      "incumbent_progress=%.2f candidate_progress=%.2f "
      "semantic_nodes=%zu virtual_semantic_nodes=%zu semantic_path_nodes=%zu semantic_max=%.3f "
      "path_edge_semantic_risks=%s path_edge_risk_min=%.3f path_edge_risk_mean=%.3f "
      "path_edge_risk_max=%.3f "
      "path_cost=%.2f geometry=%.2f semantic=%.2f clearance=%.2f "
      "local_graph_radius=%.1f m "
      "route_aligned=%d route_lateral_error=%.2f m "
      "horizon_ready=%d frontier_progress_replan=%d frontier_replan_ratio=%.2f "
      "route_reachable=%d accepted_head_edge_usable=%d verified_prefix_reachable=%d terminal_unknown=%d "
      "terminal_unknown_edge_usable=%d verified_nodes_visited=%zu route_head_attached=%d "
      "accepted_route_stale_but_safe=%d "
      "mission_goal_direct=%d "
      "route_length=%.2f route_remaining=%.2f "
      "frontier_initial_route_length=%.2f frontier_progress=%.2f frontier_progress_t=%.4f "
      "route_risk=%.3f "
      "frontier_goal=(%.2f,%.2f,%.2f) "
      "frontier_goal_distance=%.2f m "
      "found=%d",
      static_cast<int>(rebuild_running_.load()), odom_ms, astar_ms, publish_ms, ms,
      cloud_count_, skeleton_update_count_.load(), stats.bubbles,
      stats.skeleton_nodes, stats.edges, path_nodes.size(),
      accepted_route_.topology_path.size(), accepted_route_.execution_path.size(),
      stats.witness_points_raw,
      stats.witness_points, original_path_nodes, shortcut_count, route_memory_points,
      persistent_semantic_records,
      stats.skeleton_nodes, stats.edges, global_semantic_nodes,
      stats.semantic_nodes, stats.virtual_semantic_nodes,
      local_graph_nodes, local_semantic_nodes.size(),
      local_inactive_virtual_semantic_nodes,
      static_cast<double>(local_semantic_radius_m), astar_searches,
      astar_expanded_nodes, incumbent_search_stats.expanded_nodes,
      candidate_search_stats.expanded_nodes, astar_edge_evaluations,
      astar_semantic_nodes, astar_inactive_virtual_semantic_nodes,
      astar_semantic_checks,
      candidate_search_stats.candidate_frontier_goals,
      static_cast<int>(astar_timed_out),
      "TOPO_CENTERS",
      reused_path_edges,
      path_nodes.size() > 1 ? path_nodes.size() - 1 : 0,
      geometrically_remembered_edges,
      "SEGMENT_HOLD_REPLAN",
      incumbent_result,
      static_cast<unsigned long long>(accepted_route_.frontier_goal_id),
      static_cast<int>(candidate_found),
      static_cast<int>(candidate_accepted),
      route_switch_reason,
      route_decision,
      static_cast<int>(compared_route_metrics),
      static_cast<double>(incumbent_metrics.objective),
      static_cast<double>(candidate_metrics.objective),
      frontier_goal_distance_weight_, frontier_objective_scale,
      frontier_semantic_detour_budget_m_, frontier_semantic_frame_budget_m_,
      frontier_semantic_noise_floor_,
      static_cast<int>(semantic_opportunity_observed),
      static_cast<int>(semantic_opportunity_persistent),
      static_cast<int>(semantic_opportunity_cooldown_ready),
      static_cast<int>(semantic_opportunity_replan),
      static_cast<int>(semantic_opportunity_waiting_for_progress),
      semantic_opportunity.route_column, semantic_opportunity.best_column,
      static_cast<double>(semantic_opportunity.improvement_m),
      pending_semantic_opportunity_frames_,
      incumbent_goal_distance_log, candidate_goal_distance_log,
      incumbent_route_cost_log, candidate_route_cost_log,
      incumbent_objective_log, candidate_objective_log,
      static_cast<double>(incumbent_metrics.risk),
      static_cast<double>(candidate_metrics.risk),
      static_cast<double>(incumbent_metrics.progress),
      static_cast<double>(candidate_metrics.progress),
      stats.semantic_nodes, stats.virtual_semantic_nodes,
      stats.semantic_path_nodes, stats.semantic_max,
      path_semantic_risks.c_str(), static_cast<double>(path_semantic_risk_min),
      static_cast<double>(path_semantic_risk_mean),
      static_cast<double>(path_semantic_risk_max),
      static_cast<double>(stats.path_geometry_cost + stats.path_semantic_cost +
        stats.path_clearance_cost), static_cast<double>(stats.path_geometry_cost),
      static_cast<double>(stats.path_semantic_cost),
      static_cast<double>(stats.path_clearance_cost),
      local_graph_radius_m_,
      static_cast<int>(route_aligned), static_cast<double>(route_lateral_error),
      static_cast<int>(route_has_planning_horizon),
      static_cast<int>(frontier_progress_replan), frontier_replan_progress_ratio_,
      static_cast<int>(accepted_route_reachable),
      static_cast<int>(accepted_connectivity.accepted_head_edge_usable),
      static_cast<int>(accepted_connectivity.verified_prefix_reachable),
      static_cast<int>(accepted_connectivity.has_terminal_unknown),
      static_cast<int>(accepted_connectivity.terminal_unknown_edge_usable),
      accepted_connectivity.verified_nodes_visited,
      static_cast<int>(accepted_route_head_attached),
      static_cast<int>(accepted_route_stale_but_safe),
      static_cast<int>(mission_goal_direct),
      static_cast<double>(accepted_route_length),
      static_cast<double>(accepted_route_remaining),
      static_cast<double>(accepted_route_.frontier_goal_initial_route_length_m),
      static_cast<double>(accepted_route_.frontier_goal_progress_m),
      static_cast<double>(accepted_route_.frontier_goal_progress_t),
      static_cast<double>(current_route_risk),
      accepted_route_.frontier_goal.x(), accepted_route_.frontier_goal.y(), accepted_route_.frontier_goal.z(),
      (accepted_route_.frontier_goal - layer_goal).norm(),
      static_cast<int>(found));
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), diagnostic_log_period_ms_,
      "[ScaleNav frontier candidates] semantic=%zu ordinary=%zu rejected_risk_edges=%zu "
      "best_semantic(id=%llu,obj=%.3f) best_ordinary(id=%llu,obj=%.3f)",
      candidate_search_stats.semantic_frontier_candidates,
      candidate_search_stats.ordinary_frontier_candidates,
      candidate_search_stats.semantic_frontier_edge_rejections,
      static_cast<unsigned long long>(candidate_search_stats.best_semantic_frontier_id),
      static_cast<double>(candidate_search_stats.best_semantic_frontier_objective),
      static_cast<unsigned long long>(candidate_search_stats.best_ordinary_frontier_id),
      static_cast<double>(candidate_search_stats.best_ordinary_frontier_objective));
    // Persist a throttled, self-contained graph snapshot after the planner
    // timing window so diagnostic I/O is not reported as route publication
    // latency.
    writeGraphSnapshot(active_topo, path_nodes, found);
  }

  TopoNode::Ptr nearestPersistentNode(const TopoGraph::Ptr &topo,
                                      const Eigen::Vector3f &position,
                                      float maximum_distance,
                                      std::uint64_t persistent_id) const
  {
    TopoNode::Ptr nearest;
    float nearest_distance = maximum_distance;
    for (const auto &entry : topo->reg_map_idx2ptr_) {
      if (!entry.second) continue;
      for (const auto &node : entry.second->topo_nodes_) {
        if (!node || node->is_viewpoint_) continue;
        if (persistent_id != 0 && node->persistent_id_ == persistent_id) return node;
        const float distance = (node->center_ - position).norm();
        if (distance < nearest_distance) {
          nearest = node;
          nearest_distance = distance;
        }
      }
    }
    return nearest;
  }

  std::size_t ensureOdomConnectivity(const TopoGraph::Ptr &topo,
                                     const Eigen::Vector3f &position)
  {
    if (!topo || !topo->odom_node_ || !topo->parallel_bubble_astar_ ||
        !topo->odom_node_->neighbors_.empty()) return 0;

    Eigen::Vector3f query_position = position;
    if (graph_fixed_layer_) query_position.z() = static_cast<float>(graph_layer_z_);
    Eigen::Vector3f search_position = query_position;
    const double query_clearance = topo->lidar_map_interface_->getDisToOcc(query_position);
    const double safe_distance = topo->parallel_bubble_astar_->safe_distance_;
    const double start_clearance_threshold = safe_distance + 0.1;
    bool used_safe_anchor = false;
    if (query_clearance < start_clearance_threshold) {
      const auto free_space = topo->lidar_map_interface_->freeSpaceSnapshot();
      double best_clearance = -1.0;
      double best_distance = std::numeric_limits<double>::infinity();
      for (const auto &point : free_space.points) {
        Eigen::Vector3f candidate(point.x, point.y, point.z);
        if (graph_fixed_layer_) candidate.z() = static_cast<float>(graph_layer_z_);
        const double distance = (candidate - query_position).norm();
        if (!std::isfinite(distance) || distance > odom_fallback_radius_m_) continue;
        const double clearance = topo->lidar_map_interface_->getDisToOcc(candidate);
        if (clearance < start_clearance_threshold) continue;
        if (clearance > best_clearance + 1e-3 ||
            (std::abs(clearance - best_clearance) <= 1e-3 && distance < best_distance)) {
          best_distance = distance;
          best_clearance = clearance;
          search_position = candidate;
          used_safe_anchor = true;
        }
      }
      if (!used_safe_anchor) {
        for (const auto &point : free_space.points) {
          Eigen::Vector3f candidate(point.x, point.y, point.z);
          if (graph_fixed_layer_) candidate.z() = static_cast<float>(graph_layer_z_);
          const double distance = (candidate - query_position).norm();
          if (!std::isfinite(distance) || distance > odom_fallback_radius_m_) continue;
          const double clearance = topo->lidar_map_interface_->getDisToOcc(candidate);
          if (!std::isfinite(clearance)) continue;
          if (clearance > best_clearance + 1e-3 ||
              (std::abs(clearance - best_clearance) <= 1e-3 && distance < best_distance)) {
            best_distance = distance;
            best_clearance = clearance;
            search_position = candidate;
          }
        }
      }
    }
    topo->odom_node_->center_ = search_position;

    std::vector<TopoNode::Ptr> candidates;
    std::unordered_set<TopoNode::Ptr> unique;
    for (const auto &entry : topo->reg_map_idx2ptr_) {
      if (!entry.second) continue;
      for (const auto &node : entry.second->topo_nodes_) {
        if (!node || node->is_viewpoint_ || !unique.insert(node).second) continue;
        if ((node->center_ - search_position).norm() <= odom_fallback_radius_m_) {
          candidates.push_back(node);
        }
      }
    }
    std::sort(candidates.begin(), candidates.end(),
      [&search_position](const TopoNode::Ptr &left, const TopoNode::Ptr &right) {
        return (left->center_ - search_position).squaredNorm() <
               (right->center_ - search_position).squaredNorm();
      });

    const std::size_t soft_limit = std::min(
      candidates.size(), static_cast<std::size_t>(std::max(0, odom_fallback_candidates_)));
    std::size_t inserted = 0;
    std::size_t reach_end = 0;
    std::size_t no_path = 0;
    std::size_t start_fail = 0;
    std::size_t end_fail = 0;
    std::size_t timed_out = 0;
    std::size_t shorten_fail = 0;
    std::size_t tested = 0;
    auto try_candidate = [&](const TopoNode::Ptr &candidate) {
      ++tested;
      std::vector<Eigen::Vector3f> path;
      const int result = topo->parallel_bubble_astar_->search(
        search_position, candidate->center_, path,
        std::max(0.5, odom_connect_timeout_ms_) / 1000.0, false);
      switch (result) {
        case ParallelBubbleAstar::REACH_END: ++reach_end; break;
        case ParallelBubbleAstar::NO_PATH: ++no_path; break;
        case ParallelBubbleAstar::START_FAIL: ++start_fail; break;
        case ParallelBubbleAstar::END_FAIL: ++end_fail; break;
        case ParallelBubbleAstar::TIME_OUT: ++timed_out; break;
        default: break;
      }
      if (result != ParallelBubbleAstar::REACH_END || path.size() < 2) {
        return false;
      }
      if (!topo->parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
        ++shorten_fail;
        return false;
      }
      topo->odom_node_->neighbors_.insert(candidate);
      topo->odom_node_->paths_[candidate] = path;
      topo->odom_node_->weight_[candidate] = 0.0;
      ++inserted;
      // One connected real Bubble is sufficient to make the persistent graph
      // reachable. Keep a second connection when it is available for branching.
      return true;
    };
    for (std::size_t i = 0; i < soft_limit && inserted < 2; ++i) {
      if (try_candidate(candidates[i]) && inserted >= 2) break;
    }
    if (inserted == 0) {
      for (std::size_t i = soft_limit; i < candidates.size() && inserted < 2; ++i) {
        if (try_candidate(candidates[i]) && inserted >= 2) break;
      }
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), diagnostic_log_period_ms_,
      "[ScaleNav odom diagnosis] pos=(%.2f,%.2f,%.2f) clearance=%.2f candidates=%zu "
      "search=(%.2f,%.2f,%.2f) anchor=%d tested=%zu connected=%zu no_path=%zu start_fail=%zu end_fail=%zu timeout=%zu "
      "shorten_fail=%zu",
      query_position.x(), query_position.y(), query_position.z(),
      query_clearance, candidates.size(),
      search_position.x(), search_position.y(), search_position.z(),
      used_safe_anchor ? 1 : 0, tested, inserted, no_path, start_fail, end_fail,
      timed_out, shorten_fail);
    return inserted;
  }

  std::size_t buildRememberedEdges(
      const TopoGraph::Ptr &topo, const std::vector<Eigen::Vector3f> &route,
      float maximum_distance,
      std::unordered_set<std::pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash> &edges) const
  {
    if (route.size() < 2) return 0;
    std::unordered_set<TopoNode::Ptr> visited;
    std::size_t count = 0;
    for (const auto &entry : topo->reg_map_idx2ptr_) {
      if (!entry.second) continue;
      for (const auto &node : entry.second->topo_nodes_) {
        if (!node || !visited.insert(node).second) continue;
        for (const auto &neighbor : node->neighbors_) {
          if (!neighbor || !std::less<const TopoNode *>{}(node.get(), neighbor.get())) continue;
          if (!scalenav_graph::edgeFollowsRoute(
                node->center_, neighbor->center_, route, maximum_distance)) continue;
          edges.insert({node, neighbor});
          edges.insert({neighbor, node});
          ++count;
        }
      }
    }
    return count;
  }

  bool connectFrontierGoalToMissionGoal(const TopoGraph::Ptr &topo, const TopoNode::Ptr &frontier_goal,
                             std::vector<Eigen::Vector3f> &extension) const
  {
    extension.clear();
    if (!topo || !frontier_goal || !topo->parallel_bubble_astar_) return false;
    Eigen::Vector3f layer_goal = goal_;
    if (graph_fixed_layer_) layer_goal.z() = static_cast<float>(graph_layer_z_);
    const float distance = (frontier_goal->center_ - layer_goal).norm();
    const bool goal_in_window =
      (position_ - layer_goal).norm() <= static_cast<float>(local_graph_radius_m_);
    const float max_connect_m = goal_in_window ?
      static_cast<float>(local_graph_radius_m_) :
      static_cast<float>(goal_connect_distance_m_);
    if (distance > max_connect_m) return false;
    if (distance < 1e-3F) {
      extension = {frontier_goal->center_, layer_goal};
      return true;
    }
    const int result = topo->parallel_bubble_astar_->search(
      frontier_goal->center_, layer_goal, extension,
      std::max(1.0, goal_connect_timeout_ms_) / 1000.0, true);
    if (result != ParallelBubbleAstar::REACH_END || extension.size() < 2 ||
        !topo->parallel_bubble_astar_->collisionCheck_shortenPath(extension)) {
      extension.clear();
      return false;
    }
    if ((extension.front() - frontier_goal->center_).norm() >
        (extension.back() - frontier_goal->center_).norm()) {
      std::reverse(extension.begin(), extension.end());
    }
    if ((extension.front() - frontier_goal->center_).norm() > 0.5F ||
        (extension.back() - layer_goal).norm() > 0.5F) {
      extension.clear();
      return false;
    }
    extension.front() = frontier_goal->center_;
    extension.back() = layer_goal;
    return true;
  }

  // Remove redundant ordinary topology vertices only when the new chord is
  // covered by the safety bubbles that generated the original route.  This
  // is an execution-path transformation; the topology graph itself is left
  // unchanged so future searches still use its real edges.
  std::vector<TopoNode::Ptr> shortcutBubblePath(
    const TopoGraph::Ptr &topo, const std::vector<TopoNode::Ptr> &path,
    std::size_t &shortcut_count) const
  {
    shortcut_count = 0;
    if (!topo || !topo->parallel_bubble_astar_ || path.size() < 3) return path;

    const double safe_distance = std::max(
      0.0, topo->parallel_bubble_astar_->safe_distance_);
    const auto is_semantic = [](const TopoNode::Ptr &node) {
      return isVirtualSemanticEndpoint(node);
    };
    const auto is_ordinary = [&](const TopoNode::Ptr &node) {
      return node && !is_semantic(node) &&
        (node->role_ == TopoNodeRole::Odom ||
         node->geometry_state_ == TopoGeometryState::Verified);
    };
    const auto effective_radius = [&](const TopoNode::Ptr &node) {
      if (!node) return 0.0F;
      double radius = node->bubble_radius_;
      // The odom anchor has no generated BubbleNode. Use the live clearance
      // only as its bubble radius; the chord is still checked below.
      if ((!std::isfinite(radius) || radius <= 1e-3) &&
          topo->lidar_map_interface_) {
        radius = topo->lidar_map_interface_->getDisToOcc(node->center_);
      }
      radius -= safe_distance;
      return std::isfinite(radius) ? static_cast<float>(std::max(0.0, radius)) : 0.0F;
    };
    const auto chord_covered = [&](std::size_t first, std::size_t last) {
      if (last <= first + 1) return false;
      // A semantic endpoint may be retained, but it cannot provide a bubble
      // for the proof. Every sampled point must be covered by an ordinary
      // node bubble from this route segment.
      std::vector<std::pair<Eigen::Vector3f, float>> bubbles;
      bubbles.reserve(last - first + 1);
      for (std::size_t index = first; index <= last; ++index) {
        if (!is_ordinary(path[index])) continue;
        const float radius = effective_radius(path[index]);
        if (radius > 1e-3F) bubbles.emplace_back(path[index]->center_, radius);
      }
      if (bubbles.empty()) return false;
      const Eigen::Vector3f start = path[first]->center_;
      const Eigen::Vector3f end = path[last]->center_;
      const float length = (end - start).norm();
      if (!std::isfinite(length)) return false;
      const int samples = std::max(1, static_cast<int>(std::ceil(length / 0.25F)));
      for (int sample = 0; sample <= samples; ++sample) {
        const float t = static_cast<float>(sample) / static_cast<float>(samples);
        const Eigen::Vector3f point = start + t * (end - start);
        bool covered = false;
        for (const auto &bubble : bubbles) {
          if ((point - bubble.first).norm() <= bubble.second + 1e-3F) {
            covered = true;
            break;
          }
        }
        if (!covered) return false;
      }
      std::vector<Eigen::Vector3f> chord{start, end};
      return topo->parallel_bubble_astar_->collisionCheck_shortenPath(chord);
    };

    // Dynamic-programming path suction. Each safe chord is a transition;
    // dp[j] chooses the lowest-cost complete route from the start to node j.
    // A small waypoint penalty prefers fewer segments when their geometric
    // costs are otherwise equivalent, while the mandatory semantic nodes
    // remain in the recovered path.
    const std::size_t node_count = path.size();
    const float infinity = std::numeric_limits<float>::infinity();
    constexpr float waypoint_penalty = 0.01F;
    std::vector<float> dp(node_count, infinity);
    std::vector<std::size_t> predecessor(node_count, node_count);
    dp[0] = 0.0F;
    for (std::size_t end = 1; end < node_count; ++end) {
      for (std::size_t begin = 0; begin < end; ++begin) {
        if (!std::isfinite(dp[begin])) continue;
        bool semantic_between = false;
        for (std::size_t index = begin + 1; index < end; ++index) {
          if (is_semantic(path[index])) {
            semantic_between = true;
            break;
          }
        }
        if (semantic_between) continue;
        const bool adjacent = end == begin + 1;
        if (!adjacent && !chord_covered(begin, end)) continue;
        const float length = (path[end]->center_ - path[begin]->center_).norm();
        if (!std::isfinite(length)) continue;
        const float transition_cost = length + waypoint_penalty;
        const float candidate_cost = dp[begin] + transition_cost;
        if (candidate_cost < dp[end]) {
          dp[end] = candidate_cost;
          predecessor[end] = begin;
        }
      }
    }
    if (predecessor.back() == node_count) return path;

    std::vector<std::size_t> selected_indices;
    for (std::size_t index = node_count - 1; ; index = predecessor[index]) {
      selected_indices.push_back(index);
      if (index == 0) break;
      if (predecessor[index] == node_count) return path;
    }
    std::reverse(selected_indices.begin(), selected_indices.end());
    std::vector<TopoNode::Ptr> shortened;
    shortened.reserve(selected_indices.size());
    for (const std::size_t index : selected_indices) shortened.push_back(path[index]);
    shortcut_count = path.size() - shortened.size();
    return shortened;
  }

  std::int64_t activeVirtualSemanticStampNs()
  {
    if (last_semantic_applied_stamp_ns_ <= 0) return -1;
    const std::int64_t now_ns = get_clock()->now().nanoseconds();
    const double age_ms = static_cast<double>(std::llabs(
      now_ns - last_semantic_applied_stamp_ns_)) / 1.0e6;
    // Return a current-time reference while either semantic window is alive.
    // TopoGraph applies the 1.5 s frontier cutoff and the longer route-risk
    // cutoff independently.
    return age_ms <= semantic_risk_memory_ms_ ? now_ns : -1;
  }

  float semanticRiskAlongRoute(const TopoGraph::Ptr &topo,
                               const std::vector<Eigen::Vector3f> &route)
  {
    if (!topo || route.size() < 2) return 0.0F;
    const float influence = static_cast<float>(std::max(0.0, semantic_route_influence_m_));
    if (influence <= 0.0F) return 0.0F;
    const float sigma = std::max(0.25F, influence * 0.5F);
    float risk = 0.0F;
    const float query_radius = static_cast<float>(local_graph_radius_m_) + influence;
    const auto semantic_nodes = topo->semanticRiskNodes(
      &position_, query_radius, activeVirtualSemanticStampNs());
    for (const auto &node : semantic_nodes) {
      const float confidence_score = std::clamp(
        node->semantic_score_ * node->semantic_confidence_, 0.0F, 1.0F);
      if (confidence_score <= 1e-3F) continue;
      const float distance = scalenav_graph::pointPathDistance(node->center_, route);
      if (!std::isfinite(distance) || distance > influence) continue;
      risk = std::max(risk, confidence_score * std::exp(
        -0.5F * (distance * distance) / (sigma * sigma)));
    }
    return std::clamp(risk, 0.0F, 1.0F);
  }

  bool updateTopoSemanticMemory(const TopoGraph::Ptr &topo)
  {
    if (!topo) return false;
    std::optional<SemanticFrame> frame;
    {
      std::lock_guard<std::mutex> lock(semantic_mutex_);
      if (!semantic_frame_ ||
          (semantic_applied_topo_ == topo &&
           semantic_frame_->stamp_ns == last_semantic_applied_stamp_ns_)) {
        return false;
      }
      const double age_ms = static_cast<double>(std::llabs(
        get_clock()->now().nanoseconds() - semantic_frame_->stamp_ns)) / 1.0e6;
      if (age_ms > semantic_max_age_ms_) return false;
      frame = semantic_frame_;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), diagnostic_log_period_ms_,
      "[ScaleNav semantic apply] enabled=%d stamp=%lld topo=%p applied_topo=%p "
      "points=%zu scores=%zu confidences=%zu virtual_flags=%zu",
      static_cast<int>(semantic_points_enabled_),
      static_cast<long long>(frame->stamp_ns), static_cast<void *>(topo.get()),
      static_cast<void *>(semantic_applied_topo_.get()), frame->points_world.size(),
      frame->scores.size(), frame->confidences.size(), frame->is_virtual.size());

    std::size_t semantic_nodes_updated = 0;
    std::size_t semantic_candidates = 0;
    std::size_t semantic_virtual_candidates = 0;
    std::size_t semantic_measured_candidates = 0;
    std::size_t semantic_height_rejected = 0;
    std::size_t semantic_distance_rejected = 0;
    std::size_t semantic_box_rejected = 0;
    std::size_t semantic_duplicate_rejected = 0;
    std::size_t semantic_connected_nodes = 0;
    float semantic_min_range_m = std::numeric_limits<float>::infinity();
    float semantic_max_range_m = 0.0F;
    if (semantic_points_enabled_ && frame->points_world.size() == frame->scores.size() &&
        frame->points_world.size() == frame->confidences.size() &&
        frame->points_world.size() == frame->is_virtual.size() &&
        frame->points_world.size() == frame->columns.size()) {
      std::vector<Eigen::Vector3f> semantic_centers;
      std::vector<float> semantic_scores;
      std::vector<float> semantic_confidences;
      std::vector<std::uint8_t> semantic_virtual_flags;
      std::vector<std::int8_t> semantic_columns;
      const Eigen::Vector3f origin = frame->origin;
      semantic_centers.reserve(frame->points_world.size());
      semantic_scores.reserve(frame->points_world.size());
      semantic_confidences.reserve(frame->points_world.size());
      semantic_virtual_flags.reserve(frame->points_world.size());
      semantic_columns.reserve(frame->points_world.size());
      for (std::size_t i = 0; i < frame->points_world.size(); ++i) {
          const Eigen::Vector3f chosen = frame->points_world[i];
          const float score = frame->scores[i];
          const float confidence = frame->confidences[i];
          const bool is_virtual = frame->is_virtual[i] != 0U;
          const std::int8_t column = frame->columns[i];
          const float distance = (chosen - origin).norm();
          if (!std::isfinite(distance) || distance < 1.0F) {
            ++semantic_distance_rejected;
            continue;
          }
          // PEARL rays are image-space evidence. In fixed-layer mode their
          // vertical component must not reject a useful far-field anchor.
          const Eigen::Vector3f planning_point = projectPlanningPoint(
            chosen, graph_fixed_layer_, graph_layer_z_);
          const bool in_known_box = topo->lidar_map_interface_->IsInBox(planning_point);
          const bool in_global_map = topo->lidar_map_interface_->IsInMap(planning_point);
          // Virtual anchors intentionally extend beyond currently observed
          // boxes; measured surface projections remain box-gated.
          if ((!is_virtual && !in_known_box) || (is_virtual && !in_global_map)) {
            ++semantic_box_rejected;
            continue;
          }
          bool duplicate = false;
          for (std::size_t existing_index = 0;
               existing_index < semantic_centers.size(); ++existing_index) {
            // A measured surface annotation and its fixed-depth counterpart
            // are intentionally distinct products of the same heatmap patch.
            if ((semantic_virtual_flags[existing_index] != 0U) != is_virtual) continue;
            if ((semantic_centers[existing_index] - planning_point).norm() <
                static_cast<float>(std::max(1.0, semantic_point_separation_m_))) {
              duplicate = true;
              break;
            }
          }
          if (duplicate) {
            ++semantic_duplicate_rejected;
            continue;
          }
          semantic_centers.push_back(planning_point);
          semantic_scores.push_back(std::clamp(score, 0.0F, 1.0F));
          semantic_confidences.push_back(std::clamp(confidence, 0.0F, 1.0F));
          semantic_virtual_flags.push_back(is_virtual ? 1U : 0U);
          semantic_columns.push_back(column);
          if (is_virtual) {
            ++semantic_virtual_candidates;
          } else {
            ++semantic_measured_candidates;
          }
          const float range = (planning_point - origin).norm();
          semantic_min_range_m = std::min(semantic_min_range_m, range);
          semantic_max_range_m = std::max(semantic_max_range_m, range);
      }
      semantic_nodes_updated = topo->insertSemanticNodes(
        semantic_centers, semantic_scores,
        static_cast<float>(std::max(0.45, semantic_point_radius_m_)),
        origin, frame->stamp_ns, semantic_confidences, semantic_virtual_flags,
        semantic_columns);
      semantic_candidates = semantic_centers.size();
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), diagnostic_log_period_ms_,
        "[ScaleNav semantic insert] frame_points=%zu virtual_candidates=%zu measured_candidates=%zu "
        "candidates=%zu (virtual=%zu measured=%zu) rejected_distance=%zu "
        "rejected_box=%zu rejected_duplicate=%zu inserted_or_updated=%zu",
        frame->points_world.size(), semantic_virtual_candidates,
        semantic_measured_candidates,
        semantic_candidates, semantic_virtual_candidates, semantic_measured_candidates,
        semantic_distance_rejected, semantic_box_rejected, semantic_duplicate_rejected,
        semantic_nodes_updated);
      std::unordered_set<TopoNode::Ptr> counted_semantic_nodes;
      for (const auto &entry : topo->reg_map_idx2ptr_) {
        if (!entry.second) continue;
        for (const auto &node : entry.second->topo_nodes_) {
          if (!node || !counted_semantic_nodes.insert(node).second ||
              node->geometry_state_ != TopoGeometryState::Unknown ||
              node->semantic_stamp_ns_ != frame->stamp_ns ||
              node->semantic_observations_ == 0) continue;
          const bool connected_to_backbone = std::any_of(
            node->neighbors_.begin(), node->neighbors_.end(),
            [](const TopoNode::Ptr &neighbor) {
              return neighbor &&
                (neighbor->role_ == TopoNodeRole::Odom ||
                 neighbor->geometry_state_ == TopoGeometryState::Verified);
            });
          if (connected_to_backbone) ++semantic_connected_nodes;
        }
      }
      if (virtual_semantic_prune_enabled_) {
        std::unordered_set<std::uint64_t> protected_ids;
        Eigen::Vector3f forward = goal_ - position_;
        if (graph_fixed_layer_) forward.z() = 0.0F;
        const auto prune = topo->pruneVirtualSemanticNodes(
          position_, forward,
          static_cast<float>(std::max(0.0, virtual_semantic_backtrack_margin_m_)),
          static_cast<std::size_t>(std::max(1, virtual_semantic_max_nodes_)),
          frame->stamp_ns, protected_ids);
        eraseSemanticMemory(prune.removed_ids);
        if (!prune.removed_ids.empty()) {
          RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), diagnostic_log_period_ms_,
            "[ScaleNav semantic prune] before=%zu removed_behind=%zu "
            "removed_capacity=%zu after=%zu backtrack_margin=%.1f m max_nodes=%d",
            prune.before, prune.removed_behind, prune.removed_capacity, prune.after,
            virtual_semantic_backtrack_margin_m_, virtual_semantic_max_nodes_);
        }
      }
    }
    mergeSemanticMemory(topo->semanticMemorySnapshot());
    // Unknown fixed-depth endpoints are transient planning evidence. Mark the
    // newly applied frame active; older virtual nodes remain persisted but no
    // longer contribute to route cost.
    last_semantic_applied_stamp_ns_ = frame->stamp_ns;
    if (semantic_nodes_updated > 0 || semantic_height_rejected > 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), diagnostic_log_period_ms_,
        "[ScaleNav semantic graph] virtual_depth=%.2f m candidates=%zu (virtual=%zu measured=%zu) "
        "height_rejected=%zu "
        "inserted_or_updated=%zu connected=%zu range=%.2f..%.2f m "
        "mode=REPULSION",
        semantic_virtual_depth_m_, semantic_candidates,
        semantic_virtual_candidates, semantic_measured_candidates,
        semantic_height_rejected,
        semantic_nodes_updated,
        semantic_connected_nodes,
        std::isfinite(semantic_min_range_m) ? semantic_min_range_m : 0.0F,
        semantic_max_range_m);
    }
    // A detached skeleton swap creates a new persistent graph. Reapply the
    // latest frame once to that graph even when its timestamp was already
    // consumed by the previous graph.
    semantic_applied_topo_ = topo;
    return true;
  }

  static std_msgs::msg::ColorRGBA semanticColor(float normalized, bool enabled)
  {
    std_msgs::msg::ColorRGBA color;
    if (!enabled) {
      setColor(color, kTopology);
      return color;
    }
    const float t = std::clamp(normalized, 0.0F, 1.0F);
    constexpr Rgb anchors[3] = {kRiskLow, kRiskMedium, kRiskHigh};
    const float scaled = t * 2.0F;
    const int index = std::min(1, static_cast<int>(scaled));
    const float local = scaled - static_cast<float>(index);
    color.r = anchors[index].r * (1.0F - local) + anchors[index + 1].r * local;
    color.g = anchors[index].g * (1.0F - local) + anchors[index + 1].g * local;
    color.b = anchors[index].b * (1.0F - local) + anchors[index + 1].b * local;
    color.a = 1.0F;
    return color;
  }

  struct PublishStats {
    std::size_t bubbles = 0;
    std::size_t skeleton_nodes = 0;
    std::size_t edges = 0;
    std::size_t witness_points_raw = 0;
    std::size_t witness_points = 0;
    std::size_t semantic_nodes = 0;
    std::size_t virtual_semantic_nodes = 0;
    std::size_t semantic_path_nodes = 0;
    std::size_t semantic_risk_edges_checked = 0;
    std::size_t semantic_risk_edges_rejected = 0;
    float semantic_max = 0.0F;
    float path_geometry_cost = 0.0F;
    float path_semantic_cost = 0.0F;
    float path_clearance_cost = 0.0F;
    // One entry per selected topology edge, in path_nodes order.  Keeping
    // this on the publish result ties the diagnostic to the exact candidate
    // witness that was checked and potentially committed.
    std::vector<float> path_edge_semantic_risks;
    bool witness_collision_free = true;
    std::vector<Eigen::Vector3f> witness_path;
    std::vector<Eigen::Vector3f> polynomial_guide;
    scalenav_graph::WitnessParametricCurve polynomial_curve;
  };

  bool selectNextGoal(const std::vector<Eigen::Vector3f> &path, bool found,
                     float lookahead_m, Eigen::Vector3f &next_goal,
                     float minimum_progress_t = 0.0F,
                     const scalenav_graph::WitnessParametricCurve *curve = nullptr) const
  {
    Eigen::Vector3f layer_goal = goal_;
    if (graph_fixed_layer_) layer_goal.z() = static_cast<float>(graph_layer_z_);

    // Once the mission goal enters the local YOPO horizon, stop extending the
    // global frontier and keep publishing the actual endpoint.
    if (have_goal_ && mission_direct_goal_latched_) {
      next_goal = layer_goal;
      return next_goal.allFinite();
    }

    if (!found || path.size() < 2) return false;
    // When a cached replan curve is active, planar validation already happened
    // at fit time on the snapped guide path.
    if (graph_fixed_layer_ && curve == nullptr) {
      const bool path_is_planar = std::all_of(
        path.begin(), path.end(), [this](const Eigen::Vector3f &point) {
          return std::abs(point.z() - static_cast<float>(graph_layer_z_)) < 1e-3F;
        });
      if (!path_is_planar) return false;
    }
    if (have_goal_ &&
        (position_ - layer_goal).norm() <=
          static_cast<float>(goal_connect_distance_m_)) {
      next_goal = layer_goal;
      return next_goal.allFinite();
    }
    const bool lookahead_ok = curve != nullptr ?
      scalenav_graph::routeLookaheadPointFromCurve(
        *curve, position_, minimum_progress_t, lookahead_m, next_goal) :
      scalenav_graph::routeLookaheadPointFromT(
        path, position_, minimum_progress_t, lookahead_m, next_goal);
    if (!lookahead_ok) {
      return false;
    }
    if ((next_goal - position_).norm() < local_goal_min_advance_m_) {
      if (curve != nullptr && curve->valid) {
        next_goal = curve->evaluate(1.0F);
      } else if (!scalenav_graph::routePolynomialPointAtT(path, 1.0F, next_goal)) {
        next_goal = path.back();
      }
    }
    if (graph_fixed_layer_) next_goal.z() = static_cast<float>(graph_layer_z_);
    return next_goal.allFinite();
  }

  void refitPolynomialGuideFromNodes(const std::vector<Eigen::Vector3f> &nodes)
  {
    if (nodes.size() < 2) {
      polynomial_guide_path_.clear();
      polynomial_curve_ = scalenav_graph::WitnessParametricCurve();
      polynomial_curve_valid_ = false;
      return;
    }
    const float layer_z = graph_fixed_layer_ ? static_cast<float>(graph_layer_z_) :
      std::numeric_limits<float>::quiet_NaN();
    const auto fitting_nodes = densifyPolynomialInputs(nodes, 4.0F);
    polynomial_guide_path_ = scalenav_graph::buildPolynomialGuidePath(
      fitting_nodes, position_, world_velocity_, layer_z);
    if (polynomial_guide_path_.size() < 2) {
      polynomial_guide_path_.clear();
      polynomial_curve_ = scalenav_graph::WitnessParametricCurve();
      polynomial_curve_valid_ = false;
      return;
    }
    Eigen::Vector3f initial_velocity = world_velocity_;
    if (graph_fixed_layer_) initial_velocity.z() = 0.0F;
    polynomial_curve_ = scalenav_graph::WitnessParametricCurve::fitWithInitialVelocity(
      polynomial_guide_path_, initial_velocity);
    polynomial_curve_valid_ = polynomial_curve_.valid;
  }

  PublishStats publish(const TopoGraph::Ptr &topo,
                       const std::vector<TopoNode::Ptr> &path_nodes,
                       bool found, float effective_lookahead_m,
                       float minimum_route_progress_t = 0.0F,
                       bool replan_polynomial = false)
  {
    PublishStats stats;
    visualization_msgs::msg::MarkerArray graph;
    visualization_msgs::msg::Marker skeleton_nodes;
    skeleton_nodes.header.frame_id = visualization_frame_;
    skeleton_nodes.header.stamp = now();
    skeleton_nodes.ns = "scalenav_skeleton_nodes";
    skeleton_nodes.id = 0;
    skeleton_nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    skeleton_nodes.action = visualization_msgs::msg::Marker::ADD;
    skeleton_nodes.scale.x = 0.32;
    skeleton_nodes.scale.y = 0.32;
    skeleton_nodes.scale.z = 0.32;
    setColor(skeleton_nodes.color, kTopology);

    visualization_msgs::msg::Marker semantic_nodes_marker;
    semantic_nodes_marker.header = skeleton_nodes.header;
    semantic_nodes_marker.ns = "scalenav_semantic_points";
    semantic_nodes_marker.id = 0;
    semantic_nodes_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    semantic_nodes_marker.action = visualization_msgs::msg::Marker::ADD;
    semantic_nodes_marker.scale.x = 0.55;
    semantic_nodes_marker.scale.y = 0.55;
    semantic_nodes_marker.scale.z = 0.55;
    setColor(semantic_nodes_marker.color, kCandidate);
    visualization_msgs::msg::MarkerArray semantic_labels;

    std::vector<float> semantic_scores;
    std::vector<bool> semantic_associated;
    std::vector<TopoNode::Ptr> semantic_label_candidates;
    std::size_t lowest_semantic_marker = std::numeric_limits<std::size_t>::max();
    float lowest_semantic_risk = std::numeric_limits<float>::infinity();

    visualization_msgs::msg::Marker edges_marker = skeleton_nodes;
    edges_marker.ns = "scalenav_skeleton_edges";
    edges_marker.id = 2;
    edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges_marker.scale.x = 0.045;
    setColor(edges_marker.color, kTopology, 0.72F);
    edges_marker.points.clear();
    edges_marker.colors.clear();

    visualization_msgs::msg::Marker semantic_links = edges_marker;
    semantic_links.ns = "scalenav_semantic_links";
    semantic_links.id = 4;
    semantic_links.scale.x = 0.065;
    setColor(semantic_links.color, kCandidate, 0.80F);
    semantic_links.points.clear();
    semantic_links.colors.clear();

    std::unordered_set<TopoNode::Ptr> visited;
    const std::int64_t active_semantic_stamp_ns = activeVirtualSemanticStampNs();
    const std::int64_t latest_semantic_frame_stamp_ns = last_semantic_applied_stamp_ns_;
    for (const auto &entry : topo->reg_map_idx2ptr_) {
      if (!entry.second) continue;
      for (const auto &node : entry.second->topo_nodes_) {
        if (!node || node->is_viewpoint_ || !visited.insert(node).second) continue;
        if (node->role_ == TopoNodeRole::Odom) continue;
        const bool associated = node->semantic_observations_ > 0 &&
          std::isfinite(node->semantic_score_);
        const bool virtual_semantic_point = isVirtualSemanticEndpoint(node);
        if (virtual_semantic_point) {
          semantic_nodes_marker.points.push_back(toPoint(node->center_));
          const bool risk_anchor = isSemanticRiskAnchor(
            node->semantic_score_, node->semantic_confidence_,
            static_cast<float>(semantic_point_min_score_));
          const Rgb marker_rgb = risk_anchor ? kRiskHigh : kCandidate;
          std_msgs::msg::ColorRGBA marker_color;
          setColor(marker_color, marker_rgb);
          semantic_nodes_marker.colors.push_back(marker_color);
          // Match frontier ranking: confidence determines whether an
          // observation is usable, but does not rescale its patch mean.
          const float semantic_risk = std::clamp(node->semantic_score_, 0.0F, 1.0F);
          // Highlight the minimum-risk candidate from the latest heatmap
          // frame, rather than the minimum over the retained memory window.
          // This makes the per-frame choice visible in RViz even while older
          // virtual points remain available for short-term planning.
          if (semanticNodeActiveForPlanning(*node, active_semantic_stamp_ns) &&
              node->semantic_frame_stamp_ns_ == latest_semantic_frame_stamp_ns &&
              semantic_risk < lowest_semantic_risk) {
            lowest_semantic_risk = semantic_risk;
            lowest_semantic_marker = semantic_nodes_marker.colors.size() - 1;
          }
          semantic_label_candidates.push_back(node);
          ++stats.virtual_semantic_nodes;
          stats.semantic_max = std::max(stats.semantic_max, node->semantic_score_);
          if (semanticNodeActiveForPlanning(*node, active_semantic_stamp_ns)) {
            for (const auto &neighbor : node->neighbors_) {
              if (!neighbor ||
                  (neighbor->role_ != TopoNodeRole::Odom &&
                   neighbor->geometry_state_ != TopoGeometryState::Verified)) continue;
              semantic_links.points.push_back(toPoint(node->center_));
              semantic_links.points.push_back(toPoint(neighbor->center_));
            }
          }
          continue;
        }
        skeleton_nodes.points.push_back(toPoint(node->center_));
        semantic_scores.push_back(node->semantic_score_);
        semantic_associated.push_back(associated);
        ++stats.skeleton_nodes;
        if (associated) {
          ++stats.semantic_nodes;
          stats.semantic_max = std::max(stats.semantic_max, node->semantic_score_);
        }
        for (const auto &neighbor : node->neighbors_) {
          if (!neighbor || neighbor->geometry_state_ != TopoGeometryState::Verified) continue;
          if (!std::less<const TopoNode *>{}(node.get(), neighbor.get())) continue;
          stats.edges++;
          edges_marker.points.push_back(toPoint(node->center_));
          edges_marker.points.push_back(toPoint(neighbor->center_));
        }
      }
    }
    if (lowest_semantic_marker < semantic_nodes_marker.colors.size()) {
      setColor(semantic_nodes_marker.colors[lowest_semantic_marker], kSemanticBest);
    }
    std::sort(semantic_label_candidates.begin(), semantic_label_candidates.end(),
      [](const TopoNode::Ptr &left, const TopoNode::Ptr &right) {
        const float left_risk = left ? left->semantic_score_ * left->semantic_confidence_ : 0.0F;
        const float right_risk = right ? right->semantic_score_ * right->semantic_confidence_ : 0.0F;
        return left_risk > right_risk;
      });
    const std::size_t semantic_label_count = std::min<std::size_t>(
      semantic_label_candidates.size(),
      static_cast<std::size_t>(std::max(0, semantic_label_max_nodes_)));
    for (std::size_t index = 0; index < semantic_label_count; ++index) {
      const auto &node = semantic_label_candidates[index];
      visualization_msgs::msg::Marker label;
      label.header = skeleton_nodes.header;
      label.ns = "scalenav_semantic_point_labels";
      label.id = static_cast<int>(index);
      label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      label.action = visualization_msgs::msg::Marker::ADD;
      const Eigen::Vector3f label_position =
        node->center_ + Eigen::Vector3f(0.0F, 0.0F, 0.45F);
      label.pose.position = toPoint(label_position);
      label.pose.orientation.w = 1.0;
      label.scale.z = 0.42;
      const bool risk_anchor = isSemanticRiskAnchor(
        node->semantic_score_, node->semantic_confidence_,
        static_cast<float>(semantic_point_min_score_));
      setColor(label.color, risk_anchor ? kRiskHigh : kCandidate);
      label.text = risk_anchor ? "SEM-RISK" : "SEM-UNKNOWN";
      semantic_labels.markers.push_back(std::move(label));
    }
    for (std::size_t index = semantic_label_count;
         index < previous_semantic_label_count_; ++index) {
      visualization_msgs::msg::Marker stale_label;
      stale_label.header = skeleton_nodes.header;
      stale_label.ns = "scalenav_semantic_point_labels";
      stale_label.id = static_cast<int>(index);
      stale_label.action = visualization_msgs::msg::Marker::DELETE;
      semantic_labels.markers.push_back(std::move(stale_label));
    }
    previous_semantic_label_count_ = semantic_label_count;
    for (std::size_t i = 0; i < semantic_scores.size(); ++i) {
      const float normalized = semantic_scores[i] / static_cast<float>(
        std::max(semantic_visualization_max_score_, 1e-5));
      skeleton_nodes.colors.push_back(semanticColor(normalized, semantic_associated[i]));
    }
    graph.markers.push_back(skeleton_nodes);
    graph.markers.push_back(semantic_nodes_marker);
    for (auto &label : semantic_labels.markers) graph.markers.push_back(std::move(label));
    graph.markers.push_back(edges_marker);
    graph.markers.push_back(semantic_links);

    visualization_msgs::msg::Marker path_marker = edges_marker;
    path_marker.ns = "scalenav_astar_topology_path";
    path_marker.id = 4;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.points.clear();
    path_marker.colors.clear();
    path_marker.scale.x = 0.10;
    setColor(path_marker.color, kSelectedPath, found ? 0.55F : 0.20F);
    for (const auto &node : path_nodes) {
      if (!node) continue;
      path_marker.points.push_back(toPoint(node->center_));
      if (node->semantic_observations_ > 0) ++stats.semantic_path_nodes;
    }
    const auto local_semantic_nodes = topo->semanticRiskNodes(
      &position_, static_cast<float>(local_graph_radius_m_ +
        std::max(0.0, semantic_route_influence_m_)),
      activeVirtualSemanticStampNs());
    stats.path_edge_semantic_risks.reserve(path_nodes.size() > 1 ? path_nodes.size() - 1 : 0);
    for (std::size_t i = 1; i < path_nodes.size(); ++i) {
      const auto &from = path_nodes[i - 1];
      const auto &to = path_nodes[i];
      if (!from || !to) continue;
      const float edge_length = (to->center_ - from->center_).norm();
      const auto weight_it = from->weight_.find(to);
      const float geometry_cost = weight_it != from->weight_.end() &&
        std::isfinite(weight_it->second) ? weight_it->second : edge_length;
      stats.path_geometry_cost += static_cast<float>(goal_path_cost_weight_) *
        geometry_cost;
      const float risk = std::clamp(
        topo->semanticRiskForEdge(from, to, &local_semantic_nodes), 0.0F, 1.0F);
      stats.path_edge_semantic_risks.push_back(risk);
      stats.path_semantic_cost += static_cast<float>(semantic_cost_weight_) * edge_length *
        (-std::log(std::max(1e-3F, 1.0F - risk)));
      stats.path_clearance_cost += topo->clearanceCostForEdge(from, to);
    }
    std::vector<Eigen::Vector3f> selected_node_path;
    bool witness_rejected = false;
    // The route consumed by local-goal selection and polynomial fitting is
    // exactly the A* node-center sequence. Edge witnesses remain internal
    // evidence used while constructing the topology, but are not propagated
    // as a downstream trajectory.
    for (const auto &node : path_nodes) {
      if (!node) continue;
      const auto &point = node->center_;
      if (selected_node_path.empty() ||
          (selected_node_path.back() - point).norm() > 1e-3F) {
        selected_node_path.push_back(point);
      }
    }
    if (graph_fixed_layer_) {
      const float layer_z = static_cast<float>(graph_layer_z_);
      for (auto &point : selected_node_path) point.z() = layer_z;
    }
    stats.witness_points_raw = selected_node_path.size();
    // Do not append a frontier-to-mission continuous extension; trajectory
    // optimization must remain node-based.
    stats.witness_points = selected_node_path.size();
    if (found && path_nodes.size() >= 2 && topo->parallel_bubble_astar_) {
      // A* admits every active Unknown semantic frontier.  The live edge
      // safety pass must use the same admission rule; applying the stronger
      // score/confidence risk-anchor threshold here could leave an executed
      // ordinary-to-semantic edge unchecked (semantic_path_nodes > 0 while
      // semantic_risk_edges_checked == 0).
      for (std::size_t i = 1; i < path_nodes.size(); ++i) {
        const auto &from = path_nodes[i - 1];
        const auto &to = path_nodes[i];
        const bool semantic_risk_edge = isOrdinarySemanticLink(from, to);
        if (!semantic_risk_edge) continue;
        ++stats.semantic_risk_edges_checked;

        RCLCPP_DEBUG_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[ScaleNav semantic risk edge] checking edge=%zu/%zu from=(%.2f,%.2f,%.2f) "
          "to=(%.2f,%.2f,%.2f) score_from=%.3f score_to=%.3f confidence_from=%.3f "
          "confidence_to=%.3f",
          i, path_nodes.size() - 1, from->center_.x(), from->center_.y(),
          from->center_.z(), to->center_.x(), to->center_.y(), to->center_.z(),
          from->semantic_score_, to->semantic_score_, from->semantic_confidence_,
          to->semantic_confidence_);

        // Ordinary-to-semantic links are represented by the direct segment
        // between node centers.  A curved stored witness must not make a
        // displayed line through an obstacle appear executable.
        std::vector<Eigen::Vector3f> checked_edge{from->center_, to->center_};
        ParallelBubbleAstar::CollisionCheckInfo witness_info;
        if (topo->parallel_bubble_astar_->collisionCheck_shortenPath(
              checked_edge, &witness_info)) {
          continue;
        }
        stats.witness_collision_free = false;
        ++stats.semantic_risk_edges_rejected;
        const char *reason = witness_info.reason ==
            ParallelBubbleAstar::CollisionCheckInfo::CLEARANCE ? "CLEARANCE" :
          witness_info.reason == ParallelBubbleAstar::CollisionCheckInfo::BUBBLE_OVERLAP ?
            "BUBBLE_OVERLAP" : "INVALID_PATH";
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[ScaleNav semantic risk edge rejected] reason=%s edge=%zu/%zu "
          "witness_index=%zu/2 point=(%.2f,%.2f,%.2f) "
          "clearance=%.3f radius=%.3f predecessor=%zu distance=%.3f predecessor_radius=%.3f",
          reason, i, path_nodes.size() - 1, witness_info.failed_index,
          witness_info.failed_point.x(), witness_info.failed_point.y(),
          witness_info.failed_point.z(), witness_info.clearance, witness_info.radius,
          witness_info.predecessor_index, witness_info.predecessor_distance,
          witness_info.predecessor_radius);
        if (topo->removeEdge(from, to)) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[ScaleNav semantic risk edge] detached after live safety failure");
        }
        selected_node_path.clear();
        stats.witness_points = 0;
        witness_rejected = true;
        found = false;
        // The previously accepted route is no longer executable.  Do not
        // fall back to it when this tick's replacement search fails.
        accepted_route_.valid = false;
        accepted_route_.topology_path.clear();
        accepted_route_.execution_path.clear();
        accepted_route_.witness_path.clear();
        accepted_route_.frontier_goal_id = 0;
        break;
      }
    }
    // Add the topology path only after witness validation so a rejected route
    // is visibly diagnostic rather than styled like an executable route.
    setColor(path_marker.color, kSelectedPath, found ? 0.55F : 0.20F);
    graph.markers.push_back(path_marker);
    if (found && selected_node_path.size() >= 2) {
      // Return the A* node-center route to update(); it becomes accepted state
      // only after this function has completed all publication checks.
      stats.witness_path = selected_node_path;
    }

    // Visualize the same cubic (or lower-order) fit used for monotonic
    // progress and lookahead sampling. Keep it separate from the A* node path
    // so RViz makes fit error and endpoint overshoot visible.
    visualization_msgs::msg::Marker polynomial_witness = path_marker;
    polynomial_witness.ns = "scalenav_polynomial_witness_path";
    polynomial_witness.id = 13;
    polynomial_witness.type = visualization_msgs::msg::Marker::LINE_STRIP;
    polynomial_witness.scale.x = 0.075;
    polynomial_witness.points.clear();
    setColor(polynomial_witness.color, kPolynomialPath, found ? 0.95F : 0.25F);
    // Fit once on route replan; reuse ticks only advance latched progress_t.
    if (replan_polynomial && found && path_nodes.size() >= 2) {
      refitPolynomialGuideFromNodes(selected_node_path);
      stats.polynomial_guide = polynomial_guide_path_;
      stats.polynomial_curve = polynomial_curve_;
    }
    const auto *active_curve = polynomial_curve_valid_ ? &polynomial_curve_ : nullptr;
    const std::vector<Eigen::Vector3f> &local_guide_path =
      polynomial_guide_path_.empty() ? selected_node_path : polynomial_guide_path_;
    const float local_guide_progress_t = minimum_route_progress_t;
    if (active_curve != nullptr) {
        constexpr int polynomial_samples = 96;
        polynomial_witness.points.reserve(polynomial_samples + 1);
        for (int sample = 0; sample <= polynomial_samples; ++sample) {
          const float t = static_cast<float>(sample) /
            static_cast<float>(polynomial_samples);
          Eigen::Vector3f point = active_curve->evaluate(t);
          if (graph_fixed_layer_) point.z() = static_cast<float>(graph_layer_z_);
          if (point.allFinite()) polynomial_witness.points.push_back(toPoint(point));
        }
    }
    polynomial_witness.action = polynomial_witness.points.size() >= 2 ?
      visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    graph.markers.push_back(polynomial_witness);

    Eigen::Vector3f computed_next_goal = position_;
    // Subgoal follows monotonic witness time only; YOPO handles lateral avoidance.
    const bool computed_has_next_goal = selectNextGoal(
      local_guide_path, found, effective_lookahead_m, computed_next_goal,
      local_guide_progress_t, active_curve);
    if (found && !selected_node_path.empty() && !computed_has_next_goal) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "ScaleNav found a topology route but rejected its local goal; "
        "the witness path is not a valid fixed-height path");
    }
    const Eigen::Vector3f local_goal_offset = previous_local_goal_ - position_;
    const Eigen::Vector3f forward_reference = world_velocity_.norm() > 0.5F ?
      world_velocity_ : goal_ - position_;
    const bool local_goal_is_ahead = forward_reference.norm() <= 1e-3F ||
      local_goal_offset.dot(forward_reference) > 0.0F;
    const bool hold_local_goal = !computed_has_next_goal && have_previous_local_goal_ &&
      !witness_rejected &&
      std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - previous_local_goal_time_).count() <=
        local_goal_hold_timeout_ms_ &&
      local_goal_offset.norm() >= local_goal_min_advance_m_ && local_goal_is_ahead;
    const bool has_local_goal = computed_has_next_goal || hold_local_goal;
    const Eigen::Vector3f local_goal = hold_local_goal ? previous_local_goal_ : computed_next_goal;
    visualization_msgs::msg::Marker next_goal_marker = skeleton_nodes;
    next_goal_marker.ns = "scalenav_local_goal";
    next_goal_marker.id = 6;
    next_goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    next_goal_marker.points.clear();
    next_goal_marker.colors.clear();
    next_goal_marker.scale.x = 0.48;
    next_goal_marker.scale.y = 0.48;
    next_goal_marker.scale.z = 0.48;
    setColor(next_goal_marker.color, kLocalGoal);
    next_goal_marker.action = has_local_goal ? visualization_msgs::msg::Marker::ADD :
      visualization_msgs::msg::Marker::DELETE;
    if (has_local_goal) {
      previous_local_goal_ = local_goal;
      have_previous_local_goal_ = true;
      if (!hold_local_goal) previous_local_goal_time_ = std::chrono::steady_clock::now();
      next_goal_marker.pose.position = toPoint(local_goal);
      next_goal_marker.pose.orientation.w = 1.0;
      geometry_msgs::msg::PoseStamped next_goal_message;
      next_goal_message.header.stamp = now();
      next_goal_message.header.frame_id = next_goal_frame_;
      next_goal_message.pose.position = toPoint(local_goal);
      next_goal_message.pose.orientation.w = 1.0;
      next_goal_pub_->publish(next_goal_message);
      const float local_goal_to_frontier_goal =
        (local_goal - accepted_route_.frontier_goal).norm();
      const Eigen::Vector3f topology_frontier_goal = projectPlanningPoint(
        found && !path_nodes.empty() ? path_nodes.back()->center_ :
        accepted_route_.frontier_goal,
        graph_fixed_layer_, graph_layer_z_);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), diagnostic_log_period_ms_,
        "[ScaleNav goals] vehicle=(%.2f,%.2f,%.2f) mission_goal=(%.2f,%.2f,%.2f) "
        "topology_anchor=(%.2f,%.2f,%.2f) frontier_goal=(%.2f,%.2f,%.2f) "
        "local_goal=(%.2f,%.2f,%.2f) local_goal_distance=%.2f m "
        "local_goal_to_frontier=%.2f m vehicle_to_frontier=%.2f m "
        "local_goal_source=%s speed=%.2f m/s lookahead=%.2f m "
        "planner_tick=%d ms",
        position_.x(), position_.y(), position_.z(), goal_.x(), goal_.y(), goal_.z(),
        topology_frontier_goal.x(), topology_frontier_goal.y(), topology_frontier_goal.z(),
        accepted_route_.frontier_goal.x(), accepted_route_.frontier_goal.y(), accepted_route_.frontier_goal.z(),
        local_goal.x(), local_goal.y(), local_goal.z(), (local_goal - position_).norm(),
        local_goal_to_frontier_goal, (accepted_route_.frontier_goal - position_).norm(),
        hold_local_goal ? "HELD" : "CURRENT",
        speed_mps_, effective_lookahead_m, update_period_ms_);
    } else {
      have_previous_local_goal_ = false;
      // Any route without a valid next goal must retract the previous command.
      // This includes an exhausted accepted route while the next frontier
      // search is pending; otherwise the controller can keep pursuing a stale
      // subgoal and fly past the frontier.
      geometry_msgs::msg::PoseStamped stop_message;
      stop_message.header.stamp = now();
      stop_message.header.frame_id = next_goal_frame_;
      stop_message.pose.position = toPoint(position_);
      stop_message.pose.orientation.w = 1.0;
      next_goal_pub_->publish(stop_message);
    }
    graph.markers.push_back(next_goal_marker);

    visualization_msgs::msg::Marker vehicle_marker = skeleton_nodes;
    vehicle_marker.ns = "scalenav_vehicle_pose";
    vehicle_marker.id = 7;
    vehicle_marker.type = visualization_msgs::msg::Marker::ARROW;
    vehicle_marker.action = visualization_msgs::msg::Marker::ADD;
    vehicle_marker.pose.position = toPoint(position_);
    vehicle_marker.pose.orientation.x = orientation_.x();
    vehicle_marker.pose.orientation.y = orientation_.y();
    vehicle_marker.pose.orientation.z = orientation_.z();
    vehicle_marker.pose.orientation.w = orientation_.w();
    vehicle_marker.scale.x = 1.20;
    vehicle_marker.scale.y = 0.32;
    vehicle_marker.scale.z = 0.32;
    setColor(vehicle_marker.color, kUav);
    vehicle_marker.points.clear();
    vehicle_marker.colors.clear();
    graph.markers.push_back(vehicle_marker);

    visualization_msgs::msg::Marker goal_marker = skeleton_nodes;
    goal_marker.ns = "scalenav_global_goal";
    goal_marker.id = 8;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = have_goal_ ? visualization_msgs::msg::Marker::ADD :
      visualization_msgs::msg::Marker::DELETE;
    goal_marker.pose.position = toPoint(goal_);
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.scale.x = 0.80;
    goal_marker.scale.y = 0.80;
    goal_marker.scale.z = 0.80;
    setColor(goal_marker.color, kMissionGoal);
    goal_marker.points.clear();
    goal_marker.colors.clear();
    graph.markers.push_back(goal_marker);

    // The rolling A* frontier goal is distinct from both the mission goal and
    // the short local execution goal. Make all three visible in RViz.
    visualization_msgs::msg::Marker frontier_goal_marker = goal_marker;
    frontier_goal_marker.ns = "scalenav_frontier_goal";
    frontier_goal_marker.id = 10;
    frontier_goal_marker.action = accepted_route_.valid ?
      visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    frontier_goal_marker.scale.x = 0.62;
    frontier_goal_marker.scale.y = 0.62;
    frontier_goal_marker.scale.z = 0.62;
    setColor(frontier_goal_marker.color, kFrontierGoal);
    frontier_goal_marker.pose.position = toPoint(projectPlanningPoint(
      accepted_route_.frontier_goal, graph_fixed_layer_, graph_layer_z_));
    graph.markers.push_back(frontier_goal_marker);

    visualization_msgs::msg::Marker frontier_goal_label = goal_marker;
    frontier_goal_label.ns = "scalenav_frontier_goal_label";
    frontier_goal_label.id = 11;
    frontier_goal_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    frontier_goal_label.action = accepted_route_.valid ?
      visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    const Eigen::Vector3f frontier_goal_label_position =
      projectPlanningPoint(accepted_route_.frontier_goal, graph_fixed_layer_, graph_layer_z_) +
      Eigen::Vector3f(0.0F, 0.0F, 0.8F);
    frontier_goal_label.pose.position = toPoint(frontier_goal_label_position);
    setColor(frontier_goal_label.color, kFrontierGoal);
    frontier_goal_label.text = "FRONTIER GOAL";
    frontier_goal_label.scale.z = 0.45;
    graph.markers.push_back(frontier_goal_label);

    visualization_msgs::msg::Marker subgoal_label = goal_marker;
    subgoal_label.ns = "scalenav_local_goal_label";
    subgoal_label.id = 12;
    subgoal_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    subgoal_label.action = has_local_goal ?
      visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    const Eigen::Vector3f subgoal_label_position =
      local_goal + Eigen::Vector3f(0.0F, 0.0F, 0.8F);
    subgoal_label.pose.position = toPoint(subgoal_label_position);
    setColor(subgoal_label.color, kLocalGoal);
    subgoal_label.text = "LOCAL GOAL";
    subgoal_label.scale.z = 0.45;
    graph.markers.push_back(subgoal_label);

    visualization_msgs::msg::Marker vehicle_label = skeleton_nodes;
    vehicle_label.ns = "scalenav_vehicle_label";
    vehicle_label.id = 9;
    vehicle_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    vehicle_label.action = visualization_msgs::msg::Marker::ADD;
    const Eigen::Vector3f vehicle_label_position =
      position_ + Eigen::Vector3f(0.0F, 0.0F, 0.8F);
    vehicle_label.pose.position = toPoint(vehicle_label_position);
    vehicle_label.pose.orientation.w = 1.0;
    vehicle_label.scale.z = 0.45;
    setColor(vehicle_label.color, kUav);
    vehicle_label.text = "UAV";
    vehicle_label.points.clear();
    graph.markers.push_back(vehicle_label);

    visualization_msgs::msg::Marker goal_label = vehicle_label;
    goal_label.ns = "scalenav_goal_label";
    goal_label.id = 10;
    const Eigen::Vector3f goal_label_position =
      goal_ + Eigen::Vector3f(0.0F, 0.0F, 0.8F);
    goal_label.pose.position = toPoint(goal_label_position);
    setColor(goal_label.color, kMissionGoal);
    goal_label.text = "MISSION GOAL";
    goal_label.action = have_goal_ ? visualization_msgs::msg::Marker::ADD :
      visualization_msgs::msg::Marker::DELETE;
    graph.markers.push_back(goal_label);

    graph_pub_->publish(graph);

    visualization_msgs::msg::MarkerArray bubbles;
    visualization_msgs::msg::Marker delete_bubbles;
    delete_bubbles.header = skeleton_nodes.header;
    delete_bubbles.ns = "scalenav_real_bubbles";
    delete_bubbles.id = 0;
    delete_bubbles.action = visualization_msgs::msg::Marker::DELETEALL;
    bubbles.markers.push_back(delete_bubbles);
    visualization_msgs::msg::Marker delete_route_radii = delete_bubbles;
    delete_route_radii.ns = "scalenav_route_bubble_radius";
    bubbles.markers.push_back(delete_route_radii);
    const auto bubble_snapshot = topo->getBubbleSnapshot();
    stats.bubbles = bubble_snapshot.size();
    visualization_msgs::msg::Marker bubble_list;
    bubble_list.header = skeleton_nodes.header;
    bubble_list.ns = "scalenav_real_bubbles";
    bubble_list.id = 1;
    bubble_list.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    bubble_list.action = visualization_msgs::msg::Marker::ADD;
    bubble_list.pose.orientation.w = 1.0;
    bubble_list.scale.x = 0.70;
    bubble_list.scale.y = 0.70;
    bubble_list.scale.z = 0.70;
    setColor(bubble_list.color, kTopology, 0.16F);
    constexpr std::size_t max_bubble_markers = 400;
    const std::size_t bubble_stride = bubble_snapshot.size() > max_bubble_markers ?
      std::max<std::size_t>(1, bubble_snapshot.size() / max_bubble_markers) : 1;
    bubble_list.points.reserve(std::min(bubble_snapshot.size(), max_bubble_markers));
    for (std::size_t i = 0; i < bubble_snapshot.size(); i += bubble_stride) {
      const auto &source = bubble_snapshot[i];
      if (!source) continue;
      bubble_list.points.push_back(toPoint(source->center_));
    }
    bubbles.markers.push_back(std::move(bubble_list));
    // SPHERE_LIST cannot encode a different radius per point. Publish the
    // selected route nodes as machine-readable spheres carrying their
    // original topology-bubble radii. This is exact and much smaller than
    // publishing one marker for every raw bubble in the map.
    std::size_t radius_id = 0;
    for (const auto &source : (found ? path_nodes : std::vector<TopoNode::Ptr>{})) {
      if (!source || !std::isfinite(source->bubble_radius_) || source->bubble_radius_ <= 0.0F) continue;
      visualization_msgs::msg::Marker bubble_radius;
      bubble_radius.header = skeleton_nodes.header;
      bubble_radius.ns = "scalenav_route_bubble_radius";
      bubble_radius.id = static_cast<int>(++radius_id);
      bubble_radius.type = visualization_msgs::msg::Marker::SPHERE;
      bubble_radius.action = visualization_msgs::msg::Marker::ADD;
      bubble_radius.pose.orientation.w = 1.0;
      Eigen::Vector3f center = source->center_;
      if (graph_fixed_layer_) center.z() = static_cast<float>(graph_layer_z_);
      bubble_radius.pose.position = toPoint(center);
      bubble_radius.scale.x = 2.0F * source->bubble_radius_;
      bubble_radius.scale.y = 2.0F * source->bubble_radius_;
      bubble_radius.scale.z = 2.0F * source->bubble_radius_;
      setColor(bubble_radius.color, kTopology, 0.08F);
      bubbles.markers.push_back(std::move(bubble_radius));
    }
    bubble_pub_->publish(bubbles);

    nav_msgs::msg::Path path;
    path.header = skeleton_nodes.header;
    for (const auto &point : selected_node_path) {
      geometry_msgs::msg::PoseStamped pose;
      pose.header = path.header;
      pose.pose.position = toPoint(point);
      pose.pose.orientation.w = 1.0;
      path.poses.push_back(std::move(pose));
    }
    path_pub_->publish(path);

    // Report safety using the same filtered obstacle distance field as Bubble
    // A*. Sample path segments so sparse witness vertices cannot hide a close
    // obstacle between two poses.
    geometry_msgs::msg::Vector3Stamped clearance;
    clearance.header = path.header;
    Eigen::Vector3f vehicle_query = position_;
    if (graph_fixed_layer_) vehicle_query.z() = static_cast<float>(graph_layer_z_);
    clearance.vector.x = topo->lidar_map_interface_->getDisToOcc(vehicle_query);
    double path_clearance_sum = 0.0;
    double path_clearance_min = std::numeric_limits<double>::infinity();
    std::size_t path_clearance_samples = 0;
    const auto add_clearance_sample = [&](const Eigen::Vector3f &point) {
      const double value = topo->lidar_map_interface_->getDisToOcc(point);
      if (!std::isfinite(value)) return;
      path_clearance_sum += value;
      path_clearance_min = std::min(path_clearance_min, value);
      ++path_clearance_samples;
    };
    if (!selected_node_path.empty()) {
      add_clearance_sample(selected_node_path.front());
      constexpr float sample_step = 0.25F;
      for (std::size_t i = 1; i < selected_node_path.size(); ++i) {
        const Eigen::Vector3f segment = selected_node_path[i] - selected_node_path[i - 1];
        const float length = segment.norm();
        if (!std::isfinite(length)) continue;
        const int steps = std::max(1, static_cast<int>(std::ceil(length / sample_step)));
        for (int step = 1; step <= steps; ++step) {
          add_clearance_sample(
            selected_node_path[i - 1] + segment * (static_cast<float>(step) / steps));
        }
      }
    }
    clearance.vector.y = path_clearance_samples > 0 ? path_clearance_min :
      std::numeric_limits<double>::quiet_NaN();
    clearance.vector.z = path_clearance_samples > 0 ?
      path_clearance_sum / static_cast<double>(path_clearance_samples) :
      std::numeric_limits<double>::quiet_NaN();
    clearance_pub_->publish(clearance);
    return stats;
  }

  std::string cloud_topic_, free_ray_topic_, odom_topic_, goal_topic_, next_goal_topic_,
    next_goal_frame_, clearance_topic_, timing_topic_;
  std::string visualization_frame_;
  std::string odom_twist_frame_ = "world";
  std::string flight_statistics_file_ = "scalenav_flight_statistics.csv";
  std::string graph_log_file_ = "scalenav_graph_snapshots.jsonl";
  double trajectory_speed_color_max_mps_ = 6.0;
  std::size_t trajectory_max_points_ = 50000;
  double map_margin_ = 20.0;
  bool graph_fixed_layer_ = true;
  bool reuse_graph_on_goal_ = true;
  bool graph_layer_initialized_ = false;
  double graph_layer_z_ = 1.6;
  double map_voxel_size_ = 0.1;
  double map_history_radius_m_ = 0.0;
  int map_max_points_ = 20000;
  double map_prune_distance_m_ = 0.5;
  double skeleton_rebuild_period_ms_ = 200.0;
  int diagnostic_log_period_ms_ = 2000;
  double local_goal_min_advance_m_ = 0.75;
  double local_goal_lookahead_m_ = 15.0;
  double frontier_replan_progress_ratio_ = 0.40;
  int route_plan_period_ms_ = 100;  // launch compatibility; see update()
  double local_goal_reserve_m_ = 0.0;  // launch compatibility; see update()
  double local_graph_radius_m_ = 45.0;
  double frontier_goal_margin_m_ = 3.5;
  double frontier_progress_loss_weight_ = 0.5;
  double frontier_direction_loss_weight_ = 0.35;
  double frontier_fov_loss_weight_ = 0.2;
  double frontier_smoothness_loss_weight_ = 0.35;
  bool use_edge_witness_path_ = false;
  double goal_path_cost_weight_ = 1.0;
  double frontier_goal_distance_weight_ = 2.0;
  double frontier_semantic_detour_budget_m_ = 45.0;
  double frontier_semantic_frame_budget_m_ = 12.0;
  double frontier_semantic_noise_floor_ = 0.08;
  double semantic_cost_weight_ = 2.0;
  int semantic_opportunity_persistence_frames_ = 2;
  double semantic_opportunity_switch_margin_m_ = 3.0;
  double semantic_opportunity_cooldown_s_ = 0.8;
  double semantic_opportunity_direction_tolerance_deg_ = 30.0;
  double semantic_route_influence_m_ = 8.0;
  double semantic_point_influence_m_ = 8.0;
  double semantic_visualization_max_score_ = 0.4;
  double semantic_baseline_quantile_ = 0.25;
  double semantic_virtual_depth_m_ = 30.0;
  bool semantic_points_enabled_ = true;
  double semantic_point_min_score_ = 0.20;
  double semantic_point_separation_m_ = 1.5;
  double semantic_point_radius_m_ = 0.75;
  int semantic_point_max_nodes_ = 16;
  bool virtual_semantic_prune_enabled_ = true;
  double virtual_semantic_backtrack_margin_m_ = 12.0;
  int virtual_semantic_max_nodes_ = 512;
  int semantic_label_max_nodes_ = 16;
  double clearance_cost_weight_ = 2.0;
  double clearance_target_m_ = 1.2;
  double local_goal_hold_timeout_ms_ = 400.0;
  double goal_connect_distance_m_ = 6.0;
  double goal_connect_timeout_ms_ = 20.0;
  double odom_reconnect_distance_m_ = 1.0;
  double odom_reconnect_yaw_deg_ = 20.0;
  double odom_fallback_radius_m_ = 15.0;
  int odom_fallback_candidates_ = 8;
  double odom_connect_timeout_ms_ = 3.0;
  double cloud_pose_tolerance_ms_ = 50.0;
  std::string semantic_heatmap_topic_;
  double semantic_pose_tolerance_ms_ = 100.0;
  double semantic_max_age_ms_ = 1500.0;
  double semantic_risk_memory_ms_ = 5000.0;
  double semantic_risk_accumulation_alpha_ = 0.25;
  bool wait_for_initial_semantic_ = true;
  bool initial_semantic_wait_complete_ = false;
  bool initial_semantic_wait_started_ = false;
  double initial_semantic_wait_timeout_ms_ = 5000.0;
  std::chrono::steady_clock::time_point initial_semantic_wait_start_{};
  double semantic_camera_tx_ = 0.5;
  double semantic_camera_ty_ = 0.0;
  double semantic_camera_tz_ = -0.1;
  std::string semantic_depth_topic_;
  double semantic_depth_tolerance_ms_ = 50.0;
  double semantic_depth_max_m_ = 20.0;
  double semantic_horizontal_fov_deg_ = 90.0;
  double semantic_vertical_fov_deg_ = 60.0;
  int semantic_patch_cols_ = 5;
  int semantic_patch_rows_ = 3;
  int update_period_ms_ = 100;
  fast_planner::LIOInterface::Ptr map_;
  ParallelBubbleAstar::Ptr astar_;
  TopoGraph::Ptr topo_;
  TopoGraph::Ptr graph_odom_topo_;
  AcceptedRouteState accepted_route_;
  std::vector<Eigen::Vector3f> polynomial_guide_path_;
  scalenav_graph::WitnessParametricCurve polynomial_curve_;
  bool polynomial_curve_valid_ = false;
  std::atomic<std::uint64_t> topology_update_generation_{0};
  Eigen::Vector3f previous_local_goal_ = Eigen::Vector3f::Zero();
  bool have_previous_local_goal_ = false;
  std::chrono::steady_clock::time_point previous_local_goal_time_{};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr free_ray_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr semantic_heatmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr semantic_depth_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::CallbackGroup::SharedPtr cloud_callback_group_;
  rclcpp::CallbackGroup::SharedPtr semantic_callback_group_;
  rclcpp::CallbackGroup::SharedPtr state_callback_group_;
  rclcpp::CallbackGroup::SharedPtr planner_callback_group_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bubble_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr flight_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_goal_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr clearance_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr timing_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr flight_timer_;
  Eigen::Vector3f position_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f goal_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f world_velocity_ = Eigen::Vector3f::Zero();
  Eigen::Quaternionf orientation_ = Eigen::Quaternionf::Identity();
  std::deque<TimedPose> odom_history_;
  mutable std::mutex odom_mutex_;
  static constexpr std::size_t max_odom_history_size_ = 512;
  std::optional<SemanticFrame> semantic_frame_;
  std::deque<SemanticDepthFrame> semantic_depth_history_;
  static constexpr std::size_t max_semantic_depth_history_size_ = 64;
  std::int64_t last_semantic_applied_stamp_ns_ = 0;
  std::int64_t last_semantic_opportunity_evaluated_stamp_ns_ = 0;
  TopoGraph::Ptr semantic_applied_topo_;
  Eigen::Vector3f pending_semantic_opportunity_direction_ = Eigen::Vector3f::Zero();
  int pending_semantic_opportunity_frames_ = 0;
  std::chrono::steady_clock::time_point last_semantic_opportunity_probe_time_{};
  bool have_semantic_opportunity_probe_time_ = false;
  std::mutex semantic_mutex_;
  mutable std::mutex semantic_memory_mutex_;
  std::unordered_map<std::uint64_t, TopoSemanticRecord> semantic_memory_;
  std::size_t previous_semantic_label_count_ = 0;
  float speed_mps_ = 0.0F;
  std::deque<TrajectorySample> flight_trajectory_;
  double flight_start_time_s_ = 0.0;
  double flight_path_length_m_ = 0.0;
  double flight_duration_s_ = 0.0;
  double flight_speed_integral_ = 0.0;
  double flight_max_speed_mps_ = 0.0;
  double flight_max_acceleration_mps2_ = 0.0;
  double flight_max_jerk_mps3_ = 0.0;
  double flight_jerk_squared_integral_ = 0.0;
  Eigen::Vector3f flight_acceleration_ = Eigen::Vector3f::Zero();
  bool have_flight_acceleration_ = false;
  std::chrono::steady_clock::time_point last_flight_statistics_write_;
  bool have_flight_statistics_write_time_ = false;
  std::chrono::steady_clock::time_point last_graph_log_time_;
  bool have_graph_log_time_ = false;
  Eigen::Vector3f graph_odom_position_ = Eigen::Vector3f::Zero();
  float graph_odom_yaw_ = 0.0F;
  bool have_odom_ = false;
  bool have_goal_ = false;
  bool mission_direct_goal_latched_ = false;
  bool have_cloud_ = false;
  std::atomic<bool> graph_initialized_{false};
  std::atomic<bool> have_graph_odom_{false};
  std::atomic<bool> skeleton_initialized_{false};
  std::atomic<bool> map_changed_{false};
  bool have_skeleton_rebuild_time_ = false;
  std::chrono::steady_clock::time_point last_skeleton_rebuild_time_;
  std::uint64_t cloud_count_ = 0;
  std::atomic<std::uint64_t> skeleton_update_count_{0};
  std::mutex graph_mutex_;
  mutable std::mutex topology_operation_mutex_;
  std::thread rebuild_thread_;
  std::atomic<bool> rebuild_running_{false};
  std::atomic<bool> shutting_down_{false};
  std::atomic<std::uint64_t> goal_generation_{0};
};

int main(int argc, char **argv)
{
  std::signal(SIGSEGV, crashSignalHandler);
  std::signal(SIGABRT, crashSignalHandler);
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ScaleNavGraphNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
