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
  static constexpr char message[] = "\nEPIC fatal signal stack:\n";
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

void setColor(std_msgs::msg::ColorRGBA &color, const Rgb &rgb, float alpha = 1.0F)
{
  color.r = rgb.r;
  color.g = rgb.g;
  color.b = rgb.b;
  color.a = alpha;
}

}  // namespace

class EpicGraphNode final : public rclcpp::Node {
 public:
  EpicGraphNode()
  : Node("epic_graph_node"),
    map_(std::make_shared<fast_planner::LIOInterface>()),
    astar_(std::make_shared<ParallelBubbleAstar>()),
    topo_(std::make_shared<TopoGraph>())
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/depth/points");
    free_ray_topic_ = declare_parameter<std::string>("free_ray_topic", "/depth/free_rays");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/sim/odom");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal");
    next_goal_topic_ = declare_parameter<std::string>("next_goal_topic", "/epic/local_goal");
    clearance_topic_ = declare_parameter<std::string>("clearance_topic", "/epic/clearance");
    next_goal_frame_ = declare_parameter<std::string>("next_goal_frame", "world_enu");
    visualization_frame_ = declare_parameter<std::string>("visualization_frame", "odom");
    odom_twist_frame_ = declare_parameter<std::string>("odom_twist_frame", "world");
    flight_statistics_file_ = declare_parameter<std::string>(
      "flight_statistics_file", "epic_flight_statistics.csv");
    graph_log_file_ = declare_parameter<std::string>(
      "graph_log_file", "epic_graph_snapshots.jsonl");
    trajectory_speed_color_max_mps_ = declare_parameter<double>(
      "trajectory_speed_color_max_mps", 8.0);
    trajectory_max_points_ = static_cast<std::size_t>(std::max(
      1000L, static_cast<long>(declare_parameter<int>("trajectory_max_points", 50000))));
    graph_fixed_layer_ = declare_parameter<bool>("graph_fixed_layer", true);
    graph_layer_z_ = declare_parameter<double>("graph_layer_z", 1.6);
    reuse_graph_on_goal_ = declare_parameter<bool>("reuse_graph_on_goal", true);
    // Keep route planning stateless while diagnosing replanning behavior. The
    // The accepted witness remains executable state for the current tick,
    // but it must not bias or replace a fresh A* search.
    reuse_previous_route_ = declare_parameter<bool>("reuse_previous_route", false);
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
    local_goal_lookahead_m_ = declare_parameter<double>("local_goal_lookahead_m", 10.0);
    // Kept as launch-API compatibility knobs.  Planning is intentionally
    // performed on every update tick; throttling it here made the graph and
    // subgoal stale while the vehicle was moving.
    route_plan_period_ms_ = declare_parameter<int>("route_plan_period_ms", 100);
    local_goal_reserve_m_ = declare_parameter<double>("local_goal_reserve_m", 0.0);
    local_graph_radius_m_ = declare_parameter<double>("local_graph_radius_m", 35.0);
    frontier_goal_margin_m_ = declare_parameter<double>("frontier_goal_margin_m", 3.5);
    frontier_progress_loss_weight_ = declare_parameter<double>(
      "frontier_progress_loss_weight", 0.5);
    frontier_direction_loss_weight_ = declare_parameter<double>(
      "frontier_direction_loss_weight", 0.35);
    frontier_fov_loss_weight_ = declare_parameter<double>(
      "frontier_fov_loss_weight", 0.2);
    frontier_smoothness_loss_weight_ = declare_parameter<double>(
      "frontier_smoothness_loss_weight", 0.35);
    use_edge_witness_path_ = declare_parameter<bool>("use_edge_witness_path", true);
    // EPIC already stores collision-checked witness paths on each edge.  A
    // second clearance raycast over every published segment is not part of
    // the original planner and can consume most of the route-update period.
    goal_path_cost_weight_ = declare_parameter<double>("goal_path_cost_weight", 1.0);
    semantic_cost_weight_ = declare_parameter<double>("semantic_cost_weight", 2.0);
    semantic_route_replan_delta_ = declare_parameter<double>(
      "semantic_route_replan_delta", 0.05);
    semantic_route_replan_enabled_ = declare_parameter<bool>(
      "semantic_route_replan_enabled", true);
    semantic_route_high_risk_ = declare_parameter<double>(
      "semantic_route_high_risk", 0.35);
    semantic_route_high_risk_release_ = declare_parameter<double>(
      "semantic_route_high_risk_release", 0.30);
    semantic_route_switch_risk_margin_ = declare_parameter<double>(
      "semantic_route_switch_risk_margin", 0.08);
    semantic_route_switch_cost_ratio_ = declare_parameter<double>(
      "semantic_route_switch_cost_ratio", 0.90);
    semantic_route_influence_m_ = declare_parameter<double>(
      "semantic_route_influence_m", 5.0);
    semantic_visualization_max_score_ = declare_parameter<double>(
      "semantic_visualization_max_score", 0.4);
    semantic_baseline_quantile_ = declare_parameter<double>(
      "semantic_baseline_quantile", 0.25);
    previous_path_cost_factor_ = declare_parameter<double>("previous_path_cost_factor", 1.0);
    route_remap_distance_m_ = declare_parameter<double>("route_remap_distance_m", 1.25);
    route_reuse_horizon_m_ = declare_parameter<double>("route_reuse_horizon_m", 10.0);
    route_reuse_lateral_distance_m_ =
      declare_parameter<double>("route_reuse_lateral_distance_m", 1.5);
    local_goal_hold_timeout_ms_ = declare_parameter<double>(
      "local_goal_hold_timeout_ms", 400.0);
    frontier_extension_search_period_ms_ = declare_parameter<double>(
      "frontier_extension_search_period_ms", 1000.0);
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
    semantic_pose_tolerance_ms_ = declare_parameter<double>(
      "semantic_pose_tolerance_ms", 250.0);
    semantic_max_age_ms_ = declare_parameter<double>("semantic_max_age_ms", 1500.0);
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
    semantic_point_min_score_ = declare_parameter<double>("semantic_point_min_score", 0.35);
    semantic_point_separation_m_ = declare_parameter<double>(
      "semantic_point_separation_m", 1.5);
    semantic_point_radius_m_ = declare_parameter<double>("semantic_point_radius_m", 0.75);
    semantic_point_max_nodes_ = declare_parameter<int>("semantic_point_max_nodes", 16);

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
    clearance_cost_weight_ = declare_parameter<double>(
      "bubble_topo/clearance_cost_weight", 2.0);
    clearance_target_m_ = declare_parameter<double>(
      "bubble_topo/clearance_target_m", 1.2);
    declare_parameter<bool>("bubble_topo/planar_graph", graph_fixed_layer_);
    declare_parameter<double>("bubble_topo/planar_z", graph_layer_z_);
    declare_parameter<int>("max_update_region_num", 0);
    // Odom reconnection is on the online update path. Keep the EPIC local
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
      "EPIC Bubble/TopoGraph volume=3D; local goal layer=%s z=%.2f; "
      "obstacle_min_z=%.2f; planner_tick=%.2f Hz lookahead=%.2f m "
      "local_graph_radius=%.1f m",
      graph_fixed_layer_ ? "fixed" : "3D", graph_layer_z_,
      graph_fixed_layer_ ? graph_layer_z_ - 1.0 : std::numeric_limits<double>::quiet_NaN(),
      1000.0 / static_cast<double>(std::max(1, update_period_ms_)),
      local_goal_lookahead_m_, local_graph_radius_m_);
    RCLCPP_INFO(
      get_logger(),
      "EPIC config: clearance_target=%.2f m clearance_weight=%.2f "
      "geometry_map=%s reuse_previous_route=%d semantic_radius=%.2f m "
      "semantic_visual_max=%.2f baseline_q=%.2f "
      "diagnostic_period=%d ms",
      clearance_target_m_, clearance_cost_weight_,
      map_history_radius_m_ <= 0.0 ? "CURRENT_FRAME" : "SLIDING_WINDOW",
      static_cast<int>(reuse_previous_route_),
      semantic_point_radius_m_, semantic_visualization_max_score_,
      semantic_baseline_quantile_,
      diagnostic_log_period_ms_);
    RCLCPP_INFO(get_logger(), "EPIC graph snapshots: file=%s period=%d ms",
      graph_log_file_.c_str(), diagnostic_log_period_ms_);

    graph_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/epic/graph", 1);
    bubble_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/epic/bubbles", 1);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/epic/path", 1);
    flight_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/epic/flight", 1);
    next_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(next_goal_topic_, 10);
    clearance_pub_ =
      create_publisher<geometry_msgs::msg::Vector3Stamped>(clearance_topic_, 10);

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

  ~EpicGraphNode() override
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
  };

  // The only persistent representation of the route currently accepted for
  // execution.  A candidate topology path is not part of this state until
  // publish() has assembled and collision-checked its complete witness.
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
      RCLCPP_INFO(get_logger(), "EPIC graph fixed layer z=%.2f obstacle_min_z=%.2f",
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
    // Sequential gray-blue scale keeps speed distinct without rainbow hues.
    constexpr Rgb anchors[3] = {kCandidate, kTopology, kUav};
    const float scaled = t * 2.0F;
    const int index = std::min(1, static_cast<int>(scaled));
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
    trajectory.ns = "epic_flight_trajectory";
    trajectory.id = 0;
    trajectory.type = visualization_msgs::msg::Marker::LINE_LIST;
    trajectory.action = visualization_msgs::msg::Marker::ADD;
    trajectory.scale.x = 0.10;
    trajectory.color.a = 1.0;
    for (std::size_t i = 1; i < sample_indices.size(); ++i) {
      const auto &from = flight_trajectory_[sample_indices[i - 1]];
      const auto &to = flight_trajectory_[sample_indices[i]];
      const auto color = speedColor(
        0.5F * (from.velocity.norm() + to.velocity.norm()),
        trajectory_speed_color_max_mps_);
      trajectory.points.push_back(toPoint(from.position));
      trajectory.points.push_back(toPoint(to.position));
      trajectory.colors.push_back(color);
      trajectory.colors.push_back(color);
    }
    message.markers.push_back(std::move(trajectory));

    const double rms_jerk = flight_duration_s_ > 1e-6 ?
      std::sqrt(flight_jerk_squared_integral_ / flight_duration_s_) : 0.0;
    const double average_speed = flight_duration_s_ > 1e-6 ?
      flight_speed_integral_ / flight_duration_s_ : 0.0;
    visualization_msgs::msg::Marker vehicle;
    vehicle.header.frame_id = visualization_frame_;
    vehicle.header.stamp = message.markers.front().header.stamp;
    vehicle.ns = "epic_flight_vehicle";
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
    setColor(vehicle.color, kUav);
    message.markers.push_back(std::move(vehicle));
    flight_pub_->publish(message);
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "[EPIC flight] path=%.2f m duration=%.2f s speed=%.2f/%.2f m/s "
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
      << wall_time << ",epic," << (final ? 1 : 0) << ","
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
    Eigen::Vector3f forward = orientation_ * Eigen::Vector3f::UnitX();
    forward.z() = 0.0F;
    if (forward.squaredNorm() > 1e-6F) forward.normalize();
    for (const auto &node : nodes) {
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
      "EPIC graph stats: nodes=%zu edges=%zu directed=%zu degree0=%zu "
      "asymmetric=%zu dangling=%zu duplicate<0.25m=%zu",
      nodes.size(), edge_count, directed_edge_count, zero_degree_nodes,
      asymmetric_edge_count, dangling_neighbor_count, duplicate_pair_count);
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
        << ",\"geometry_miss_count\":" << static_cast<unsigned>(node->geometry_miss_count_)
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

    const std::int64_t stamp_ns = stampNanoseconds(message->header.stamp);
    std::size_t semantic_point_count = 0;
    float semantic_frame_baseline = 0.0F;
    float semantic_raw_min = 0.0F;
    float semantic_raw_max = 0.0F;
    float semantic_risk_min = 0.0F;
    float semantic_risk_max = 0.0F;
    const Eigen::Vector3f camera_translation(
      static_cast<float>(semantic_camera_tx_), static_cast<float>(semantic_camera_ty_),
      static_cast<float>(semantic_camera_tz_));
    // The published heatmap is an upsampled image. Its pixels are not
    // independent semantic observations: keep one strongest ray per model
    // patch, producing the compact 5x3 (15-ray) semantic table by default.
    // Every selected patch pixel is projected with a fixed optical Z depth,
    // using the same pinhole convention as the ordinary depth point cloud.
    // The semantic layer is intentionally independent of measured depth.
    const std::size_t patch_cols = static_cast<std::size_t>(semantic_patch_cols_);
    const std::size_t patch_rows = static_cast<std::size_t>(semantic_patch_rows_);
    const std::size_t patch_count = patch_cols * patch_rows;
    std::vector<float> patch_scores(patch_count, 0.0F);
    std::vector<std::uint8_t> patch_valid(patch_count, 0U);
    std::vector<std::uint32_t> patch_pixel_u(patch_count, 0U);
    std::vector<std::uint32_t> patch_pixel_v(patch_count, 0U);
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
        if (!patch_valid[patch_index] || semantic > patch_scores[patch_index]) {
          patch_scores[patch_index] = semantic;
          patch_pixel_u[patch_index] = u;
          patch_pixel_v[patch_index] = v;
          patch_valid[patch_index] = 1U;
        }
      }
    }
    {
      std::lock_guard<std::mutex> lock(semantic_mutex_);
      SemanticFrame frame;
      frame.stamp_ns = stamp_ns;
      frame.origin = capture_pose.position;
      frame.points_world.reserve(patch_count);
      frame.scores.reserve(patch_count);
      frame.confidences.reserve(patch_count);
      std::vector<float> valid_patch_scores;
      valid_patch_scores.reserve(patch_count);
      for (std::size_t i = 0; i < patch_count; ++i) {
        if (patch_valid[i]) valid_patch_scores.push_back(patch_scores[i]);
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
        if (patch_valid[i]) {
          calibrated_scores[i] = calibrateSemanticScore(patch_scores[i], frame_baseline);
        }
      }
      for (std::size_t i = 0; i < patch_count; ++i) {
        if (!patch_valid[i]) continue;
        const float normalized_u =
          (static_cast<float>(patch_pixel_u[i]) + 0.5F) /
          static_cast<float>(message->width);
        const float normalized_v =
          (static_cast<float>(patch_pixel_v[i]) + 0.5F) /
          static_cast<float>(message->height);
        const Eigen::Vector3f body = virtualSemanticPointFlu(
          normalized_u, normalized_v,
          static_cast<float>(semantic_horizontal_fov_deg_),
          static_cast<float>(semantic_vertical_fov_deg_),
          static_cast<float>(semantic_virtual_depth_m_), camera_translation);
        const Eigen::Vector3f point_world =
          capture_pose.position + capture_pose.orientation * body;
        frame.points_world.push_back(point_world);
        const float calibrated_score = calibrated_scores[i];
        frame.scores.push_back(calibrated_score);

        const float fov_radius = std::clamp(std::max(
          std::abs(2.0F * normalized_u - 1.0F),
          std::abs(2.0F * normalized_v - 1.0F)), 0.0F, 1.0F);
        const float fov_confidence = 1.0F - 0.35F * fov_radius * fov_radius;
        const std::size_t patch_col = i % patch_cols;
        const float support_threshold = std::max(
          static_cast<float>(semantic_point_min_score_), 0.65F * calibrated_score);
        std::size_t row_support = 0;
        for (std::size_t row = 0; row < patch_rows; ++row) {
          const std::size_t neighbor_index = row * patch_cols + patch_col;
          if (patch_valid[neighbor_index] &&
              calibrated_scores[neighbor_index] >= support_threshold) {
            ++row_support;
          }
        }
        const float row_confidence = patch_rows <= 1 ? 0.7F :
          0.65F + 0.35F * static_cast<float>(row_support > 0 ? row_support - 1 : 0) /
            static_cast<float>(patch_rows - 1);
        const float below_layer = static_cast<float>(graph_layer_z_) - point_world.z();
        const float ground_confidence = !graph_fixed_layer_ || below_layer <= 0.5F ?
          1.0F : std::clamp(
            1.0F - (below_layer - 0.5F) / 5.0F, 0.25F, 1.0F);
        frame.confidences.push_back(std::clamp(
          fov_confidence * row_confidence * ground_confidence, 0.05F, 1.0F));
        raw_min = std::min(raw_min, patch_scores[i]);
        raw_max = std::max(raw_max, patch_scores[i]);
        calibrated_min = std::min(calibrated_min, calibrated_score);
        calibrated_max = std::max(calibrated_max, calibrated_score);
      }
      semantic_point_count = frame.points_world.size();
      semantic_frame_baseline = frame_baseline;
      semantic_raw_min = std::isfinite(raw_min) ? raw_min : 0.0F;
      semantic_raw_max = raw_max;
      semantic_risk_min = std::isfinite(calibrated_min) ? calibrated_min : 0.0F;
      semantic_risk_max = calibrated_max;
      semantic_frame_ = std::move(frame);
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), diagnostic_log_period_ms_,
      "[EPIC semantic] image=%ux%u patches=%zux%zu points=%zu virtual_depth=%.2f m "
      "raw=%.3f..%.3f baseline=%.3f risk=%.3f..%.3f pose_sync=%.1f ms",
      message->width, message->height, patch_cols, patch_rows,
      semantic_point_count, semantic_virtual_depth_m_, semantic_raw_min,
      semantic_raw_max, semantic_frame_baseline, semantic_risk_min,
      semantic_risk_max, pose_delta_ms);
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
      // A new mission goal gets a new route, even when the existing topology
      // and its edge witness paths are reused.
      accepted_route_.clear();
      polynomial_guide_path_.clear();
      polynomial_curve_ = scalenav_graph::WitnessParametricCurve();
      polynomial_curve_valid_ = false;
      have_last_extension_search_ = false;
      semantic_replan_requested_ = false;
      have_evaluated_route_risk_ = false;
      evaluated_route_risk_ = 0.0F;
      high_risk_evaluated_ = false;
      have_previous_local_goal_ = false;
      corridor_hint_route_.clear();
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
        corridor_hint_route_ = {position_, next_goal};
        if (graph_fixed_layer_) {
          const float layer_z = static_cast<float>(
            graph_layer_initialized_ ? graph_layer_z_ : next_goal.z());
          for (auto &point : corridor_hint_route_) point.z() = layer_z;
        }
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
      "[EPIC goal] target=(%.2f,%.2f,%.2f) graph=%s reuse=%d bounds_expanded=%d "
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
        "[EPIC input] dropped cloud: points=%zu have_odom=%d",
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
        "[EPIC input] dropped cloud after voxel filtering: input=%zu leaf=%.3f m",
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
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), diagnostic_log_period_ms_,
      "[EPIC timing][cloud] decode=%.3f ms transform=%.3f ms map_update=%.3f ms "
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
            // EPIC's updateSkeleton is an in-place V_remove/V_remain/V_insert
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
              "EPIC rebuild rejected: no real Bubble topology (points=%zu bubbles=%zu nodes=%zu)",
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
          RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), diagnostic_log_period_ms_,
            "[EPIC timing][background %s] points=%zu snapshot_kdtree=%.3f ms "
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
          RCLCPP_ERROR(get_logger(), "EPIC background rebuild failed: %s", error.what());
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
    updateTopoSemanticMemory(active_topo);

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
    // EPIC publishes the rolling graph and local subgoal on every planner
    // tick.  The graph search itself is already bounded by local_graph_radius;
    // throttling this block to a multi-second period makes both the graph and
    // subgoal stale while the vehicle keeps moving.  Keep the configured
    // lookahead as a geometric distance instead of coupling it to speed and a
    // planning period (which can otherwise jump to 15--20 m at flight speed).
    const float effective_lookahead_m = static_cast<float>(
      std::max(0.0, local_goal_lookahead_m_));
    std::vector<TopoNode::Ptr> path_nodes;
    std::unordered_set<std::pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash>
      witness_path_edges;
    std::unordered_set<std::pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash>
      corridor_hint_edges;
    std::unordered_set<std::pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash>
      last_path_edges;
    const auto witness_route = reuse_previous_route_ ? scalenav_graph::forwardRouteWindow(
      accepted_route_.witness_path, position_, static_cast<float>(route_reuse_horizon_m_)) :
      std::vector<Eigen::Vector3f>();
    if (reuse_previous_route_ && witness_route.size() >= 2) {
      buildRememberedEdges(
        active_topo, witness_route, static_cast<float>(route_remap_distance_m_),
        witness_path_edges);
    }
    if (reuse_previous_route_ && corridor_hint_route_.size() >= 2) {
      buildRememberedEdges(
        active_topo, corridor_hint_route_,
        static_cast<float>(route_remap_distance_m_), corridor_hint_edges);
    }
    // Route reuse is governed by monotonic witness progress and the temporal
    // extension timer.  Lateral YOPO avoidance must not invalidate the guide.
    const bool route_aligned = reuse_previous_route_ && scalenav_graph::canReuseForwardRoute(
      position_, accepted_route_.witness_path, 0.0F,
      std::numeric_limits<float>::infinity());
    const float route_lateral_error = accepted_route_.witness_path.size() >= 2 ?
      scalenav_graph::pointPathDistance(position_, accepted_route_.witness_path) :
      std::numeric_limits<float>::infinity();
    last_path_edges.clear();
    if (reuse_previous_route_ && witness_route.size() >= 2) {
      last_path_edges.insert(witness_path_edges.begin(), witness_path_edges.end());
    }
    // Apply the mission corridor hint only when there is no active witness,
    // for example after a goal flip. YOPO is free to leave the witness
    // laterally; that deviation must not alter global route memory.
    if (reuse_previous_route_ && witness_route.size() < 2) {
      last_path_edges.insert(corridor_hint_edges.begin(), corridor_hint_edges.end());
    }
    const std::size_t geometrically_remembered_edges = last_path_edges.size() / 2;

    const auto astar_start = std::chrono::steady_clock::now();
    bool candidate_found = false;
    bool candidate_accepted = false;
    const char *route_switch_reason = "NONE";
    bool found = false;
    TopoGraphSearchStats incumbent_search_stats;
    TopoGraphSearchStats candidate_search_stats;
    const bool semantic_request_for_plan = semantic_replan_requested_;
    Eigen::Vector3f layer_goal = goal_;
    if (graph_fixed_layer_) layer_goal.z() = static_cast<float>(graph_layer_z_);
    const float vehicle_to_goal = (position_ - layer_goal).norm();
    const bool goal_in_window = have_goal_ &&
      vehicle_to_goal <= static_cast<float>(local_graph_radius_m_);
    const float accepted_route_length = scalenav_graph::routeLength(accepted_route_.witness_path);
    if (accepted_route_.valid && accepted_route_length > 1e-3F) {
      float projected_t = 0.0F;
      if (polynomial_curve_valid_ && scalenav_graph::routeProgressTAlongCurve(
            polynomial_curve_, position_,
            accepted_route_.frontier_goal_progress_t, projected_t)) {
        accepted_route_.frontier_goal_progress_t = std::max(
          accepted_route_.frontier_goal_progress_t, std::clamp(projected_t, 0.0F, 1.0F));
        accepted_route_.frontier_goal_progress_m =
          accepted_route_.frontier_goal_progress_t * accepted_route_length;
      }
    }
    const auto accepted_forward_route = scalenav_graph::forwardRouteFromT(
      accepted_route_.witness_path, accepted_route_.frontier_goal_progress_t);
    const float accepted_route_remaining =
      scalenav_graph::routeLength(accepted_forward_route);
    const bool frontier_half_consumed = accepted_route_.valid &&
      std::isfinite(accepted_route_.frontier_goal_initial_route_length_m) &&
      accepted_route_.frontier_goal_initial_route_length_m > 1e-3F &&
      accepted_route_.frontier_goal_progress_m >=
        0.5F * accepted_route_.frontier_goal_initial_route_length_m;
    const bool route_has_planning_horizon = accepted_route_.valid &&
      std::isfinite(accepted_route_.frontier_goal_initial_route_length_m) &&
      !frontier_half_consumed;
    const std::int64_t active_virtual_semantic_stamp_ns =
      activeVirtualSemanticStampNs();
    // The accepted witness is a guide for YOPO, not a second local obstacle
    // planner.  Do not invalidate it from a current-frame clearance probe;
    // only an explicit graph-level route decision can replace it.
    const float current_route_risk =
      semanticRiskAlongRoute(active_topo, accepted_route_.witness_path);
    // A blocked corridor is a hard invalidation. Semantic risk only requests
    // a candidate evaluation; it must not erase the incumbent route here.
    // Keep the accepted guide route while YOPO handles local obstacle
    // avoidance. Replan only when the current guide is exhausted halfway,
    // explicitly blocked, or missing. A goal change clears accepted_route_,
    // so it naturally takes the missing-route branch. Semantic observations
    // still affect the next A* edge costs, but do not seize control by
    // triggering a separate replanning cycle.
    // `reuse_previous_route_` remains separate: it only controls historical
    // witness/edge cost reuse and stays false in the default configuration.
    const bool accepted_witness_usable = accepted_route_.valid &&
      accepted_route_.witness_path.size() >= 2 &&
      accepted_route_.topology_path.size() >= 2;
    const std::uint64_t topology_generation = topology_update_generation_.load();
    const bool frontier_route_exhausted = accepted_route_.valid &&
      (accepted_route_remaining <= 1.0F ||
       accepted_route_.frontier_goal_progress_t >= 0.99F);
    const bool frontier_extension_needed = accepted_witness_usable &&
      (!route_has_planning_horizon || frontier_route_exhausted);
    const auto extension_clock = std::chrono::steady_clock::now();
    const double ms_since_last_extension = have_last_extension_search_ ?
      std::chrono::duration<double, std::milli>(
        extension_clock - last_extension_search_time_).count() :
      std::numeric_limits<double>::infinity();
    // Search immediately when the route first becomes exhausted, then use the
    // normal retry period while waiting for the map to expose a continuation.
    // A zero period here caused the planner to either thrash at 10 Hz or remain
    // in an ambiguous incumbent-reuse state between graph updates.
    const double extension_period_ms =
      std::max(0.0, frontier_extension_search_period_ms_);
    // Local witness half consumed: re-search for the next frontier on a timer.
    const bool frontier_horizon_expired = frontier_extension_needed &&
      ms_since_last_extension >= extension_period_ms;
    const bool route_search_allowed =
      last_route_search_generation_ != topology_generation;
    const bool need_candidate_search =
      (!accepted_witness_usable && route_search_allowed) ||
      frontier_horizon_expired;
    if (need_candidate_search) {
      last_route_search_generation_ = topology_generation;
    }
    const char *incumbent_result = accepted_witness_usable ? "ACCEPTED" : "NONE";
    bool using_accepted_route = false;
    std::vector<TopoNode::Ptr> candidate_nodes;
    if (!need_candidate_search && accepted_witness_usable) {
      // No graph decision is needed this tick. Re-emit the accepted topology
      // path so visualization and the existing publish-level witness check
      // remain consistent, without running another A* search.
      path_nodes = accepted_route_.topology_path;
      found = true;
      using_accepted_route = true;
    }
    if (need_candidate_search) {
      candidate_found = active_topo->goalDirectedSearch(
        active_topo->odom_node_, goal_, path_nodes, 0.0,
        static_cast<float>(goal_path_cost_weight_),
        reuse_previous_route_ ? static_cast<float>(previous_path_cost_factor_) : 1.0F,
        last_path_edges,
        static_cast<float>(semantic_cost_weight_),
        static_cast<float>(local_graph_radius_m_),
        &position_, 0.0F, goal_in_window,
        std::numeric_limits<float>::infinity(),
        effective_lookahead_m, nullptr, 0.0F,
        0.0F, 0.0F, 0.0F, 0.0F,
        &candidate_search_stats, active_virtual_semantic_stamp_ns);
      candidate_nodes.swap(path_nodes);
      if (candidate_found && candidate_nodes.size() < 2) {
        candidate_found = false;
        candidate_nodes.clear();
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "EPIC rejected topology search result without node path");
      }
    }
    // Compare candidate against the accepted route; only switch on blockage,
    // goal window, missing incumbent, or materially better progress/cost.
    auto route_points = [](const std::vector<TopoNode::Ptr> &nodes) {
      std::vector<Eigen::Vector3f> points;
      points.reserve(nodes.size());
      for (const auto &node : nodes) if (node) points.push_back(node->center_);
      return points;
    };
    struct RouteMetrics { float risk; float objective; float progress; };
    const float local_semantic_radius_m = static_cast<float>(
      local_graph_radius_m_ + std::max(0.0, semantic_route_influence_m_));
    std::size_t local_inactive_virtual_semantic_nodes = 0;
    const auto local_semantic_nodes = active_topo->semanticNodes(
      &position_, local_semantic_radius_m, active_virtual_semantic_stamp_ns,
      &local_inactive_virtual_semantic_nodes);
    const auto metrics_for = [&](const std::vector<TopoNode::Ptr> &nodes) {
      const auto points = route_points(nodes);
      RouteMetrics metrics{semanticRiskAlongRoute(active_topo, points), 0.0F, 0.0F};
      if (points.empty()) return metrics;
      metrics.progress = (points.back() - position_).norm();
      for (std::size_t i = 1; i < nodes.size(); ++i) {
        const auto &from = nodes[i - 1];
        const auto &to = nodes[i];
        if (!from || !to) continue;
        metrics.objective += active_topo->routeEdgeCost(
          from, to, static_cast<float>(goal_path_cost_weight_),
          static_cast<float>(semantic_cost_weight_), false, 1.0F,
          &local_semantic_nodes);
      }
      metrics.objective += static_cast<float>(goal_path_cost_weight_) *
        (points.back() - layer_goal).norm();
      return metrics;
    };
    RouteMetrics incumbent_metrics{0.0F, 0.0F, 0.0F};
    RouteMetrics candidate_metrics{0.0F, 0.0F, 0.0F};
    bool compared_route_metrics = false;
    if (using_accepted_route) {
      incumbent_metrics = metrics_for(path_nodes);
    } else if (accepted_witness_usable && accepted_route_.topology_path.size() >= 2) {
      incumbent_metrics = metrics_for(accepted_route_.topology_path);
    }
    if (candidate_found) {
      candidate_metrics = metrics_for(candidate_nodes);
      const bool hard_switch = !accepted_witness_usable;
      bool switch_route = hard_switch;
      if (!accepted_witness_usable) {
        // Distinguish the first commit from the diagnostic mode where a
        // fresh candidate is intentionally evaluated every tick.
        route_switch_reason = accepted_route_.valid ? "FRESH_SEARCH" : "INITIAL_ACCEPT";
      }
      if (!switch_route && accepted_witness_usable) {
        compared_route_metrics = true;
        switch_route = scalenav_graph::shouldSwitchRoute(
          false, incumbent_metrics.risk, candidate_metrics.risk,
          static_cast<float>(semantic_route_switch_risk_margin_),
          incumbent_metrics.objective, candidate_metrics.objective,
          candidate_metrics.progress - incumbent_metrics.progress,
          static_cast<float>(frontier_goal_margin_m_),
          static_cast<float>(semantic_route_switch_cost_ratio_));
        if (switch_route) route_switch_reason = "LOWER_LOSS";
      }
      if (!switch_route && accepted_witness_usable && frontier_extension_needed &&
          candidate_found && !candidate_nodes.empty()) {
        const TopoNode::Ptr &candidate_frontier = candidate_nodes.back();
        if (candidate_frontier &&
            candidate_frontier->persistent_id_ != accepted_route_.frontier_goal_id) {
          const float incumbent_goal_dist =
            (accepted_route_.frontier_goal - layer_goal).norm();
          const float candidate_goal_dist =
            (candidate_frontier->center_ - layer_goal).norm();
          if (candidate_goal_dist + 0.5F < incumbent_goal_dist) {
            switch_route = true;
            route_switch_reason = "FRONTIER_HALF";
          }
        }
      }
      if (!switch_route && accepted_witness_usable && frontier_extension_needed) {
        switch_route = scalenav_graph::candidateExtendsAcceptedRoute(
          accepted_forward_route, route_points(candidate_nodes), 0.25F,
          std::numeric_limits<float>::infinity());
        if (switch_route) route_switch_reason = "COMPATIBLE_EXTENSION";
      }
      if (!switch_route && semantic_request_for_plan && accepted_witness_usable) {
        switch_route = candidate_metrics.risk +
          static_cast<float>(semantic_route_switch_risk_margin_) < current_route_risk;
        if (switch_route) route_switch_reason = "SEMANTIC_RISK";
      }
      const bool compatible_extension_switch =
        switch_route && std::strcmp(route_switch_reason, "COMPATIBLE_EXTENSION") == 0;
      if (switch_route) {
        path_nodes = candidate_nodes;
        found = true;
        using_accepted_route = false;
        candidate_accepted = true;
      } else if (accepted_witness_usable && !frontier_route_exhausted) {
        const auto frontier_node = nearestPersistentNode(
          active_topo, accepted_route_.frontier_goal,
          static_cast<float>(route_remap_distance_m_), accepted_route_.frontier_goal_id);
        if (frontier_node && active_topo->graphSearch(
              active_topo->odom_node_, frontier_node, path_nodes, 0.05, true,
              last_path_edges, static_cast<float>(semantic_cost_weight_),
              static_cast<float>(local_graph_radius_m_), nullptr,
              active_virtual_semantic_stamp_ns,
              reuse_previous_route_ ? static_cast<float>(previous_path_cost_factor_) : 1.0F,
              static_cast<float>(goal_path_cost_weight_)) &&
            path_nodes.size() >= 2) {
          found = true;
          using_accepted_route = true;
          incumbent_result = "ACCEPTED";
        } else if (accepted_route_.topology_path.size() >= 2) {
          path_nodes = accepted_route_.topology_path;
          found = true;
          using_accepted_route = true;
          incumbent_result = "ACCEPTED";
        }
      }
    }
    if (frontier_horizon_expired) {
      last_extension_search_time_ = extension_clock;
      have_last_extension_search_ = true;
      if (!candidate_accepted) {
        const std::uint64_t candidate_frontier_id = candidate_found && !candidate_nodes.empty() &&
          candidate_nodes.back() ? candidate_nodes.back()->persistent_id_ : 0;
        if (!candidate_found) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
          "[EPIC frontier extension] route horizon exhausted but no candidate topology; "
          "frontier_goal_id=%llu progress_t=%.3f remaining=%.2f m retry_in=%.0f ms",
            static_cast<unsigned long long>(accepted_route_.frontier_goal_id),
            static_cast<double>(accepted_route_.frontier_goal_progress_t),
            static_cast<double>(accepted_route_remaining),
            extension_period_ms);
        } else if (candidate_frontier_id == accepted_route_.frontier_goal_id) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[EPIC frontier extension] half consumed but search returned same "
            "frontier_goal_id=%llu; retry_in=%.0f ms",
            static_cast<unsigned long long>(candidate_frontier_id),
            extension_period_ms);
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[EPIC frontier extension] half consumed but candidate "
            "frontier_goal_id=%llu rejected (incumbent=%llu) retry_in=%.0f ms",
            static_cast<unsigned long long>(candidate_frontier_id),
            static_cast<unsigned long long>(accepted_route_.frontier_goal_id),
            extension_period_ms);
        }
      }
    }
    if (found && !using_accepted_route &&
        (path_nodes.size() < 2 || !active_topo->odom_node_ ||
         active_topo->odom_node_->neighbors_.empty())) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "EPIC rejected route without a connected topology head: path_nodes=%zu odom_degree=%zu",
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
    std::vector<Eigen::Vector3f> frontier_goal_extension;
    if (found && !path_nodes.empty()) {
      connectFrontierGoalToMissionGoal(active_topo, path_nodes.back(), frontier_goal_extension);
    }
    astar_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - astar_start).count();
    // Keep the candidate frontier_goal local until publish() has accepted the
    // complete witness.  A topology path is not an incumbent by itself.
    const Eigen::Vector3f proposed_frontier_goal =
      (found && !path_nodes.empty()) ?
      path_nodes.back()->center_ :
      accepted_route_.frontier_goal;
    const std::uint64_t proposed_frontier_goal_id =
      (found && !path_nodes.empty()) ? path_nodes.back()->persistent_id_ :
      accepted_route_.frontier_goal_id;
    const bool proposed_have_frontier_goal = found && !path_nodes.empty();
    // A semantic search attempt is itself an evaluation. If it finds no
    // candidate while the incumbent remains valid, latch the observed risk
    // and wait for a further accumulated increase instead of retrying at 10 Hz.
    const bool semantic_evaluation_attempted =
      semantic_request_for_plan && need_candidate_search && accepted_route_.valid;
    const bool route_evaluation_completed = candidate_found ||
      semantic_evaluation_attempted || !have_evaluated_route_risk_;
    const std::size_t route_memory_points = accepted_route_.witness_path.size();
    const auto publish_start = std::chrono::steady_clock::now();
    const float publish_progress_t = using_accepted_route ?
      accepted_route_.frontier_goal_progress_t : 0.0F;
    const auto stats = publish(
      active_topo, path_nodes, frontier_goal_extension, found,
      effective_lookahead_m, using_accepted_route ? &accepted_forward_route : nullptr,
      publish_progress_t, candidate_accepted);
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
      frontier_goal_extension.clear();
      if (using_accepted_route) {
        accepted_route_.clear();
        polynomial_guide_path_.clear();
        polynomial_curve_ = scalenav_graph::WitnessParametricCurve();
        polynomial_curve_valid_ = false;
        have_last_extension_search_ = false;
        if (using_accepted_route) {
          last_route_search_generation_ = std::numeric_limits<std::uint64_t>::max();
        }
      }
      if (!using_accepted_route) {
        // The candidate curve was provisional until its witness passed the
        // final collision check.
        polynomial_guide_path_.clear();
        polynomial_curve_ = scalenav_graph::WitnessParametricCurve();
        polynomial_curve_valid_ = false;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[EPIC route rejected] final published witness failed collision check; "
        "frontier goal state cleared for fresh search");
    }
    if (found && stats.witness_collision_free && !using_accepted_route) {
      // Commit route state only after the exact witness that will be executed
      // has passed the publish-level collision check.
      const std::uint64_t old_frontier_goal_id = accepted_route_.frontier_goal_id;
      accepted_route_.topology_path = path_nodes;
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
        "[EPIC route switch] reason=%s old_frontier_goal_id=%llu new_frontier_goal_id=%llu "
        "route_aligned=%d route_lateral_error=%.2f m compatible_extension=%d "
        "route_length=%.2f route_remaining=%.2f",
        route_switch_reason,
        static_cast<unsigned long long>(old_frontier_goal_id),
        static_cast<unsigned long long>(accepted_route_.frontier_goal_id),
        static_cast<int>(route_aligned), static_cast<double>(route_lateral_error),
        static_cast<int>(std::strcmp(route_switch_reason, "COMPATIBLE_EXTENSION") == 0),
        static_cast<double>(scalenav_graph::routeLength(stats.witness_path)),
        static_cast<double>(scalenav_graph::routeLength(stats.witness_path)));
      if (frontier_goal_changed) {
        have_last_extension_search_ = false;
        last_route_search_generation_ = std::numeric_limits<std::uint64_t>::max();
      }
      if (candidate_accepted) {
        // A newly committed route gets one fresh chance to react if the next
        // current-frame probe immediately discovers a blockage.
        last_route_search_generation_ = std::numeric_limits<std::uint64_t>::max();
      }
    }
    if (found && route_evaluation_completed && accepted_route_.witness_path.size() >= 2) {
      evaluated_route_risk_ = semanticRiskAlongRoute(active_topo, accepted_route_.witness_path);
      have_evaluated_route_risk_ = true;
      high_risk_evaluated_ = evaluated_route_risk_ >=
        static_cast<float>(semantic_route_high_risk_);
      semantic_replan_requested_ = false;
    }
    if (!found) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "EPIC rolling route has no reachable real Bubble topology: "
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
    const bool waiting_for_frontier_extension = accepted_route_.valid &&
      frontier_route_exhausted && !found && !candidate_accepted;
    const char *route_decision = candidate_accepted && found && stats.witness_collision_free ?
      "CANDIDATE_COMMITTED" :
      (using_accepted_route && found && stats.witness_collision_free ? "ACCEPTED_ROUTE_REUSED" :
      (candidate_found && !stats.witness_collision_free ? "CANDIDATE_WITNESS_REJECTED" :
      (candidate_found ? "CANDIDATE_REJECTED" :
      (waiting_for_frontier_extension ? "WAITING_FOR_FRONTIER" : "NO_CANDIDATE"))));
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
      "[EPIC timing][update] rebuild_running=%d odom_connect=%.3f ms "
      "astar=%.3f ms publish=%.3f ms total=%.3f ms cloud=%zu skeleton_updates=%zu "
      "bubbles=%zu nodes=%zu edges=%zu path_nodes=%zu witness_points=%zu->%zu "
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
      "semantic_request=%d incumbent=%s frontier_goal_id=%llu candidate_found=%d candidate_accepted=%d "
      "switch_reason=%s route_decision=%s "
      "route_compare=%d incumbent_loss=%.2f candidate_loss=%.2f "
      "incumbent_risk=%.3f candidate_risk=%.3f "
      "incumbent_progress=%.2f candidate_progress=%.2f "
      "semantic_nodes=%zu virtual_semantic_nodes=%zu semantic_path_nodes=%zu semantic_max=%.3f "
      "path_edge_semantic_risks=%s path_edge_risk_min=%.3f path_edge_risk_mean=%.3f "
      "path_edge_risk_max=%.3f "
      "path_cost=%.2f geometry=%.2f semantic=%.2f clearance=%.2f "
      "local_graph_radius=%.1f m "
      "route_aligned=%d route_lateral_error=%.2f m "
      "horizon_ready=%d frontier_half_replan=%d "
      "route_length=%.2f route_remaining=%.2f "
      "frontier_initial_route_length=%.2f frontier_progress=%.2f frontier_progress_t=%.4f "
      "route_risk=%.3f "
      "frontier_goal=(%.2f,%.2f,%.2f) "
      "frontier_goal_distance=%.2f m "
      "frontier_goal_extension=%zu found=%d",
      static_cast<int>(rebuild_running_.load()), odom_ms, astar_ms, publish_ms, ms,
      cloud_count_, skeleton_update_count_.load(), stats.bubbles,
      stats.skeleton_nodes, stats.edges, path_nodes.size(), stats.witness_points_raw,
      stats.witness_points, route_memory_points, persistent_semantic_records,
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
      use_edge_witness_path_ ? "EDGE_WITNESS" : "TOPO_CENTERS",
      reused_path_edges,
      path_nodes.size() > 1 ? path_nodes.size() - 1 : 0,
      geometrically_remembered_edges,
      "EXTEND",
      static_cast<int>(semantic_request_for_plan), incumbent_result,
      static_cast<unsigned long long>(accepted_route_.frontier_goal_id),
      static_cast<int>(candidate_found),
      static_cast<int>(candidate_accepted),
      route_switch_reason,
      route_decision,
      static_cast<int>(compared_route_metrics),
      static_cast<double>(incumbent_metrics.objective),
      static_cast<double>(candidate_metrics.objective),
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
      static_cast<int>(frontier_half_consumed),
      static_cast<double>(accepted_route_length),
      static_cast<double>(accepted_route_remaining),
      static_cast<double>(accepted_route_.frontier_goal_initial_route_length_m),
      static_cast<double>(accepted_route_.frontier_goal_progress_m),
      static_cast<double>(accepted_route_.frontier_goal_progress_t),
      static_cast<double>(current_route_risk),
      accepted_route_.frontier_goal.x(), accepted_route_.frontier_goal.y(), accepted_route_.frontier_goal.z(),
      (accepted_route_.frontier_goal - layer_goal).norm(),
      frontier_goal_extension.size(), static_cast<int>(found));
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
      "[EPIC odom diagnosis] pos=(%.2f,%.2f,%.2f) clearance=%.2f candidates=%zu "
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

  std::int64_t activeVirtualSemanticStampNs()
  {
    if (last_semantic_applied_stamp_ns_ <= 0) return -1;
    const double age_ms = static_cast<double>(std::llabs(
      get_clock()->now().nanoseconds() - last_semantic_applied_stamp_ns_)) / 1.0e6;
    return age_ms <= semantic_max_age_ms_ ? last_semantic_applied_stamp_ns_ : -1;
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
    const auto semantic_nodes = topo->semanticNodes(
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

  void updateTopoSemanticMemory(const TopoGraph::Ptr &topo)
  {
    if (!topo) return;
    std::optional<SemanticFrame> frame;
    {
      std::lock_guard<std::mutex> lock(semantic_mutex_);
      if (!semantic_frame_ ||
          (semantic_applied_topo_ == topo &&
           semantic_frame_->stamp_ns == last_semantic_applied_stamp_ns_)) {
        return;
      }
      const double age_ms = static_cast<double>(std::llabs(
        get_clock()->now().nanoseconds() - semantic_frame_->stamp_ns)) / 1.0e6;
      if (age_ms > semantic_max_age_ms_) return;
      frame = semantic_frame_;
    }

    const float route_risk_before = semanticRiskAlongRoute(topo, accepted_route_.witness_path);
    std::size_t semantic_nodes_updated = 0;
    std::size_t semantic_candidates = 0;
    std::size_t semantic_connected_nodes = 0;
    float semantic_min_range_m = std::numeric_limits<float>::infinity();
    float semantic_max_range_m = 0.0F;
    if (semantic_points_enabled_ && frame->points_world.size() == frame->scores.size() &&
        frame->points_world.size() == frame->confidences.size()) {
      struct Candidate {
        Eigen::Vector3f point;
        float score;
        float confidence;
      };
      std::vector<Candidate> rays;
      rays.reserve(frame->points_world.size());
      for (std::size_t i = 0; i < frame->points_world.size(); ++i) {
        rays.push_back(
          {frame->points_world[i], frame->scores[i], frame->confidences[i]});
      }
      std::sort(rays.begin(), rays.end(),
        [](const Candidate &left, const Candidate &right) {
          return left.score > right.score;
        });
      std::vector<Eigen::Vector3f> semantic_centers;
      std::vector<float> semantic_scores;
      std::vector<float> semantic_confidences;
      const Eigen::Vector3f origin = frame->origin;
      for (const auto &ray : rays) {
        if (static_cast<int>(semantic_centers.size()) >=
            std::max(0, semantic_point_max_nodes_)) break;
        const Eigen::Vector3f chosen = ray.point;
        const float distance = (chosen - origin).norm();
        if (!std::isfinite(distance) || distance < 1.0F) continue;
        if (!topo->lidar_map_interface_->IsInBox(chosen)) continue;
        bool duplicate = false;
        for (const auto &existing : semantic_centers) {
          if ((existing - chosen).norm() <
              static_cast<float>(std::max(1.0, semantic_point_separation_m_))) {
            duplicate = true;
            break;
          }
        }
        if (!duplicate) {
          semantic_centers.push_back(chosen);
          semantic_scores.push_back(std::clamp(ray.score, 0.0F, 1.0F));
          semantic_confidences.push_back(std::clamp(ray.confidence, 0.0F, 1.0F));
          const float range = (chosen - origin).norm();
          semantic_min_range_m = std::min(semantic_min_range_m, range);
          semantic_max_range_m = std::max(semantic_max_range_m, range);
        }
      }
      semantic_nodes_updated = topo->insertSemanticNodes(
        semantic_centers, semantic_scores,
        static_cast<float>(std::max(0.45, semantic_point_radius_m_)),
        origin, frame->stamp_ns, semantic_confidences);
      semantic_candidates = semantic_centers.size();
      for (const auto &node : topo->semanticNodes()) {
        if (node && !node->neighbors_.empty()) ++semantic_connected_nodes;
      }
    }
    mergeSemanticMemory(topo->semanticMemorySnapshot());
    // Unknown fixed-depth endpoints are transient planning evidence. Mark the
    // newly applied frame active before evaluating its route impact; older
    // virtual nodes remain persisted but no longer contribute to route cost.
    last_semantic_applied_stamp_ns_ = frame->stamp_ns;
    // Semantic costs can change the preferred route without changing the
    // geometric graph. Request a candidate evaluation, but keep the accepted
    // frontier_goal and witness alive until that candidate is compared.
    const float route_risk_after = semanticRiskAlongRoute(topo, accepted_route_.witness_path);
    const bool has_route_memory = accepted_route_.witness_path.size() >= 2;
    const float trigger_delta = static_cast<float>(std::max(0.0, semantic_route_replan_delta_));
    const bool frame_delta_trigger = scalenav_graph::semanticRiskIncreaseRequiresReplan(
      route_risk_before, route_risk_after, trigger_delta);
    const bool accumulated_delta_trigger = have_evaluated_route_risk_ &&
      route_risk_after >= evaluated_route_risk_ + trigger_delta;
    if (route_risk_after <= static_cast<float>(semantic_route_high_risk_release_)) {
      high_risk_evaluated_ = false;
    }
    const bool high_risk_trigger = route_risk_after >=
      static_cast<float>(semantic_route_high_risk_) && !high_risk_evaluated_;
    const bool semantic_reset_requested = semantic_route_replan_enabled_ &&
      has_route_memory && (frame_delta_trigger || accumulated_delta_trigger || high_risk_trigger);
    semantic_replan_requested_ = semantic_replan_requested_ || semantic_reset_requested;
    if (semantic_nodes_updated > 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), diagnostic_log_period_ms_,
        "[EPIC semantic route] node_updates=%zu candidates=%zu guide_points=%zu "
        "risk=%.3f->%.3f delta=%.3f reference=%.3f request=%d high_latched=%d",
        semantic_nodes_updated, semantic_candidates, accepted_route_.witness_path.size(),
        route_risk_before, route_risk_after,
        std::abs(route_risk_after - route_risk_before),
        evaluated_route_risk_, static_cast<int>(semantic_replan_requested_),
        static_cast<int>(high_risk_evaluated_));
    }
    if (semantic_nodes_updated > 0) {
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), diagnostic_log_period_ms_,
        "[EPIC semantic graph] virtual_depth=%.2f m candidates=%zu "
        "inserted_or_updated=%zu connected=%zu range=%.2f..%.2f m "
        "mode=REPULSION route_risk=%.3f->%.3f replan_requested=%d",
        semantic_virtual_depth_m_, semantic_candidates, semantic_nodes_updated,
        semantic_connected_nodes, semantic_min_range_m,
        semantic_max_range_m, route_risk_before, route_risk_after,
        static_cast<int>(semantic_replan_requested_));
    }
    // A detached skeleton swap creates a new persistent graph. Reapply the
    // latest frame once to that graph even when its timestamp was already
    // consumed by the previous graph.
    semantic_applied_topo_ = topo;
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
                     float terminal_progress_t = -1.0F,
                     const scalenav_graph::WitnessParametricCurve *curve = nullptr) const
  {
    Eigen::Vector3f layer_goal = goal_;
    if (graph_fixed_layer_) layer_goal.z() = static_cast<float>(graph_layer_z_);

    // A consumed route can collapse to a single/empty witness while the
    // vehicle is already at the mission endpoint.  In that terminal state,
    // bypass witness validation and keep publishing the mission goal so the
    // controller can finish instead of receiving a stop at its current pose.
    const float terminal_t = terminal_progress_t >= 0.0F ? terminal_progress_t :
      minimum_progress_t;
    const bool terminal_goal = have_goal_ && terminal_t >= 0.99F &&
      (position_ - layer_goal).norm() <= static_cast<float>(goal_connect_distance_m_);
    if (terminal_goal) {
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

  void refitPolynomialGuideFromWitness(const std::vector<Eigen::Vector3f> &witness)
  {
    if (witness.size() < 2) {
      polynomial_guide_path_.clear();
      polynomial_curve_ = scalenav_graph::WitnessParametricCurve();
      polynomial_curve_valid_ = false;
      return;
    }
    const float layer_z = graph_fixed_layer_ ? static_cast<float>(graph_layer_z_) :
      std::numeric_limits<float>::quiet_NaN();
    polynomial_guide_path_ = scalenav_graph::buildPolynomialGuidePath(
      witness, position_, world_velocity_, layer_z);
    if (polynomial_guide_path_.size() < 2) {
      polynomial_guide_path_.clear();
      polynomial_curve_ = scalenav_graph::WitnessParametricCurve();
      polynomial_curve_valid_ = false;
      return;
    }
    polynomial_curve_ = scalenav_graph::WitnessParametricCurve::fit(polynomial_guide_path_);
    polynomial_curve_valid_ = polynomial_curve_.valid;
  }

  PublishStats publish(const TopoGraph::Ptr &topo,
                       const std::vector<TopoNode::Ptr> &path_nodes,
                       const std::vector<Eigen::Vector3f> &frontier_goal_extension,
                       bool found, float effective_lookahead_m,
                       const std::vector<Eigen::Vector3f> *accepted_witness_override = nullptr,
                       float minimum_route_progress_t = 0.0F,
                       bool replan_polynomial = false)
  {
    PublishStats stats;
    visualization_msgs::msg::MarkerArray graph;
    visualization_msgs::msg::Marker skeleton_nodes;
    skeleton_nodes.header.frame_id = visualization_frame_;
    skeleton_nodes.header.stamp = now();
    skeleton_nodes.ns = "epic_skeleton_nodes";
    skeleton_nodes.id = 0;
    skeleton_nodes.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    skeleton_nodes.action = visualization_msgs::msg::Marker::ADD;
    skeleton_nodes.scale.x = 0.32;
    skeleton_nodes.scale.y = 0.32;
    skeleton_nodes.scale.z = 0.32;
    setColor(skeleton_nodes.color, kTopology);

    visualization_msgs::msg::Marker semantic_nodes_marker;
    semantic_nodes_marker.header = skeleton_nodes.header;
    semantic_nodes_marker.ns = "epic_semantic_points";
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

    visualization_msgs::msg::Marker edges_marker = skeleton_nodes;
    edges_marker.ns = "epic_skeleton_edges";
    edges_marker.id = 2;
    edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges_marker.scale.x = 0.045;
    setColor(edges_marker.color, kTopology, 0.72F);
    edges_marker.points.clear();

    visualization_msgs::msg::Marker witness_edges = edges_marker;
    witness_edges.ns = "epic_edge_witness_paths";
    witness_edges.id = 3;
    witness_edges.scale.x = 0.025;
    setColor(witness_edges.color, kCandidate, 0.55F);
    witness_edges.points.clear();

    std::unordered_set<TopoNode::Ptr> visited;
    for (const auto &entry : topo->reg_map_idx2ptr_) {
      if (!entry.second) continue;
      for (const auto &node : entry.second->topo_nodes_) {
        if (!node || node->is_viewpoint_ || !visited.insert(node).second) continue;
        if (node->role_ == TopoNodeRole::Odom) continue;
        const bool associated = node->semantic_observations_ > 0 &&
          std::isfinite(node->semantic_score_);
        const bool virtual_semantic_point = associated &&
          node->geometry_state_ == TopoGeometryState::Unknown;
        skeleton_nodes.points.push_back(toPoint(node->center_));
        semantic_scores.push_back(node->semantic_score_);
        semantic_associated.push_back(associated);
        ++stats.skeleton_nodes;
        if (virtual_semantic_point) {
          semantic_nodes_marker.points.push_back(toPoint(node->center_));
          visualization_msgs::msg::Marker label;
          label.header = skeleton_nodes.header;
          label.ns = "epic_semantic_point_labels";
          label.id = static_cast<int>(semantic_labels.markers.size());
          label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
          label.action = visualization_msgs::msg::Marker::ADD;
          const Eigen::Vector3f label_position =
            node->center_ + Eigen::Vector3f(0.0F, 0.0F, 0.45F);
          label.pose.position = toPoint(label_position);
          label.scale.z = 0.42;
          const bool risk_anchor = isSemanticRiskAnchor(
            node->semantic_score_, node->semantic_confidence_,
            static_cast<float>(semantic_point_min_score_));
          const Rgb marker_rgb = risk_anchor ? kRiskHigh : kCandidate;
          std_msgs::msg::ColorRGBA marker_color;
          setColor(marker_color, marker_rgb);
          semantic_nodes_marker.colors.push_back(marker_color);
          setColor(label.color, marker_rgb);
          // A low-score virtual endpoint is not verified free space; it
          // is an unknown optimistic branch. Keep that distinction explicit
          // in RViz instead of implying that every non-risk candidate is safe.
          label.text = risk_anchor ? "SEM-RISK" : "SEM-UNKNOWN";
          semantic_labels.markers.push_back(std::move(label));
          ++stats.virtual_semantic_nodes;
        } else if (associated) {
          ++stats.semantic_nodes;
        }
        if (associated) {
          stats.semantic_max = std::max(stats.semantic_max, node->semantic_score_);
        }
        for (const auto &neighbor : node->neighbors_) {
          if (!neighbor) continue;
          if (!std::less<const TopoNode *>{}(node.get(), neighbor.get())) continue;
          stats.edges++;
          edges_marker.points.push_back(toPoint(node->center_));
          edges_marker.points.push_back(toPoint(neighbor->center_));
          if (use_edge_witness_path_) {
            const auto path_it = node->paths_.find(neighbor);
            if (path_it == node->paths_.end()) continue;
            const auto &witness = path_it->second;
            for (std::size_t i = 1; i < witness.size(); ++i) {
              witness_edges.points.push_back(toPoint(witness[i - 1]));
              witness_edges.points.push_back(toPoint(witness[i]));
            }
          }
        }
      }
    }
    for (std::size_t i = 0; i < semantic_scores.size(); ++i) {
      const float normalized = semantic_scores[i] / static_cast<float>(
        std::max(semantic_visualization_max_score_, 1e-5));
      skeleton_nodes.colors.push_back(semanticColor(normalized, semantic_associated[i]));
    }
    graph.markers.push_back(skeleton_nodes);
    graph.markers.push_back(semantic_nodes_marker);
    for (auto &label : semantic_labels.markers) graph.markers.push_back(std::move(label));
    graph.markers.push_back(edges_marker);
    graph.markers.push_back(witness_edges);

    visualization_msgs::msg::Marker path_marker = edges_marker;
    path_marker.ns = "epic_astar_topology_path";
    path_marker.id = 4;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.points.clear();
    path_marker.scale.x = 0.10;
    setColor(path_marker.color, kSelectedPath, found ? 0.55F : 0.20F);
    for (const auto &node : path_nodes) {
      if (!node) continue;
      path_marker.points.push_back(toPoint(node->center_));
      if (node->semantic_observations_ > 0) ++stats.semantic_path_nodes;
    }
    const auto local_semantic_nodes = topo->semanticNodes(
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
    std::vector<Eigen::Vector3f> selected_witness_path;
    bool witness_rejected = false;
    // An accepted route is already a complete, collision-checked witness.
    // Do not reconstruct it from topology edges that may have been replaced
    // by an incremental graph update; missing/stale edge caches must not
    // invalidate the executable route.
    if (accepted_witness_override && !accepted_witness_override->empty()) {
      selected_witness_path = *accepted_witness_override;
    } else if (!use_edge_witness_path_) {
      for (const auto &node : path_nodes) {
        if (!node) continue;
        const auto &point = node->center_;
        if (selected_witness_path.empty() ||
            (selected_witness_path.back() - point).norm() > 1e-3F) {
          selected_witness_path.push_back(point);
        }
      }
    } else {
      for (std::size_t i = 1; i < path_nodes.size(); ++i) {
        const auto &from = path_nodes[i - 1];
        const auto &to = path_nodes[i];
        if (!from || !to) continue;
        std::vector<Eigen::Vector3f> edge_path;
        const auto path_it = from->paths_.find(to);
        if (path_it == from->paths_.end() || path_it->second.size() < 2) {
          // A missing witness is not an invitation to use the straight chord:
          // the chord may cross an obstacle while both endpoint bubbles are
          // free. Reject the route and force graph recovery/replanning.
          stats.witness_collision_free = false;
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "[EPIC route rejected] selected edge has no valid witness: "
            "from=%llu to=%llu",
            static_cast<unsigned long long>(from->persistent_id_),
            static_cast<unsigned long long>(to->persistent_id_));
          witness_rejected = true;
          found = false;
          continue;
        }
        edge_path = path_it->second;
        if ((edge_path.front() - from->center_).norm() >
            (edge_path.back() - from->center_).norm()) {
          std::reverse(edge_path.begin(), edge_path.end());
        }
        // A TopoNode can survive an incremental graph diff while its cached
        // witness is replaced asynchronously.  Never publish that stale
        // endpoint: the selected path must start/end at the actual topology
        // nodes used by A*.
        if (edge_path.empty()) edge_path.push_back(from->center_);
        edge_path.front() = from->center_;
        if (edge_path.size() == 1) {
          edge_path.push_back(to->center_);
        } else {
          edge_path.back() = to->center_;
        }
        for (const auto &point : edge_path) {
          if (selected_witness_path.empty() ||
              (selected_witness_path.back() - point).norm() > 1e-3F) {
            selected_witness_path.push_back(point);
          }
        }
      }
    }
    if (graph_fixed_layer_) {
      const float layer_z = static_cast<float>(graph_layer_z_);
      for (auto &point : selected_witness_path) point.z() = layer_z;
    }
    stats.witness_points_raw = selected_witness_path.size();
    const bool route_already_reaches_extension =
      !frontier_goal_extension.empty() && !selected_witness_path.empty() &&
      (selected_witness_path.back() - frontier_goal_extension.back()).norm() <= 1e-3F;
    if (!route_already_reaches_extension) {
      for (const auto &point : frontier_goal_extension) {
        if (selected_witness_path.empty() ||
            (selected_witness_path.back() - point).norm() > 1e-3F) {
          selected_witness_path.push_back(point);
        }
      }
    }
    stats.witness_points = selected_witness_path.size();
    if (found && selected_witness_path.size() >= 2 && topo->parallel_bubble_astar_ &&
        accepted_witness_override == nullptr) {
      std::vector<Eigen::Vector3f> checked_witness = selected_witness_path;
      ParallelBubbleAstar::CollisionCheckInfo witness_info;
      stats.witness_collision_free =
        topo->parallel_bubble_astar_->collisionCheck_shortenPath(
          checked_witness, &witness_info);
      if (!stats.witness_collision_free) {
        const char *reason = witness_info.reason ==
            ParallelBubbleAstar::CollisionCheckInfo::CLEARANCE ? "CLEARANCE" :
          witness_info.reason == ParallelBubbleAstar::CollisionCheckInfo::BUBBLE_OVERLAP ?
            "BUBBLE_OVERLAP" : "INVALID_PATH";
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "[EPIC candidate rejected] reason=%s witness_index=%zu/%zu point=(%.2f,%.2f,%.2f) "
          "clearance=%.3f radius=%.3f predecessor=%zu distance=%.3f predecessor_radius=%.3f",
          reason, witness_info.failed_index, selected_witness_path.size(),
          witness_info.failed_point.x(), witness_info.failed_point.y(),
          witness_info.failed_point.z(), witness_info.clearance, witness_info.radius,
          witness_info.predecessor_index, witness_info.predecessor_distance,
          witness_info.predecessor_radius);
        selected_witness_path.clear();
        stats.witness_points = 0;
        witness_rejected = true;
        found = false;
      }
    }
    // Add the topology path only after witness validation so a rejected route
    // is visibly diagnostic rather than styled like an executable route.
    setColor(path_marker.color, kSelectedPath, found ? 0.55F : 0.20F);
    graph.markers.push_back(path_marker);
    if (found && selected_witness_path.size() >= 2) {
      // Return the exact executable witness to update(); it becomes accepted
      // state only after this function has completed all publication checks.
      stats.witness_path = selected_witness_path;
    }

    visualization_msgs::msg::Marker selected_witness = path_marker;
    selected_witness.ns = "epic_selected_witness_path";
    selected_witness.id = 5;
    selected_witness.scale.x = 0.14;
    setColor(selected_witness.color, kSelectedPath, found ? 1.0F : 0.25F);
    selected_witness.points.clear();
    for (const auto &point : selected_witness_path) {
      selected_witness.points.push_back(toPoint(point));
    }
    graph.markers.push_back(selected_witness);

    // Visualize the same cubic (or lower-order) fit used for monotonic
    // progress and lookahead sampling.  Keep it separate from the executable
    // witness so RViz makes fit error and endpoint overshoot visible.
    visualization_msgs::msg::Marker polynomial_witness = path_marker;
    polynomial_witness.ns = "epic_polynomial_witness_path";
    polynomial_witness.id = 13;
    polynomial_witness.type = visualization_msgs::msg::Marker::LINE_STRIP;
    polynomial_witness.scale.x = 0.075;
    polynomial_witness.points.clear();
    setColor(polynomial_witness.color, kPolynomialPath, found ? 0.95F : 0.25F);
    // Fit once on route replan; reuse ticks only advance latched progress_t.
    if (replan_polynomial && found && selected_witness_path.size() >= 2) {
      refitPolynomialGuideFromWitness(selected_witness_path);
      stats.polynomial_guide = polynomial_guide_path_;
      stats.polynomial_curve = polynomial_curve_;
    }
    const auto *active_curve = polynomial_curve_valid_ ? &polynomial_curve_ : nullptr;
    const std::vector<Eigen::Vector3f> &local_guide_path =
      polynomial_guide_path_.empty() ? selected_witness_path : polynomial_guide_path_;
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
      local_guide_progress_t, minimum_route_progress_t, active_curve);
    if (found && !selected_witness_path.empty() && !computed_has_next_goal) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "EPIC found a topology route but rejected its local goal; "
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
    next_goal_marker.ns = "epic_local_goal";
    next_goal_marker.id = 6;
    next_goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
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
      const Eigen::Vector3f topology_frontier_goal = found && !path_nodes.empty() ?
        path_nodes.back()->center_ : accepted_route_.frontier_goal;
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), diagnostic_log_period_ms_,
        "[EPIC goals] vehicle=(%.2f,%.2f,%.2f) mission_goal=(%.2f,%.2f,%.2f) "
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
    vehicle_marker.ns = "epic_vehicle_pose";
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
    graph.markers.push_back(vehicle_marker);

    visualization_msgs::msg::Marker goal_marker = skeleton_nodes;
    goal_marker.ns = "epic_global_goal";
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
    graph.markers.push_back(goal_marker);

    // The rolling A* frontier goal is distinct from both the mission goal and
    // the short local execution goal. Make all three visible in RViz.
    visualization_msgs::msg::Marker frontier_goal_marker = goal_marker;
    frontier_goal_marker.ns = "epic_frontier_goal";
    frontier_goal_marker.id = 10;
    frontier_goal_marker.action = accepted_route_.valid ?
      visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    frontier_goal_marker.scale.x = 0.62;
    frontier_goal_marker.scale.y = 0.62;
    frontier_goal_marker.scale.z = 0.62;
    setColor(frontier_goal_marker.color, kFrontierGoal);
    frontier_goal_marker.pose.position = toPoint(accepted_route_.frontier_goal);
    graph.markers.push_back(frontier_goal_marker);

    visualization_msgs::msg::Marker frontier_goal_label = goal_marker;
    frontier_goal_label.ns = "epic_frontier_goal_label";
    frontier_goal_label.id = 11;
    frontier_goal_label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    frontier_goal_label.action = accepted_route_.valid ?
      visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    const Eigen::Vector3f frontier_goal_label_position =
      accepted_route_.frontier_goal + Eigen::Vector3f(0.0F, 0.0F, 0.8F);
    frontier_goal_label.pose.position = toPoint(frontier_goal_label_position);
    setColor(frontier_goal_label.color, kFrontierGoal);
    frontier_goal_label.text = "FRONTIER GOAL";
    frontier_goal_label.scale.z = 0.45;
    graph.markers.push_back(frontier_goal_label);

    visualization_msgs::msg::Marker subgoal_label = goal_marker;
    subgoal_label.ns = "epic_local_goal_label";
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
    vehicle_label.ns = "epic_vehicle_label";
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
    goal_label.ns = "epic_goal_label";
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
    delete_bubbles.ns = "epic_real_bubbles";
    delete_bubbles.id = 0;
    delete_bubbles.action = visualization_msgs::msg::Marker::DELETEALL;
    bubbles.markers.push_back(delete_bubbles);
    const auto bubble_snapshot = topo->getBubbleSnapshot();
    stats.bubbles = bubble_snapshot.size();
    visualization_msgs::msg::Marker bubble_list;
    bubble_list.header = skeleton_nodes.header;
    bubble_list.ns = "epic_real_bubbles";
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
    bubble_pub_->publish(bubbles);

    nav_msgs::msg::Path path;
    path.header = skeleton_nodes.header;
    for (const auto &point : selected_witness_path) {
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
    if (!selected_witness_path.empty()) {
      add_clearance_sample(selected_witness_path.front());
      constexpr float sample_step = 0.25F;
      for (std::size_t i = 1; i < selected_witness_path.size(); ++i) {
        const Eigen::Vector3f segment = selected_witness_path[i] - selected_witness_path[i - 1];
        const float length = segment.norm();
        if (!std::isfinite(length)) continue;
        const int steps = std::max(1, static_cast<int>(std::ceil(length / sample_step)));
        for (int step = 1; step <= steps; ++step) {
          add_clearance_sample(
            selected_witness_path[i - 1] + segment * (static_cast<float>(step) / steps));
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
    next_goal_frame_, clearance_topic_;
  std::string visualization_frame_;
  std::string odom_twist_frame_ = "world";
  std::string flight_statistics_file_ = "epic_flight_statistics.csv";
  std::string graph_log_file_ = "epic_graph_snapshots.jsonl";
  double trajectory_speed_color_max_mps_ = 8.0;
  std::size_t trajectory_max_points_ = 50000;
  double map_margin_ = 20.0;
  bool graph_fixed_layer_ = true;
  bool reuse_graph_on_goal_ = true;
  bool reuse_previous_route_ = false;
  bool graph_layer_initialized_ = false;
  double graph_layer_z_ = 1.6;
  double map_voxel_size_ = 0.1;
  double map_history_radius_m_ = 0.0;
  int map_max_points_ = 20000;
  double map_prune_distance_m_ = 0.5;
  double skeleton_rebuild_period_ms_ = 200.0;
  int diagnostic_log_period_ms_ = 2000;
  double local_goal_min_advance_m_ = 0.75;
  double local_goal_lookahead_m_ = 10.0;
  int route_plan_period_ms_ = 100;  // launch compatibility; see update()
  double local_goal_reserve_m_ = 0.0;  // launch compatibility; see update()
  double local_graph_radius_m_ = 35.0;
  double frontier_goal_margin_m_ = 3.5;
  double frontier_progress_loss_weight_ = 0.5;
  double frontier_direction_loss_weight_ = 0.35;
  double frontier_fov_loss_weight_ = 0.2;
  double frontier_smoothness_loss_weight_ = 0.35;
  bool use_edge_witness_path_ = true;
  double goal_path_cost_weight_ = 1.0;
  double semantic_cost_weight_ = 2.0;
  double semantic_route_replan_delta_ = 0.15;
  bool semantic_route_replan_enabled_ = true;
  double semantic_route_high_risk_ = 0.35;
  double semantic_route_high_risk_release_ = 0.30;
  double semantic_route_switch_risk_margin_ = 0.08;
  double semantic_route_switch_cost_ratio_ = 0.90;
  double semantic_route_influence_m_ = 5.0;
  double semantic_visualization_max_score_ = 0.4;
  double semantic_baseline_quantile_ = 0.25;
  double semantic_virtual_depth_m_ = 30.0;
  bool semantic_points_enabled_ = true;
  double semantic_point_min_score_ = 0.35;
  double semantic_point_separation_m_ = 1.5;
  double semantic_point_radius_m_ = 0.75;
  int semantic_point_max_nodes_ = 16;
  double clearance_cost_weight_ = 2.0;
  double clearance_target_m_ = 1.2;
  double previous_path_cost_factor_ = 0.9;
  double route_remap_distance_m_ = 1.25;
  double route_reuse_horizon_m_ = 10.0;
  double route_reuse_lateral_distance_m_ = 1.5;
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
  double semantic_camera_tx_ = 0.5;
  double semantic_camera_ty_ = 0.0;
  double semantic_camera_tz_ = -0.1;
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
  double frontier_extension_search_period_ms_ = 1000.0;
  bool have_last_extension_search_ = false;
  std::chrono::steady_clock::time_point last_extension_search_time_{};
  std::atomic<std::uint64_t> topology_update_generation_{0};
  std::uint64_t last_route_search_generation_ =
    std::numeric_limits<std::uint64_t>::max();
  // Straight vehicle→goal polyline captured at each reused-graph goal change.
  // Supplies remembered-edge priors after witness memory is cleared on a new
  // mission goal (e.g. the return leg of an out-and-back run).
  std::vector<Eigen::Vector3f> corridor_hint_route_;
  bool semantic_replan_requested_ = false;
  float evaluated_route_risk_ = 0.0F;
  bool have_evaluated_route_risk_ = false;
  bool high_risk_evaluated_ = false;
  Eigen::Vector3f previous_local_goal_ = Eigen::Vector3f::Zero();
  bool have_previous_local_goal_ = false;
  std::chrono::steady_clock::time_point previous_local_goal_time_{};
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr free_ray_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr semantic_heatmap_sub_;
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
  std::int64_t last_semantic_applied_stamp_ns_ = 0;
  TopoGraph::Ptr semantic_applied_topo_;
  std::mutex semantic_mutex_;
  mutable std::mutex semantic_memory_mutex_;
  std::unordered_map<std::uint64_t, TopoSemanticRecord> semantic_memory_;
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
  auto node = std::make_shared<EpicGraphNode>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 4);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
