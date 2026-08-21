#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <cstdint>
#include <deque>
#include <functional>
#include <iterator>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <set>
#include <thread>
#include <tuple>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <Eigen/Dense>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "openseek_epic_ros2/route_memory.hpp"
#include "pointcloud_topo/graph.h"

namespace {

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

}  // namespace

class EpicGraphNode final : public rclcpp::Node {
 public:
  EpicGraphNode()
  : Node("epic_graph_node"),
    map_(std::make_shared<fast_planner::LIOInterface>()),
    astar_(std::make_shared<ParallelBubbleAstar>()),
    topo_(std::make_shared<TopoGraph>())
  {
    cloud_topic_ = declare_parameter<std::string>("cloud_topic", "/frgraph/points");
    free_ray_topic_ = declare_parameter<std::string>("free_ray_topic", "/frgraph/free_rays");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/sim/odom");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal");
    next_goal_topic_ = declare_parameter<std::string>("next_goal_topic", "/epic/yopo_goal");
    next_goal_frame_ = declare_parameter<std::string>("next_goal_frame", "world_enu");
    visualization_frame_ = declare_parameter<std::string>("visualization_frame", "odom");
    graph_fixed_layer_ = declare_parameter<bool>("graph_fixed_layer", true);
    graph_layer_z_ = declare_parameter<double>("graph_layer_z", 1.6);
    reuse_graph_on_goal_ = declare_parameter<bool>("reuse_graph_on_goal", true);
    map_margin_ = declare_parameter<double>("map_margin", 20.0);
    map_voxel_size_ = declare_parameter<double>("map_voxel_size", 0.25);
    map_history_radius_m_ = declare_parameter<double>("map_history_radius_m", 20.0);
    map_max_points_ = declare_parameter<int>("map_max_points", 20000);
    map_prune_distance_m_ = declare_parameter<double>("map_prune_distance_m", 0.5);
    update_period_ms_ = declare_parameter<int>("update_period_ms", 100);
    skeleton_rebuild_period_ms_ = declare_parameter<double>("skeleton_rebuild_period_ms", 500.0);
    local_goal_min_advance_m_ = declare_parameter<double>("local_goal_min_advance_m", 0.75);
    local_goal_lookahead_m_ = declare_parameter<double>("local_goal_lookahead_m", 10.0);
    route_plan_period_ms_ = declare_parameter<int>("route_plan_period_ms", 2000);
    local_goal_reserve_m_ = declare_parameter<double>("local_goal_reserve_m", 5.0);
    use_edge_witness_path_ = declare_parameter<bool>("use_edge_witness_path", true);
    raycast_shortcut_sample_step_m_ =
      declare_parameter<double>("raycast_shortcut_sample_step_m", 0.25);
    raycast_shortcut_clearance_margin_m_ =
      declare_parameter<double>("raycast_shortcut_clearance_margin_m", 0.05);
    goal_path_cost_weight_ = declare_parameter<double>("goal_path_cost_weight", 0.2);
    semantic_cost_weight_ = declare_parameter<double>("semantic_cost_weight", 1.0);
    semantic_node_ema_alpha_ = declare_parameter<double>("semantic_node_ema_alpha", 0.3);
    semantic_visualization_max_score_ = declare_parameter<double>(
      "semantic_visualization_max_score", 1.0);
    previous_path_cost_factor_ = declare_parameter<double>("previous_path_cost_factor", 0.0);
    route_remap_distance_m_ = declare_parameter<double>("route_remap_distance_m", 1.25);
    route_reuse_horizon_m_ = declare_parameter<double>("route_reuse_horizon_m", 6.0);
    route_reuse_lateral_distance_m_ =
      declare_parameter<double>("route_reuse_lateral_distance_m", 1.5);
    route_terminal_release_distance_m_ =
      declare_parameter<double>("route_terminal_release_distance_m", 1.0);
    goal_connect_distance_m_ = declare_parameter<double>("goal_connect_distance_m", 6.0);
    goal_connect_timeout_ms_ = declare_parameter<double>("goal_connect_timeout_ms", 20.0);
    odom_reconnect_distance_m_ = declare_parameter<double>("odom_reconnect_distance_m", 1.0);
    odom_reconnect_yaw_deg_ = declare_parameter<double>("odom_reconnect_yaw_deg", 20.0);
    odom_fallback_radius_m_ = declare_parameter<double>("odom_fallback_radius_m", 15.0);
    odom_fallback_candidates_ = declare_parameter<int>("odom_fallback_candidates", 8);
    odom_connect_timeout_ms_ = declare_parameter<double>("odom_connect_timeout_ms", 3.0);
    cloud_pose_tolerance_ms_ = declare_parameter<double>("cloud_pose_tolerance_ms", 50.0);
    semantic_heatmap_topic_ = declare_parameter<std::string>(
      "semantic_heatmap_topic", "/openseek/text_heatmap_raw");
    semantic_pose_tolerance_ms_ = declare_parameter<double>(
      "semantic_pose_tolerance_ms", 100.0);
    semantic_max_age_ms_ = declare_parameter<double>("semantic_max_age_ms", 1500.0);
    semantic_camera_tx_ = declare_parameter<double>("semantic_camera_translation_flu.x", 0.5);
    semantic_camera_ty_ = declare_parameter<double>("semantic_camera_translation_flu.y", 0.0);
    semantic_camera_tz_ = declare_parameter<double>("semantic_camera_translation_flu.z", -0.1);
    semantic_horizontal_fov_deg_ = declare_parameter<double>("semantic_horizontal_fov_deg", 90.0);
    semantic_vertical_fov_deg_ = declare_parameter<double>("semantic_vertical_fov_deg", 60.0);
    semantic_association_radius_m_ = declare_parameter<double>(
      "semantic_association_radius_m", 1.5);
    semantic_voxel_size_m_ = declare_parameter<double>("semantic_voxel_size_m", 0.5);
    depth_image_topic_ = declare_parameter<std::string>(
      "depth_image_topic", "/camera/depth/image");

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
    declare_parameter<int>("max_update_region_num", 100);
    // Odom reconnection is on the online update path. Keep the EPIC local
    // connection search bounded; terminal-to-goal uses its separate budget.
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

    RCLCPP_INFO(
      get_logger(),
      "EPIC Bubble/TopoGraph volume=3D; YOPO subgoal layer=%s z=%.2f; "
      "route/subgoal=%.2f Hz lookahead>=%.2f m reserve=%.2f m",
      graph_fixed_layer_ ? "fixed" : "3D", graph_layer_z_,
      1000.0 / static_cast<double>(std::max(1, route_plan_period_ms_)),
      local_goal_lookahead_m_, local_goal_reserve_m_);
    RCLCPP_INFO(
      get_logger(),
      "EPIC costs: clearance_target=%.2f m clearance_weight=%.2f "
      "semantic_radius=%.2f m semantic_visual_max=%.2f",
      clearance_target_m_, clearance_cost_weight_,
      semantic_association_radius_m_, semantic_visualization_max_score_);

    graph_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/epic/graph", 1);
    bubble_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("/epic/bubbles", 1);
    semantic_voxel_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
      "/epic/semantic_voxels", 1);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/epic/path", 1);
    next_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(next_goal_topic_, 10);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { onCloud(message); });
    free_ray_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      free_ray_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { onFreeRays(message); });
    auto semantic_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();
    semantic_heatmap_sub_ = create_subscription<sensor_msgs::msg::Image>(
      semantic_heatmap_topic_, semantic_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) {
        onSemanticHeatmap(message);
      });
    depth_image_sub_ = create_subscription<sensor_msgs::msg::Image>(
      depth_image_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) { onDepthImage(message); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { onOdom(message); });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic_, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) { onGoal(message); });
    timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(1, update_period_ms_)), [this]() { update(); });
  }

  ~EpicGraphNode() override
  {
    shutting_down_.store(true);
    if (rebuild_thread_.joinable()) rebuild_thread_.join();
  }

 private:
  struct TimedPose
  {
    std::int64_t stamp_ns = 0;
    Eigen::Vector3f position = Eigen::Vector3f::Zero();
    Eigen::Quaternionf orientation = Eigen::Quaternionf::Identity();
  };

  struct SemanticFrame
  {
    std::int64_t stamp_ns = 0;
    std::vector<Eigen::Vector3f> surface_points_world;
    std::vector<float> scores;
  };

  struct DepthFrame
  {
    std::int64_t stamp_ns = 0;
    std::uint32_t width = 0;
    std::uint32_t height = 0;
    std::uint32_t step = 0;
    std::vector<std::uint8_t> data;
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
    speed_mps_ = static_cast<float>(std::sqrt(
      linear_velocity.x * linear_velocity.x +
      linear_velocity.y * linear_velocity.y +
      linear_velocity.z * linear_velocity.z));
    odom_history_.push_back(TimedPose{
      stampNanoseconds(message->header.stamp), next_position, next_orientation});
    while (odom_history_.size() > max_odom_history_size_) odom_history_.pop_front();
    if (graph_fixed_layer_ && !graph_layer_initialized_) {
      graph_layer_z_ = position_.z();
      graph_layer_initialized_ = true;
      RCLCPP_INFO(get_logger(), "EPIC graph fixed layer z=%.2f", graph_layer_z_);
    }
    have_odom_ = true;
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
    DepthFrame depth;
    {
      std::lock_guard<std::mutex> lock(semantic_mutex_);
      auto match = std::find_if(depth_history_.begin(), depth_history_.end(),
        [stamp_ns](const DepthFrame &candidate) { return candidate.stamp_ns == stamp_ns; });
      if (match == depth_history_.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Ignoring semantic heatmap without same-timestamp depth frame");
        return;
      }
      depth = *match;
    }

    std::map<std::tuple<int, int, int>, std::pair<Eigen::Vector3f, float>> frame_voxels;
    const double fx = 0.5 * static_cast<double>(depth.width) /
      std::tan(semantic_horizontal_fov_deg_ * M_PI / 360.0);
    const double fy = 0.5 * static_cast<double>(depth.height) /
      std::tan(semantic_vertical_fov_deg_ * M_PI / 360.0);
    const double cx = (static_cast<double>(depth.width) - 1.0) * 0.5;
    const double cy = (static_cast<double>(depth.height) - 1.0) * 0.5;
    const Eigen::Vector3f camera_translation(
      static_cast<float>(semantic_camera_tx_), static_cast<float>(semantic_camera_ty_),
      static_cast<float>(semantic_camera_tz_));
    for (std::uint32_t v = 0; v < depth.height; ++v) {
      const auto *depth_row = reinterpret_cast<const float *>(
        depth.data.data() + static_cast<std::size_t>(v) * depth.step);
      const std::uint32_t heatmap_v = std::min(
        message->height - 1,
        static_cast<std::uint32_t>((static_cast<std::uint64_t>(v) * message->height) /
                                  depth.height));
      const auto *heatmap_row = reinterpret_cast<const float *>(
        message->data.data() + static_cast<std::size_t>(heatmap_v) * message->step);
      for (std::uint32_t u = 0; u < depth.width; ++u) {
        const float z = depth_row[u];
        if (!std::isfinite(z) || z <= 0.0F || z >= 20.0F - 1e-4F) continue;
        const std::uint32_t heatmap_u = std::min(
          message->width - 1,
          static_cast<std::uint32_t>((static_cast<std::uint64_t>(u) * message->width) /
                                    depth.width));
        const float semantic = std::clamp(heatmap_row[heatmap_u], 0.0F, 1.0F);
        if (!std::isfinite(semantic)) continue;
        const Eigen::Vector3f body(
          z + camera_translation.x(),
          static_cast<float>(-(static_cast<double>(u) - cx) * z / fx) +
            camera_translation.y(),
          static_cast<float>(-(static_cast<double>(v) - cy) * z / fy) +
            camera_translation.z());
        const Eigen::Vector3f world = capture_pose.position + capture_pose.orientation * body;
        const auto key = std::make_tuple(
          static_cast<int>(std::floor(world.x() / semantic_voxel_size_m_)),
          static_cast<int>(std::floor(world.y() / semantic_voxel_size_m_)),
          static_cast<int>(std::floor(world.z() / semantic_voxel_size_m_)));
        auto [entry, inserted] = frame_voxels.emplace(key, std::make_pair(world, semantic));
        if (!inserted && semantic > entry->second.second) entry->second = {world, semantic};
      }
    }
    {
      std::lock_guard<std::mutex> lock(semantic_mutex_);
      SemanticFrame frame;
      frame.stamp_ns = stamp_ns;
      frame.surface_points_world.reserve(frame_voxels.size());
      frame.scores.reserve(frame_voxels.size());
      for (const auto &[key, value] : frame_voxels) {
        (void)key;
        frame.surface_points_world.push_back(value.first);
        frame.scores.push_back(value.second);
      }
      semantic_frame_ = std::move(frame);
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[EPIC semantic] heatmap=%ux%u pose_sync=%.1f ms prompt topic=%s",
      message->width, message->height, pose_delta_ms, semantic_heatmap_topic_.c_str());
  }

  void onDepthImage(const sensor_msgs::msg::Image::ConstSharedPtr &message)
  {
    if (message->encoding != "32FC1" || message->is_bigendian ||
        message->width == 0 || message->height == 0 ||
        message->step < message->width * sizeof(float) ||
        message->data.size() < message->step * message->height) return;
    DepthFrame frame;
    frame.stamp_ns = stampNanoseconds(message->header.stamp);
    frame.width = message->width;
    frame.height = message->height;
    frame.step = message->step;
    frame.data = message->data;
    std::lock_guard<std::mutex> lock(semantic_mutex_);
    depth_history_.push_back(std::move(frame));
    while (depth_history_.size() > max_depth_history_size_) depth_history_.pop_front();
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

  void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &message)
  {
    Eigen::Vector3f next_goal(
      static_cast<float>(message->pose.position.x),
      static_cast<float>(message->pose.position.y),
      static_cast<float>(message->pose.position.z));
    if (graph_fixed_layer_) {
      next_goal.z() = static_cast<float>(graph_layer_initialized_ ? graph_layer_z_ : next_goal.z());
    }
    if (have_goal_ && (next_goal - goal_).norm() < 1e-3F) return;
    vector<TopoSemanticRecord> semantic_memory;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      if (topo_) mergeSemanticMemory(topo_->semanticMemorySnapshot());
      semantic_memory = semanticMemorySnapshot();
      goal_ = next_goal;
      have_goal_ = true;
      route_plan_requested_ = true;
      // A new mission goal gets a new route, even when the existing topology
      // and its edge witness paths are reused.
      last_topology_path_centers_.clear();
      last_witness_path_.clear();
      have_route_terminal_ = false;
      const bool in_existing_map = topo_ && topo_->lidar_map_interface_ &&
        topo_->lidar_map_interface_->IsInBox(next_goal);
      const bool can_reuse = reuse_graph_on_goal_ && graph_initialized_.load() &&
        skeleton_initialized_.load() && topo_ && astar_ && in_existing_map;
      if (!can_reuse) {
        graph_initialized_ = false;
        skeleton_initialized_ = false;
        map_changed_ = true;
        have_graph_odom_ = false;
        astar_ = std::make_shared<ParallelBubbleAstar>();
        topo_ = std::make_shared<TopoGraph>();
        topo_->loadSemanticMemory(semantic_memory);
      }
      ++goal_generation_;
      RCLCPP_INFO(
        get_logger(), "EPIC goal graph mode: %s (reuse_graph_on_goal=%d in_map=%d)",
        can_reuse ? "REUSE_EXISTING_GRAPH" : "REBUILD_GRAPH",
        static_cast<int>(reuse_graph_on_goal_),
        static_cast<int>(in_existing_map));
    }
    RCLCPP_INFO(get_logger(),
      "EPIC goal set: %.2f %.2f %.2f semantic_memory=%zu",
      goal_.x(), goal_.y(), goal_.z(), semantic_memory.size());
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message)
  {
    const auto callback_start = std::chrono::steady_clock::now();
    const auto decode_start = callback_start;
    pcl::PointCloud<fast_planner::PointType> cloud_body;
    pcl::fromROSMsg(*message, cloud_body);
    if (cloud_body.empty() || !have_odom_) return;
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
    float nearest_body_distance = std::numeric_limits<float>::infinity();
    Eigen::Vector3f nearest_body_point = Eigen::Vector3f::Zero();
    for (const auto &point : cloud_body.points) {
      const Eigen::Vector3f body(point.x, point.y, point.z);
      const float distance = body.norm();
      if (distance < nearest_body_distance) {
        nearest_body_distance = distance;
        nearest_body_point = body;
      }
      const Eigen::Vector3f world =
        capture_pose.position + capture_pose.orientation * body;
      cloud_world.push_back(fast_planner::PointType(world.x(), world.y(), world.z()));
    }
    const double transform_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - transform_start).count();

    const auto map_start = std::chrono::steady_clock::now();
    fast_planner::LIOInterface::Ptr active_map;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      active_map = map_;
    }
    const bool changed = active_map->updateCloudWorld(
      cloud_world, capture_pose.position, capture_pose.orientation);
    if (changed) map_changed_.store(true);
    const double map_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - map_start).count();
    std::size_t ray_hits = 0;
    std::size_t ray_free = 0;
    std::size_t ray_carved = 0;
    active_map->lastRayCarvingStats(ray_hits, ray_free, ray_carved);
    cloud_count_++;
    have_cloud_ = true;
    const double total_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - callback_start).count();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[EPIC timing][cloud] decode=%.3f ms transform=%.3f ms map_update=%.3f ms "
      "total=%.3f ms pose_sync=%.3f ms input=%zu map_points=%zu "
      "ray_voxels=%zu/%zu carved=%zu nearest_body=%.2f@"
      "(%.2f,%.2f,%.2f)",
      decode_ms, transform_ms, map_ms, total_ms, pose_sync_ms, cloud_body.size(),
      active_map->pointCount(), ray_hits, ray_free, ray_carved,
      nearest_body_distance, nearest_body_point.x(),
      nearest_body_point.y(), nearest_body_point.z());
  }

  void onFreeRays(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message)
  {
    if (!have_odom_ || message->width == 0) return;
    pcl::PointCloud<fast_planner::PointType> cloud_body;
    pcl::fromROSMsg(*message, cloud_body);
    if (cloud_body.empty()) return;
    TimedPose capture_pose;
    double pose_sync_ms = 0.0;
    if (!poseForCloud(message->header.stamp, capture_pose, pose_sync_ms)) return;
    pcl::PointCloud<fast_planner::PointType> rays_world;
    rays_world.reserve(cloud_body.size());
    for (const auto &point : cloud_body.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) continue;
      const Eigen::Vector3f body(point.x, point.y, point.z);
      const Eigen::Vector3f world = capture_pose.position +
        capture_pose.orientation * body;
      rays_world.push_back(fast_planner::PointType(world.x(), world.y(), world.z()));
    }
    if (rays_world.empty()) return;
    fast_planner::LIOInterface::Ptr active_map;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      active_map = map_;
    }
    const bool changed = active_map->updateFreeRaysWorld(
      rays_world, capture_pose.position, capture_pose.orientation);
    have_cloud_ = true;
    if (changed) {
      map_changed_.store(true);
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[EPIC free rays] endpoints=%zu pose_sync=%.1f ms changed=%d",
      rays_world.size(), pose_sync_ms, static_cast<int>(changed));
  }

  static std::int64_t stampNanoseconds(const builtin_interfaces::msg::Time &stamp)
  {
    return static_cast<std::int64_t>(stamp.sec) * 1000000000LL +
      static_cast<std::int64_t>(stamp.nanosec);
  }

  bool poseForCloud(const builtin_interfaces::msg::Time &stamp, TimedPose &pose,
                    double &delta_ms) const
  {
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
    Eigen::Vector3f lower = position.cwiseMin(goal) -
      Eigen::Vector3f::Constant(static_cast<float>(margin));
    Eigen::Vector3f upper = position.cwiseMax(goal) +
      Eigen::Vector3f::Constant(static_cast<float>(margin));
    map->configureBounds(lower, upper);
  }

  void startSkeletonRebuild()
  {
    if (rebuild_running_.exchange(true)) return;
    if (rebuild_thread_.joinable()) rebuild_thread_.join();

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
          auto next_map = incremental_update ? current_topo->lidar_map_interface_ :
            std::make_shared<fast_planner::LIOInterface>();
          if (!incremental_update) {
            // Bubble generation remains 3D; only the derived topology is planar.
            configureMapBounds(
              next_map, position, goal, std::max(map_margin_, map_history_radius_m_));
            next_map->configureStorage(
              static_cast<float>(map_voxel_size_), static_cast<float>(map_history_radius_m_),
              static_cast<std::size_t>(std::max(map_max_points_, 1000)),
              static_cast<float>(map_prune_distance_m_));
          }
          const auto snapshot_start = std::chrono::steady_clock::now();
          next_map->loadSnapshot(accumulated, latest, free_space, position, orientation);
          const double snapshot_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - snapshot_start).count();
          const auto init_start = std::chrono::steady_clock::now();
          auto next_astar = incremental_update ? current_astar :
            std::make_shared<ParallelBubbleAstar>();
          auto next_topo = incremental_update ? current_topo :
            std::make_shared<TopoGraph>();
          if (!incremental_update) {
            ros::NodeHandle nh(shared_from_this());
            next_astar->init(nh, next_map);
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
          next_topo->getRegionsToUpdate();
          const double regions_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - regions_start).count();
          const auto skeleton_start = std::chrono::steady_clock::now();
          next_topo->updateSkeleton();
          const double skeleton_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - skeleton_start).count();

          float yaw = std::atan2((orientation * Eigen::Vector3f::UnitX()).y(),
                                 (orientation * Eigen::Vector3f::UnitX()).x());
          Eigen::Vector3f rebuild_position = position;
          const auto odom_start = std::chrono::steady_clock::now();
          next_topo->updateOdomNode(rebuild_position, yaw);
          const std::size_t fallback_edges = ensureOdomConnectivity(next_topo, rebuild_position);
          const double odom_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - odom_start).count();
          const auto timing = next_topo->getLastUpdateTiming();
          mergeSemanticMemory(next_topo->semanticMemorySnapshot());

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
            astar_ = std::move(next_astar);
            topo_ = std::move(next_topo);
            graph_initialized_ = true;
            skeleton_initialized_ = true;
            ++skeleton_update_count_;
          }
          const double total_ms = std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - total_start).count();
          RCLCPP_INFO(get_logger(),
            "[EPIC timing][background %s] points=%zu snapshot_kdtree=%.3f ms "
            "free_memory=%zu init=%.3f ms regions=%zu occupied_regions=%zu "
            "free_regions=%zu region_select=%.3f ms skeleton=%.3f ms "
            "odom_connect=%.3f ms "
            "total=%.3f ms bubbles_3d=%zu bubbles_planar=%zu nodes=%zu "
            "semantic_restored=%zu semantic_memory=%zu",
            incremental_update ? "incremental" : "initialize",
            accumulated.size(), snapshot_ms, free_space.size(), init_ms,
            timing.regions, timing.occupied_regions, timing.free_regions,
            regions_ms, skeleton_ms, odom_ms, total_ms,
            timing.bubbles, timing.planar_bubbles, timing.new_nodes,
            timing.semantic_restored_nodes, timing.semantic_memory_records);
          if (fallback_edges > 0) {
            RCLCPP_INFO(get_logger(),
              "[EPIC odom] expanded real Bubble connectivity: edges=%zu", fallback_edges);
          }
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
    // TopoGraph is updated in place after initialization. Do not read it while
    // the background worker is applying the EPIC V_remove/V_remain/V_insert diff.
    if (rebuild_running_.load()) return;

    TopoGraph::Ptr active_topo;
    {
      std::lock_guard<std::mutex> lock(graph_mutex_);
      active_topo = topo_;
    }
    if (!graph_initialized_ || !active_topo || !active_topo->odom_node_) return;
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
    if (!have_graph_odom_ ||
        (position_ - graph_odom_position_).norm() > odom_reconnect_distance_m_ ||
        std::abs(yaw_delta) > odom_reconnect_yaw_deg_ * static_cast<float>(M_PI / 180.0)) {
      const auto odom_start = std::chrono::steady_clock::now();
      Eigen::Vector3f graph_position = position_;
      active_topo->updateOdomNode(graph_position, yaw);
      const std::size_t fallback_edges = ensureOdomConnectivity(active_topo, graph_position);
      if (fallback_edges > 0) {
        RCLCPP_INFO(get_logger(),
          "[EPIC odom] expanded real Bubble connectivity: edges=%zu", fallback_edges);
      }
      odom_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - odom_start).count();
      graph_odom_position_ = position_;
      graph_odom_yaw_ = yaw;
      graph_odom_topo_ = active_topo;
      have_graph_odom_ = true;
    }
    const bool route_plan_due = route_plan_requested_ || !have_route_plan_time_ ||
      std::chrono::duration<double, std::milli>(
        now_steady - last_route_plan_time_).count() >= std::max(1, route_plan_period_ms_);
    if (!route_plan_due) return;
    last_route_plan_time_ = now_steady;
    have_route_plan_time_ = true;
    route_plan_requested_ = false;

    const float effective_lookahead_m = openseek_epic::velocityCompensatedLookahead(
      static_cast<float>(local_goal_lookahead_m_), speed_mps_,
      static_cast<float>(std::max(1, route_plan_period_ms_)) / 1000.0F,
      static_cast<float>(local_goal_reserve_m_));
    std::vector<TopoNode::Ptr> path_nodes;
    std::unordered_set<std::pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash>
      last_path_edges;
    const auto reusable_route = openseek_epic::forwardRouteWindow(
      last_witness_path_, position_,
      std::max(static_cast<float>(route_reuse_horizon_m_), effective_lookahead_m));
    const std::size_t geometrically_remembered_edges = buildRememberedEdges(
      active_topo, reusable_route, static_cast<float>(route_remap_distance_m_),
      last_path_edges);

    const auto astar_start = std::chrono::steady_clock::now();
    bool reused_terminal = false;
    bool found = false;
    const bool route_aligned = openseek_epic::canReuseForwardRoute(
      position_, last_witness_path_, 0.0F,
      static_cast<float>(route_reuse_lateral_distance_m_));
    const bool route_has_planning_horizon = openseek_epic::canReuseForwardRoute(
      position_, last_witness_path_,
      std::max(effective_lookahead_m,
        static_cast<float>(route_terminal_release_distance_m_)),
      static_cast<float>(route_reuse_lateral_distance_m_));
    // Leaving the directed corridor invalidates its remembered edges. Merely
    // approaching its terminal does not: retain those edges as the A* prior
    // and extend the rolling route before the local lookahead is exhausted.
    if (have_route_terminal_ && !route_aligned) {
      last_path_edges.clear();
    }
    if (have_route_terminal_ && route_has_planning_horizon) {
      const auto terminal = nearestPersistentNode(
        active_topo, route_terminal_, static_cast<float>(route_remap_distance_m_));
      if (terminal) {
        found = active_topo->graphSearch(
          active_topo->odom_node_, terminal, path_nodes, 0.2, true, last_path_edges,
          static_cast<float>(semantic_cost_weight_));
        reused_terminal = found;
      }
    }
    if (!found) {
      found = active_topo->goalDirectedSearch(
        active_topo->odom_node_, goal_, path_nodes, 0.2,
        static_cast<float>(goal_path_cost_weight_),
        static_cast<float>(previous_path_cost_factor_), last_path_edges,
        static_cast<float>(semantic_cost_weight_));
    }
    std::size_t reused_path_edges = 0;
    for (std::size_t i = 1; i < path_nodes.size(); ++i) {
      if (last_path_edges.find({path_nodes[i - 1], path_nodes[i]}) != last_path_edges.end()) {
        ++reused_path_edges;
      }
    }
    std::vector<Eigen::Vector3f> terminal_extension;
    if (found) {
      connectTerminalToGoal(active_topo, path_nodes.back(), terminal_extension);
    }
    astar_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - astar_start).count();
    if (found) {
      last_topology_path_centers_.clear();
      last_topology_path_centers_.reserve(path_nodes.size());
      for (const auto &node : path_nodes) {
        if (node) last_topology_path_centers_.push_back(node->center_);
      }
      if (!path_nodes.empty()) {
        route_terminal_ = path_nodes.back()->center_;
        have_route_terminal_ = true;
      }
    }
    const auto publish_start = std::chrono::steady_clock::now();
    const auto stats = publish(
      active_topo, path_nodes, terminal_extension, found, reused_terminal,
      effective_lookahead_m);
    publish_ms = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - publish_start).count();
    if (!found) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "EPIC rolling route has no reachable real Bubble topology: "
        "odom_degree=%zu skeleton_nodes=%zu edges=%zu",
        active_topo->odom_node_->neighbors_.size(), stats.skeleton_nodes, stats.edges);
    }

    const auto end = std::chrono::steady_clock::now();
    const double ms = std::chrono::duration<double, std::milli>(end - start).count();
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
      "[EPIC timing][update] rebuild_running=%d odom_connect=%.3f ms "
      "astar=%.3f ms publish=%.3f ms total=%.3f ms cloud=%zu skeleton_updates=%zu "
      "bubbles=%zu nodes=%zu edges=%zu path_nodes=%zu witness_points=%zu->%zu "
      "geometry_source=%s "
      "remembered_edges=%zu/%zu geometric_edges=%zu route_mode=%s "
      "semantic_nodes=%zu semantic_max=%.3f "
      "route_aligned=%d horizon_ready=%d terminal=(%.2f,%.2f,%.2f) "
      "terminal_goal_distance=%.2f m "
      "vehicle_to_terminal=%.2f m "
      "terminal_extension=%zu raycast_shortcut=%.3f ms queries=%zu "
      "segments=%zu/%zu found=%d",
      static_cast<int>(rebuild_running_.load()), odom_ms, astar_ms, publish_ms, ms,
      cloud_count_, skeleton_update_count_.load(), stats.bubbles,
      stats.skeleton_nodes, stats.edges, path_nodes.size(), stats.witness_points_raw,
      stats.witness_points, use_edge_witness_path_ ? "EDGE_WITNESS" : "TOPO_CENTERS",
      reused_path_edges,
      path_nodes.size() > 1 ? path_nodes.size() - 1 : 0,
      geometrically_remembered_edges, reused_terminal ? "RHC_REUSE" : "EXTEND",
      stats.semantic_nodes, stats.semantic_max,
      static_cast<int>(route_aligned), static_cast<int>(route_has_planning_horizon),
      route_terminal_.x(), route_terminal_.y(), route_terminal_.z(),
      path_nodes.empty() ? std::numeric_limits<float>::infinity() :
        (path_nodes.back()->center_ - goal_).norm(),
      (position_ - route_terminal_).norm(),
      terminal_extension.size(), stats.witness_shortcut_ms,
      stats.raycast_clearance_queries, stats.raycast_accepted_segments,
      stats.raycast_tested_segments,
      static_cast<int>(found));
  }

  TopoNode::Ptr nearestPersistentNode(const TopoGraph::Ptr &topo,
                                      const Eigen::Vector3f &position,
                                      float maximum_distance) const
  {
    TopoNode::Ptr nearest;
    float nearest_distance = maximum_distance;
    for (const auto &entry : topo->reg_map_idx2ptr_) {
      if (!entry.second) continue;
      for (const auto &node : entry.second->topo_nodes_) {
        if (!node || node->is_viewpoint_) continue;
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
    topo->odom_node_->center_ = query_position;

    std::vector<TopoNode::Ptr> candidates;
    std::unordered_set<TopoNode::Ptr> unique;
    for (const auto &entry : topo->reg_map_idx2ptr_) {
      if (!entry.second) continue;
      for (const auto &node : entry.second->topo_nodes_) {
        if (!node || node->is_viewpoint_ || !unique.insert(node).second) continue;
        if ((node->center_ - query_position).norm() <= odom_fallback_radius_m_) {
          candidates.push_back(node);
        }
      }
    }
    std::sort(candidates.begin(), candidates.end(),
      [&query_position](const TopoNode::Ptr &left, const TopoNode::Ptr &right) {
        return (left->center_ - query_position).squaredNorm() <
               (right->center_ - query_position).squaredNorm();
      });

    const std::size_t limit = std::min(
      candidates.size(), static_cast<std::size_t>(std::max(0, odom_fallback_candidates_)));
    std::size_t inserted = 0;
    std::size_t reach_end = 0;
    std::size_t no_path = 0;
    std::size_t start_fail = 0;
    std::size_t end_fail = 0;
    std::size_t timed_out = 0;
    std::size_t shorten_fail = 0;
    for (std::size_t i = 0; i < limit; ++i) {
      std::vector<Eigen::Vector3f> path;
      const int result = topo->parallel_bubble_astar_->search(
        query_position, candidates[i]->center_, path,
        std::max(0.5, odom_connect_timeout_ms_) / 1000.0, false);
      switch (result) {
        case ParallelBubbleAstar::REACH_END: ++reach_end; break;
        case ParallelBubbleAstar::NO_PATH: ++no_path; break;
        case ParallelBubbleAstar::START_FAIL: ++start_fail; break;
        case ParallelBubbleAstar::END_FAIL: ++end_fail; break;
        case ParallelBubbleAstar::TIME_OUT: ++timed_out; break;
        default: break;
      }
      if (result != ParallelBubbleAstar::REACH_END || path.size() < 2) continue;
      if (!topo->parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
        ++shorten_fail;
        continue;
      }
      topo->odom_node_->neighbors_.insert(candidates[i]);
      topo->odom_node_->paths_[candidates[i]] = path;
      topo->odom_node_->weight_[candidates[i]] = 0.0;
      ++inserted;
      // One connected real Bubble is sufficient to make the persistent graph
      // reachable. Keep a second connection when it is available for branching.
      if (inserted >= 2) break;
    }
    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "[EPIC odom diagnosis] pos=(%.2f,%.2f,%.2f) clearance=%.2f candidates=%zu "
      "tested=%zu connected=%zu no_path=%zu start_fail=%zu end_fail=%zu timeout=%zu "
      "shorten_fail=%zu",
      query_position.x(), query_position.y(), query_position.z(),
      topo->lidar_map_interface_->getDisToOcc(query_position),
      candidates.size(), limit, inserted, no_path, start_fail, end_fail, timed_out,
      shorten_fail);
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
          if (!openseek_epic::edgeFollowsRoute(
                node->center_, neighbor->center_, route, maximum_distance)) continue;
          edges.insert({node, neighbor});
          edges.insert({neighbor, node});
          ++count;
        }
      }
    }
    return count;
  }

  bool connectTerminalToGoal(const TopoGraph::Ptr &topo, const TopoNode::Ptr &terminal,
                             std::vector<Eigen::Vector3f> &extension) const
  {
    extension.clear();
    if (!topo || !terminal || !topo->parallel_bubble_astar_) return false;
    Eigen::Vector3f layer_goal = goal_;
    if (graph_fixed_layer_) layer_goal.z() = static_cast<float>(graph_layer_z_);
    const float distance = (terminal->center_ - layer_goal).norm();
    if (distance > goal_connect_distance_m_) return false;
    if (distance < 1e-3F) {
      extension = {terminal->center_, layer_goal};
      return true;
    }
    const int result = topo->parallel_bubble_astar_->search(
      terminal->center_, layer_goal, extension,
      std::max(1.0, goal_connect_timeout_ms_) / 1000.0, true);
    if (result != ParallelBubbleAstar::REACH_END || extension.size() < 2 ||
        !topo->parallel_bubble_astar_->collisionCheck_shortenPath(extension)) {
      extension.clear();
      return false;
    }
    if ((extension.front() - terminal->center_).norm() >
        (extension.back() - terminal->center_).norm()) {
      std::reverse(extension.begin(), extension.end());
    }
    if ((extension.front() - terminal->center_).norm() > 0.5F ||
        (extension.back() - layer_goal).norm() > 0.5F) {
      extension.clear();
      return false;
    }
    extension.front() = terminal->center_;
    extension.back() = layer_goal;
    return true;
  }

  bool semanticScore(const SemanticFrame &frame, const TopoNode::Ptr &node,
                     float &score) const
  {
    score = 0.0F;
    if (!node || frame.surface_points_world.empty() ||
        frame.surface_points_world.size() != frame.scores.size()) {
      return false;
    }
    const float bubble_radius = std::max(0.0F, node->bubble_radius_);
    const float semantic_boundary_radius = static_cast<float>(
      std::max(semantic_association_radius_m_, 1e-3));
    const float association_radius = bubble_radius + semantic_boundary_radius;
    const float radius_sq = association_radius * association_radius;
    bool associated = false;
    for (std::size_t i = 0; i < frame.surface_points_world.size(); ++i) {
      const float distance_sq =
        (frame.surface_points_world[i] - node->center_).squaredNorm();
      if (!std::isfinite(distance_sq) || distance_sq > radius_sq) continue;
      const float center_distance = std::sqrt(distance_sq);
      // Semantic risk is measured from the semantic surface to the Bubble
      // boundary. A node whose Bubble nearly touches the surface remains warm;
      // using center distance alone incorrectly paints that node blue.
      const float boundary_gap = std::max(0.0F, center_distance - bubble_radius);
      const float surface_proximity = std::max(
        0.0F, 1.0F - boundary_gap / semantic_boundary_radius);
      const float weighted = frame.scores[i] * surface_proximity;
      if (weighted > score) score = weighted;
      associated = true;
    }
    return associated;
  }

  void updateTopoSemanticMemory(const TopoGraph::Ptr &topo)
  {
    if (!topo) return;
    std::optional<SemanticFrame> frame;
    {
      std::lock_guard<std::mutex> lock(semantic_mutex_);
      if (!semantic_frame_ || semantic_frame_->stamp_ns == last_semantic_applied_stamp_ns_) {
        return;
      }
      const double age_ms = static_cast<double>(std::llabs(
        get_clock()->now().nanoseconds() - semantic_frame_->stamp_ns)) / 1.0e6;
      if (age_ms > semantic_max_age_ms_) return;
      frame = semantic_frame_;
      last_semantic_applied_stamp_ns_ = semantic_frame_->stamp_ns;
    }

    std::unordered_set<TopoNode::Ptr> visited;
    const float alpha = static_cast<float>(std::clamp(semantic_node_ema_alpha_, 0.0, 1.0));
    for (const auto &entry : topo->reg_map_idx2ptr_) {
      if (!entry.second) continue;
      for (const auto &node : entry.second->topo_nodes_) {
        if (!node || node->is_viewpoint_ || !visited.insert(node).second) continue;
        float score = 0.0F;
        if (!semanticScore(*frame, node, score)) continue;
        topo->updateNodeSemantic(node, score, alpha, frame->stamp_ns);
      }
    }
    mergeSemanticMemory(topo->semanticMemorySnapshot());
  }

  static std_msgs::msg::ColorRGBA semanticColor(float normalized, bool enabled)
  {
    std_msgs::msg::ColorRGBA color;
    if (!enabled) {
      color.r = 0.15F;
      color.g = 0.20F;
      color.b = 0.55F;
      color.a = 1.0F;
      return color;
    }
    const float t = std::clamp(normalized, 0.0F, 1.0F);
    // Continuous cool-to-warm scale: blue -> cyan -> green -> yellow -> red.
    constexpr float anchors[5][3] = {
      {0.08F, 0.20F, 0.95F}, {0.05F, 0.85F, 0.95F}, {0.10F, 0.80F, 0.20F},
      {1.00F, 0.85F, 0.05F}, {0.95F, 0.05F, 0.03F}};
    const float scaled = t * 4.0F;
    const int index = std::min(3, static_cast<int>(scaled));
    const float local = scaled - static_cast<float>(index);
    color.r = anchors[index][0] * (1.0F - local) + anchors[index + 1][0] * local;
    color.g = anchors[index][1] * (1.0F - local) + anchors[index + 1][1] * local;
    color.b = anchors[index][2] * (1.0F - local) + anchors[index + 1][2] * local;
    color.a = 1.0F;
    return color;
  }

  struct PublishStats {
    std::size_t bubbles = 0;
    std::size_t skeleton_nodes = 0;
    std::size_t edges = 0;
    std::size_t witness_points_raw = 0;
    std::size_t witness_points = 0;
    double witness_shortcut_ms = 0.0;
    std::size_t raycast_clearance_queries = 0;
    std::size_t raycast_tested_segments = 0;
    std::size_t raycast_accepted_segments = 0;
    std::size_t semantic_nodes = 0;
    float semantic_max = 0.0F;
  };

  bool selectNextGoal(const std::vector<Eigen::Vector3f> &path, bool found,
                     float lookahead_m, Eigen::Vector3f &next_goal) const
  {
    if (!found || path.size() < 2) return false;
    if (graph_fixed_layer_) {
      const bool path_is_planar = std::all_of(
        path.begin(), path.end(), [this](const Eigen::Vector3f &point) {
          return std::abs(point.z() - static_cast<float>(graph_layer_z_)) < 1e-3F;
        });
      if (!path_is_planar) return false;
    }
    float nearest_distance_sq = std::numeric_limits<float>::infinity();
    float nearest_progress = 0.0F;
    float progress = 0.0F;
    for (std::size_t i = 1; i < path.size(); ++i) {
      const Eigen::Vector3f segment = path[i] - path[i - 1];
      const float length = segment.norm();
      if (length < 1e-4F) continue;
      const float t = std::clamp(
        (position_ - path[i - 1]).dot(segment) / (length * length), 0.0F, 1.0F);
      const Eigen::Vector3f projection = path[i - 1] + t * segment;
      const float distance_sq = (position_ - projection).squaredNorm();
      if (distance_sq < nearest_distance_sq) {
        nearest_distance_sq = distance_sq;
        nearest_progress = progress + t * length;
      }
      progress += length;
    }
    const float target_progress = std::min(
      progress, nearest_progress + lookahead_m);
    progress = 0.0F;
    next_goal = path.back();
    for (std::size_t i = 1; i < path.size(); ++i) {
      const Eigen::Vector3f segment = path[i] - path[i - 1];
      const float length = segment.norm();
      if (length < 1e-4F) continue;
      if (progress + length >= target_progress) {
        next_goal = path[i - 1] + ((target_progress - progress) / length) * segment;
        break;
      }
      progress += length;
    }
    if ((next_goal - position_).norm() < local_goal_min_advance_m_) {
      next_goal = path.back();
    }
    return next_goal.allFinite();
  }

  PublishStats publish(const TopoGraph::Ptr &topo,
                       const std::vector<TopoNode::Ptr> &path_nodes,
                       const std::vector<Eigen::Vector3f> &terminal_extension,
                       bool found, bool preserve_route_memory,
                       float effective_lookahead_m)
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
    skeleton_nodes.color.r = 0.1;
    skeleton_nodes.color.g = 0.9;
    skeleton_nodes.color.b = 0.2;
    skeleton_nodes.color.a = 1.0;

    std::optional<SemanticFrame> semantic_frame;
    {
      std::lock_guard<std::mutex> lock(semantic_mutex_);
      if (semantic_frame_) {
        const double age_ms = static_cast<double>(std::llabs(
          stampNanoseconds(skeleton_nodes.header.stamp) - semantic_frame_->stamp_ns)) / 1.0e6;
        if (age_ms <= semantic_max_age_ms_) semantic_frame = semantic_frame_;
      }
    }
    const bool semantic_available = semantic_frame.has_value();
    std::vector<float> semantic_scores;
    std::vector<bool> semantic_associated;

    visualization_msgs::msg::MarkerArray semantic_markers;
    visualization_msgs::msg::Marker semantic_voxels_marker;
    semantic_voxels_marker.header = skeleton_nodes.header;
    semantic_voxels_marker.ns = "epic_semantic_voxels";
    semantic_voxels_marker.id = 0;
    semantic_voxels_marker.type = visualization_msgs::msg::Marker::CUBE_LIST;
    semantic_voxels_marker.action = semantic_available ?
      visualization_msgs::msg::Marker::ADD : visualization_msgs::msg::Marker::DELETE;
    semantic_voxels_marker.scale.x = semantic_voxel_size_m_;
    semantic_voxels_marker.scale.y = semantic_voxel_size_m_;
    semantic_voxels_marker.scale.z = semantic_voxel_size_m_;
    semantic_voxels_marker.color.a = 0.65F;
    if (semantic_available && !semantic_frame->scores.empty()) {
      for (std::size_t i = 0; i < semantic_frame->surface_points_world.size(); ++i) {
        semantic_voxels_marker.points.push_back(toPoint(semantic_frame->surface_points_world[i]));
        const float normalized = semantic_frame->scores[i] / static_cast<float>(
          std::max(semantic_visualization_max_score_, 1e-5));
        semantic_voxels_marker.colors.push_back(semanticColor(normalized, true));
      }
    }
    semantic_markers.markers.push_back(std::move(semantic_voxels_marker));
    semantic_voxel_pub_->publish(semantic_markers);

    visualization_msgs::msg::Marker edges_marker = skeleton_nodes;
    edges_marker.ns = "epic_skeleton_edges";
    edges_marker.id = 2;
    edges_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edges_marker.scale.x = 0.045;
    edges_marker.color.r = 0.35;
    edges_marker.color.g = 0.55;
    edges_marker.color.b = 0.95;
    edges_marker.color.a = 0.75;
    edges_marker.points.clear();

    visualization_msgs::msg::Marker witness_edges = edges_marker;
    witness_edges.ns = "epic_edge_witness_paths";
    witness_edges.id = 3;
    witness_edges.scale.x = 0.025;
    witness_edges.color.r = 0.95;
    witness_edges.color.g = 0.85;
    witness_edges.color.b = 0.15;
    witness_edges.color.a = 0.9;
    witness_edges.points.clear();

    std::unordered_set<TopoNode::Ptr> visited;
    for (const auto &entry : topo->reg_map_idx2ptr_) {
      if (!entry.second) continue;
      for (const auto &node : entry.second->topo_nodes_) {
        if (!node || !visited.insert(node).second) continue;
        if (!node->is_viewpoint_) {
          skeleton_nodes.points.push_back(toPoint(node->center_));
          // Use the persistent TopoNode EMA rather than only the current
          // camera frame. A node keeps its semantic color after leaving the
          // camera FOV, matching the cost used by the global A* search.
          const float node_score = node->semantic_score_;
          const bool associated = node->semantic_observations_ > 0 &&
            std::isfinite(node_score);
          semantic_scores.push_back(node_score);
          semantic_associated.push_back(associated);
          if (associated) {
            ++stats.semantic_nodes;
            stats.semantic_max = std::max(stats.semantic_max, node_score);
          }
          stats.skeleton_nodes++;
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
    graph.markers.push_back(edges_marker);
    graph.markers.push_back(witness_edges);

    visualization_msgs::msg::Marker path_marker = edges_marker;
    path_marker.ns = "epic_astar_topology_path";
    path_marker.id = 4;
    path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    path_marker.points.clear();
    path_marker.scale.x = 0.10;
    path_marker.color.r = 0.1;
    path_marker.color.g = 0.8;
    path_marker.color.b = 1.0;
    path_marker.color.a = found ? 1.0 : 0.25;
    for (const auto &node : path_nodes) {
      if (node) path_marker.points.push_back(toPoint(node->center_));
    }
    graph.markers.push_back(path_marker);

    std::vector<Eigen::Vector3f> selected_witness_path;
    if (!use_edge_witness_path_) {
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
        if (path_it != from->paths_.end()) edge_path = path_it->second;
        if (edge_path.empty()) edge_path = {from->center_, to->center_};
        if ((edge_path.front() - from->center_).norm() >
            (edge_path.back() - from->center_).norm()) {
          std::reverse(edge_path.begin(), edge_path.end());
        }
        for (const auto &point : edge_path) {
          if (selected_witness_path.empty() ||
              (selected_witness_path.back() - point).norm() > 1e-3F) {
            selected_witness_path.push_back(point);
          }
        }
      }
    }
    // RHC memory ends at the persistent EPIC terminal.  The optional direct
    // terminal-to-goal extension is only a final execution hint; retaining it
    // would let the vehicle pass the terminal while the old terminal was
    // still considered "ahead" on the remembered route.
    const std::vector<Eigen::Vector3f> route_memory_path = selected_witness_path;
    for (const auto &point : terminal_extension) {
      if (selected_witness_path.empty() ||
          (selected_witness_path.back() - point).norm() > 1e-3F) {
        selected_witness_path.push_back(point);
      }
    }
    stats.witness_points_raw = selected_witness_path.size();

    // A* selects the topology route; a clearance raycast then removes only
    // geometrically redundant witness points. This queries every candidate
    // line segment against the same observed obstacle distance field used by
    // Bubble A*, instead of inferring visibility from endpoint bubble overlap.
    if (selected_witness_path.size() >= 2 && topo->parallel_bubble_astar_) {
      const auto shortcut_start = std::chrono::steady_clock::now();
      openseek_epic::RaycastShortcutStats shortcut_stats;
      const float minimum_clearance = static_cast<float>(
        topo->parallel_bubble_astar_->safe_distance_ +
        std::max(0.0, raycast_shortcut_clearance_margin_m_));
      auto clearance_query = [topo](const Eigen::Vector3f &point) {
        // Unknown voxels must not be treated as infinitely clear.  The
        // topology witness is already collision-checked; shortcut only when
        // the live ray map explicitly observed every sampled voxel as free.
        if (!topo->lidar_map_interface_->isKnownFree(point)) {
          return std::numeric_limits<double>::quiet_NaN();
        }
        return topo->lidar_map_interface_->getDisToOcc(point);
      };
      selected_witness_path = openseek_epic::farthestVisibleShortcut(
        selected_witness_path,
        static_cast<float>(std::max(0.01, raycast_shortcut_sample_step_m_)),
        minimum_clearance, clearance_query, &shortcut_stats);
      stats.witness_shortcut_ms = std::chrono::duration<double, std::milli>(
        std::chrono::steady_clock::now() - shortcut_start).count();
      stats.raycast_clearance_queries = shortcut_stats.clearance_queries;
      stats.raycast_tested_segments = shortcut_stats.tested_segments;
      stats.raycast_accepted_segments = shortcut_stats.accepted_segments;
    }
    stats.witness_points = selected_witness_path.size();
    if (found && route_memory_path.size() >= 2 && !preserve_route_memory) {
      // Bubble Planner's RHC principle: retain the geometrically valid route,
      // not TopoNode addresses that can be replaced by EPIC's incremental diff.
      last_witness_path_ = route_memory_path;
    }

    visualization_msgs::msg::Marker selected_witness = path_marker;
    selected_witness.ns = "epic_selected_witness_path";
    selected_witness.id = 5;
    selected_witness.scale.x = 0.14;
    selected_witness.color.r = 1.0;
    selected_witness.color.g = 0.2;
    selected_witness.color.b = 0.8;
    selected_witness.points.clear();
    for (const auto &point : selected_witness_path) {
      selected_witness.points.push_back(toPoint(point));
    }
    graph.markers.push_back(selected_witness);

    Eigen::Vector3f computed_next_goal;
    const bool computed_has_next_goal = selectNextGoal(
      selected_witness_path, found, effective_lookahead_m, computed_next_goal);
    if (found && !selected_witness_path.empty() && !computed_has_next_goal) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "EPIC found a topology route but rejected its YOPO subgoal; "
        "the witness path is not a valid fixed-height path");
    }
    const bool has_next_goal = computed_has_next_goal;
    const Eigen::Vector3f next_goal = computed_next_goal;
    visualization_msgs::msg::Marker next_goal_marker = skeleton_nodes;
    next_goal_marker.ns = "epic_yopo_next_goal";
    next_goal_marker.id = 6;
    next_goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    next_goal_marker.scale.x = 0.48;
    next_goal_marker.scale.y = 0.48;
    next_goal_marker.scale.z = 0.48;
    next_goal_marker.color.r = 1.0;
    next_goal_marker.color.g = 0.05;
    next_goal_marker.color.b = 0.75;
    next_goal_marker.color.a = 1.0;
    next_goal_marker.action = has_next_goal ? visualization_msgs::msg::Marker::ADD :
      visualization_msgs::msg::Marker::DELETE;
    if (has_next_goal) {
      next_goal_marker.pose.position = toPoint(next_goal);
      next_goal_marker.pose.orientation.w = 1.0;
      geometry_msgs::msg::PoseStamped next_goal_message;
      next_goal_message.header.stamp = now();
      next_goal_message.header.frame_id = next_goal_frame_;
      next_goal_message.pose.position = toPoint(next_goal);
      next_goal_message.pose.orientation.w = 1.0;
      next_goal_pub_->publish(next_goal_message);
      RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 1000,
        "[EPIC route] vehicle=(%.2f,%.2f,%.2f) local_yopo_goal=(%.2f,%.2f,%.2f) "
        "global_goal=(%.2f,%.2f,%.2f) speed=%.2f m/s lookahead=%.2f m "
        "plan_period=%d ms",
        position_.x(), position_.y(), position_.z(), next_goal.x(), next_goal.y(),
        next_goal.z(), goal_.x(), goal_.y(), goal_.z(), speed_mps_,
        effective_lookahead_m, route_plan_period_ms_);
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
    vehicle_marker.color.r = 0.0;
    vehicle_marker.color.g = 0.95;
    vehicle_marker.color.b = 1.0;
    vehicle_marker.color.a = 1.0;
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
    goal_marker.color.r = 1.0;
    goal_marker.color.g = 0.10;
    goal_marker.color.b = 0.05;
    goal_marker.color.a = 1.0;
    goal_marker.points.clear();
    graph.markers.push_back(goal_marker);

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
    vehicle_label.color.r = 0.0;
    vehicle_label.color.g = 0.95;
    vehicle_label.color.b = 1.0;
    vehicle_label.color.a = 1.0;
    vehicle_label.text = "UAV";
    vehicle_label.points.clear();
    graph.markers.push_back(vehicle_label);

    visualization_msgs::msg::Marker goal_label = vehicle_label;
    goal_label.ns = "epic_goal_label";
    goal_label.id = 10;
    const Eigen::Vector3f goal_label_position =
      goal_ + Eigen::Vector3f(0.0F, 0.0F, 0.8F);
    goal_label.pose.position = toPoint(goal_label_position);
    goal_label.color.r = 1.0;
    goal_label.color.g = 0.10;
    goal_label.color.b = 0.05;
    goal_label.text = "GOAL";
    goal_label.action = have_goal_ ? visualization_msgs::msg::Marker::ADD :
      visualization_msgs::msg::Marker::DELETE;
    graph.markers.push_back(goal_label);

    graph_pub_->publish(graph);

    visualization_msgs::msg::MarkerArray bubbles;
    visualization_msgs::msg::Marker delete_bubbles;
    delete_bubbles.header = skeleton_nodes.header;
    delete_bubbles.action = visualization_msgs::msg::Marker::DELETEALL;
    bubbles.markers.push_back(delete_bubbles);
    const auto bubble_snapshot = topo->getBubbleSnapshot();
    stats.bubbles = bubble_snapshot.size();
    int marker_id = 1;
    for (const auto &source : bubble_snapshot) {
      if (!source) continue;
      visualization_msgs::msg::Marker bubble;
      bubble.header = skeleton_nodes.header;
      bubble.ns = "epic_real_bubbles";
      bubble.id = marker_id++;
      bubble.type = visualization_msgs::msg::Marker::SPHERE;
      bubble.action = visualization_msgs::msg::Marker::ADD;
      bubble.pose.position = toPoint(source->center_);
      bubble.pose.orientation.w = 1.0;
      bubble.scale.x = 2.0 * source->radius_;
      bubble.scale.y = 2.0 * source->radius_;
      bubble.scale.z = 2.0 * source->radius_;
      bubble.color.r = 0.1;
      bubble.color.g = 0.8;
      bubble.color.b = 0.3;
      bubble.color.a = 0.10;
      bubbles.markers.push_back(std::move(bubble));
    }
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
    return stats;
  }

  std::string cloud_topic_, free_ray_topic_, odom_topic_, goal_topic_, next_goal_topic_, next_goal_frame_;
  std::string visualization_frame_;
  double map_margin_ = 20.0;
  bool graph_fixed_layer_ = true;
  bool reuse_graph_on_goal_ = true;
  bool graph_layer_initialized_ = false;
  double graph_layer_z_ = 1.6;
  double map_voxel_size_ = 0.25;
  double map_history_radius_m_ = 20.0;
  int map_max_points_ = 20000;
  double map_prune_distance_m_ = 0.5;
  double skeleton_rebuild_period_ms_ = 500.0;
  double local_goal_min_advance_m_ = 0.75;
  double local_goal_lookahead_m_ = 10.0;
  int route_plan_period_ms_ = 2000;
  double local_goal_reserve_m_ = 5.0;
  bool use_edge_witness_path_ = true;
  double raycast_shortcut_sample_step_m_ = 0.25;
  double raycast_shortcut_clearance_margin_m_ = 0.05;
  double goal_path_cost_weight_ = 0.2;
  double semantic_cost_weight_ = 1.0;
  double semantic_node_ema_alpha_ = 0.3;
  double semantic_visualization_max_score_ = 1.0;
  double clearance_cost_weight_ = 2.0;
  double clearance_target_m_ = 1.2;
  double previous_path_cost_factor_ = 0.0;
  double route_remap_distance_m_ = 1.25;
  double route_reuse_horizon_m_ = 6.0;
  double route_reuse_lateral_distance_m_ = 1.5;
  double route_terminal_release_distance_m_ = 1.0;
  double goal_connect_distance_m_ = 6.0;
  double goal_connect_timeout_ms_ = 20.0;
  double odom_reconnect_distance_m_ = 1.0;
  double odom_reconnect_yaw_deg_ = 20.0;
  double odom_fallback_radius_m_ = 15.0;
  int odom_fallback_candidates_ = 8;
  double odom_connect_timeout_ms_ = 3.0;
  double cloud_pose_tolerance_ms_ = 50.0;
  std::string semantic_heatmap_topic_, depth_image_topic_;
  double semantic_pose_tolerance_ms_ = 100.0;
  double semantic_max_age_ms_ = 1500.0;
  double semantic_camera_tx_ = 0.5;
  double semantic_camera_ty_ = 0.0;
  double semantic_camera_tz_ = -0.1;
  double semantic_horizontal_fov_deg_ = 90.0;
  double semantic_vertical_fov_deg_ = 60.0;
  double semantic_association_radius_m_ = 1.5;
  double semantic_voxel_size_m_ = 0.5;
  int update_period_ms_ = 100;
  fast_planner::LIOInterface::Ptr map_;
  ParallelBubbleAstar::Ptr astar_;
  TopoGraph::Ptr topo_;
  TopoGraph::Ptr graph_odom_topo_;
  std::vector<Eigen::Vector3f> last_topology_path_centers_;
  std::vector<Eigen::Vector3f> last_witness_path_;
  Eigen::Vector3f route_terminal_ = Eigen::Vector3f::Zero();
  bool have_route_terminal_ = false;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr free_ray_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr semantic_heatmap_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_image_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr bubble_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr semantic_voxel_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr next_goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  Eigen::Vector3f position_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f goal_ = Eigen::Vector3f::Zero();
  Eigen::Quaternionf orientation_ = Eigen::Quaternionf::Identity();
  std::deque<TimedPose> odom_history_;
  static constexpr std::size_t max_odom_history_size_ = 512;
  std::deque<DepthFrame> depth_history_;
  static constexpr std::size_t max_depth_history_size_ = 8;
  std::optional<SemanticFrame> semantic_frame_;
  std::int64_t last_semantic_applied_stamp_ns_ = 0;
  std::mutex semantic_mutex_;
  mutable std::mutex semantic_memory_mutex_;
  std::unordered_map<std::uint64_t, TopoSemanticRecord> semantic_memory_;
  std::chrono::steady_clock::time_point last_route_plan_time_;
  bool have_route_plan_time_ = false;
  bool route_plan_requested_ = false;
  float speed_mps_ = 0.0F;
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
  std::thread rebuild_thread_;
  std::atomic<bool> rebuild_running_{false};
  std::atomic<bool> shutting_down_{false};
  std::atomic<std::uint64_t> goal_generation_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<EpicGraphNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
