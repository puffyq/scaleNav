#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "coarse_grid_astar/coarse_grid_astar.h"

namespace {

geometry_msgs::msg::Point toPoint(const Eigen::Vector3f &value) {
  geometry_msgs::msg::Point point;
  point.x = value.x();
  point.y = value.y();
  point.z = value.z();
  return point;
}

}  // namespace

class CoarseGridAstarNode : public rclcpp::Node {
 public:
  CoarseGridAstarNode() : Node("coarse_grid_astar") {
    scalenav::CoarseGridAstarConfig config;
    config.resolution_m = static_cast<float>(declare_parameter<double>("resolution_m", 0.5));
    config.inflate_radius_m =
      static_cast<float>(declare_parameter<double>("inflate_radius_m", 0.61));
    config.origin_x_m = static_cast<float>(declare_parameter<double>("origin_x_m", -50.0));
    config.origin_y_m = static_cast<float>(declare_parameter<double>("origin_y_m", -10.0));
    config.origin_z_m = static_cast<float>(declare_parameter<double>("origin_z_m", 0.0));
    const double size_x = declare_parameter<double>("size_x_m", 100.0);
    const double size_y = declare_parameter<double>("size_y_m", 170.0);
    const double size_z = declare_parameter<double>("size_z_m", 10.0);
    config.width = std::max(2, static_cast<int>(std::lround(size_x / config.resolution_m)));
    config.height = std::max(2, static_cast<int>(std::lround(size_y / config.resolution_m)));
    config.depth = std::max(2, static_cast<int>(std::lround(size_z / config.resolution_m)));
    lookahead_m_ = declare_parameter<double>("local_goal_lookahead_m", 15.0);
    config.local_radius_m =
      static_cast<float>(declare_parameter<double>("local_radius_m", 20.0));
    grid_ = std::make_unique<scalenav::CoarseGridAstar>(config);

    const auto cloud_topic = declare_parameter<std::string>("cloud_topic", "/depth/points");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/sim/odom");
    const auto goal_topic = declare_parameter<std::string>("goal_topic", "/goal_pose");
    const auto next_goal_topic =
      declare_parameter<std::string>("next_goal_topic", "/scalenav/local_goal");
    frame_id_ = declare_parameter<std::string>("frame_id", "world_enu");
    const int plan_period_ms = declare_parameter<int>("plan_period_ms", 100);

    cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { onCloud(message); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, rclcpp::SensorDataQoS(),
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { onOdom(message); });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) { onGoal(message); });
    local_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(next_goal_topic, 10);
    path_pub_ = create_publisher<nav_msgs::msg::Path>("/scalenav/coarse_astar/path", 1);
    grid_pub_ =
      create_publisher<nav_msgs::msg::OccupancyGrid>("/scalenav/coarse_astar/occupancy", 1);
    voxel_pub_ =
      create_publisher<visualization_msgs::msg::MarkerArray>("/scalenav/coarse_astar/voxels", 1);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(std::max(plan_period_ms, 20)),
      [this]() { plan(); });

    RCLCPP_INFO(
      get_logger(),
      "online coarse 3D A*: resolution=%.2f m inflate=%.2f m local_radius=%.1f m "
      "grid=%dx%dx%d z=[%.2f, %.2f] lookahead=%.1f m",
      config.resolution_m, config.inflate_radius_m, config.local_radius_m,
      config.width, config.height, config.depth, config.origin_z_m,
      config.origin_z_m + static_cast<float>(config.depth) * config.resolution_m,
      lookahead_m_);
  }

 private:
  void onOdom(const nav_msgs::msg::Odometry::ConstSharedPtr &message) {
    std::lock_guard<std::mutex> lock(mutex_);
    position_ = Eigen::Vector3f(
      static_cast<float>(message->pose.pose.position.x),
      static_cast<float>(message->pose.pose.position.y),
      static_cast<float>(message->pose.pose.position.z));
    orientation_ = Eigen::Quaternionf(
      static_cast<float>(message->pose.pose.orientation.w),
      static_cast<float>(message->pose.pose.orientation.x),
      static_cast<float>(message->pose.pose.orientation.y),
      static_cast<float>(message->pose.pose.orientation.z));
    orientation_.normalize();
    have_odom_ = true;
  }

  void onGoal(const geometry_msgs::msg::PoseStamped::ConstSharedPtr &message) {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_ = Eigen::Vector3f(
      static_cast<float>(message->pose.position.x),
      static_cast<float>(message->pose.position.y),
      static_cast<float>(message->pose.position.z));
    have_goal_ = true;
  }

  void onCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &message) {
    pcl::PointCloud<pcl::PointXYZ> cloud_body;
    pcl::fromROSMsg(*message, cloud_body);
    Eigen::Vector3f position;
    Eigen::Quaternionf orientation;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_odom_ || cloud_body.empty()) return;
      position = position_;
      orientation = orientation_;
    }
    std::size_t added = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      for (const auto &point : cloud_body.points) {
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) {
          continue;
        }
        const Eigen::Vector3f world =
          position + orientation * Eigen::Vector3f(point.x, point.y, point.z);
        if (grid_->markWorld(world.x(), world.y(), world.z())) ++added;
      }
    }
    if (added > 0) {
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "accumulated %zu new occupied cells (total %zu)", added,
        grid_->occupiedCount());
    }
  }

  void plan() {
    scalenav::CoarseGridSearchResult result;
    Eigen::Vector3f local_goal = Eigen::Vector3f::Zero();
    bool have_local_goal = false;
    std::size_t occupied = 0;
    std::size_t blocked = 0;
    std::vector<int8_t> occupancy;
    std::vector<Eigen::Vector3f> voxels;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (!have_odom_ || !have_goal_) return;
      Eigen::Vector3f position = position_;
      Eigen::Vector3f goal = goal_;
      grid_->retainAround(position.x(), position.y(), position.z());
      result = grid_->search(position, goal);
      occupied = grid_->occupiedCount();
      blocked = grid_->blockedCount();
      occupancy = grid_->occupancyGrid();
      voxels = grid_->occupiedCenters();
      Eigen::Vector3f candidate = position;
      bool useful = false;
      if (result.success && result.path.size() >= 2) {
        candidate = grid_->progressLookahead(
          result.path, position, goal, static_cast<float>(lookahead_m_));
        useful = grid_->isForwardGoal(position, goal, candidate);
      }
      if (useful) {
        last_local_goal_ = candidate;
        have_last_local_goal_ = true;
        local_goal = candidate;
        have_local_goal = true;
      } else if (have_last_local_goal_ &&
                 grid_->isForwardGoal(position, goal, last_local_goal_)) {
        local_goal = last_local_goal_;
        have_local_goal = true;
      } else {
        have_last_local_goal_ = false;
        local_goal = position;
        have_local_goal = false;
      }
    }

    if (have_local_goal) {
      geometry_msgs::msg::PoseStamped goal_message;
      goal_message.header.stamp = now();
      goal_message.header.frame_id = frame_id_;
      goal_message.pose.position = toPoint(local_goal);
      goal_message.pose.orientation.w = 1.0;
      local_goal_pub_->publish(goal_message);
    }
    publishPath(result, have_local_goal);
    publishOccupancy(occupancy);
    publishVoxels(voxels, local_goal, have_local_goal);

    RCLCPP_INFO_THROTTLE(
      get_logger(), *get_clock(), 1000,
      "online 3D A* %s length=%.1f m occupied=%zu blocked=%zu "
      "horizon=(%.1f, %.1f, %.1f) local_goal=(%.1f, %.1f, %.1f)",
      result.reason.c_str(), static_cast<double>(result.path_length_m),
      occupied, blocked,
      static_cast<double>(result.clipped_goal.x()),
      static_cast<double>(result.clipped_goal.y()),
      static_cast<double>(result.clipped_goal.z()),
      static_cast<double>(local_goal.x()), static_cast<double>(local_goal.y()),
      static_cast<double>(local_goal.z()));
  }

  void publishPath(const scalenav::CoarseGridSearchResult &result, bool valid) {
    nav_msgs::msg::Path path;
    path.header.stamp = now();
    path.header.frame_id = frame_id_;
    if (valid) {
      path.poses.reserve(result.path.size());
      for (const auto &point : result.path) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = path.header;
        pose.pose.position = toPoint(point);
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
      }
    }
    path_pub_->publish(path);
  }

  void publishVoxels(const std::vector<Eigen::Vector3f> &voxels,
                     const Eigen::Vector3f &local_goal, bool have_local_goal) {
    visualization_msgs::msg::MarkerArray array;
    visualization_msgs::msg::Marker cubes;
    cubes.header.stamp = now();
    cubes.header.frame_id = frame_id_;
    cubes.ns = "coarse_occupied";
    cubes.id = 0;
    cubes.type = visualization_msgs::msg::Marker::CUBE_LIST;
    cubes.action = visualization_msgs::msg::Marker::ADD;
    cubes.pose.orientation.w = 1.0;
    const float scale = std::max(0.05F, grid_->config().resolution_m * 0.9F);
    cubes.scale.x = scale;
    cubes.scale.y = scale;
    cubes.scale.z = scale;
    cubes.color.r = 0.95F;
    cubes.color.g = 0.45F;
    cubes.color.b = 0.12F;
    cubes.color.a = 0.72F;
    cubes.points.reserve(voxels.size());
    for (const auto &center : voxels) {
      cubes.points.push_back(toPoint(center));
    }
    array.markers.push_back(cubes);

    visualization_msgs::msg::Marker goal;
    goal.header = cubes.header;
    goal.ns = "coarse_local_goal";
    goal.id = 1;
    goal.type = visualization_msgs::msg::Marker::SPHERE;
    goal.action = have_local_goal ? visualization_msgs::msg::Marker::ADD :
      visualization_msgs::msg::Marker::DELETE;
    goal.pose.position = toPoint(local_goal);
    goal.pose.orientation.w = 1.0;
    goal.scale.x = 0.9;
    goal.scale.y = 0.9;
    goal.scale.z = 0.9;
    goal.color.r = 0.92F;
    goal.color.g = 0.18F;
    goal.color.b = 0.62F;
    goal.color.a = 0.95F;
    array.markers.push_back(goal);
    voxel_pub_->publish(array);
  }

  void publishOccupancy(const std::vector<int8_t> &data) {
    nav_msgs::msg::OccupancyGrid grid_msg;
    grid_msg.header.stamp = now();
    grid_msg.header.frame_id = frame_id_;
    const auto &config = grid_->config();
    grid_msg.info.resolution = config.resolution_m;
    grid_msg.info.width = static_cast<std::uint32_t>(config.width);
    grid_msg.info.height = static_cast<std::uint32_t>(config.height);
    grid_msg.info.origin.position.x = config.origin_x_m;
    grid_msg.info.origin.position.y = config.origin_y_m;
    grid_msg.info.origin.position.z = config.origin_z_m;
    grid_msg.info.origin.orientation.w = 1.0;
    grid_msg.data.assign(data.begin(), data.end());
    grid_pub_->publish(grid_msg);
  }

  std::mutex mutex_;
  std::unique_ptr<scalenav::CoarseGridAstar> grid_;
  Eigen::Vector3f position_ = Eigen::Vector3f::Zero();
  Eigen::Vector3f goal_ = Eigen::Vector3f::Zero();
  Eigen::Quaternionf orientation_ = Eigen::Quaternionf::Identity();
  bool have_odom_ = false;
  bool have_goal_ = false;
  bool have_last_local_goal_ = false;
  Eigen::Vector3f last_local_goal_ = Eigen::Vector3f::Zero();
  double lookahead_m_ = 15.0;
  std::string frame_id_ = "world_enu";
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_goal_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr voxel_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CoarseGridAstarNode>());
  rclcpp::shutdown();
  return 0;
}
