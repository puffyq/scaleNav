#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <unordered_set>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <ros/ros.h>

namespace fast_planner {

using PointType = pcl::PointXYZ;
using PointVector = std::vector<PointType, Eigen::aligned_allocator<PointType>>;

struct LIOInterfaceParam {
  Eigen::Vector3f global_box_min_boundary_{-50.0F, -50.0F, -20.0F};
  Eigen::Vector3f global_box_max_boundary_{50.0F, 50.0F, 20.0F};
  Eigen::Vector3f global_map_min_boundary_{-50.0F, -50.0F, -20.0F};
  Eigen::Vector3f global_map_max_boundary_{50.0F, 50.0F, 20.0F};
  int box_num_ = 1;
  int dead_area_num_ = 0;
  std::vector<Eigen::Vector3f> global_box_min_boundary_vec_;
  std::vector<Eigen::Vector3f> global_box_max_boundary_vec_;
  std::vector<Eigen::Vector3f> dead_area_min_boundary_vec_;
  std::vector<Eigen::Vector3f> dead_area_max_boundary_vec_;
  double lidar_pitch_ = 0.0;
  double max_ray_length_ = 20.0;
  double fov_up = 0.5;
  double fov_down = -0.5;
  double fov_vp_up = 0.5;
  double fov_vp_down = -0.5;
};

struct LIOInterfaceData {
  bool map_update = false;
  bool first_map_flag_ = true;
  Eigen::Vector3f lidar_pose_ = Eigen::Vector3f::Zero();
  Eigen::Quaternionf lidar_q_ = Eigen::Quaternionf::Identity();
  Eigen::Vector3f lidar_vel_ = Eigen::Vector3f::Zero();
  pcl::PointCloud<PointType> lidar_cloud_;
};

class LIOInterface {
 public:
  using Ptr = std::shared_ptr<LIOInterface>;

  LIOInterface() : lp_(std::make_unique<LIOInterfaceParam>()),
                   ld_(std::make_unique<LIOInterfaceData>()),
                   cloud_(std::make_shared<pcl::PointCloud<PointType>>()) {}

  void init(ros::NodeHandle &) {}

  void configureBounds(const Eigen::Vector3f &min_bound,
                       const Eigen::Vector3f &max_bound) {
    lp_->global_box_min_boundary_ = min_bound;
    lp_->global_box_max_boundary_ = max_bound;
    lp_->global_map_min_boundary_ = min_bound;
    lp_->global_map_max_boundary_ = max_bound;
    lp_->global_box_min_boundary_vec_ = {min_bound};
    lp_->global_box_max_boundary_vec_ = {max_bound};
    lp_->box_num_ = 1;
  }

  void configureStorage(float voxel_size, float history_radius,
                        std::size_t max_points, float prune_distance) {
    std::lock_guard<std::mutex> lock(mutex_);
    voxel_size_ = std::max(voxel_size, 0.05F);
    history_radius_ = std::max(history_radius, 1.0F);
    max_points_ = std::max<std::size_t>(max_points, 1000);
    prune_distance_ = std::max(prune_distance, 0.1F);
  }

  bool updateCloudWorld(const pcl::PointCloud<PointType> &points_world,
                        const Eigen::Vector3f &pose,
                        const Eigen::Quaternionf &orientation) {
    std::lock_guard<std::mutex> lock(mutex_);
    ld_->map_update = true;
    ld_->first_map_flag_ = false;
    ld_->lidar_pose_ = pose;
    ld_->lidar_q_ = orientation;
    ld_->lidar_cloud_ = points_world;
    bool changed = false;
    for (const auto &point : points_world.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }
      const VoxelKey key{
        static_cast<int>(std::floor(point.x / voxel_size_)),
        static_cast<int>(std::floor(point.y / voxel_size_)),
        static_cast<int>(std::floor(point.z / voxel_size_))};
      if (occupied_voxels_.insert(key).second) {
        cloud_->push_back(point);
        changed = true;
      }
    }
    const bool moved_for_prune = !have_prune_pose_ ||
      (pose - last_prune_pose_).norm() >= prune_distance_;
    if (moved_for_prune || cloud_->size() > max_points_) {
      pruneLocked(pose);
    }
    // The live accumulator is never queried. Rebuilding a PCL KD-tree here
    // would make every camera frame O(all historical points). Graph workers
    // build one immutable KD-tree from accumulatedCloudSnapshot() instead.
    return changed;
  }

  pcl::PointCloud<PointType> accumulatedCloudSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return *cloud_;
  }

  pcl::PointCloud<PointType> latestCloudSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ld_->lidar_cloud_;
  }

  void loadSnapshot(const pcl::PointCloud<PointType> &accumulated_world,
                    const pcl::PointCloud<PointType> &latest_world,
                    const Eigen::Vector3f &pose,
                    const Eigen::Quaternionf &orientation) {
    std::lock_guard<std::mutex> lock(mutex_);
    ld_->map_update = true;
    ld_->first_map_flag_ = false;
    ld_->lidar_pose_ = pose;
    ld_->lidar_q_ = orientation;
    ld_->lidar_cloud_ = latest_world;
    cloud_ = std::make_shared<pcl::PointCloud<PointType>>(accumulated_world);
    occupied_voxels_.clear();
    occupied_voxels_.reserve(cloud_->size());
    for (const auto &point : cloud_->points) {
      occupied_voxels_.insert(VoxelKey{
        static_cast<int>(std::floor(point.x / voxel_size_)),
        static_cast<int>(std::floor(point.y / voxel_size_)),
        static_cast<int>(std::floor(point.z / voxel_size_))});
    }
    if (!cloud_->empty()) kd_.setInputCloud(cloud_);
  }

  bool IsInBox(const Eigen::Vector3f &pos) const {
    return (pos.array() >= lp_->global_box_min_boundary_.array()).all() &&
           (pos.array() <= lp_->global_box_max_boundary_.array()).all();
  }
  bool IsInBox(const PointType &pos) const {
    return IsInBox(Eigen::Vector3f(pos.x, pos.y, pos.z));
  }
  bool IsInMap(const Eigen::Vector3f &pos) const { return IsInBox(pos); }
  bool IsInMap(const PointType &pos) const { return IsInBox(pos); }

  std::size_t pointCount() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return cloud_->size();
  }

  double getDisToOcc(const PointType &point) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (cloud_->empty()) return lp_->max_ray_length_;
    std::vector<int> indices(1);
    std::vector<float> distances(1);
    if (kd_.nearestKSearch(point, 1, indices, distances) == 0) {
      return lp_->max_ray_length_;
    }
    return std::sqrt(std::max(0.0F, distances.front()));
  }
  double getDisToOcc(const Eigen::Vector3f &point) const {
    return getDisToOcc(PointType(point.x(), point.y(), point.z()));
  }
  double getDisToOcc(const Eigen::Vector3d &point) const {
    return getDisToOcc(Eigen::Vector3f(point.cast<float>()));
  }

  void KNN(const PointType &point, int k, PointVector &points,
           std::vector<float> &distances) const {
    std::lock_guard<std::mutex> lock(mutex_);
    points.clear();
    distances.clear();
    if (cloud_->empty()) return;
    std::vector<int> indices(static_cast<size_t>(std::max(k, 1)));
    distances.resize(indices.size());
    const int count = kd_.nearestKSearch(point, static_cast<int>(indices.size()),
                                         indices, distances);
    points.reserve(static_cast<size_t>(std::max(count, 0)));
    for (int i = 0; i < count; ++i) points.push_back(cloud_->points[indices[i]]);
    distances.resize(static_cast<size_t>(std::max(count, 0)));
  }

  void boxSearch(const Eigen::Vector3f &min_bound,
                 const Eigen::Vector3f &max_bound,
                 PointVector &points) const {
    std::lock_guard<std::mutex> lock(mutex_);
    points.clear();
    for (const auto &point : cloud_->points) {
      if (point.x >= min_bound.x() && point.x <= max_bound.x() &&
          point.y >= min_bound.y() && point.y <= max_bound.y() &&
          point.z >= min_bound.z() && point.z <= max_bound.z()) {
        points.push_back(point);
      }
    }
  }

  std::unique_ptr<LIOInterfaceParam> lp_;
  std::unique_ptr<LIOInterfaceData> ld_;

 private:
  struct VoxelKey {
    int x;
    int y;
    int z;
    bool operator==(const VoxelKey &other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };

  struct VoxelKeyHash {
    std::size_t operator()(const VoxelKey &key) const {
      std::size_t seed = std::hash<int>{}(key.x);
      seed ^= std::hash<int>{}(key.y) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      seed ^= std::hash<int>{}(key.z) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
      return seed;
    }
  };

  void pruneLocked(const Eigen::Vector3f &pose) {
    std::vector<std::pair<float, PointType>> retained;
    retained.reserve(cloud_->size());
    const float radius_sq = history_radius_ * history_radius_;
    for (const auto &point : cloud_->points) {
      const Eigen::Vector3f delta(point.x - pose.x(), point.y - pose.y(), point.z - pose.z());
      const float distance_sq = delta.squaredNorm();
      if (distance_sq <= radius_sq) retained.emplace_back(distance_sq, point);
    }
    if (retained.size() > max_points_) {
      std::nth_element(
        retained.begin(), retained.begin() + static_cast<std::ptrdiff_t>(max_points_),
        retained.end(),
        [](const auto &left, const auto &right) { return left.first < right.first; });
      retained.resize(max_points_);
    }
    cloud_->clear();
    cloud_->reserve(retained.size());
    occupied_voxels_.clear();
    occupied_voxels_.reserve(retained.size());
    for (const auto &entry : retained) {
      const auto &point = entry.second;
      cloud_->push_back(point);
      occupied_voxels_.insert(VoxelKey{
        static_cast<int>(std::floor(point.x / voxel_size_)),
        static_cast<int>(std::floor(point.y / voxel_size_)),
        static_cast<int>(std::floor(point.z / voxel_size_))});
    }
    last_prune_pose_ = pose;
    have_prune_pose_ = true;
  }

  float voxel_size_ = 0.25F;
  float history_radius_ = 20.0F;
  float prune_distance_ = 0.5F;
  std::size_t max_points_ = 20000;
  Eigen::Vector3f last_prune_pose_ = Eigen::Vector3f::Zero();
  bool have_prune_pose_ = false;
  mutable std::mutex mutex_;
  pcl::PointCloud<PointType>::Ptr cloud_;
  std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels_;
  mutable pcl::KdTreeFLANN<PointType> kd_;
};

}  // namespace fast_planner
