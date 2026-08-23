#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <unordered_map>
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

  bool expandBounds(const Eigen::Vector3f &min_bound,
                    const Eigen::Vector3f &max_bound) {
    const Eigen::Vector3f expanded_min =
        lp_->global_box_min_boundary_.cwiseMin(min_bound);
    const Eigen::Vector3f expanded_max =
        lp_->global_box_max_boundary_.cwiseMax(max_bound);
    if (expanded_min.isApprox(lp_->global_box_min_boundary_) &&
        expanded_max.isApprox(lp_->global_box_max_boundary_)) {
      return false;
    }
    configureBounds(expanded_min, expanded_max);
    return true;
  }

  void configureStorage(float voxel_size, float history_radius,
                        std::size_t max_points, float prune_distance) {
    std::lock_guard<std::mutex> lock(mutex_);
    voxel_size_ = std::max(voxel_size, 0.05F);
    history_radius_ = std::max(history_radius, 1.0F);
    max_points_ = std::max<std::size_t>(max_points, 1000);
    max_free_voxels_ = std::max<std::size_t>(max_points_ * 8U, 10000U);
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
    last_hit_voxels_ = 0;
    last_free_voxels_ = 0;
    last_carved_voxels_ = 0;
    bool changed = false;
    // A depth return certifies the ray from the sensor to the measured surface
    // as free.  Without carving this free prefix, an old occupied voxel can
    // remain in the rolling map after the vehicle has moved, eventually
    // placing a ghost obstacle on top of the odometry query point.
    std::unordered_map<VoxelKey, Eigen::Vector3f, VoxelKeyHash> hit_voxels;
    std::unordered_set<VoxelKey, VoxelKeyHash> free_voxels;
    hit_voxels.reserve(points_world.points.size());
    free_voxels.reserve(points_world.points.size() * 4U);
    for (const auto &point : points_world.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }
      const VoxelKey key{
        static_cast<int>(std::floor(point.x / voxel_size_)),
        static_cast<int>(std::floor(point.y / voxel_size_)),
        static_cast<int>(std::floor(point.z / voxel_size_))};
      hit_voxels.try_emplace(key, point.x, point.y, point.z);
      if (occupied_voxels_.insert(key).second) {
        cloud_->push_back(point);
        changed = true;
      }
    }
    latest_hit_voxels_.clear();
    latest_hit_voxels_.reserve(hit_voxels.size());
    for (const auto &entry : hit_voxels) latest_hit_voxels_.insert(entry.first);

    // Neighboring pixels commonly terminate in the same map voxel. Cast one
    // ray per endpoint voxel so this update scales with map resolution rather
    // than raw image resolution.
    for (const auto &[key, endpoint] : hit_voxels) {
      (void)key;
      const Eigen::Vector3f ray = endpoint - pose;
      const float length = ray.norm();
      if (length <= voxel_size_ * 0.5F) continue;
      const int steps = std::max(1, static_cast<int>(std::ceil(
        length / std::max(0.05F, voxel_size_ * 0.5F))));
      const Eigen::Vector3f direction = ray / length;
      // Exclude the hit voxel itself.  It is an occupied surface, not free
      // space, and is protected by hit_voxels below in case another ray
      // traverses the same voxel.
      for (int step = 1; step < steps; ++step) {
        const Eigen::Vector3f sample = pose + direction *
          (length * static_cast<float>(step) / static_cast<float>(steps));
        free_voxels.insert(VoxelKey{
          static_cast<int>(std::floor(sample.x() / voxel_size_)),
          static_cast<int>(std::floor(sample.y() / voxel_size_)),
          static_cast<int>(std::floor(sample.z() / voxel_size_))});
      }
    }

    // Keep explicitly observed free voxels as a rolling memory. A hit in the
    // current frame invalidates an older free-space claim for that voxel.
    for (const auto &entry : hit_voxels) {
      if (observed_free_voxels_.erase(entry.first) > 0) changed = true;
    }
    for (const auto &key : free_voxels) {
      if (hit_voxels.find(key) == hit_voxels.end() &&
          observed_free_voxels_.insert(key).second) {
        changed = true;
      }
    }

    // Remove only voxels that are explicitly observed free and are not a
    // measured endpoint in this frame.  Rebuild the compact cloud so its
    // contents and occupied_voxels_ remain exactly consistent.
    for (const auto &key : free_voxels) {
      if (hit_voxels.find(key) != hit_voxels.end()) continue;
      const auto erased = occupied_voxels_.erase(key);
      last_carved_voxels_ += erased;
      changed = changed || erased > 0;
    }
    const bool moved_for_prune = !have_prune_pose_ ||
      (pose - last_prune_pose_).norm() >= prune_distance_;
    const bool needs_prune = moved_for_prune || cloud_->size() > max_points_;
    // A repeated frame that touches no new occupied/free voxel does not alter
    // geometry. Avoid rebuilding the PCL KD-tree and rescanning the rolling
    // cloud on the hot callback path.
    if (!changed && !needs_prune) {
      last_hit_voxels_ = hit_voxels.size();
      last_free_voxels_ = free_voxels.size();
      return false;
    }
    if (!free_voxels.empty()) {
      pcl::PointCloud<PointType>::Ptr carved(new pcl::PointCloud<PointType>);
      carved->reserve(cloud_->size());
      for (const auto &point : cloud_->points) {
        const VoxelKey key{
          static_cast<int>(std::floor(point.x / voxel_size_)),
          static_cast<int>(std::floor(point.y / voxel_size_)),
          static_cast<int>(std::floor(point.z / voxel_size_))};
        if (occupied_voxels_.find(key) != occupied_voxels_.end()) {
          carved->push_back(point);
        } else {
          changed = true;
        }
      }
      cloud_ = std::move(carved);
    }
    last_hit_voxels_ = hit_voxels.size();
    last_free_voxels_ = free_voxels.size();
    if (moved_for_prune || cloud_->size() > max_points_) {
      pruneLocked(pose);
    }
    const bool moved_for_free_prune = !have_free_prune_pose_ ||
      (pose - free_last_prune_pose_).norm() >= prune_distance_;
    if (moved_for_free_prune || observed_free_voxels_.size() > max_free_voxels_) {
      pruneFreeLocked(pose);
    }
    // Clearance queries are made by the live planner while the cloud callback
    // updates this map. Keep the KD-tree consistent with the compact cloud;
    // otherwise carved voxels remain visible as ghost obstacles until the next
    // skeleton snapshot.
    rebuildKdTreeLocked();
    return changed;
  }

  // Add free-space evidence for rays that reached the sensor far plane. These
  // samples have no occupied endpoint and therefore must not enter cloud_.
  bool updateFreeRaysWorld(const pcl::PointCloud<PointType> &ray_endpoints_world,
                           const Eigen::Vector3f &pose,
                           const Eigen::Quaternionf &orientation) {
    std::lock_guard<std::mutex> lock(mutex_);
    ld_->map_update = true;
    ld_->first_map_flag_ = false;
    ld_->lidar_pose_ = pose;
    ld_->lidar_q_ = orientation;
    last_hit_voxels_ = 0;
    last_free_voxels_ = 0;
    last_carved_voxels_ = 0;
    std::unordered_set<VoxelKey, VoxelKeyHash> free_voxels;
    free_voxels.reserve(ray_endpoints_world.points.size() * 8U);
    for (const auto &endpoint : ray_endpoints_world.points) {
      if (!std::isfinite(endpoint.x) || !std::isfinite(endpoint.y) ||
          !std::isfinite(endpoint.z)) continue;
      const Eigen::Vector3f ray(endpoint.x - pose.x(), endpoint.y - pose.y(),
                                endpoint.z - pose.z());
      const float length = ray.norm();
      if (length <= voxel_size_ * 0.5F) continue;
      const int steps = std::max(1, static_cast<int>(std::ceil(
        length / std::max(0.05F, voxel_size_))));
      const Eigen::Vector3f direction = ray / length;
      for (int step = 1; step < steps; ++step) {
        const Eigen::Vector3f sample = pose + direction *
          (length * static_cast<float>(step) / static_cast<float>(steps));
        free_voxels.insert(VoxelKey{
          static_cast<int>(std::floor(sample.x() / voxel_size_)),
          static_cast<int>(std::floor(sample.y() / voxel_size_)),
          static_cast<int>(std::floor(sample.z() / voxel_size_))});
      }
    }
    bool changed = false;
    for (const auto &key : free_voxels) {
      if (latest_hit_voxels_.find(key) != latest_hit_voxels_.end()) continue;
      if (observed_free_voxels_.insert(key).second) changed = true;
      if (occupied_voxels_.erase(key) > 0) {
        last_carved_voxels_++;
        changed = true;
      }
    }
    const bool moved_for_prune = !have_prune_pose_ ||
      (pose - last_prune_pose_).norm() >= prune_distance_;
    const bool needs_prune = moved_for_prune || cloud_->size() > max_points_;
    // Far-plane rays are published for every depth frame. If they only repeat
    // already-known free voxels, leave the compact cloud and KD-tree untouched
    // so the point-cloud callback is not serialized behind an O(map_size) copy.
    if (!changed && !needs_prune &&
        (!have_free_prune_pose_ ||
         (pose - free_last_prune_pose_).norm() < prune_distance_) &&
        observed_free_voxels_.size() <= max_free_voxels_) {
      last_free_voxels_ = free_voxels.size();
      return false;
    }
    if (!free_voxels.empty()) {
      pcl::PointCloud<PointType>::Ptr retained(new pcl::PointCloud<PointType>);
      retained->reserve(cloud_->size());
      for (const auto &point : cloud_->points) {
        const VoxelKey key{
          static_cast<int>(std::floor(point.x / voxel_size_)),
          static_cast<int>(std::floor(point.y / voxel_size_)),
          static_cast<int>(std::floor(point.z / voxel_size_))};
        if (occupied_voxels_.find(key) != occupied_voxels_.end()) retained->push_back(point);
      }
      cloud_ = std::move(retained);
    }
    last_free_voxels_ = free_voxels.size();
    if (moved_for_prune || cloud_->size() > max_points_) pruneLocked(pose);
    const bool moved_for_free_prune = !have_free_prune_pose_ ||
      (pose - free_last_prune_pose_).norm() >= prune_distance_;
    if (moved_for_free_prune || observed_free_voxels_.size() > max_free_voxels_)
      pruneFreeLocked(pose);
    rebuildKdTreeLocked();
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

  Eigen::Vector3f poseSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ld_->lidar_pose_;
  }

  pcl::PointCloud<PointType> freeSpaceSnapshot() const {
    std::lock_guard<std::mutex> lock(mutex_);
    pcl::PointCloud<PointType> snapshot;
    snapshot.reserve(observed_free_voxels_.size());
    for (const auto &key : observed_free_voxels_) {
      snapshot.push_back(PointType(
        (static_cast<float>(key.x) + 0.5F) * voxel_size_,
        (static_cast<float>(key.y) + 0.5F) * voxel_size_,
        (static_cast<float>(key.z) + 0.5F) * voxel_size_));
    }
    return snapshot;
  }

  void loadSnapshot(const pcl::PointCloud<PointType> &accumulated_world,
                    const pcl::PointCloud<PointType> &latest_world,
                    const pcl::PointCloud<PointType> &free_world,
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
    observed_free_voxels_.clear();
    observed_free_voxels_.reserve(free_world.size());
    for (const auto &point : free_world.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) continue;
      observed_free_voxels_.insert(VoxelKey{
        static_cast<int>(std::floor(point.x / voxel_size_)),
        static_cast<int>(std::floor(point.y / voxel_size_)),
        static_cast<int>(std::floor(point.z / voxel_size_))});
    }
    last_prune_pose_ = pose;
    have_prune_pose_ = true;
    free_last_prune_pose_ = pose;
    have_free_prune_pose_ = true;
    rebuildKdTreeLocked();
  }

  void loadSnapshot(const pcl::PointCloud<PointType> &accumulated_world,
                    const pcl::PointCloud<PointType> &latest_world,
                    const Eigen::Vector3f &pose,
                    const Eigen::Quaternionf &orientation) {
    // Incremental EPIC rebuilds refresh occupied/latest clouds from the same
    // live map. Preserve ray-carved free-space evidence here; clearing it on
    // every skeleton update makes previously observed corridors look unknown
    // again and is both a correctness and replanning regression.
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
    for (const auto &key : occupied_voxels_) observed_free_voxels_.erase(key);
    last_prune_pose_ = pose;
    have_prune_pose_ = true;
    free_last_prune_pose_ = pose;
    have_free_prune_pose_ = true;
    rebuildKdTreeLocked();
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

  float voxelSize() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return voxel_size_;
  }

  void lastRayCarvingStats(std::size_t &hit_voxels,
                           std::size_t &free_voxels,
                           std::size_t &carved_voxels) const {
    std::lock_guard<std::mutex> lock(mutex_);
    hit_voxels = last_hit_voxels_;
    free_voxels = last_free_voxels_;
    carved_voxels = last_carved_voxels_;
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

  // A large distance from getDisToOcc() is not sufficient evidence for a
  // shortcut: it also occurs when the voxel has never been observed.  Keep
  // this explicit free-space predicate for consumers that must distinguish
  // known free space from unknown space.
  bool isKnownFree(const Eigen::Vector3f &point) const {
    if (!point.allFinite()) return false;
    std::lock_guard<std::mutex> lock(mutex_);
    const VoxelKey key{
      static_cast<int>(std::floor(point.x() / voxel_size_)),
      static_cast<int>(std::floor(point.y() / voxel_size_)),
      static_cast<int>(std::floor(point.z() / voxel_size_))};
    return occupied_voxels_.find(key) == occupied_voxels_.end() &&
      observed_free_voxels_.find(key) != observed_free_voxels_.end();
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

  void rebuildKdTreeLocked() {
    if (cloud_->empty()) {
      kd_.setInputCloud(pcl::PointCloud<PointType>::ConstPtr());
    } else {
      kd_.setInputCloud(cloud_);
    }
  }

  void pruneFreeLocked(const Eigen::Vector3f &pose) {
    const float radius_sq = history_radius_ * history_radius_;
    std::vector<std::pair<float, VoxelKey>> retained;
    retained.reserve(observed_free_voxels_.size());
    for (const auto &key : observed_free_voxels_) {
      const Eigen::Vector3f center(
        (static_cast<float>(key.x) + 0.5F) * voxel_size_,
        (static_cast<float>(key.y) + 0.5F) * voxel_size_,
        (static_cast<float>(key.z) + 0.5F) * voxel_size_);
      const float distance_sq = (center - pose).squaredNorm();
      if (distance_sq <= radius_sq) retained.emplace_back(distance_sq, key);
    }
    if (retained.size() > max_free_voxels_) {
      std::nth_element(
        retained.begin(), retained.begin() + static_cast<std::ptrdiff_t>(max_free_voxels_),
        retained.end(),
        [](const auto &left, const auto &right) { return left.first < right.first; });
      retained.resize(max_free_voxels_);
    }
    observed_free_voxels_.clear();
    observed_free_voxels_.reserve(retained.size());
    for (const auto &entry : retained) observed_free_voxels_.insert(entry.second);
    free_last_prune_pose_ = pose;
    have_free_prune_pose_ = true;
  }

  float voxel_size_ = 0.25F;
  float history_radius_ = 20.0F;
  float prune_distance_ = 0.5F;
  std::size_t max_points_ = 20000;
  std::size_t max_free_voxels_ = 160000;
  Eigen::Vector3f last_prune_pose_ = Eigen::Vector3f::Zero();
  bool have_prune_pose_ = false;
  Eigen::Vector3f free_last_prune_pose_ = Eigen::Vector3f::Zero();
  bool have_free_prune_pose_ = false;
  std::size_t last_hit_voxels_ = 0;
  std::size_t last_free_voxels_ = 0;
  std::size_t last_carved_voxels_ = 0;
  mutable std::mutex mutex_;
  pcl::PointCloud<PointType>::Ptr cloud_;
  std::unordered_set<VoxelKey, VoxelKeyHash> occupied_voxels_;
  std::unordered_set<VoxelKey, VoxelKeyHash> observed_free_voxels_;
  std::unordered_set<VoxelKey, VoxelKeyHash> latest_hit_voxels_;
  mutable pcl::KdTreeFLANN<PointType> kd_;
};

}  // namespace fast_planner
