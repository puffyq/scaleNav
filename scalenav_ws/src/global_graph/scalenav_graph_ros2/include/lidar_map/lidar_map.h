#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <numeric>
#include <unordered_map>
#include <vector>

#include <Eigen/Eigen>
#include <Eigen/StdVector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "lidar_map/ikd_Tree.h"

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
                   cloud_(std::make_shared<pcl::PointCloud<PointType>>()) {
    // EPIC rebuilds immutable snapshots with Build(). The legacy KD-tree
    // worker is for concurrent Add_Points() rebuilds and races with Build()
    // replacing Root_Node, so keep it disabled for these snapshot trees.
    ikd_Tree_map = std::make_unique<KD_TREE<PointType>>(0.5F, 0.6F, voxel_size_, false);
    ikd_Tree_map->setMap_ikdtree(this);
    ikd_Tree_map->Set_delete_criterion_param(0.3F);
    ikd_Tree_map->Set_balance_criterion_param(0.6F);
    ikd_Tree_map->set_downsample_param(voxel_size_);
    ikd_Tree_layer = std::make_unique<KD_TREE<PointType>>(0.5F, 0.6F, voxel_size_, false);
    ikd_Tree_layer->setMap_ikdtree(this);
    ikd_Tree_layer->Set_delete_criterion_param(0.3F);
    ikd_Tree_layer->Set_balance_criterion_param(0.6F);
    ikd_Tree_layer->set_downsample_param(voxel_size_);
  }

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
    prune_distance_ = std::max(prune_distance, 0.1F);
    ikd_Tree_map->set_downsample_param(voxel_size_);
    ikd_Tree_layer->set_downsample_param(voxel_size_);
  }

  // Planar topology: keep ground in cloud_, but ignore it when measuring
  // clearance. Points below min_z do not shrink Bubble radius or block edges.
  void setGraphObstacleMinZ(float min_z) {
    std::lock_guard<std::mutex> lock(mutex_);
    graph_obstacle_min_z_ = min_z;
    rebuildLayerTreeLocked();
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
    std::size_t hit_voxels = 0;
    for (const auto &point : points_world.points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) {
        continue;
      }
      const VoxelKey key = voxelKey(point);
      ++hit_voxels;
      // Match EPIC ikd-tree Add_Points downsample: one representative per
      // voxel, the sample closest to the voxel center.
      const auto found = occupied_index_.find(key);
      if (found == occupied_index_.end()) {
        occupied_index_.emplace(key, cloud_->size());
        cloud_->push_back(point);
        changed = true;
        continue;
      }
      auto &existing = cloud_->points[found->second];
      if (distanceToVoxelCenterSq(point, key) + 1e-8F <
          distanceToVoxelCenterSq(existing, key)) {
        existing = point;
        changed = true;
      }
    }
    last_hit_voxels_ = hit_voxels;
    const bool moved_for_prune = !have_prune_pose_ ||
      (pose - last_prune_pose_).norm() >= prune_distance_;
    const bool needs_prune = moved_for_prune || cloud_->size() > max_points_;
    if (!changed && !needs_prune) return false;
    if (needs_prune) pruneLocked(pose);
    rebuildKdTreeLocked();
    return changed || needs_prune;
  }

  bool updateFreeRaysWorld(const pcl::PointCloud<PointType> &,
                           const Eigen::Vector3f &,
                           const Eigen::Quaternionf &) {
    return false;
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
    return pcl::PointCloud<PointType>{};
  }

  void loadSnapshot(const pcl::PointCloud<PointType> &accumulated_world,
                    const pcl::PointCloud<PointType> &latest_world,
                    const pcl::PointCloud<PointType> &,
                    const Eigen::Vector3f &pose,
                    const Eigen::Quaternionf &orientation) {
    loadSnapshot(accumulated_world, latest_world, pose, orientation);
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
    rebuildOccupiedIndexLocked();
    last_prune_pose_ = pose;
    have_prune_pose_ = true;
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
    const bool use_layer = std::isfinite(graph_obstacle_min_z_);
    if (use_layer && !have_layer_points_) return lp_->max_ray_length_;
    if (!use_layer && cloud_->empty()) return lp_->max_ray_length_;
    KD_TREE<PointType> *tree = use_layer ? ikd_Tree_layer.get() : ikd_Tree_map.get();
    if (!tree) return lp_->max_ray_length_;
    PointVector nearest_points;
    std::vector<float> distances;
    tree->Nearest_Search(
      point, 1, nearest_points, distances, std::numeric_limits<double>::infinity());
    if (nearest_points.empty() || distances.empty()) {
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
  bool isKnownFree(const Eigen::Vector3f &) const {
    return false;
  }

  void KNN(const PointType &point, int k, PointVector &points,
           std::vector<float> &distances) const {
    std::lock_guard<std::mutex> lock(mutex_);
    points.clear();
    distances.clear();
    if (cloud_->empty()) return;
    ikd_Tree_map->Nearest_Search(
      point, std::max(k, 1), points, distances,
      std::numeric_limits<double>::infinity());
  }

  void boxSearch(const Eigen::Vector3f &min_bound,
                 const Eigen::Vector3f &max_bound,
                 PointVector &points) const {
    std::lock_guard<std::mutex> lock(mutex_);
    BoxPointType boxpoint;
    for (int axis = 0; axis < 3; ++axis) {
      boxpoint.vertex_min[axis] = min_bound(axis);
      // ikd-tree uses an exclusive upper bound.  Move it by one representable
      // float so this public API retains its historical inclusive semantics.
      boxpoint.vertex_max[axis] = std::nextafter(max_bound(axis),
                                                 std::numeric_limits<float>::infinity());
    }
    ikd_Tree_map->Box_Search(boxpoint, points);
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

  VoxelKey voxelKey(const PointType &point) const {
    return VoxelKey{
      static_cast<int>(std::floor(point.x / voxel_size_)),
      static_cast<int>(std::floor(point.y / voxel_size_)),
      static_cast<int>(std::floor(point.z / voxel_size_))};
  }

  Eigen::Vector3f voxelCenter(const VoxelKey &key) const {
    return Eigen::Vector3f(
      (static_cast<float>(key.x) + 0.5F) * voxel_size_,
      (static_cast<float>(key.y) + 0.5F) * voxel_size_,
      (static_cast<float>(key.z) + 0.5F) * voxel_size_);
  }

  float distanceToVoxelCenterSq(const PointType &point, const VoxelKey &key) const {
    const Eigen::Vector3f center = voxelCenter(key);
    const Eigen::Vector3f delta(point.x - center.x(), point.y - center.y(),
                                point.z - center.z());
    return delta.squaredNorm();
  }

  void rebuildOccupiedIndexLocked() {
    occupied_index_.clear();
    occupied_index_.reserve(cloud_->size());
    pcl::PointCloud<PointType>::Ptr compacted(new pcl::PointCloud<PointType>);
    compacted->reserve(cloud_->size());
    for (const auto &point : cloud_->points) {
      if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
          !std::isfinite(point.z)) continue;
      const VoxelKey key = voxelKey(point);
      const auto found = occupied_index_.find(key);
      if (found == occupied_index_.end()) {
        occupied_index_.emplace(key, compacted->size());
        compacted->push_back(point);
        continue;
      }
      auto &existing = compacted->points[found->second];
      if (distanceToVoxelCenterSq(point, key) + 1e-8F <
          distanceToVoxelCenterSq(existing, key)) {
        existing = point;
      }
    }
    cloud_ = std::move(compacted);
  }

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
    occupied_index_.clear();
    occupied_index_.reserve(retained.size());
    for (const auto &entry : retained) {
      occupied_index_.emplace(voxelKey(entry.second), cloud_->size());
      cloud_->push_back(entry.second);
    }
    last_prune_pose_ = pose;
    have_prune_pose_ = true;
  }

  void rebuildKdTreeLocked() {
    PointVector points;
    points.reserve(cloud_->size());
    for (const auto &point : cloud_->points) points.push_back(point);
    ikd_Tree_map->Build(points);
    rebuildLayerTreeLocked();
  }

  void rebuildLayerTreeLocked() {
    have_layer_points_ = false;
    if (!std::isfinite(graph_obstacle_min_z_) || !ikd_Tree_layer) return;
    PointVector points;
    points.reserve(cloud_->size());
    for (const auto &point : cloud_->points) {
      if (point.z >= graph_obstacle_min_z_) points.push_back(point);
    }
    ikd_Tree_layer->Build(points);
    have_layer_points_ = !points.empty();
  }

  float voxel_size_ = 0.1F;
  float history_radius_ = 20.0F;
  float prune_distance_ = 0.5F;
  float graph_obstacle_min_z_ = std::numeric_limits<float>::quiet_NaN();
  bool have_layer_points_ = false;
  std::size_t max_points_ = 20000;
  Eigen::Vector3f last_prune_pose_ = Eigen::Vector3f::Zero();
  bool have_prune_pose_ = false;
  std::size_t last_hit_voxels_ = 0;
  std::size_t last_free_voxels_ = 0;
  std::size_t last_carved_voxels_ = 0;
  mutable std::mutex mutex_;
  pcl::PointCloud<PointType>::Ptr cloud_;
  std::unordered_map<VoxelKey, std::size_t, VoxelKeyHash> occupied_index_;
  std::unique_ptr<KD_TREE<PointType>> ikd_Tree_map;
  std::unique_ptr<KD_TREE<PointType>> ikd_Tree_layer;
};

}  // namespace fast_planner
