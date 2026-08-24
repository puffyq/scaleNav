/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-02 16:33:15
 * @LastEditTime: 2024-03-14 11:41:02
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */

#pragma once
#include <Eigen/Eigen>
#include <geometry_msgs/Point.h>
#include <omp.h>
#include <pcl/common/distances.h>

#include <pcl/filters/voxel_grid.h>
// #include <pcl/kdtree/kdtree_flann.h>
#include <pcl/io/pcd_io.h>

#include <pcl/octree/octree.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pointcloud_topo/parallel_bubble_astar.h>
#include <random>
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <limits>
#include <mutex>
#include <vector>
#include <ros/ros.h>
#include <thread>
#include <lidar_map/lidar_map.h>
#include <unordered_map>
#include <unordered_set>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
using namespace std;

struct Vector3iHash {
  std::size_t operator()(const Eigen::Vector3i &v) const {
    std::size_t seed = 0;
    for (int i = 0; i < 3; ++i) {
      seed ^= std::hash<int>{}(v[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

struct Vector2iHash {
  std::size_t operator()(const Eigen::Vector2i &v) const {
    std::size_t seed = 0;
    for (int i = 0; i < 2; ++i) {
      seed ^= std::hash<int>{}(v[i]) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    }
    return seed;
  }
};

struct PairHash {
  std::size_t operator()(const std::pair<Eigen::Vector3i, Eigen::Vector3i> &p) const {
    std::size_t seed = 0;
    seed ^= Vector3iHash{}(p.first) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    seed ^= Vector3iHash{}(p.second) + 0x9e3779b9 + (seed << 6) + (seed >> 2);
    return seed;
  }
};

struct PairHashSet {
  std::shared_timed_mutex hs_mtx;

  void insert(const Eigen::Vector3i &v1, const Eigen::Vector3i &v2) {
    std::unique_lock<std::shared_timed_mutex> lk(hs_mtx);
    mySet.insert(std::make_pair(v1, v2));
    mySet.insert(std::make_pair(v2, v1));
    lk.unlock();
  }

  void remove(const Eigen::Vector3i &v1, const Eigen::Vector3i &v2) {
    std::unique_lock<std::shared_timed_mutex> lk(hs_mtx);
    mySet.erase(std::make_pair(v1, v2));
    mySet.erase(std::make_pair(v2, v1));
    lk.unlock();
  }

  bool check(const Eigen::Vector3i &v1, const Eigen::Vector3i &v2) {
    std::shared_lock<std::shared_timed_mutex> lk(hs_mtx);
    bool fail = mySet.find(std::make_pair(v1, v2)) == mySet.end();
    lk.unlock();
    if (fail) {
      return false;
    } else {
      return true;
    }
  }

  void reset() { mySet.clear(); }

  // private:
  std::unordered_set<std::pair<Eigen::Vector3i, Eigen::Vector3i>, PairHash> mySet;
};

class BubbleNode {
public:
  typedef std::shared_ptr<BubbleNode> Ptr;
  BubbleNode(double radius, Eigen::Vector3f center);
  double radius_;
  int idx_;
  Eigen::Vector3f center_;
};

enum class TopoNodeRole : std::uint8_t {
  Geometric = 0,
  Speculative = 1,
  Odom = 2,
};

enum class TopoGeometryState : std::uint8_t {
  Verified = 0,
  Unknown = 1,
};

// A speculative endpoint is a risk anchor only when the semantic evidence is
// strong enough to overcome the heatmap/EMA noise floor.  Keep this predicate
// shared by planning diagnostics and RViz so a tiny residual score is never
// presented as a block risk.
inline bool isSemanticRiskAnchor(
    float score, float confidence, float minimum_score = 0.35F,
    float minimum_confidence = 0.5F) {
  return std::isfinite(score) && std::isfinite(confidence) &&
         score >= minimum_score && confidence >= minimum_confidence;
}

// PEARL's per-pixel output is a similarity/probability map, not a calibrated
// obstacle probability: an empty frame commonly has a non-zero background.
// Subtract a frame background estimate and retain only positive contrast.
inline float calibrateSemanticScore(float score, float frame_baseline) {
  if (!std::isfinite(score) || !std::isfinite(frame_baseline)) return 0.0F;
  const float denominator = std::max(1.0e-3F, 1.0F - frame_baseline);
  return std::clamp((score - frame_baseline) / denominator, 0.0F, 1.0F);
}

// Patch scores are max-pooled, so their median is biased toward the most
// salient pixels in every patch and is not a stable background estimate.
// A lower quantile keeps ordinary patches as the reference while preserving
// positive contrast for the few high-risk patches.
inline float semanticFrameBaseline(std::vector<float> scores,
                                   float quantile = 0.25F) {
  if (scores.empty()) return 0.0F;
  scores.erase(std::remove_if(scores.begin(), scores.end(),
    [](float value) { return !std::isfinite(value); }), scores.end());
  if (scores.empty()) return 0.0F;
  const float q = std::clamp(quantile, 0.0F, 1.0F);
  const std::size_t index = static_cast<std::size_t>(
    std::floor(q * static_cast<float>(scores.size() - 1)));
  auto middle = scores.begin() + static_cast<std::ptrdiff_t>(index);
  std::nth_element(scores.begin(), middle, scores.end());
  return std::clamp(*middle, 0.0F, 1.0F);
}

inline bool retainGeometryAfterMiss(
    std::uint8_t miss_count, std::uint8_t grace = 2U) {
  return miss_count <= grace;
}

class TopoNode {
public:
  typedef std::shared_ptr<TopoNode> Ptr;
  std::uint64_t persistent_id_ = 0;
  bool is_viewpoint_ = false;
  bool is_history_odom_node_ = false;
  TopoNodeRole role_ = TopoNodeRole::Geometric;
  TopoGeometryState geometry_state_ = TopoGeometryState::Verified;
  // A Bubble can be absent for one map snapshot while the ray-carved map and
  // the topology update are catching up.  Keep a short miss count so one
  // failed regeneration cannot make a persistent node disappear.
  std::uint8_t geometry_miss_count_ = 0;
  float yaw_;
  Eigen::Vector3f center_;
  // Radius of the representative real BubbleNode used to create this node.
  // It is retained after the transient BubbleNode list is released so
  // semantic association can distinguish tight corridors from open space.
  float bubble_radius_ = 0.0F;
  // Planning reads semantics directly from the persistent topology node.
  float semantic_score_ = 0.0F;
  float semantic_confidence_ = 0.0F;
  std::uint32_t semantic_observations_ = 0;
  std::int64_t semantic_stamp_ns_ = 0;
  vector<BubbleNode::Ptr> bubbles_; // 过程量，计算出topoNode后会清空
  unordered_set<TopoNode::Ptr> neighbors_;
  unordered_map<TopoNode::Ptr, uint8_t> unreachable_nbrs_;
  unordered_map<TopoNode::Ptr, vector<Eigen::Vector3f>> paths_;
  unordered_map<TopoNode::Ptr, float> weight_;
};

struct PairPtrHash {
  std::size_t operator()(const std::pair<TopoNode::Ptr, TopoNode::Ptr> &p) const {
    return std::hash<TopoNode::Ptr>()(p.first) ^ std::hash<TopoNode::Ptr>()(p.second);
  }
};

struct PtrPair {
  PtrPair() { flatten_data.reserve(2000); };

  void insert(TopoNode::Ptr a, TopoNode::Ptr b) {
    if (map.count(a) && map[a].count(b))
      return;
    if (map.count(b) && map[b].count(a))
      return;
    map[a].insert(b);
  }

  unordered_map<TopoNode::Ptr, unordered_set<TopoNode::Ptr>> map;

  struct iter_elem {
    TopoNode::Ptr p1;
    TopoNode::Ptr p2;
    bool insert;
    // True when this pair was already a graph neighbour before the update.
    // Existing edges must not be dropped on TIME_OUT / NO_PATH; only an
    // occupied witness (or a colliding replacement) may remove them.
    bool existing = false;
    // Soft retry: keep the previous witness and cool down before rechecking.
    bool soft_retry = false;
    uint8_t doubt_streak = 0;
    vector<Eigen::Vector3f> path;
  };

  void flatten() {
    flatten_data.clear();
    for (auto it = map.begin(); it != map.end(); it++) {
      for (auto it2 = it->second.begin(); it2 != it->second.end(); it2++) {
        flatten_data.push_back(iter_elem{it->first, *it2, true, false, 0, {}});
      }
    }
  }

  vector<iter_elem> flatten_data;
};

class RegionNode {
public:
  typedef std::shared_ptr<RegionNode> Ptr;
  RegionNode(Eigen::Vector3i region_idx);
  Eigen::Vector3i region_idx_;
  int his_odom_id_;
  unordered_set<TopoNode::Ptr> topo_nodes_;
};

class BubbleUnionSet {
public:
  BubbleUnionSet(double min_topobubble_radius) : min_topobubble_radius_(min_topobubble_radius) {};
  typedef std::shared_ptr<BubbleUnionSet> Ptr;
  void updateRegionNode(RegionNode::Ptr region_ptr, const Eigen::Vector3f &region_center_);
  void unionSetCluster(const vector<BubbleNode::Ptr> &bubbles, vector<TopoNode::Ptr> &topos, Eigen::Vector3f &center);

private:
  std::unordered_map<BubbleNode::Ptr, BubbleNode::Ptr> parent;
  std::unordered_map<BubbleNode::Ptr, int> rank;
  std::vector<BubbleNode::Ptr> clusters;
  std::vector<BubbleNode::Ptr> bubbles;
  double min_topobubble_radius_;
  std::unordered_map<BubbleNode::Ptr, TopoNode::Ptr> topo_map;
  void init(const std::vector<BubbleNode::Ptr> &bubbles_);
  BubbleNode::Ptr find(BubbleNode::Ptr b);
  void merge(BubbleNode::Ptr b1, BubbleNode::Ptr b2);
  void getClusters();
  void getTopoNodes(unordered_set<TopoNode::Ptr> &topo_nodes_, const Eigen::Vector3f &center);
};

struct TopoGraphUpdateTiming {
  double total_ms = 0.0;
  double prepare_ms = 0.0;
  double parallel_wall_ms = 0.0;
  double bubble_cpu_ms = 0.0;
  double cluster_cpu_ms = 0.0;
  double diff_ms = 0.0;
  double remove_ms = 0.0;
  double reconnect_ms = 0.0;
  double insert_ms = 0.0;
  size_t regions = 0;
  size_t occupied_regions = 0;
  size_t free_regions = 0;
  size_t bubbles = 0;
  size_t planar_bubbles = 0;
  size_t new_nodes = 0;
  size_t remained_nodes = 0;
  size_t removed_nodes = 0;
  size_t deferred_nodes = 0;
  size_t inserted_nodes = 0;
  // Connection diagnostics are intentionally separate from inserted_nodes:
  // the latter counts vertices accepted by the topology diff, not reachable
  // vertices.  A vertex can therefore be inserted while all candidate edges
  // time out or fail collision validation.
  size_t insert_candidate_edges = 0;
  size_t insert_success_edges = 0;
  size_t insert_timeout_edges = 0;
  size_t insert_no_path_edges = 0;
  size_t insert_start_fail_edges = 0;
  size_t insert_end_fail_edges = 0;
  size_t insert_collision_reject_edges = 0;
  // Diagnostics for re-validation of edges that already exist in the graph.
  // Without these, edge_no_path only reflected new-neighbour insertion and
  // hid progressive deletion of previously verified corridors.
  size_t existing_edges_checked = 0;
  size_t existing_edges_kept = 0;
  size_t existing_edges_repaired = 0;
  size_t existing_edges_removed = 0;
  size_t existing_edges_soft_retry = 0;
  size_t existing_edges_cooldown_skipped = 0;
  size_t duplicate_nodes_merged = 0;
  size_t half_edges_removed = 0;
  size_t semantic_restored_nodes = 0;
  size_t semantic_memory_records = 0;
};

struct TopoSemanticRecord {
  std::uint64_t node_id = 0;
  Eigen::Vector3f center = Eigen::Vector3f::Zero();
  Eigen::Vector3i region_idx = Eigen::Vector3i::Zero();
  float score = 0.0F;
  float confidence = 0.0F;
  std::uint32_t observations = 0;
  std::int64_t stamp_ns = 0;
};

// The online graph is optionally represented on one horizontal layer.  All
// observations (occupied returns and free-ray endpoints) must use the same
// projection before region selection; filtering by the original camera
// endpoint height drops valid side-corridor evidence at long range.
inline Eigen::Vector3f projectGraphPoint(const Eigen::Vector3f &point,
                                         bool planar, float planar_z) {
  Eigen::Vector3f projected = point;
  if (planar) projected.z() = planar_z;
  return projected;
}

class TopoGraph {
public:
  TopoGraph(float res = 0.1) : check_pts_octree_(res) {}

  ros::NodeHandle nh;

  typedef std::unordered_map<Eigen::Vector3i, TopoNode::Ptr, Vector3iHash> HashMap;
  TopoNode::Ptr odom_node_;
  ParallelBubbleAstar::Ptr parallel_bubble_astar_;
  std::ofstream log;
  void removeNodes(vector<TopoNode::Ptr> &nodes);
  void updateRemainedConnections(vector<TopoNode::Ptr> &nodes);
  void insertNodes(vector<TopoNode::Ptr> &nodes, bool only_raycast = false);
  void insertNode(TopoNode::Ptr &new_node, vector<TopoNode::Ptr> &nbr_nodes, vector<vector<Eigen::Vector3f>> &paths);
  // void getUnreachableLocalNodes(vector<TopoNode::Ptr> &nodes_unreachable);
  void updateSkeleton();
  // Copy persistent topology, including speculative semantic candidates, into
  // a freshly initialized graph before applying the current local Bubble diff.
  // The transient odometry query node is recreated by updateOdomNode().
  void copyPersistentNodesFrom(const TopoGraph &source);
  void updateHistoricalOdoms();
  void updateOdomNode(Eigen::Vector3f &odom_pos, float &yaw);
  size_t insertSpeculativeNodes(
      const vector<Eigen::Vector3f> &centers, const vector<float> &semantic_scores,
      float bubble_radius, const Eigen::Vector3f &odom_pos,
      std::int64_t stamp_ns);
  Eigen::Vector3f min_bd, max_bd, map_bd_min, map_bd_max;
  double min_x_, min_y_, min_z_; // 最小格子尺寸
  double bubble_min_radius_, frt_bubble_radius_;
  double init_region_size_x_, init_region_size_y_, init_region_size_z_; // 初始分区大小
  int x_len, y_len, z_len;                                              // 分区数量
  double max_radius, cube_discrete_size;
  bool view_graph_;
  bool planar_graph_ = false;
  float planar_z_ = 0.0F;
  int getBoxId(const Eigen::Vector3f &pt);
  vector<Eigen::Vector3i> update_idx_vec_; //

  vector<RegionNode::Ptr> toponodes_update_region_arr_;
  vector<RegionNode::Ptr> viewpoints_update_region_arr_;

  vector<RegionNode::Ptr> regions_arr_;
  unordered_map<Eigen::Vector3i, RegionNode::Ptr, Vector3iHash> reg_map_idx2ptr_;
  vector<Eigen::Vector3f> global_path_;
  vector<Eigen::Vector3f> global_view_points_;
  LIOInterface::Ptr lidar_map_interface_;
  typedef std::shared_ptr<TopoGraph> Ptr;
  void getIndex(const Eigen::Vector3f &point, Eigen::Vector3i &region_idx_);
  bool index2boundary(const Eigen::Vector3i &region_idx_, Eigen::Vector3f &low_bd, Eigen::Vector3f &high_bd);
  RegionNode::Ptr getRegionNode(const Eigen::Vector3i &region_idx_);
  bool graphSearch(const TopoNode::Ptr &start_node, const TopoNode::Ptr &end_node, std::vector<TopoNode::Ptr> &path, double time_out,
                   bool kino = false, std::unordered_set<pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash> last_path = {},
                   float semantic_cost_weight = 0.0F,
                   float max_search_radius_m = std::numeric_limits<float>::infinity());
  bool goalDirectedSearch(
      const TopoNode::Ptr &start_node, const Eigen::Vector3f &goal,
      std::vector<TopoNode::Ptr> &path, double time_out,
      float path_cost_weight = 0.2f, float previous_path_cost_factor = 0.05f,
      const std::unordered_set<pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash> &last_path = {},
      float semantic_cost_weight = 0.0F,
      float max_search_radius_m = std::numeric_limits<float>::infinity());
  float semanticRiskForEdge(const TopoNode::Ptr &from, const TopoNode::Ptr &to) const;
  float clearanceCostForEdge(const TopoNode::Ptr &from, const TopoNode::Ptr &to) const;
  void init(ros::NodeHandle &nh, LIOInterface::Ptr &lidar_map, ParallelBubbleAstar::Ptr &parallel_bubble_astar);
  void cauculateMemoryConsumption();
  double getPathLength(const vector<TopoNode::Ptr> &topo_path);

  void inline posToIndex(const Eigen::Vector3f &pt, Eigen::Vector3i &idx) {
    idx = ((pt - lidar_map_interface_->lp_->global_box_min_boundary_) * 1000).array().floor().cast<int>();
  }

  void inline indexToPos(const Eigen::Vector3i &idx, Eigen::Vector3f &pt) {
    pt = (idx.cast<float>() + Eigen::Vector3f(0.5, 0.5, 0.5)) / 1000.0f + lidar_map_interface_->lp_->global_box_min_boundary_;
  }

  void overlap(vector<TopoNode::Ptr> &set1, vector<TopoNode::Ptr> &set2, vector<TopoNode::Ptr> &overlap);
  void setdiff(vector<TopoNode::Ptr> &set1, vector<TopoNode::Ptr> &set2, vector<TopoNode::Ptr> &set_1diff2);
  void getPreNbrs(TopoNode::Ptr &node, vector<TopoNode::Ptr> &nbrs);
  void setUpdateGoal(const Eigen::Vector3f &goal) {
    update_goal_ = goal;
    has_update_goal_ = true;
  }
  void getRegionsToUpdate();
  vector<BubbleNode::Ptr> getBubbleSnapshot() const;
  const TopoGraphUpdateTiming &getLastUpdateTiming() const { return last_update_timing_; }
  void updateNodeSemantic(const TopoNode::Ptr &node, float observation,
                          float ema_alpha, std::int64_t stamp_ns);
  vector<TopoSemanticRecord> semanticMemorySnapshot() const;
  void loadSemanticMemory(const vector<TopoSemanticRecord> &records);
  size_t semanticMemorySize() const;
  size_t restoreNodeSemanticMemory(
      vector<TopoNode::Ptr> &nodes,
      const unordered_set<std::uint64_t> &unavailable_ids = {});
  // Merge geometrically duplicate persistent vertices without discarding
  // their incident edges or semantic memory.  The return value is the number
  // of vertices removed from the graph.
  // Bubble centers can move by a few centimetres between map snapshots. A
  // quarter-metre tolerance is the map voxel size, so it also merges genuine
  // adjacent topology vertices and removes valid branches.
  size_t deduplicateNearbyNodes(float tolerance_m = 0.05F);
  // Remove stale one-way neighbor references left by an incremental diff.
  // EPIC edges are undirected; a half-edge is never a valid planning edge.
  size_t normalizeConnectivity();
  void removeNode(TopoNode::Ptr &node);
  std::vector<TopoNode::Ptr> speculativeNodes() const;
  float estimateRoughDistance(const Eigen::Vector3f &goal, const int his_idx);
  vector<TopoNode::Ptr> history_odom_nodes_;
  vector<float> his_odom_dis_vec_;

private:
  TopoGraphUpdateTiming last_update_timing_;
  mutable std::mutex region_map_mutex_;
  mutable std::mutex bubble_snapshot_mutex_;
  unordered_map<Eigen::Vector3i, vector<BubbleNode::Ptr>, Vector3iHash> bubble_snapshots_by_region_;
  PointVector check_pts_;
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> check_pts_octree_;
  int max_update_region_num_;
  Eigen::Vector3f update_goal_ = Eigen::Vector3f::Zero();
  bool has_update_goal_ = false;
  bool use_prior_map_;
  double update_connection_timeout, insert_node_timeout;
  double semantic_node_match_distance_ = 2.5;
  // Speculative semantic observations influence nearby ordinary edges too;
  // otherwise A* can pass beside a risk node without ever visiting it.
  double semantic_speculative_influence_m_ = 5.0;
  double clearance_cost_weight_ = 2.0;
  double clearance_target_m_ = 1.2;
  mutable std::mutex semantic_memory_mutex_;
  unordered_map<std::uint64_t, TopoSemanticRecord> semantic_memory_;
  std::uint64_t next_semantic_node_id_ = 1;
  bool hasOverlapWithBox(const Eigen::Vector3f &low_bd, const Eigen::Vector3f &high_bd);
  float edgeSemanticRisk(const TopoNode::Ptr &from, const TopoNode::Ptr &to) const;

  size_t selected_occupied_regions_ = 0;
  size_t selected_free_regions_ = 0;

  void generateBubble(const Eigen::Vector3f &low_bd, const Eigen::Vector3f &high_bd, vector<BubbleNode::Ptr> &bubble_node_vec,
                      vector<bool> &check_flags);
  void splitCubeBubbleGeneration(const Eigen::Vector3f &low_bd, const Eigen::Vector3f &high_bd, vector<BubbleNode::Ptr> &bubble_node_vec,
                                 vector<bool> &check_flags);
  void supplementCubeBubbleGeneration(const Eigen::Vector3f &low_bd, const Eigen::Vector3f &high_bd, vector<BubbleNode::Ptr> &bubble_node_vec,
                                      vector<bool> &check_flags, const BubbleNode::Ptr &bubble_node);
  bool isCubeCoveredByBubble(const Eigen::Vector3f &low_bd, const Eigen::Vector3f &high_bd, const vector<BubbleNode::Ptr> &bubble_node_vec);

  int searchPathWithBoundary(const Eigen::Vector3f &start, const Eigen::Vector3f &end, double &time_out, vector<Eigen::Vector3f> &path);
  float edgeClearancePenalty(const TopoNode::Ptr &from,
                             const TopoNode::Ptr &to,
                             float edge_length) const;
};
