/***
 * @Author: ning-zelin && zl.ning@qq.com
 * @Date: 2023-12-02 21:33:08
 * @LastEditTime: 2024-03-05 12:12:19
 * @Description:
 * @
 * @Copyright (c) 2024 by ning-zelin, All Rights Reserved.
 */

#include "pointcloud_topo/graph.h"

#include <chrono>

namespace {

Eigen::Vector3i semanticWorldRegion(const Eigen::Vector3f &center,
                                    double size_x, double size_y, double size_z) {
  return Eigen::Vector3i(
    static_cast<int>(std::floor(center.x() / std::max(size_x, 1e-3))),
    static_cast<int>(std::floor(center.y() / std::max(size_y, 1e-3))),
    static_cast<int>(std::floor(center.z() / std::max(size_z, 1e-3))));
}

struct NodeMatchCandidate {
  size_t old_index;
  size_t new_index;
  float dist_sq;
};

}  // namespace

void debug_exit(const std::string &location) {
  std::cout << "\033[1;31m Terminating process at location: " << location << "\033[0m" << std::endl;
  exit(0);
}

void TopoGraph::init(ros::NodeHandle &nh, LIOInterface::Ptr &lidar_map, ParallelBubbleAstar::Ptr &parallel_bubble_astar) {
  lidar_map_interface_ = lidar_map;

  min_bd = lidar_map_interface_->lp_->global_box_min_boundary_;
  max_bd = lidar_map_interface_->lp_->global_box_max_boundary_;

  parallel_bubble_astar_ = parallel_bubble_astar;
  odom_node_ = make_shared<TopoNode>();
  odom_node_->is_viewpoint_ = true;
  odom_node_->role_ = TopoNodeRole::Odom;
  odom_node_->geometry_state_ = TopoGeometryState::Verified;
  // 分区，初始化regions_arr_
  // 10m * 10m * 2m ==> 0.315 * 0.315 * 0.5
  nh = nh;
  nh.param("bubble_topo/min_x", min_x_, 0.0);
  nh.param("bubble_topo/min_y", min_y_, 0.0);
  nh.param("bubble_topo/min_z", min_z_, 0.0);
  nh.param("bubble_topo/init_region_size_x", init_region_size_x_, 0.0);
  nh.param("bubble_topo/init_region_size_y", init_region_size_y_, 0.0);
  nh.param("bubble_topo/init_region_size_z", init_region_size_z_, 0.0);
  nh.param("bubble_topo/bubble_min_radius", bubble_min_radius_, 0.5);
  nh.param("bubble_topo/frontier_bubble_min_radius", frt_bubble_radius_, 0.5);
  nh.param("bubble_topo/cube_discrete_size", cube_discrete_size, 0.3);
  nh.param("bubble_topo/planar_graph", planar_graph_, false);
  double planar_z = 0.0;
  nh.param("bubble_topo/planar_z", planar_z, 0.0);
  planar_z_ = static_cast<float>(planar_z);

  nh.getParam("parallel_astar/update_connection_timeout", update_connection_timeout);
  nh.getParam("parallel_astar/insert_node_timeout", insert_node_timeout);

  nh.getParam("max_update_region_num", max_update_region_num_);
  nh.param("bubble_topo/semantic_node_match_distance", semantic_node_match_distance_, 2.5);
  nh.param("bubble_topo/semantic_speculative_influence_m",
           semantic_speculative_influence_m_, 5.0);
  nh.param("bubble_topo/clearance_cost_weight", clearance_cost_weight_, 2.0);
  nh.param("bubble_topo/clearance_target_m", clearance_target_m_, 1.2);
  update_idx_vec_.reserve(100);
  global_path_.reserve(200);
  x_len = std::ceil((max_bd - min_bd).x() / init_region_size_x_);
  y_len = std::ceil((max_bd - min_bd).y() / init_region_size_y_);
  z_len = std::ceil((max_bd - min_bd).z() / init_region_size_z_);
  for (size_t i = 0; i < x_len; i++)
    for (int j = 0; j < y_len; j++)
      for (int k = 0; k < z_len; k++) {
        Eigen::Vector3i idx(i, j, k);
        RegionNode::Ptr region_node = std::make_shared<RegionNode>(idx);
        Eigen::Vector3f hb, lb;
        index2boundary(idx, lb, hb);
        if (!hasOverlapWithBox(lb, hb))
          continue;
        reg_map_idx2ptr_[idx] = region_node;
      }
  // 建立一个octree作为球形覆盖的check-point
  check_pts_.clear();
  for (float x = 0; x < init_region_size_x_; x += cube_discrete_size) {
    for (float y = 0; y < init_region_size_y_; y += cube_discrete_size) {
      for (float z = 0; z < init_region_size_z_; z += cube_discrete_size) {
        check_pts_.emplace_back(x, y, z);
      }
    }
  }
  pcl::PointCloud<pcl::PointXYZ>::Ptr check_pts_pc(new pcl::PointCloud<pcl::PointXYZ>);
  check_pts_pc->points = check_pts_;
  check_pts_octree_.setResolution(cube_discrete_size);
  check_pts_octree_.setInputCloud(check_pts_pc);
  check_pts_octree_.addPointsFromInputCloud();
}

void TopoGraph::copyPersistentNodesFrom(const TopoGraph &source) {
  if (this == &source) return;
  std::unordered_map<TopoNode::Ptr, TopoNode::Ptr> copied;
  std::vector<TopoNode::Ptr> source_nodes;
  {
    std::lock_guard<std::mutex> lock(source.semantic_memory_mutex_);
    for (const auto &entry : source.reg_map_idx2ptr_) {
      if (!entry.second) continue;
      for (const auto &node : entry.second->topo_nodes_) {
        if (!node || node->is_viewpoint_ || node->role_ == TopoNodeRole::Odom ||
            !copied.emplace(node, nullptr).second) continue;
        auto clone = std::make_shared<TopoNode>();
        clone->persistent_id_ = node->persistent_id_;
        clone->role_ = node->role_;
        clone->geometry_state_ = node->geometry_state_;
        clone->center_ = node->center_;
        clone->bubble_radius_ = node->bubble_radius_;
        clone->semantic_score_ = node->semantic_score_;
        clone->semantic_confidence_ = node->semantic_confidence_;
        clone->semantic_observations_ = node->semantic_observations_;
        clone->semantic_stamp_ns_ = node->semantic_stamp_ns_;
        copied[node] = clone;
        source_nodes.emplace_back(node);
      }
    }
  }

  for (const auto &source_node : source_nodes) {
    const auto clone = copied.at(source_node);
    Eigen::Vector3i region_idx;
    getIndex(clone->center_, region_idx);
    const auto region = getRegionNode(region_idx);
    if (!region) continue;
    region->topo_nodes_.insert(clone);
  }

  for (const auto &source_node : source_nodes) {
    const auto from_it = copied.find(source_node);
    if (from_it == copied.end()) continue;
    const auto from = from_it->second;
    for (const auto &source_neighbor : source_node->neighbors_) {
      const auto to_it = copied.find(source_neighbor);
      if (to_it == copied.end() ||
          std::less<const TopoNode *>{}(to_it->second.get(), from.get())) continue;
      const auto to = to_it->second;
      from->neighbors_.insert(to);
      to->neighbors_.insert(from);
      const auto path_it = source_node->paths_.find(source_neighbor);
      if (path_it != source_node->paths_.end()) {
        from->paths_[to] = path_it->second;
        auto reverse_path = path_it->second;
        std::reverse(reverse_path.begin(), reverse_path.end());
        to->paths_[from] = reverse_path;
      }
      const auto weight_it = source_node->weight_.find(source_neighbor);
      if (weight_it != source_node->weight_.end()) {
        from->weight_[to] = weight_it->second;
        to->weight_[from] = weight_it->second;
      }
    }
  }
}

BubbleNode::BubbleNode(double radius, Eigen::Vector3f center) {
  radius_ = radius;
  center_ = center;
}

float TopoGraph::edgeClearancePenalty(const TopoNode::Ptr &from,
                                      const TopoNode::Ptr &to,
                                      float edge_length) const {
  if (!from || !to || clearance_cost_weight_ <= 0.0 ||
      clearance_target_m_ <= 0.0 || !lidar_map_interface_) {
    return 0.0F;
  }
  std::vector<Eigen::Vector3f> samples;
  const auto path_it = from->paths_.find(to);
  if (path_it != from->paths_.end()) samples = path_it->second;
  if (samples.empty()) samples = {from->center_, to->center_};
  float minimum_clearance = std::numeric_limits<float>::infinity();
  for (const auto &sample : samples) {
    const double clearance = lidar_map_interface_->getDisToOcc(sample);
    if (std::isfinite(clearance)) {
      minimum_clearance = std::min(minimum_clearance,
                                   static_cast<float>(clearance));
    }
  }
  if (!std::isfinite(minimum_clearance) ||
      minimum_clearance >= clearance_target_m_) {
    return 0.0F;
  }
  const float deficit = static_cast<float>(
    (clearance_target_m_ - minimum_clearance) / clearance_target_m_);
  return static_cast<float>(clearance_cost_weight_) * edge_length *
    deficit * deficit;
}

std::vector<TopoNode::Ptr> TopoGraph::speculativeNodes() const {
  std::vector<TopoNode::Ptr> nodes;
  for (const auto &entry : reg_map_idx2ptr_) {
    if (!entry.second) continue;
    for (const auto &node : entry.second->topo_nodes_) {
      if (node && node->role_ == TopoNodeRole::Speculative)
        nodes.emplace_back(node);
    }
  }
  return nodes;
}

float TopoGraph::edgeSemanticRisk(const TopoNode::Ptr &from,
                                  const TopoNode::Ptr &to) const {
  if (!from || !to) return 0.0F;
  float risk = std::clamp(0.5F *
    (std::clamp(from->semantic_score_ * from->semantic_confidence_, 0.0F, 1.0F) +
     std::clamp(to->semantic_score_ * to->semantic_confidence_, 0.0F, 1.0F)),
    0.0F, 1.0F);

  // A speculative node is an observation at the end of a semantic ray, not
  // merely a destination with a cost. Project it onto the edge witness so
  // ordinary edges near the predicted risk are penalized before A* reaches
  // the candidate itself. This keeps the risk representation unified in
  // TopoNode while providing the intended early turn-away behavior.
  // Use EPIC's collision-free witness polyline, not the chord between the
  // node centers.  A witness can bend around an obstacle; measuring only the
  // chord can miss a semantic risk that lies directly on the executed edge.
  std::vector<Eigen::Vector3f> witness;
  const auto path_it = from->paths_.find(to);
  if (path_it != from->paths_.end()) {
    witness = path_it->second;
  } else {
    const auto reverse_it = to->paths_.find(from);
    if (reverse_it != to->paths_.end()) {
      witness = reverse_it->second;
      std::reverse(witness.begin(), witness.end());
    }
  }
  if (witness.size() < 2) witness = {from->center_, to->center_};
  const float influence = static_cast<float>(std::max(
    0.5, semantic_speculative_influence_m_));
  const float sigma = std::max(0.5F, 0.5F * influence);
  const float influence_sq = influence * influence;
  for (const auto &entry : reg_map_idx2ptr_) {
    if (!entry.second) continue;
    for (const auto &candidate : entry.second->topo_nodes_) {
      if (!candidate || candidate->role_ != TopoNodeRole::Speculative) continue;
      const float confidence_score = std::clamp(
        candidate->semantic_score_ * candidate->semantic_confidence_, 0.0F, 1.0F);
      if (confidence_score <= 1e-3F) continue;
      float distance_sq = std::numeric_limits<float>::infinity();
      for (std::size_t i = 1; i < witness.size(); ++i) {
        const Eigen::Vector3f a = witness[i - 1];
        const Eigen::Vector3f segment = witness[i] - a;
        const float segment_sq = segment.squaredNorm();
        const float t = segment_sq > 1e-6F ?
          std::clamp((candidate->center_ - a).dot(segment) / segment_sq,
                     0.0F, 1.0F) : 0.0F;
        const Eigen::Vector3f closest = a + t * segment;
        distance_sq = std::min(distance_sq,
          (candidate->center_ - closest).squaredNorm());
      }
      if (distance_sq > influence_sq) continue;
      const float field = confidence_score * std::exp(
        -0.5F * distance_sq / (sigma * sigma));
      risk = std::max(risk, field);
    }
  }
  return std::clamp(risk, 0.0F, 1.0F);
}

float TopoGraph::semanticRiskForEdge(const TopoNode::Ptr &from,
                                     const TopoNode::Ptr &to) const {
  return edgeSemanticRisk(from, to);
}

void TopoGraph::updateNodeSemantic(const TopoNode::Ptr &node, float observation,
                                   float ema_alpha, std::int64_t stamp_ns) {
  if (!node || node->is_viewpoint_ || !std::isfinite(observation))
    return;
  std::lock_guard<std::mutex> lock(semantic_memory_mutex_);
  if (node->persistent_id_ == 0)
    node->persistent_id_ = next_semantic_node_id_++;
  const float alpha = std::clamp(ema_alpha, 0.0F, 1.0F);
  const float score = std::clamp(observation, 0.0F, 1.0F);
  if (node->semantic_observations_ == 0) {
    node->semantic_score_ = score;
    node->semantic_confidence_ = 1.0F;
  } else {
    // Semantic memory follows the current evidence. A stale maximum would
    // make one accidental high-score association permanent and would keep
    // painting/planning through a node long after the target left the view.
    node->semantic_score_ = (1.0F - alpha) * node->semantic_score_ + alpha * score;
    node->semantic_confidence_ = std::min(
      1.0F, node->semantic_confidence_ + alpha * (1.0F - node->semantic_confidence_));
  }
  ++node->semantic_observations_;
  node->semantic_stamp_ns_ = stamp_ns;
  const Eigen::Vector3i region_idx = semanticWorldRegion(
    node->center_, init_region_size_x_, init_region_size_y_, init_region_size_z_);
  semantic_memory_[node->persistent_id_] = TopoSemanticRecord{
    node->persistent_id_, node->center_, region_idx, node->semantic_score_,
    node->semantic_confidence_, node->semantic_observations_, stamp_ns};
}

vector<TopoSemanticRecord> TopoGraph::semanticMemorySnapshot() const {
  std::lock_guard<std::mutex> lock(semantic_memory_mutex_);
  vector<TopoSemanticRecord> records;
  records.reserve(semantic_memory_.size());
  for (const auto &entry : semantic_memory_)
    records.push_back(entry.second);
  return records;
}

void TopoGraph::loadSemanticMemory(const vector<TopoSemanticRecord> &records) {
  std::lock_guard<std::mutex> lock(semantic_memory_mutex_);
  for (const auto &record : records) {
    if (record.node_id == 0 || !record.center.allFinite() ||
        !std::isfinite(record.score) || !std::isfinite(record.confidence))
      continue;
    semantic_memory_[record.node_id] = record;
    next_semantic_node_id_ = std::max(next_semantic_node_id_, record.node_id + 1);
  }
}

size_t TopoGraph::semanticMemorySize() const {
  std::lock_guard<std::mutex> lock(semantic_memory_mutex_);
  return semantic_memory_.size();
}

size_t TopoGraph::restoreNodeSemanticMemory(
    vector<TopoNode::Ptr> &nodes,
    const unordered_set<std::uint64_t> &unavailable_ids) {
  std::lock_guard<std::mutex> lock(semantic_memory_mutex_);
  const float maximum_distance_sq = static_cast<float>(
    semantic_node_match_distance_ * semantic_node_match_distance_);
  unordered_set<std::uint64_t> claimed_records = unavailable_ids;
  size_t restored = 0;
  for (const auto &node : nodes) {
    if (!node || node->is_viewpoint_)
      continue;
    const TopoSemanticRecord *nearest = nullptr;
    float nearest_distance_sq = maximum_distance_sq;
    const Eigen::Vector3i node_region = semanticWorldRegion(
      node->center_, init_region_size_x_, init_region_size_y_, init_region_size_z_);
    for (const auto &entry : semantic_memory_) {
      const auto &record = entry.second;
      if (claimed_records.count(record.node_id))
        continue;
      const Eigen::Vector3i record_region = record.region_idx;
      if ((record_region - node_region).cwiseAbs().maxCoeff() > 1)
        continue;
      const float distance_sq = (record.center - node->center_).squaredNorm();
      const bool same_region = record_region == node_region;
      const bool nearest_is_same_region = nearest &&
        (nearest->region_idx == node_region);
      if (distance_sq <= nearest_distance_sq &&
          (!nearest || same_region || !nearest_is_same_region)) {
        nearest = &record;
        nearest_distance_sq = distance_sq;
      }
    }
    if (nearest) {
      node->persistent_id_ = nearest->node_id;
      node->semantic_score_ = nearest->score;
      node->semantic_confidence_ = nearest->confidence;
      node->semantic_observations_ = nearest->observations;
      node->semantic_stamp_ns_ = nearest->stamp_ns;
      claimed_records.insert(nearest->node_id);
      semantic_memory_[nearest->node_id].center = node->center_;
      semantic_memory_[nearest->node_id].region_idx = node_region;
      ++restored;
    } else {
      node->persistent_id_ = next_semantic_node_id_++;
    }
  }
  return restored;
}

RegionNode::RegionNode(Eigen::Vector3i region_idx) {
  region_idx_ = region_idx;
  his_odom_id_ = -1;
}

RegionNode::Ptr TopoGraph::getRegionNode(const Eigen::Vector3i &region_idx_) {
  auto it = reg_map_idx2ptr_.find(region_idx_);
  if (it != reg_map_idx2ptr_.end()) return it->second;
  // The online map is rolling and the mission is unbounded.  Regions outside
  // the initial goal-sized box are created on demand when new occupied/free
  // voxels are observed instead of making the graph disappear at the box edge.
  auto region = std::make_shared<RegionNode>(region_idx_);
  reg_map_idx2ptr_[region_idx_] = region;
  return region;
}

void TopoGraph::getIndex(const Eigen::Vector3f &point, Eigen::Vector3i &region_idx_) {
  region_idx_.x() = int((point[0] - min_bd[0]) / init_region_size_x_);
  region_idx_.y() = int((point[1] - min_bd[1]) / init_region_size_y_);
  region_idx_.z() = int((point[2] - min_bd[2]) / init_region_size_z_);
}

bool TopoGraph::index2boundary(const Eigen::Vector3i &region_idx_, Eigen::Vector3f &low_bd, Eigen::Vector3f &high_bd) {
  low_bd = Eigen::Vector3f(min_bd[0] + region_idx_.x() * init_region_size_x_, min_bd[1] + region_idx_.y() * init_region_size_y_,
                           min_bd[2] + region_idx_.z() * init_region_size_z_);
  high_bd = low_bd + Eigen::Vector3f(init_region_size_x_, init_region_size_y_, init_region_size_z_);
  return true;
}


void BubbleUnionSet::init(const std::vector<BubbleNode::Ptr> &bubbles_) {
  parent.clear();
  rank.clear();
  clusters.clear();
  bubbles.clear();
  topo_map.clear();
  bubbles = bubbles_;
  rank.reserve(bubbles.size());
  parent.reserve(bubbles.size());
  clusters.reserve(bubbles.size());
  parent.clear();
  rank.clear();
  for (auto &b : bubbles) {
    parent[b] = b;
    rank[b] = 0;
  }
}

BubbleNode::Ptr BubbleUnionSet::find(BubbleNode::Ptr b) {
  if (parent[b] != b) {
    parent[b] = find(parent[b]);
  }
  return parent[b];
}

void BubbleUnionSet::merge(BubbleNode::Ptr b1, BubbleNode::Ptr b2) {
  b1 = find(b1);
  b2 = find(b2);
  if (rank[b1] > rank[b2]) {
    parent[b2] = b1;
  } else {
    parent[b1] = b2;
    if (rank[b1] == rank[b2]) {
      rank[b2]++;
    }
  }
}

void BubbleUnionSet::getClusters() {
  clusters.clear();
  for (auto &b : parent) {
    if (b.second == b.first) {
      clusters.push_back(b.first);
      topo_map[b.first] = TopoNode::Ptr(new TopoNode);
    }
  }
  for (auto &b : bubbles) {
    auto topo_ptr = topo_map[find(b)];
    topo_ptr->bubbles_.push_back(b);
  }
}

bool TopoGraph::graphSearch(const TopoNode::Ptr &start_node, const TopoNode::Ptr &end_node, std::vector<TopoNode::Ptr> &path, double time_out,
                            bool kino, std::unordered_set<pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash> last_path,
                            float semantic_cost_weight) {
  path.clear();
  std::unordered_map<TopoNode::Ptr, float> g_score, f_score;
  std::unordered_map<TopoNode::Ptr, TopoNode::Ptr> parent_map;
  std::unordered_set<TopoNode::Ptr> close_set, open_set_set_;
  float tie_breaker_ = 1.0 + 1.0 / 1000;
  std::priority_queue<std::pair<float, TopoNode::Ptr>, std::vector<std::pair<float, TopoNode::Ptr>>, std::greater<std::pair<float, TopoNode::Ptr>>>
  open_set;
  auto getHeuristic = [&](const TopoNode::Ptr &n) -> float { return tie_breaker_ * (n->center_ - end_node->center_).norm(); };
  semantic_cost_weight = std::max(0.0F, semantic_cost_weight);
  auto backtrack = [&]() {
    TopoNode::Ptr cur_node = end_node;
    path.push_back(cur_node);
    while (parent_map.find(cur_node) != parent_map.end()) {
      cur_node = parent_map[cur_node];
      path.push_back(cur_node);
    }
    std::reverse(path.begin(), path.end());
  };
  auto cur_node = start_node;
  std::unordered_map<TopoNode::Ptr, Eigen::Vector3f> node_vel;
  g_score[cur_node] = 0.0;
  f_score[cur_node] = getHeuristic(cur_node);
  open_set.push({f_score[cur_node], cur_node});
  open_set_set_.insert(cur_node);
  const auto t1 = ros::Time::now();
  while (!open_set.empty()) {
    cur_node = open_set.top().second;
    open_set_set_.erase(cur_node);
    open_set.pop();
    close_set.insert(cur_node);
    if (cur_node == end_node) {
      backtrack();
      return true;
    }
    if ((ros::Time::now() - t1).toSec() > time_out) {
      // ROS_ERROR("topo a* timeout");
      return false;
    }
    for (auto &neighbor : cur_node->neighbors_) {
      // if (!neighbor->reachable_)
      //   continue;
      if (close_set.find(neighbor) != close_set.end())
        continue;

      const float edge_length = (neighbor->center_ - cur_node->center_).norm();
      const float clearance_penalty = edgeClearancePenalty(
        cur_node, neighbor, edge_length);
      const float semantic_risk = edgeSemanticRisk(cur_node, neighbor);
      // Smooth risk barrier: low semantic risk remains inexpensive, while a
      // route through a highly confident block becomes rapidly unattractive
      // without introducing a discontinuous hard threshold.
      const float semantic_barrier = -std::log(std::max(
        1e-3F, 1.0F - semantic_risk));
      const float semantic_penalty =
        semantic_cost_weight * edge_length * semantic_barrier;
      float tentative_g_score;
      if (kino) {
        if (last_path.find({cur_node, neighbor}) != last_path.end()) {
          // tentative_g_score = g_score[cur_node] + 1e-3 * cur_node->weight_[neighbor];
          tentative_g_score = g_score[cur_node] + 0 * cur_node->weight_[neighbor] +
            semantic_penalty + clearance_penalty;
        } else
          tentative_g_score = g_score[cur_node] + cur_node->weight_[neighbor] +
            semantic_penalty + clearance_penalty;
      } else {
        tentative_g_score = g_score[cur_node] + cur_node->weight_[neighbor] +
          semantic_penalty + clearance_penalty;
      }
      if (open_set_set_.find(neighbor) == open_set_set_.end() || tentative_g_score < g_score[neighbor]) {
        parent_map[neighbor] = cur_node;
        g_score[neighbor] = tentative_g_score;
        f_score[neighbor] = g_score[neighbor] + getHeuristic(neighbor);
        open_set.push({f_score[neighbor], neighbor});
        open_set_set_.insert(neighbor);
      } else
        continue;
    }
  }
  return false;
}

bool TopoGraph::goalDirectedSearch(
    const TopoNode::Ptr &start_node, const Eigen::Vector3f &goal,
    std::vector<TopoNode::Ptr> &path, double time_out,
    float path_cost_weight, float previous_path_cost_factor,
    const std::unordered_set<pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash> &last_path,
    float semantic_cost_weight) {
  path.clear();
  if (start_node == nullptr || !start_node->center_.allFinite() || !goal.allFinite())
    return false;

  path_cost_weight = std::max(0.0f, path_cost_weight);
  previous_path_cost_factor = std::clamp(previous_path_cost_factor, 0.0f, 1.0f);
  semantic_cost_weight = std::max(0.0f, semantic_cost_weight);

  std::unordered_map<TopoNode::Ptr, float> g_score;
  std::unordered_map<TopoNode::Ptr, float> f_score;
  std::unordered_map<TopoNode::Ptr, float> geometry_score;
  std::unordered_map<TopoNode::Ptr, float> risk_score;
  std::unordered_map<TopoNode::Ptr, TopoNode::Ptr> parent_map;
  std::unordered_set<TopoNode::Ptr> closed;
  using QueueEntry = std::pair<float, TopoNode::Ptr>;
  std::priority_queue<QueueEntry, std::vector<QueueEntry>, std::greater<QueueEntry>> open;
  const auto heuristic = [&goal](const TopoNode::Ptr &node) {
    return (node->center_ - goal).norm();
  };
  g_score[start_node] = 0.0f;
  f_score[start_node] = heuristic(start_node);
  geometry_score[start_node] = 0.0f;
  risk_score[start_node] = 0.0f;
  open.push({heuristic(start_node), start_node});

  TopoNode::Ptr best_node;
  float best_objective = std::numeric_limits<float>::infinity();
  float best_goal_distance = std::numeric_limits<float>::infinity();
  const auto start_time = std::chrono::steady_clock::now();

  auto isPreviousEdge = [&last_path](const TopoNode::Ptr &a, const TopoNode::Ptr &b) {
    return last_path.find({a, b}) != last_path.end() ||
           last_path.find({b, a}) != last_path.end();
  };
  auto isDeterministicallyBefore = [](const Eigen::Vector3f &a, const Eigen::Vector3f &b) {
    if (a.x() != b.x()) return a.x() < b.x();
    if (a.y() != b.y()) return a.y() < b.y();
    return a.z() < b.z();
  };

  while (!open.empty()) {
    const auto [queued_cost, current] = open.top();
    open.pop();
    const auto current_cost_it = g_score.find(current);
    const auto current_f_it = f_score.find(current);
    if (current_cost_it == g_score.end() || current_f_it == f_score.end() ||
        queued_cost > current_f_it->second + 1e-5f ||
        !closed.insert(current).second)
      continue;

    // Speculative nodes use the same A* role as every other persistent
    // TopoNode. Semantic evidence only changes edge cost; it does not turn a
    // node into a forbidden destination.
    if (current != start_node && !current->is_viewpoint_) {
      const float goal_distance = (current->center_ - goal).norm();
      // Weight geometric progress separately. Risk must not be discounted by
      // path_cost_weight, otherwise a risky node that is slightly closer to
      // the mission goal can still win the rolling endpoint selection.
      const float objective = goal_distance +
        path_cost_weight * geometry_score[current] + risk_score[current];
      const bool better = objective < best_objective - 1e-4f;
      const bool same_objective = std::abs(objective - best_objective) <= 1e-4f;
      const bool better_tie = same_objective &&
        (goal_distance < best_goal_distance - 1e-4f ||
         (std::abs(goal_distance - best_goal_distance) <= 1e-4f &&
          (best_node == nullptr ||
           isDeterministicallyBefore(current->center_, best_node->center_))));
      if (better || better_tie) {
        best_node = current;
        best_objective = objective;
        best_goal_distance = goal_distance;
      }
    }

    if (std::chrono::duration<double>(std::chrono::steady_clock::now() - start_time).count() > time_out)
      break;

    for (const auto &neighbor : current->neighbors_) {
      if (neighbor == nullptr || closed.find(neighbor) != closed.end())
        continue;
      const auto weight_it = current->weight_.find(neighbor);
      if (weight_it == current->weight_.end() || !std::isfinite(weight_it->second))
        continue;
      float edge_cost = std::max(0.0f, weight_it->second);
      if (isPreviousEdge(current, neighbor))
        edge_cost *= previous_path_cost_factor;
      const float edge_length = (neighbor->center_ - current->center_).norm();
      const float clearance_penalty = edgeClearancePenalty(
        current, neighbor, edge_length);
      const float semantic_risk = edgeSemanticRisk(current, neighbor);
      const float semantic_barrier = -std::log(std::max(
        1e-3f, 1.0f - semantic_risk));
      const float semantic_penalty =
        semantic_cost_weight * edge_length * semantic_barrier;
      const float risk_penalty = semantic_penalty + clearance_penalty;
      const float geometry_edge_cost = edge_cost;
      edge_cost += risk_penalty;
      const float tentative_cost = current_cost_it->second + edge_cost;
      const auto neighbor_cost_it = g_score.find(neighbor);
      if (neighbor_cost_it == g_score.end() || tentative_cost < neighbor_cost_it->second - 1e-5f) {
        g_score[neighbor] = tentative_cost;
        f_score[neighbor] = tentative_cost + heuristic(neighbor);
        geometry_score[neighbor] = geometry_score[current] + geometry_edge_cost;
        risk_score[neighbor] = risk_score[current] + risk_penalty;
        parent_map[neighbor] = current;
        // This is a bounded rolling search.  Queue by f=g+h so the timeout
        // reaches the forward mission corridor and speculative risk points;
        // using g alone turns this into Dijkstra and starves far branches.
        open.push({tentative_cost + heuristic(neighbor), neighbor});
      }
    }
  }

  if (best_node == nullptr)
    return false;
  for (auto current = best_node; current != nullptr;) {
    path.push_back(current);
    if (current == start_node)
      break;
    const auto parent_it = parent_map.find(current);
    if (parent_it == parent_map.end()) {
      path.clear();
      return false;
    }
    current = parent_it->second;
  }
  std::reverse(path.begin(), path.end());
  return path.size() >= 2 && path.front() == start_node;
}

void TopoGraph::cauculateMemoryConsumption() {
  size_t graph_cost = 0;
  size_t graph_cost2 = 0;

  double node_size = 0;
  double nbr_size = 0;
  double ur_nbr_size = 0;
  for (auto &[_, region] : reg_map_idx2ptr_) {
    size_t single_cost = 0;
    size_t single_cost2 = 0;
    if (region->topo_nodes_.empty())
      continue;
    for (auto &topo : region->topo_nodes_) {
      node_size++;
      nbr_size += topo->neighbors_.size();
      ur_nbr_size += topo->unreachable_nbrs_.size();
      if (topo->neighbors_.size() != topo->paths_.size()) {
        ROS_ERROR("memory error 644");
        exit(1);
      }
      if (topo->neighbors_.size() != topo->weight_.size()) {
        ROS_ERROR("memory error 648");
        exit(1);
      }

      single_cost += sizeof(bool);                                                       // is_viewpoint_
      single_cost += sizeof(float);                                                      // yaw_
      single_cost += sizeof(Eigen::Vector3f);                                            // center
      single_cost += (sizeof(TopoNode::Ptr) + 1) * topo->neighbors_.size();              // neighbors_
      single_cost += (sizeof(TopoNode::Ptr) + 2) * topo->unreachable_nbrs_.size();       // unreachable_nbrs_
      single_cost += (sizeof(float) + sizeof(TopoNode::Ptr) + 1) * topo->weight_.size(); // weight_
      single_cost2 = single_cost;
      single_cost2 -= (sizeof(TopoNode::Ptr) + 1) * topo->neighbors_.size();
      for (auto &nei : topo->neighbors_) {
        single_cost += sizeof(Eigen::Vector3f) * topo->paths_[nei].size(); // paths_
        single_cost2 += sizeof(Eigen::Vector3f) * topo->paths_[nei].size() / 2.0;
      }
    }
    single_cost += (sizeof(Eigen::Vector3i) + sizeof(RegionNode::Ptr) + 1); // region_node key-value pair
    graph_cost += single_cost;
    graph_cost2 += single_cost2;
  }
  for (auto &topo : history_odom_nodes_) {
    size_t single_cost = 0;
    size_t single_cost2 = 0;
    single_cost += sizeof(bool);                                                       // is_viewpoint_
    single_cost += sizeof(float);                                                      // yaw_
    single_cost += sizeof(Eigen::Vector3f);                                            // center
    single_cost += (sizeof(TopoNode::Ptr) + 1) * topo->neighbors_.size();              // neighbors_
    single_cost += (sizeof(TopoNode::Ptr) + 2) * topo->unreachable_nbrs_.size();       // unreachable_nbrs_
    single_cost += (sizeof(float) + sizeof(TopoNode::Ptr) + 1) * topo->weight_.size(); // weight_
    single_cost2 = single_cost;
    single_cost2 -= (sizeof(TopoNode::Ptr) + 1) * topo->neighbors_.size();
    for (auto &nei : topo->neighbors_) {
      single_cost += sizeof(Eigen::Vector3f) * topo->paths_[nei].size(); // paths_
      single_cost2 += sizeof(Eigen::Vector3f) * topo->paths_[nei].size() / 2.0;
    }

    single_cost += (sizeof(Eigen::Vector3i) + sizeof(RegionNode::Ptr) + 1); // region_node key-value pair
    graph_cost += single_cost;
    graph_cost2 += single_cost2;
  }

}

int TopoGraph::getBoxId(const Eigen::Vector3f &pt) {
  auto inbox = [&](const Eigen::Vector3f &pt, const Eigen::Vector3f &min, const Eigen::Vector3f &max) -> bool {
    for (size_t i = 0; i < 3; i++) {
      if (pt(i) < min(i) || pt(i) > max(i))
        return false;
    }
    return true;
  };

  for (size_t i = 0; i < lidar_map_interface_->lp_->box_num_; i++) {
    Eigen::Vector3f min_ = lidar_map_interface_->lp_->global_box_min_boundary_vec_[i];
    Eigen::Vector3f max_ = lidar_map_interface_->lp_->global_box_max_boundary_vec_[i];
    if (inbox(pt, min_, max_))
      return i;
  }
  return -1;
}

void BubbleUnionSet::unionSetCluster(const vector<BubbleNode::Ptr> &bubbles, vector<TopoNode::Ptr> &topos, Eigen::Vector3f &center) {
  auto is_bubble_connected = [&](BubbleNode::Ptr b1, BubbleNode::Ptr b2) -> bool {
    double center_distance = (b1->center_ - b2->center_).norm();
    return center_distance < (b1->radius_ + b2->radius_) - 0.5;
  };
  init(bubbles);
  for (size_t i = 0; i < bubbles.size(); i++)
    for (int j = i + 1; j < bubbles.size(); j++)
      if (is_bubble_connected(bubbles[i], bubbles[j]))
        merge(bubbles[i], bubbles[j]);
  getClusters();
  // getTopoNodes(topos, center);
  for (auto &tpair : topo_map) {
    auto node = tpair.second;
    if (node->bubbles_.empty())
      continue;
    BubbleNode::Ptr max_raduis_bubble = node->bubbles_[0];
    BubbleNode::Ptr center_big_bubble = node->bubbles_[0];
    double dis2center = (center_big_bubble->center_ - center).norm();
    for (auto &b : node->bubbles_) {
      if (b->radius_ > max_raduis_bubble->radius_)
        max_raduis_bubble = b; // 半径最大的bubble
      double dis2center_now = (b->center_ - center).norm();
      if (dis2center_now < dis2center && b->radius_ > min_topobubble_radius_)
        center_big_bubble = b; // 半径大于阈值，距离中心最近的bubble
    }
    if (center_big_bubble->radius_ > min_topobubble_radius_)
      node->center_ = center_big_bubble->center_;
    else
      node->center_ = max_raduis_bubble->center_;
    node->bubble_radius_ = static_cast<float>(
      center_big_bubble->radius_ > min_topobubble_radius_ ?
      center_big_bubble->radius_ : max_raduis_bubble->radius_);
    node->is_viewpoint_ = false;
    topos.push_back(node);
    vector<BubbleNode::Ptr>().swap(node->bubbles_);
  }
}

void TopoGraph::overlap(vector<TopoNode::Ptr> &set1, vector<TopoNode::Ptr> &set2, vector<TopoNode::Ptr> &overlap) {
  overlap.clear();
  if (set1.empty() || set2.empty()) return;

  // TopoNode centers are regenerated from Bubble clusters and can drift by
  // more than 1 mm between updates. Exact voxel hashing would therefore mark
  // stable nodes as "removed" and reinsert them every frame.
  const float max_match_distance_m = static_cast<float>(
    std::max(semantic_node_match_distance_, 1.0));
  const float max_match_distance_sq = max_match_distance_m * max_match_distance_m;

  std::vector<NodeMatchCandidate> candidates;
  candidates.reserve(set1.size() * std::min<size_t>(set2.size(), 16));
  for (size_t i = 0; i < set1.size(); ++i) {
    const auto &old_node = set1[i];
    if (!old_node) continue;
    Eigen::Vector3i old_region;
    getIndex(old_node->center_, old_region);
    for (size_t j = 0; j < set2.size(); ++j) {
      const auto &new_node = set2[j];
      if (!new_node) continue;
      Eigen::Vector3i new_region;
      getIndex(new_node->center_, new_region);
      if ((old_region - new_region).cwiseAbs().maxCoeff() > 1)
        continue;
      const float dist_sq = (old_node->center_ - new_node->center_).squaredNorm();
      if (dist_sq <= max_match_distance_sq) {
        candidates.push_back(NodeMatchCandidate{i, j, dist_sq});
      }
    }
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const NodeMatchCandidate &lhs, const NodeMatchCandidate &rhs) {
              return lhs.dist_sq < rhs.dist_sq;
            });

  std::vector<bool> old_used(set1.size(), false);
  std::vector<bool> new_used(set2.size(), false);
  for (const auto &candidate : candidates) {
    if (old_used[candidate.old_index] || new_used[candidate.new_index])
      continue;
    old_used[candidate.old_index] = true;
    new_used[candidate.new_index] = true;
    overlap.push_back(set2[candidate.new_index]);
  }
}

void TopoGraph::setdiff(vector<TopoNode::Ptr> &set1, vector<TopoNode::Ptr> &set2, vector<TopoNode::Ptr> &set_1diff2) {
  set_1diff2.clear();
  if (set1.empty()) return;

  // See overlap(): use a spatial tolerance instead of exact millimeter bins.
  const float max_match_distance_m = static_cast<float>(
    std::max(semantic_node_match_distance_, 1.0));
  const float max_match_distance_sq = max_match_distance_m * max_match_distance_m;

  std::vector<NodeMatchCandidate> candidates;
  candidates.reserve(set1.size() * std::min<size_t>(set2.size(), 16));
  for (size_t i = 0; i < set1.size(); ++i) {
    const auto &old_node = set1[i];
    if (!old_node) continue;
    Eigen::Vector3i old_region;
    getIndex(old_node->center_, old_region);
    for (size_t j = 0; j < set2.size(); ++j) {
      const auto &new_node = set2[j];
      if (!new_node) continue;
      Eigen::Vector3i new_region;
      getIndex(new_node->center_, new_region);
      if ((old_region - new_region).cwiseAbs().maxCoeff() > 1)
        continue;
      const float dist_sq = (old_node->center_ - new_node->center_).squaredNorm();
      if (dist_sq <= max_match_distance_sq) {
        candidates.push_back(NodeMatchCandidate{i, j, dist_sq});
      }
    }
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const NodeMatchCandidate &lhs, const NodeMatchCandidate &rhs) {
              return lhs.dist_sq < rhs.dist_sq;
            });

  std::vector<bool> old_used(set1.size(), false);
  std::vector<bool> new_used(set2.size(), false);
  for (const auto &candidate : candidates) {
    if (old_used[candidate.old_index] || new_used[candidate.new_index])
      continue;
    old_used[candidate.old_index] = true;
    new_used[candidate.new_index] = true;
  }
  for (size_t i = 0; i < set1.size(); ++i) {
    if (!old_used[i] && set1[i]) {
      set_1diff2.push_back(set1[i]);
    }
  }
}

void TopoGraph::removeNodes(vector<TopoNode::Ptr> &nodes) {

  // region_set
  for (auto &node : nodes) {
    if (node == nullptr)
      continue;
    Eigen::Vector3i region_idx;
    getIndex(node->center_, region_idx);
    auto region_node = getRegionNode(region_idx);
    ROS_ASSERT(region_node != nullptr);
    // if (region_node == nullptr) {
    //   continue;
    //   debug_exit("TopoGraph::removeNodes :region_node == nullptr ");
    // }
    region_node->topo_nodes_.erase(node);
  }

  // nbrs
  for (auto &node : nodes) {
    if (node == nullptr)
      continue;
    for (auto &nbr : node->neighbors_) {
      // if (nbr->is_history_odom_node_)
      //   continue;
      nbr->neighbors_.erase(node);
      nbr->paths_.erase(node);
      nbr->weight_.erase(node);
      nbr->unreachable_nbrs_.erase(node);
    }
    node->unreachable_nbrs_.clear();
    node->neighbors_.clear();
    node->weight_.clear();
    node->paths_.clear();
  }
}

void TopoGraph::updateRemainedConnections(vector<TopoNode::Ptr> &nodes) {

  // 处理已有的邻居：检查，如果不行就重新搜索
  auto checkNbr = [&](PtrPair::iter_elem &elem) {
    auto node = elem.p1;
    auto nbr = elem.p2;

    vector<Eigen::Vector3f> path = node->paths_[nbr];
    bool safe = parallel_bubble_astar_->collisionCheck_shortenPath(path);
    if (safe) {
      elem.insert = true;
      elem.path = path;
      return;
    }
    // 并不安全：重新搜路

    path.clear();
    // int res =
    // parallel_bubble_astar_->search(node->center_, nbr->center_, path, update_connection_timeout);
    int res = searchPathWithBoundary(node->center_, nbr->center_, update_connection_timeout, path);

    if (res == ParallelBubbleAstar::REACH_END && parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
      elem.insert = true;
      elem.path = path;
    } else {
      elem.insert = false;
    }
  };
  // 处理可能的邻居：搜一条路看看
  auto testPreNbr = [&](PtrPair::iter_elem &elem) {
    auto node = elem.p1;
    auto pre_nbr = elem.p2;
    // if ((node->center_ - pre_nbr->center_).norm() > 3.0) {
    //   elem.insert = false;
    //   return;
    // }
    vector<Eigen::Vector3f> path;
    int res = searchPathWithBoundary(node->center_, pre_nbr->center_, update_connection_timeout, path);
    if (res == ParallelBubbleAstar::REACH_END && parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
      elem.insert = true;
      elem.path = path;
    } else {
      elem.insert = false;
    }
  };
  PtrPair edge2test, edge2check;
  for (auto &node : nodes) {
    vector<TopoNode::Ptr> pre_nbrs;
    getPreNbrs(node, pre_nbrs);
    unordered_set<TopoNode::Ptr> pre_nbrs_set(pre_nbrs.begin(), pre_nbrs.end());
    for (auto &nbr : node->neighbors_) {
      if (nbr->is_history_odom_node_)
        continue;
      pre_nbrs_set.insert(nbr);
    }
    unordered_set<TopoNode::Ptr> pre_nbrs_set_tmp;
    unordered_map<TopoNode::Ptr, uint8_t> unreachable_nbrs_tmp;
    for (auto &pre_nbr : pre_nbrs_set) {
      if (node->unreachable_nbrs_.count(pre_nbr) && node->unreachable_nbrs_[pre_nbr] > 2) {
        continue;
      }
      pre_nbrs_set_tmp.insert(pre_nbr);
    }
    for (auto &pre_nbr : node->unreachable_nbrs_) {
      if (pre_nbrs_set.count(pre_nbr.first) && pre_nbr.first != odom_node_)
        unreachable_nbrs_tmp.insert(pre_nbr);
    }
    pre_nbrs_set_tmp.swap(pre_nbrs_set);
    node->unreachable_nbrs_.swap(unreachable_nbrs_tmp);
    for (auto &pre_nbr : pre_nbrs_set) {
      if (node->neighbors_.find(pre_nbr) == node->neighbors_.end()) {
        edge2test.insert(node, pre_nbr);
        // testPreNbr(node, pre_nbr);
      } else {
        // checkNbr(node, pre_nbr);
        edge2check.insert(node, pre_nbr);
      }
    }
  }
  edge2test.flatten();
  edge2check.flatten();
  omp_set_num_threads(6);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &elem : edge2test.flatten_data) {
    testPreNbr(elem);
  }
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &elem : edge2check.flatten_data) {
    checkNbr(elem);
  }
  edge2test.flatten_data.insert(edge2test.flatten_data.end(), edge2check.flatten_data.begin(), edge2check.flatten_data.end());
  for (auto &elem : edge2test.flatten_data) {
    if (elem.insert) {
      auto node1 = elem.p1;
      auto node2 = elem.p2;
      node1->paths_[node2] = elem.path;
      std::reverse(elem.path.begin(), elem.path.end());
      node2->paths_[node1] = elem.path;
      double cost;
      parallel_bubble_astar_->calculatePathCost(elem.path, cost);
      node1->unreachable_nbrs_.erase(node2);
      node2->unreachable_nbrs_.erase(node1);
      node1->neighbors_.insert(node2);
      node2->neighbors_.insert(node1);
      node1->weight_[node2] = cost;
      node2->weight_[node1] = cost;
    } else {
      auto node1 = elem.p1;
      auto node2 = elem.p2;
      node1->neighbors_.erase(node2);
      node2->neighbors_.erase(node1);
      node1->weight_.erase(node2);
      node2->weight_.erase(node1);
      node1->paths_.erase(node2);
      node2->paths_.erase(node1);
      if (node1->unreachable_nbrs_.count(node2)) {
        node1->unreachable_nbrs_[node2]++;
      } else {
        node1->unreachable_nbrs_[node2] = 1;
      }
      if (node2->unreachable_nbrs_.count(node1)) {
        node2->unreachable_nbrs_[node1]++;
      } else {
        node2->unreachable_nbrs_[node1] = 1;
      }
    }
  }
}

void TopoGraph::getPreNbrs(TopoNode::Ptr &node, vector<TopoNode::Ptr> &nbrs) {
  Eigen::Vector3i idx, odom_idx;
  getIndex(node->center_, idx);
  nbrs.clear();
  // getIndex(odom_node_->center_, odom_idx);
  // Eigen::Vector3i diff = (idx - odom_idx).cwiseAbs();
  // if (diff.maxCoeff() <= 1)
  //   nbrs.push_back(odom_node_);
  vector<Eigen::Vector3i> steps1{Eigen::Vector3i(0, 0, 0),  Eigen::Vector3i(1, 0, 0), Eigen::Vector3i(-1, 0, 0), Eigen::Vector3i(0, 1, 0),
                                 Eigen::Vector3i(0, -1, 0), Eigen::Vector3i(0, 0, 1), Eigen::Vector3i(0, 0, -1)};
  vector<Eigen::Vector3i> steps2{Eigen::Vector3i(1, 1, 0), Eigen::Vector3i(1, -1, 0), Eigen::Vector3i(-1, 1, 0), Eigen::Vector3i(-1, -1, 0),
                                 Eigen::Vector3i(1, 0, 1), Eigen::Vector3i(1, 0, -1), Eigen::Vector3i(-1, 0, 1), Eigen::Vector3i(-1, 0, -1),
                                 Eigen::Vector3i(0, 1, 1), Eigen::Vector3i(0, 1, -1), Eigen::Vector3i(0, -1, 1), Eigen::Vector3i(0, -1, -1)};

  // for (int i = 0; i < steps1.size() + steps2.size(); i++) {
  //   if (i >= steps1.size() && nbrs.size() > 4)
  //     break;
  for (int i = 0; i < steps1.size() ; i++) {
    Eigen::Vector3i step = i < steps1.size() ? steps1[i] : steps2[i - steps1.size()];
    Eigen::Vector3i nbr_idx = idx + step;
    auto nbr_region_node = getRegionNode(nbr_idx);
    if (nbr_region_node == nullptr)
      continue;
    for (auto &nbr_topo_node : nbr_region_node->topo_nodes_) {
      if (nbr_topo_node == nullptr) {
        cout << "wtf 970" << endl;
        continue;
      }
      if (nbr_topo_node == node)
        continue;
      // if (nbr_topo_node->is_viewpoint_ && node->is_viewpoint_)
      //   continue;
      nbrs.push_back(nbr_topo_node);
    }
  }
}

// void TopoGraph::getPreNbrs(TopoNode::Ptr &node, vector<TopoNode::Ptr> &nbrs) {
//   Eigen::Vector3i idx, odom_idx;
//   getIndex(node->center_, idx);
//   if (!node->is_viewpoint_) {
//     for (int i = 0; i < 3; i++) {
//       for (int j = -1; j <= 1; j++) {
//         Eigen::Vector3i idx_tmp = idx;
//         idx_tmp[i] += j;
//         auto nbr_region_node = getRegionNode(idx_tmp);
//         if (nbr_region_node == nullptr)
//           continue;
//         for (auto &nbr_topo_node : nbr_region_node->topo_nodes_) {
//           if (nbr_topo_node == nullptr)
//             continue;
//           if (nbr_topo_node == node)
//             continue;
//           nbrs.push_back(nbr_topo_node);
//         }
//       }
//     }
//   } else {
//     for (int i = -1; i <= 1; i++) {
//       for (int j = -1; j <= 1; j++) {
//         for (int k = -1; k <= 1; k++) {
//           Eigen::Vector3i idx_tmp = idx;
//           idx_tmp[0] += i;
//           idx_tmp[1] += j;
//           idx_tmp[2] += k;
//           auto nbr_region_node = getRegionNode(idx_tmp);
//           if (nbr_region_node == nullptr)
//             continue;
//           for (auto &nbr_topo_node : nbr_region_node->topo_nodes_) {
//             if (nbr_topo_node == nullptr)
//               continue;
//             if (nbr_topo_node == node)
//               continue;
//             nbrs.push_back(nbr_topo_node);
//           }
//         }
//       }
//     }
//   }
// }

void TopoGraph::insertNodes(vector<TopoNode::Ptr> &nodes, bool only_raycast) {
  // insert到region里
  if (nodes.empty())
    return;
  for (auto &node : nodes) {
    if (node == nullptr)
      continue;
    Eigen::Vector3i region_idx;
    getIndex(node->center_, region_idx);
    // else
    auto region_node = getRegionNode(region_idx);
    // if (region_node == nullptr) {
    //   continue;
    // }
    ROS_ASSERT(region_node != nullptr);
    region_node->topo_nodes_.insert(node);
  }

  // 找到邻居region和自己region的其他节点

  std::unordered_set<std::pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash> ptr_pair_set;
  vector<pair<TopoNode::Ptr, TopoNode::Ptr>> pair_vector; // 使用vector支持并行运算
  for (auto &node : nodes) {
    vector<TopoNode::Ptr> nbrs;
    getPreNbrs(node, nbrs);
    for (auto &nbr : nbrs) {
      if (ptr_pair_set.find({node, nbr}) == ptr_pair_set.end()) {
        pair_vector.push_back({node, nbr});
        ptr_pair_set.insert({node, nbr});
        ptr_pair_set.insert({nbr, node});
      }
    }
  }

  // 获得节点对的vector
  vector<vector<Eigen::Vector3f>> path_vec; // 初始是start和end两个点, 算完是path+一个zero/one表示成功/失败
  path_vec.resize(pair_vector.size());

  // 并行A*搜索路径并写入结果
  omp_set_num_threads(6);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (size_t i = 0; i < path_vec.size(); i++) {
    Eigen::Vector3f start = pair_vector[i].first->center_;
    Eigen::Vector3f end = pair_vector[i].second->center_;
    vector<Eigen::Vector3f> path;
    int res;
    if (!only_raycast) {
      res = searchPathWithBoundary(start, end, insert_node_timeout, path);
    } else
      res = parallel_bubble_astar_->search(start, end, path, insert_node_timeout, false, true);
    if (res != ParallelBubbleAstar::REACH_END)
      path.push_back(Eigen::Vector3f::Zero());
    else if (!only_raycast) {
      bool safe = parallel_bubble_astar_->collisionCheck_shortenPath(path);
      if (safe)
        path.push_back(Eigen::Vector3f::Ones());
      else
        path.push_back(Eigen::Vector3f::Zero());
    } else {
      path.push_back(Eigen::Vector3f::Ones()); // 1表示安全，0表示危险
    }
    path_vec[i].swap(path);
  }

  // 串行更新节点
  for (size_t i = 0; i < path_vec.size(); i++) {
    if (path_vec[i].back().norm() < 0.5)
      continue;
    auto node1 = pair_vector[i].first;
    auto node2 = pair_vector[i].second;
    node1->neighbors_.insert(node2);
    node2->neighbors_.insert(node1);
    path_vec[i].pop_back();
    node1->paths_[node2] = path_vec[i];
    std::reverse(path_vec[i].begin(), path_vec[i].end());
    node2->paths_[node1] = path_vec[i];
    double cost;
    parallel_bubble_astar_->calculatePathCost(path_vec[i], cost);
    node1->weight_[node2] = cost;
    node2->weight_[node1] = cost;
  }
}

size_t TopoGraph::insertSpeculativeNodes(
    const vector<Eigen::Vector3f> &centers, const vector<float> &semantic_scores,
    float bubble_radius, const Eigen::Vector3f &odom_pos,
    std::int64_t stamp_ns) {
  const auto is_stale = [stamp_ns](const TopoNode::Ptr &node) {
    if (!node || node->semantic_stamp_ns_ <= 0 || stamp_ns <= 0) return true;
    // Keep a short prediction horizon across one missed/low-confidence frame.
    // This prevents a 2 Hz semantic stream from deleting the only far-field
    // branch before the next inference result arrives.
    return std::llabs(stamp_ns - node->semantic_stamp_ns_) > 1500LL * 1000000LL;
  };
  if (centers.empty() || !lidar_map_interface_ || !parallel_bubble_astar_)
  {
    if (centers.empty()) {
      vector<TopoNode::Ptr> stale_speculative;
      for (const auto &entry : reg_map_idx2ptr_) {
        if (!entry.second) continue;
        for (const auto &node : entry.second->topo_nodes_) {
          if (node && node->role_ == TopoNodeRole::Speculative && is_stale(node)) {
            stale_speculative.emplace_back(node);
          }
        }
      }
      for (auto &node : stale_speculative) removeNode(node);
    }
    return 0;
  }
  const float min_separation = std::max(0.75F, bubble_radius);
  if (odom_node_) odom_node_->center_ = odom_pos;
  vector<TopoNode::Ptr> created;
  created.reserve(centers.size());
  std::unordered_set<TopoNode::Ptr> updated_speculative;
  size_t influenced_existing = 0;
  for (size_t center_index = 0; center_index < centers.size(); ++center_index) {
    const auto &center = centers[center_index];
    if (!center.allFinite() || !lidar_map_interface_->IsInBox(center)) continue;
    const float score = std::clamp(
      center_index < semantic_scores.size() ? semantic_scores[center_index] : 1.0F,
      0.0F, 1.0F);
    TopoNode::Ptr match;
    float match_distance = min_separation;
    for (const auto &entry : reg_map_idx2ptr_) {
      if (!entry.second) continue;
      for (const auto &node : entry.second->topo_nodes_) {
        if (!node || node->is_viewpoint_) continue;
        const float distance = (node->center_ - center).norm();
        if (distance >= match_distance) continue;
        if (node->role_ == TopoNodeRole::Speculative ||
            node->geometry_state_ == TopoGeometryState::Verified) {
          match = node;
          match_distance = distance;
        }
      }
    }
    if (match) {
      updateNodeSemantic(match, score, 1.0F, stamp_ns);
      if (match->role_ == TopoNodeRole::Speculative) {
        updated_speculative.insert(match);
      } else {
        ++influenced_existing;
      }
      continue;
    }
    auto node = std::make_shared<TopoNode>();
    node->center_ = center;
    node->bubble_radius_ = std::max(0.45F, bubble_radius);
    node->role_ = TopoNodeRole::Speculative;
    node->geometry_state_ = TopoGeometryState::Unknown;
    updateNodeSemantic(node, score, 1.0F, stamp_ns);
    created.emplace_back(std::move(node));
  }
  // Keep the speculative layer bounded to the newest semantic frame. Nodes
  // not observed in this frame are stale predictions, not geometric memory.
  vector<TopoNode::Ptr> stale_speculative;
  for (const auto &entry : reg_map_idx2ptr_) {
    if (!entry.second) continue;
    for (const auto &node : entry.second->topo_nodes_) {
      if (node && node->role_ == TopoNodeRole::Speculative &&
          is_stale(node) &&
          !updated_speculative.count(node)) {
        stale_speculative.emplace_back(node);
      }
    }
  }
  for (auto &node : stale_speculative) removeNode(node);
  if (created.empty()) return influenced_existing + updated_speculative.size();

  // Attach each semantic candidate through the same TopoGraph insertion API
  // used by ordinary nodes. Use EPIC's one-shot raycast mode for the witness;
  // this is the cheap path used by insertNodes(..., true), and avoids a
  // second full A* search or a separate semantic connection mechanism.
  size_t accepted = 0;
  for (auto &node : created) {
    vector<TopoNode::Ptr> nearby;
    if (odom_node_ && odom_node_ != node) {
      nearby.emplace_back(odom_node_);
    }
    for (const auto &entry : reg_map_idx2ptr_) {
      if (!entry.second) continue;
      for (const auto &candidate : entry.second->topo_nodes_) {
        if (!candidate || candidate == node || candidate->is_viewpoint_) continue;
        const float distance = (candidate->center_ - node->center_).norm();
        if (distance <= std::max(4.0F, 6.0F * node->bubble_radius_))
          nearby.emplace_back(candidate);
      }
    }
    std::sort(nearby.begin(), nearby.end(),
      [&node](const TopoNode::Ptr &left, const TopoNode::Ptr &right) {
        return (left->center_ - node->center_).squaredNorm() <
               (right->center_ - node->center_).squaredNorm();
    });
    const size_t limit = std::min<size_t>(nearby.size(), 4);
    vector<TopoNode::Ptr> neighbors;
    vector<vector<Eigen::Vector3f>> paths;
    neighbors.reserve(limit);
    paths.reserve(limit);
    for (size_t i = 0; i < limit; ++i) {
      const auto &neighbor = nearby[i];
      const Eigen::Vector3f delta = neighbor->center_ - node->center_;
      const float length = delta.norm();
      if (!std::isfinite(length) || length <= 1e-3F) continue;
      vector<Eigen::Vector3f> path;
      const int result = parallel_bubble_astar_->search(
        node->center_, neighbor->center_, path, insert_node_timeout,
        false, true);
      if (result == ParallelBubbleAstar::REACH_END && path.size() >= 2) {
        neighbors.push_back(neighbor);
        paths.push_back(std::move(path));
      }
    }
    insertNode(node, neighbors, paths);
    ++accepted;
  }
  return accepted + influenced_existing;
}

void TopoGraph::getRegionsToUpdate() {
  update_idx_vec_.clear();
  viewpoints_update_region_arr_.clear();
  toponodes_update_region_arr_.clear();
  unordered_set<RegionNode::Ptr> region_set;
  unordered_set<RegionNode::Ptr> occupied_region_set;
  unordered_set<RegionNode::Ptr> free_region_set;
  auto graphPoint = [this](Eigen::Vector3f point) {
    if (planar_graph_)
      point.z() = planar_z_;
    return point;
  };
  const Eigen::Vector3f lidar_pose = lidar_map_interface_->poseSnapshot();
  const auto latest_cloud = lidar_map_interface_->latestCloudSnapshot();
  const float max_ray_length = static_cast<float>(lidar_map_interface_->lp_->max_ray_length_);
  const Eigen::Vector3f graph_pose = graphPoint(lidar_pose);
  for (const auto &pt : latest_cloud.points) {
    Eigen::Vector3f pt3d = pt.getArray3fMap();
    if ((pt3d - lidar_pose).norm() > max_ray_length)
      pt3d = lidar_pose + max_ray_length * (pt3d - lidar_pose) /
                                                   (pt3d - lidar_pose).norm();
    pt3d = graphPoint(pt3d);
    Eigen::Vector3i region_idx;
    getIndex(pt3d, region_idx);
    auto region = getRegionNode(region_idx);
    if (region != nullptr) {
      region_set.insert(region);
      occupied_region_set.insert(region);
    }
  }

  selected_occupied_regions_ = occupied_region_set.size();
  // Free-ray evidence is part of EPIC's topology input, not visualization
  // metadata. It is what lets Bubble generation populate clear side
  // corridors where the depth frame has no occupied return. Keep it in the
  // same bounded region set as occupied returns.
  const auto free_cloud = lidar_map_interface_->freeSpaceSnapshot();
  const float free_layer_tolerance = planar_graph_ ?
    std::max(0.5F * lidar_map_interface_->voxelSize(), 0.05F) :
    std::numeric_limits<float>::infinity();
  for (const auto &point : free_cloud.points) {
    if (planar_graph_ && std::abs(point.z - planar_z_) > free_layer_tolerance)
      continue;
    const Eigen::Vector3f free_point = graphPoint(
      Eigen::Vector3f(point.x, point.y, point.z));
    Eigen::Vector3i region_idx;
    getIndex(free_point, region_idx);
    const auto region = getRegionNode(region_idx);
    if (region != nullptr) {
      region_set.insert(region);
      free_region_set.insert(region);
    }
  }
  selected_free_regions_ = free_region_set.size();
  for (auto &region : region_set) {
    toponodes_update_region_arr_.push_back(region);
  }
  auto shorten_by_distance_insert_update_arr = [&](vector<RegionNode::Ptr> &arr) {
    // Deduplicate before applying the update budget. Otherwise the ray-fill
    // pass can consume the budget with repeated regions and discard distinct
    // side corridors.
    unordered_set<RegionNode::Ptr> region2update(arr.begin(), arr.end());
    arr = vector<RegionNode::Ptr>(region2update.begin(), region2update.end());
    std::sort(arr.begin(), arr.end(), [this, &graphPoint, &lidar_pose](const RegionNode::Ptr &region1, const RegionNode::Ptr &region2) {
      Eigen::Vector3f lb1, hb1, lb2, hb2;
      index2boundary(region1->region_idx_, lb1, hb1);
      index2boundary(region2->region_idx_, lb2, hb2);
      Eigen::Vector3f diff1 = ((hb1 + lb1) * 0.5 - graphPoint(lidar_pose));
      Eigen::Vector3f diff2 = ((hb2 + lb2) * 0.5 - graphPoint(lidar_pose));
      double dist1 = diff1.norm();
      double dist2 = diff2.norm();
      return dist1 < dist2;
    });
    arr.resize(std::min((int)arr.size(), max_update_region_num_));
  };
  // 向四周发射射线，超过当前单位球大概一格子的范围
  double step_size = min(init_region_size_x_, init_region_size_y_);
  step_size = min(step_size, init_region_size_z_);
  step_size /= 2.0;
  for (auto &region : toponodes_update_region_arr_) {
    Eigen::Vector3f lb, hb, goal;
    index2boundary(region->region_idx_, lb, hb);
    goal = 0.5 * (lb + hb);
    goal = graphPoint(goal);
    Eigen::Vector3f dir = goal - graph_pose;
    int step_num = (int)(dir.norm() / step_size) + 1;
    dir.normalize();
    Eigen::Vector3f step = dir * step_size;
    for (int i = 0; i < step_num; ++i) {
      Eigen::Vector3f pos = graph_pose + step * i;
      Eigen::Vector3i region_idx;
      getIndex(pos, region_idx);
      auto region = getRegionNode(region_idx);
      if (region != nullptr)
        region_set.insert(region);
    }
  }

  for (auto &region : region_set) {
    toponodes_update_region_arr_.push_back(region);
  }

  shorten_by_distance_insert_update_arr(toponodes_update_region_arr_);
  for (auto &region : toponodes_update_region_arr_) {
    update_idx_vec_.push_back(region->region_idx_);
  }
}

void TopoGraph::updateSkeleton() {
  using Clock = std::chrono::steady_clock;
  const auto total_start = Clock::now();
  last_update_timing_ = TopoGraphUpdateTiming{};
  last_update_timing_.regions = toponodes_update_region_arr_.size();
  last_update_timing_.occupied_regions = selected_occupied_regions_;
  last_update_timing_.free_regions = selected_free_regions_;
  parallel_bubble_astar_->reset();
  vector<TopoNode::Ptr> nodes2insert, nodes_remained, nodes2remove, new_nodes, old_nodes;
  mutex new_nodes_mtx;
  const auto prepare_start = Clock::now();
  for (auto &region : toponodes_update_region_arr_) {
    for (auto &node : region->topo_nodes_) {
      if (!node->is_viewpoint_ && node->role_ != TopoNodeRole::Speculative)
        old_nodes.push_back(node);
    }
  }
  last_update_timing_.prepare_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - prepare_start).count();

  double bubble_cpu_ms = 0.0;
  double cluster_cpu_ms = 0.0;
  unsigned long long generated_bubbles = 0;
  unsigned long long generated_planar_bubbles = 0;
  unsigned long long generated_nodes = 0;
  const auto parallel_start = Clock::now();
  omp_set_num_threads(6);
  // clang-format off
  #pragma omp parallel for reduction(+:bubble_cpu_ms, cluster_cpu_ms, generated_bubbles, generated_planar_bubbles, generated_nodes)
  // clang-format on
  for (auto &region_ptr : toponodes_update_region_arr_) {
    Eigen::Vector3f lb, hb;
    index2boundary(region_ptr->region_idx_, lb, hb);
    vector<BubbleNode::Ptr> tmp_bubbles;
    vector<bool> check_pt_flag(check_pts_.size(), false);
    for (int i = 0; i < check_pts_.size(); ++i) {
      Eigen::Vector3f pt = check_pts_[i].getArray3fMap();
      pt += lb;
      if (!lidar_map_interface_->IsInBox(pt))
        check_pt_flag[i] = true;
    }
    const auto bubble_start = Clock::now();
    generateBubble(lb, hb, tmp_bubbles, check_pt_flag);
    bubble_cpu_ms +=
        std::chrono::duration<double, std::milli>(Clock::now() - bubble_start).count();
    generated_bubbles += tmp_bubbles.size();
    {
      std::lock_guard<std::mutex> lock(bubble_snapshot_mutex_);
      bubble_snapshots_by_region_[region_ptr->region_idx_] = tmp_bubbles;
    }
    vector<BubbleNode::Ptr> topology_bubbles;
    if (planar_graph_) {
      topology_bubbles.reserve(tmp_bubbles.size());
      for (const auto &bubble : tmp_bubbles) {
        const double dz = static_cast<double>(bubble->center_.z() - planar_z_);
        const double cross_section_sq = bubble->radius_ * bubble->radius_ - dz * dz;
        if (cross_section_sq <= bubble_min_radius_ * bubble_min_radius_)
          continue;
        Eigen::Vector3f center = bubble->center_;
        center.z() = planar_z_;
        topology_bubbles.push_back(
          std::make_shared<BubbleNode>(std::sqrt(cross_section_sq), center));
      }
    } else {
      topology_bubbles = tmp_bubbles;
    }
    generated_planar_bubbles += topology_bubbles.size();
    BubbleUnionSet::Ptr union_set_ = std::make_shared<BubbleUnionSet>(bubble_min_radius_); // TODO: 这个参数是topo节点2occ的最小距离
    vector<TopoNode::Ptr> new_nodes_region;
    Eigen::Vector3f region_center = (lb + hb) * 0.5;
    if (planar_graph_)
      region_center.z() = planar_z_;
    const auto cluster_start = Clock::now();
    union_set_->unionSetCluster(topology_bubbles, new_nodes_region, region_center);
    cluster_cpu_ms +=
        std::chrono::duration<double, std::milli>(Clock::now() - cluster_start).count();
    generated_nodes += new_nodes_region.size();
    {
      std::lock_guard<std::mutex> lock(new_nodes_mtx);
      for (auto &node : new_nodes_region) {
        if (!lidar_map_interface_->IsInBox(node->center_))
          continue;
        new_nodes.emplace_back(node);
      }
    }
  }
  last_update_timing_.parallel_wall_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - parallel_start).count();
  last_update_timing_.bubble_cpu_ms = bubble_cpu_ms;
  last_update_timing_.cluster_cpu_ms = cluster_cpu_ms;
  last_update_timing_.bubbles = static_cast<size_t>(generated_bubbles);
  last_update_timing_.planar_bubbles = static_cast<size_t>(generated_planar_bubbles);
  last_update_timing_.new_nodes = static_cast<size_t>(generated_nodes);

  // A newly observed Bubble is authoritative geometry. If it lands on a
  // speculative candidate, promote the candidate instead of keeping two
  // vertices at the same location. This preserves semantic identity while
  // changing only the geometry state.
  vector<TopoNode::Ptr> speculative_promotions;
  for (const auto &measured : new_nodes) {
    if (!measured) continue;
    TopoNode::Ptr match;
    float best_distance = std::numeric_limits<float>::infinity();
    const Eigen::Vector3i region_idx = [&]() {
      Eigen::Vector3i index;
      getIndex(measured->center_, index);
      return index;
    }();
    const auto region = getRegionNode(region_idx);
    if (region) {
      for (const auto &candidate : region->topo_nodes_) {
        if (!candidate || candidate->role_ != TopoNodeRole::Speculative) continue;
        const float distance = (candidate->center_ - measured->center_).norm();
        const float limit = std::max(1.0F,
          measured->bubble_radius_ + candidate->bubble_radius_);
        if (distance <= limit && distance < best_distance) {
          best_distance = distance;
          match = candidate;
        }
      }
    }
    if (!match) continue;
    measured->persistent_id_ = match->persistent_id_;
    measured->semantic_score_ = match->semantic_score_;
    measured->semantic_confidence_ = match->semantic_confidence_;
    measured->semantic_observations_ = match->semantic_observations_;
    measured->semantic_stamp_ns_ = match->semantic_stamp_ns_;
    speculative_promotions.emplace_back(match);
  }
  for (auto &speculative : speculative_promotions) removeNode(speculative);

  const auto diff_start = Clock::now();
  overlap(new_nodes, old_nodes, nodes_remained);
  setdiff(old_nodes, new_nodes, nodes2remove);
  setdiff(new_nodes, old_nodes, nodes2insert);
  unordered_set<TopoNode::Ptr> removed_set(nodes2remove.begin(), nodes2remove.end());
  unordered_set<std::uint64_t> active_semantic_ids;
  for (const auto &entry : reg_map_idx2ptr_) {
    if (!entry.second)
      continue;
    for (const auto &node : entry.second->topo_nodes_) {
      if (node && !removed_set.count(node) && node->persistent_id_ != 0)
        active_semantic_ids.insert(node->persistent_id_);
    }
  }
  last_update_timing_.semantic_restored_nodes =
    restoreNodeSemanticMemory(nodes2insert, active_semantic_ids);
  last_update_timing_.semantic_memory_records = semanticMemorySize();
  last_update_timing_.diff_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - diff_start).count();
  last_update_timing_.remained_nodes = nodes_remained.size();
  last_update_timing_.removed_nodes = nodes2remove.size();
  last_update_timing_.inserted_nodes = nodes2insert.size();

  const auto remove_start = Clock::now();
  removeNodes(nodes2remove);
  last_update_timing_.remove_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - remove_start).count();
  const auto reconnect_start = Clock::now();
  updateRemainedConnections(nodes_remained);
  last_update_timing_.reconnect_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - reconnect_start).count();
  const auto insert_start = Clock::now();
  insertNodes(nodes2insert);
  last_update_timing_.insert_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - insert_start).count();
  last_update_timing_.total_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
  vector<TopoNode::Ptr> unreachable_nodes;


}

vector<BubbleNode::Ptr> TopoGraph::getBubbleSnapshot() const {
  std::lock_guard<std::mutex> lock(bubble_snapshot_mutex_);
  vector<BubbleNode::Ptr> bubbles;
  for (const auto &entry : bubble_snapshots_by_region_) {
    bubbles.insert(bubbles.end(), entry.second.begin(), entry.second.end());
  }
  return bubbles;
}

void TopoGraph::updateOdomNode(Eigen::Vector3f &odom_pos, float &yaw) {
  struct PairPtrHash {
    std::size_t operator()(const std::pair<TopoNode::Ptr, TopoNode::Ptr> &p) const {
      return std::hash<TopoNode::Ptr>()(p.first) ^ std::hash<TopoNode::Ptr>()(p.second);
    }
  };

  if (planar_graph_)
    odom_pos.z() = planar_z_;
  // The odometry query node is transient, but its previous edges are still
  // better than an empty graph if the local reconnect fails. Keep the old
  // neighborhood until we have at least one valid replacement edge.
  const auto old_neighbors = odom_node_->neighbors_;
  const auto old_paths = odom_node_->paths_;
  const auto old_weights = odom_node_->weight_;
  const auto old_unreachable = odom_node_->unreachable_nbrs_;

  odom_node_->center_ = odom_pos;
  odom_node_->yaw_ = yaw;
  for (auto &nei : odom_node_->neighbors_) {
    nei->neighbors_.erase(odom_node_);
    nei->weight_.erase(odom_node_);
    nei->paths_.erase(odom_node_);
    nei->unreachable_nbrs_.erase(odom_node_);
  }
  odom_node_->neighbors_.clear();
  odom_node_->weight_.clear();
  odom_node_->paths_.clear();
  odom_node_->unreachable_nbrs_.clear();
  Eigen::Vector3i idx;
  getIndex(odom_pos, idx);
  vector<TopoNode::Ptr> pre_nbrs;
  for (int i = -1; i <= 1; i++)
    for (int j = -1; j <= 1; j++)
      for (int k = -1; k <= 1; k++) {
        Eigen::Vector3i tmp_idx = idx;
        tmp_idx(0) = idx(0) + i;
        tmp_idx(1) = idx(1) + j;
        tmp_idx(2) = idx(2) + k;
        if (tmp_idx.x() == 0 && tmp_idx.y() == 0 && tmp_idx.z() != 0)
          continue;
        auto region = getRegionNode(tmp_idx);
        if (region) {
          for (auto &topo : region->topo_nodes_) {
            if (topo == odom_node_)
              continue;
            // Viewpoints are transient EPIC query nodes. Connecting odometry
            // to every query repeats many bounded 3D A* searches and can
            // dominate the online update latency.
            if (topo->is_viewpoint_)
              continue;
            pre_nbrs.emplace_back(topo);
          }
        }
      }
  std::unordered_map<std::pair<TopoNode::Ptr, TopoNode::Ptr>, vector<Eigen::Vector3f>, PairPtrHash> edge2insert;
  mutex edge2insert_mtx;
  omp_set_num_threads(4);
  // clang-format off
  #pragma omp parallel for
  // clang-format on
  for (auto &nbr : pre_nbrs) {
    vector<Eigen::Vector3f> path;
    // Odom-to-topology is a real graph edge. In a depth-camera forest the
    // nearest BubbleNode is often occluded by a trunk, so the one-shot raycast
    // used by the exploration-only EPIC frontend is insufficient. Use the
    // bounded Bubble A* connection here as for every persistent edge.
    int res = parallel_bubble_astar_->search(odom_pos, nbr->center_, path, update_connection_timeout, false);
    if (res == ParallelBubbleAstar::REACH_END && parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
      edge2insert_mtx.lock();
      edge2insert.insert({std::make_pair(odom_node_, nbr), path});
      edge2insert_mtx.unlock();
    }
  }
  if (edge2insert.empty()) {
    odom_node_->neighbors_ = old_neighbors;
    odom_node_->paths_ = old_paths;
    odom_node_->weight_ = old_weights;
    odom_node_->unreachable_nbrs_ = old_unreachable;
    for (const auto &nei : old_neighbors) {
      if (!nei) continue;
      nei->neighbors_.insert(odom_node_);
      const auto path_it = old_paths.find(nei);
      if (path_it != old_paths.end()) {
        auto reverse_path = path_it->second;
        std::reverse(reverse_path.begin(), reverse_path.end());
        nei->paths_[odom_node_] = reverse_path;
      }
      const auto weight_it = old_weights.find(nei);
      nei->weight_[odom_node_] = weight_it != old_weights.end() ?
        weight_it->second : 0.0F;
      nei->unreachable_nbrs_.erase(odom_node_);
    }
    return;
  }
  for (auto &edge : edge2insert) {
    odom_node_->neighbors_.insert(edge.first.second);
    odom_node_->paths_.insert({edge.first.second, edge.second});
    double cost;
    // parallel_bubble_astar_->calculatePathCost(edge.second, cost);
    // odom_node_->weight_[edge.first.second] = cost;
    odom_node_->weight_[edge.first.second] = 0;
    edge.first.second->neighbors_.insert(odom_node_);
    auto reverse_path = edge.second;
    std::reverse(reverse_path.begin(), reverse_path.end());
    edge.first.second->paths_[odom_node_] = reverse_path;
    edge.first.second->weight_[odom_node_] = 0;
    // auto nbr = edge.first.second;
    // nbr->neighbors_.insert(odom_node_);
    // nbr->weight_[odom_node_] = cost;
    // vector<Eigen::Vector3f> path = edge.second;
    // std::reverse(path.begin(), path.end());
    // nbr->paths_[odom_node_] = path;
  }
  // }
}

void TopoGraph::removeNode(TopoNode::Ptr &node) {
  if (node == nullptr)
    return;
  Eigen::Vector3i region_idx;
  getIndex(node->center_, region_idx);
  auto region_node = getRegionNode(region_idx);
  if (region_node == nullptr) {
    debug_exit("TopoGraph::removeNodes :region_node == nullptr ");
  }
  region_node->topo_nodes_.erase(node);

  // nbrs
  for (auto &nbr : node->neighbors_) {
    nbr->neighbors_.erase(node);
    nbr->paths_.erase(node);
    nbr->weight_.erase(node);
    nbr->unreachable_nbrs_.erase(node);
  }
  node->unreachable_nbrs_.clear();
  node->neighbors_.clear();
  node->weight_.clear();
  node->paths_.clear();
}

void TopoGraph::insertNode(TopoNode::Ptr &new_node, vector<TopoNode::Ptr> &nbr_nodes, vector<vector<Eigen::Vector3f>> &paths) {
  Eigen::Vector3i region_idx;
  getIndex(new_node->center_, region_idx);
  auto region_node = getRegionNode(region_idx);
  if (region_node == nullptr) {
    debug_exit("TopoGraph::insertNodes :region_node == nullptr ");
  }
  region_node->topo_nodes_.insert(new_node);
  for (int i = 0; i < nbr_nodes.size(); i++) {
    new_node->neighbors_.insert(nbr_nodes[i]);
    nbr_nodes[i]->neighbors_.insert(new_node);
    auto path = paths[i];
    new_node->paths_.insert({nbr_nodes[i], path});
    std::reverse(path.begin(), path.end());
    nbr_nodes[i]->paths_.insert({new_node, path});
    double cost;
    parallel_bubble_astar_->calculatePathCost(path, cost);
    new_node->weight_[nbr_nodes[i]] = cost;
    nbr_nodes[i]->weight_[new_node] = cost;
  }
}



int TopoGraph::searchPathWithBoundary(const Eigen::Vector3f &start, const Eigen::Vector3f &end, double &time_out, vector<Eigen::Vector3f> &path) {
  Eigen::Vector3f bd_min, bd_max;
  for (int i = 0; i < 3; i++) {
    bd_min(i) = min(start(i), end(i));
    bd_max(i) = max(start(i), end(i));
  }
  bd_min -= Eigen::Vector3f(init_region_size_x_ / 2.0, init_region_size_y_ / 2.0, init_region_size_z_ / 2.0);
  bd_max += Eigen::Vector3f(init_region_size_x_ / 2.0, init_region_size_y_ / 2.0, init_region_size_z_ / 2.0);
  int res = parallel_bubble_astar_->search(start, end, path, time_out, false, false, bd_min, bd_max);
  return res;
}

double TopoGraph::getPathLength(const vector<TopoNode::Ptr> &topo_path) {
  vector<Eigen::Vector3f> path;
  for (int i = 0; i < topo_path.size() - 1; i++) {
    auto back = topo_path[i];
    auto front = topo_path[i + 1];
    for (auto &pt : back->paths_[front]) {
      path.emplace_back(pt);
    }
  }
  double length = 0.0;
  for (int i = 0; i < path.size() - 1; ++i)
    length += (path[i + 1] - path[i]).norm();
  return length;
}

bool TopoGraph::hasOverlapWithBox(const Eigen::Vector3f &low_bd, const Eigen::Vector3f &high_bd) {
  const static vector<Eigen::Vector3f> tmp_vec{{0, 0, 0}, {0, 0, 1}, {0, 1, 0}, {1, 0, 0}, {0, 1, 1}, {1, 0, 1}, {1, 1, 0}, {1, 1, 1}};
  for (auto &tmp : tmp_vec) {
    Eigen::Vector3f pt;
    for (int i = 0; i < 3; i++) {
      pt(i) = tmp(i) * low_bd(i) + (1 - tmp(i)) * high_bd(i);
    }
    if (lidar_map_interface_->IsInBox(pt))
      return true;
  }
  return false;
}
