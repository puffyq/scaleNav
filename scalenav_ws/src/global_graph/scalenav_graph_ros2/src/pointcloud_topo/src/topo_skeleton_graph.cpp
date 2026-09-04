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

class TopoGraph::SemanticSpatialIndex {
public:
  SemanticSpatialIndex(const std::vector<TopoNode::Ptr> &nodes, float cell_size_m)
      : cell_size_m_(std::max(0.5F, cell_size_m)) {
    cells_.reserve(nodes.size());
    for (const auto &node : nodes) {
      if (!node || !node->center_.allFinite()) continue;
      cells_[cellIndex(node->center_)].push_back(node);
    }
  }

  void queryEdgeNeighborhood(
      const std::vector<Eigen::Vector3f> &witness, float radius_m,
      std::vector<TopoNode::Ptr> &candidates,
      std::size_t maximum_candidates = 0) const {
    candidates.clear();
    if (witness.size() < 2 || cells_.empty()) return;

    const Eigen::Vector3f padding = Eigen::Vector3f::Constant(std::max(0.0F, radius_m));
    std::unordered_set<TopoNode::Ptr> seen;
    for (std::size_t i = 1; i < witness.size(); ++i) {
      if (!witness[i - 1].allFinite() || !witness[i].allFinite()) continue;
      const Eigen::Vector3i minimum = cellIndex(
        witness[i - 1].cwiseMin(witness[i]) - padding);
      const Eigen::Vector3i maximum = cellIndex(
        witness[i - 1].cwiseMax(witness[i]) + padding);
      for (int x = minimum.x(); x <= maximum.x(); ++x) {
        for (int y = minimum.y(); y <= maximum.y(); ++y) {
          for (int z = minimum.z(); z <= maximum.z(); ++z) {
            const auto cell = cells_.find(Eigen::Vector3i(x, y, z));
            if (cell == cells_.end()) continue;
            for (const auto &node : cell->second) {
              if (seen.insert(node).second) candidates.push_back(node);
            }
          }
        }
      }
    }
    if (maximum_candidates == 0 || candidates.size() <= maximum_candidates) return;

    // A dense history can put many semantic spheres inside one edge's
    // influence tube. Keep the strongest approximate fields first; the
    // exact polyline distance is still evaluated by edgeSemanticRisk().
    const float sigma = std::max(0.5F, 0.5F * std::max(0.5F, radius_m));
    const float sigma_sq = sigma * sigma;
    Eigen::Vector3f witness_min = witness.front();
    Eigen::Vector3f witness_max = witness.front();
    for (const auto &point : witness) {
      if (!point.allFinite()) continue;
      witness_min = witness_min.cwiseMin(point);
      witness_max = witness_max.cwiseMax(point);
    }
    std::vector<std::pair<float, TopoNode::Ptr>> ranked;
    ranked.reserve(candidates.size());
    for (const auto &node : candidates) {
      if (!node) continue;
      // This is only a ranking heuristic. The exact polyline distance is
      // evaluated once, below, for the retained candidates.
      const Eigen::Vector3f outside =
        (witness_min - node->center_).cwiseMax(Eigen::Vector3f::Zero()) +
        (node->center_ - witness_max).cwiseMax(Eigen::Vector3f::Zero());
      const float distance_sq = outside.squaredNorm();
      const float utility = std::clamp(
        node->semantic_score_ * node->semantic_confidence_, 0.0F, 1.0F);
      const float field = utility * std::exp(-0.5F * distance_sq / sigma_sq);
      ranked.emplace_back(field, node);
    }
    const std::size_t keep = std::min(maximum_candidates, ranked.size());
    std::partial_sort(ranked.begin(), ranked.begin() + keep, ranked.end(),
      [](const auto &left, const auto &right) { return left.first > right.first; });
    candidates.clear();
    candidates.reserve(keep);
    for (std::size_t i = 0; i < keep; ++i) candidates.push_back(ranked[i].second);
  }

private:
  Eigen::Vector3i cellIndex(const Eigen::Vector3f &point) const {
    return (point.array() / cell_size_m_).floor().cast<int>();
  }

  float cell_size_m_;
  std::unordered_map<Eigen::Vector3i, std::vector<TopoNode::Ptr>, Vector3iHash> cells_;
};

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

  nh.param("max_update_region_num", max_update_region_num_, 0);
  nh.param("bubble_topo/semantic_node_match_distance", semantic_node_match_distance_, 2.5);
  nh.param("bubble_topo/semantic_point_connection_candidates",
           semantic_point_connection_candidates_, 4);
  nh.param("bubble_topo/semantic_point_max_connections",
           semantic_point_max_connections_, 2);
  nh.param("bubble_topo/semantic_point_influence_m",
           semantic_point_influence_m_, 5.0);
  nh.param("bubble_topo/semantic_edge_candidate_limit",
           semantic_edge_candidate_limit_, 8);
  nh.param("bubble_topo/semantic_risk_memory_ms",
           semantic_risk_memory_ms_, 5000.0);
  semantic_risk_memory_ms_ = std::max(0.0, semantic_risk_memory_ms_);
  nh.param("bubble_topo/semantic_risk_accumulation_alpha",
           semantic_risk_accumulation_alpha_, 0.25);
  semantic_risk_accumulation_alpha_ = std::clamp(
    semantic_risk_accumulation_alpha_, 0.0, 1.0);
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
  const auto project_persistent_point = [this](const Eigen::Vector3f &point) {
    Eigen::Vector3f projected = point;
    if (planar_graph_) projected.z() = planar_z_;
    return projected;
  };
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
        clone->is_route_anchor_ = node->is_route_anchor_;
        clone->role_ = node->role_;
        clone->geometry_state_ = node->geometry_state_;
        clone->geometry_miss_count_ = node->geometry_miss_count_;
        clone->center_ = project_persistent_point(node->center_);
        clone->bubble_radius_ = node->bubble_radius_;
        clone->semantic_score_ = node->semantic_score_;
        clone->semantic_confidence_ = node->semantic_confidence_;
        clone->semantic_risk_ = node->semantic_risk_;
        clone->semantic_risk_stamp_ns_ = node->semantic_risk_stamp_ns_;
        clone->semantic_observations_ = node->semantic_observations_;
        clone->semantic_stamp_ns_ = node->semantic_stamp_ns_;
        clone->is_virtual_semantic_ = node->is_virtual_semantic_;
        clone->semantic_frame_stamp_ns_ = node->semantic_frame_stamp_ns_;
        clone->semantic_column_ = node->semantic_column_;
        clone->semantic_edges_attempted_stamp_ns_ =
          node->semantic_edges_attempted_stamp_ns_;
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
        auto projected_path = path_it->second;
        for (auto &point : projected_path) point = project_persistent_point(point);
        from->paths_[to] = projected_path;
        auto reverse_path = projected_path;
        std::reverse(reverse_path.begin(), reverse_path.end());
        to->paths_[from] = reverse_path;
      }
      const auto clearance_it = source_node->edge_clearance_.find(source_neighbor);
      if (clearance_it != source_node->edge_clearance_.end()) {
        from->edge_clearance_[to] = clearance_it->second;
        to->edge_clearance_[from] = clearance_it->second;
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
      clearance_target_m_ <= 0.0) {
    return 0.0F;
  }
  float minimum_clearance = std::numeric_limits<float>::infinity();
  if (std::isfinite(from->bubble_radius_) && from->bubble_radius_ > 1e-3F)
    minimum_clearance = std::min(minimum_clearance, from->bubble_radius_);
  if (std::isfinite(to->bubble_radius_) && to->bubble_radius_ > 1e-3F)
    minimum_clearance = std::min(minimum_clearance, to->bubble_radius_);
  const auto clearance_it = from->edge_clearance_.find(to);
  if (clearance_it != from->edge_clearance_.end() &&
      std::isfinite(clearance_it->second) && clearance_it->second >= 0.0F) {
    minimum_clearance = std::min(minimum_clearance, clearance_it->second);
  }
  if (!std::isfinite(minimum_clearance)) {
    return 0.0F;
  }
  const auto weight_it = from->weight_.find(to);
  if (weight_it != from->weight_.end() && std::isfinite(weight_it->second))
    edge_length = std::max(0.0F, weight_it->second);
  // Treat the configured target as a safety-distance scale, not a cutoff.
  // This keeps the cost continuous and lets A* distinguish two feasible
  // corridors: larger clearance produces a smaller safety cost.  The square
  // gives low-clearance edges a stronger preference without exceeding the
  // former maximum penalty (weight * length).
  const float safety_ratio = static_cast<float>(
    clearance_target_m_ / (clearance_target_m_ + std::max(0.0F, minimum_clearance)));
  const float safety_factor = safety_ratio * safety_ratio;
  return static_cast<float>(clearance_cost_weight_) * edge_length * safety_factor;
}

float TopoGraph::witnessMinimumClearance(
    const vector<Eigen::Vector3f> &path) const {
  if (path.empty() || !parallel_bubble_astar_)
    return std::numeric_limits<float>::infinity();
  vector<float> clearances;
  clearances.reserve(path.size());
  float minimum = std::numeric_limits<float>::infinity();
  for (const auto &point : path) {
    const float clearance = point.allFinite() ?
      static_cast<float>(parallel_bubble_astar_->graphClearance(point)) :
      std::numeric_limits<float>::infinity();
    clearances.push_back(clearance);
    if (std::isfinite(clearance)) minimum = std::min(minimum, clearance);
  }
  // Distance-to-obstacle is 1-Lipschitz. This is a conservative lower bound
  // for the clearance between sparse witness samples, including the neck
  // where two safe endpoint bubbles only just overlap.
  for (std::size_t i = 1; i < path.size(); ++i) {
    const float left = clearances[i - 1];
    const float right = clearances[i];
    const float length = (path[i] - path[i - 1]).norm();
    if (!std::isfinite(left) || !std::isfinite(right) || !std::isfinite(length))
      continue;
    const float segment_clearance = std::abs(left - right) >= length ?
      std::min(left, right) : 0.5F * (left + right - length);
    minimum = std::min(minimum, std::max(0.0F, segment_clearance));
  }
  return minimum;
}

std::vector<TopoNode::Ptr> TopoGraph::semanticNodes(
    const Eigen::Vector3f *origin, float maximum_distance_m,
    std::int64_t active_virtual_stamp_ns,
    size_t *inactive_virtual_nodes_skipped) const {
  std::vector<TopoNode::Ptr> nodes;
  if (inactive_virtual_nodes_skipped) *inactive_virtual_nodes_skipped = 0;
  const bool bounded = origin != nullptr && origin->allFinite() &&
    std::isfinite(maximum_distance_m) && maximum_distance_m >= 0.0F;
  const float maximum_distance_sq = bounded ?
    maximum_distance_m * maximum_distance_m :
    std::numeric_limits<float>::infinity();
  for (const auto &entry : reg_map_idx2ptr_) {
    if (!entry.second) continue;
    for (const auto &node : entry.second->topo_nodes_) {
      if (!node || node->is_viewpoint_ || node->role_ == TopoNodeRole::Odom ||
          node->semantic_observations_ == 0) continue;
      if (bounded && (node->center_ - *origin).squaredNorm() > maximum_distance_sq)
        continue;
      if (!semanticNodeActiveForPlanning(*node, active_virtual_stamp_ns)) {
        if (inactive_virtual_nodes_skipped) ++*inactive_virtual_nodes_skipped;
        continue;
      }
      nodes.emplace_back(node);
    }
  }
  return nodes;
}

std::vector<TopoNode::Ptr> TopoGraph::semanticRiskNodes(
    const Eigen::Vector3f *origin, float maximum_distance_m,
    std::int64_t reference_stamp_ns,
    size_t *inactive_virtual_nodes_skipped) const {
  std::vector<TopoNode::Ptr> nodes;
  if (inactive_virtual_nodes_skipped) *inactive_virtual_nodes_skipped = 0;
  const bool bounded = origin != nullptr && origin->allFinite() &&
    std::isfinite(maximum_distance_m) && maximum_distance_m >= 0.0F;
  const float maximum_distance_sq = bounded ?
    maximum_distance_m * maximum_distance_m :
    std::numeric_limits<float>::infinity();
  const std::int64_t risk_memory_ns = static_cast<std::int64_t>(
    std::llround(semantic_risk_memory_ms_ * 1.0e6));
  for (const auto &entry : reg_map_idx2ptr_) {
    if (!entry.second) continue;
    for (const auto &node : entry.second->topo_nodes_) {
      if (!node || node->is_viewpoint_ || node->role_ == TopoNodeRole::Odom ||
          node->semantic_observations_ == 0) continue;
      if (bounded && (node->center_ - *origin).squaredNorm() > maximum_distance_sq)
        continue;
      if (!semanticNodeActiveForRisk(*node, reference_stamp_ns, risk_memory_ns)) {
        if (inactive_virtual_nodes_skipped) ++*inactive_virtual_nodes_skipped;
        continue;
      }
      nodes.emplace_back(node);
    }
  }
  return nodes;
}

size_t TopoGraph::nodeCountWithinRadius(
    const Eigen::Vector3f &origin, float maximum_distance_m) const {
  if (!origin.allFinite()) return 0;
  const bool bounded = std::isfinite(maximum_distance_m) && maximum_distance_m >= 0.0F;
  const float maximum_distance_sq = bounded ?
    maximum_distance_m * maximum_distance_m :
    std::numeric_limits<float>::infinity();
  std::unordered_set<TopoNode::Ptr> nodes;
  for (const auto &entry : reg_map_idx2ptr_) {
    if (!entry.second) continue;
    for (const auto &node : entry.second->topo_nodes_) {
      if (!node || !node->center_.allFinite()) continue;
      if (bounded && (node->center_ - origin).squaredNorm() > maximum_distance_sq)
        continue;
      nodes.insert(node);
    }
  }
  if (odom_node_ && odom_node_->center_.allFinite() &&
      (!bounded || (odom_node_->center_ - origin).squaredNorm() <= maximum_distance_sq)) {
    nodes.insert(odom_node_);
  }
  return nodes.size();
}

float TopoGraph::edgeSemanticRisk(
    const TopoNode::Ptr &from, const TopoNode::Ptr &to,
    const std::vector<TopoNode::Ptr> *semantic_nodes,
    size_t *semantic_candidate_checks,
    const SemanticSpatialIndex *semantic_index) const {
  if (!from || !to) return 0.0F;
  float risk = std::clamp(0.5F *
    (std::clamp(from->semantic_score_ * from->semantic_confidence_, 0.0F, 1.0F) +
     std::clamp(to->semantic_score_ * to->semantic_confidence_, 0.0F, 1.0F)),
    0.0F, 1.0F);

  // A semantic node is an observation at the end of a fixed-range ray, not
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
    0.5, semantic_point_influence_m_));
  const float sigma = std::max(0.5F, 0.5F * influence);
  const float influence_sq = influence * influence;
  const std::vector<TopoNode::Ptr> snapshot = semantic_nodes == nullptr ?
    semanticNodes() : std::vector<TopoNode::Ptr>();
  const auto &semantic_snapshot = semantic_nodes == nullptr ? snapshot : *semantic_nodes;
  std::vector<TopoNode::Ptr> nearby_candidates;
  if (semantic_index != nullptr) {
    semantic_index->queryEdgeNeighborhood(
      witness, influence, nearby_candidates,
      static_cast<std::size_t>(std::max(0, semantic_edge_candidate_limit_)));
  }
  const auto &candidates = semantic_index == nullptr ? semantic_snapshot : nearby_candidates;
  for (const auto &candidate : candidates) {
    if (semantic_candidate_checks) ++*semantic_candidate_checks;
    if (!candidate || candidate->semantic_observations_ == 0) continue;
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
  return std::clamp(risk, 0.0F, 1.0F);
}

float TopoGraph::semanticRiskForEdge(const TopoNode::Ptr &from,
                                     const TopoNode::Ptr &to,
                                     const std::vector<TopoNode::Ptr> *semantic_nodes) const {
  return edgeSemanticRisk(from, to, semantic_nodes, nullptr);
}

float TopoGraph::clearanceCostForEdge(const TopoNode::Ptr &from,
                                      const TopoNode::Ptr &to) const {
  if (!from || !to) return 0.0F;
  const float edge_length = (to->center_ - from->center_).norm();
  return edgeClearancePenalty(from, to, edge_length);
}

float TopoGraph::routeEdgeCost(
    const TopoNode::Ptr &from, const TopoNode::Ptr &to,
    float path_cost_weight, float semantic_cost_weight,
    bool apply_previous_path_discount, float previous_path_cost_factor,
    const std::vector<TopoNode::Ptr> *semantic_nodes,
    size_t *semantic_candidate_checks) const {
  return routeEdgeCostIndexed(
    from, to, path_cost_weight, semantic_cost_weight,
    apply_previous_path_discount, previous_path_cost_factor,
    semantic_nodes, semantic_candidate_checks, nullptr);
}

float TopoGraph::routeEdgeCostIndexed(
    const TopoNode::Ptr &from, const TopoNode::Ptr &to,
    float path_cost_weight, float semantic_cost_weight,
    bool apply_previous_path_discount, float previous_path_cost_factor,
    const std::vector<TopoNode::Ptr> *semantic_nodes,
    size_t *semantic_candidate_checks,
    const SemanticSpatialIndex *semantic_index) const {
  if (!from || !to) return std::numeric_limits<float>::infinity();
  const float edge_length = (to->center_ - from->center_).norm();
  const auto weight_it = from->weight_.find(to);
  float geometry_cost = weight_it != from->weight_.end() &&
    std::isfinite(weight_it->second) ? std::max(0.0F, weight_it->second) : edge_length;
  if (apply_previous_path_discount) {
    geometry_cost *= std::clamp(previous_path_cost_factor, 0.0F, 1.0F);
  }
  geometry_cost *= std::max(0.0F, path_cost_weight);

  float semantic_cost = 0.0F;
  if (semantic_cost_weight > 0.0F) {
    const float risk = edgeSemanticRisk(
      from, to, semantic_nodes, semantic_candidate_checks, semantic_index);
    semantic_cost = semantic_cost_weight * edge_length *
      (-std::log(std::max(1e-3F, 1.0F - risk)));
  }
  return geometry_cost + semantic_cost +
    edgeClearancePenalty(from, to, edge_length);
}

void TopoGraph::updateNodeSemantic(const TopoNode::Ptr &node, float observation,
                                   float ema_alpha, std::int64_t stamp_ns,
                                   float observation_confidence) {
  if (!node || node->is_viewpoint_ || !std::isfinite(observation) ||
      !std::isfinite(observation_confidence))
    return;
  std::lock_guard<std::mutex> lock(semantic_memory_mutex_);
  if (node->persistent_id_ == 0)
    node->persistent_id_ = next_semantic_node_id_++;
  const float alpha = std::clamp(ema_alpha, 0.0F, 1.0F);
  const float score = std::clamp(observation, 0.0F, 1.0F);
  const float confidence = std::clamp(observation_confidence, 0.0F, 1.0F);
  const std::int64_t previous_risk_stamp_ns = node->semantic_risk_stamp_ns_;
  const float previous_risk = std::clamp(node->semantic_risk_, 0.0F, 1.0F);
  if (node->semantic_stamp_ns_ != stamp_ns) {
    node->semantic_edges_attempted_stamp_ns_ = 0;
  }
  if (node->semantic_observations_ == 0) {
    node->semantic_score_ = score;
    node->semantic_confidence_ = confidence;
  } else {
    // Semantic memory follows the current evidence. A stale maximum would
    // make one accidental high-score association permanent and would keep
    // painting/planning through a node long after the target left the view.
    node->semantic_score_ = (1.0F - alpha) * node->semantic_score_ + alpha * score;
    node->semantic_confidence_ = (1.0F - alpha) * node->semantic_confidence_ +
      alpha * confidence;
  }
  // Keep the latest patch score above for frame-relative frontier ranking,
  // while maintaining a separate bounded risk memory for route costs. Older
  // evidence decays over the configured memory horizon; each new observation
  // then contributes through a probabilistic union rather than an unbounded
  // sum. Confidence is retained separately for edge-field compatibility and
  // does not rescale the patch risk used for frontier ranking.
  float decayed_risk = previous_risk;
  if (previous_risk_stamp_ns > 0 && stamp_ns > previous_risk_stamp_ns &&
      semantic_risk_memory_ms_ > 0.0) {
    const double age_ms = static_cast<double>(stamp_ns - previous_risk_stamp_ns) / 1.0e6;
    // Normal heatmap frames arrive within roughly one second. Do not erase
    // accumulated evidence merely because the next frame arrived; decay only
    // after an observation gap, using the configured risk-memory horizon.
    const double decay_age_ms = std::max(0.0, age_ms - 1000.0);
    decayed_risk *= static_cast<float>(std::exp(-decay_age_ms / semantic_risk_memory_ms_));
  }
  const float evidence = std::clamp(
    static_cast<float>(semantic_risk_accumulation_alpha_) * score,
    0.0F, 1.0F);
  node->semantic_risk_ = (node->semantic_observations_ == 0 ||
      previous_risk_stamp_ns <= 0) ?
    score :
    std::clamp(1.0F - (1.0F - decayed_risk) * (1.0F - evidence), 0.0F, 1.0F);
  node->semantic_risk_stamp_ns_ = stamp_ns;
  ++node->semantic_observations_;
  node->semantic_stamp_ns_ = stamp_ns;
  const Eigen::Vector3i region_idx = semanticWorldRegion(
    node->center_, init_region_size_x_, init_region_size_y_, init_region_size_z_);
  semantic_memory_[node->persistent_id_] = TopoSemanticRecord{
    node->persistent_id_, node->center_, region_idx, node->semantic_score_,
    node->semantic_confidence_, node->semantic_observations_, stamp_ns,
    node->semantic_risk_, node->semantic_risk_stamp_ns_,
    node->is_virtual_semantic_, node->semantic_frame_stamp_ns_,
    node->semantic_column_};
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
    const auto existing = semantic_memory_.find(record.node_id);
    if (existing != semantic_memory_.end() &&
        existing->second.stamp_ns > record.stamp_ns) {
      continue;
    }
    semantic_memory_[record.node_id] = record;
    next_semantic_node_id_ = std::max(next_semantic_node_id_, record.node_id + 1);
  }
}

size_t TopoGraph::semanticMemorySize() const {
  std::lock_guard<std::mutex> lock(semantic_memory_mutex_);
  return semantic_memory_.size();
}

size_t TopoGraph::protectRouteGeometry(const vector<TopoNode::Ptr> &nodes) {
  size_t newly_protected = 0;
  for (const auto &node : nodes) {
    if (!node || node->is_viewpoint_ || node->role_ != TopoNodeRole::Geometric ||
        node->geometry_state_ != TopoGeometryState::Verified) {
      continue;
    }
    if (!node->is_route_anchor_) {
      node->is_route_anchor_ = true;
      ++newly_protected;
    }
  }
  return newly_protected;
}

size_t TopoGraph::routeAnchorCount() const {
  size_t count = 0;
  unordered_set<TopoNode::Ptr> seen;
  for (const auto &entry : reg_map_idx2ptr_) {
    if (!entry.second) continue;
    for (const auto &node : entry.second->topo_nodes_) {
      if (node && node->is_route_anchor_ && seen.insert(node).second) ++count;
    }
  }
  return count;
}

VirtualSemanticPruneResult TopoGraph::pruneVirtualSemanticNodes(
    const Eigen::Vector3f &position,
    const Eigen::Vector3f &forward_direction,
    float backtrack_margin_m,
    size_t maximum_nodes,
    std::int64_t active_stamp_ns,
    const unordered_set<std::uint64_t> &protected_ids) {
  VirtualSemanticPruneResult result;
  vector<TopoNode::Ptr> virtual_nodes;
  unordered_set<TopoNode::Ptr> seen;
  for (const auto &entry : reg_map_idx2ptr_) {
    if (!entry.second) continue;
    for (const auto &node : entry.second->topo_nodes_) {
      if (!isVirtualSemanticEndpoint(node) || !seen.insert(node).second) {
        continue;
      }
      virtual_nodes.push_back(node);
    }
  }
  result.before = virtual_nodes.size();
  if (virtual_nodes.empty()) return result;

  const float margin = std::max(0.0F, backtrack_margin_m);
  Eigen::Vector3f forward = forward_direction;
  const bool have_forward = forward.allFinite() && forward.norm() > 1e-3F;
  if (have_forward) forward.normalize();
  auto protected_node = [&](const TopoNode::Ptr &node) {
    return node &&
      ((node->persistent_id_ != 0 && protected_ids.count(node->persistent_id_) != 0) ||
       semanticNodeActiveForPlanning(*node, active_stamp_ns));
  };

  unordered_set<TopoNode::Ptr> remove_set;
  if (have_forward && position.allFinite()) {
    for (const auto &node : virtual_nodes) {
      if (protected_node(node)) continue;
      const float longitudinal = (node->center_ - position).dot(forward);
      if (std::isfinite(longitudinal) && longitudinal < -margin) {
        remove_set.insert(node);
        ++result.removed_behind;
      }
    }
  }

  const size_t remaining_after_behind = virtual_nodes.size() - remove_set.size();
  if (maximum_nodes > 0 && remaining_after_behind > maximum_nodes) {
    vector<TopoNode::Ptr> capacity_candidates;
    capacity_candidates.reserve(remaining_after_behind);
    for (const auto &node : virtual_nodes) {
      if (!remove_set.count(node) && !protected_node(node)) {
        capacity_candidates.push_back(node);
      }
    }
    // Remove the least useful history first. Current-frame and accepted-route
    // identities are protected above. Nearby, repeatedly observed, high-risk
    // evidence is retained ahead of weak or distant one-shot observations.
    std::sort(capacity_candidates.begin(), capacity_candidates.end(),
      [&](const TopoNode::Ptr &left, const TopoNode::Ptr &right) {
        const float left_utility = std::clamp(
          left->semantic_score_ * left->semantic_confidence_, 0.0F, 1.0F);
        const float right_utility = std::clamp(
          right->semantic_score_ * right->semantic_confidence_, 0.0F, 1.0F);
        if (std::abs(left_utility - right_utility) > 1e-5F)
          return left_utility < right_utility;
        if (left->semantic_observations_ != right->semantic_observations_)
          return left->semantic_observations_ < right->semantic_observations_;
        if (left->semantic_stamp_ns_ != right->semantic_stamp_ns_)
          return left->semantic_stamp_ns_ < right->semantic_stamp_ns_;
        return (left->center_ - position).squaredNorm() >
          (right->center_ - position).squaredNorm();
      });
    size_t excess = remaining_after_behind - maximum_nodes;
    for (const auto &node : capacity_candidates) {
      if (excess == 0) break;
      if (remove_set.insert(node).second) {
        --excess;
        ++result.removed_capacity;
      }
    }
  }

  result.removed_ids.reserve(remove_set.size());
  for (auto &node : virtual_nodes) {
    if (!remove_set.count(node)) continue;
    if (node->persistent_id_ != 0) result.removed_ids.push_back(node->persistent_id_);
    removeNode(node);
  }
  if (!result.removed_ids.empty()) {
    std::lock_guard<std::mutex> lock(semantic_memory_mutex_);
    for (const auto id : result.removed_ids) semantic_memory_.erase(id);
  }
  result.after = result.before - remove_set.size();
  return result;
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
      node->semantic_risk_ = nearest->risk > 0.0F ? nearest->risk :
        nearest->score;
      node->semantic_risk_stamp_ns_ = nearest->risk_stamp_ns > 0 ?
        nearest->risk_stamp_ns : nearest->stamp_ns;
      node->semantic_observations_ = nearest->observations;
      node->semantic_stamp_ns_ = nearest->stamp_ns;
      // A freshly measured Bubble is ordinary geometry even when it inherits
      // semantic evidence from a former virtual endpoint at the same place.
      node->is_virtual_semantic_ = nearest->is_virtual &&
        node->geometry_state_ == TopoGeometryState::Unknown;
      node->semantic_frame_stamp_ns_ = nearest->frame_stamp_ns;
      node->semantic_column_ = nearest->column;
      claimed_records.insert(nearest->node_id);
      semantic_memory_[nearest->node_id].center = node->center_;
      semantic_memory_[nearest->node_id].region_idx = node_region;
      semantic_memory_[nearest->node_id].is_virtual = node->is_virtual_semantic_;
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
  // Bubble generation runs one region per OpenMP worker. Recursive splits can
  // land on a neighbouring region at a floating-point boundary, so this path
  // may lazily insert into the shared unordered_map from multiple workers.
  // Serialise both lookup and insertion: concurrent insertion can invalidate
  // buckets while another worker is reading them. The recorded crash itself
  // was a Bubble-recursion stack overflow, but this is a separate race on the
  // same OpenMP rebuild path.
  std::lock_guard<std::mutex> lock(region_map_mutex_);
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
  // C++ integer conversion truncates toward zero.  For points just below a
  // rolling-map boundary that maps them to the wrong (non-negative) region,
  // so removal/merge can look in a different RegionNode than insertion.
  region_idx_.x() = static_cast<int>(std::floor(
    (point[0] - min_bd[0]) / std::max(init_region_size_x_, 1e-6)));
  region_idx_.y() = static_cast<int>(std::floor(
    (point[1] - min_bd[1]) / std::max(init_region_size_y_, 1e-6)));
  region_idx_.z() = static_cast<int>(std::floor(
    (point[2] - min_bd[2]) / std::max(init_region_size_z_, 1e-6)));
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
                            float semantic_cost_weight, float max_search_radius_m,
                            TopoGraphSearchStats *search_stats,
                            std::int64_t active_virtual_semantic_stamp_ns,
                            float previous_path_cost_factor,
                            float path_cost_weight,
                            const Eigen::Vector3f *mission_goal) {
  path.clear();
  if (search_stats) *search_stats = {};
  if (start_node == nullptr || end_node == nullptr ||
      !start_node->center_.allFinite() || !end_node->center_.allFinite())
    return false;
  const bool bounded_search = std::isfinite(max_search_radius_m) && max_search_radius_m >= 0.0F;
  const float radius_sq = bounded_search ? max_search_radius_m * max_search_radius_m :
    std::numeric_limits<float>::infinity();
  const auto within_search_radius = [&](const TopoNode::Ptr &node) {
    return node != nullptr && (!bounded_search ||
      (node->center_ - start_node->center_).squaredNorm() <= radius_sq);
  };
  if (!within_search_radius(end_node))
    return false;
  const float semantic_query_radius = bounded_search ?
    max_search_radius_m + static_cast<float>(std::max(0.0, semantic_point_influence_m_)) :
    std::numeric_limits<float>::infinity();
  const std::vector<TopoNode::Ptr> semantic_nodes = semantic_cost_weight > 0.0F ?
    semanticRiskNodes(
      &start_node->center_, semantic_query_radius,
      active_virtual_semantic_stamp_ns,
      search_stats ? &search_stats->semantic_inactive_virtual_nodes_skipped : nullptr) :
    std::vector<TopoNode::Ptr>();
  const SemanticSpatialIndex semantic_index(
    semantic_nodes, static_cast<float>(std::max(0.5, semantic_point_influence_m_)));
  if (search_stats) search_stats->semantic_query_nodes = semantic_nodes.size();

  std::unordered_map<TopoNode::Ptr, float> g_score, f_score;
  std::unordered_map<TopoNode::Ptr, TopoNode::Ptr> parent_map;
  std::unordered_set<TopoNode::Ptr> close_set, open_set_set_;
  float tie_breaker_ = 1.0 + 1.0 / 1000;
  std::priority_queue<std::pair<float, TopoNode::Ptr>, std::vector<std::pair<float, TopoNode::Ptr>>, std::greater<std::pair<float, TopoNode::Ptr>>>
  open_set;
  path_cost_weight = std::max(0.0F, path_cost_weight);
  (void)mission_goal;
  auto getHeuristic = [&](const TopoNode::Ptr &n) -> float {
    return path_cost_weight * tie_breaker_ * (n->center_ - end_node->center_).norm();
  };
  semantic_cost_weight = std::max(0.0F, semantic_cost_weight);
  previous_path_cost_factor = std::clamp(previous_path_cost_factor, 0.0F, 1.0F);
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
    if (search_stats) search_stats->expanded_nodes = close_set.size();
    if (cur_node == end_node) {
      backtrack();
      return true;
    }
    if ((ros::Time::now() - t1).toSec() > time_out) {
      // ROS_ERROR("topo a* timeout");
      if (search_stats) search_stats->timed_out = true;
      return false;
    }
    for (auto &neighbor : cur_node->neighbors_) {
      // if (!neighbor->reachable_)
      //   continue;
      if (!within_search_radius(neighbor) || close_set.find(neighbor) != close_set.end() ||
          neighbor->geometry_state_ != TopoGeometryState::Verified)
        continue;
      // Backward edges remain eligible. A valid route around a large obstacle
      // can require an initial retreat; its extra distance is already charged
      // in g and must not be converted into a false reachability failure.
      if (search_stats) ++search_stats->edge_evaluations;

      const float edge_length = (neighbor->center_ - cur_node->center_).norm();
      const auto witness_it = cur_node->paths_.find(neighbor);
      if (witness_it == cur_node->paths_.end() || witness_it->second.size() < 2)
        continue;
      const auto edge_clearance_it = cur_node->edge_clearance_.find(neighbor);
      if (edge_clearance_it != cur_node->edge_clearance_.end() &&
          std::isfinite(edge_clearance_it->second) && parallel_bubble_astar_ &&
          edge_clearance_it->second <=
            static_cast<float>(parallel_bubble_astar_->safe_distance_) + 0.02F) {
        continue;
      }
      if (parallel_bubble_astar_) {
        const float live_witness_clearance = witnessMinimumClearance(witness_it->second);
        if (std::isfinite(live_witness_clearance) &&
            live_witness_clearance <=
              static_cast<float>(parallel_bubble_astar_->safe_distance_) + 0.02F) {
          continue;
        }
      }
      const float clearance_penalty = edgeClearancePenalty(
        cur_node, neighbor, edge_length);
      float semantic_penalty = 0.0F;
      if (semantic_cost_weight > 0.0F) {
        const float semantic_risk = edgeSemanticRisk(
          cur_node, neighbor, &semantic_nodes,
          search_stats ? &search_stats->semantic_candidate_checks : nullptr,
          &semantic_index);
        const float semantic_barrier = -std::log(std::max(
          1e-3F, 1.0F - semantic_risk));
        semantic_penalty = semantic_cost_weight * edge_length * semantic_barrier;
      }
      float geometry_cost = path_cost_weight * cur_node->weight_[neighbor];
      if (kino && last_path.find({cur_node, neighbor}) != last_path.end()) {
        geometry_cost *= previous_path_cost_factor;
      }
      const float tentative_g_score = g_score[cur_node] + geometry_cost +
        semantic_penalty + clearance_penalty;
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
    float semantic_cost_weight, float max_search_radius_m,
    const Eigen::Vector3f *progress_origin, float preferred_frontier_goal_forward_m,
    bool prefer_mission_goal, float preferred_frontier_goal_radial_m,
    float minimum_execution_path_m, const Eigen::Vector3f *view_direction,
    float horizontal_fov_deg, float progress_penalty_weight,
    float direction_penalty_weight, float fov_penalty_weight,
    float smoothness_penalty_weight, TopoGraphSearchStats *search_stats,
    std::int64_t active_virtual_semantic_stamp_ns,
    float frontier_goal_distance_weight,
    float frontier_semantic_detour_budget_m,
    float frontier_semantic_frame_budget_m,
    float frontier_semantic_noise_floor) {
  path.clear();
  if (search_stats) *search_stats = {};
  if (start_node == nullptr || !start_node->center_.allFinite() || !goal.allFinite())
    return false;

  path_cost_weight = std::max(0.0f, path_cost_weight);
  previous_path_cost_factor = std::clamp(previous_path_cost_factor, 0.0f, 1.0f);
  semantic_cost_weight = std::max(0.0f, semantic_cost_weight);
  preferred_frontier_goal_forward_m = std::max(0.0f, preferred_frontier_goal_forward_m);
  minimum_execution_path_m = std::max(0.0F, minimum_execution_path_m);
  progress_penalty_weight = std::max(0.0F, progress_penalty_weight);
  direction_penalty_weight = std::max(0.0F, direction_penalty_weight);
  fov_penalty_weight = std::max(0.0F, fov_penalty_weight);
  smoothness_penalty_weight = std::max(0.0F, smoothness_penalty_weight);
  // Keep the distance term bounded and dimensionless so it cannot drown out
  // semantic/clearance costs accumulated in g. A value of 1.0 gives equal
  // normalized weight to route cost and remaining mission distance.
  frontier_goal_distance_weight = std::clamp(
    frontier_goal_distance_weight, 0.0F, 10.0F);
  frontier_semantic_detour_budget_m = std::max(0.0F, frontier_semantic_detour_budget_m);
  frontier_semantic_frame_budget_m = std::max(0.0F, frontier_semantic_frame_budget_m);
  frontier_semantic_noise_floor = std::clamp(
    frontier_semantic_noise_floor, 1e-3F, 1.0F);
  const bool radial_preference_enabled = std::isfinite(preferred_frontier_goal_radial_m) &&
    preferred_frontier_goal_radial_m >= 0.0F;
  if (radial_preference_enabled) {
    preferred_frontier_goal_radial_m = std::max(0.0F, preferred_frontier_goal_radial_m);
  }
  const Eigen::Vector3f progress_ref =
    progress_origin != nullptr && progress_origin->allFinite() ?
    *progress_origin : start_node->center_;
  const bool bounded_search = std::isfinite(max_search_radius_m) && max_search_radius_m >= 0.0F;
  const float radius_sq = bounded_search ? max_search_radius_m * max_search_radius_m :
    std::numeric_limits<float>::infinity();
  const auto within_search_radius = [&](const TopoNode::Ptr &node) {
    return node != nullptr && (!bounded_search ||
      (node->center_ - start_node->center_).squaredNorm() <= radius_sq);
  };

  const Eigen::Vector3f mission_vector = goal - progress_ref;
  const float mission_span = mission_vector.norm();
  const float objective_scale = std::max(1.0F, mission_span);
  Eigen::Vector3f mission_dir = Eigen::Vector3f::Zero();
  if (mission_span > 1e-3F) mission_dir = mission_vector / mission_span;
  const float semantic_query_radius = bounded_search ?
    max_search_radius_m + static_cast<float>(std::max(0.0, semantic_point_influence_m_)) :
    std::numeric_limits<float>::infinity();
  // Frontier scoring always needs the active five-direction observations.
  // semantic_cost_weight only controls risk integrated along graph edges; it
  // must not disable semantic endpoint ranking or its per-frame reference.
  const std::vector<TopoNode::Ptr> frontier_semantic_nodes = semanticNodes(
    &start_node->center_, semantic_query_radius,
    active_virtual_semantic_stamp_ns,
    search_stats ? &search_stats->semantic_inactive_virtual_nodes_skipped : nullptr);
  // A semantic endpoint is a short-lived frontier, but its world-space risk
  // must survive until the next progress-triggered A* refresh. Keeping these
  // windows separate prevents an old point from becoming a goal while still
  // allowing the route to remember why it started a wide detour.
  const std::vector<TopoNode::Ptr> semantic_nodes = semanticRiskNodes(
    &start_node->center_, semantic_query_radius,
    active_virtual_semantic_stamp_ns, nullptr);
  const SemanticSpatialIndex semantic_index(
    semantic_nodes, static_cast<float>(std::max(0.5, semantic_point_influence_m_)));
  if (search_stats) search_stats->semantic_query_nodes = semantic_nodes.size();
  auto isPreviousEdge = [&last_path](const TopoNode::Ptr &a, const TopoNode::Ptr &b) {
    return last_path.find({a, b}) != last_path.end() ||
           last_path.find({b, a}) != last_path.end();
  };

  struct AstarEntry {
    float f;
    float h;
    float g;
    TopoNode::Ptr node;
  };
  struct AstarEntryGreater {
    bool operator()(const AstarEntry &left, const AstarEntry &right) const {
      if (std::abs(left.f - right.f) > 1e-5F) return left.f > right.f;
      if (std::abs(left.h - right.h) > 1e-5F) return left.h > right.h;
      const auto &a = left.node->center_;
      const auto &b = right.node->center_;
      if (a.x() != b.x()) return a.x() > b.x();
      if (a.y() != b.y()) return a.y() > b.y();
      return a.z() > b.z();
    }
  };
  const auto goalDistance = [&goal](const TopoNode::Ptr &node) {
    return (node->center_ - goal).norm();
  };
  const auto heuristic = [&goalDistance, path_cost_weight](const TopoNode::Ptr &node) {
    // Scale h by the same geometry coefficient used in g. This preserves the
    // configured distance/semantic trade-off and keeps the A* lower bound in
    // the same units as the accumulated edge cost.
    return path_cost_weight * goalDistance(node);
  };
  const auto forwardProgress = [&](const TopoNode::Ptr &node) {
    return mission_span > 1e-3F ?
      (node->center_ - progress_ref).dot(mission_dir) :
      (node->center_ - progress_ref).norm();
  };
  (void)preferred_frontier_goal_forward_m;
  (void)preferred_frontier_goal_radial_m;
  (void)radial_preference_enabled;
  (void)view_direction;
  (void)horizontal_fov_deg;
  (void)progress_penalty_weight;
  (void)fov_penalty_weight;
  (void)smoothness_penalty_weight;

  Eigen::Vector3f continuity_direction = Eigen::Vector3f::Zero();
  if (view_direction && view_direction->allFinite()) {
    continuity_direction = *view_direction;
    continuity_direction.z() = 0.0F;
    const float norm = continuity_direction.norm();
    if (norm > 1e-3F) continuity_direction /= norm;
    else continuity_direction.setZero();
  }

  constexpr float kGoalNodeToleranceM = 1.5F;

  const auto isActiveSemanticFrontier = [&](const TopoNode::Ptr &node) {
    return isVirtualSemanticEndpoint(node) &&
      active_virtual_semantic_stamp_ns > 0 &&
      semanticNodeActiveForPlanning(*node, active_virtual_semantic_stamp_ns);
  };
  auto isFrontierEndpoint = [&](const TopoNode::Ptr &node, float route_dist) {
    if (node == nullptr || node == start_node || node->is_viewpoint_ ||
        route_dist + 1e-3F < minimum_execution_path_m) {
      return false;
    }
    if (node->geometry_state_ == TopoGeometryState::Verified) return true;
    // Risk is a continuous ranking term.  Do not gate semantic candidates by
    // a score threshold: even a high-risk frame must expose its lowest-risk
    // semantic option so the frontier objective can choose the safest one.
    return isActiveSemanticFrontier(node);
  };

  const auto isSearchNode = [&](const TopoNode::Ptr &node) {
    return node &&
      (node->geometry_state_ == TopoGeometryState::Verified ||
       isActiveSemanticFrontier(node));
  };
  const auto accumulatedSemanticRisk = [](const TopoNode::Ptr &node) {
    if (!node) return 0.0F;
    // Keep compatibility with nodes restored from older logs that do not have
    // a populated temporal accumulator yet.
    return node->semantic_risk_ > 0.0F ?
      std::clamp(node->semantic_risk_, 0.0F, 1.0F) :
      std::clamp(node->semantic_score_, 0.0F, 1.0F);
  };

  std::unordered_map<TopoNode::Ptr, float> g_score;
  std::unordered_map<TopoNode::Ptr, float> route_distance;
  std::unordered_map<TopoNode::Ptr, float> expanded_g;
  std::unordered_map<TopoNode::Ptr, TopoNode::Ptr> parent_map;
  std::priority_queue<AstarEntry, std::vector<AstarEntry>, AstarEntryGreater> open;

  auto backtrackPath = [&](const TopoNode::Ptr &end) -> bool {
    path.clear();
    for (auto current = end; current != nullptr;) {
      path.push_back(current);
      if (current == start_node) break;
      const auto parent_it = parent_map.find(current);
      if (parent_it == parent_map.end()) {
        path.clear();
        return false;
      }
      current = parent_it->second;
    }
    std::reverse(path.begin(), path.end());
    return path.size() >= 2 && path.front() == start_node;
  };
  const auto initialRouteDirection = [&](const TopoNode::Ptr &end) -> Eigen::Vector3f {
    std::vector<TopoNode::Ptr> reverse_chain;
    for (auto current = end; current && current != start_node;) {
      reverse_chain.push_back(current);
      const auto parent_it = parent_map.find(current);
      if (parent_it == parent_map.end()) return Eigen::Vector3f::Zero();
      current = parent_it->second;
    }
    if (reverse_chain.empty()) return Eigen::Vector3f::Zero();
    std::reverse(reverse_chain.begin(), reverse_chain.end());
    Eigen::Vector3f direction = Eigen::Vector3f::Zero();
    // Ignore a tiny rolling-odom connector. Use the first point at least two
    // metres along the A* path, or the furthest available point for a short
    // route, so the term represents an actual branch choice.
    for (const auto &node : reverse_chain) {
      if (!node) continue;
      direction = node->center_ - start_node->center_;
      direction.z() = 0.0F;
      if (direction.norm() >= 2.0F) break;
    }
    const float norm = direction.norm();
    if (norm > 1e-3F) return Eigen::Vector3f(direction / norm);
    return Eigen::Vector3f::Zero();
  };
  const auto backtrackMetrics = [&](const TopoNode::Ptr &end) {
    const Eigen::Vector3f initial_direction = initialRouteDirection(end);
    if (continuity_direction.isZero(1e-3F) || initial_direction.isZero(1e-3F))
      return std::make_pair(1.0F, 0.0F);
    const float cosine = std::clamp(
      continuity_direction.dot(initial_direction), -1.0F, 1.0F);
    // direction_penalty_weight is dimensionless. Convert angular change to
    // equivalent metres before the common objective normalization. A reverse
    // branch pays the full cost and a lateral switch pays half; a straight
    // continuation pays nothing.
    const float cost_m = direction_penalty_weight * objective_scale *
      0.5F * (1.0F - cosine);
    return std::make_pair(cosine, cost_m);
  };
  const float start_h = heuristic(start_node);
  g_score[start_node] = 0.0F;
  route_distance[start_node] = 0.0F;
  open.push({start_h, start_h, 0.0F, start_node});

  TopoNode::Ptr best_frontier;
  float best_frontier_objective = std::numeric_limits<float>::infinity();
  float best_frontier_route_distance = -std::numeric_limits<float>::infinity();
  TopoNode::Ptr best_semantic_frontier;
  float best_semantic_frontier_objective = std::numeric_limits<float>::infinity();
  float best_semantic_frontier_route_distance = -std::numeric_limits<float>::infinity();
  int best_semantic_column = std::numeric_limits<int>::max();
  TopoNode::Ptr best_ordinary_frontier;
  float best_ordinary_frontier_objective = std::numeric_limits<float>::infinity();
  float best_ordinary_frontier_route_distance = -std::numeric_limits<float>::infinity();
  struct ReachableSemanticFrontier {
    TopoNode::Ptr node;
    TopoFrontierCandidateStats stats;
  };
  std::vector<ReachableSemanticFrontier> reachable_semantic_frontiers;

  const auto search_start = std::chrono::steady_clock::now();
  while (!open.empty()) {
    const AstarEntry entry = open.top();
    open.pop();
    const auto score_it = g_score.find(entry.node);
    if (score_it == g_score.end() || entry.g > score_it->second + 1e-5F) continue;
    const auto expanded_it = expanded_g.find(entry.node);
    if (expanded_it != expanded_g.end() && entry.g >= expanded_it->second - 1e-5F) continue;
    expanded_g[entry.node] = entry.g;
    if (search_stats) search_stats->expanded_nodes = expanded_g.size();

    const auto distance_it = route_distance.find(entry.node);
    const float route_dist = distance_it != route_distance.end() ? distance_it->second : 0.0F;

    if (prefer_mission_goal && goalDistance(entry.node) <= kGoalNodeToleranceM &&
        isFrontierEndpoint(entry.node, route_dist)) {
      if (search_stats) search_stats->candidate_frontier_goals = 1;
      return backtrackPath(entry.node);
    }

    if (isFrontierEndpoint(entry.node, route_dist)) {
      const float h = goalDistance(entry.node);
      const bool semantic_frontier = isActiveSemanticFrontier(entry.node);
      const float raw_semantic_risk = semantic_frontier ?
        accumulatedSemanticRisk(entry.node) : 0.0F;
      if (semantic_frontier) {
        TopoFrontierCandidateStats candidate;
        candidate.node_id = entry.node->persistent_id_;
        candidate.frame_stamp_ns = entry.node->semantic_frame_stamp_ns_;
        candidate.column = entry.node->semantic_column_;
        candidate.semantic_risk = raw_semantic_risk;
        candidate.astar_cost = entry.g;
        candidate.route_distance = route_dist;
        candidate.mission_distance = h;
        const auto [direction_cosine, backtrack_cost_m] =
          backtrackMetrics(entry.node);
        candidate.direction_cosine = direction_cosine;
        candidate.backtrack_cost_m = backtrack_cost_m;
        reachable_semantic_frontiers.push_back({entry.node, candidate});
        if (search_stats) ++search_stats->semantic_frontier_candidates;
      } else {
        const auto [direction_cosine, backtrack_cost_m] =
          backtrackMetrics(entry.node);
        (void)direction_cosine;
        const float frontier_objective =
          (entry.g + frontier_goal_distance_weight * h + backtrack_cost_m) /
          objective_scale;
        if (search_stats) {
          ++search_stats->ordinary_frontier_candidates;
          if (frontier_objective < search_stats->best_ordinary_frontier_objective) {
            search_stats->best_ordinary_frontier_objective = frontier_objective;
            search_stats->best_ordinary_frontier_id = entry.node->persistent_id_;
          }
        }
        if (best_ordinary_frontier == nullptr ||
            frontier_objective < best_ordinary_frontier_objective - 1e-5F ||
            (std::abs(frontier_objective - best_ordinary_frontier_objective) <= 1e-5F &&
             route_dist > best_ordinary_frontier_route_distance + 1e-3F)) {
          best_ordinary_frontier = entry.node;
          best_ordinary_frontier_objective = frontier_objective;
          best_ordinary_frontier_route_distance = route_dist;
        }
        if (best_frontier == nullptr ||
            frontier_objective < best_frontier_objective - 1e-5F ||
            (std::abs(frontier_objective - best_frontier_objective) <= 1e-5F &&
             route_dist > best_frontier_route_distance + 1e-3F)) {
          best_frontier = entry.node;
          best_frontier_objective = frontier_objective;
          best_frontier_route_distance = route_dist;
        }
      }
    }

    if (time_out > 0.0 && std::chrono::duration<double>(
        std::chrono::steady_clock::now() - search_start).count() > time_out) {
      if (search_stats) search_stats->timed_out = true;
      break;
    }

    // A virtual semantic point extends the measured graph only as a frontier
    // endpoint. Never use its two provisional anchors as an Unknown-space
    // shortcut between otherwise unrelated Verified corridors.
    if (isActiveSemanticFrontier(entry.node)) continue;

    for (const auto &neighbor : entry.node->neighbors_) {
      if (!within_search_radius(neighbor) || !isSearchNode(neighbor)) continue;
      // Do not discard a backward edge solely from its projection on the
      // mission direction.  A valid corridor may require a U-turn around a
      // large obstacle; its A* route cost and mission-distance term already
      // account for that detour.
      const auto weight_it = entry.node->weight_.find(neighbor);
      if (weight_it == entry.node->weight_.end() || !std::isfinite(weight_it->second))
        continue;
      const auto witness_it = entry.node->paths_.find(neighbor);
      if (witness_it == entry.node->paths_.end() || witness_it->second.size() < 2)
        continue;
      // Recheck provisional ordinary-semantic links against the live map. A
      // link can become blocked between semantic insertion and this search;
      // admitting it would select a frontier that publish() immediately
      // rejects, causing repeated INITIAL_ACCEPT replans.
      if (isOrdinarySemanticLink(entry.node, neighbor) && parallel_bubble_astar_) {
        std::vector<Eigen::Vector3f> direct_path{entry.node->center_, neighbor->center_};
        if (!parallel_bubble_astar_->collisionCheck_shortenPath(direct_path)) {
          continue;
        }
      }
      if (search_stats) ++search_stats->edge_evaluations;
      const float edge_cost = routeEdgeCostIndexed(
        entry.node, neighbor, path_cost_weight, semantic_cost_weight,
        isPreviousEdge(entry.node, neighbor), previous_path_cost_factor,
        &semantic_nodes,
        search_stats ? &search_stats->semantic_candidate_checks : nullptr,
        &semantic_index);
      const float tentative_cost = entry.g + edge_cost;
      const auto neighbor_cost_it = g_score.find(neighbor);
      if (neighbor_cost_it == g_score.end() ||
          tentative_cost < neighbor_cost_it->second - 1e-5f) {
        float edge_length = (neighbor->center_ - entry.node->center_).norm();
        if (witness_it->second.size() >= 2) {
          edge_length = 0.0F;
          for (std::size_t i = 1; i < witness_it->second.size(); ++i) {
            const float segment =
              (witness_it->second[i] - witness_it->second[i - 1]).norm();
            if (std::isfinite(segment)) edge_length += segment;
          }
        }
        g_score[neighbor] = tentative_cost;
        route_distance[neighbor] = route_dist + edge_length;
        parent_map[neighbor] = entry.node;
        const float h = heuristic(neighbor);
        open.push({tentative_cost + h, h, tentative_cost, neighbor});
      }
    }
  }

  // The five columns from one heatmap are relative alternatives.  Rank them
  // only after the complete A* expansion has established which columns are
  // reachable, then convert semantic regret to equivalent metres before the
  // same normalization used by route and mission distance.
  std::unordered_map<std::int64_t, std::pair<float, float>> frame_risk_bounds;
  // Use every active virtual direction as the per-frame reference, including
  // directions whose provisional edge is currently unreachable. Reachability
  // decides which endpoint may be selected; it must not change the semantic
  // scale of the remaining directions from one graph update to the next.
  for (const auto &node : frontier_semantic_nodes) {
    if (!isVirtualSemanticEndpoint(node) || node->semantic_frame_stamp_ns_ <= 0)
      continue;
    const float risk = accumulatedSemanticRisk(node);
    auto [it, inserted] = frame_risk_bounds.emplace(
      node->semantic_frame_stamp_ns_, std::make_pair(risk, risk));
    if (!inserted) {
      it->second.first = std::min(it->second.first, risk);
      it->second.second = std::max(it->second.second, risk);
    }
  }
  for (auto &candidate : reachable_semantic_frontiers) {
    const auto bounds_it = frame_risk_bounds.find(candidate.stats.frame_stamp_ns);
    const auto bounds = bounds_it != frame_risk_bounds.end() ? bounds_it->second :
      std::make_pair(candidate.stats.semantic_risk, candidate.stats.semantic_risk);
    candidate.stats.frame_min_risk = bounds.first;
    candidate.stats.frame_risk_range = std::max(0.0F, bounds.second - bounds.first);
    candidate.stats.risk_regret = std::clamp(
      (candidate.stats.semantic_risk - bounds.first) /
        std::max(frontier_semantic_noise_floor, candidate.stats.frame_risk_range),
      0.0F, 1.0F);
    candidate.stats.semantic_cost_m =
      frontier_semantic_frame_budget_m * candidate.stats.frame_min_risk +
      frontier_semantic_detour_budget_m * candidate.stats.risk_regret;
    candidate.stats.objective =
      (candidate.stats.astar_cost +
       frontier_goal_distance_weight * candidate.stats.mission_distance +
       candidate.stats.semantic_cost_m + candidate.stats.backtrack_cost_m) /
      objective_scale;

    const bool objective_tie = std::abs(candidate.stats.objective -
      best_semantic_frontier_objective) <= 1e-5F;
    const bool route_tie = std::abs(candidate.stats.route_distance -
      best_semantic_frontier_route_distance) <= 1e-3F;
    if (best_semantic_frontier == nullptr ||
        candidate.stats.objective < best_semantic_frontier_objective - 1e-5F ||
        (objective_tie &&
         (candidate.stats.route_distance > best_semantic_frontier_route_distance + 1e-3F ||
          (route_tie && static_cast<int>(candidate.stats.column) < best_semantic_column)))) {
      best_semantic_frontier = candidate.node;
      best_semantic_frontier_objective = candidate.stats.objective;
      best_semantic_frontier_route_distance = candidate.stats.route_distance;
      best_semantic_column = static_cast<int>(candidate.stats.column);
      if (search_stats) {
        search_stats->selected_semantic_frame_min_risk = candidate.stats.frame_min_risk;
        search_stats->selected_semantic_frame_risk_range = candidate.stats.frame_risk_range;
        search_stats->selected_semantic_risk_regret = candidate.stats.risk_regret;
        search_stats->selected_semantic_cost_m = candidate.stats.semantic_cost_m;
      }
    }
    if (search_stats) search_stats->semantic_frontier_ranking.push_back(candidate.stats);
  }
  if (search_stats && best_semantic_frontier) {
    search_stats->best_semantic_frontier_objective = best_semantic_frontier_objective;
    search_stats->best_semantic_frontier_id = best_semantic_frontier->persistent_id_;
  }

  // Keep an ordinary fallback for the case where no semantic endpoint is
  // reachable. Normal exploration commits the best semantic frontier after
  // the complete, odom-rooted search.
  if (best_frontier == nullptr) {
    for (const auto &entry : route_distance) {
      if (!isFrontierEndpoint(entry.first, entry.second)) continue;
      const float h = goalDistance(entry.first);
      const auto g_it = g_score.find(entry.first);
      const float route_cost = g_it != g_score.end() ? g_it->second : 0.0F;
      const auto [direction_cosine, backtrack_cost_m] =
        backtrackMetrics(entry.first);
      (void)direction_cosine;
      const float frontier_objective =
        (route_cost + frontier_goal_distance_weight * h + backtrack_cost_m) /
        objective_scale;
      if (best_frontier == nullptr || frontier_objective < best_frontier_objective - 1e-5F ||
          (std::abs(frontier_objective - best_frontier_objective) <= 1e-5F &&
           entry.second > best_frontier_route_distance + 1e-3F)) {
        best_frontier = entry.first;
        best_frontier_objective = frontier_objective;
        best_frontier_route_distance = entry.second;
      }
    }
  }
  // Apply the semantic preference after the complete search, so a semantic
  // frontier is not lost merely because an ordinary node was expanded first.
  if (best_semantic_frontier != nullptr) {
    best_frontier = best_semantic_frontier;
    best_frontier_objective = best_semantic_frontier_objective;
    best_frontier_route_distance = best_semantic_frontier_route_distance;
    if (search_stats) {
      search_stats->selected_semantic_frame_stamp_ns =
        best_frontier->semantic_frame_stamp_ns_;
      search_stats->selected_semantic_column = best_frontier->semantic_column_;
    }
  } else if (best_ordinary_frontier != nullptr) {
    best_frontier = best_ordinary_frontier;
    best_frontier_objective = best_ordinary_frontier_objective;
    best_frontier_route_distance = best_ordinary_frontier_route_distance;
  }
  if (!best_frontier) return false;
  if (search_stats) {
    std::sort(search_stats->semantic_frontier_ranking.begin(),
      search_stats->semantic_frontier_ranking.end(),
      [](const TopoFrontierCandidateStats &left,
         const TopoFrontierCandidateStats &right) {
        if (std::abs(left.objective - right.objective) > 1e-5F)
          return left.objective < right.objective;
        if (left.frame_stamp_ns != right.frame_stamp_ns)
          return left.frame_stamp_ns > right.frame_stamp_ns;
        return left.column < right.column;
      });
  }
  if (search_stats) search_stats->candidate_frontier_goals = 1;
  return backtrackPath(best_frontier);
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
  // Semantic association distance is intentionally much larger than the
  // geometric Bubble displacement. Reusing it here collapses distinct
  // branches whenever two valid vertices are within 2.5 m. EPIC's original
  // diff was voxel-exact; retain only small numerical motion for geometry.
  constexpr float max_match_distance_m = 0.50F;
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
  // Keep this in lockstep with overlap(). Semantic matching must never decide
  // that two geometric branches are the same vertex.
  constexpr float max_match_distance_m = 0.50F;
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
    for (const auto &entry : reg_map_idx2ptr_) {
      if (entry.second) entry.second->topo_nodes_.erase(node);
    }
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
      nbr->edge_clearance_.erase(node);
      nbr->unreachable_nbrs_.erase(node);
    }
    node->unreachable_nbrs_.clear();
    node->neighbors_.clear();
    node->weight_.clear();
    node->paths_.clear();
    node->edge_clearance_.clear();
  }
}

void TopoGraph::updateRemainedConnections(vector<TopoNode::Ptr> &nodes) {
  // Cooldown (values 1..kNeighbourCooldownFrames) applies only to NEW candidate
  // probes.  Existing edges are always re-checked.
  // Doubt streak (values >= kDoubtStreakBase) counts consecutive
  // occupied+repair-miss cycles before removing an established edge.
  constexpr uint8_t kNeighbourCooldownFrames = 5;
  constexpr uint8_t kDoubtStreakBase = 100;
  constexpr uint8_t kMaxDoubtStreak = 3;

  auto doubtStreak = [](const TopoNode::Ptr &node, const TopoNode::Ptr &nbr) -> uint8_t {
    const auto it = node->unreachable_nbrs_.find(nbr);
    if (it == node->unreachable_nbrs_.end() || it->second < kDoubtStreakBase) return 0;
    return static_cast<uint8_t>(it->second - kDoubtStreakBase);
  };

  auto setDoubtStreak = [](TopoNode::Ptr a, TopoNode::Ptr b, uint8_t streak) {
    if (!a || !b || streak == 0) return;
    const uint8_t encoded = static_cast<uint8_t>(kDoubtStreakBase + streak);
    a->unreachable_nbrs_[b] = encoded;
    b->unreachable_nbrs_[a] = encoded;
  };

  auto clearEdgeState = [](TopoNode::Ptr a, TopoNode::Ptr b) {
    if (!a || !b) return;
    a->unreachable_nbrs_.erase(b);
    b->unreachable_nbrs_.erase(a);
  };

  auto setCandidateCooldown = [](TopoNode::Ptr a, TopoNode::Ptr b, uint8_t frames) {
    if (!a || !b || frames == 0) return;
    a->unreachable_nbrs_[b] = frames;
    b->unreachable_nbrs_[a] = frames;
  };

  auto isCandidateCoolingDown = [&](const TopoNode::Ptr &node, const TopoNode::Ptr &nbr) {
    const auto it = node->unreachable_nbrs_.find(nbr);
    return it != node->unreachable_nbrs_.end() && it->second > 0 &&
           it->second <= kNeighbourCooldownFrames;
  };

  // Re-validate an existing neighbour edge.
  auto checkNbr = [&](PtrPair::iter_elem &elem) {
    auto node = elem.p1;
    auto nbr = elem.p2;
    const bool ordinarySemanticEdge = isOrdinarySemanticLink(node, nbr);
    elem.existing = true;
    elem.soft_retry = false;
    elem.doubt_streak = 0;

    const auto path_it = node->paths_.find(nbr);
    if (path_it == node->paths_.end() || path_it->second.size() < 2) {
      elem.insert = false;
      return;
    }
    const vector<Eigen::Vector3f> original_path = path_it->second;
    auto keepExistingWithDoubt = [&]() {
      const uint8_t streak = static_cast<uint8_t>(doubtStreak(node, nbr) + 1);
      if (streak >= kMaxDoubtStreak) {
        elem.insert = false;
        return;
      }
      elem.insert = true;
      elem.path = original_path;
      elem.soft_retry = true;
      elem.doubt_streak = streak;
    };

    if (ordinarySemanticEdge) {
      vector<Eigen::Vector3f> direct_path{node->center_, nbr->center_};
      if (parallel_bubble_astar_->collisionCheck_shortenPath(direct_path)) {
        elem.insert = true;
        elem.path = direct_path;
      } else {
        elem.insert = false;
      }
      return;
    }

    vector<Eigen::Vector3f> path = original_path;
    if (parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
      elem.insert = true;
      elem.path = path;
      return;
    }

    // Witness appears occupied — attempt a short repair search.
    path.clear();
    const int res =
        searchPathWithBoundary(node->center_, nbr->center_, update_connection_timeout, path);
    if (res == ParallelBubbleAstar::REACH_END &&
        parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
      elem.insert = true;
      elem.path = path;
      return;
    }
    if (res == ParallelBubbleAstar::TIME_OUT || res == ParallelBubbleAstar::NO_PATH ||
        res == ParallelBubbleAstar::START_FAIL || res == ParallelBubbleAstar::END_FAIL) {
      vector<Eigen::Vector3f> chord = {node->center_, nbr->center_};
      if (parallel_bubble_astar_->collisionCheck_shortenPath(chord)) {
        elem.insert = true;
        elem.path = chord;
        const uint8_t streak = static_cast<uint8_t>(doubtStreak(node, nbr) + 1);
        if (streak >= kMaxDoubtStreak) {
          elem.insert = false;
          return;
        }
        elem.soft_retry = true;
        elem.doubt_streak = streak;
        return;
      }
      keepExistingWithDoubt();
      return;
    }
    // REACH_END but the replacement also collides: keep the established edge
    // until several consecutive occupied cycles confirm the corridor is gone.
    keepExistingWithDoubt();
  };

  // Probe a not-yet-connected candidate neighbour.
  auto testPreNbr = [&](PtrPair::iter_elem &elem) {
    auto node = elem.p1;
    auto pre_nbr = elem.p2;
    elem.existing = false;
    elem.soft_retry = false;
    elem.doubt_streak = 0;
    // A missing ordinary-semantic link must not be recreated through a curved
    // A* detour.  The displayed/executable link is the direct center segment;
    // only that segment may authorize insertion.
    if (isOrdinarySemanticLink(node, pre_nbr)) {
      vector<Eigen::Vector3f> direct_path{node->center_, pre_nbr->center_};
      if (parallel_bubble_astar_->collisionCheck_shortenPath(direct_path)) {
        elem.insert = true;
        elem.path = direct_path;
      } else {
        elem.insert = false;
      }
      return;
    }
    vector<Eigen::Vector3f> path;
    const int res =
        searchPathWithBoundary(node->center_, pre_nbr->center_, update_connection_timeout, path);
    if (res == ParallelBubbleAstar::REACH_END &&
        parallel_bubble_astar_->collisionCheck_shortenPath(path)) {
      elem.insert = true;
      elem.path = path;
    } else {
      elem.insert = false;
    }
  };

  PtrPair edge2test, edge2check;
  size_t cooldown_skipped = 0;
  for (auto &node : nodes) {
    vector<TopoNode::Ptr> pre_nbrs;
    getPreNbrs(node, pre_nbrs);
    unordered_set<TopoNode::Ptr> pre_nbrs_set(pre_nbrs.begin(), pre_nbrs.end());
    for (auto &nbr : node->neighbors_) {
      if (nbr->is_history_odom_node_)
        continue;
      pre_nbrs_set.insert(nbr);
    }

    // Decay candidate cooldowns; preserve doubt streaks for existing edges.
    unordered_map<TopoNode::Ptr, uint8_t> unreachable_nbrs_tmp;
    for (auto &entry : node->unreachable_nbrs_) {
      if (entry.first == odom_node_) continue;
      if (!pre_nbrs_set.count(entry.first)) continue;
      if (entry.second >= kDoubtStreakBase) {
        unreachable_nbrs_tmp.emplace(entry.first, entry.second);
        continue;
      }
      if (entry.second <= 1) continue;
      unreachable_nbrs_tmp.emplace(entry.first, static_cast<uint8_t>(entry.second - 1));
    }
    node->unreachable_nbrs_.swap(unreachable_nbrs_tmp);

    for (auto &pre_nbr : pre_nbrs_set) {
      if (node->neighbors_.find(pre_nbr) == node->neighbors_.end()) {
        if (isCandidateCoolingDown(node, pre_nbr)) {
          ++cooldown_skipped;
          continue;
        }
        edge2test.insert(node, pre_nbr);
      } else {
        edge2check.insert(node, pre_nbr);
      }
    }
  }
  last_update_timing_.existing_edges_cooldown_skipped += cooldown_skipped;

  edge2test.flatten();
  edge2check.flatten();
  for (auto &elem : edge2check.flatten_data) elem.existing = true;
  for (auto &elem : edge2test.flatten_data) elem.existing = false;

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
  edge2test.flatten_data.insert(edge2test.flatten_data.end(), edge2check.flatten_data.begin(),
                                edge2check.flatten_data.end());

  size_t existing_checked = 0;
  size_t existing_kept = 0;
  size_t existing_repaired = 0;
  size_t existing_removed = 0;
  size_t existing_soft_retry = 0;

  for (auto &elem : edge2test.flatten_data) {
    auto node1 = elem.p1;
    auto node2 = elem.p2;
    if (elem.existing) ++existing_checked;

    if (elem.insert) {
      const bool already_neighbours =
          node1->neighbors_.count(node2) && node2->neighbors_.count(node1);
      const bool repaired_existing =
          elem.existing && already_neighbours && !elem.soft_retry &&
          (node1->paths_.count(node2) == 0 || node1->paths_[node2] != elem.path);

      const float edge_clearance = witnessMinimumClearance(elem.path);
      node1->paths_[node2] = elem.path;
      std::reverse(elem.path.begin(), elem.path.end());
      node2->paths_[node1] = elem.path;
      node1->edge_clearance_[node2] = edge_clearance;
      node2->edge_clearance_[node1] = edge_clearance;
      double cost;
      parallel_bubble_astar_->calculatePathCost(elem.path, cost);
      if (elem.soft_retry) {
        setDoubtStreak(node1, node2, elem.doubt_streak);
        if (elem.existing) ++existing_soft_retry;
      } else {
        clearEdgeState(node1, node2);
        if (elem.existing) {
          if (repaired_existing) ++existing_repaired;
          else ++existing_kept;
        }
      }
      node1->neighbors_.insert(node2);
      node2->neighbors_.insert(node1);
      node1->weight_[node2] = cost;
      node2->weight_[node1] = cost;
    } else {
      if (elem.existing) {
        node1->neighbors_.erase(node2);
        node2->neighbors_.erase(node1);
        node1->weight_.erase(node2);
        node2->weight_.erase(node1);
        node1->paths_.erase(node2);
        node2->paths_.erase(node1);
        node1->edge_clearance_.erase(node2);
        node2->edge_clearance_.erase(node1);
        ++existing_removed;
      }
      clearEdgeState(node1, node2);
      if (!elem.existing) {
        setCandidateCooldown(node1, node2, kNeighbourCooldownFrames);
      }
    }
  }

  last_update_timing_.existing_edges_checked += existing_checked;
  last_update_timing_.existing_edges_kept += existing_kept;
  last_update_timing_.existing_edges_repaired += existing_repaired;
  last_update_timing_.existing_edges_removed += existing_removed;
  last_update_timing_.existing_edges_soft_retry += existing_soft_retry;
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

  // Include diagonal neighboring regions as well as axis-aligned regions.
  // Bubble centers near a region corner can be connected through a clear
  // diagonal corridor; restricting this to steps1 leaves two valid bubbles
  // present but permanently disconnected.
  for (int i = 0; i < static_cast<int>(steps1.size() + steps2.size()); ++i) {
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
  vector<int> connection_results(pair_vector.size(), ParallelBubbleAstar::NO_PATH);

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
    connection_results[i] = res;
    if (res != ParallelBubbleAstar::REACH_END)
      path.push_back(Eigen::Vector3f::Zero());
    else if (!only_raycast) {
      bool safe = parallel_bubble_astar_->collisionCheck_shortenPath(path);
      if (safe)
        path.push_back(Eigen::Vector3f::Ones());
      else {
        connection_results[i] = -1;  // reached end, but shortening rejected it
        path.push_back(Eigen::Vector3f::Zero());
      }
    } else {
      path.push_back(Eigen::Vector3f::Ones()); // 1表示安全，0表示危险
    }
    path_vec[i].swap(path);
  }

  last_update_timing_.insert_candidate_edges += pair_vector.size();
  for (size_t i = 0; i < connection_results.size(); ++i) {
    const int result = connection_results[i];
    if (result == ParallelBubbleAstar::REACH_END) {
      ++last_update_timing_.insert_success_edges;
    } else if (result == -1) {
      ++last_update_timing_.insert_collision_reject_edges;
    } else if (result == ParallelBubbleAstar::TIME_OUT) {
      ++last_update_timing_.insert_timeout_edges;
    } else if (result == ParallelBubbleAstar::START_FAIL) {
      ++last_update_timing_.insert_start_fail_edges;
    } else if (result == ParallelBubbleAstar::END_FAIL) {
      ++last_update_timing_.insert_end_fail_edges;
    } else {
      ++last_update_timing_.insert_no_path_edges;
    }
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
    const float edge_clearance = witnessMinimumClearance(path_vec[i]);
    node1->paths_[node2] = path_vec[i];
    std::reverse(path_vec[i].begin(), path_vec[i].end());
    node2->paths_[node1] = path_vec[i];
    node1->edge_clearance_[node2] = edge_clearance;
    node2->edge_clearance_[node1] = edge_clearance;
    double cost;
    parallel_bubble_astar_->calculatePathCost(path_vec[i], cost);
    node1->weight_[node2] = cost;
    node2->weight_[node1] = cost;
  }

}

size_t TopoGraph::insertSemanticNodes(
    const vector<Eigen::Vector3f> &centers, const vector<float> &semantic_scores,
    float bubble_radius, const Eigen::Vector3f &odom_pos,
    std::int64_t stamp_ns, const vector<float> &semantic_confidences,
    const vector<std::uint8_t> &semantic_virtual_flags,
    const vector<std::int8_t> &semantic_columns) {
  if (centers.empty() || !lidar_map_interface_ || !parallel_bubble_astar_) return 0;
  const float min_separation = std::max(
    std::max(0.75F, bubble_radius),
    static_cast<float>(std::max(0.0, semantic_node_match_distance_)));
  if (odom_node_) {
    odom_node_->center_ = projectGraphPoint(odom_pos, planar_graph_, planar_z_);
  }
  vector<TopoNode::Ptr> created;
  created.reserve(centers.size());
  size_t updated_semantic = 0;
  size_t influenced_existing = 0;
  for (size_t center_index = 0; center_index < centers.size(); ++center_index) {
    const Eigen::Vector3f center = projectGraphPoint(
      centers[center_index], planar_graph_, planar_z_);
    // Semantic virtual anchors may lie in as-yet unobserved space outside
    // the current occupied boxes. Keep them within the configured global map;
    // the scale-manager stage still box-gates measured surface projections.
    if (!center.allFinite() || !lidar_map_interface_->IsInMap(center)) continue;
    const float score = std::clamp(
      center_index < semantic_scores.size() ? semantic_scores[center_index] : 1.0F,
      0.0F, 1.0F);
    const float confidence = std::clamp(
      center_index < semantic_confidences.size() ?
        semantic_confidences[center_index] : 1.0F,
      0.0F, 1.0F);
    // The legacy overload predates the measured/virtual split and represented
    // fixed-depth semantic frontiers. Preserve that API meaning when callers
    // omit the source vector; the ScaleNav callback always supplies explicit
    // flags for the two projection products.
    const bool is_virtual = semantic_virtual_flags.empty() ||
      (center_index < semantic_virtual_flags.size() &&
       semantic_virtual_flags[center_index] != 0U);
    TopoNode::Ptr match;
    float match_distance = min_separation;
    for (const auto &entry : reg_map_idx2ptr_) {
      if (!entry.second) continue;
      for (const auto &node : entry.second->topo_nodes_) {
        if (!node || node->is_viewpoint_ || node->role_ == TopoNodeRole::Odom) continue;
        const float distance = (node->center_ - center).norm();
        if (distance >= match_distance) continue;
        // A fixed-depth virtual anchor must remain an Unknown semantic
        // frontier even when it overlaps a measured backbone node. Measured
        // projections may still annotate an existing Verified node.
        if (is_virtual ? isVirtualSemanticEndpoint(node) :
            (node->geometry_state_ == TopoGeometryState::Verified &&
             !node->is_virtual_semantic_)) {
          match = node;
          match_distance = distance;
        }
      }
    }
    if (match) {
      match->is_virtual_semantic_ = is_virtual;
      updateNodeSemantic(match, score, 1.0F, stamp_ns, confidence);
      match->semantic_frame_stamp_ns_ = stamp_ns;
      match->semantic_column_ = center_index < semantic_columns.size() ?
        semantic_columns[center_index] : static_cast<std::int8_t>(-1);
      {
        std::lock_guard<std::mutex> lock(semantic_memory_mutex_);
        semantic_memory_[match->persistent_id_].frame_stamp_ns =
          match->semantic_frame_stamp_ns_;
        semantic_memory_[match->persistent_id_].column = match->semantic_column_;
        semantic_memory_[match->persistent_id_].is_virtual =
          match->is_virtual_semantic_;
      }
      if (match->geometry_state_ == TopoGeometryState::Unknown) {
        ++updated_semantic;
      } else {
        ++influenced_existing;
      }
      continue;
    }
    // Surface projections only annotate measured backbone geometry. Creating
    // an Unknown node here would make a depth return indistinguishable from a
    // fixed-depth frontier and was the cause of incorrect initial goals.
    if (!is_virtual) continue;
    auto node = std::make_shared<TopoNode>();
    node->center_ = center;
    node->bubble_radius_ = std::max(0.45F, bubble_radius);
    node->role_ = TopoNodeRole::Geometric;
    node->geometry_state_ = TopoGeometryState::Unknown;
    node->is_virtual_semantic_ = true;
    updateNodeSemantic(node, score, 1.0F, stamp_ns, confidence);
    node->semantic_frame_stamp_ns_ = stamp_ns;
    node->semantic_column_ = center_index < semantic_columns.size() ?
      semantic_columns[center_index] : static_cast<std::int8_t>(-1);
    {
      std::lock_guard<std::mutex> lock(semantic_memory_mutex_);
      semantic_memory_[node->persistent_id_].frame_stamp_ns =
        node->semantic_frame_stamp_ns_;
      semantic_memory_[node->persistent_id_].column = node->semantic_column_;
      semantic_memory_[node->persistent_id_].is_virtual = true;
    }
    created.emplace_back(std::move(node));
  }
  // A current semantic endpoint is provisional geometry: attach it to the
  // nearest measured graph backbone so the next normal frontier search can
  // reach the configured virtual depth. Unknown-to-Unknown links are forbidden
  // because they form isolated semantic-only components. When a measured
  // Bubble reaches this position updateSkeleton() promotes the semantic state
  // onto that Bubble and removes these provisional incident edges.
  const auto isBackboneAnchor = [](const TopoNode::Ptr &node) {
    return node && (node->role_ == TopoNodeRole::Odom ||
                    node->geometry_state_ == TopoGeometryState::Verified);
  };
  vector<TopoNode::Ptr> nodes_to_connect = created;
  for (const auto &entry : reg_map_idx2ptr_) {
    if (!entry.second) continue;
    for (const auto &node : entry.second->topo_nodes_) {
      if (!isVirtualSemanticEndpoint(node) ||
          !semanticNodeActiveForPlanning(*node, stamp_ns)) continue;
      const size_t backbone_degree = std::count_if(
        node->neighbors_.begin(), node->neighbors_.end(), isBackboneAnchor);
      if (backbone_degree < static_cast<size_t>(
            std::max(1, semantic_point_max_connections_)) &&
          std::find(nodes_to_connect.begin(), nodes_to_connect.end(), node) ==
            nodes_to_connect.end()) {
        nodes_to_connect.push_back(node);
      }
    }
  }

  size_t accepted = 0;
  for (auto &node : nodes_to_connect) {
    if (node->semantic_edges_attempted_stamp_ns_ == node->semantic_stamp_ns_ &&
        node->semantic_stamp_ns_ > 0) {
      continue;
    }
    vector<TopoNode::Ptr> anchors;
    for (const auto &entry : reg_map_idx2ptr_) {
      if (!entry.second) continue;
      for (const auto &candidate : entry.second->topo_nodes_) {
        if (!candidate || candidate == node || candidate->is_viewpoint_ ||
            candidate->geometry_state_ != TopoGeometryState::Verified ||
            node->neighbors_.count(candidate)) continue;
        anchors.push_back(candidate);
      }
    }
    // Prefer the measured frontier. A long odom-to-semantic chord can tie the
    // measured route in distance but fail the final Bubble-overlap witness
    // check. Odom is only the bootstrap fallback before any Verified anchor is
    // available.
    if (anchors.empty() && odom_node_ && !node->neighbors_.count(odom_node_)) {
      anchors.push_back(odom_node_);
    }
    std::sort(anchors.begin(), anchors.end(),
      [&node](const TopoNode::Ptr &left, const TopoNode::Ptr &right) {
        return (left->center_ - node->center_).squaredNorm() <
               (right->center_ - node->center_).squaredNorm();
      });
    anchors.erase(std::unique(anchors.begin(), anchors.end()), anchors.end());

    vector<TopoNode::Ptr> neighbors;
    vector<vector<Eigen::Vector3f>> paths;
    const size_t existing_backbone_degree = std::count_if(
      node->neighbors_.begin(), node->neighbors_.end(), isBackboneAnchor);
    const size_t desired_connections = static_cast<size_t>(
      std::max(1, semantic_point_max_connections_));
    const size_t missing_connections = existing_backbone_degree < desired_connections ?
      desired_connections - existing_backbone_degree : 0U;
    const size_t candidate_limit = std::min<size_t>(
      anchors.size(), static_cast<size_t>(
        std::max(1, semantic_point_connection_candidates_)));
    for (size_t i = 0; i < candidate_limit && neighbors.size() < missing_connections; ++i) {
      const auto &anchor = anchors[i];
      if (!anchor || (anchor->center_ - node->center_).norm() <= 1e-3F) continue;
      // Ordinary-semantic links are executable contracts.  Test the direct
      // center-line while selecting anchors so blocked nearest anchors do not
      // consume the connection budget and leave the semantic frontier
      // unreachable even when a later anchor is clear.
      std::vector<Eigen::Vector3f> direct_path{node->center_, anchor->center_};
      if (!parallel_bubble_astar_->collisionCheck_shortenPath(direct_path)) {
        continue;
      }
      neighbors.push_back(anchor);
      paths.push_back(std::move(direct_path));
    }
    insertNode(node, neighbors, paths);
    if (std::find(created.begin(), created.end(), node) != created.end()) ++accepted;
  }

  ROS_INFO("[ScaleNav semantic source] stamp=%lld input=%zu created_virtual=%zu "
           "updated=%zu measured_annotations=%zu",
           static_cast<long long>(stamp_ns), centers.size(), created.size(),
           updated_semantic, influenced_existing);

  revalidateSemanticEdges();
  return accepted + influenced_existing + updated_semantic;
}

size_t TopoGraph::revalidateSemanticEdges() {
  if (!parallel_bubble_astar_) return 0;
  std::vector<std::pair<TopoNode::Ptr, TopoNode::Ptr>> to_remove;
  std::unordered_set<std::pair<TopoNode::Ptr, TopoNode::Ptr>, PairPtrHash> checked;
  for (const auto &entry : reg_map_idx2ptr_) {
    if (!entry.second) continue;
    for (const auto &semantic : entry.second->topo_nodes_) {
      if (!isVirtualSemanticEndpoint(semantic)) continue;
      for (const auto &ordinary : semantic->neighbors_) {
        // Do not apply this check to semantic-semantic links.  A promoted
        // semantic node is Verified, so exclude semantic endpoints from the
        // ordinary side explicitly.
        if (!isOrdinaryBackboneEndpoint(ordinary)) continue;
        const auto pair = std::make_pair(semantic, ordinary);
        const auto reverse = std::make_pair(ordinary, semantic);
        if (checked.count(pair) || checked.count(reverse)) continue;
        checked.insert(pair);
        const auto path_it = semantic->paths_.find(ordinary);
        bool safe = true;
        std::vector<Eigen::Vector3f> direct_path{semantic->center_, ordinary->center_};
        ParallelBubbleAstar::CollisionCheckInfo info;
        safe = parallel_bubble_astar_->collisionCheck_shortenPath(direct_path, &info);
        const std::size_t witness_count =
          path_it == semantic->paths_.end() ? 0U : path_it->second.size();
        if (!safe) {
          semantic->semantic_edges_attempted_stamp_ns_ = semantic->semantic_stamp_ns_;
          to_remove.emplace_back(semantic, ordinary);
          ROS_WARN("[ScaleNav semantic edge] removing blocked edge semantic=%llu "
                   "ordinary=%llu s=(%.2f,%.2f,%.2f) o=(%.2f,%.2f,%.2f) "
                   "reason=%d clearance=%.3f witness=%zu",
                   static_cast<unsigned long long>(semantic->persistent_id_),
                   static_cast<unsigned long long>(ordinary->persistent_id_),
                   semantic->center_.x(), semantic->center_.y(), semantic->center_.z(),
                   ordinary->center_.x(), ordinary->center_.y(), ordinary->center_.z(),
                   static_cast<int>(info.reason), info.clearance, witness_count);
        }
      }
    }
  }
  size_t removed = 0;
  for (const auto &edge : to_remove) {
    if (removeEdge(edge.first, edge.second)) ++removed;
  }
  return removed;
}

void TopoGraph::getRegionsToUpdate() {
  // 清空上一帧的区域选择结果，避免旧区域混入本次更新。
  update_idx_vec_.clear();
  viewpoints_update_region_arr_.clear();
  toponodes_update_region_arr_.clear();

  // 分别收集所有候选区域、占用区域和自由空间区域。
  unordered_set<RegionNode::Ptr> region_set;
  unordered_set<RegionNode::Ptr> occupied_region_set;
  unordered_set<RegionNode::Ptr> free_region_set;

  // 将三维传感器坐标投影到当前拓扑图使用的平面或三维空间。
  auto graphPoint = [this](const Eigen::Vector3f &point) {
    return projectGraphPoint(point, planar_graph_, planar_z_);
  };

  // 读取当前雷达位姿、最新点云和传感器最大量程。
  const Eigen::Vector3f lidar_pose = lidar_map_interface_->poseSnapshot();
  const auto latest_cloud = lidar_map_interface_->latestCloudSnapshot();
  const float max_ray_length = static_cast<float>(lidar_map_interface_->lp_->max_ray_length_);
  const Eigen::Vector3f graph_pose = graphPoint(lidar_pose);

  // 用占用点所在的区域作为第一批待更新区域，并截断超量程点。
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

  // 保存占用区域数量，供更新统计和后续日志使用。
  selected_occupied_regions_ = occupied_region_set.size();
  // Free-ray evidence is part of EPIC's topology input, not visualization
  // metadata. It is what lets Bubble generation populate clear side
  // corridors where the depth frame has no occupied return. Keep it in the
  // same bounded region set as occupied returns.
  const auto free_cloud = lidar_map_interface_->freeSpaceSnapshot();

  // 把自由射线终点也加入候选区域，补足没有障碍回波的可通行空间。
  for (const auto &point : free_cloud.points) {
    // A camera ray endpoint is generally far above/below the fixed graph
    // layer (especially at 20 m).  Its x/y footprint is still valid free
    // evidence for the planar graph, so project it exactly like an occupied
    // return instead of rejecting it by endpoint height.
    const Eigen::Vector3f free_point = graphPoint(Eigen::Vector3f(
      point.x, point.y, point.z));
    Eigen::Vector3i region_idx;
    getIndex(free_point, region_idx);
    const auto region = getRegionNode(region_idx);
    if (region != nullptr) {
      region_set.insert(region);
      free_region_set.insert(region);
    }
  }

  // 保存自由区域数量，供更新统计和后续日志使用。
  selected_free_regions_ = free_region_set.size();
  // 将无序集合转换为可排序的待更新区域数组。
  for (auto &region : region_set) {
    toponodes_update_region_arr_.push_back(region);
  }
  auto shorten_by_distance_insert_update_arr = [&](vector<RegionNode::Ptr> &arr) {
    unordered_set<RegionNode::Ptr> region2update(arr.begin(), arr.end());
    arr = vector<RegionNode::Ptr>(region2update.begin(), region2update.end());
    if (max_update_region_num_ <= 0) return;
    const Eigen::Vector3f pose = graphPoint(lidar_pose);
    Eigen::Vector3f to_goal = Eigen::Vector3f::Zero();
    float goal_len = 0.0F;
    if (has_update_goal_) {
      to_goal = graphPoint(update_goal_) - pose;
      goal_len = to_goal.norm();
      if (goal_len > 1e-3F) to_goal /= goal_len;
    }
    auto regionScore = [&](const RegionNode::Ptr &region) {
      Eigen::Vector3f lb, hb;
      index2boundary(region->region_idx_, lb, hb);
      const Eigen::Vector3f rel = graphPoint(0.5F * (hb + lb)) - pose;
      const float dist = rel.norm();
      if (!has_update_goal_ || goal_len <= 1e-3F) return dist;
      const float along = rel.dot(to_goal);
      const float lateral = (rel - along * to_goal).norm();
      return 0.35F * dist + 0.65F * lateral - 0.25F * std::max(0.0F, along);
    };
    std::sort(arr.begin(), arr.end(),
      [&](const RegionNode::Ptr &region1, const RegionNode::Ptr &region2) {
        const float s1 = regionScore(region1);
        const float s2 = regionScore(region2);
        if (s1 != s2) return s1 < s2;
        return region1 < region2;
    });
    arr.resize(std::min((int)arr.size(), max_update_region_num_));
  };

  // 沿当前位姿到各候选区域中心的方向扩展邻近区域。
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

  // 合并射线扩展得到的区域，并按距离和目标方向限制数量。
  for (auto &region : region_set) {
    toponodes_update_region_arr_.push_back(region);
  }

  shorten_by_distance_insert_update_arr(toponodes_update_region_arr_);

  // 输出区域索引，供 updateSkeleton() 生成和匹配 Bubble 节点。
  for (auto &region : toponodes_update_region_arr_) {
    update_idx_vec_.push_back(region->region_idx_);
  }
}

void TopoGraph::updateSkeleton() {
  using Clock = std::chrono::steady_clock;
  const auto total_start = Clock::now();
  last_update_timing_ = TopoGraphUpdateTiming{};
  // A region rebuild can produce the same Bubble center from two adjacent
  // regions.  Clean the persistent graph before matching the new skeleton;
  // otherwise each copy keeps a different subset of the incident edges and
  // isolated degree-zero vertices accumulate over time.
  last_update_timing_.duplicate_nodes_merged = deduplicateNearbyNodes();
  last_update_timing_.regions = toponodes_update_region_arr_.size();
  last_update_timing_.occupied_regions = selected_occupied_regions_;
  last_update_timing_.free_regions = selected_free_regions_;
  parallel_bubble_astar_->reset();
  vector<TopoNode::Ptr> nodes2insert, nodes_remained, nodes2remove, new_nodes, old_nodes;
  mutex new_nodes_mtx;
  const auto prepare_start = Clock::now();
  for (auto &region : toponodes_update_region_arr_) {
    for (auto &node : region->topo_nodes_) {
      if (!node->is_viewpoint_ &&
          node->geometry_state_ == TopoGeometryState::Verified)
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

  // Bubble clustering is local to a region, so adjacent regions can still
  // emit duplicate centers in the same rebuild.  Collapse those candidates
  // before overlap/setdiff; they must not enter insertNodes as separate
  // vertices.
  // Only collapse numerical jitter from adjacent-region clustering.  The
  // 0.25 m map voxel is a valid topology spacing and must not be treated as a
  // duplicate distance.
  const float new_node_merge_distance = 0.05F;
  vector<TopoNode::Ptr> unique_new_nodes;
  unique_new_nodes.reserve(new_nodes.size());
  for (const auto &candidate : new_nodes) {
    if (!candidate) continue;
    TopoNode::Ptr match;
    for (const auto &existing : unique_new_nodes) {
      if ((candidate->center_ - existing->center_).norm() <= new_node_merge_distance) {
        match = existing;
        break;
      }
    }
    if (!match) {
      unique_new_nodes.push_back(candidate);
      continue;
    }
    // Keep the more open representative.  Semantic state is restored by
    // persistent id after the geometric diff, so this merge is lossless for
    // newly generated candidates.
    if (candidate->bubble_radius_ > match->bubble_radius_) {
      match->center_ = candidate->center_;
      match->bubble_radius_ = candidate->bubble_radius_;
    }
  }
  new_nodes.swap(unique_new_nodes);

  // A newly observed Bubble is authoritative geometry. If it lands on a
  // semantic point, promote the semantic node instead of keeping two
  // vertices at the same location. This preserves semantic identity while
  // changing only the geometry state.
  // Keep the existing semantic pointer when promoting it.  Accepted routes
  // retain TopoNode::Ptr values; replacing a virtual node with a newly
  // allocated Bubble leaves those route pointers detached even when the
  // persistent id is unchanged.  That false topology change was the source
  // of repeated fresh A* searches and left/right oscillation.
  for (size_t measured_index = 0; measured_index < new_nodes.size();
       ++measured_index) {
    const auto &measured = new_nodes[measured_index];
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
        if (!candidate || candidate->geometry_state_ != TopoGeometryState::Unknown ||
            candidate->semantic_observations_ == 0) continue;
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
    const auto persistent_id = match->persistent_id_;
    const auto semantic_score = match->semantic_score_;
    const auto semantic_confidence = match->semantic_confidence_;
    const auto semantic_risk = match->semantic_risk_;
    const auto semantic_risk_stamp = match->semantic_risk_stamp_ns_;
    const auto semantic_observations = match->semantic_observations_;
    const auto semantic_stamp = match->semantic_stamp_ns_;
    const auto semantic_frame_stamp = match->semantic_frame_stamp_ns_;
    const auto semantic_column = match->semantic_column_;

    // Remove provisional semantic edges, but keep the object alive.  The
    // normal insertNodes() pass below will rebuild verified backbone edges on
    // this same pointer.
    removeNode(match);
    match->center_ = measured->center_;
    match->bubble_radius_ = measured->bubble_radius_;
    match->is_viewpoint_ = measured->is_viewpoint_;
    match->role_ = TopoNodeRole::Geometric;
    match->geometry_state_ = TopoGeometryState::Verified;
    match->persistent_id_ = persistent_id;
    match->semantic_score_ = semantic_score;
    match->semantic_confidence_ = semantic_confidence;
    match->semantic_risk_ = semantic_risk;
    match->semantic_risk_stamp_ns_ = semantic_risk_stamp;
    match->semantic_observations_ = semantic_observations;
    match->semantic_stamp_ns_ = semantic_stamp;
    match->is_virtual_semantic_ = false;
    match->semantic_frame_stamp_ns_ = semantic_frame_stamp;
    match->semantic_column_ = semantic_column;
    match->geometry_miss_count_ = 0;
    match->semantic_edges_attempted_stamp_ns_ = 0;
    {
      std::lock_guard<std::mutex> lock(semantic_memory_mutex_);
      const auto memory = semantic_memory_.find(persistent_id);
      if (memory != semantic_memory_.end()) {
        memory->second.center = match->center_;
        memory->second.is_virtual = false;
      }
    }
    new_nodes[measured_index] = match;
  }

  const auto diff_start = Clock::now();
  overlap(new_nodes, old_nodes, nodes_remained);
  setdiff(old_nodes, new_nodes, nodes2remove);

  // A missing Bubble is not proof that the corresponding free space vanished:
  // the current map snapshot may still be ray-carving that region, or an
  // insertion search may have failed transiently. Keep the old vertex and its
  // identity for two consecutive misses, while refreshing its incident edges
  // through the normal remained-node path. Only a persistent miss is removed.
  constexpr std::uint8_t kGeometryMissGrace = 2;
  for (const auto &node : nodes_remained) {
    if (node) node->geometry_miss_count_ = 0;
  }
  vector<TopoNode::Ptr> deferred_removals;
  deferred_removals.reserve(nodes2remove.size());
  vector<TopoNode::Ptr> confirmed_removals;
  confirmed_removals.reserve(nodes2remove.size());
  for (const auto &node : nodes2remove) {
    if (!node) continue;
    if (node->is_route_anchor_) {
      node->geometry_miss_count_ = 0;
      deferred_removals.push_back(node);
      continue;
    }
    node->geometry_miss_count_ = static_cast<std::uint8_t>(
      std::min<int>(255, static_cast<int>(node->geometry_miss_count_) + 1));
    if (retainGeometryAfterMiss(node->geometry_miss_count_, kGeometryMissGrace)) {
      deferred_removals.push_back(node);
    } else {
      confirmed_removals.push_back(node);
    }
  }
  nodes2remove.swap(confirmed_removals);
  last_update_timing_.deferred_nodes = deferred_removals.size();
  nodes_remained.insert(nodes_remained.end(), deferred_removals.begin(),
                        deferred_removals.end());
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
  // Keep an isolated Bubble in the persistent graph.  EPIC's bounded A* can
  // reject all candidate edges for one frame because the map snapshot is
  // still being updated; deleting the vertex here loses the branch before
  // the next frame can retry it.  The existing isolated-node retry in
  // insertNodes() and the next incremental update are the recovery path.
  last_update_timing_.total_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - total_start).count();
  // New vertices can expose stale half-edges from an earlier incremental
  // diff, and duplicate candidates only become visible after insertion.
  last_update_timing_.duplicate_nodes_merged += deduplicateNearbyNodes();
  last_update_timing_.half_edges_removed = normalizeConnectivity();
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
  odom_node_->center_ = odom_pos;
  odom_node_->yaw_ = yaw;
  for (auto &nei : odom_node_->neighbors_) {
    nei->neighbors_.erase(odom_node_);
    nei->weight_.erase(odom_node_);
    nei->paths_.erase(odom_node_);
    nei->edge_clearance_.erase(odom_node_);
    nei->unreachable_nbrs_.erase(odom_node_);
  }
  odom_node_->neighbors_.clear();
  odom_node_->weight_.clear();
  odom_node_->paths_.clear();
  odom_node_->edge_clearance_.clear();
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
    // A witness generated from the previous odom pose cannot be attached to
    // this new center. Doing so creates a discontinuous path whose topology
    // starts at the vehicle while its first witness starts metres behind. The
    // ROS adapter immediately performs its wider fallback connection search;
    // if that also fails, publishing no route is safer than a stale route.
    return;
  }
  for (auto &edge : edge2insert) {
    odom_node_->neighbors_.insert(edge.first.second);
    odom_node_->paths_.insert({edge.first.second, edge.second});
    const float edge_clearance = witnessMinimumClearance(edge.second);
    odom_node_->edge_clearance_[edge.first.second] = edge_clearance;
    double cost = 0.0;
    // Odom links are real A* edges.  A zero weight made every nearby bubble
    // equally cheap, so a stale bubble behind the vehicle could become the
    // first route node solely because its eventual frontier looked better.
    parallel_bubble_astar_->calculatePathCost(edge.second, cost);
    odom_node_->weight_[edge.first.second] = static_cast<float>(cost);
    edge.first.second->neighbors_.insert(odom_node_);
    auto reverse_path = edge.second;
    std::reverse(reverse_path.begin(), reverse_path.end());
    edge.first.second->paths_[odom_node_] = reverse_path;
    edge.first.second->edge_clearance_[odom_node_] = edge_clearance;
    edge.first.second->weight_[odom_node_] = static_cast<float>(cost);
    // auto nbr = edge.first.second;
    // nbr->neighbors_.insert(odom_node_);
    // nbr->weight_[odom_node_] = cost;
    // vector<Eigen::Vector3f> path = edge.second;
    // std::reverse(path.begin(), path.end());
    // nbr->paths_[odom_node_] = path;
  }
  // }
}

size_t TopoGraph::deduplicateNearbyNodes(float tolerance_m) {
  if (!(tolerance_m > 0.0F) || !std::isfinite(tolerance_m)) return 0;

  vector<TopoNode::Ptr> nodes;
  unordered_set<TopoNode::Ptr> seen;
  auto collect_node = [&](const TopoNode::Ptr &node) {
    if (!node || node->is_viewpoint_ || node->role_ != TopoNodeRole::Geometric ||
        !seen.insert(node).second) return;
    nodes.push_back(node);
  };
  for (const auto &entry : reg_map_idx2ptr_) {
    if (!entry.second) continue;
    for (const auto &node : entry.second->topo_nodes_) {
      collect_node(node);
    }
  }
  // A stale edge can retain a node after its region membership was removed.
  // Include that reachable component in the same pass; otherwise the node is
  // visible in diagnostics but can never be merged by the region-only scan.
  for (size_t i = 0; i < nodes.size(); ++i) {
    for (const auto &neighbor : nodes[i]->neighbors_) collect_node(neighbor);
  }

  auto preferred = [](const TopoNode::Ptr &left, const TopoNode::Ptr &right) {
    if (left->is_route_anchor_ != right->is_route_anchor_)
      return left->is_route_anchor_;
    const bool left_verified = left->geometry_state_ == TopoGeometryState::Verified;
    const bool right_verified = right->geometry_state_ == TopoGeometryState::Verified;
    if (left_verified != right_verified) return left_verified;
    const bool left_persistent = left->persistent_id_ != 0;
    const bool right_persistent = right->persistent_id_ != 0;
    if (left_persistent != right_persistent) return left_persistent;
    if (left->neighbors_.size() != right->neighbors_.size())
      return left->neighbors_.size() > right->neighbors_.size();
    return left->persistent_id_ < right->persistent_id_;
  };

  const float tolerance_sq = tolerance_m * tolerance_m;
  vector<bool> consumed(nodes.size(), false);
  size_t merged = 0;
  for (size_t i = 0; i < nodes.size(); ++i) {
    if (consumed[i] || !nodes[i]) continue;
    vector<size_t> members;
    members.push_back(i);
    consumed[i] = true;
    for (size_t j = i + 1; j < nodes.size(); ++j) {
      if (consumed[j] || !nodes[j]) continue;
      if ((nodes[i]->center_ - nodes[j]->center_).squaredNorm() <= tolerance_sq) {
        members.push_back(j);
        consumed[j] = true;
      }
    }
    if (members.size() < 2) continue;

    size_t canonical_index = members.front();
    for (const auto index : members) {
      if (preferred(nodes[index], nodes[canonical_index])) canonical_index = index;
    }
    const auto canonical = nodes[canonical_index];
    unordered_set<TopoNode::Ptr> duplicate_set;
    for (const auto index : members) duplicate_set.insert(nodes[index]);

    // Preserve semantic memory and the largest free-space representative.
    for (const auto index : members) {
      const auto duplicate = nodes[index];
      if (duplicate == canonical) continue;
      canonical->is_route_anchor_ =
        canonical->is_route_anchor_ || duplicate->is_route_anchor_;
      canonical->bubble_radius_ = std::max(canonical->bubble_radius_, duplicate->bubble_radius_);
      if (duplicate->semantic_observations_ > canonical->semantic_observations_ ||
          (duplicate->semantic_observations_ == canonical->semantic_observations_ &&
           duplicate->semantic_score_ > canonical->semantic_score_)) {
        canonical->semantic_score_ = duplicate->semantic_score_;
        canonical->semantic_confidence_ = duplicate->semantic_confidence_;
        canonical->semantic_risk_ = duplicate->semantic_risk_;
        canonical->semantic_risk_stamp_ns_ = duplicate->semantic_risk_stamp_ns_;
        canonical->semantic_stamp_ns_ = duplicate->semantic_stamp_ns_;
        canonical->semantic_frame_stamp_ns_ = duplicate->semantic_frame_stamp_ns_;
        canonical->semantic_column_ = duplicate->semantic_column_;
        canonical->is_virtual_semantic_ = duplicate->is_virtual_semantic_;
      }
      canonical->semantic_risk_ = std::max(
        canonical->semantic_risk_, duplicate->semantic_risk_);
      canonical->semantic_observations_ = std::max(
        canonical->semantic_observations_, duplicate->semantic_observations_);
      if (canonical->geometry_state_ == TopoGeometryState::Verified) {
        canonical->is_virtual_semantic_ = false;
      }
    }

    // Redirect every incident edge to the canonical vertex.  The old edge is
    // removed from both endpoint maps first, so no one-way neighbor survives.
    for (const auto index : members) {
      const auto duplicate = nodes[index];
      if (duplicate == canonical) continue;
      const auto neighbors = duplicate->neighbors_;
      for (const auto &neighbor : neighbors) {
        if (!neighbor || duplicate_set.count(neighbor)) continue;
        const auto path_it = duplicate->paths_.find(neighbor);
        const auto weight_it = duplicate->weight_.find(neighbor);
        const auto clearance_it = duplicate->edge_clearance_.find(neighbor);
        neighbor->neighbors_.erase(duplicate);
        neighbor->paths_.erase(duplicate);
        neighbor->weight_.erase(duplicate);
        neighbor->edge_clearance_.erase(duplicate);
        canonical->neighbors_.insert(neighbor);
        neighbor->neighbors_.insert(canonical);
        if (canonical->paths_.find(neighbor) == canonical->paths_.end() &&
            path_it != duplicate->paths_.end()) {
          canonical->paths_[neighbor] = path_it->second;
          auto reverse_path = path_it->second;
          std::reverse(reverse_path.begin(), reverse_path.end());
          neighbor->paths_[canonical] = reverse_path;
        }
        if (canonical->edge_clearance_.find(neighbor) == canonical->edge_clearance_.end() &&
            clearance_it != duplicate->edge_clearance_.end()) {
          canonical->edge_clearance_[neighbor] = clearance_it->second;
          neighbor->edge_clearance_[canonical] = clearance_it->second;
        }
        if (canonical->weight_.find(neighbor) == canonical->weight_.end() &&
            weight_it != duplicate->weight_.end()) {
          canonical->weight_[neighbor] = weight_it->second;
          neighbor->weight_[canonical] = weight_it->second;
        }
      }
      // The rolling map can expand after insertion, so the current getIndex
      // is not guaranteed to be the region that owns this pointer.  Remove
      // it from every region to prevent stale duplicates from reappearing.
      for (const auto &entry : reg_map_idx2ptr_) {
        if (entry.second) entry.second->topo_nodes_.erase(duplicate);
      }
      duplicate->neighbors_.clear();
      duplicate->paths_.clear();
      duplicate->weight_.clear();
      duplicate->edge_clearance_.clear();
      duplicate->unreachable_nbrs_.clear();
      ++merged;
    }
  }
  return merged;
}

size_t TopoGraph::normalizeConnectivity() {
  unordered_set<TopoNode::Ptr> active;
  vector<TopoNode::Ptr> nodes;
  if (odom_node_) {
    active.insert(odom_node_);
    nodes.push_back(odom_node_);
  }
  for (const auto &entry : reg_map_idx2ptr_) {
    if (!entry.second) continue;
    for (const auto &node : entry.second->topo_nodes_) {
      if (node && active.insert(node).second) nodes.push_back(node);
    }
  }
  size_t removed = 0;
  for (const auto &node : nodes) {
    vector<TopoNode::Ptr> stale;
    for (const auto &neighbor : node->neighbors_) {
      if (!neighbor || !active.count(neighbor) ||
          !neighbor->neighbors_.count(node)) {
        stale.push_back(neighbor);
      }
    }
    for (const auto &neighbor : stale) {
      node->neighbors_.erase(neighbor);
      node->paths_.erase(neighbor);
      node->weight_.erase(neighbor);
      node->edge_clearance_.erase(neighbor);
      node->unreachable_nbrs_.erase(neighbor);
      if (neighbor) {
        neighbor->neighbors_.erase(node);
        neighbor->paths_.erase(node);
        neighbor->weight_.erase(node);
        neighbor->edge_clearance_.erase(node);
        neighbor->unreachable_nbrs_.erase(node);
      }
      ++removed;
    }
  }
  return removed;
}

bool TopoGraph::removeEdge(const TopoNode::Ptr &from, const TopoNode::Ptr &to) {
  if (!from || !to) return false;
  const bool existed = from->neighbors_.count(to) != 0 ||
    to->neighbors_.count(from) != 0 || from->paths_.count(to) != 0 ||
    to->paths_.count(from) != 0 || from->weight_.count(to) != 0 ||
    to->weight_.count(from) != 0;
  from->neighbors_.erase(to);
  to->neighbors_.erase(from);
  from->paths_.erase(to);
  to->paths_.erase(from);
  from->weight_.erase(to);
  to->weight_.erase(from);
  from->edge_clearance_.erase(to);
  to->edge_clearance_.erase(from);
  from->unreachable_nbrs_.erase(to);
  to->unreachable_nbrs_.erase(from);
  return existed;
}

void TopoGraph::removeNode(TopoNode::Ptr &node) {
  if (node == nullptr)
    return;
  // A rolling map may have changed the node's computed region since it was
  // inserted.  Erase by pointer from all regions instead of trusting the
  // current coordinate-to-region lookup.
  for (const auto &entry : reg_map_idx2ptr_) {
    if (entry.second) entry.second->topo_nodes_.erase(node);
  }

  // nbrs
  for (auto &nbr : node->neighbors_) {
    nbr->neighbors_.erase(node);
    nbr->paths_.erase(node);
    nbr->weight_.erase(node);
    nbr->edge_clearance_.erase(node);
    nbr->unreachable_nbrs_.erase(node);
  }
  node->unreachable_nbrs_.clear();
  node->neighbors_.clear();
  node->weight_.clear();
  node->paths_.clear();
  node->edge_clearance_.clear();
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
    const float edge_clearance = witnessMinimumClearance(path);
    new_node->paths_.insert({nbr_nodes[i], path});
    std::reverse(path.begin(), path.end());
    nbr_nodes[i]->paths_.insert({new_node, path});
    new_node->edge_clearance_[nbr_nodes[i]] = edge_clearance;
    nbr_nodes[i]->edge_clearance_[new_node] = edge_clearance;
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
