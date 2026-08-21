#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Dense>

namespace openseek_epic {

struct RaycastShortcutStats
{
  std::size_t clearance_queries = 0;
  std::size_t tested_segments = 0;
  std::size_t accepted_segments = 0;
};

template<typename ClearanceQuery>
inline bool segmentHasClearance(const Eigen::Vector3f &start,
                                const Eigen::Vector3f &end,
                                float sample_step,
                                float minimum_clearance,
                                ClearanceQuery &query,
                                RaycastShortcutStats *stats = nullptr)
{
  if (!start.allFinite() || !end.allFinite() || !std::isfinite(sample_step) ||
      !std::isfinite(minimum_clearance) || sample_step <= 0.0F ||
      minimum_clearance < 0.0F) {
    return false;
  }

  const Eigen::Vector3f segment = end - start;
  const float length = segment.norm();
  const std::size_t intervals = std::max<std::size_t>(
    1, static_cast<std::size_t>(std::ceil(length / sample_step)));
  for (std::size_t i = 0; i <= intervals; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(intervals);
    const float clearance = static_cast<float>(query(start + t * segment));
    if (stats) ++stats->clearance_queries;
    // Bubble A* has already collision-validated the witness endpoints, but a
    // Bubble center need not coincide with a ray-carved voxel. Unknown
    // endpoints are therefore allowed; unknown interior samples are not.
    if (std::isnan(clearance)) {
      if (i == 0 || i == intervals) continue;
      return false;
    }
    if (!std::isfinite(clearance) || clearance < minimum_clearance) return false;
  }
  return true;
}

// Greedily connect every retained point to the farthest later witness point
// whose complete line segment satisfies the live obstacle-distance query.
// Unknown space inherits the map query's policy; the online EPIC adapter
// returns max range when no observed obstacle is available.
template<typename ClearanceQuery>
inline std::vector<Eigen::Vector3f> farthestVisibleShortcut(
    const std::vector<Eigen::Vector3f> &path,
    float sample_step,
    float minimum_clearance,
    ClearanceQuery query,
    RaycastShortcutStats *stats = nullptr)
{
  std::vector<Eigen::Vector3f> compact;
  compact.reserve(path.size());
  for (const auto &point : path) {
    if (compact.empty() || (compact.back() - point).norm() > 1e-4F) {
      compact.push_back(point);
    }
  }
  if (compact.size() <= 2) return compact;

  std::vector<Eigen::Vector3f> shortened;
  shortened.reserve(compact.size());
  shortened.push_back(compact.front());
  std::size_t anchor = 0;
  while (anchor + 1 < compact.size()) {
    std::size_t selected = anchor + 1;
    bool found_visible = false;
    for (std::size_t candidate = compact.size() - 1; candidate > anchor; --candidate) {
      if (stats) ++stats->tested_segments;
      if (segmentHasClearance(
          compact[anchor], compact[candidate], sample_step, minimum_clearance,
          query, stats)) {
        selected = candidate;
        found_visible = true;
        break;
      }
    }
    if (found_visible && stats) ++stats->accepted_segments;
    shortened.push_back(compact[selected]);
    anchor = selected;
  }
  return shortened;
}

inline float pointSegmentDistance(const Eigen::Vector3f &point,
                                  const Eigen::Vector3f &start,
                                  const Eigen::Vector3f &end)
{
  const Eigen::Vector3f segment = end - start;
  const float length_sq = segment.squaredNorm();
  if (length_sq < 1e-8F) return (point - start).norm();
  const float t = std::clamp((point - start).dot(segment) / length_sq, 0.0F, 1.0F);
  return (point - (start + t * segment)).norm();
}

inline float pointPathDistance(const Eigen::Vector3f &point,
                               const std::vector<Eigen::Vector3f> &path)
{
  if (path.empty()) return std::numeric_limits<float>::infinity();
  if (path.size() == 1) return (point - path.front()).norm();
  float distance = std::numeric_limits<float>::infinity();
  for (std::size_t i = 1; i < path.size(); ++i) {
    distance = std::min(distance, pointSegmentDistance(point, path[i - 1], path[i]));
  }
  return distance;
}

inline std::vector<Eigen::Vector3f> forwardRouteWindow(
    const std::vector<Eigen::Vector3f> &path, const Eigen::Vector3f &position,
    float horizon)
{
  if (path.size() < 2 || horizon <= 0.0F) return {};
  float best_distance_sq = std::numeric_limits<float>::infinity();
  std::size_t best_segment = 0;
  float best_t = 0.0F;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector3f segment = path[i] - path[i - 1];
    const float length_sq = segment.squaredNorm();
    if (length_sq < 1e-8F) continue;
    const float t = std::clamp(
      (position - path[i - 1]).dot(segment) / length_sq, 0.0F, 1.0F);
    const float distance_sq =
      (position - (path[i - 1] + t * segment)).squaredNorm();
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_segment = i - 1;
      best_t = t;
    }
  }

  std::vector<Eigen::Vector3f> window;
  const Eigen::Vector3f projection =
    path[best_segment] + best_t * (path[best_segment + 1] - path[best_segment]);
  window.push_back(projection);
  float remaining = horizon;
  Eigen::Vector3f cursor = projection;
  for (std::size_t i = best_segment + 1; i < path.size() && remaining > 0.0F; ++i) {
    const Eigen::Vector3f segment = path[i] - cursor;
    const float length = segment.norm();
    if (length < 1e-4F) {
      cursor = path[i];
      continue;
    }
    if (length > remaining) {
      window.push_back(cursor + (remaining / length) * segment);
      remaining = 0.0F;
    } else {
      window.push_back(path[i]);
      remaining -= length;
      cursor = path[i];
    }
  }
  return window;
}

inline bool edgeFollowsRoute(const Eigen::Vector3f &start,
                             const Eigen::Vector3f &end,
                             const std::vector<Eigen::Vector3f> &route,
                             float maximum_distance)
{
  const Eigen::Vector3f midpoint = 0.5F * (start + end);
  return pointPathDistance(start, route) <= maximum_distance &&
         pointPathDistance(end, route) <= maximum_distance &&
         pointPathDistance(midpoint, route) <= maximum_distance;
}

inline bool shouldReuseTerminal(const Eigen::Vector3f &vehicle,
                                const Eigen::Vector3f &terminal,
                                float release_distance)
{
  return (vehicle - terminal).norm() > std::max(0.0F, release_distance);
}

inline float velocityCompensatedLookahead(float minimum_lookahead,
                                          float speed,
                                          float planning_period_seconds,
                                          float reserve_distance)
{
  if (!std::isfinite(minimum_lookahead) || !std::isfinite(speed) ||
      !std::isfinite(planning_period_seconds) || !std::isfinite(reserve_distance)) {
    return std::max(0.0F, minimum_lookahead);
  }
  return std::max(
    std::max(0.0F, minimum_lookahead),
    std::max(0.0F, speed) * std::max(0.0F, planning_period_seconds) +
      std::max(0.0F, reserve_distance));
}

// A remembered terminal is valid only while the vehicle remains on the
// directed witness route that led to it.  Distance to the terminal alone is
// insufficient: a vehicle that has passed or circled around the terminal is
// still far away and would otherwise keep reusing the same stale edge.
inline bool canReuseForwardRoute(const Eigen::Vector3f &vehicle,
                                 const std::vector<Eigen::Vector3f> &route,
                                 float release_distance,
                                 float maximum_lateral_distance)
{
  if (route.size() < 2 || maximum_lateral_distance < 0.0F) return false;

  float nearest_distance_sq = std::numeric_limits<float>::infinity();
  float nearest_progress = 0.0F;
  float total_length = 0.0F;
  for (std::size_t i = 1; i < route.size(); ++i) {
    const Eigen::Vector3f segment = route[i] - route[i - 1];
    const float length_sq = segment.squaredNorm();
    if (length_sq < 1e-8F) continue;
    const float length = std::sqrt(length_sq);
    const float t = std::clamp(
      (vehicle - route[i - 1]).dot(segment) / length_sq, 0.0F, 1.0F);
    const Eigen::Vector3f projection = route[i - 1] + t * segment;
    const float distance_sq = (vehicle - projection).squaredNorm();
    if (distance_sq < nearest_distance_sq) {
      nearest_distance_sq = distance_sq;
      nearest_progress = total_length + t * length;
    }
    total_length += length;
  }

  if (!std::isfinite(nearest_distance_sq) ||
      std::sqrt(nearest_distance_sq) > maximum_lateral_distance) {
    return false;
  }
  return total_length - nearest_progress > std::max(0.0F, release_distance);
}

}  // namespace openseek_epic
