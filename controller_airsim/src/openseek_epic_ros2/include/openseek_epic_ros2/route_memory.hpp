#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Dense>

namespace openseek_epic {

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
