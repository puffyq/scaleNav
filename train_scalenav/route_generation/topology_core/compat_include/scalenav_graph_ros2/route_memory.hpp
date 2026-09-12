#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>

#include <Eigen/Dense>

namespace scalenav_graph {

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

inline bool routeLookaheadPoint(
    const std::vector<Eigen::Vector3f> &path, const Eigen::Vector3f &position,
    float lookahead, Eigen::Vector3f &point)
{
  if (!position.allFinite() || !std::isfinite(lookahead) || lookahead <= 0.0F) {
    return false;
  }
  const auto window = forwardRouteWindow(path, position, lookahead);
  if (window.empty() || !window.back().allFinite()) return false;
  point = window.back();
  return true;
}

// Return the complete remembered route ahead of the vehicle.  Unlike
// forwardRouteWindow(), this does not truncate at a planning horizon; it is
// used for the persistent route display and for selecting a subgoal from the
// same route that A* already committed to.
inline std::vector<Eigen::Vector3f> forwardRouteFromPosition(
  const std::vector<Eigen::Vector3f> &path, const Eigen::Vector3f &position)
{
  if (path.size() < 2) return {};
  float best_distance_sq = std::numeric_limits<float>::infinity();
  std::size_t best_segment = 0;
  float best_t = 0.0F;
  for (std::size_t i = 1; i < path.size(); ++i) {
    const Eigen::Vector3f segment = path[i] - path[i - 1];
    const float length_sq = segment.squaredNorm();
    if (length_sq < 1e-8F) continue;
    const float t = std::clamp(
      (position - path[i - 1]).dot(segment) / length_sq, 0.0F, 1.0F);
    const Eigen::Vector3f projection = path[i - 1] + t * segment;
    const float distance_sq = (position - projection).squaredNorm();
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_segment = i - 1;
      best_t = t;
    }
  }
  if (!std::isfinite(best_distance_sq)) return {};

  std::vector<Eigen::Vector3f> forward;
  const Eigen::Vector3f projection = path[best_segment] + best_t *
    (path[best_segment + 1] - path[best_segment]);
  forward.push_back(projection);
  for (std::size_t i = best_segment + 1; i < path.size(); ++i) {
    if ((forward.back() - path[i]).norm() > 1e-4F) forward.push_back(path[i]);
  }
  return forward;
}

inline bool isContinuousForwardRoute(
  const Eigen::Vector3f &vehicle, const std::vector<Eigen::Vector3f> &route,
  float maximum_lateral_distance)
{
  if (route.size() < 2 || maximum_lateral_distance < 0.0F) return false;
  return pointPathDistance(vehicle, route) <= maximum_lateral_distance &&
         forwardRouteFromPosition(route, vehicle).size() >= 2;
}

inline bool shouldSwitchRoute(
  bool hard_switch, float incumbent_risk, float candidate_risk,
  float risk_margin, float incumbent_objective, float candidate_objective,
  float progress_delta, float progress_margin, float cost_ratio)
{
  if (hard_switch) return true;
  const bool risk_better = std::isfinite(incumbent_risk) &&
    std::isfinite(candidate_risk) && candidate_risk + std::max(0.0F, risk_margin) < incumbent_risk;
  const bool cost_better = std::isfinite(incumbent_objective) &&
    std::isfinite(candidate_objective) &&
    std::abs(progress_delta) <= std::max(0.0F, progress_margin) &&
    candidate_objective < incumbent_objective * std::clamp(cost_ratio, 0.0F, 1.0F);
  return risk_better || cost_better;
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

inline float routeLength(const std::vector<Eigen::Vector3f> &route)
{
  float length = 0.0F;
  for (std::size_t i = 1; i < route.size(); ++i) {
    const float segment = (route[i] - route[i - 1]).norm();
    if (std::isfinite(segment)) length += segment;
  }
  return length;
}

// A rolling frontier may move forward without reopening the already accepted
// corridor. Require a longer candidate and protect the prefix of the current
// route; a candidate that changes lanes near the vehicle is a route switch and
// must pass the normal risk/cost hysteresis instead.
inline bool candidateExtendsAcceptedRoute(
    const std::vector<Eigen::Vector3f> &accepted,
    const std::vector<Eigen::Vector3f> &candidate,
    float minimum_progress_gain, float maximum_lateral_distance,
    float protected_fraction = 0.7F)
{
  if (accepted.size() < 2 || candidate.size() < 2 ||
      maximum_lateral_distance < 0.0F) return false;
  const float accepted_length = routeLength(accepted);
  const float candidate_length = routeLength(candidate);
  if (!std::isfinite(accepted_length) || !std::isfinite(candidate_length) ||
      candidate_length < accepted_length + std::max(0.0F, minimum_progress_gain)) {
    return false;
  }

  const float protected_length = accepted_length *
    std::clamp(protected_fraction, 0.0F, 1.0F);
  float progress = 0.0F;
  for (std::size_t i = 0; i < accepted.size(); ++i) {
    if (pointPathDistance(accepted[i], candidate) > maximum_lateral_distance)
      return false;
    if (i + 1 >= accepted.size()) break;
    progress += (accepted[i + 1] - accepted[i]).norm();
    if (progress >= protected_length) break;
  }
  return true;
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

// Semantic heatmaps arrive more often than a route should be replaced.  A
// route is invalidated only when the risk field changes materially; ordinary
// EMA updates and unrelated patches must leave the YOPO guide stable.
// Semantic replan is triggered only when risk along the current guide rises
// materially.  Decreases or semantic-point insertion artifacts must not wipe
// the rolling corridor memory.
inline bool semanticRiskIncreaseRequiresReplan(float before, float after,
                                               float minimum_delta)
{
  if (!std::isfinite(before) || !std::isfinite(after)) return false;
  return after >= before + std::max(0.0F, minimum_delta);
}

inline bool semanticRiskChangeRequiresReplan(float before, float after,
                                             float minimum_delta)
{
  return semanticRiskIncreaseRequiresReplan(before, after, minimum_delta);
}

// The original EPIC route is geometry-driven.  Semantic frames may update the
// cost field without invalidating a valid rolling route; route replacement is
// opt-in because noisy semantic streams otherwise cause branch oscillation.
inline bool semanticRouteResetRequested(bool enabled, float before, float after,
                                        float minimum_delta)
{
  return enabled && semanticRiskChangeRequiresReplan(before, after, minimum_delta);
}

}  // namespace scalenav_graph
