#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>

#include <Eigen/Dense>

namespace scalenav_graph {

struct SemanticOpportunity
{
  bool valid = false;
  int best_column = -1;
  int route_column = -1;
  float best_risk = 0.0F;
  float route_risk = 0.0F;
  float frame_range = 0.0F;
  float risk_regret = 0.0F;
  float improvement_m = 0.0F;
  Eigen::Vector3f best_world_direction = Eigen::Vector3f::Zero();
};

inline SemanticOpportunity evaluateSemanticOpportunity(
  const std::vector<Eigen::Vector3f> &points,
  const std::vector<float> &scores,
  const std::vector<std::uint8_t> &is_virtual,
  const std::vector<std::int8_t> &columns,
  const Eigen::Vector3f &origin,
  const Eigen::Vector3f &route_target,
  float detour_budget_m,
  float risk_noise_floor)
{
  SemanticOpportunity result;
  if (points.size() != scores.size() || points.size() != is_virtual.size() ||
      points.size() != columns.size() || !origin.allFinite() ||
      !route_target.allFinite()) {
    return result;
  }
  const Eigen::Vector3f target_delta = route_target - origin;
  const float target_norm = target_delta.norm();
  if (!std::isfinite(target_norm) || target_norm < 1e-3F) return result;
  const Eigen::Vector3f target_direction = target_delta / target_norm;

  std::size_t best_index = points.size();
  std::size_t route_index = points.size();
  float frame_max = -std::numeric_limits<float>::infinity();
  float best_alignment = -std::numeric_limits<float>::infinity();
  for (std::size_t index = 0; index < points.size(); ++index) {
    if (is_virtual[index] == 0U || columns[index] < 0 ||
        !points[index].allFinite() || !std::isfinite(scores[index])) {
      continue;
    }
    const float risk = std::clamp(scores[index], 0.0F, 1.0F);
    frame_max = std::max(frame_max, risk);
    if (best_index == points.size() || risk < result.best_risk - 1e-6F ||
        (std::abs(risk - result.best_risk) <= 1e-6F &&
         columns[index] < columns[best_index])) {
      best_index = index;
      result.best_risk = risk;
    }
    const Eigen::Vector3f delta = points[index] - origin;
    const float norm = delta.norm();
    if (!std::isfinite(norm) || norm < 1e-3F) continue;
    const float alignment = (delta / norm).dot(target_direction);
    if (route_index == points.size() || alignment > best_alignment) {
      best_alignment = alignment;
      route_index = index;
    }
  }
  if (best_index == points.size() || route_index == points.size() ||
      !std::isfinite(frame_max)) {
    return result;
  }

  result.valid = true;
  result.best_column = static_cast<int>(columns[best_index]);
  result.route_column = static_cast<int>(columns[route_index]);
  result.route_risk = std::clamp(scores[route_index], 0.0F, 1.0F);
  result.frame_range = std::max(0.0F, frame_max - result.best_risk);
  result.risk_regret = std::clamp(
    (result.route_risk - result.best_risk) /
      std::max(std::clamp(risk_noise_floor, 1e-3F, 1.0F), result.frame_range),
    0.0F, 1.0F);
  result.improvement_m = std::max(0.0F, detour_budget_m) * result.risk_regret;
  result.best_world_direction = (points[best_index] - origin).normalized();
  return result;
}

inline bool updateSemanticOpportunityPersistence(
  const SemanticOpportunity &observation,
  float minimum_improvement_m,
  float direction_match_cosine,
  int required_frames,
  Eigen::Vector3f &pending_world_direction,
  int &pending_frames)
{
  const bool is_opportunity = observation.valid &&
    observation.best_column != observation.route_column &&
    observation.improvement_m >= std::max(0.0F, minimum_improvement_m) &&
    observation.best_world_direction.allFinite() &&
    observation.best_world_direction.norm() > 0.5F;
  if (!is_opportunity) {
    pending_world_direction.setZero();
    pending_frames = 0;
    return false;
  }

  const Eigen::Vector3f direction = observation.best_world_direction.normalized();
  const float threshold = std::clamp(direction_match_cosine, -1.0F, 1.0F);
  if (pending_frames > 0 && pending_world_direction.norm() > 0.5F &&
      pending_world_direction.normalized().dot(direction) >= threshold) {
    ++pending_frames;
    pending_world_direction =
      (pending_world_direction.normalized() + direction).normalized();
  } else {
    pending_world_direction = direction;
    pending_frames = 1;
  }
  return pending_frames >= std::max(1, required_frames);
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

inline bool routeProgressReachedFraction(
  float progress_m, float initial_length_m, float fraction)
{
  if (!std::isfinite(progress_m) || !std::isfinite(initial_length_m) ||
      !std::isfinite(fraction) || initial_length_m <= 1e-3F) {
    return false;
  }
  return progress_m >= std::clamp(fraction, 0.0F, 1.0F) * initial_length_m;
}

inline bool frontierCommandReleaseAllowed(
  bool have_active_command, float progress_m, float initial_length_m,
  float progress_fraction, bool semantic_edge_invalid, bool mission_goal_direct)
{
  return !have_active_command || semantic_edge_invalid || mission_goal_direct ||
    routeProgressReachedFraction(progress_m, initial_length_m, progress_fraction);
}

inline bool missionGoalWithinDirectHorizon(
  float vehicle_to_goal_m, float goal_connect_distance_m, float lookahead_m)
{
  if (!std::isfinite(vehicle_to_goal_m) || vehicle_to_goal_m < 0.0F) return false;
  const float direct_horizon_m = std::max(
    std::max(0.0F, goal_connect_distance_m), std::max(0.0F, lookahead_m));
  return vehicle_to_goal_m <= direct_horizon_m;
}

inline bool semanticPointCanInfluenceFixedLayer(
  float point_z, float layer_z, float influence_m)
{
  if (!std::isfinite(point_z) || !std::isfinite(layer_z) ||
      !std::isfinite(influence_m)) {
    return false;
  }
  return std::abs(point_z - layer_z) <= std::max(0.0F, influence_m);
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

// Return the arc-length parameter of the closest point on an ordered route.
// The parameter follows the route direction; it is therefore suitable for
// tracking execution progress even when the vehicle moves laterally around a
// local obstacle.
inline float routeProgressAlongPath(const std::vector<Eigen::Vector3f> &route,
                                    const Eigen::Vector3f &position)
{
  if (route.size() < 2 || !position.allFinite()) {
    return std::numeric_limits<float>::quiet_NaN();
  }
  float total_before = 0.0F;
  float best_progress = 0.0F;
  float best_distance_sq = std::numeric_limits<float>::infinity();
  for (std::size_t i = 1; i < route.size(); ++i) {
    const Eigen::Vector3f segment = route[i] - route[i - 1];
    const float length_sq = segment.squaredNorm();
    if (length_sq < 1e-8F) continue;
    const float length = std::sqrt(length_sq);
    const float t = std::clamp(
      (position - route[i - 1]).dot(segment) / length_sq, 0.0F, 1.0F);
    const Eigen::Vector3f projection = route[i - 1] + t * segment;
    const float distance_sq = (position - projection).squaredNorm();
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_progress = total_before + t * length;
    }
    total_before += length;
  }
  return std::isfinite(best_distance_sq) ? best_progress :
    std::numeric_limits<float>::quiet_NaN();
}

// Low-order polynomial P(t) fitted to witness polyline control points.  Arc-length
// of the original witness defines t in [0, 1].  Progress and subgoal sampling both
// use this curve; witness polylines remain the route source for A* and clearance.
struct WitnessParametricCurve
{
  int degree = 0;
  Eigen::MatrixXf coefficients;
  float total_length = 0.0F;
  bool valid = false;

  static WitnessParametricCurve fit(const std::vector<Eigen::Vector3f> &route)
  {
    WitnessParametricCurve curve;
    if (route.size() < 2) return curve;
    curve.degree = std::min<int>(3, static_cast<int>(route.size()) - 1);
    const int rows = static_cast<int>(route.size());
    Eigen::MatrixXf basis(rows, curve.degree + 1);
    Eigen::MatrixXf values(rows, 3);
    float total_length = 0.0F;
    std::vector<float> arc(route.size(), 0.0F);
    for (std::size_t i = 1; i < route.size(); ++i) {
      const float length = (route[i] - route[i - 1]).norm();
      if (std::isfinite(length)) total_length += length;
      arc[i] = total_length;
    }
    if (!std::isfinite(total_length) || total_length < 1e-4F) return curve;
    curve.total_length = total_length;
    for (int row = 0; row < rows; ++row) {
      const float t = arc[static_cast<std::size_t>(row)] / total_length;
      float power = 1.0F;
      for (int column = 0; column <= curve.degree; ++column) {
        basis(row, column) = power;
        power *= t;
      }
      values.row(row) = route[static_cast<std::size_t>(row)].transpose();
    }
    curve.coefficients = basis.colPivHouseholderQr().solve(values);
    curve.valid = curve.coefficients.allFinite();
    return curve;
  }

  // Fit P(t) while enforcing the replanning state exactly. Here t is the
  // normalized traversal time and T=route_length/|velocity|, so
  // P'(0)=velocity*T is equivalent to the physical boundary dP/dtime=velocity.
  // P(1) is also fixed at the witness endpoint; only the remaining cubic
  // shape coefficient is least-squares fitted to the interior witness points.
  static WitnessParametricCurve fitWithInitialVelocity(
      const std::vector<Eigen::Vector3f> &route,
      const Eigen::Vector3f &initial_velocity)
  {
    WitnessParametricCurve curve;
    if (route.size() < 2 || !route.front().allFinite() ||
        !route.back().allFinite()) {
      return curve;
    }

    float total_length = 0.0F;
    std::vector<float> arc(route.size(), 0.0F);
    for (std::size_t i = 1; i < route.size(); ++i) {
      const float length = (route[i] - route[i - 1]).norm();
      if (!std::isfinite(length)) return curve;
      total_length += length;
      arc[i] = total_length;
    }
    if (!std::isfinite(total_length) || total_length < 1e-4F) return curve;

    Eigen::Vector3f start_tangent = Eigen::Vector3f::Zero();
    const float speed = initial_velocity.norm();
    if (initial_velocity.allFinite() && std::isfinite(speed) && speed > 0.1F) {
      start_tangent = initial_velocity * (total_length / speed);
    } else {
      for (std::size_t i = 1; i < route.size(); ++i) {
        const Eigen::Vector3f segment = route[i] - route.front();
        const float distance = segment.norm();
        if (std::isfinite(distance) && distance > 1e-4F) {
          start_tangent = segment * (total_length / distance);
          break;
        }
      }
    }
    if (!start_tangent.allFinite() || start_tangent.norm() < 1e-4F) return curve;

    const Eigen::Vector3f start = route.front();
    const Eigen::Vector3f endpoint_remainder = route.back() - start - start_tangent;
    Eigen::Vector3f fitted_quadratic = Eigen::Vector3f::Zero();
    float basis_energy = 0.0F;
    for (std::size_t i = 1; i + 1 < route.size(); ++i) {
      const float t = arc[i] / total_length;
      const float t2 = t * t;
      const float t3 = t2 * t;
      const float basis = t2 - t3;
      const Eigen::Vector3f residual = route[i] - start - start_tangent * t -
        endpoint_remainder * t3;
      fitted_quadratic += basis * residual;
      basis_energy += basis * basis;
    }
    if (basis_energy > 1e-8F) fitted_quadratic /= basis_energy;

    curve.degree = 3;
    curve.total_length = total_length;
    curve.coefficients.resize(4, 3);
    curve.coefficients.row(0) = start.transpose();
    curve.coefficients.row(1) = start_tangent.transpose();
    curve.coefficients.row(2) = fitted_quadratic.transpose();
    curve.coefficients.row(3) =
      (endpoint_remainder - fitted_quadratic).transpose();
    curve.valid = curve.coefficients.allFinite();
    return curve;
  }

  Eigen::Vector3f evaluate(float t) const
  {
    if (!valid) {
      return Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());
    }
    const float clamped = std::clamp(t, 0.0F, 1.0F);
    Eigen::VectorXf powers(degree + 1);
    float power = 1.0F;
    for (int column = 0; column <= degree; ++column) {
      powers(column) = power;
      power *= clamped;
    }
    return (powers.transpose() * coefficients).transpose();
  }

  Eigen::Vector3f derivative(float t) const
  {
    if (!valid) {
      return Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());
    }
    const float clamped = std::clamp(t, 0.0F, 1.0F);
    Eigen::Vector3f result = Eigen::Vector3f::Zero();
    float power = 1.0F;
    for (int column = 1; column <= degree; ++column) {
      result += static_cast<float>(column) * power *
        coefficients.row(column).transpose();
      power *= clamped;
    }
    return result;
  }
};

inline bool routeProgressTAlongCurve(const WitnessParametricCurve &curve,
                                     const Eigen::Vector3f &position,
                                     float minimum_t,
                                     float &progress_t)
{
  if (!curve.valid || !position.allFinite()) return false;

  const float start_t = std::clamp(minimum_t, 0.0F, 1.0F);
  constexpr int samples = 128;
  float best_t = start_t;
  float best_distance_sq = std::numeric_limits<float>::infinity();
  for (int sample = 0; sample <= samples; ++sample) {
    const float t = start_t + (1.0F - start_t) *
      static_cast<float>(sample) / static_cast<float>(samples);
    const Eigen::Vector3f estimate = curve.evaluate(t);
    if (!estimate.allFinite()) continue;
    const float distance_sq = (position - estimate).squaredNorm();
    if (distance_sq < best_distance_sq) {
      best_distance_sq = distance_sq;
      best_t = t;
    }
  }
  if (!std::isfinite(best_distance_sq)) return false;
  progress_t = std::max(start_t, best_t);
  return true;
}

inline bool routeProgressTAlongPath(const std::vector<Eigen::Vector3f> &route,
                                    const Eigen::Vector3f &position,
                                    float minimum_t,
                                    float &progress_t)
{
  if (route.size() < 2 || !position.allFinite()) return false;
  const WitnessParametricCurve curve = WitnessParametricCurve::fit(route);
  return routeProgressTAlongCurve(curve, position, minimum_t, progress_t);
}

// Return the original witness suffix at a monotonic normalized parameter.
// Interpolation is done on the witness itself so this helper never publishes
// points introduced by the polynomial fit.
inline std::vector<Eigen::Vector3f> forwardRouteFromT(
    const std::vector<Eigen::Vector3f> &route, float progress_t)
{
  if (route.size() < 2) return {};
  const float target_t = std::clamp(progress_t, 0.0F, 1.0F);
  float total_length = 0.0F;
  for (std::size_t i = 1; i < route.size(); ++i) {
    const float length = (route[i] - route[i - 1]).norm();
    if (std::isfinite(length)) total_length += length;
  }
  if (!std::isfinite(total_length) || total_length < 1e-4F) return {route.back()};
  const float target_length = target_t * total_length;
  float accumulated = 0.0F;
  std::vector<Eigen::Vector3f> forward;
  for (std::size_t i = 1; i < route.size(); ++i) {
    const Eigen::Vector3f segment = route[i] - route[i - 1];
    const float length = segment.norm();
    if (!std::isfinite(length) || length < 1e-6F) continue;
    if (target_length <= accumulated + length) {
      const float local = std::clamp((target_length - accumulated) / length, 0.0F, 1.0F);
      forward.push_back(route[i - 1] + local * segment);
      for (std::size_t j = i; j < route.size(); ++j) {
        if ((forward.back() - route[j]).norm() > 1e-4F) forward.push_back(route[j]);
      }
      return forward;
    }
    accumulated += length;
  }
  return {route.back()};
}

// Build the polynomial guide used for monotonic progress and subgoal sampling.
// The vehicle pose and optional velocity tangent anchor t=0; the ordered
// witness supplies the remaining samples. All points are snapped to layer_z
// when it is finite so fixed-height validation stays consistent.
inline std::vector<Eigen::Vector3f> buildPolynomialGuidePath(
    const std::vector<Eigen::Vector3f> &witness,
    const Eigen::Vector3f &position, const Eigen::Vector3f &velocity,
    float layer_z = std::numeric_limits<float>::quiet_NaN())
{
  const auto snap = [layer_z](const Eigen::Vector3f &point) {
    Eigen::Vector3f snapped = point;
    if (std::isfinite(layer_z)) snapped.z() = layer_z;
    return snapped;
  };
  if (witness.size() < 2 || !position.allFinite()) return {};

  std::vector<Eigen::Vector3f> guide;
  guide.push_back(snap(position));
  const float speed = velocity.norm();
  if (std::isfinite(speed) && speed > 0.1F) {
    constexpr float velocity_boundary_dt_s = 0.2F;
    guide.push_back(snap(position + velocity_boundary_dt_s * velocity));
  }
  for (const auto &point : witness) {
    const Eigen::Vector3f snapped = snap(point);
    if ((snapped - guide.back()).norm() > 1e-3F) guide.push_back(snapped);
  }
  return guide.size() >= 2 ? guide : std::vector<Eigen::Vector3f>{};
}

inline bool routeLookaheadPointFromCurve(
    const WitnessParametricCurve &curve, const Eigen::Vector3f &position,
    float minimum_t, float lookahead, Eigen::Vector3f &point);

inline bool routeLookaheadPointFromT(
    const std::vector<Eigen::Vector3f> &route, const Eigen::Vector3f &position,
    float minimum_t, float lookahead, Eigen::Vector3f &point)
{
  if (!position.allFinite() || !std::isfinite(lookahead) || lookahead <= 0.0F) {
    return false;
  }
  const WitnessParametricCurve curve = WitnessParametricCurve::fit(route);
  return routeLookaheadPointFromCurve(curve, position, minimum_t, lookahead, point);
}

inline bool routeLookaheadPointFromCurve(
    const WitnessParametricCurve &curve, const Eigen::Vector3f &position,
    float minimum_t, float lookahead, Eigen::Vector3f &point)
{
  if (!position.allFinite() || !std::isfinite(lookahead) || lookahead <= 0.0F ||
      !curve.valid || curve.total_length < 1e-4F) {
    return false;
  }
  float current_t = minimum_t;
  if (!routeProgressTAlongCurve(curve, position, minimum_t, current_t)) return false;
  const float delta_t = lookahead / curve.total_length;
  point = curve.evaluate(std::clamp(current_t + delta_t, current_t, 1.0F));
  return point.allFinite();
}

inline bool isContinuousForwardRouteFromT(
    const Eigen::Vector3f &vehicle, const std::vector<Eigen::Vector3f> &route,
    float minimum_t, float maximum_lateral_distance)
{
  if (route.size() < 2 || maximum_lateral_distance < 0.0F) return false;
  float projected_t = 0.0F;
  if (!routeProgressTAlongPath(route, vehicle, minimum_t, projected_t)) return false;
  const WitnessParametricCurve curve = WitnessParametricCurve::fit(route);
  if (!curve.valid) return false;
  const Eigen::Vector3f on_curve = curve.evaluate(projected_t);
  if (!on_curve.allFinite()) return false;
  return (vehicle - on_curve).norm() <= maximum_lateral_distance &&
         forwardRouteFromT(route, projected_t).size() >= 2;
}

// Evaluate the cubic (or lower-order) least-squares curve used for ordered
// witness parameterization. The witness endpoints are kept exact.
inline bool routePolynomialPointAtT(const std::vector<Eigen::Vector3f> &route,
                                    float t, Eigen::Vector3f &point)
{
  if (route.size() < 2 || !std::isfinite(t)) return false;
  const int degree = std::min<int>(3, static_cast<int>(route.size()) - 1);
  const int rows = static_cast<int>(route.size());
  Eigen::MatrixXf basis(rows, degree + 1);
  Eigen::MatrixXf values(rows, 3);
  float total_length = 0.0F;
  std::vector<float> arc(route.size(), 0.0F);
  for (std::size_t i = 1; i < route.size(); ++i) {
    const float length = (route[i] - route[i - 1]).norm();
    if (std::isfinite(length)) total_length += length;
    arc[i] = total_length;
  }
  if (!std::isfinite(total_length) || total_length < 1e-4F) return false;
  for (int row = 0; row < rows; ++row) {
    const float sample_t = arc[static_cast<std::size_t>(row)] / total_length;
    float power = 1.0F;
    for (int column = 0; column <= degree; ++column) {
      basis(row, column) = power;
      power *= sample_t;
    }
    values.row(row) = route[static_cast<std::size_t>(row)].transpose();
  }
  const Eigen::MatrixXf coefficients = basis.colPivHouseholderQr().solve(values);
  if (!coefficients.allFinite()) return false;
  const float target_t = std::clamp(t, 0.0F, 1.0F);
  Eigen::VectorXf powers(degree + 1);
  float power = 1.0F;
  for (int column = 0; column <= degree; ++column) {
    powers(column) = power;
    power *= target_t;
  }
  point = (powers.transpose() * coefficients).transpose();
  if (target_t <= 1e-5F) point = route.front();
  if (target_t >= 1.0F - 1e-5F) point = route.back();
  return point.allFinite();
}

// Return the route suffix after a monotonic arc-length progress value. The
// nearest projection is only allowed on segments at or after min_progress;
// nearby loops therefore cannot move the execution list back to an older
// segment.
inline std::vector<Eigen::Vector3f> forwardRouteFromProgress(
    const std::vector<Eigen::Vector3f> &route, const Eigen::Vector3f &position,
    float min_progress)
{
  if (route.size() < 2 || !position.allFinite()) return {};
  const float required_progress = std::max(0.0F, min_progress);
  float total_before = 0.0F;
  float best_distance_sq = std::numeric_limits<float>::infinity();
  std::size_t best_segment = route.size() - 2;
  float best_t = 1.0F;
  bool found = false;
  for (std::size_t i = 1; i < route.size(); ++i) {
    const Eigen::Vector3f segment = route[i] - route[i - 1];
    const float length_sq = segment.squaredNorm();
    if (length_sq < 1e-8F) continue;
    const float length = std::sqrt(length_sq);
    const float segment_end = total_before + length;
    if (segment_end + 1e-4F < required_progress) {
      total_before = segment_end;
      continue;
    }
    const float min_t = std::clamp(
      (required_progress - total_before) / length, 0.0F, 1.0F);
    const float projected_t = std::clamp(
      (position - route[i - 1]).dot(segment) / length_sq, min_t, 1.0F);
    const Eigen::Vector3f projection = route[i - 1] + projected_t * segment;
    const float distance_sq = (position - projection).squaredNorm();
    if (!found || distance_sq < best_distance_sq) {
      found = true;
      best_distance_sq = distance_sq;
      best_segment = i - 1;
      best_t = projected_t;
    }
    total_before = segment_end;
  }
  if (!found) return {route.back()};
  std::vector<Eigen::Vector3f> forward;
  const Eigen::Vector3f projection = route[best_segment] + best_t *
    (route[best_segment + 1] - route[best_segment]);
  forward.push_back(projection);
  for (std::size_t i = best_segment + 1; i < route.size(); ++i) {
    if ((forward.back() - route[i]).norm() > 1e-4F) forward.push_back(route[i]);
  }
  return forward;
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

inline bool shouldReuseFrontierGoal(const Eigen::Vector3f &vehicle,
                                const Eigen::Vector3f &frontier_goal,
                                float release_distance)
{
  return (vehicle - frontier_goal).norm() > std::max(0.0F, release_distance);
}

// A remembered frontier_goal is valid only while the vehicle remains on the
// directed witness route that led to it.  Distance to the frontier_goal alone is
// insufficient: a vehicle that has passed or circled around the frontier_goal is
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
