#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <string>
#include <utility>
#include <vector>

#include <Eigen/Dense>

namespace scalenav {

struct CoarseGridAstarConfig {
  float resolution_m = 0.5F;
  float inflate_radius_m = 0.61F;
  float origin_x_m = -50.0F;
  float origin_y_m = -10.0F;
  float origin_z_m = 0.0F;
  int width = 200;
  int height = 340;
  int depth = 20;
  float local_radius_m = 20.0F;
};

struct CoarseGridSearchResult {
  bool success = false;
  std::string reason = "uninitialized";
  std::vector<Eigen::Vector3f> path;
  Eigen::Vector3f clipped_goal = Eigen::Vector3f::Zero();
  float path_length_m = 0.0F;
  int expansions = 0;
  bool clipped = false;
};

class CoarseGridAstar {
 public:
  explicit CoarseGridAstar(CoarseGridAstarConfig config = {})
      : config_(sanitize(config)),
        occupied_(cellCount(), 0),
        blocked_(occupied_),
        g_score_(cellCount(), std::numeric_limits<float>::infinity()),
        parent_(cellCount(), -1),
        stamp_(cellCount(), 0) {}

  const CoarseGridAstarConfig &config() const { return config_; }

  void clear() {
    std::fill(occupied_.begin(), occupied_.end(), 0);
    std::fill(blocked_.begin(), blocked_.end(), 0);
    occupied_count_ = 0;
    blocked_count_ = 0;
    dirty_ = false;
  }

  bool markWorld(float x, float y, float z) {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z)) return false;
    int ix = 0;
    int iy = 0;
    int iz = 0;
    if (!worldToIndex(x, y, z, ix, iy, iz)) return false;
    auto &cell = occupied_[static_cast<std::size_t>(index(ix, iy, iz))];
    if (cell != 0) return false;
    cell = 1;
    ++occupied_count_;
    dirty_ = true;
    return true;
  }

  template <typename Point>
  std::size_t markHits(const Point *points, std::size_t count) {
    std::size_t added = 0;
    for (std::size_t i = 0; i < count; ++i) {
      if (markWorld(points[i].x, points[i].y, points[i].z)) ++added;
    }
    return added;
  }

  std::size_t occupiedCount() const { return occupied_count_; }
  std::size_t blockedCount() {
    inflateIfNeeded();
    return blocked_count_;
  }

  void retainAround(float x, float y, float z) {
    const float radius = std::max(config_.local_radius_m, config_.resolution_m) + 4.0F;
    const float radius_sq = radius * radius;
    bool changed = false;
    for (int iz = 0; iz < config_.depth; ++iz) {
      for (int iy = 0; iy < config_.height; ++iy) {
        for (int ix = 0; ix < config_.width; ++ix) {
          auto &cell = occupied_[static_cast<std::size_t>(index(ix, iy, iz))];
          if (cell == 0) continue;
          const Eigen::Vector3f center = indexToCenter(ix, iy, iz);
          const Eigen::Vector3f delta(center.x() - x, center.y() - y, center.z() - z);
          if (delta.squaredNorm() <= radius_sq) continue;
          cell = 0;
          if (occupied_count_ > 0) --occupied_count_;
          changed = true;
        }
      }
    }
    if (changed) dirty_ = true;
  }

  Eigen::Vector3f clipToLocalHorizon(const Eigen::Vector3f &start,
                                     const Eigen::Vector3f &goal) const {
    const float radius = std::max(config_.local_radius_m, config_.resolution_m);
    const Eigen::Vector3f delta = goal - start;
    const float distance = delta.norm();
    Eigen::Vector3f clipped = goal;
    if (distance > radius) clipped = start + (delta / distance) * radius;
    return clampToGrid(clipped);
  }

  bool isOccupied(float x, float y, float z) const {
    int ix = 0;
    int iy = 0;
    int iz = 0;
    if (!worldToIndex(x, y, z, ix, iy, iz)) return false;
    return occupied_[static_cast<std::size_t>(index(ix, iy, iz))] != 0;
  }

  bool isBlocked(float x, float y, float z) {
    inflateIfNeeded();
    int ix = 0;
    int iy = 0;
    int iz = 0;
    if (!worldToIndex(x, y, z, ix, iy, iz)) return true;
    return blocked_[static_cast<std::size_t>(index(ix, iy, iz))] != 0;
  }

  bool worldToIndex(float x, float y, float z, int &ix, int &iy, int &iz) const {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        config_.resolution_m <= 0.0F) {
      return false;
    }
    ix = static_cast<int>(std::floor((x - config_.origin_x_m) / config_.resolution_m));
    iy = static_cast<int>(std::floor((y - config_.origin_y_m) / config_.resolution_m));
    iz = static_cast<int>(std::floor((z - config_.origin_z_m) / config_.resolution_m));
    return inBounds(ix, iy, iz);
  }

  Eigen::Vector3f indexToCenter(int ix, int iy, int iz) const {
    return {
      config_.origin_x_m + (static_cast<float>(ix) + 0.5F) * config_.resolution_m,
      config_.origin_y_m + (static_cast<float>(iy) + 0.5F) * config_.resolution_m,
      config_.origin_z_m + (static_cast<float>(iz) + 0.5F) * config_.resolution_m};
  }

  CoarseGridSearchResult search(const Eigen::Vector3f &start,
                                const Eigen::Vector3f &goal) {
    inflateIfNeeded();
    CoarseGridSearchResult result;
    if (!start.allFinite() || !goal.allFinite()) {
      result.reason = "invalid_pose";
      return result;
    }
    const Eigen::Vector3f clipped = clipToLocalHorizon(start, goal);
    result.clipped_goal = clipped;
    result.clipped = (clipped - goal).norm() > 1.0e-3F;
    int start_ix = 0;
    int start_iy = 0;
    int start_iz = 0;
    int goal_ix = 0;
    int goal_iy = 0;
    int goal_iz = 0;
    if (!worldToIndex(start.x(), start.y(), start.z(), start_ix, start_iy, start_iz)) {
      result.reason = "start_out_of_bounds";
      return result;
    }
    if (!worldToIndex(clipped.x(), clipped.y(), clipped.z(), goal_ix, goal_iy, goal_iz)) {
      result.reason = "goal_out_of_bounds";
      return result;
    }
    const int start_id = index(start_ix, start_iy, start_iz);
    const int goal_id = index(goal_ix, goal_iy, goal_iz);
    if (start_id == goal_id) {
      result.success = true;
      result.reason = "same_cell";
      result.path = {start, clipped};
      result.path_length_m = (clipped - start).norm();
      return result;
    }

    ++search_stamp_;
    if (search_stamp_ == 0) {
      std::fill(stamp_.begin(), stamp_.end(), 0);
      search_stamp_ = 1;
    }
    setGScore(start_id, 0.0F);
    setParent(start_id, -1);
    using QueueItem = std::pair<float, int>;
    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> open;
    open.emplace(heuristic(start_ix, start_iy, start_iz, goal_ix, goal_iy, goal_iz), start_id);

    const Eigen::Vector3f mission = goal - start;
    const float mission_norm = mission.norm();
    const Eigen::Vector3f mission_dir = mission_norm > 1.0e-3F ?
      mission / mission_norm : Eigen::Vector3f(0.0F, 1.0F, 0.0F);
    int expansions = 0;
    while (!open.empty()) {
      const int current = open.top().second;
      open.pop();
      if (current == goal_id) break;
      const float current_g = gScore(current);
      if (current_g == std::numeric_limits<float>::infinity()) continue;
      int cx = 0;
      int cy = 0;
      int cz = 0;
      decode(current, cx, cy, cz);
      ++expansions;
      for (int dz = -1; dz <= 1; ++dz) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dx = -1; dx <= 1; ++dx) {
            if (dx == 0 && dy == 0 && dz == 0) continue;
            const int nx = cx + dx;
            const int ny = cy + dy;
            const int nz = cz + dz;
            if (!inBounds(nx, ny, nz) ||
                !inLocalWindow(nx, ny, nz, start_ix, start_iy, start_iz)) {
              continue;
            }
            const int nid = index(nx, ny, nz);
            const bool start_cell = (nid == start_id);
            if (!start_cell &&
                blocked_[static_cast<std::size_t>(nid)] != 0) {
              continue;
            }
            const Eigen::Vector3f center = indexToCenter(nx, ny, nz);
            const float step = std::hypot(static_cast<float>(dx),
                                          std::hypot(static_cast<float>(dy),
                                                     static_cast<float>(dz))) *
                               config_.resolution_m;
            const Eigen::Vector3f offset = center - start;
            const float progress = offset.dot(mission_dir);
            const float backward = std::max(0.0F, -progress);
            const float lateral = (offset - progress * mission_dir).norm();
            const float candidate = current_g + step + 4.0F * backward + 0.5F * lateral;
            if (candidate + 1.0e-6F >= gScore(nid)) continue;
            setGScore(nid, candidate);
            setParent(nid, current);
            open.emplace(
              candidate + heuristic(nx, ny, nz, goal_ix, goal_iy, goal_iz), nid);
          }
        }
      }
    }

    result.expansions = expansions;
    int end_id = goal_id;
    const bool reached_goal = parentOf(goal_id) >= 0 || start_id == goal_id;
    if (!reached_goal || blocked_[static_cast<std::size_t>(goal_id)] != 0) {
      int best_id = start_id;
      float best_progress = -std::numeric_limits<float>::infinity();
      float best_remaining = std::numeric_limits<float>::infinity();
      const int radius_cells = localRadiusCells();
      for (int iz = std::max(0, start_iz - radius_cells);
           iz <= std::min(config_.depth - 1, start_iz + radius_cells); ++iz) {
        for (int iy = std::max(0, start_iy - radius_cells);
             iy <= std::min(config_.height - 1, start_iy + radius_cells); ++iy) {
          for (int ix = std::max(0, start_ix - radius_cells);
               ix <= std::min(config_.width - 1, start_ix + radius_cells); ++ix) {
            if (!inLocalWindow(ix, iy, iz, start_ix, start_iy, start_iz)) continue;
            const int nid = index(ix, iy, iz);
            if (gScore(nid) == std::numeric_limits<float>::infinity()) continue;
            const Eigen::Vector3f center = indexToCenter(ix, iy, iz);
            const float progress = (center - start).dot(mission_dir);
            const float remaining = (center - clipped).norm();
            if (progress < 0.5F * config_.resolution_m) continue;
            if (progress > best_progress + 1.0e-3F ||
                (std::abs(progress - best_progress) <= 1.0e-3F &&
                 remaining < best_remaining)) {
              best_progress = progress;
              best_remaining = remaining;
              best_id = nid;
            }
          }
        }
      }
      end_id = best_id;
    }

    if (end_id == start_id) {
      result.reason = reached_goal ? "goal_blocked" : "blocked_forward";
      result.path = {start};
      return result;
    }

    std::vector<int> cells;
    for (int cursor = end_id; cursor >= 0; cursor = parentOf(cursor)) {
      cells.push_back(cursor);
      if (cursor == start_id) break;
    }
    std::reverse(cells.begin(), cells.end());
    result.path.reserve(cells.size());
    result.path.push_back(start);
    for (std::size_t i = 1; i < cells.size(); ++i) {
      int ix = 0;
      int iy = 0;
      int iz = 0;
      decode(cells[i], ix, iy, iz);
      result.path.push_back(indexToCenter(ix, iy, iz));
    }
    result.path_length_m = pathLength(result.path);
    const Eigen::Vector3f end_delta = result.path.back() - start;
    if (end_delta.dot(mission_dir) < 0.5F * config_.resolution_m) {
      result.reason = "blocked_forward";
      result.path = {start};
      result.path_length_m = 0.0F;
      return result;
    }
    result.success = true;
    result.reason = (end_id == goal_id) ? "ok" : "best_effort";
    return result;
  }

  Eigen::Vector3f lookahead(const std::vector<Eigen::Vector3f> &path,
                            const Eigen::Vector3f &position,
                            float lookahead_m) const {
    if (path.empty()) return position;
    if (path.size() == 1) return path.front();
    const float horizon = std::max(lookahead_m, 0.0F);
    std::size_t nearest = 0;
    float nearest_distance = std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < path.size(); ++i) {
      const float distance = (path[i] - position).norm();
      if (distance < nearest_distance) {
        nearest_distance = distance;
        nearest = i;
      }
    }
    float remaining = 0.0F;
    for (std::size_t i = nearest; i + 1 < path.size(); ++i) {
      const float segment = (path[i + 1] - path[i]).norm();
      if (remaining + segment >= horizon) {
        const float mix = segment > 1.0e-6F ? (horizon - remaining) / segment : 1.0F;
        return path[i] + mix * (path[i + 1] - path[i]);
      }
      remaining += segment;
    }
    return path.back();
  }

  bool isForwardGoal(const Eigen::Vector3f &start,
                     const Eigen::Vector3f &goal,
                     const Eigen::Vector3f &candidate) const {
    if (!start.allFinite() || !goal.allFinite() || !candidate.allFinite()) {
      return false;
    }
    const Eigen::Vector3f mission = goal - start;
    const float mission_norm = mission.norm();
    const Eigen::Vector3f mission_dir = mission_norm > 1.0e-3F ?
      mission / mission_norm : Eigen::Vector3f(0.0F, 1.0F, 0.0F);
    const Eigen::Vector3f delta = candidate - start;
    const float distance = delta.norm();
    const float progress = delta.dot(mission_dir);
    const float min_progress = std::max(
      0.5F * config_.resolution_m, 0.25F * distance);
    return distance >= 0.5F * config_.resolution_m && progress >= min_progress;
  }

  Eigen::Vector3f progressLookahead(const std::vector<Eigen::Vector3f> &path,
                                    const Eigen::Vector3f &position,
                                    const Eigen::Vector3f &goal,
                                    float lookahead_m) const {
    if (path.empty()) return position;
    const Eigen::Vector3f mission = goal - position;
    const float mission_norm = mission.norm();
    const Eigen::Vector3f mission_dir = mission_norm > 1.0e-3F ?
      mission / mission_norm : Eigen::Vector3f(0.0F, 1.0F, 0.0F);
    const float horizon = std::max(lookahead_m, 0.0F);
    Eigen::Vector3f best = path.front();
    float best_progress = -std::numeric_limits<float>::infinity();
    float travelled = 0.0F;
    for (std::size_t i = 0; i < path.size(); ++i) {
      if (i > 0) travelled += (path[i] - path[i - 1]).norm();
      const float progress = (path[i] - position).dot(mission_dir);
      if (progress > best_progress) {
        best_progress = progress;
        best = path[i];
      }
      if (travelled >= horizon) break;
    }
    return best;
  }

  std::vector<int8_t> occupancyGrid() {
    inflateIfNeeded();
    std::vector<int8_t> data(
      static_cast<std::size_t>(config_.width) * static_cast<std::size_t>(config_.height), 0);
    for (int iz = 0; iz < config_.depth; ++iz) {
      for (int iy = 0; iy < config_.height; ++iy) {
        for (int ix = 0; ix < config_.width; ++ix) {
          const std::size_t voxel = static_cast<std::size_t>(index(ix, iy, iz));
          const std::size_t cell = static_cast<std::size_t>(iy * config_.width + ix);
          if (occupied_[voxel] != 0) data[cell] = 100;
          else if (blocked_[voxel] != 0 && data[cell] < 80) data[cell] = 80;
        }
      }
    }
    return data;
  }

  std::vector<Eigen::Vector3f> occupiedCenters() const {
    std::vector<Eigen::Vector3f> centers;
    centers.reserve(occupied_count_);
    for (int iz = 0; iz < config_.depth; ++iz) {
      for (int iy = 0; iy < config_.height; ++iy) {
        for (int ix = 0; ix < config_.width; ++ix) {
          if (occupied_[static_cast<std::size_t>(index(ix, iy, iz))] == 0) continue;
          centers.push_back(indexToCenter(ix, iy, iz));
        }
      }
    }
    return centers;
  }

 private:
  static CoarseGridAstarConfig sanitize(CoarseGridAstarConfig config) {
    config.resolution_m = std::max(config.resolution_m, 0.05F);
    config.inflate_radius_m = std::max(config.inflate_radius_m, 0.0F);
    config.width = std::max(config.width, 2);
    config.height = std::max(config.height, 2);
    config.depth = std::max(config.depth, 2);
    config.local_radius_m = std::max(config.local_radius_m, config.resolution_m);
    return config;
  }

  std::size_t cellCount() const {
    return static_cast<std::size_t>(config_.width) *
           static_cast<std::size_t>(config_.height) *
           static_cast<std::size_t>(config_.depth);
  }

  bool inBounds(int ix, int iy, int iz) const {
    return ix >= 0 && iy >= 0 && iz >= 0 &&
           ix < config_.width && iy < config_.height && iz < config_.depth;
  }

  int localRadiusCells() const {
    return std::max(1, static_cast<int>(std::ceil(
      config_.local_radius_m / config_.resolution_m)));
  }

  bool inLocalWindow(int ix, int iy, int iz, int start_ix, int start_iy, int start_iz) const {
    const int radius_cells = localRadiusCells();
    return std::abs(ix - start_ix) <= radius_cells &&
           std::abs(iy - start_iy) <= radius_cells &&
           std::abs(iz - start_iz) <= radius_cells;
  }

  int index(int ix, int iy, int iz) const {
    return (iz * config_.height + iy) * config_.width + ix;
  }

  void decode(int id, int &ix, int &iy, int &iz) const {
    const int layer = config_.width * config_.height;
    iz = id / layer;
    const int rem = id - iz * layer;
    iy = rem / config_.width;
    ix = rem - iy * config_.width;
  }

  float heuristic(int ix, int iy, int iz, int gx, int gy, int gz) const {
    return std::hypot(static_cast<float>(ix - gx),
                      std::hypot(static_cast<float>(iy - gy),
                                 static_cast<float>(iz - gz))) *
           config_.resolution_m;
  }

  Eigen::Vector3f clampToGrid(const Eigen::Vector3f &point) const {
    const float half = 0.5F * config_.resolution_m;
    const float min_x = config_.origin_x_m + half;
    const float min_y = config_.origin_y_m + half;
    const float min_z = config_.origin_z_m + half;
    const float max_x =
      config_.origin_x_m + static_cast<float>(config_.width) * config_.resolution_m - half;
    const float max_y =
      config_.origin_y_m + static_cast<float>(config_.height) * config_.resolution_m - half;
    const float max_z =
      config_.origin_z_m + static_cast<float>(config_.depth) * config_.resolution_m - half;
    return {
      std::clamp(point.x(), min_x, max_x),
      std::clamp(point.y(), min_y, max_y),
      std::clamp(point.z(), min_z, max_z)};
  }

  static float pathLength(const std::vector<Eigen::Vector3f> &path) {
    float length = 0.0F;
    for (std::size_t i = 1; i < path.size(); ++i) {
      length += (path[i] - path[i - 1]).norm();
    }
    return length;
  }

  float gScore(int id) const {
    const std::size_t idx = static_cast<std::size_t>(id);
    if (stamp_[idx] != search_stamp_) return std::numeric_limits<float>::infinity();
    return g_score_[idx];
  }

  int parentOf(int id) const {
    const std::size_t idx = static_cast<std::size_t>(id);
    if (stamp_[idx] != search_stamp_) return -1;
    return parent_[idx];
  }

  void setGScore(int id, float value) {
    const std::size_t idx = static_cast<std::size_t>(id);
    g_score_[idx] = value;
    stamp_[idx] = search_stamp_;
  }

  void setParent(int id, int parent) {
    const std::size_t idx = static_cast<std::size_t>(id);
    parent_[idx] = parent;
    stamp_[idx] = search_stamp_;
  }

  void inflateIfNeeded() {
    if (!dirty_) return;
    std::fill(blocked_.begin(), blocked_.end(), 0);
    blocked_count_ = 0;
    const float extra = 0.5F * config_.resolution_m * std::sqrt(3.0F);
    const float radius = config_.inflate_radius_m + extra;
    const int radius_cells = static_cast<int>(std::ceil(radius / config_.resolution_m));
    for (int iz = 0; iz < config_.depth; ++iz) {
      for (int iy = 0; iy < config_.height; ++iy) {
        for (int ix = 0; ix < config_.width; ++ix) {
          if (occupied_[static_cast<std::size_t>(index(ix, iy, iz))] == 0) continue;
          for (int dz = -radius_cells; dz <= radius_cells; ++dz) {
            for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
              for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
                const int nx = ix + dx;
                const int ny = iy + dy;
                const int nz = iz + dz;
                if (!inBounds(nx, ny, nz)) continue;
                const float distance =
                  std::hypot(static_cast<float>(dx),
                             std::hypot(static_cast<float>(dy),
                                        static_cast<float>(dz))) *
                  config_.resolution_m;
                if (distance > radius + 1.0e-6F) continue;
                auto &cell = blocked_[static_cast<std::size_t>(index(nx, ny, nz))];
                if (cell == 0) {
                  cell = 1;
                  ++blocked_count_;
                }
              }
            }
          }
        }
      }
    }
    dirty_ = false;
  }

  CoarseGridAstarConfig config_;
  std::vector<uint8_t> occupied_;
  std::vector<uint8_t> blocked_;
  std::vector<float> g_score_;
  std::vector<int> parent_;
  std::vector<uint32_t> stamp_;
  std::size_t occupied_count_ = 0;
  std::size_t blocked_count_ = 0;
  uint32_t search_stamp_ = 0;
  bool dirty_ = false;
};

}  // namespace scalenav
