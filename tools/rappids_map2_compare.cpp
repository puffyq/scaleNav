#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>

#include <opencv2/imgcodecs.hpp>

#include "CommonMath/Vec3.hpp"
#include "RapidQuadcopterTrajectories/RapidTrajectoryGenerator.hpp"
#include "RectangularPyramidPlanner/DepthImagePlanner.hpp"

using CommonMath::Vec3;
using RapidQuadrocopterTrajectoryGenerator::RapidTrajectoryGenerator;
using RectangularPyramidPlanner::DepthImagePlanner;

struct OneTrajectory {
  Vec3 goal;
  bool emitted = false;
};

int EmitOneTrajectory(void* object, RapidTrajectoryGenerator& output) {
  auto* candidate = static_cast<OneTrajectory*>(object);
  if (candidate->emitted) {
    return -1;
  }
  RapidTrajectoryGenerator generated(
      Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0),
      Vec3(0.0, 9.81, 0.0));
  generated.SetGoalPosition(candidate->goal);
  generated.SetGoalVelocity(Vec3(0.0, 0.0, 0.0));
  generated.SetGoalAcceleration(Vec3(0.0, 0.0, 0.0));
  generated.Generate(2.0);
  output = generated;
  candidate->emitted = true;
  return 0;
}

double ZeroCost(
    void*, RapidTrajectoryGenerator&) {
  return 0.0;
}

Vec3 FluToRappids(Vec3 body_flu) {
  // RAPPIDS camera coordinates are right, down, forward. OpenSeek Graph uses
  // forward, left, up (FLU).
  return Vec3(-body_flu.y, -body_flu.z, body_flu.x);
}

// OpenSeek's optimistic rule: only a measured obstacle can invalidate an edge.
// Out-of-FOV, invalid, and max-range pixels are unknown and therefore pass.
bool KnownDepthSegmentClear(
    const cv::Mat& depth_mm, Vec3 start_flu, Vec3 goal_flu) {
  constexpr double fx = 80.0;
  constexpr double fy = 80.0;
  constexpr double cx = 79.5;
  constexpr double cy = 47.5;
  const Vec3 segment = goal_flu - start_flu;
  const double length_squared = segment.GetNorm2Squared();
  double minimum_clearance = INFINITY;
  int closest_u = -1;
  int closest_v = -1;
  for (int v = 0; v < depth_mm.rows; ++v) {
    for (int u = 0; u < depth_mm.cols; ++u) {
      const double observed = depth_mm.at<uint16_t>(v, u) * 0.001;
      if (!(observed > 0.0) || observed >= 19.99) {
        continue;
      }
      const Vec3 obstacle_flu(
          observed,
          -(u - cx) * observed / fx,
          -(v - cy) * observed / fy);
      const Vec3 relative = obstacle_flu - start_flu;
      const double progress = std::max(
          0.0, std::min(1.0, relative.Dot(segment) / length_squared));
      const double distance =
          (obstacle_flu - (start_flu + segment * progress)).GetNorm2();
      // Treat each organized depth sample as the surface patch represented by
      // one pixel. This rejects actual line/surface intersections without
      // confusing camera-ray occlusion with collision.
      const double pixel_half_diagonal = 0.5 * observed * std::sqrt(
          1.0 / (fx * fx) + 1.0 / (fy * fy));
      const double clearance = distance - pixel_half_diagonal;
      if (clearance < minimum_clearance) {
        minimum_clearance = clearance;
        closest_u = u;
        closest_v = v;
      }
    }
  }
  std::cerr << "known_depth_surface_clearance=" << minimum_clearance
            << " closest_pixel=" << closest_u << "," << closest_v << "\n";
  return minimum_clearance > 0.0;
}

bool CheckCandidate(DepthImagePlanner& planner, Vec3 goal_flu) {
  OneTrajectory candidate;
  candidate.goal = FluToRappids(goal_flu);
  candidate.emitted = false;
  RapidTrajectoryGenerator initial(
      Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0), Vec3(0.0, 0.0, 0.0),
      Vec3(0.0, 9.81, 0.0));
  initial.SetGoalPosition(Vec3(0.0, 0.0, 0.0));
  initial.SetGoalVelocity(Vec3(0.0, 0.0, 0.0));
  initial.SetGoalAcceleration(Vec3(0.0, 0.0, 0.0));
  initial.Generate(2.0);
  RapidTrajectoryGenerator output = initial;
  return planner.FindLowestCostTrajectory(
      output, 0.5, &candidate, &ZeroCost, &candidate, &EmitOneTrajectory);
}

int main(int argc, char** argv) {
  if (argc < 2 || argc > 5) {
    std::cerr << "usage: rappids_map2_compare DEPTH_EXR [physical_radius] [planning_radius] [min_check_distance]\n";
    return 2;
  }
  cv::Mat depth = cv::imread(argv[1], cv::IMREAD_ANYCOLOR | cv::IMREAD_ANYDEPTH);
  if (depth.empty()) {
    std::cerr << "failed to read depth image: " << argv[1] << "\n";
    return 3;
  }
  if (depth.channels() != 1) {
    depth = depth.reshape(1);
  }
  cv::Mat depth_mm(depth.size(), CV_16UC1);
  for (int row = 0; row < depth.rows; ++row) {
    for (int column = 0; column < depth.cols; ++column) {
      const float value = depth.at<float>(row, column);
      const float meters = std::isfinite(value) ? std::max(0.0f, value) : 20.0f;
      depth_mm.at<uint16_t>(row, column) = static_cast<uint16_t>(
          std::min(65535.0f, std::round(meters * 1000.0f)));
    }
  }

  const double physical_radius = argc >= 3 ? std::stod(argv[2]) : 0.6;
  const double planning_radius = argc >= 4 ? std::stod(argv[3]) : 0.75;
  const double min_check_distance = argc >= 5 ? std::stod(argv[4]) : 1.0;

  // Map2 data is 160x96, horizontal FOV 90 deg. RAPPIDS uses the planar
  // camera depth directly as its z coordinate.
  DepthImagePlanner planner(
      depth_mm, 0.001, 80.0, 79.5, 47.5, physical_radius, planning_radius,
      min_check_distance);
  planner.SetDynamicFeasiblityParameters(0.0, 100.0, 100.0, 0.001);

  const Vec3 node2(4.095760221, -2.867882181, 0.0);
  const Vec3 node5(4.768584753, 1.503528998, 0.0);
  const Vec3 node6(4.095760221, 2.867882181, 0.0);
  // RAPPIDS checks trajectories that start at the current camera origin.
  // Candidate-to-goal edges require a new depth frame after reaching the
  // candidate and must not be tested by subtracting the two endpoints here.
  const bool current_to_2 = CheckCandidate(planner, node2);
  const int pyramids_after_2 = planner.GetNumPyramids();
  const bool current_to_5 = CheckCandidate(planner, node5);
  const int pyramids_after_5 = planner.GetNumPyramids();
  const bool current_to_6 = CheckCandidate(planner, node6);
  const int pyramids_after_6 = planner.GetNumPyramids();
  const bool known_2_to_goal = KnownDepthSegmentClear(
      depth_mm, node2, Vec3(20.0, 0.0, 0.0));
  const bool known_6_to_goal = KnownDepthSegmentClear(
      depth_mm, node6, Vec3(20.0, 0.0, 0.0));

  std::cout << "depth=" << depth.cols << "x" << depth.rows
            << " physical_radius=" << physical_radius
            << " planning_radius=" << planning_radius
            << " min_check_distance=" << min_check_distance << "\n"
            << "edge_current_to_2=" << (current_to_2 ? "FREE" : "COLLISION")
            << " pyramids=" << pyramids_after_2
            << " checks=" << planner.GetNumCollisionChecks() << "\n"
            << "edge_current_to_5=" << (current_to_5 ? "FREE" : "COLLISION")
            << " pyramids=" << pyramids_after_5
            << " checks=" << planner.GetNumCollisionChecks() << "\n"
            << "edge_current_to_6=" << (current_to_6 ? "FREE" : "COLLISION")
            << " pyramids=" << pyramids_after_6
            << " checks=" << planner.GetNumCollisionChecks() << "\n"
            << "edge_2_to_goal=DEFERRED_NEW_DEPTH_FRAME\n"
            << "edge_5_to_goal=DEFERRED_NEW_DEPTH_FRAME\n"
            << "edge_6_to_goal=DEFERRED_NEW_DEPTH_FRAME\n"
            << "known_depth_edge_2_to_goal="
            << (known_2_to_goal ? "NO_VISIBLE_COLLISION" : "VISIBLE_COLLISION") << "\n"
            << "known_depth_edge_6_to_goal="
            << (known_6_to_goal ? "NO_VISIBLE_COLLISION" : "VISIBLE_COLLISION") << "\n"
            << "path_start_2_goal="
            << (current_to_2 ? "START_TO_2_FREE," : "START_TO_2_COLLISION,")
            << (known_2_to_goal ? "2_TO_GOAL_NO_VISIBLE_COLLISION" :
                "2_TO_GOAL_VISIBLE_COLLISION") << "\n"
            << "path_start_6_goal="
            << (current_to_6 ? "START_TO_6_FREE," : "START_TO_6_COLLISION,")
            << (known_6_to_goal ? "6_TO_GOAL_NO_VISIBLE_COLLISION" :
                "6_TO_GOAL_VISIBLE_COLLISION") << "\n";
  return 0;
}
