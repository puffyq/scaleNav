#pragma once

#include <Eigen/Dense>
#include <decomp_geometry/ellipsoid.h>
#include <decomp_geometry/polyhedron.h>
#include <decomp_ros_msgs/EllipsoidArray.h>
#include <decomp_ros_msgs/PolyhedronArray.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <vector>

namespace DecompROS {

inline vec_Vec3f cloud_to_vec(const sensor_msgs::PointCloud &cloud) {
  vec_Vec3f result;
  result.reserve(cloud.points.size());
  for (const auto &point : cloud.points) {
    result.emplace_back(point.x, point.y, point.z);
  }
  return result;
}

template <typename Polyhedra>
inline decomp_ros_msgs::PolyhedronArray polyhedron_array_to_ros(
    const Polyhedra &) {
  return {};
}

template <typename Ellipsoids>
inline decomp_ros_msgs::EllipsoidArray ellipsoid_array_to_ros(
    const Ellipsoids &) {
  return {};
}

}  // namespace DecompROS
