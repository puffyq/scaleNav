#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <vector>

namespace openseek_frgraph_ros2 {

struct CameraModel {
  double fx = 0.0;
  double fy = 0.0;
  double cx = 0.0;
  double cy = 0.0;
  double body_from_camera_tx = 0.0;
  double body_from_camera_ty = 0.0;
  double body_from_camera_tz = 0.0;
};

struct PointXYZ {
  float x = 0.0F;
  float y = 0.0F;
  float z = 0.0F;
};

inline PointXYZ project_depth_sample(std::size_t u, std::size_t v, double depth,
                                     const CameraModel &camera) {
  const double x_optical = (static_cast<double>(u) - camera.cx) * depth / camera.fx;
  const double y_optical = (static_cast<double>(v) - camera.cy) * depth / camera.fy;
  return PointXYZ{
      static_cast<float>(depth + camera.body_from_camera_tx),
      static_cast<float>(-x_optical + camera.body_from_camera_ty),
      static_cast<float>(-y_optical + camera.body_from_camera_tz)};
}

// Converts optical-frame Z depth into the FRGraph scan convention: forward,
// left, up. Invalid, zero, and beyond max range samples are omitted.
inline std::vector<PointXYZ> depth_planar_to_flu(
    const float *depth,
    std::size_t width,
    std::size_t height,
    std::size_t row_step_bytes,
    const CameraModel &camera,
    double max_range_m) {
  if (depth == nullptr || width == 0 || height == 0 || camera.fx <= 0.0 ||
      camera.fy <= 0.0 || max_range_m <= 0.0) {
    throw std::invalid_argument("invalid DepthPlanar projection input");
  }

  const std::size_t row_stride = row_step_bytes / sizeof(float);
  if (row_stride < width) {
    throw std::invalid_argument("DepthPlanar row step is smaller than width");
  }

  std::vector<PointXYZ> points;
  points.reserve(width * height);
  for (std::size_t v = 0; v < height; ++v) {
    const float *row = depth + v * row_stride;
    for (std::size_t u = 0; u < width; ++u) {
      const double z_optical = static_cast<double>(row[u]);
      // A DepthPlanar value at the configured maximum is a clipped/unknown
      // return, not an occupied surface. Do not create a fake range shell.
      if (!std::isfinite(z_optical) || z_optical <= 0.0 ||
          z_optical >= max_range_m - 1e-4) {
        continue;
      }

      // AirSim publishes optical-frame Z depth. The static camera transform
      // is optical -> body FLU: (x,y,z)_body = (z,-x,-y)_optical.
      points.push_back(project_depth_sample(u, v, z_optical, camera));
    }
  }
  return points;
}

// Returns endpoints for pixels that reached the sensor's far plane. They are
// free-ray evidence, not occupied returns, and must be consumed separately
// from the obstacle point cloud.
inline std::vector<PointXYZ> depth_planar_far_plane_to_flu(
    const float *depth,
    std::size_t width,
    std::size_t height,
    std::size_t row_step_bytes,
    const CameraModel &camera,
    double max_range_m,
    std::size_t sample_stride = 1) {
  if (depth == nullptr || width == 0 || height == 0 || camera.fx <= 0.0 ||
      camera.fy <= 0.0 || max_range_m <= 0.0) {
    throw std::invalid_argument("invalid DepthPlanar projection input");
  }
  const std::size_t row_stride = row_step_bytes / sizeof(float);
  if (row_stride < width) {
    throw std::invalid_argument("DepthPlanar row step is smaller than width");
  }
  std::vector<PointXYZ> rays;
  rays.reserve(width * height);
  sample_stride = std::max<std::size_t>(sample_stride, 1);
  for (std::size_t v = 0; v < height; v += sample_stride) {
    const float *row = depth + v * row_stride;
    for (std::size_t u = 0; u < width; u += sample_stride) {
      const double z_optical = static_cast<double>(row[u]);
      if (!std::isfinite(z_optical) ||
          z_optical < max_range_m - 1e-4 || z_optical <= 0.0) {
        continue;
      }
      rays.push_back(project_depth_sample(u, v, max_range_m, camera));
    }
  }
  return rays;
}

}  // namespace openseek_frgraph_ros2
