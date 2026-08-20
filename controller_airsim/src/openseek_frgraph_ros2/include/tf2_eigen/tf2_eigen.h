#pragma once

#include <Eigen/Geometry>
#include <geometry_msgs/TransformStamped.h>

namespace tf2 {

inline Eigen::Affine3d transformToEigen(
    const geometry_msgs::TransformStamped &message) {
  const auto &translation = message.transform.translation;
  const auto &rotation = message.transform.rotation;
  Eigen::Quaterniond quaternion(rotation.w, rotation.x, rotation.y, rotation.z);
  if (quaternion.norm() < 1e-12) quaternion = Eigen::Quaterniond::Identity();
  quaternion.normalize();
  Eigen::Affine3d result = Eigen::Affine3d::Identity();
  result.linear() = quaternion.toRotationMatrix();
  result.translation() = Eigen::Vector3d(
      translation.x, translation.y, translation.z);
  return result;
}

}  // namespace tf2
