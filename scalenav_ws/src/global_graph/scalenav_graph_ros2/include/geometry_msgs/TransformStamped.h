#pragma once
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <geometry_msgs/msg/vector3.hpp>
namespace geometry_msgs {
struct TransformStamped : msg::TransformStamped {
  using Ptr = std::shared_ptr<TransformStamped>;
};
using TransformStampedPtr = TransformStamped::Ptr;
using Point = msg::Point;
using Vector3 = msg::Vector3;
using PoseStampedConstPtr = msg::PoseStamped::ConstSharedPtr;
}
