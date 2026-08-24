#pragma once
#include <memory>
#include <geometry_msgs/msg/pose_stamped.hpp>
namespace geometry_msgs {
using PoseStamped = geometry_msgs::msg::PoseStamped;
using PoseStampedConstPtr = std::shared_ptr<const geometry_msgs::msg::PoseStamped>;
}
