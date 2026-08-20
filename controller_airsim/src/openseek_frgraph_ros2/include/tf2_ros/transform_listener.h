#pragma once

#include <geometry_msgs/TransformStamped.h>
#include <stdexcept>
#include <string>

#include <ros/ros.h>

namespace tf2 {
class TransformException : public std::runtime_error {
 public:
  using std::runtime_error::runtime_error;
};
}

namespace tf2_ros {

class Buffer {
 public:
  Buffer() = default;

  geometry_msgs::TransformStamped lookupTransform(
      const std::string &target_frame,
      const std::string &source_frame,
      const ros::Time &,
      const ros::Duration & = ros::Duration()) const {
    geometry_msgs::TransformStamped transform;
    transform.header.frame_id = target_frame;
    transform.child_frame_id = source_frame;
    transform.transform.rotation.w = 1.0;
    return transform;
  }
};

class TransformListener {
 public:
  explicit TransformListener(Buffer &) {}
};

}  // namespace tf2_ros
