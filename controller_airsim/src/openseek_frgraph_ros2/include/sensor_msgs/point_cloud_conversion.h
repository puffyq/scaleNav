#pragma once

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <std_msgs/msg/header.hpp>
#include <vector>

namespace sensor_msgs {
struct PointCloud {
  std_msgs::msg::Header header;
  std::vector<geometry_msgs::msg::Point32> points;
};

inline void convertPointCloud2ToPointCloud(
    const msg::PointCloud2 &input, PointCloud &output) {
  output.header = input.header;
  output.points.clear();
  try {
    PointCloud2ConstIterator<float> x(input, "x");
    PointCloud2ConstIterator<float> y(input, "y");
    PointCloud2ConstIterator<float> z(input, "z");
    for (std::size_t i = 0; i < input.width * input.height;
         ++i, ++x, ++y, ++z) {
      geometry_msgs::msg::Point32 point;
      point.x = *x;
      point.y = *y;
      point.z = *z;
      output.points.push_back(point);
    }
  } catch (...) {
    output.points.clear();
  }
}
}
