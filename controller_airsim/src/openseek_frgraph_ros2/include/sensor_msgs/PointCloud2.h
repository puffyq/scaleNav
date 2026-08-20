#pragma once
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/image.hpp>
namespace sensor_msgs {
using PointCloud2 = msg::PointCloud2;
using PointCloud2ConstPtr = msg::PointCloud2::ConstSharedPtr;
using Image = msg::Image;
}
