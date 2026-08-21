#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <chrono>
#include <memory>
#include <limits>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "openseek_frgraph_ros2/depth_projection.hpp"

namespace openseek_frgraph_ros2 {

class DepthPlanarToPointCloudNode final : public rclcpp::Node {
 public:
  DepthPlanarToPointCloudNode() : Node("depth_planar_to_pointcloud") {
    depth_topic_ = declare_parameter<std::string>(
        "depth_topic", "/camera/depth/image");
    camera_info_topic_ = declare_parameter<std::string>(
        "camera_info_topic", "/camera/depth/camera_info");
    pointcloud_topic_ = declare_parameter<std::string>(
        "pointcloud_topic", "/frgraph/points");
    free_ray_topic_ = declare_parameter<std::string>(
        "free_ray_topic", "/frgraph/free_rays");
    output_frame_ = declare_parameter<std::string>("output_frame", "base_link");
    max_range_m_ = declare_parameter<double>("max_range_m", 20.0);
    free_ray_pixel_stride_ = static_cast<int>(std::max<std::int64_t>(
        1, declare_parameter<int>("free_ray_pixel_stride", 4)));
    default_fx_ = declare_parameter<double>("fx", 0.0);
    default_fy_ = declare_parameter<double>("fy", 0.0);
    default_cx_ = declare_parameter<double>("cx", 0.0);
    default_cy_ = declare_parameter<double>("cy", 0.0);
    camera_tx_ = declare_parameter<double>("camera_translation_flu.x", 0.0);
    camera_ty_ = declare_parameter<double>("camera_translation_flu.y", 0.0);
    camera_tz_ = declare_parameter<double>("camera_translation_flu.z", 0.0);

    camera_info_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
        camera_info_topic_, rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr message) {
          if (message->k[0] > 0.0 && message->k[4] > 0.0) {
            camera_.fx = message->k[0];
            camera_.fy = message->k[4];
            camera_.cx = message->k[2];
            camera_.cy = message->k[5];
            have_camera_info_ = true;
          }
        });
    pointcloud_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        pointcloud_topic_, rclcpp::SensorDataQoS());
    free_ray_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        free_ray_topic_, rclcpp::SensorDataQoS());
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(
        depth_topic_, rclcpp::SensorDataQoS(),
        std::bind(&DepthPlanarToPointCloudNode::on_depth, this,
                  std::placeholders::_1));

    RCLCPP_INFO(get_logger(),
                "DepthPlanar adapter: %s -> %s + %s, frame=%s, max_range=%.2fm, "
                "camera_translation_flu=(%.2f,%.2f,%.2f)",
                depth_topic_.c_str(), pointcloud_topic_.c_str(), free_ray_topic_.c_str(),
                output_frame_.c_str(), max_range_m_, camera_tx_, camera_ty_,
                camera_tz_);
  }

 private:
  void on_depth(sensor_msgs::msg::Image::ConstSharedPtr image) {
    const auto t_start = std::chrono::steady_clock::now();
    if (image->encoding != "32FC1" || image->is_bigendian) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
                           "Expected little-endian 32FC1 DepthPlanar, got %s",
                           image->encoding.c_str());
      return;
    }
    if (image->step < image->width * sizeof(float) ||
        image->data.size() < image->step * image->height) {
      RCLCPP_WARN(get_logger(), "Invalid DepthPlanar buffer dimensions");
      return;
    }

    CameraModel model = camera_;
    if (!have_camera_info_) {
      if (default_fx_ <= 0.0 || default_fy_ <= 0.0) {
        RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 5000,
            "Waiting for CameraInfo before projecting DepthPlanar");
        return;
      }
      model.fx = default_fx_;
      model.fy = default_fy_;
      model.cx = default_cx_ > 0.0 ? default_cx_ : (image->width - 1) * 0.5;
      model.cy = default_cy_ > 0.0 ? default_cy_ : (image->height - 1) * 0.5;
    }
    model.body_from_camera_tx = camera_tx_;
    model.body_from_camera_ty = camera_ty_;
    model.body_from_camera_tz = camera_tz_;

    const auto *depth = reinterpret_cast<const float *>(image->data.data());
    std::vector<PointXYZ> points;
    std::vector<PointXYZ> free_rays;
    try {
      points = depth_planar_to_flu(depth, image->width, image->height,
                                   image->step, model, max_range_m_);
      free_rays = depth_planar_far_plane_to_flu(
          depth, image->width, image->height, image->step, model, max_range_m_,
          static_cast<std::size_t>(free_ray_pixel_stride_));
    } catch (const std::exception &error) {
      RCLCPP_WARN(get_logger(), "DepthPlanar conversion failed: %s", error.what());
      return;
    }

    sensor_msgs::msg::PointCloud2 output;
    output.header = image->header;
    output.header.frame_id = output_frame_;
    output.height = 1;
    output.width = static_cast<uint32_t>(points.size());
    output.is_bigendian = false;
    output.is_dense = false;
    sensor_msgs::PointCloud2Modifier modifier(output);
    modifier.setPointCloud2FieldsByString(1, "xyz");
    modifier.resize(points.size());
    sensor_msgs::PointCloud2Iterator<float> iter_x(output, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(output, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(output, "z");
    for (const auto &point : points) {
      *iter_x = point.x;
      *iter_y = point.y;
      *iter_z = point.z;
      ++iter_x;
      ++iter_y;
      ++iter_z;
    }
    pointcloud_pub_->publish(std::move(output));

    sensor_msgs::msg::PointCloud2 free_output;
    free_output.header = image->header;
    free_output.header.frame_id = output_frame_;
    free_output.height = 1;
    free_output.width = static_cast<uint32_t>(free_rays.size());
    free_output.is_bigendian = false;
    free_output.is_dense = false;
    sensor_msgs::PointCloud2Modifier free_modifier(free_output);
    free_modifier.setPointCloud2FieldsByString(1, "xyz");
    free_modifier.resize(free_rays.size());
    sensor_msgs::PointCloud2Iterator<float> free_x(free_output, "x");
    sensor_msgs::PointCloud2Iterator<float> free_y(free_output, "y");
    sensor_msgs::PointCloud2Iterator<float> free_z(free_output, "z");
    for (const auto &point : free_rays) {
      *free_x = point.x;
      *free_y = point.y;
      *free_z = point.z;
      ++free_x;
      ++free_y;
      ++free_z;
    }
    free_ray_pub_->publish(std::move(free_output));

    const auto t_end = std::chrono::steady_clock::now();
    const double elapsed_ms = std::chrono::duration<double, std::milli>(
        t_end - t_start).count();
    float free_y_min = std::numeric_limits<float>::quiet_NaN();
    float free_y_max = std::numeric_limits<float>::quiet_NaN();
    if (!free_rays.empty()) {
      const auto bounds = std::minmax_element(
          free_rays.begin(), free_rays.end(),
          [](const PointXYZ &left, const PointXYZ &right) {
            return left.y < right.y;
          });
      free_y_min = bounds.first->y;
      free_y_max = bounds.second->y;
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[FRGraph timing] DepthPlanar->PointCloud2: %.3f ms, input=%ux%u, "
        "points=%zu free_rays=%zu free_y=[%.2f,%.2f]",
        elapsed_ms, image->width, image->height, points.size(), free_rays.size(),
        free_y_min, free_y_max);
  }

  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string pointcloud_topic_;
  std::string free_ray_topic_;
  std::string output_frame_;
  double max_range_m_ = 20.0;
  int free_ray_pixel_stride_ = 4;
  double default_fx_ = 0.0;
  double default_fy_ = 0.0;
  double default_cx_ = 0.0;
  double default_cy_ = 0.0;
  double camera_tx_ = 0.0;
  double camera_ty_ = 0.0;
  double camera_tz_ = 0.0;
  CameraModel camera_;
  bool have_camera_info_ = false;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr free_ray_pub_;
};

}  // namespace openseek_frgraph_ros2

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<openseek_frgraph_ros2::DepthPlanarToPointCloudNode>());
  rclcpp::shutdown();
  return 0;
}
