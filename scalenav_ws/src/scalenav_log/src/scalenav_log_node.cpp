#include "scalenav_log/sliding_log_store.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <memory>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace fs = std::filesystem;
using scalenav_log::SlidingLogStore;
using scalenav_log::jsonNumber;
using scalenav_log::jsonQuote;

namespace {

std::int64_t stampNs(const builtin_interfaces::msg::Time &stamp)
{
  return static_cast<std::int64_t>(stamp.sec) * 1000000000LL + stamp.nanosec;
}

std::string expandUser(const std::string &value)
{
  if (value.rfind("~/", 0) != 0) return value;
  const char *home = std::getenv("HOME");
  return std::string(home ? home : ".") + value.substr(1);
}

template<typename T>
std::string array3(const T &value)
{
  return "[" + jsonNumber(value.x) + "," + jsonNumber(value.y) + "," + jsonNumber(value.z) + "]";
}

std::string quaternion(const geometry_msgs::msg::Quaternion &value)
{
  return "[" + jsonNumber(value.x) + "," + jsonNumber(value.y) + "," +
    jsonNumber(value.z) + "," + jsonNumber(value.w) + "]";
}

std::vector<std::uint8_t> bytesFromString(const std::string &value)
{
  return std::vector<std::uint8_t>(value.begin(), value.end());
}

template<std::size_t Bytes>
std::uint64_t readInteger(const std::uint8_t *data, const bool big_endian)
{
  static_assert(Bytes > 0 && Bytes <= sizeof(std::uint64_t));
  std::uint64_t value = 0;
  if (big_endian) {
    for (std::size_t index = 0; index < Bytes; ++index)
      value = (value << 8U) | data[index];
  } else {
    for (std::size_t index = 0; index < Bytes; ++index)
      value |= static_cast<std::uint64_t>(data[index]) << (index * 8U);
  }
  return value;
}

std::uint16_t readU16(const std::uint8_t *data, const bool big_endian)
{
  return static_cast<std::uint16_t>(readInteger<2>(data, big_endian));
}

float readFloat(const std::uint8_t *data, const bool big_endian)
{
  const auto bits = static_cast<std::uint32_t>(readInteger<4>(data, big_endian));
  float result = 0.0F;
  std::memcpy(&result, &bits, sizeof(result));
  return result;
}

double readDouble(const std::uint8_t *data, const bool big_endian)
{
  const auto bits = readInteger<8>(data, big_endian);
  double result = 0.0;
  std::memcpy(&result, &bits, sizeof(result));
  return result;
}

struct FieldLocation {
  int offset = -1;
  int datatype = 0;
  int count = 0;
};

FieldLocation findField(const sensor_msgs::msg::PointCloud2 &message, const std::string &name)
{
  for (const auto &field : message.fields) {
    if (field.name == name) return {static_cast<int>(field.offset), field.datatype, static_cast<int>(field.count)};
  }
  return {};
}

bool readFieldFloat(const sensor_msgs::msg::PointCloud2 &message, const std::uint8_t *point,
                   const FieldLocation &field, float &result)
{
  if (field.offset < 0 || field.count < 1 || field.datatype == 0) return false;
  std::size_t field_size = 0;
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::INT8:
    case sensor_msgs::msg::PointField::UINT8: field_size = 1; break;
    case sensor_msgs::msg::PointField::INT16:
    case sensor_msgs::msg::PointField::UINT16: field_size = 2; break;
    case sensor_msgs::msg::PointField::INT32:
    case sensor_msgs::msg::PointField::UINT32:
    case sensor_msgs::msg::PointField::FLOAT32: field_size = 4; break;
    case sensor_msgs::msg::PointField::FLOAT64: field_size = 8; break;
    default: return false;
  }
  const auto offset = static_cast<std::size_t>(field.offset);
  if (offset > message.point_step || field_size > message.point_step - offset) return false;
  const auto *data = point + field.offset;
  switch (field.datatype) {
    case sensor_msgs::msg::PointField::INT8:
      result = static_cast<float>(static_cast<std::int8_t>(*data));
      return true;
    case sensor_msgs::msg::PointField::UINT8:
      result = static_cast<float>(*data);
      return true;
    case sensor_msgs::msg::PointField::INT16:
      result = static_cast<float>(static_cast<std::int16_t>(readInteger<2>(data, message.is_bigendian)));
      return true;
    case sensor_msgs::msg::PointField::FLOAT32:
      result = readFloat(data, message.is_bigendian);
      return true;
    case sensor_msgs::msg::PointField::FLOAT64: {
      result = static_cast<float>(readDouble(data, message.is_bigendian));
      return true;
    }
    case sensor_msgs::msg::PointField::UINT16:
      result = static_cast<float>(readU16(data, message.is_bigendian));
      return true;
    case sensor_msgs::msg::PointField::INT32:
      result = static_cast<float>(static_cast<std::int32_t>(readInteger<4>(data, message.is_bigendian)));
      return true;
    case sensor_msgs::msg::PointField::UINT32:
      result = static_cast<float>(readInteger<4>(data, message.is_bigendian));
      return true;
    default:
      return false;
  }
}

class ScalenavLogNode final : public rclcpp::Node {
public:
  ScalenavLogNode()
  : Node("scalenav_log_node")
  {
    const auto output_dir = expandUser(declare_parameter<std::string>("output_dir", "~/scalenav_logs"));
    pointcloud_max_points_ = static_cast<std::size_t>(std::max<std::int64_t>(1, declare_parameter<std::int64_t>("pointcloud_max_points", 200000)));
    pointcloud_stride_ = static_cast<std::size_t>(std::max<std::int64_t>(1, declare_parameter<std::int64_t>("pointcloud_stride", 1)));
    const int qos_depth = std::max(1, static_cast<int>(declare_parameter<int>("qos_depth", 10)));

    depth_topic_ = declare_parameter<std::string>("depth_topic", "/camera/depth/image");
    rgb_topic_ = declare_parameter<std::string>("rgb_topic", "/camera/color/image");
    pointcloud_topic_ = declare_parameter<std::string>("pointcloud_topic", "/depth/points");
    free_ray_topic_ = declare_parameter<std::string>("free_ray_topic", "/depth/free_rays");
    graph_topic_ = declare_parameter<std::string>("graph_topic", "/epic/graph");
    bubble_topic_ = declare_parameter<std::string>("bubble_topic", "/epic/bubbles");
    path_topic_ = declare_parameter<std::string>("path_topic", "/epic/path");
    odom_topic_ = declare_parameter<std::string>("odom_topic", "/sim/odom");
    control_topic_ = declare_parameter<std::string>("control_topic", "/scalenav/trajectory_point");
    semantic_topic_ = declare_parameter<std::string>("semantic_topic", "/scalenav/text_heatmap_raw");
    goal_topic_ = declare_parameter<std::string>("goal_topic", "/goal_pose");
    local_goal_topic_ = declare_parameter<std::string>("local_goal_topic", "/epic/local_goal");
    clearance_topic_ = declare_parameter<std::string>("clearance_topic", "/epic/clearance");

    store_ = std::make_unique<SlidingLogStore>(fs::path(output_dir));
    std::ostringstream manifest;
    manifest << "{\"schema\":\"scalenav_log.v1\",\"created_unix_ns\":"
      << std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now().time_since_epoch()).count()
      << ",\"topics\":{";
    manifest << "\"depth\":" << jsonQuote(depth_topic_) << ",\"rgb\":" << jsonQuote(rgb_topic_)
      << ",\"pointcloud\":" << jsonQuote(pointcloud_topic_)
      << ",\"free_ray\":" << jsonQuote(free_ray_topic_) << ",\"graph\":" << jsonQuote(graph_topic_)
      << ",\"bubbles\":" << jsonQuote(bubble_topic_) << ",\"path\":" << jsonQuote(path_topic_)
      << ",\"odom\":" << jsonQuote(odom_topic_) << ",\"control\":" << jsonQuote(control_topic_)
      << ",\"semantic\":" << jsonQuote(semantic_topic_) << ",\"goal\":" << jsonQuote(goal_topic_)
      << ",\"local_goal\":" << jsonQuote(local_goal_topic_)
      << ",\"clearance\":" << jsonQuote(clearance_topic_)
      << "},\"pointcloud_max_points\":" << pointcloud_max_points_
      << ",\"pointcloud_stride\":" << pointcloud_stride_ << "}";
    store_->open(manifest.str());

    auto sensor_qos = rclcpp::SensorDataQoS().keep_last(qos_depth);
    depth_sub_ = create_subscription<sensor_msgs::msg::Image>(depth_topic_, sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) { captureDepth(*message); });
    rgb_sub_ = create_subscription<sensor_msgs::msg::Image>(rgb_topic_, sensor_qos,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) { captureRgb(*message); });
    pointcloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(pointcloud_topic_, sensor_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { capturePointCloud(*message, "pointcloud"); });
    free_ray_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(free_ray_topic_, sensor_qos,
      [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr message) { capturePointCloud(*message, "free_ray"); });
    graph_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(graph_topic_, qos_depth,
      [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr message) { captureMarkers(*message, "graph"); });
    bubble_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(bubble_topic_, qos_depth,
      [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr message) { captureMarkers(*message, "bubbles"); });
    path_sub_ = create_subscription<nav_msgs::msg::Path>(path_topic_, qos_depth,
      [this](nav_msgs::msg::Path::ConstSharedPtr message) { capturePath(*message); });
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(odom_topic_, sensor_qos,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr message) { captureOdom(*message); });
    control_sub_ = create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(control_topic_, qos_depth,
      [this](trajectory_msgs::msg::MultiDOFJointTrajectoryPoint::ConstSharedPtr message) { captureControl(*message); });
    semantic_sub_ = create_subscription<sensor_msgs::msg::Image>(semantic_topic_, qos_depth,
      [this](sensor_msgs::msg::Image::ConstSharedPtr message) { captureDepth(*message, "semantic"); });
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(goal_topic_, qos_depth,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) { captureGoal(*message); });
    local_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(local_goal_topic_, qos_depth,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr message) { captureLocalGoal(*message); });
    clearance_sub_ = create_subscription<geometry_msgs::msg::Vector3Stamped>(
      clearance_topic_, qos_depth,
      [this](geometry_msgs::msg::Vector3Stamped::ConstSharedPtr message) {
        captureClearance(*message);
      });
    RCLCPP_INFO(get_logger(), "scalenav log session: %s", store_->activeSession().string().c_str());
  }

private:
  void captureRgb(const sensor_msgs::msg::Image &message)
  {
    const std::size_t width = message.width, height = message.height;
    const bool bgr = message.encoding == "bgr8";
    const bool rgb = message.encoding == "rgb8";
    const bool bgra = message.encoding == "bgra8";
    const bool rgba = message.encoding == "rgba8";
    if (width == 0 || height == 0 || message.step == 0 || (!bgr && !rgb && !bgra && !rgba)) return;
    const std::size_t channels = (bgra || rgba) ? 4 : 3;
    if (message.step < width * channels || message.data.size() < message.step * height) return;
    std::ostringstream header;
    header << "P6\n" << width << " " << height << "\n255\n";
    const std::string header_text = header.str();
    std::vector<std::uint8_t> ppm(header_text.begin(), header_text.end());
    ppm.reserve(ppm.size() + width * height * 3);
    for (std::size_t y = 0; y < height; ++y) {
      const auto *row = message.data.data() + y * message.step;
      for (std::size_t x = 0; x < width; ++x) {
        const auto *pixel = row + x * channels;
        const bool swap = bgr || bgra;
        ppm.push_back(pixel[swap ? 2 : 0]);
        ppm.push_back(pixel[1]);
        ppm.push_back(pixel[swap ? 0 : 2]);
      }
    }
    const auto file = "rgb/rgb_" + std::to_string(++rgb_seq_) + ".ppm";
    const auto relative = store_->writeAsset(file, ppm);
    const auto extra = "{\"encoding\":" + jsonQuote(message.encoding) +
      ",\"width\":" + std::to_string(width) + ",\"height\":" + std::to_string(height) +
      ",\"step\":" + std::to_string(message.step) + ",\"frame_id\":" +
      jsonQuote(message.header.frame_id) + "}";
    store_->record("rgb", stampNs(message.header.stamp), relative, ppm.size(), extra);
  }

  void captureDepth(const sensor_msgs::msg::Image &message, const std::string &kind = "depth")
  {
    const std::size_t width = message.width, height = message.height;
    if (width == 0 || height == 0 || message.step == 0) return;
    std::vector<std::uint8_t> pgm;
    const bool is_float = message.encoding == "32FC1" || message.encoding == "TYPE_32FC1";
    const bool is_u16 = message.encoding == "16UC1" || message.encoding == "TYPE_16UC1";
    if (!is_float && !is_u16) {
      const std::string file = kind + "/" + kind + "_" + std::to_string(++depth_seq_) + ".bin";
      const auto bytes = message.data;
      const auto relative = store_->writeAsset(file, bytes);
      store_->record(kind + "_raw", stampNs(message.header.stamp), relative, bytes.size(), depthMetadata(message, false));
      return;
    }
    std::ostringstream header;
    header << "P5\n" << width << " " << height << "\n65535\n";
    const std::string text = header.str();
    pgm.assign(text.begin(), text.end());
    pgm.reserve(pgm.size() + width * height * 2);
    const std::size_t source_bytes = is_float ? sizeof(float) : sizeof(std::uint16_t);
    for (std::size_t y = 0; y < height; ++y) {
      const auto *row = message.data.data() + std::min<std::size_t>(y * message.step, message.data.size());
      for (std::size_t x = 0; x < width; ++x) {
        const std::size_t offset = x * source_bytes;
        double value = 0.0;
        if (offset + source_bytes <= message.step && y * message.step + offset + source_bytes <= message.data.size()) {
          value = is_float ? readFloat(row + offset, message.is_bigendian)
                           : static_cast<double>(readU16(row + offset, message.is_bigendian)) / 1000.0;
        }
        if (!std::isfinite(value) || value < 0.0) value = 0.0;
        const auto millimeters = static_cast<std::uint16_t>(std::clamp(value * 1000.0, 0.0, 65535.0));
        pgm.push_back(static_cast<std::uint8_t>(millimeters >> 8));
        pgm.push_back(static_cast<std::uint8_t>(millimeters & 0xff));
      }
    }
    const std::string file = kind + "/" + kind + "_" + std::to_string(++depth_seq_) + ".pgm";
    const auto relative = store_->writeAsset(file, pgm);
    store_->record(kind, stampNs(message.header.stamp), relative, pgm.size(), depthMetadata(message, true));
  }

  std::string depthMetadata(const sensor_msgs::msg::Image &message, bool pgm) const
  {
    return "{\"encoding\":" + jsonQuote(message.encoding) + ",\"width\":" + std::to_string(message.width) +
      ",\"height\":" + std::to_string(message.height) + ",\"step\":" + std::to_string(message.step) +
      ",\"frame_id\":" + jsonQuote(message.header.frame_id) + ",\"format\":" + jsonQuote(pgm ? "pgm16_mm" : "raw") + "}";
  }

  void capturePointCloud(const sensor_msgs::msg::PointCloud2 &message, const std::string &kind)
  {
    const auto x = findField(message, "x"), y = findField(message, "y"), z = findField(message, "z");
    if (x.offset < 0 || y.offset < 0 || z.offset < 0 || message.point_step == 0) return;
    const auto intensity = findField(message, "intensity");
    std::ostringstream out;
    const bool has_intensity = intensity.offset >= 0;
    out << "# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z" << (has_intensity ? " intensity" : "")
      << "\nSIZE 4 4 4" << (has_intensity ? " 4" : "") << "\nTYPE F F F" << (has_intensity ? " F" : "")
      << "\nCOUNT 1 1 1" << (has_intensity ? " 1" : "") << "\nWIDTH ";
    const std::size_t total = static_cast<std::size_t>(message.width) * std::max<std::size_t>(1, message.height);
    const std::size_t stride = std::max<std::size_t>(1, message.point_step);
    std::vector<std::array<float, 4>> points;
    points.reserve(std::min(total / pointcloud_stride_ + 1, pointcloud_max_points_));
    for (std::size_t index = 0; index < total && points.size() < pointcloud_max_points_; index += pointcloud_stride_) {
      const std::size_t offset = (index / std::max<std::size_t>(1, message.width)) * message.row_step +
        (index % std::max<std::size_t>(1, message.width)) * stride;
      if (offset + message.point_step > message.data.size()) break;
      float px = 0.0F, py = 0.0F, pz = 0.0F, pi = 0.0F;
      if (!readFieldFloat(message, message.data.data() + offset, x, px) ||
          !readFieldFloat(message, message.data.data() + offset, y, py) ||
          !readFieldFloat(message, message.data.data() + offset, z, pz) ||
          !std::isfinite(px) || !std::isfinite(py) || !std::isfinite(pz)) continue;
      if (has_intensity) readFieldFloat(message, message.data.data() + offset, intensity, pi);
      points.push_back({px, py, pz, pi});
    }
    out << points.size() << "\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS " << points.size() << "\nDATA ascii\n";
    out << std::setprecision(7);
    for (const auto &point : points) {
      out << point[0] << ' ' << point[1] << ' ' << point[2];
      if (has_intensity) out << ' ' << point[3];
      out << '\n';
    }
    const auto bytes = bytesFromString(out.str());
    const std::string file = "pointcloud/" + kind + "_" + std::to_string(++pointcloud_seq_) + ".pcd";
    const auto relative = store_->writeAsset(file, bytes);
    const std::string extra = "{\"frame_id\":" + jsonQuote(message.header.frame_id) +
      ",\"width\":" + std::to_string(message.width) + ",\"height\":" + std::to_string(message.height) +
      ",\"point_step\":" + std::to_string(message.point_step) + ",\"source_points\":" + std::to_string(total) +
      ",\"stored_points\":" + std::to_string(points.size()) + ",\"stride\":" + std::to_string(pointcloud_stride_) + "}";
    store_->record(kind, stampNs(message.header.stamp), relative, bytes.size(), extra);
  }

  std::string markerJson(const visualization_msgs::msg::Marker &marker) const
  {
    std::ostringstream out;
    out << "{\"ns\":" << jsonQuote(marker.ns) << ",\"id\":" << marker.id << ",\"type\":" << marker.type
      << ",\"action\":" << marker.action << ",\"frame_id\":" << jsonQuote(marker.header.frame_id)
      << ",\"pose\":{\"position\":" << array3(marker.pose.position) << ",\"orientation\":" << quaternion(marker.pose.orientation)
      << "},\"scale\":" << array3(marker.scale) << ",\"color\":[" << jsonNumber(marker.color.r) << ","
      << jsonNumber(marker.color.g) << "," << jsonNumber(marker.color.b) << "," << jsonNumber(marker.color.a) << "],\"points\":[";
    for (std::size_t i = 0; i < marker.points.size(); ++i) {
      if (i) out << ',';
      out << array3(marker.points[i]);
    }
    out << "],\"colors\":[";
    for (std::size_t i = 0; i < marker.colors.size(); ++i) {
      if (i) out << ',';
      out << '[' << jsonNumber(marker.colors[i].r) << ',' << jsonNumber(marker.colors[i].g)
        << ',' << jsonNumber(marker.colors[i].b) << ',' << jsonNumber(marker.colors[i].a) << ']';
    }
    out << "]}";
    return out.str();
  }

  void captureMarkers(const visualization_msgs::msg::MarkerArray &message, const std::string &kind)
  {
    std::ostringstream out;
    out << "{\"markers\":[";
    for (std::size_t i = 0; i < message.markers.size(); ++i) {
      if (i) out << ',';
      out << markerJson(message.markers[i]);
    }
    out << "]}";
    const auto bytes = bytesFromString(out.str());
    const std::string file = "graph/" + kind + "_" + std::to_string(++graph_seq_) + ".json";
    const auto relative = store_->writeAsset(file, bytes);
    const auto stamp = message.markers.empty() ? 0 : stampNs(message.markers.front().header.stamp);
    store_->record(kind, stamp, relative, bytes.size(), "{\"marker_count\":" + std::to_string(message.markers.size()) + "}");
  }

  void capturePath(const nav_msgs::msg::Path &message)
  {
    std::ostringstream out;
    out << "{\"frame_id\":" << jsonQuote(message.header.frame_id) << ",\"poses\":[";
    for (std::size_t i = 0; i < message.poses.size(); ++i) {
      if (i) out << ',';
      out << array3(message.poses[i].pose.position);
    }
    out << "]}";
    const auto bytes = bytesFromString(out.str());
    const auto file = "graph/path_" + std::to_string(++path_seq_) + ".json";
    const auto relative = store_->writeAsset(file, bytes);
    store_->record("path", stampNs(message.header.stamp), relative, bytes.size(), "{\"pose_count\":" + std::to_string(message.poses.size()) + "}");
  }

  void captureOdom(const nav_msgs::msg::Odometry &message)
  {
    const auto &p = message.pose.pose.position;
    const auto &v = message.twist.twist.linear;
    const auto extra = "{\"frame_id\":" + jsonQuote(message.header.frame_id) + ",\"position\":" + array3(p) +
      ",\"orientation\":" + quaternion(message.pose.pose.orientation) + ",\"velocity\":" + array3(v) + "}";
    store_->record("odom", stampNs(message.header.stamp), "", extra.size(), extra);
  }

  void captureControl(const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint &message)
  {
    std::ostringstream extra;
    extra << "{\"transforms\":" << message.transforms.size() << ",\"velocities\":" << message.velocities.size()
      << ",\"accelerations\":" << message.accelerations.size();
    if (!message.transforms.empty()) extra << ",\"position\":" << array3(message.transforms.front().translation);
    if (!message.velocities.empty()) extra << ",\"velocity\":" << array3(message.velocities.front().linear);
    if (!message.accelerations.empty()) extra << ",\"acceleration\":" << array3(message.accelerations.front().linear);
    extra << "}";
    store_->record("control", 0, "", extra.str().size(), extra.str());
  }

  void captureGoal(const geometry_msgs::msg::PoseStamped &message)
  {
    const auto extra = "{\"frame_id\":" + jsonQuote(message.header.frame_id) +
      ",\"position\":" + array3(message.pose.position) +
      ",\"orientation\":" + quaternion(message.pose.orientation) + "}";
    store_->record("goal", stampNs(message.header.stamp), "", extra.size(), extra);
  }

  void captureLocalGoal(const geometry_msgs::msg::PoseStamped &message)
  {
    const auto extra = "{\"frame_id\":" + jsonQuote(message.header.frame_id) +
      ",\"position\":" + array3(message.pose.position) +
      ",\"orientation\":" + quaternion(message.pose.orientation) + "}";
    store_->record("local_goal", stampNs(message.header.stamp), "", extra.size(), extra);
  }

  void captureClearance(const geometry_msgs::msg::Vector3Stamped &message)
  {
    const auto extra = "{\"frame_id\":" + jsonQuote(message.header.frame_id) +
      ",\"vehicle_m\":" + jsonNumber(message.vector.x) +
      ",\"path_min_m\":" + jsonNumber(message.vector.y) +
      ",\"path_mean_m\":" + jsonNumber(message.vector.z) + "}";
    store_->record("clearance", stampNs(message.header.stamp), "", extra.size(), extra);
  }

  std::unique_ptr<SlidingLogStore> store_;
  std::size_t pointcloud_max_points_ = 200000;
  std::size_t pointcloud_stride_ = 1;
  std::uint64_t depth_seq_ = 0, rgb_seq_ = 0, pointcloud_seq_ = 0, graph_seq_ = 0, path_seq_ = 0;
  std::string depth_topic_, rgb_topic_, pointcloud_topic_, free_ray_topic_, graph_topic_,
    bubble_topic_, path_topic_, odom_topic_, control_topic_, semantic_topic_, goal_topic_,
    local_goal_topic_, clearance_topic_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_sub_, rgb_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_, free_ray_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr graph_sub_, bubble_sub_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr control_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr semantic_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_, local_goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr clearance_sub_;
};

}  // namespace

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ScalenavLogNode>());
  rclcpp::shutdown();
  return 0;
}
