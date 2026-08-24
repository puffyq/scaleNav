// ROS2 SO3 controller and rigid-body simulation used by the AirSim renderer.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <quadrotor_simulator/Quadrotor.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

namespace
{

using Quadrotor = QuadrotorSimulator::Quadrotor;

bool finite(double value)
{
  return std::isfinite(value);
}

bool finite(const Eigen::Vector3d & value)
{
  return value.allFinite();
}

Eigen::Vector3d array3(const std::vector<double> & value, const Eigen::Vector3d & fallback)
{
  if (value.size() != 3 || !finite(value[0]) || !finite(value[1]) || !finite(value[2])) {
    return fallback;
  }
  return {value[0], value[1], value[2]};
}

double quaternion_yaw(const geometry_msgs::msg::Quaternion & q)
{
  const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
  if (!finite(norm) || norm < 1e-6) {
    return 0.0;
  }
  const double x = q.x / norm;
  const double y = q.y / norm;
  const double z = q.z / norm;
  const double w = q.w / norm;
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

double wrap_angle(double angle)
{
  return std::atan2(std::sin(angle), std::cos(angle));
}

Eigen::Vector3d clamp_norm(const Eigen::Vector3d & value, double maximum_norm)
{
  if (value.norm() <= maximum_norm) {
    return value;
  }
  return value.normalized() * maximum_norm;
}

geometry_msgs::msg::Quaternion quaternion_msg(const Eigen::Matrix3d & rotation)
{
  const Eigen::Quaterniond q(rotation);
  geometry_msgs::msg::Quaternion message;
  message.x = q.x();
  message.y = q.y();
  message.z = q.z();
  message.w = q.w();
  return message;
}

Eigen::Vector3d vee(const Eigen::Matrix3d & matrix)
{
  return {matrix(2, 1), matrix(0, 2), matrix(1, 0)};
}

}  // namespace

class UavSim final : public rclcpp::Node
{
public:
  UavSim()
  : Node("uav_sim"), tf_broadcaster_(*this)
  {
    simulation_rate_ = declare_parameter("simulation_rate", 500.0);
    odom_rate_ = declare_parameter("odom_rate", 100.0);
    command_timeout_ = declare_parameter("command_timeout", 0.5);
    minimum_altitude_ = declare_parameter("minimum_altitude", 0.15);
    maximum_acceleration_ = declare_parameter("maximum_acceleration", 12.0);
    maximum_linear_speed_ = declare_parameter("maximum_linear_speed", 3.0);
    maximum_yaw_rate_ = declare_parameter("maximum_yaw_rate", 1.0);
    maximum_command_acceleration_ = declare_parameter("maximum_command_acceleration", 4.0);
    maximum_yaw_acceleration_ = declare_parameter("maximum_yaw_acceleration", 2.0);
    world_frame_ = declare_parameter("world_frame", std::string("map"));
    body_frame_ = declare_parameter("body_frame", std::string("base_link"));

    initial_position_ = array3(
      declare_parameter("initial_position", std::vector<double>{5.28, -2.21, 2.0}),
      {5.28, -2.21, 2.0});
    kp_ = array3(
      declare_parameter("position_gain", std::vector<double>{4.0, 4.0, 6.0}),
      {4.0, 4.0, 6.0});
    kv_ = array3(
      declare_parameter("velocity_gain", std::vector<double>{3.0, 3.0, 4.0}),
      {3.0, 3.0, 4.0});
    kr_ = array3(
      declare_parameter("attitude_gain", std::vector<double>{2.0, 2.0, 1.0}),
      {2.0, 2.0, 1.0});
    kw_ = array3(
      declare_parameter("angular_rate_gain", std::vector<double>{0.15, 0.15, 0.10}),
      {0.15, 0.15, 0.10});

    if (simulation_rate_ <= 0.0 || odom_rate_ <= 0.0 || odom_rate_ > simulation_rate_) {
      throw std::invalid_argument("simulation_rate and odom_rate are inconsistent");
    }
    if (!finite(initial_position_) || initial_position_.z() < minimum_altitude_) {
      throw std::invalid_argument("initial_position is invalid or below minimum_altitude");
    }
    if (maximum_linear_speed_ <= 0.0 || maximum_yaw_rate_ <= 0.0 ||
      maximum_command_acceleration_ <= 0.0 || maximum_yaw_acceleration_ <= 0.0)
    {
      throw std::invalid_argument("command speed and acceleration limits must be positive");
    }

    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/sim/odom", 20);
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("/sim/imu", 20);
    rpm_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>("/sim/motor_rpm", 10);

    pose_cmd_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/scalenav/position_cmd", 10,
      std::bind(&UavSim::pose_command, this, std::placeholders::_1));
    trajectory_cmd_sub_ =
      create_subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
      "/scalenav/trajectory_point", 10,
      std::bind(&UavSim::trajectory_command, this, std::placeholders::_1));
    collision_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/sim/collision", 10,
      std::bind(&UavSim::emergency_stop, this, std::placeholders::_1));
    emergency_sub_ = create_subscription<std_msgs::msg::Bool>(
      "/scalenav/emergency_stop", 10,
      std::bind(&UavSim::emergency_stop, this, std::placeholders::_1));
    reset_service_ = create_service<std_srvs::srv::Trigger>(
      "/scalenav/reset_sim",
      std::bind(
        &UavSim::reset_service, this, std::placeholders::_1,
        std::placeholders::_2));

    reset_state();
    last_command_time_ = now();
    const auto period = std::chrono::duration<double>(1.0 / simulation_rate_);
    simulation_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&UavSim::simulation_step, this));

    RCLCPP_INFO(
      get_logger(),
      "UAV simulator ready: command=/scalenav/trajectory_point, odom=/sim/odom");
  }

private:
  struct DesiredState
  {
    Eigen::Vector3d position{Eigen::Vector3d::Zero()};
    Eigen::Vector3d velocity{Eigen::Vector3d::Zero()};
    Eigen::Vector3d acceleration{Eigen::Vector3d::Zero()};
    double yaw{0.0};
    double yaw_rate{0.0};
  };

  void reset_state()
  {
    Quadrotor::State state;
    state.x = initial_position_;
    state.v.setZero();
    state.R.setIdentity();
    state.omega.setZero();
    state.motor_rpm.setZero();
    quadrotor_.setState(state);
    quadrotor_.setExternalForce(Eigen::Vector3d::Zero());
    quadrotor_.setExternalMoment(Eigen::Vector3d::Zero());
    desired_.position = initial_position_;
    desired_.velocity.setZero();
    desired_.acceleration.setZero();
    desired_.yaw = 0.0;
    desired_.yaw_rate = 0.0;
    emergency_stop_latched_ = false;
    timeout_hold_active_ = false;
    persistent_command_ = true;
    publish_count_ = 0;
  }

  void pose_command(const geometry_msgs::msg::PoseStamped::SharedPtr message)
  {
    const Eigen::Vector3d position(
      message->pose.position.x, message->pose.position.y, message->pose.position.z);
    if (!finite(position) || position.z() < minimum_altitude_) {
      RCLCPP_WARN(get_logger(), "Rejected invalid position command");
      return;
    }
    desired_.position = position;
    desired_.velocity.setZero();
    desired_.acceleration.setZero();
    desired_.yaw = quaternion_yaw(message->pose.orientation);
    desired_.yaw_rate = 0.0;
    accept_command(true);
  }

  void trajectory_command(
    const trajectory_msgs::msg::MultiDOFJointTrajectoryPoint::SharedPtr message)
  {
    if (message->transforms.empty()) {
      RCLCPP_WARN(get_logger(), "Rejected trajectory point without transform");
      return;
    }
    const auto & transform = message->transforms.front();
    const Eigen::Vector3d position(
      transform.translation.x, transform.translation.y, transform.translation.z);
    if (!finite(position) || position.z() < minimum_altitude_) {
      RCLCPP_WARN(get_logger(), "Rejected invalid trajectory position");
      return;
    }

    Eigen::Vector3d requested_velocity = Eigen::Vector3d::Zero();
    Eigen::Vector3d acceleration = Eigen::Vector3d::Zero();
    double requested_yaw_rate = 0.0;
    if (!message->velocities.empty()) {
      const auto & velocity = message->velocities.front();
      requested_velocity = {velocity.linear.x, velocity.linear.y, velocity.linear.z};
      requested_yaw_rate = velocity.angular.z;
    }
    if (!message->accelerations.empty()) {
      const auto & linear = message->accelerations.front().linear;
      acceleration = {linear.x, linear.y, linear.z};
    }
    if (!finite(requested_velocity) || !finite(acceleration) || !std::isfinite(requested_yaw_rate)) {
      RCLCPP_WARN(get_logger(), "Rejected non-finite trajectory derivative");
      return;
    }

    const auto command_time = now();
    const double command_dt = std::clamp(
      (command_time - last_command_time_).seconds(), 1.0 / simulation_rate_, 0.1);
    requested_velocity = clamp_norm(requested_velocity, maximum_linear_speed_);
    const Eigen::Vector3d velocity_delta = clamp_norm(
      requested_velocity - desired_.velocity,
      maximum_command_acceleration_ * command_dt);
    const Eigen::Vector3d limited_velocity = desired_.velocity + velocity_delta;

    requested_yaw_rate = std::clamp(
      requested_yaw_rate, -maximum_yaw_rate_, maximum_yaw_rate_);
    const double limited_yaw_rate = desired_.yaw_rate + std::clamp(
      requested_yaw_rate - desired_.yaw_rate,
      -maximum_yaw_acceleration_ * command_dt,
      maximum_yaw_acceleration_ * command_dt);
    const double requested_yaw = quaternion_yaw(transform.rotation);
    const double limited_yaw_delta = std::clamp(
      wrap_angle(requested_yaw - desired_.yaw),
      -maximum_yaw_rate_ * command_dt,
      maximum_yaw_rate_ * command_dt);

    desired_.position = position;
    desired_.velocity = limited_velocity;
    desired_.acceleration = acceleration;
    desired_.yaw = wrap_angle(desired_.yaw + limited_yaw_delta);
    desired_.yaw_rate = limited_yaw_rate;
    accept_command(false);
  }

  void accept_command(bool persistent)
  {
    last_command_time_ = now();
    timeout_hold_active_ = false;
    persistent_command_ = persistent;
  }

  void emergency_stop(const std_msgs::msg::Bool::SharedPtr message)
  {
    if (!message->data || emergency_stop_latched_) {
      return;
    }
    emergency_stop_latched_ = true;
    hold_current_position();
    RCLCPP_ERROR(get_logger(), "Emergency stop latched; call /scalenav/reset_sim to reset");
  }

  void reset_service(
    const std_srvs::srv::Trigger::Request::SharedPtr,
    std_srvs::srv::Trigger::Response::SharedPtr response)
  {
    reset_state();
    last_command_time_ = now();
    response->success = true;
    response->message = "UAV simulator reset";
  }

  void hold_current_position()
  {
    desired_.position = quadrotor_.getState().x;
    desired_.position.z() = std::max(desired_.position.z(), minimum_altitude_);
    desired_.velocity.setZero();
    desired_.acceleration.setZero();
    const Eigen::Vector3d body_x = quadrotor_.getState().R.col(0);
    desired_.yaw = std::atan2(body_x.y(), body_x.x());
    desired_.yaw_rate = 0.0;
  }

  void simulation_step()
  {
    const auto current_time = now();
    if (!emergency_stop_latched_ && !persistent_command_ && command_timeout_ > 0.0 &&
      (current_time - last_command_time_).seconds() > command_timeout_ &&
      !timeout_hold_active_)
    {
      hold_current_position();
      timeout_hold_active_ = true;
    }

    const auto & state = quadrotor_.getState();
    const Eigen::Array4d rpm = motor_command(state);
    quadrotor_.setInput(rpm[0], rpm[1], rpm[2], rpm[3]);
    quadrotor_.step(1.0 / simulation_rate_);

    ++publish_count_;
    const auto publish_divisor = std::max(
      1, static_cast<int>(std::lround(simulation_rate_ / odom_rate_)));
    if (publish_count_ % publish_divisor == 0) {
      publish_state(current_time);
    }
  }

  Eigen::Array4d motor_command(const Quadrotor::State & state) const
  {
    Eigen::Vector3d commanded_acceleration = desired_.acceleration +
      kp_.cwiseProduct(desired_.position - state.x) +
      kv_.cwiseProduct(desired_.velocity - state.v);
    if (commanded_acceleration.norm() > maximum_acceleration_) {
      commanded_acceleration =
        commanded_acceleration.normalized() * maximum_acceleration_;
    }

    const Eigen::Vector3d gravity(0.0, 0.0, quadrotor_.getGravity());
    Eigen::Vector3d desired_force = quadrotor_.getMass() * (commanded_acceleration + gravity);
    if (desired_force.norm() < 1e-6) {
      desired_force = quadrotor_.getMass() * gravity;
    }

    const Eigen::Vector3d b3 = desired_force.normalized();
    const Eigen::Vector3d heading(std::cos(desired_.yaw), std::sin(desired_.yaw), 0.0);
    Eigen::Vector3d b2 = b3.cross(heading);
    if (b2.norm() < 1e-6) {
      b2 = Eigen::Vector3d::UnitY();
    } else {
      b2.normalize();
    }
    const Eigen::Vector3d b1 = b2.cross(b3).normalized();
    Eigen::Matrix3d desired_rotation;
    desired_rotation.col(0) = b1;
    desired_rotation.col(1) = b2;
    desired_rotation.col(2) = b3;

    const Eigen::Matrix3d rotation_error_matrix =
      0.5 * (desired_rotation.transpose() * state.R -
      state.R.transpose() * desired_rotation);
    const Eigen::Vector3d rotation_error = vee(rotation_error_matrix);
    const Eigen::Matrix3d inertia = quadrotor_.getInertia();
    const Eigen::Vector3d moment = -kr_.cwiseProduct(rotation_error) -
      kw_.cwiseProduct(state.omega) + state.omega.cross(inertia * state.omega);
    const double thrust = std::max(0.0, desired_force.dot(state.R.col(2)));

    const double kf = quadrotor_.getPropellerThrustCoefficient();
    const double km = quadrotor_.getPropellerMomentCoefficient();
    const double arm = quadrotor_.getArmLength();
    Eigen::Array4d squared;
    squared[0] = thrust / (4.0 * kf) - moment.y() / (2.0 * arm * kf) +
      moment.z() / (4.0 * km);
    squared[1] = thrust / (4.0 * kf) + moment.y() / (2.0 * arm * kf) +
      moment.z() / (4.0 * km);
    squared[2] = thrust / (4.0 * kf) + moment.x() / (2.0 * arm * kf) -
      moment.z() / (4.0 * km);
    squared[3] = thrust / (4.0 * kf) - moment.x() / (2.0 * arm * kf) -
      moment.z() / (4.0 * km);

    for (Eigen::Index index = 0; index < squared.size(); ++index) {
      squared[index] = std::sqrt(std::max(0.0, squared[index]));
    }
    return squared;
  }

  void publish_state(const rclcpp::Time & stamp)
  {
    const auto & state = quadrotor_.getState();
    const auto orientation = quaternion_msg(state.R);

    nav_msgs::msg::Odometry odom;
    odom.header.stamp = stamp;
    odom.header.frame_id = world_frame_;
    odom.child_frame_id = body_frame_;
    odom.pose.pose.position.x = state.x.x();
    odom.pose.pose.position.y = state.x.y();
    odom.pose.pose.position.z = state.x.z();
    odom.pose.pose.orientation = orientation;
    const Eigen::Vector3d body_velocity = state.R.transpose() * state.v;
    odom.twist.twist.linear.x = body_velocity.x();
    odom.twist.twist.linear.y = body_velocity.y();
    odom.twist.twist.linear.z = body_velocity.z();
    odom.twist.twist.angular.x = state.omega.x();
    odom.twist.twist.angular.y = state.omega.y();
    odom.twist.twist.angular.z = state.omega.z();
    odom_pub_->publish(odom);

    sensor_msgs::msg::Imu imu;
    imu.header = odom.header;
    imu.header.frame_id = body_frame_;
    imu.orientation = orientation;
    imu.angular_velocity = odom.twist.twist.angular;
    const Eigen::Vector3d proper_acceleration = state.R.transpose() *
      (quadrotor_.getAcc() + Eigen::Vector3d(0.0, 0.0, quadrotor_.getGravity()));
    imu.linear_acceleration.x = proper_acceleration.x();
    imu.linear_acceleration.y = proper_acceleration.y();
    imu.linear_acceleration.z = proper_acceleration.z();
    imu_pub_->publish(imu);

    geometry_msgs::msg::TransformStamped transform;
    transform.header = odom.header;
    transform.child_frame_id = body_frame_;
    transform.transform.translation.x = state.x.x();
    transform.transform.translation.y = state.x.y();
    transform.transform.translation.z = state.x.z();
    transform.transform.rotation = orientation;
    tf_broadcaster_.sendTransform(transform);

    std_msgs::msg::Float64MultiArray rpm;
    rpm.data.assign(state.motor_rpm.data(), state.motor_rpm.data() + 4);
    rpm_pub_->publish(rpm);
  }

  Quadrotor quadrotor_;
  DesiredState desired_;
  Eigen::Vector3d initial_position_;
  Eigen::Vector3d kp_;
  Eigen::Vector3d kv_;
  Eigen::Vector3d kr_;
  Eigen::Vector3d kw_;
  double simulation_rate_;
  double odom_rate_;
  double command_timeout_;
  double minimum_altitude_;
  double maximum_acceleration_;
  double maximum_linear_speed_;
  double maximum_yaw_rate_;
  double maximum_command_acceleration_;
  double maximum_yaw_acceleration_;
  std::string world_frame_;
  std::string body_frame_;
  bool emergency_stop_latched_{false};
  bool timeout_hold_active_{false};
  bool persistent_command_{false};
  int publish_count_{0};
  rclcpp::Time last_command_time_;

  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rpm_pub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_cmd_sub_;
  rclcpp::Subscription<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr
    trajectory_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr collision_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_sub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_service_;
  rclcpp::TimerBase::SharedPtr simulation_timer_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<UavSim>());
  rclcpp::shutdown();
  return 0;
}
