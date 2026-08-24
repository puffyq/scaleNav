// Optional keyboard command source for the standalone ROS2 controller.
#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/multi_dof_joint_trajectory_point.hpp>

#include <X11/Xlib.h>
#include <X11/keysym.h>
#undef None

namespace
{

double yaw_from_quaternion(const geometry_msgs::msg::Quaternion & q)
{
  return std::atan2(
    2.0 * (q.w * q.z + q.x * q.y),
    1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}

double move_toward(double value, double target, double maximum_delta)
{
  return value + std::clamp(target - value, -maximum_delta, maximum_delta);
}

std::array<double, 3> move_vector_toward(
  const std::array<double, 3> & value,
  const std::array<double, 3> & target,
  double maximum_delta)
{
  std::array<double, 3> delta{
    target[0] - value[0], target[1] - value[1], target[2] - value[2]};
  const double norm = std::sqrt(
    delta[0] * delta[0] + delta[1] * delta[1] + delta[2] * delta[2]);
  if (norm > maximum_delta && norm > 0.0) {
    const double scale = maximum_delta / norm;
    delta[0] *= scale;
    delta[1] *= scale;
    delta[2] *= scale;
  }
  return {value[0] + delta[0], value[1] + delta[1], value[2] + delta[2]};
}

struct KeyboardState
{
  explicit KeyboardState(Display * display)
  : display(display)
  {
    keycodes = {
      XKeysymToKeycode(display, XK_w), XKeysymToKeycode(display, XK_s),
      XKeysymToKeycode(display, XK_a), XKeysymToKeycode(display, XK_d),
      XKeysymToKeycode(display, XK_q), XKeysymToKeycode(display, XK_e),
      XKeysymToKeycode(display, XK_r), XKeysymToKeycode(display, XK_f),
      XKeysymToKeycode(display, XK_t), XKeysymToKeycode(display, XK_h)};
  }

  void update()
  {
    XQueryKeymap(display, keys.data());
  }

  bool down(std::size_t index) const
  {
    const auto code = keycodes.at(index);
    return code != 0 && (keys[code / 8] & (1 << (code % 8))) != 0;
  }

  Display * display;
  std::array<KeyCode, 10> keycodes{};
  std::array<char, 32> keys{};
};

}  // namespace

class UavKeyboardNode final : public rclcpp::Node
{
public:
  UavKeyboardNode()
  : Node("uav_keyboard"), display_(XOpenDisplay(nullptr))
  {
    if (display_ == nullptr) {
      throw std::runtime_error("cannot open X11 display; run this node in the graphical desktop session");
    }
    keyboard_ = std::make_unique<KeyboardState>(display_);

    maximum_linear_speed_ = declare_parameter("maximum_linear_speed", 3.0);
    maximum_yaw_rate_ = declare_parameter("maximum_yaw_rate", 1.0);
    linear_acceleration_ = declare_parameter("linear_acceleration", 4.0);
    yaw_acceleration_ = declare_parameter("yaw_acceleration", 2.0);
    control_rate_ = declare_parameter("control_rate", 50.0);
    minimum_altitude_ = declare_parameter("minimum_altitude", 0.15);

    if (maximum_linear_speed_ <= 0.0 || maximum_yaw_rate_ <= 0.0 ||
      linear_acceleration_ <= 0.0 || yaw_acceleration_ <= 0.0 || control_rate_ <= 0.0)
    {
      throw std::invalid_argument("keyboard speed, acceleration, and control rate must be positive");
    }

    command_pub_ = create_publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>(
      "/scalenav/trajectory_point", 10);
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "/sim/odom", 20,
      std::bind(&UavKeyboardNode::receive_odometry, this, std::placeholders::_1));

    const auto period = std::chrono::duration<double>(1.0 / control_rate_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&UavKeyboardNode::control_step, this));
    previous_update_ = std::chrono::steady_clock::now();
    print_help();
  }

  ~UavKeyboardNode() override
  {
    keyboard_.reset();
    if (display_ != nullptr) {
      XCloseDisplay(display_);
    }
  }

private:
  void receive_odometry(const nav_msgs::msg::Odometry::SharedPtr message)
  {
    latest_odom_ = *message;
    if (command_initialized_) {
      return;
    }

    target_x_ = message->pose.pose.position.x;
    target_y_ = message->pose.pose.position.y;
    target_z_ = std::max(minimum_altitude_, message->pose.pose.position.z);
    target_yaw_ = yaw_from_quaternion(message->pose.pose.orientation);
    command_initialized_ = true;
    RCLCPP_INFO(get_logger(), "Continuous keyboard control initialized from /sim/odom");
  }

  void control_step()
  {
    const auto current_update = std::chrono::steady_clock::now();
    const double dt = std::clamp(
      std::chrono::duration<double>(current_update - previous_update_).count(), 0.0, 0.1);
    previous_update_ = current_update;
    keyboard_->update();

    if (!command_initialized_) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000, "Waiting for /sim/odom before accepting keys");
      return;
    }

    if (keyboard_->down(8) && !reset_was_down_) {
      synchronize_target();
    }
    reset_was_down_ = keyboard_->down(8);
    if (keyboard_->down(9) && !help_was_down_) {
      print_help();
    }
    help_was_down_ = keyboard_->down(9);

    double body_forward = static_cast<double>(keyboard_->down(0)) - keyboard_->down(1);
    double body_left = static_cast<double>(keyboard_->down(2)) - keyboard_->down(3);
    double vertical = static_cast<double>(keyboard_->down(6)) - keyboard_->down(7);
    const double direction_norm = std::sqrt(
      body_forward * body_forward + body_left * body_left + vertical * vertical);
    if (direction_norm > 1.0) {
      body_forward /= direction_norm;
      body_left /= direction_norm;
      vertical /= direction_norm;
    }

    const double cosine = std::cos(target_yaw_);
    const double sine = std::sin(target_yaw_);
    const double requested_vx = maximum_linear_speed_ *
      (body_forward * cosine - body_left * sine);
    const double requested_vy = maximum_linear_speed_ *
      (body_forward * sine + body_left * cosine);
    const double requested_vz = maximum_linear_speed_ * vertical;
    const double requested_yaw_rate = maximum_yaw_rate_ *
      (static_cast<double>(keyboard_->down(4)) - keyboard_->down(5));

    const auto limited_velocity = move_vector_toward(
      {velocity_x_, velocity_y_, velocity_z_},
      {requested_vx, requested_vy, requested_vz},
      linear_acceleration_ * dt);
    velocity_x_ = limited_velocity[0];
    velocity_y_ = limited_velocity[1];
    velocity_z_ = limited_velocity[2];
    yaw_rate_ = move_toward(yaw_rate_, requested_yaw_rate, yaw_acceleration_ * dt);

    target_x_ += velocity_x_ * dt;
    target_y_ += velocity_y_ * dt;
    target_z_ = std::max(minimum_altitude_, target_z_ + velocity_z_ * dt);
    if (target_z_ <= minimum_altitude_ && velocity_z_ < 0.0) {
      velocity_z_ = 0.0;
    }
    target_yaw_ += yaw_rate_ * dt;
    publish_target();
  }

  void synchronize_target()
  {
    target_x_ = latest_odom_.pose.pose.position.x;
    target_y_ = latest_odom_.pose.pose.position.y;
    target_z_ = std::max(minimum_altitude_, latest_odom_.pose.pose.position.z);
    target_yaw_ = yaw_from_quaternion(latest_odom_.pose.pose.orientation);
    velocity_x_ = 0.0;
    velocity_y_ = 0.0;
    velocity_z_ = 0.0;
    yaw_rate_ = 0.0;
    RCLCPP_INFO(get_logger(), "Keyboard target synchronized to current UAV pose");
  }

  void publish_target()
  {
    trajectory_msgs::msg::MultiDOFJointTrajectoryPoint command;
    geometry_msgs::msg::Transform transform;
    transform.translation.x = target_x_;
    transform.translation.y = target_y_;
    transform.translation.z = target_z_;
    transform.rotation.z = std::sin(target_yaw_ * 0.5);
    transform.rotation.w = std::cos(target_yaw_ * 0.5);
    command.transforms.push_back(transform);

    geometry_msgs::msg::Twist velocity;
    velocity.linear.x = velocity_x_;
    velocity.linear.y = velocity_y_;
    velocity.linear.z = velocity_z_;
    velocity.angular.z = yaw_rate_;
    command.velocities.push_back(velocity);
    command_pub_->publish(command);
  }

  void print_help() const
  {
    std::printf(
      "\nScaleNav UAV continuous keyboard control (keys are read globally)\n"
      "  W/S: forward/backward    A/D: left/right\n"
      "  Q/E: yaw left/right      R/F: up/down\n"
      "  T: stop and sync target   H: show help   Ctrl+C: quit\n"
      "  limits: linear %.1f m/s @ %.1f m/s^2, yaw %.1f rad/s @ %.1f rad/s^2\n\n",
      maximum_linear_speed_, linear_acceleration_, maximum_yaw_rate_, yaw_acceleration_);
    std::fflush(stdout);
  }

  Display * display_{nullptr};
  std::unique_ptr<KeyboardState> keyboard_;
  rclcpp::Publisher<trajectory_msgs::msg::MultiDOFJointTrajectoryPoint>::SharedPtr command_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  nav_msgs::msg::Odometry latest_odom_;
  std::chrono::steady_clock::time_point previous_update_;
  double maximum_linear_speed_{3.0};
  double maximum_yaw_rate_{1.0};
  double linear_acceleration_{4.0};
  double yaw_acceleration_{2.0};
  double control_rate_{50.0};
  double minimum_altitude_{0.15};
  double target_x_{0.0};
  double target_y_{0.0};
  double target_z_{0.0};
  double target_yaw_{0.0};
  double velocity_x_{0.0};
  double velocity_y_{0.0};
  double velocity_z_{0.0};
  double yaw_rate_{0.0};
  bool command_initialized_{false};
  bool reset_was_down_{false};
  bool help_was_down_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<UavKeyboardNode>());
  } catch (const std::exception & error) {
    std::fprintf(stderr, "uav_keyboard_node: %s\n", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
