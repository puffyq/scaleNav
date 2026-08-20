#pragma once

#include <cassert>
#include <builtin_interfaces/msg/time.hpp>
#include <rclcpp/rclcpp.hpp>
#include <boost/make_shared.hpp>

#include <functional>
#include <memory>
#include <string>
#include <type_traits>
#include <utility>

namespace ros {

struct TimerEvent {};
class Duration;

class Time {
 public:
  Time() = default;
  explicit Time(const builtin_interfaces::msg::Time &value) : value_(value) {}
  static Time now() {
    const int64_t nanoseconds = rclcpp::Clock(RCL_SYSTEM_TIME).now().nanoseconds();
    builtin_interfaces::msg::Time value;
    value.sec = static_cast<int32_t>(nanoseconds / 1000000000LL);
    value.nanosec = static_cast<uint32_t>(nanoseconds % 1000000000LL);
    return Time(value);
  }
  explicit Time(int) {}
  double toSec() const {
    return static_cast<double>(value_.sec) +
           static_cast<double>(value_.nanosec) * 1e-9;
  }
  friend Duration operator-(const Time &lhs, const Time &rhs);
  operator builtin_interfaces::msg::Time() const { return value_; }
 private:
  builtin_interfaces::msg::Time value_;
};

class Duration {
 public:
  explicit Duration(double seconds = 0.0)
      : duration_(rclcpp::Duration::from_seconds(seconds)) {}
  int64_t nanoseconds() const { return duration_.nanoseconds(); }
  double toSec() const { return duration_.seconds(); }
 private:
  rclcpp::Duration duration_;
};

inline Duration operator-(const Time &lhs, const Time &rhs) {
  return Duration(lhs.toSec() - rhs.toSec());
}

class Timer {
 public:
  Timer() = default;
  explicit Timer(rclcpp::TimerBase::SharedPtr timer) : timer_(std::move(timer)) {}
 private:
  rclcpp::TimerBase::SharedPtr timer_;
};

class Subscriber {
 public:
  Subscriber() = default;
  explicit Subscriber(rclcpp::SubscriptionBase::SharedPtr subscription)
      : subscription_(std::move(subscription)) {}
 private:
  rclcpp::SubscriptionBase::SharedPtr subscription_;
};

class Publisher {
 private:
  struct Base {
    virtual ~Base() = default;
  };
  template <typename Message>
  struct Impl final : Base {
    explicit Impl(typename rclcpp::Publisher<Message>::SharedPtr publisher)
        : publisher(std::move(publisher)) {}
    typename rclcpp::Publisher<Message>::SharedPtr publisher;
  };

 public:
  Publisher() = default;
  template <typename Message>
  explicit Publisher(typename rclcpp::Publisher<Message>::SharedPtr publisher)
      : impl_(std::make_shared<Impl<Message>>(std::move(publisher))) {}

  template <typename Message>
  static Publisher make(typename rclcpp::Publisher<Message>::SharedPtr publisher) {
    Publisher result;
    result.impl_ = std::make_shared<Impl<Message>>(std::move(publisher));
    return result;
  }

  template <typename Message>
  void publish(const Message &message) const {
    if constexpr (rclcpp::is_ros_compatible_type<Message>::value) {
      auto typed = std::dynamic_pointer_cast<Impl<Message>>(impl_);
      if (typed) typed->publisher->publish(message);
    } else {
      (void)message;
    }
  }

 private:
  std::shared_ptr<Base> impl_;
};

class NodeHandle {
 public:
  NodeHandle() = default;
  explicit NodeHandle(const rclcpp::Node::SharedPtr &node) : node_(node) {}

  template <typename T>
  void param(const std::string &name, T &value, const T &default_value) const {
    if constexpr (std::is_floating_point_v<T>) {
      if (!node_->has_parameter(name)) {
        node_->declare_parameter<double>(name, static_cast<double>(default_value));
      }
      value = static_cast<T>(node_->get_parameter(name).as_double());
    } else if constexpr (std::is_integral_v<T> && !std::is_same_v<T, bool>) {
      if (!node_->has_parameter(name)) {
        node_->declare_parameter<int64_t>(name, static_cast<int64_t>(default_value));
      }
      value = static_cast<T>(node_->get_parameter(name).as_int());
    } else {
      if (!node_->has_parameter(name)) node_->declare_parameter<T>(name, default_value);
      value = node_->get_parameter(name).get_value<T>();
    }
  }

  template <typename T>
  bool getParam(const std::string &name, T &value) const {
    if (!node_->has_parameter(name)) return false;
    value = node_->get_parameter(name).get_value<T>();
    return true;
  }

  template <typename Message, typename Object>
  Subscriber subscribe(
      const std::string &topic, std::size_t,
      void (Object::*callback)(const std::shared_ptr<const Message> &),
      Object *object) const {
    auto subscription = node_->create_subscription<Message>(
        topic, rclcpp::SensorDataQoS(),
        [object, callback](const std::shared_ptr<const Message> message) {
          (object->*callback)(message);
        });
    return Subscriber(subscription);
  }

  template <typename Message>
  Publisher advertise(const std::string &topic, std::size_t queue_size) const {
    if constexpr (rclcpp::is_ros_compatible_type<Message>::value) {
      return Publisher::make<Message>(
          node_->create_publisher<Message>(topic, queue_size));
    } else {
      (void)topic;
      (void)queue_size;
      return Publisher();
    }
  }

  template <typename Message>
  Publisher advertise(const std::string &topic, std::size_t queue_size,
                      bool) const {
    return advertise<Message>(topic, queue_size);
  }

  template <typename Object>
  Timer createTimer(const Duration &period,
                    void (Object::*callback)(const TimerEvent &),
                    Object *object) const {
    auto timer = node_->create_wall_timer(
        std::chrono::nanoseconds(period.nanoseconds()),
        [object, callback]() { (object->*callback)(TimerEvent{}); });
    return Timer(timer);
  }

  const rclcpp::Node::SharedPtr &node() const { return node_; }

 private:
  rclcpp::Node::SharedPtr node_;
};

inline void init(int, char **) {}
inline void shutdown() { rclcpp::shutdown(); }
inline void spin(const rclcpp::Node::SharedPtr &node) { rclcpp::spin(node); }

}  // namespace ros

#define ROS_INFO(...) RCLCPP_INFO(rclcpp::get_logger("frgraph"), __VA_ARGS__)
#define ROS_WARN(...) RCLCPP_WARN(rclcpp::get_logger("frgraph"), __VA_ARGS__)
#define ROS_ERROR(...) RCLCPP_ERROR(rclcpp::get_logger("frgraph"), __VA_ARGS__)
#define ROS_INFO_STREAM(...) RCLCPP_INFO_STREAM(rclcpp::get_logger("frgraph"), __VA_ARGS__)
#define ROS_ASSERT(condition) assert(condition)
