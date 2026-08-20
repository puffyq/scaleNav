#include <atomic>
#include <chrono>
#include <csignal>
#include <memory>
#include <thread>

#include <rclcpp/rclcpp.hpp>

#include "gap_extractor/gap_extractor.h"
#include <ros/ros.h>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void handle_signal(int) { stop_requested = 1; }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
  auto node = std::make_shared<rclcpp::Node>("frgraph_gap_extractor");
  ros::NodeHandle node_handle(node);
  auto extractor = std::make_shared<GapExtractor>();
  extractor->initialize(node_handle, true);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (!stop_requested && rclcpp::ok()) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  executor.remove_node(node);
  extractor.reset();
  rclcpp::shutdown();
  return 0;
}
