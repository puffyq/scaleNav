#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <memory>
#include <thread>
#include <vector>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "decomp_geometry/geometric_utils.h"
#include "planner_manager.h"
#include <ros/ros.h>

namespace {
volatile std::sig_atomic_t stop_requested = 0;
void handle_signal(int) { stop_requested = 1; }

geometry_msgs::msg::Point point(const Eigen::Vector3d &p)
{
  geometry_msgs::msg::Point out;
  out.x = p.x(); out.y = p.y(); out.z = p.z();
  return out;
}
}

class FrGraphRos2Node final : public rclcpp::Node {
 public:
  FrGraphRos2Node()
      : rclcpp::Node("frgraph_planner_manager"),
        planner_(std::make_shared<PlannerManager>()) {}

  void initializePlanner()
  {
    graph_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/frgraph/graph", 1);
    free_space_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        "/frgraph/free_space", 1);

    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/sim/odom");
    const auto goal_topic = declare_parameter<std::string>("goal_topic", "/goal");
    const auto goal_alias_topic = declare_parameter<std::string>("goal_alias_topic", "/goal_pose");
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic, rclcpp::SensorDataQoS(),
        [this](const nav_msgs::msg::Odometry::ConstSharedPtr msg) {
          odom_ = Eigen::Vector3d(msg->pose.pose.position.x,
                                  msg->pose.pose.position.y,
                                  msg->pose.pose.position.z);
          have_odom_ = true;
          planner_->setOdometry(*msg);
        });
    auto goal_callback = [this](const geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
      const Eigen::Vector3d new_goal(msg->pose.position.x,
                                     msg->pose.position.y,
                                     msg->pose.position.z);
      if (have_goal_ && (new_goal - goal_).norm() < 1e-3) return;
      goal_ = new_goal;
      have_goal_ = true;
      planner_->resetGraph();
      RCLCPP_INFO(get_logger(), "FRGraph goal set: %.2f %.2f %.2f",
                  goal_.x(), goal_.y(), goal_.z());
    };
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(goal_topic, 10, goal_callback);
    if (goal_alias_topic != goal_topic) {
      goal_alias_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
          goal_alias_topic, 10, goal_callback);
    }
    timer_ = create_wall_timer(std::chrono::milliseconds(100),
                               [this]() { update(); });
    ros::NodeHandle nh(shared_from_this());
    planner_->setEnvType(1);
    planner_->setSizeOfCroppedPointcloud(
        declare_parameter<double>("size_of_cropped_pointcloud", 20.0));
    planner_->initPlannerModule(nh);
  }

 private:
  void update()
  {
    const bool had_graph = last_edge_count_ > 0;
    const auto t_start = std::chrono::steady_clock::now();
    if (have_odom_ && have_goal_) planner_->buildGraphOnce(odom_, goal_);
    const auto t_graph = std::chrono::steady_clock::now();
    publishVisualization();
    const auto t_end = std::chrono::steady_clock::now();

    const double graph_ms = std::chrono::duration<double, std::milli>(
        t_graph - t_start).count();
    const double visualization_ms = std::chrono::duration<double, std::milli>(
        t_end - t_graph).count();
    const double total_ms = std::chrono::duration<double, std::milli>(
        t_end - t_start).count();
    ++update_count_;
    if (!had_graph && last_edge_count_ > 0) {
      RCLCPP_INFO(
          get_logger(),
          "[FRGraph timing] first graph: graph=%.3f ms visualization=%.3f ms total=%.3f ms nodes=%zu edges=%zu",
          graph_ms, visualization_ms, total_ms, last_node_count_, last_edge_count_);
    }
    RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "[FRGraph timing] update=%llu graph=%.3f ms visualization=%.3f ms total=%.3f ms nodes=%zu edges=%zu",
        static_cast<unsigned long long>(update_count_), graph_ms,
        visualization_ms, total_ms, last_node_count_, last_edge_count_);
  }

  void publishVisualization()
  {
    std::vector<Eigen::Vector3d> nodes;
    std::vector<PlannerManager::GraphVisualEdge> edges;
    planner_->getGraphVisualSnapshot(nodes, edges);
    last_node_count_ = nodes.size();
    last_edge_count_ = edges.size();

    visualization_msgs::msg::MarkerArray graph;
    visualization_msgs::msg::Marker node_marker;
    node_marker.header.frame_id = "odom";
    node_marker.header.stamp = now();
    node_marker.ns = "frgraph_nodes";
    node_marker.id = 0;
    node_marker.type = visualization_msgs::msg::Marker::SPHERE_LIST;
    node_marker.action = visualization_msgs::msg::Marker::ADD;
    node_marker.scale.x = 0.22;
    node_marker.scale.y = 0.22;
    node_marker.scale.z = 0.22;
    node_marker.color.r = 0.1;
    node_marker.color.g = 0.9;
    node_marker.color.b = 0.2;
    node_marker.color.a = 1.0;
    for (const auto &n : nodes) node_marker.points.push_back(point(n));
    graph.markers.push_back(node_marker);

    visualization_msgs::msg::Marker frontier_nodes = node_marker;
    frontier_nodes.ns = "frgraph_frontier_nodes";
    frontier_nodes.id = 3;
    frontier_nodes.points.clear();
    frontier_nodes.scale.x = 0.18;
    frontier_nodes.scale.y = 0.18;
    frontier_nodes.scale.z = 0.18;
    frontier_nodes.color.r = 1.0;
    frontier_nodes.color.g = 0.65;
    frontier_nodes.color.b = 0.05;
    for (const auto &e : edges) {
      if (e.frontier) frontier_nodes.points.push_back(point(e.to));
    }
    graph.markers.push_back(frontier_nodes);

    // The root is the actual current state node.  Frontier endpoints are
    // one-step candidate nodes; the global goal is shown separately because
    // the current-frame graph has not certified any edge to it yet.
    visualization_msgs::msg::Marker start_node = node_marker;
    start_node.ns = "frgraph_start_node";
    start_node.id = 6;
    start_node.points.clear();
    start_node.scale.x = 0.34;
    start_node.scale.y = 0.34;
    start_node.scale.z = 0.34;
    start_node.color.r = 0.1;
    start_node.color.g = 1.0;
    start_node.color.b = 0.2;
    if (have_odom_) start_node.points.push_back(point(odom_));
    graph.markers.push_back(start_node);

    visualization_msgs::msg::Marker edge_marker;
    edge_marker.header = node_marker.header;
    edge_marker.ns = "frgraph_edges";
    edge_marker.id = 1;
    edge_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    edge_marker.action = visualization_msgs::msg::Marker::ADD;
    edge_marker.scale.x = 0.045;
    edge_marker.color.r = 0.1;
    edge_marker.color.g = 0.6;
    edge_marker.color.b = 1.0;
    edge_marker.color.a = 1.0;
    for (const auto &e : edges) {
      edge_marker.points.push_back(point(e.from));
      edge_marker.points.push_back(point(e.to));
    }
    graph.markers.push_back(edge_marker);

    visualization_msgs::msg::Marker frontier_marker = edge_marker;
    frontier_marker.ns = "frgraph_frontier_edges";
    frontier_marker.id = 2;
    frontier_marker.points.clear();
    frontier_marker.color.r = 1.0;
    frontier_marker.color.g = 0.65;
    frontier_marker.color.b = 0.05;
    for (const auto &e : edges) {
      if (!e.frontier) continue;
      frontier_marker.points.push_back(point(e.from));
      frontier_marker.points.push_back(point(e.to));
    }
    graph.markers.push_back(frontier_marker);

    visualization_msgs::msg::Marker goal_marker;
    goal_marker.header = node_marker.header;
    goal_marker.ns = "frgraph_global_goal";
    goal_marker.id = 4;
    goal_marker.type = visualization_msgs::msg::Marker::SPHERE;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.pose.position = point(goal_);
    goal_marker.pose.orientation.w = 1.0;
    goal_marker.scale.x = 0.32;
    goal_marker.scale.y = 0.32;
    goal_marker.scale.z = 0.32;
    goal_marker.color.r = 0.75;
    goal_marker.color.g = 0.1;
    goal_marker.color.b = 0.9;
    goal_marker.color.a = 1.0;
    graph.markers.push_back(goal_marker);

    visualization_msgs::msg::Marker goal_node = goal_marker;
    goal_node.ns = "frgraph_goal_node";
    goal_node.id = 7;
    goal_node.scale.x = 0.48;
    goal_node.scale.y = 0.48;
    goal_node.scale.z = 0.48;
    graph.markers.push_back(goal_node);

    // No candidate-to-goal edge is emitted here. The current-frame FRGraph
    // only certifies its local free-space edge; a global search layer must
    // create and validate any later connection to the goal.
    visualization_msgs::msg::Marker optimistic_edges = edge_marker;
    optimistic_edges.ns = "frgraph_optimistic_edges";
    optimistic_edges.id = 8;
    optimistic_edges.points.clear();
    optimistic_edges.scale.x = 0.025;
    optimistic_edges.color.r = 0.75;
    optimistic_edges.color.g = 0.15;
    optimistic_edges.color.b = 0.95;
    optimistic_edges.color.a = 0.45;
    optimistic_edges.action = visualization_msgs::msg::Marker::DELETE;
    graph.markers.push_back(optimistic_edges);

    // The first edge is the primary FRGraph choice after goal-aware ranking.
    // Only the locally generated edge is shown; goal remains unvalidated.
    visualization_msgs::msg::Marker optimistic_path = edge_marker;
    optimistic_path.ns = "frgraph_optimistic_path";
    optimistic_path.id = 9;
    optimistic_path.type = visualization_msgs::msg::Marker::LINE_STRIP;
    optimistic_path.points.clear();
    optimistic_path.scale.x = 0.09;
    optimistic_path.color.r = 1.0;
    optimistic_path.color.g = 0.85;
    optimistic_path.color.b = 0.1;
    optimistic_path.color.a = 1.0;
    if (!edges.empty()) {
      optimistic_path.points.push_back(point(odom_));
      optimistic_path.points.push_back(point(edges.front().to));
    }
    graph.markers.push_back(optimistic_path);

    visualization_msgs::msg::Marker labels;
    labels.header = node_marker.header;
    labels.ns = "frgraph_node_labels";
    labels.id = 10;
    labels.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    labels.action = visualization_msgs::msg::Marker::ADD;
    labels.scale.z = 0.45;
    labels.color.r = 1.0;
    labels.color.g = 1.0;
    labels.color.b = 1.0;
    labels.color.a = 1.0;
    labels.pose.position = point(odom_);
    labels.pose.position.z += 0.45;
    labels.pose.orientation.w = 1.0;
    labels.text = "START";
    graph.markers.push_back(labels);

    labels.id = 11;
    labels.pose.position = point(goal_);
    labels.pose.position.z += 0.55;
    labels.text = "GOAL (unvalidated)";
    graph.markers.push_back(labels);

    if (!edges.empty()) {
      labels.id = 12;
      labels.pose.position = point(edges.front().to);
      labels.pose.position.z += 0.45;
      labels.text = "PRIMARY WAYPOINT";
      graph.markers.push_back(labels);
    }

    visualization_msgs::msg::Marker goal_direction;
    goal_direction.header = node_marker.header;
    goal_direction.ns = "frgraph_global_goal_direction";
    goal_direction.id = 5;
    goal_direction.type = visualization_msgs::msg::Marker::LINE_LIST;
    goal_direction.action = visualization_msgs::msg::Marker::ADD;
    goal_direction.scale.x = 0.035;
    goal_direction.color.r = 0.75;
    goal_direction.color.g = 0.1;
    goal_direction.color.b = 0.9;
    goal_direction.color.a = 0.9;
    goal_direction.points.push_back(point(odom_));
    goal_direction.points.push_back(point(goal_));
    graph.markers.push_back(goal_direction);
    graph_pub_->publish(graph);

    visualization_msgs::msg::MarkerArray free_space;
    int marker_id = 0;
    for (const auto &e : edges) {
      const auto faces = cal_vertices(e.corridor);
      for (const auto &face : faces) {
        if (face.size() < 2) continue;
        visualization_msgs::msg::Marker face_marker;
        face_marker.header = node_marker.header;
        face_marker.ns = "frgraph_free_regions";
        face_marker.id = marker_id++;
        face_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        face_marker.action = visualization_msgs::msg::Marker::ADD;
        face_marker.scale.x = 0.025;
        face_marker.color.r = e.frontier ? 1.0 : 0.15;
        face_marker.color.g = e.frontier ? 0.75 : 0.9;
        face_marker.color.b = 0.1;
        face_marker.color.a = 0.42;
        for (const auto &v : face) {
          face_marker.points.push_back(point(v.cast<double>()));
        }
        face_marker.points.push_back(face_marker.points.front());
        free_space.markers.push_back(std::move(face_marker));
      }
    }
    free_space_pub_->publish(free_space);
  }

  std::shared_ptr<PlannerManager> planner_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_alias_sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr graph_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr free_space_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  Eigen::Vector3d odom_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d goal_ = Eigen::Vector3d::Zero();
  bool have_odom_ = false;
  bool have_goal_ = false;
  std::size_t last_node_count_ = 0;
  std::size_t last_edge_count_ = 0;
  std::uint64_t update_count_ = 0;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv, rclcpp::InitOptions(),
               rclcpp::SignalHandlerOptions::None);
  std::signal(SIGINT, handle_signal);
  std::signal(SIGTERM, handle_signal);
  auto node = std::make_shared<FrGraphRos2Node>();
  node->initializePlanner();
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  while (!stop_requested && rclcpp::ok()) {
    executor.spin_some();
    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }
  executor.remove_node(node);
  rclcpp::shutdown();
  return 0;
}
