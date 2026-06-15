#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "pmr_tp2/visualizer.hpp"

#include <eigen3/Eigen/Dense>

#include <chrono>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <memory>
#include <random>
#include <string>
#include <vector>

class RRT : public rclcpp::Node
{
public:
  RRT() : Node("rrt"), visualizer(this)
  {
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&RRT::odomCallback, this, std::placeholders::_1)
    );

    goal_sub = this->create_subscription<geometry_msgs::msg::Point>(
      "/goal",
      10,
      std::bind(&RRT::goalCallback, this, std::placeholders::_1)
    );

    auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      map_qos,
      std::bind(&RRT::mapCallback, this, std::placeholders::_1)
    );

    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    control_timer = this->create_wall_timer(
      std::chrono::milliseconds(LOOP_DT_MS),
      std::bind(&RRT::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "RRT node started.");
  }

private:
  struct RRTNode
  {
    Eigen::Vector2d position;
    int parent = -1;
  };

  using RRTTree = std::vector<RRTNode>;

  // ---------------------- Callbacks -------------------------

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // get position
    robot_pos.x() = msg->pose.pose.position.x;
    robot_pos.y() = msg->pose.pose.position.y;
    has_odom = true;

    // get quaternion and extract yaw
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;

    double siny_cosp = 2.0 * (w * z + x * y);
    double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    robot_yaw = std::atan2(siny_cosp, cosy_cosp);
  }

  void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
  {
    const int received_width = msg->info.width;
    const int received_height = msg->info.height;

    if (received_width <= 0 || received_height <= 0)
    {
      RCLCPP_WARN(this->get_logger(), "Received an empty occupancy grid.");
      return;
    }

    const std::size_t expected_size = static_cast<std::size_t>(received_width) * received_height;
    if (msg->data.size() != expected_size)
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Occupancy grid data size (%zu) does not match width * height (%zu).",
        msg->data.size(),
        expected_size
      );
      return;
    }

    map_width = received_width;
    map_height = received_height;
    map_resolution = msg->info.resolution;
    map_origin_x = msg->info.origin.position.x;
    map_origin_y = msg->info.origin.position.y;
    map_frame_id = msg->header.frame_id;

    occupancy_grid.resize(expected_size);
    for (std::size_t i = 0; i < expected_size; ++i)
    {
      occupancy_grid[i] = (msg->data[i] == 0) ? FREE_CELL : BLOCKED_CELL;
    }

    has_map = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded occupancy grid: %dx%d, resolution %.3f m/cell.",
      map_width,
      map_height,
      map_resolution
    );
  }

  void goalCallback(geometry_msgs::msg::Point::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: (%.2lf, %.2lf)", msg->x, msg->y);

    goal = Eigen::Vector2d(msg->x, msg->y);
    visualizer.publishPoint("goal_marker", goal, "map", 0, 0.0, 1.0, 0.0);
    received_goal = true;

    has_path = false;
    waypoint_i = 0;

    if (!has_map)
    {
      RCLCPP_WARN(this->get_logger(), "Cannot run RRT: map has not been loaded yet.");
      return;
    }

    if (!has_odom)
    {
      RCLCPP_WARN(this->get_logger(), "Cannot run RRT: robot pose has not been received yet.");
      return;
    }
  }

  // --------------------- Control Loop -----------------------

  void controlLoop()
  {
    if (!has_map) return;
    if (!has_odom) return;
    if (!received_goal) return;

    if (!has_path)
    {
      path = runRRT(robot_pos, goal);
      has_path = !path.empty();
      waypoint_i = 0;
      if (has_path) visualizer.publishPath(path, map_frame_id);
    }

    sendVelocity(Eigen::Vector2d::Zero());
  }

  std::vector<Eigen::Vector2d> runRRT(
    const Eigen::Vector2d &start,
    const Eigen::Vector2d &end)
  {
    RRTTree start_tree = initializeTree(start);
    RRTTree goal_tree = initializeTree(end);
    bool active_tree_is_start_tree = true;

    for (int iteration = 0; iteration < MAX_RRT_ITERATIONS; ++iteration)
    {
      const Eigen::Vector2d tree_target = active_tree_is_start_tree ? end : start;
      const Eigen::Vector2d random_sample = sampleRandomPoint(tree_target);
      const int new_node_index = extendTree(start_tree, random_sample);
      if (new_node_index >= 0 && tryConnectTrees(start_tree, goal_tree, new_node_index))
      {
        return buildPathFromTrees(start_tree, goal_tree);
      }

      std::swap(start_tree, goal_tree);
      active_tree_is_start_tree = !active_tree_is_start_tree;
    }

    RCLCPP_INFO(this->get_logger(), "RRT planning placeholder: tree/path generation not implemented yet.");
    return {};
  }

  RRTTree initializeTree(const Eigen::Vector2d &root) const
  {
    return RRTTree{{root, -1}};
  }

  Eigen::Vector2d sampleRandomPoint(const Eigen::Vector2d &tree_target) const
  {
    // bias to sample the tree target
    std::uniform_real_distribution<double> target_probability(0.0, 1.0);
    if (target_probability(rng) < GOAL_SAMPLE_PROBABILITY) return tree_target;

    // map guard
    if (!has_map || map_resolution <= 0.0)
    {
      return Eigen::Vector2d::Zero();
    }

    // uniform sampling
    std::uniform_real_distribution<double> sample_x(
      map_origin_x,
      map_origin_x + map_width * map_resolution
    );
    std::uniform_real_distribution<double> sample_y(
      map_origin_y,
      map_origin_y + map_height * map_resolution
    );

    return Eigen::Vector2d(sample_x(rng), sample_y(rng));
  }

  int nearestNodeIndex(const RRTTree &tree, const Eigen::Vector2d &point) const
  {
    int nearest_index = -1;
    double nearest_distance = std::numeric_limits<double>::infinity();

    for (int i = 0; i < static_cast<int>(tree.size()); ++i)
    {
      const double distance = (tree[i].position - point).squaredNorm();
      if (distance < nearest_distance)
      {
        nearest_distance = distance;
        nearest_index = i;
      }
    }

    return nearest_index;
  }

  Eigen::Vector2d steerToward(
    const Eigen::Vector2d &from,
    const Eigen::Vector2d &to) const
  {
    const Eigen::Vector2d direction = to - from;
    const double distance = direction.norm();
    if (distance <= STEP_SIZE) return to;
    if (distance <= 0.0) return from;

    return from + direction / distance * STEP_SIZE;
  }

  int extendTree(
    RRTTree &tree,
    const Eigen::Vector2d &target)
  {
    const int nearest_index = nearestNodeIndex(tree, target);
    if (nearest_index < 0) return -1;

    const Eigen::Vector2d new_point = steerToward(tree[nearest_index].position, target);
    if (!isPointValid(new_point)) return -1;

    tree.push_back({new_point, nearest_index});
    return static_cast<int>(tree.size()) - 1;
  }

  bool tryConnectTrees(
    const RRTTree &tree_a,
    const RRTTree &tree_b,
    int new_node_index)
  {
    if (new_node_index < 0 || new_node_index >= static_cast<int>(tree_a.size()))
    {
      return false;
    }

    const int nearest_other_index = nearestNodeIndex(tree_b, tree_a[new_node_index].position);
    if (nearest_other_index < 0) return false;

    const double connection_distance =
      (tree_a[new_node_index].position - tree_b[nearest_other_index].position).norm();

    return connection_distance <= CONNECT_DISTANCE;
  }

  std::vector<Eigen::Vector2d> buildPathFromTrees(
    const RRTTree &start_tree,
    const RRTTree &goal_tree) const
  {
    (void)start_tree;
    (void)goal_tree;
    return {};
  }

  bool isPointValid(const Eigen::Vector2d &point) const
  {
    (void)point;
    return true;
  }

  // ------------------ Utility Functions ---------------------

  void sendVelocity(Eigen::Vector2d vel)
  {
    const double v_x = vel.x();
    const double v_y = vel.y();

    const double v = v_x * std::cos(robot_yaw) + v_y * std::sin(robot_yaw);
    const double w = (-v_x * std::sin(robot_yaw) + v_y * std::cos(robot_yaw)) / D;

    geometry_msgs::msg::Twist vel_twist;
    vel_twist.linear.x = v;
    vel_twist.angular.z = w;

    cmd_vel_pub->publish(vel_twist);
  }

  // --------------------- Variables --------------------------

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr goal_sub;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub;
  rclcpp::TimerBase::SharedPtr control_timer;

  Visualizer visualizer;

  Eigen::Vector2d robot_pos;
  Eigen::Vector2d goal;
  double robot_yaw = 0.0;
  bool has_odom = false;
  bool received_goal = false;

  std::vector<uint8_t> occupancy_grid;
  std::string map_frame_id;
  int map_width = 0;
  int map_height = 0;
  double map_resolution = 0.0;
  double map_origin_x = 0.0;
  double map_origin_y = 0.0;
  bool has_map = false;

  std::vector<Eigen::Vector2d> path;
  std::size_t waypoint_i = 0;
  bool has_path = false;

  const unsigned LOOP_DT_MS = 100;
  const double D = 0.1;
  const int MAX_RRT_ITERATIONS = 1000;
  const double STEP_SIZE = 0.3;
  const double CONNECT_DISTANCE = 0.4;
  const double GOAL_SAMPLE_PROBABILITY = 0.1;

  const uint8_t FREE_CELL = 0;
  const uint8_t BLOCKED_CELL = 1;

  mutable std::mt19937 rng{std::random_device{}()};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RRT>());
  rclcpp::shutdown();
  return 0;
}
