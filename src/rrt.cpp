#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include "pmr_tp2/visualizer.hpp"

#include <eigen3/Eigen/Dense>

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
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

    RCLCPP_INFO(this->get_logger(), "RRT planning placeholder: tree/path generation not implemented yet.");
  }

  // --------------------- Control Loop -----------------------

  void controlLoop()
  {
    if (!has_map) return;
    if (!has_odom) return;
    if (!received_goal) return;

    sendVelocity(Eigen::Vector2d::Zero());
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

  const uint8_t FREE_CELL = 0;
  const uint8_t BLOCKED_CELL = 1;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RRT>());
  rclcpp::shutdown();
  return 0;
}
