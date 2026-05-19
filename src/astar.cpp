#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "pmr_tp2/visualizer.hpp"

#include <eigen3/Eigen/Dense>

#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class AStar : public rclcpp::Node
{
public:
  AStar() : Node("a_star"), visualizer(this)
  {
    // publishers and subscribers
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&AStar::odomCallback, this, std::placeholders::_1)
    );

    goal_sub  = this->create_subscription<geometry_msgs::msg::Point>(
      "/goal",
      10,
      std::bind(&AStar::goalCallback, this, std::placeholders::_1)
    );

    auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      map_qos,
      std::bind(&AStar::mapCallback, this, std::placeholders::_1)
    );

    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10
    );

    // timer for the control loop
    control_timer = this->create_wall_timer(
      std::chrono::milliseconds(LOOP_DT_MS),
      std::bind(&AStar::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "A* node started.");
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

    const size_t expected_size = received_width * received_height;

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

    if (!has_map)
    {
      RCLCPP_WARN(this->get_logger(), "Cannot run A*: map has not been loaded yet.");
      return;
    }

    if (!has_odom)
    {
      RCLCPP_WARN(this->get_logger(), "Cannot run A*: robot pose has not been received yet.");
      return;
    }

    path = runAStar(robot_pos, goal);
    has_path = !path.empty();

    if (has_path) visualizer.publishPath(path, map_frame_id);
  }

  // --------------------- Control Loop -----------------------
  
  void controlLoop()
  {
    if (!has_map) return;
    if (!received_goal) return;
    if (!has_path) return;

    // TODO: implement control loop
  }

  // --------------------- Astar Core ------------------------

  struct Cell
  {
    int x;
    int y;
  };

  Cell worldToGrid(const Eigen::Vector2d &point)
  {
    Cell cell;
    cell.x = std::floor((point.x() - map_origin_x) / map_resolution);
    cell.y = std::floor((point.y() - map_origin_y) / map_resolution);

    return cell;
  }

  Eigen::Vector2d gridToWorld(const Cell &cell)
  {
    const double x = map_origin_x + (cell.x + 0.5) * map_resolution;
    const double y = map_origin_y + (cell.y + 0.5) * map_resolution;

    return Eigen::Vector2d(x, y);
  }

  std::vector<Eigen::Vector2d> runAStar(
    const Eigen::Vector2d &start,
    const Eigen::Vector2d &end)
  {
    (void) start;
    (void) end;

    // TODO: implement A*.
    
    // create priority queue O and vector C
    //    priority = distance + heuristic
    // add start to O
    //
    // while O is not empty:
    //    remove lowest priority vertex u from O
    //    if u == end: stop
    //    if u not in C:
    //      add u to C
    //      add all neighbours from u with better new costs to O
    return {};
  }

  // ------------------ Utility Functions ---------------------

  void sendVelocity(Eigen::Vector2d vel)
  {
    double v_x = vel.x();
    double v_y = vel.y();

    // feedback linearization
    double v = (v_x * std::cos(robot_yaw)) + (v_y * std::sin(robot_yaw));
    double w = (-v_x * std::sin(robot_yaw) + v_y * std::cos(robot_yaw)) / D;

    // ros2 msg
    geometry_msgs::msg::Twist vel_twist;
    vel_twist.linear.x = v;
    vel_twist.angular.z = w;

    cmd_vel_pub->publish(vel_twist);
  }

  // --------------------- Variables --------------------------

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub;
  rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr    goal_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       cmd_vel_pub;
  rclcpp::TimerBase::SharedPtr                                  control_timer;

  Visualizer visualizer;

  // robot and goal
  Eigen::Vector2d goal;
  Eigen::Vector2d robot_pos;
  double          robot_yaw;
  bool            received_goal = false;
  bool            has_odom = false;

  // path
  std::vector<Eigen::Vector2d> path;
  bool                         has_path = false;

  // map
  std::vector<uint8_t> occupancy_grid;
  std::string          map_frame_id;
  int                  map_width = 0;
  int                  map_height = 0;
  double               map_resolution = 0.0;
  double               map_origin_x = 0.0;
  double               map_origin_y = 0.0;
  bool                 has_map = false;

  // consts
  const unsigned LOOP_DT_MS = 100;
  const double   D = 0.1;
  const uint8_t  FREE_CELL = 0;
  const uint8_t  BLOCKED_CELL = 1;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AStar>());
  rclcpp::shutdown();
  return 0;
}
