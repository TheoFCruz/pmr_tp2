#include <rclcpp/rclcpp.hpp>

#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/point.hpp>

#include "pmr_tp2/visualizer.hpp"

#include <eigen3/Eigen/Dense>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <limits>
#include <map>
#include <memory>
#include <queue>
#include <stdexcept>
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
    waypoint_i = 0;

    if (has_path) visualizer.publishPath(path, map_frame_id);
  }

  // --------------------- Control Loop -----------------------
  
  void controlLoop()
  {
    if (!has_map) return;
    if (!received_goal) return;
    if (!has_path) return;
    
    if ((robot_pos - goal).norm() <= ERROR_TH)
    {
      sendVelocity({0, 0});
      received_goal = false;
      has_path = false; 
      RCLCPP_INFO(this->get_logger(), "Reached goal. Stopping control");
      return;
    }

    std::vector<Eigen::Vector2d> followed_path = simplifyPath(path);
    followed_path.push_back(goal);

    Eigen::Vector2d waypoint = followed_path[waypoint_i];

    Eigen::Vector2d heading = {cos(robot_yaw), sin(robot_yaw)};
    Eigen::Vector2d x_d = robot_pos + D*heading;
    Eigen::Vector2d vel = (waypoint - x_d)*KP;

    if (vel.norm() > MAX_SPEED) vel = vel.normalized()*MAX_SPEED;
    sendVelocity(vel);

    if ((waypoint - robot_pos).norm() <= ERROR_TH)
    {
      waypoint_i++;
    }
  }

  // --------------------- Astar Core ------------------------

  struct CellEntry
  {
    int index;
    double distance = -1.0;
    double heuristic = -1.0;
    int came_from = -1;
  };

  struct CompareCellEntry
  {
    bool operator()(const CellEntry &a, const CellEntry &b)
    {
      if (a.distance < 0.0 || b.distance < 0.0)
      {
        throw std::logic_error("A* open cell has invalid distance.");
      }
      if (a.heuristic < 0.0 || b.heuristic < 0.0)
      {
        throw std::logic_error("A* open cell has invalid heuristic.");
      }

      return a.distance + a.heuristic > b.distance + b.heuristic;
    }
  };

  int worldToIndex(const Eigen::Vector2d &point)
  {
    const int x = std::floor((point.x() - map_origin_x) / map_resolution);
    const int y = std::floor((point.y() - map_origin_y) / map_resolution);

    if (!isInsideMap(x, y)) return -1;

    return y * map_width + x;
  }

  Eigen::Vector2d indexToWorld(int index)
  {
    const double x = map_origin_x + (index % map_width + 0.5) * map_resolution;
    const double y = map_origin_y + (index / map_width + 0.5) * map_resolution;

    return Eigen::Vector2d(x, y);
  }

  bool isValidCell(int index)
  {
    if (index < 0) return false;
    if (index >= static_cast<int>(occupancy_grid.size())) return false;
    if (occupancy_grid[index] != FREE_CELL) return false;

    return true;
  }

  bool isInsideMap(int x, int y)
  {
    return x >= 0 && x < map_width &&
      y >= 0 && y < map_height;
  }

  std::vector<int> getNeighbours(int index)
  {
    std::vector<int> neighbours;
    const int x = index % map_width;
    const int y = index / map_width;

    const std::vector<int> candidates = {
      isInsideMap(x + 1, y) ? y * map_width + x + 1 : -1,
      isInsideMap(x - 1, y) ? y * map_width + x - 1 : -1,
      isInsideMap(x, y + 1) ? (y + 1) * map_width + x : -1,
      isInsideMap(x, y - 1) ? (y - 1) * map_width + x : -1
    };

    for (int candidate : candidates)
    {
      if (isValidCell(candidate)) neighbours.push_back(candidate);
    }

    return neighbours;
  }

  double calculateHeuristic(int index, int end_index)
  {
    const int x = index % map_width;
    const int y = index / map_width;
    const int end_x = end_index % map_width;
    const int end_y = end_index / map_width;

    return std::abs(x - end_x) + std::abs(y - end_y);
  }

  std::vector<Eigen::Vector2d> buildPath(
    const std::map<int, CellEntry>& closed,
    int start_index,
    int end_index)
  {
    std::vector<Eigen::Vector2d> path_points;
    std::vector<int> path_indices;

    int current_index = end_index;
    while (current_index != -1)
    {
      path_indices.push_back(current_index);
      if (current_index == start_index) break;

      current_index = closed.at(current_index).came_from;
    }

    if (path_indices.back() != start_index) return {};

    std::reverse(path_indices.begin(), path_indices.end());

    for (int index : path_indices)
    {
      path_points.push_back(indexToWorld(index));
    }
    return path_points;
  }

  std::vector<Eigen::Vector2d> runAStar(
    const Eigen::Vector2d &start,
    const Eigen::Vector2d &end)
  {
    // Convert start and goal from world coordinates to grid indices.
    const int start_index = worldToIndex(start);
    const int end_index = worldToIndex(end);

    // Abort early if either endpoint is outside the map or occupied.
    if (!isValidCell(start_index) || !isValidCell(end_index))
    {
      RCLCPP_WARN(this->get_logger(), "A* start or goal cell is not valid.");
      return {};
    }

    // Prepare the open queue and closed set.
    std::priority_queue< CellEntry, std::vector<CellEntry>, CompareCellEntry> open;
    std::map<int, CellEntry> closed;

    // Seed the search with the start cell.
    open.push({start_index, 0, calculateHeuristic(start_index, end_index), -1});

    while (!open.empty())
    {
      // Expand the currently most promising cell.
      const CellEntry current = open.top();
      open.pop();

      // Skip stale queue entries for cells already expanded.
      if (closed.find(current.index) != closed.end()) continue;

      // Add current cell to closed.
      closed[current.index] = current;

      // Stop when the goal is reached and reconstruct the path.
      if (current.index == end_index) return buildPath(closed, start_index, end_index);

      // Relax all free non-closed neighbours.
      for (int neighbour_index : getNeighbours(current.index))
      {
        if (closed.find(neighbour_index) != closed.end()) continue;

        // Add neighbour to open set
        CellEntry neighbour = {
          neighbour_index,
          current.distance + 1.0,
          calculateHeuristic(neighbour_index, end_index),
          current.index
        };
        // Duplicates are ok: the first popped entry for an index is the best one.
        open.push(neighbour);
      }
    }

    RCLCPP_WARN(this->get_logger(), "A* could not find a path.");
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

  std::vector<Eigen::Vector2d> simplifyPath(const std::vector<Eigen::Vector2d>& path)
  {
    std::vector<Eigen::Vector2d> result;

    result.push_back(path[0]);
    for (size_t i = 1; i < path.size()-1; i++)
    {
      if (path[i].x() == path[i-1].x() && path[i].x() == path[i+1].x()) continue;
      if (path[i].y() == path[i-1].y() && path[i].y() == path[i+1].y()) continue;
      result.push_back(path[i]);
    }
    result.push_back(path[path.size()-1]);

    return result;
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
  size_t                       waypoint_i;
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
  const double   KP = 3.0;
  const double   MAX_SPEED = 1.0;
  const double   ERROR_TH = 0.25;
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
