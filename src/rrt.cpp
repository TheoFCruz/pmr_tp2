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

    std::vector<uint8_t> raw_occupancy_grid(expected_size);
    for (std::size_t i = 0; i < expected_size; ++i)
    {
      raw_occupancy_grid[i] = (msg->data[i] == 0) ? FREE_CELL : BLOCKED_CELL;
    }

    occupancy_grid = inflateMap(raw_occupancy_grid);

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

    has_path = false;
    waypoint_i = 0;

    if (!has_map)
    {
      RCLCPP_WARN(this->get_logger(), "Cannot run RRT: map has not been loaded yet.");
      return;
    }

    if (!isPointInFreeCell(goal))
    {
      RCLCPP_WARN(
        this->get_logger(),
        "Ignoring goal: (%.2lf, %.2lf) is blocked or outside the inflated map.",
        goal.x(),
        goal.y()
      );
      received_goal = false;
      return;
    }

    visualizer.publishPoint("goal_marker", goal, "map", 0, 0.0, 1.0, 0.0);
    received_goal = true;

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
      const auto raw_path = runRRT(robot_pos, goal);
      path = simplifyPath(raw_path);
      has_path = !path.empty();
      waypoint_i = 0;
      if (!raw_path.empty()) visualizer.publishLineStrip("rrt_raw_path", raw_path, map_frame_id, 1, 0, 0, 0.1);
      if (has_path) visualizer.publishLineStrip("rrt_path", path, map_frame_id);

      sendVelocity(Eigen::Vector2d::Zero());
      return;
    }

    if ((robot_pos - goal).norm() <= ERROR_TH)
    {
      sendVelocity(Eigen::Vector2d::Zero());
      received_goal = false;
      has_path = false; 
      RCLCPP_INFO(this->get_logger(), "Reached goal. Stopping control");
      return;
    }

    Eigen::Vector2d waypoint = path[waypoint_i];
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

  std::vector<Eigen::Vector2d> simplifyPath(
    const std::vector<Eigen::Vector2d> &raw_path) const
  {
    if (raw_path.size() <= 2) return raw_path;

    std::vector<Eigen::Vector2d> simplified_path;
    simplified_path.push_back(raw_path.front());

    std::size_t current_index = 0;

    while (current_index < raw_path.size() - 1) {
      std::size_t best_next_index = current_index + 1;

      const std::size_t max_next_index = std::min(
        raw_path.size() - 1, current_index + MAX_SIMPLIFY_LOOKAHEAD);

      for (std::size_t candidate_index = max_next_index;
        candidate_index > current_index + 1;
        --candidate_index)
      {
        if (isSegmentValid(raw_path[current_index], raw_path[candidate_index])) {
          best_next_index = candidate_index;
          break;
        }
      }

      simplified_path.push_back(raw_path[best_next_index]);
      current_index = best_next_index;
    }

    return simplified_path;
  }

  // --------------------- RRT Core -----------------------

  std::vector<Eigen::Vector2d> runRRT(
    const Eigen::Vector2d &start,
    const Eigen::Vector2d &end)
  {
    // initialize both trees
    RRTTree start_tree{{start, -1}};
    RRTTree goal_tree{{end, -1}};
    bool active_tree_is_start_tree = true;

    for (int iteration = 0; iteration < MAX_RRT_ITERATIONS; ++iteration)
    {
      // extend the active tree
      const Eigen::Vector2d tree_target = active_tree_is_start_tree ? end : start;
      const Eigen::Vector2d random_sample = sampleRandomPoint(tree_target);
      const int new_node_index = extendTree(start_tree, random_sample);

      // check for tree connection
      const int connection_index_other_tree =
        tryConnectTrees(start_tree, goal_tree, new_node_index);

      if (connection_index_other_tree >= 0)
      {
        if (active_tree_is_start_tree)
        {
          // publish the logical start tree in blue
          publishTree("rrt_start_tree", start_tree, 0.0, 0.4, 1.0);

          // publish the logical goal tree in green
          publishTree("rrt_goal_tree", goal_tree, 0.0, 1.0, 0.0);

          return buildPathFromTrees(
            start_tree,
            new_node_index,
            goal_tree,
            connection_index_other_tree);
        }

        // publish the logical start tree in blue
        publishTree("rrt_start_tree", goal_tree, 0.0, 0.4, 1.0);

        // publish the logical goal tree in green
        publishTree("rrt_goal_tree", start_tree, 0.0, 1.0, 0.0);

        return buildPathFromTrees(
          goal_tree,
          connection_index_other_tree,
          start_tree,
          new_node_index);
      }

      // alternate tree growth
      std::swap(start_tree, goal_tree);
      active_tree_is_start_tree = !active_tree_is_start_tree;
    }

    return {};
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

    // uniform sampling in a box scaled around the map center
    const double map_size_x = map_width * map_resolution;
    const double map_size_y = map_height * map_resolution;
    const double sample_margin_x = (SAMPLE_AREA_SCALE - 1.0) * map_size_x / 2.0;
    const double sample_margin_y = (SAMPLE_AREA_SCALE - 1.0) * map_size_y / 2.0;

    std::uniform_real_distribution<double> sample_x(
      map_origin_x - sample_margin_x,
      map_origin_x + map_size_x + sample_margin_x
    );
    std::uniform_real_distribution<double> sample_y(
      map_origin_y - sample_margin_y,
      map_origin_y + map_size_y + sample_margin_y
    );

    return Eigen::Vector2d(sample_x(rng), sample_y(rng));
  }

  int nearestNodeIndex(const RRTTree &tree, const Eigen::Vector2d &point) const
  {
    int nearest_index = -1;
    double nearest_distance = std::numeric_limits<double>::infinity();

    // brute-force nearest search
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
    // compute capped step toward target
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
    // find nearest node
    const int nearest_index = nearestNodeIndex(tree, target);
    if (nearest_index < 0) return -1;

    // steer toward sample
    const Eigen::Vector2d nearest_point = tree[nearest_index].position;
    const Eigen::Vector2d new_point = steerToward(nearest_point, target);
    if (!isSegmentValid(nearest_point, new_point)) return -1;

    // add valid node
    tree.push_back({new_point, nearest_index});
    return static_cast<int>(tree.size()) - 1;
  }

  int tryConnectTrees(
    const RRTTree &tree_a,
    const RRTTree &tree_b,
    int new_node_index) const
  {
    // validate new node index
    if (new_node_index < 0 || new_node_index >= static_cast<int>(tree_a.size()))
    {
      return -1;
    }

    // find nearest node in other tree
    const int nearest_other_index = nearestNodeIndex(tree_b, tree_a[new_node_index].position);
    if (nearest_other_index < 0) return -1;

    // check connection distance
    const double connection_distance =
      (tree_a[new_node_index].position - tree_b[nearest_other_index].position).norm();

    if (connection_distance > CONNECT_DISTANCE)
    {
      return -1;
    }

    if (!isSegmentValid(tree_a[new_node_index].position, tree_b[nearest_other_index].position))
    {
      return -1;
    }

    return nearest_other_index;
  }

  std::vector<std::vector<Eigen::Vector2d>> treeToSegments(const RRTTree &tree) const
  {
    std::vector<std::vector<Eigen::Vector2d>> segments;

    // convert each parent-child edge into a line segment
    for (std::size_t i = 1; i < tree.size(); ++i)
    {
      const int parent_index = tree[i].parent;
      if (parent_index < 0) continue;

      segments.push_back({tree[parent_index].position, tree[i].position});
    }

    return segments;
  }

  void publishTree(
    const std::string &topic,
    const RRTTree &tree,
    double r,
    double g,
    double b)
  {
    visualizer.publishLineList(topic, treeToSegments(tree), map_frame_id, 0, r, g, b, 1.0, 0.02);
  }

  std::vector<Eigen::Vector2d> buildPathFromTrees(
    const RRTTree &start_tree,
    int start_tree_connection_index,
    const RRTTree &goal_tree,
    int goal_tree_connection_index) const
  {
    std::vector<Eigen::Vector2d> start_path;
    std::vector<Eigen::Vector2d> goal_path;

    // trace the start tree back to the root
    int current_index = start_tree_connection_index;
    while (current_index >= 0)
    {
      start_path.push_back(start_tree[current_index].position);
      current_index = start_tree[current_index].parent;
    }

    // reverse the start-side path to go from root to connection
    std::reverse(start_path.begin(), start_path.end());

    // trace the goal tree back to the root
    current_index = goal_tree_connection_index;
    while (current_index >= 0)
    {
      goal_path.push_back(goal_tree[current_index].position);
      current_index = goal_tree[current_index].parent;
    }

    // append the goal-side path to complete the route
    start_path.insert(start_path.end(), goal_path.begin(), goal_path.end());
    return start_path;
  }

  bool isSegmentValid(
    const Eigen::Vector2d &start,
    const Eigen::Vector2d &end) const
  {
    // reject segments when map data is not ready
    if (!has_map || map_resolution <= 0.0 || occupancy_grid.empty()) return false;

    // compute how many points are needed to sample the segment
    const Eigen::Vector2d direction = end - start;
    const double distance = direction.norm();
    const int samples = std::max(1, static_cast<int>(std::ceil(distance / COLLISION_CHECK_STEP)));

    // interpolate along the segment and check each sampled point
    for (int i = 0; i <= samples; ++i)
    {
      const double t = static_cast<double>(i) / samples;
      const Eigen::Vector2d point = start + t * direction;

      if (!isPointInFreeCell(point)) return false;
    }

    return true;
  }

  bool isPointInFreeCell(const Eigen::Vector2d &point) const
  {
    // reject points when map data is not ready
    if (!has_map || map_resolution <= 0.0 || occupancy_grid.empty()) return false;

    // convert world coordinates to grid coordinates
    const int cell_x = static_cast<int>(std::floor((point.x() - map_origin_x) / map_resolution));
    const int cell_y = static_cast<int>(std::floor((point.y() - map_origin_y) / map_resolution));

    // reject points outside the known map bounds
    if (cell_x < 0 || cell_x >= map_width || cell_y < 0 || cell_y >= map_height)
    {
      return false;
    }

    // accept only cells that are free in the inflated map
    const std::size_t cell_index = static_cast<std::size_t>(cell_y) * map_width + cell_x;
    return occupancy_grid[cell_index] == FREE_CELL;
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

  std::vector<uint8_t> inflateMap(const std::vector<uint8_t> &raw_grid) const
  {
    // start from the original occupancy grid
    std::vector<uint8_t> inflated_grid = raw_grid;

    if (map_resolution <= 0.0) return inflated_grid;

    // convert the robot radius from meters to grid cells
    const int inflation_cells = static_cast<int>(std::ceil(ROBOT_RADIUS / map_resolution));
    const int inflation_cells_squared = inflation_cells * inflation_cells;

    // visit each cell looking for obstacles to inflate
    for (int y = 0; y < map_height; ++y)
    {
      for (int x = 0; x < map_width; ++x)
      {
        const std::size_t cell_index = static_cast<std::size_t>(y) * map_width + x;
        if (raw_grid[cell_index] == FREE_CELL) continue;

        // mark neighboring cells inside the circular inflation radius
        for (int dy = -inflation_cells; dy <= inflation_cells; ++dy)
        {
          for (int dx = -inflation_cells; dx <= inflation_cells; ++dx)
          {
            if (dx * dx + dy * dy > inflation_cells_squared) continue;

            // skip inflated cells that would fall outside the map
            const int inflated_x = x + dx;
            const int inflated_y = y + dy;
            if (inflated_x < 0 || inflated_x >= map_width || inflated_y < 0 || inflated_y >= map_height)
            {
              continue;
            }

            const std::size_t inflated_index =
              static_cast<std::size_t>(inflated_y) * map_width + inflated_x;
            inflated_grid[inflated_index] = BLOCKED_CELL;
          }
        }
      }
    }

    return inflated_grid;
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
  const double KP = 3.0;
  const double MAX_SPEED = 1.0;
  const double ERROR_TH = 0.25;
  const int MAX_RRT_ITERATIONS = 1000;
  const double STEP_SIZE = 0.1;
  const double CONNECT_DISTANCE = 0.5;
  const double GOAL_SAMPLE_PROBABILITY = 0.1;
  const double COLLISION_CHECK_STEP = 0.02;
  const double ROBOT_RADIUS = 0.20;
  const double SAMPLE_AREA_SCALE = 2.0;
  const std::size_t MAX_SIMPLIFY_LOOKAHEAD = 5;

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
