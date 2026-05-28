#include <rclcpp/rclcpp.hpp>

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
#include <queue>
#include <string>
#include <vector>

class GVD : public rclcpp::Node
{
public:
  GVD() : Node("gvd"), visualizer(this)
  {
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&GVD::odomCallback, this, std::placeholders::_1)
    );

    auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      map_qos,
      std::bind(&GVD::mapCallback, this, std::placeholders::_1)
    );

    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10
    );

    control_timer = this->create_wall_timer(
      std::chrono::milliseconds(LOOP_DT_MS),
      std::bind(&GVD::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "GVD node started.");
  }

private:
  // ---------------------- Callbacks -------------------------

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    robot_pos.x() = msg->pose.pose.position.x;
    robot_pos.y() = msg->pose.pose.position.y;
    has_odom = true;

    const double x = msg->pose.pose.orientation.x;
    const double y = msg->pose.pose.orientation.y;
    const double z = msg->pose.pose.orientation.z;
    const double w = msg->pose.pose.orientation.w;

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
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

    const std::size_t expected_size = received_width * received_height;
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

    brushfire_distance_grid.assign(expected_size, UNVISITED);
    brushfire_source_grid.assign(expected_size, NO_SOURCE);
    gvd_grid.assign(expected_size, NON_GVD_CELL);

    has_map = true;
    has_gvd = false;

    RCLCPP_INFO(
      this->get_logger(),
      "Loaded occupancy grid: %dx%d, resolution %.3f m/cell.",
      map_width,
      map_height,
      map_resolution
    );

    buildGVD();
  }

  // --------------------- Control Loop -----------------------

  void controlLoop()
  {
    if (!has_map) return;
    if (!has_odom) return;

    // TODO: implement GVD planning and control.
  }

  // --------------------- GVD Core ---------------------------

  void buildGVD()
  {
    // create queue shared by source labeling and brushfire expansion
    std::queue<int> brushfire_queue;

    labelObstacleSources(brushfire_queue);
    runBrushfire(brushfire_queue);
    publishGVDCells();

    // TODO: label obstacle sources, run brushfire, and mark GVD cells.
  }

  void labelObstacleSources(std::queue<int> &brushfire_queue)
  {
    const int map_size = map_width * map_height;

    // initialize grids
    brushfire_distance_grid.assign(map_size, UNVISITED);
    brushfire_source_grid.assign(map_size, NO_SOURCE);
    gvd_grid.assign(map_size, NON_GVD_CELL);

    int source_id = 0;
    for (int index = 0; index < map_size; ++index)
    {
      // skip free cells and already labeled obstacle components
      if (occupancy_grid[index] != BLOCKED_CELL) continue;
      if (brushfire_source_grid[index] != NO_SOURCE) continue;

      // start a new connected obstacle source
      std::queue<int> obstacle_queue;
      obstacle_queue.push(index);
      brushfire_source_grid[index] = source_id;
      brushfire_distance_grid[index] = 1;

      while (!obstacle_queue.empty())
      {
        // add every obstacle cell to the brushfire seed queue
        const int current = obstacle_queue.front();
        obstacle_queue.pop();
        brushfire_queue.push(current);

        // inspect 4-connected obstacle neighbours
        const int x = current % map_width;
        const int y = current / map_width;
        const std::vector<int> candidates = {
          (x + 1 < map_width) ? y * map_width + x + 1 : -1,
          (x - 1 >= 0) ? y * map_width + x - 1 : -1,
          (y + 1 < map_height) ? (y + 1) * map_width + x : -1,
          (y - 1 >= 0) ? (y - 1) * map_width + x : -1
        };

        for (int candidate : candidates)
        {
          // grow only through unlabeled occupied cells
          if (candidate < 0) continue;
          if (occupancy_grid[candidate] != BLOCKED_CELL) continue;
          if (brushfire_source_grid[candidate] != NO_SOURCE) continue;

          brushfire_source_grid[candidate] = source_id;
          brushfire_distance_grid[candidate] = 1;
          obstacle_queue.push(candidate);
        }
      }

      source_id++;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Labeled %d obstacle sources for GVD brushfire.",
      source_id
    );
  }

  void runBrushfire(std::queue<int> &brushfire_queue)
  {
    int gvd_cell_count = 0;

    while (!brushfire_queue.empty())
    {
      const int current = brushfire_queue.front();
      brushfire_queue.pop();

      // inspect 4-connected neighbours
      const int x = current % map_width;
      const int y = current / map_width;
      const std::vector<int> candidates = {
        (x + 1 < map_width) ? y * map_width + x + 1 : -1,
        (x - 1 >= 0) ? y * map_width + x - 1 : -1,
        (y + 1 < map_height) ? (y + 1) * map_width + x : -1,
        (y - 1 >= 0) ? (y - 1) * map_width + x : -1
      };

      for (int candidate : candidates)
      {
        if (candidate < 0) continue;

        // do not propagate through obstacle cells
        if (occupancy_grid[candidate] == BLOCKED_CELL)
        {
          if (brushfire_source_grid[candidate] != brushfire_source_grid[current])
          {
            if (occupancy_grid[current] == FREE_CELL &&
                gvd_grid[current] != GVD_CELL)
            {
              gvd_grid[current] = GVD_CELL;
              gvd_cell_count++;
            }
          }
          continue;
        }

        // first wavefront to reach this free cell assigns its distance/source
        if (brushfire_distance_grid[candidate] == UNVISITED)
        {
          brushfire_distance_grid[candidate] =
            brushfire_distance_grid[current] + 1;
          brushfire_source_grid[candidate] = brushfire_source_grid[current];
          brushfire_queue.push(candidate);
          continue;
        }

        // different source labels meeting define a Voronoi candidate
        if (brushfire_source_grid[candidate] != brushfire_source_grid[current])
        {
          if (gvd_grid[candidate] != GVD_CELL)
          {
            gvd_grid[candidate] = GVD_CELL;
            gvd_cell_count++;
          }
          if (occupancy_grid[current] == FREE_CELL &&
              gvd_grid[current] != GVD_CELL)
          {
            gvd_grid[current] = GVD_CELL;
            gvd_cell_count++;
          }
        }
      }
    }

    has_gvd = true;

    RCLCPP_INFO(
      this->get_logger(),
      "Brushfire GVD marked %d cells.",
      gvd_cell_count
    );
  }

  void publishGVDCells()
  {
    std::vector<int> gvd_cells;

    for (int index = 0; index < map_width * map_height; ++index)
    {
      if (gvd_grid[index] != GVD_CELL) continue;
      gvd_cells.push_back(index);
    }

    if (gvd_cells.empty())
    {
      RCLCPP_WARN(this->get_logger(), "No GVD cells to publish.");
      return;
    }

    visualizer.publishCells(
      "gvd_cells",
      gvd_cells,
      map_width,
      map_origin_x,
      map_origin_y,
      map_resolution,
      map_frame_id,
      0,
      0.0,
      1.0,
      1.0
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Published %zu GVD cells.",
      gvd_cells.size()
    );
  }

  // ------------------ Utility Functions ---------------------

  void sendVelocity(Eigen::Vector2d vel)
  {
    const double v_x = vel.x();
    const double v_y = vel.y();

    const double v = v_x * std::cos(robot_yaw) + v_y * std::sin(robot_yaw);
    const double w =
      (-v_x * std::sin(robot_yaw) + v_y * std::cos(robot_yaw)) / D;

    geometry_msgs::msg::Twist vel_twist;
    vel_twist.linear.x = v;
    vel_twist.angular.z = w;

    cmd_vel_pub->publish(vel_twist);
  }

  // --------------------- Variables --------------------------

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr      odom_sub;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr       cmd_vel_pub;
  rclcpp::TimerBase::SharedPtr                                  control_timer;

  Visualizer visualizer;

  // robot
  Eigen::Vector2d robot_pos;
  double          robot_yaw = 0.0;
  bool            has_odom = false;

  // map
  std::vector<uint8_t> occupancy_grid;
  std::string          map_frame_id;
  int                  map_width = 0;
  int                  map_height = 0;
  double               map_resolution = 0.0;
  double               map_origin_x = 0.0;
  double               map_origin_y = 0.0;
  bool                 has_map = false;

  // gvd
  std::vector<int>     brushfire_distance_grid;
  std::vector<int>     brushfire_source_grid;
  std::vector<uint8_t> gvd_grid;
  bool                 has_gvd = false;

  // consts
  const unsigned LOOP_DT_MS = 100;
  const double   D = 0.1;

  // labels
  const uint8_t  FREE_CELL = 0;
  const uint8_t  BLOCKED_CELL = 1;
  const uint8_t  NON_GVD_CELL = 0;
  const uint8_t  GVD_CELL = 1;
  const int      UNVISITED = -1;
  const int      NO_SOURCE = -1;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GVD>());
  rclcpp::shutdown();
  return 0;
}
