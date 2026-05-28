#include <rclcpp/rclcpp.hpp>

#include <algorithm>
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
#include <limits>
#include <memory>
#include <queue>
#include <string>
#include <utility>
#include <vector>

class GVD : public rclcpp::Node
{
public:
  GVD() : Node("gvd"), visualizer(this)
  {
    // subscribe to odometry for robot pose feedback
    odom_sub = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odom",
      10,
      std::bind(&GVD::odomCallback, this, std::placeholders::_1)
    );

    // subscribe to the goal point
    goal_sub = this->create_subscription<geometry_msgs::msg::Point>(
      "/goal",
      10,
      std::bind(&GVD::goalCallback, this, std::placeholders::_1)
    );

    // subscribe to the latched occupancy grid map
    auto map_qos = rclcpp::QoS(1).transient_local().reliable();
    map_sub = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
      "/map",
      map_qos,
      std::bind(&GVD::mapCallback, this, std::placeholders::_1)
    );

    // publish velocity commands to the robot
    cmd_vel_pub = this->create_publisher<geometry_msgs::msg::Twist>(
      "/cmd_vel",
      10
    );

    // run the control loop periodically
    control_timer = this->create_wall_timer(
      std::chrono::milliseconds(LOOP_DT_MS),
      std::bind(&GVD::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "GVD node started.");
  }

private:
  struct GraphNode
  {
    Eigen::Vector2d position;
    std::vector<int> edge_ids;
  };

  struct GraphEdge
  {
    int from_node;
    int to_node;
    std::vector<int> cells;
    std::vector<Eigen::Vector2d> path;
    double cost = 0.0;
  };

  // ---------------------- Callbacks -------------------------

  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // store the robot position
    robot_pos.x() = msg->pose.pose.position.x;
    robot_pos.y() = msg->pose.pose.position.y;
    has_odom = true;

    // convert the odometry quaternion to yaw
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
    // validate the received map dimensions
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

    // store the map metadata
    map_width = received_width;
    map_height = received_height;
    map_resolution = msg->info.resolution;
    map_origin_x = msg->info.origin.position.x;
    map_origin_y = msg->info.origin.position.y;
    map_frame_id = msg->header.frame_id;

    // convert occupancy values to the internal free/blocked representation
    occupancy_grid.resize(expected_size);
    for (std::size_t i = 0; i < expected_size; ++i)
    {
      occupancy_grid[i] = (msg->data[i] == 0) ? FREE_CELL : BLOCKED_CELL;
    }

    // reset the brushfire and GVD grids
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

    // rebuild all derived structures from the new map
    buildGVD();
  }

  void goalCallback(geometry_msgs::msg::Point::SharedPtr msg)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal: (%.2lf, %.2lf)", msg->x, msg->y);

    goal = Eigen::Vector2d(msg->x, msg->y);
    visualizer.publishPoint("goal_marker", goal, "map", 0, 0.5, 0.0, 0.5);
    received_goal = true;

    if (!has_map)
    {
      RCLCPP_WARN(this->get_logger(), "Cannot run Dijkstra: map has not been loaded yet.");
      return;
    }
    if (!has_odom)
    {
      RCLCPP_WARN(this->get_logger(), "Cannot run Dijkstra: robot pose has not been received yet.");
      return;
    }
    if (graph_nodes.empty())
    {
      RCLCPP_WARN(this->get_logger(), "Cannot run Dijkstra: graph has no nodes.");
      return;
    }

    const int start_node = nearestGraphNode(robot_pos);
    const int goal_node = nearestGraphNode(goal);
    const std::vector<Eigen::Vector2d> graph_path = runDijkstra(start_node, goal_node);

    if (graph_path.empty())
    {
      followed_path.clear();
      has_path = false;
      return;
    }

    followed_path.clear();
    followed_path.push_back(robot_pos);
    followed_path.insert(followed_path.end(), graph_path.begin(), graph_path.end());
    followed_path.push_back(goal);
    has_path = true;
    waypoint_i = 0;

    visualizer.publishPath(followed_path, map_frame_id);
  }

  // --------------------- Control Loop -----------------------

  void controlLoop()
  {
    // wait until both map and odometry are available
    if (!has_map) return;
    if (!has_odom) return;

    // TODO: implement GVD planning and control.
  }

  // --------------------- GVD Core ---------------------------

  void buildGVD()
  {
    // create queue shared by source labeling and brushfire expansion
    std::queue<int> brushfire_queue;

    // label obstacle sources before growing the distance field
    labelObstacleSources(brushfire_queue);

    // expand the brushfire and mark raw GVD cells
    runBrushfire(brushfire_queue);

    // publish the raw GVD and graph node candidates
    detectGraphNodes();
    extractGraphEdges();
    publishGVDCells();
    publishGraphEdges();
  }

  void labelObstacleSources(std::queue<int> &brushfire_queue)
  {
    const int map_size = map_width * map_height;

    // initialize grids
    brushfire_distance_grid.assign(map_size, UNVISITED);
    brushfire_source_grid.assign(map_size, NO_SOURCE);
    gvd_grid.assign(map_size, NON_GVD_CELL);

    // label borders separately
    for (int index = 0; index < map_size; ++index)
    {
      if (occupancy_grid[index] != BLOCKED_CELL) continue;

      const int source_id = borderSourceId(index);
      if (source_id == NO_SOURCE) continue;

      brushfire_source_grid[index] = source_id;
      brushfire_distance_grid[index] = 1;
      brushfire_queue.push(index);
    }

    int source_id = BORDER_SOURCE_COUNT;
    for (int index = 0; index < map_size; ++index)
    {
      // skip free cells and already labeled obstacle components
      if (occupancy_grid[index] != BLOCKED_CELL) continue;
      if (borderSourceId(index) != NO_SOURCE) continue;
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
          if (borderSourceId(candidate) != NO_SOURCE) continue;
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

  int borderSourceId(int index)
  {
    // compute the cell coordinates from the map index
    const int x = index % map_width;
    const int y = index / map_width;

    // assign one source id to each side of the border
    if (y < BORDER_WIDTH_CELLS) return TOP_BORDER_SOURCE;
    if (x >= map_width - BORDER_WIDTH_CELLS) return RIGHT_BORDER_SOURCE;
    if (y >= map_height - BORDER_WIDTH_CELLS) return BOTTOM_BORDER_SOURCE;
    if (x < BORDER_WIDTH_CELLS) return LEFT_BORDER_SOURCE;

    return NO_SOURCE;
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
        if (occupancy_grid[candidate] == BLOCKED_CELL) continue;

        // first wavefront to reach this free cell assigns its distance/source
        if (brushfire_distance_grid[candidate] == UNVISITED)
        {
          brushfire_distance_grid[candidate] = brushfire_distance_grid[current] + 1;
          brushfire_source_grid[candidate] = brushfire_source_grid[current];
          brushfire_queue.push(candidate);
          continue;
        }

        // different source labels meeting define a Voronoi candidate
        if (brushfire_source_grid[candidate] != brushfire_source_grid[current])
        {
          const bool has_clearance =
            brushfire_distance_grid[current] >= MIN_GVD_DISTANCE &&
            brushfire_distance_grid[candidate] >= MIN_GVD_DISTANCE;

          if (has_clearance && gvd_grid[candidate] != GVD_CELL)
          {
            gvd_grid[candidate] = GVD_CELL;
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

    // collect all cells marked as part of the GVD
    for (int index = 0; index < map_width * map_height; ++index)
    {
      if (gvd_grid[index] == GVD_CELL)
      {
        gvd_cells.push_back(index);
      }
    }

    // publish the GVD cells as a marker point cloud
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
      0.9,
      0.9
    );

    if (gvd_cells.empty())
    {
      RCLCPP_WARN(this->get_logger(), "No GVD cells to publish.");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Published %zu GVD cells.",
      gvd_cells.size()
    );
  }

  void publishGraphEdges()
  {
    std::vector<std::vector<Eigen::Vector2d>> edge_paths;

    // collect the world-frame path stored in each graph edge
    for (const GraphEdge &edge : graph_edges)
    {
      if (edge.path.size() < 2) continue;
      edge_paths.push_back(edge.path);
    }

    // publish all graph edges as disconnected line segments
    visualizer.publishPaths(
      "graph_edges",
      edge_paths,
      map_frame_id,
      0,
      0.0,
      1.0,
      1.0,
      1.0,
      0.04,
      0.08
    );

    if (edge_paths.empty())
    {
      RCLCPP_WARN(this->get_logger(), "No graph edges to publish.");
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Published %zu graph edges.",
      edge_paths.size()
    );
  }

  void detectGraphNodes()
  {
    const int map_size = map_width * map_height;
    std::vector<uint8_t> graph_node_candidate_grid;
    std::vector<Eigen::Vector2d> graph_node_points;
    std::size_t graph_node_cell_count = 0;

    graph_node_candidate_grid.assign(map_size, NON_GRAPH_NODE_CELL);
    graph_node_grid.assign(map_size, NO_GRAPH_NODE);
    graph_nodes.clear();

    const int scan_radius = GRAPH_NODE_SCAN_SIDE / 2;

    // scan a square around each GVD cell to find node candidates
    for (int y = scan_radius; y < map_height - scan_radius; ++y)
    {
      for (int x = scan_radius; x < map_width - scan_radius; ++x)
      {
        const int index = y * map_width + x;
        if (gvd_grid[index] != GVD_CELL) continue;

        std::vector<uint8_t> perimeter_cells;

        // read the square perimeter in order
        // bottom
        for (int scan_x = x - scan_radius; scan_x <= x + scan_radius; ++scan_x)
        {
          const int scan_index = (y - scan_radius) * map_width + scan_x;
          perimeter_cells.push_back(gvd_grid[scan_index] == GVD_CELL);
        }

        // right
        for (int scan_y = y - scan_radius + 1; scan_y <= y + scan_radius; ++scan_y)
        {
          const int scan_index = scan_y * map_width + x + scan_radius;
          perimeter_cells.push_back(gvd_grid[scan_index] == GVD_CELL);
        }

        // top
        for (int scan_x = x + scan_radius - 1; scan_x >= x - scan_radius; --scan_x)
        {
          const int scan_index = (y + scan_radius) * map_width + scan_x;
          perimeter_cells.push_back(gvd_grid[scan_index] == GVD_CELL);
        }

        // left
        for (int scan_y = y + scan_radius - 1; scan_y > y - scan_radius; --scan_y)
        {
          const int scan_index = scan_y * map_width + x - scan_radius;
          perimeter_cells.push_back(gvd_grid[scan_index] == GVD_CELL);
        }

        // count separated GVD groups crossing the perimeter
        int branch_count = 0;
        for (std::size_t i = 0; i < perimeter_cells.size(); ++i)
        {
          const std::size_t previous = (i + perimeter_cells.size() - 1) %
            perimeter_cells.size();
          if (perimeter_cells[i] && !perimeter_cells[previous])
          {
            branch_count++;
          }
        }

        // mark endpoints and junctions as graph node candidates
        if (branch_count != 2)
        {
          graph_node_candidate_grid[index] = GRAPH_NODE_CELL;
        }
      }
    }

    // create graph nodes out of cell clusters
    std::vector<uint8_t> visited(map_size, 0);
    for (int index = 0; index < map_size; ++index)
    {
      if (graph_node_candidate_grid[index] != GRAPH_NODE_CELL) continue;
      if (visited[index]) continue;

      std::queue<int> cluster_queue;
      std::vector<int> cluster_cells;
      double position_x_sum = 0.0;
      double position_y_sum = 0.0;

      // grow one connected node-candidate cluster
      cluster_queue.push(index);
      visited[index] = 1;

      while (!cluster_queue.empty())
      {
        const int current = cluster_queue.front();
        cluster_queue.pop();
        cluster_cells.push_back(current);

        const int x = current % map_width;
        const int y = current / map_width;
        position_x_sum += map_origin_x + (x + 0.5) * map_resolution;
        position_y_sum += map_origin_y + (y + 0.5) * map_resolution;

        // inspect 8-connected neighbours inside the candidate grid
        for (int dy = -1; dy <= 1; ++dy)
        {
          for (int dx = -1; dx <= 1; ++dx)
          {
            if (dx == 0 && dy == 0) continue;

            const int candidate_x = x + dx;
            const int candidate_y = y + dy;
            if (candidate_x < 0 || candidate_x >= map_width) continue;
            if (candidate_y < 0 || candidate_y >= map_height) continue;

            const int candidate = candidate_y * map_width + candidate_x;
            if (graph_node_candidate_grid[candidate] != GRAPH_NODE_CELL) continue;
            if (visited[candidate]) continue;

            visited[candidate] = 1;
            cluster_queue.push(candidate);
          }
        }
      }

      // discard very small node-candidate clusters
      if (static_cast<int>(cluster_cells.size()) < MIN_GRAPH_NODE_CLUSTER_CELLS)
      {
        continue;
      }

      // store one graph node at the mean position of the cluster
      const int node_id = graph_nodes.size();
      GraphNode node;
      node.position.x() = position_x_sum / cluster_cells.size();
      node.position.y() = position_y_sum / cluster_cells.size();
      graph_nodes.push_back(node);
      graph_node_points.push_back(node.position);
      graph_node_cell_count += cluster_cells.size();

      // mark every cell in this node cluster with its graph node id
      for (int cell : cluster_cells)
      {
        graph_node_grid[cell] = node_id;
      }
    }

    // publish graph node mean positions for RViz visualization
    visualizer.publishPointsArray(
      "graph_nodes",
      graph_node_points,
      map_frame_id,
      0,
      0.1,
      0.0,
      0.4,
      0.12
    );

    RCLCPP_INFO(
      this->get_logger(),
      "Detected %zu graph nodes from %zu candidate cells.",
      graph_nodes.size(),
      graph_node_cell_count
    );
  }

  void extractGraphEdges()
  {
    const int map_size = map_width * map_height;
    std::vector<int> edge_keys;

    // clear previous edge data before rebuilding the graph
    graph_edges.clear();
    for (GraphNode &node : graph_nodes)
    {
      node.edge_ids.clear();
    }

    // start each edge search from GVD cells adjacent to a graph node
    for (int node_id = 0; node_id < static_cast<int>(graph_nodes.size()); ++node_id)
    {
      for (int node_cell = 0; node_cell < map_size; ++node_cell)
      {
        if (graph_node_grid[node_cell] != node_id) continue;

        const int node_x = node_cell % map_width;
        const int node_y = node_cell / map_width;

        for (int dy = -1; dy <= 1; ++dy)
        {
          for (int dx = -1; dx <= 1; ++dx)
          {
            if (dx == 0 && dy == 0) continue;

            const int start_x = node_x + dx;
            const int start_y = node_y + dy;
            if (start_x < 0 || start_x >= map_width) continue;
            if (start_y < 0 || start_y >= map_height) continue;

            const int start_cell = start_y * map_width + start_x;
            if (gvd_grid[start_cell] != GVD_CELL) continue;
            if (graph_node_grid[start_cell] != NO_GRAPH_NODE) continue;

            std::queue<int> edge_queue;
            std::vector<uint8_t> visited(map_size, 0);
            std::vector<int> parent(map_size, -1);
            int target_node = NO_GRAPH_NODE;
            int target_cell = -1;

            // search through non-node GVD cells until another node is reached
            edge_queue.push(start_cell);
            visited[start_cell] = 1;

            while (!edge_queue.empty() && target_node == NO_GRAPH_NODE)
            {
              const int current = edge_queue.front();
              edge_queue.pop();

              const int x = current % map_width;
              const int y = current / map_width;

              for (int search_dy = -1; search_dy <= 1; ++search_dy)
              {
                for (int search_dx = -1; search_dx <= 1; ++search_dx)
                {
                  if (search_dx == 0 && search_dy == 0) continue;

                  const int candidate_x = x + search_dx;
                  const int candidate_y = y + search_dy;
                  if (candidate_x < 0 || candidate_x >= map_width) continue;
                  if (candidate_y < 0 || candidate_y >= map_height) continue;

                  const int candidate = candidate_y * map_width + candidate_x;
                  if (gvd_grid[candidate] != GVD_CELL) continue;

                  const int candidate_node = graph_node_grid[candidate];
                  if (candidate_node != NO_GRAPH_NODE)
                  {
                    if (candidate_node != node_id)
                    {
                      target_node = candidate_node;
                      target_cell = current;
                    }
                    continue;
                  }

                  if (visited[candidate]) continue;

                  visited[candidate] = 1;
                  parent[candidate] = current;
                  edge_queue.push(candidate);
                }
              }
            }

            if (target_node == NO_GRAPH_NODE) continue;

            const int first_node = std::min(node_id, target_node);
            const int second_node = std::max(node_id, target_node);
            const int edge_key =
              first_node * static_cast<int>(graph_nodes.size()) + second_node;
            if (std::find(edge_keys.begin(), edge_keys.end(), edge_key) !=
                edge_keys.end())
            {
              continue;
            }
            edge_keys.push_back(edge_key);

            GraphEdge edge;
            edge.from_node = node_id;
            edge.to_node = target_node;

            // reconstruct the ordered cell path found by the BFS
            for (int cell = target_cell; cell >= 0; cell = parent[cell])
            {
              edge.cells.push_back(cell);
            }
            std::reverse(edge.cells.begin(), edge.cells.end());

            edge.path.push_back(graph_nodes[edge.from_node].position);
            for (int cell : edge.cells)
            {
              const int x = cell % map_width;
              const int y = cell / map_width;
              edge.path.push_back(Eigen::Vector2d(
                map_origin_x + (x + 0.5) * map_resolution,
                map_origin_y + (y + 0.5) * map_resolution
              ));
            }
            edge.path.push_back(graph_nodes[edge.to_node].position);
            edge.path = smoothPath(edge.path);

            for (std::size_t i = 1; i < edge.path.size(); ++i)
            {
              edge.cost += (edge.path[i] - edge.path[i - 1]).norm();
            }

            const int edge_id = graph_edges.size();
            graph_edges.push_back(edge);
            graph_nodes[edge.from_node].edge_ids.push_back(edge_id);
            graph_nodes[edge.to_node].edge_ids.push_back(edge_id);
          }
        }
      }
    }

    RCLCPP_INFO(
      this->get_logger(),
      "Extracted %zu graph edges.",
      graph_edges.size()
    );
  }

  std::vector<Eigen::Vector2d> runDijkstra(int start_node, int goal_node)
  {
    if (start_node < 0 || start_node >= static_cast<int>(graph_nodes.size()))
    {
      RCLCPP_WARN(this->get_logger(), "Invalid Dijkstra start node.");
      return {};
    }
    if (goal_node < 0 || goal_node >= static_cast<int>(graph_nodes.size()))
    {
      RCLCPP_WARN(this->get_logger(), "Invalid Dijkstra goal node.");
      return {};
    }
    if (start_node == goal_node)
    {
      return {graph_nodes[start_node].position};
    }

    const double infinity = std::numeric_limits<double>::infinity();
    std::vector<double> best_cost(graph_nodes.size(), infinity);
    std::vector<int> previous_node(graph_nodes.size(), NO_GRAPH_NODE);
    std::vector<int> previous_edge(graph_nodes.size(), NO_EDGE);

    using QueueEntry = std::pair<double, int>;
    std::priority_queue<
      QueueEntry,
      std::vector<QueueEntry>,
      std::greater<QueueEntry>
    > open;

    best_cost[start_node] = 0.0;
    open.push({0.0, start_node});

    while (!open.empty())
    {
      const double current_cost = open.top().first;
      const int current_node = open.top().second;
      open.pop();

      if (current_cost > best_cost[current_node]) continue;
      if (current_node == goal_node) break;

      // relax every graph edge connected to the current node
      for (int edge_id : graph_nodes[current_node].edge_ids)
      {
        const GraphEdge &edge = graph_edges[edge_id];
        const int next_node =
          (edge.from_node == current_node) ? edge.to_node : edge.from_node;
        const double new_cost = current_cost + edge.cost;

        if (new_cost >= best_cost[next_node]) continue;

        best_cost[next_node] = new_cost;
        previous_node[next_node] = current_node;
        previous_edge[next_node] = edge_id;
        open.push({new_cost, next_node});
      }
    }

    if (best_cost[goal_node] == infinity)
    {
      RCLCPP_WARN(this->get_logger(), "Dijkstra could not find a graph path.");
      return {};
    }

    std::vector<int> edge_path;
    for (int node = goal_node; node != start_node; node = previous_node[node])
    {
      if (node == NO_GRAPH_NODE || previous_edge[node] == NO_EDGE) return {};
      edge_path.push_back(previous_edge[node]);
    }
    std::reverse(edge_path.begin(), edge_path.end());

    std::vector<Eigen::Vector2d> path;
    int current_node = start_node;

    // concatenate graph edge paths in the direction selected by Dijkstra
    for (int edge_id : edge_path)
    {
      const GraphEdge &edge = graph_edges[edge_id];
      const bool forward = edge.from_node == current_node;
      const std::vector<Eigen::Vector2d> &edge_points = edge.path;

      for (std::size_t i = 0; i < edge_points.size(); ++i)
      {
        const std::size_t point_index = forward ? i : edge_points.size() - 1 - i;
        if (!path.empty() && i == 0) continue;
        path.push_back(edge_points[point_index]);
      }

      current_node = forward ? edge.to_node : edge.from_node;
    }

    return path;
  }

  // ------------------ Utility Functions ---------------------

  int nearestGraphNode(const Eigen::Vector2d &point)
  {
    int nearest_node = NO_GRAPH_NODE;
    double nearest_distance = std::numeric_limits<double>::infinity();

    // find the graph node closest to the given world point
    for (std::size_t i = 0; i < graph_nodes.size(); ++i)
    {
      const double distance = (graph_nodes[i].position - point).squaredNorm();
      if (distance < nearest_distance)
      {
        nearest_node = static_cast<int>(i);
        nearest_distance = distance;
      }
    }

    return nearest_node;
  }

  std::vector<Eigen::Vector2d> smoothPath(const std::vector<Eigen::Vector2d> &path)
  {
    if (path.size() <= 2) return path;

    std::vector<Eigen::Vector2d> smoothed_path = path;

    // keep graph node endpoints fixed and smooth only intermediate points
    for (std::size_t i = 1; i + 1 < smoothed_path.size(); ++i)
    {
      smoothed_path[i] =
        SMOOTHING_NEIGHBOUR_WEIGHT * smoothed_path[i - 1] +
        SMOOTHING_CENTER_WEIGHT * smoothed_path[i] +
        SMOOTHING_NEIGHBOUR_WEIGHT * smoothed_path[i + 1];
    }

    return smoothed_path;
  }

  void sendVelocity(Eigen::Vector2d vel)
  {
    // split the desired planar velocity into x and y components
    const double v_x = vel.x();
    const double v_y = vel.y();

    // apply feedback linearization for the differential-drive robot
    const double v = v_x * std::cos(robot_yaw) + v_y * std::sin(robot_yaw);
    const double w =
      (-v_x * std::sin(robot_yaw) + v_y * std::cos(robot_yaw)) / D;

    // publish the resulting linear and angular velocity
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

  // robot
  Eigen::Vector2d robot_pos;
  Eigen::Vector2d goal;
  double          robot_yaw = 0.0;
  bool            has_odom = false;
  bool            received_goal = false;

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

  // graph
  std::vector<GraphNode> graph_nodes;
  std::vector<GraphEdge> graph_edges;
  std::vector<int>       graph_node_grid;

  // path
  std::vector<Eigen::Vector2d> followed_path;
  int                          waypoint_i = 0;
  bool                         has_path = false;

  // consts
  const unsigned LOOP_DT_MS = 100;
  const double   D = 0.1;
  const int      BORDER_SOURCE_COUNT = 4;
  const int      BORDER_WIDTH_CELLS = 5;
  const int      MIN_GVD_DISTANCE = 10;
  const int      GRAPH_NODE_SCAN_SIDE = 7;
  const int      MIN_GRAPH_NODE_CLUSTER_CELLS = 3;
  const double   SMOOTHING_CENTER_WEIGHT = 0.3;
  const double   SMOOTHING_NEIGHBOUR_WEIGHT = 0.35;

  // labels
  const uint8_t  FREE_CELL = 0;
  const uint8_t  BLOCKED_CELL = 1;
  const uint8_t  NON_GVD_CELL = 0;
  const uint8_t  GVD_CELL = 1;
  const uint8_t  NON_GRAPH_NODE_CELL = 0;
  const uint8_t  GRAPH_NODE_CELL = 1;
  const int      UNVISITED = -1;
  const int      NO_SOURCE = -1;
  const int      NO_GRAPH_NODE = -1;
  const int      NO_EDGE = -1;
  const int      TOP_BORDER_SOURCE = 0;
  const int      RIGHT_BORDER_SOURCE = 1;
  const int      BOTTOM_BORDER_SOURCE = 2;
  const int      LEFT_BORDER_SOURCE = 3;
};

int main(int argc, char ** argv)
{
  // initialize and spin the GVD node
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GVD>());
  rclcpp::shutdown();
  return 0;
}
