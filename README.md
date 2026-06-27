# PMR TP2

ROS 2 package with map-based path planning experiments for the iRobot Create 3 in Gazebo.

## Project Structure

- `src/astar.cpp`: A* planner and waypoint follower. It receives `/odom`, `/map`, and `/goal`, searches the occupancy grid with a Manhattan heuristic, publishes visited cells and the planned path, and commands `/cmd_vel`.
- `src/gvd.cpp`: Generalized Voronoi Diagram planner. It builds a brushfire GVD from `/map`, extracts a graph, runs Dijkstra between the graph nodes closest to the robot and goal, publishes graph/GVD markers, and follows the resulting path.
- `src/rrt.cpp`: bidirectional geometric RRT planner. It inflates the occupancy grid, grows start and goal trees, collision-checks segments, simplifies the resulting path, publishes the trees and paths, and commands `/cmd_vel`.
- `include/pmr_tp2/visualizer.hpp`: shared RViz marker helper used to publish points, paths, line lists, and occupancy-grid cell markers.
- `launch/`: top-level launch files for A*, GVD, and RRT, plus reusable includes for Gazebo, robot spawning, controllers, RViz, and the map server.
- `description/`: Create 3 xacro model files, including the robot base model and Gazebo odometry plugin setup.
- `config/`: ros2_control differential-drive controller parameters and ros_gz_bridge configuration.
- `worlds/`: Gazebo world files used by the simulations.
- `maps/`: occupancy-grid maps and YAML metadata loaded by `nav2_map_server`.
- `rviz/`: RViz configurations for visualizing each planner.
- `docs/`: implementation notes and plans for the RRT planner and path simplification.

## Setup

Source ROS 2 and install package dependencies from the workspace root:

```bash
cd ~/Development/ros2/ros2_ws
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
```

Build the package:

```bash
colcon build --packages-select pmr_tp2 --symlink-install
source install/setup.bash
```

## A*

Launch the simulation, RViz, map server, and A* node:

```bash
ros2 launch pmr_tp2 astar.launch.py
```

Send a goal:

```bash
ros2 topic pub --once /goal geometry_msgs/msg/Point "{x: 2.0, y: 0.0, z: 0.0}"
```

The node runs A* on the occupancy grid using the robot pose as the start and
the received `/goal` as the target. It publishes the visited cells and planned
path, simplifies straight path segments, and follows the waypoints with
feedback linearization.

Optional launch arguments:

```bash
ros2 launch pmr_tp2 astar.launch.py map_name:=maze_map.yaml
```

## GVD

Launch the simulation, RViz, map server, and GVD node:

```bash
ros2 launch pmr_tp2 gvd.launch.py
```

Send a goal:

```bash
ros2 topic pub --once /goal geometry_msgs/msg/Point "{x: 2.0, y: 0.0, z: 0.0}"
```

The node converts the map into a Voronoi skeleton with brushfire expansion,
extracts graph nodes and edges, and uses Dijkstra to plan along the skeleton.

## RRT

Launch the simulation, RViz, map server, and RRT node:

```bash
ros2 launch pmr_tp2 rrt.launch.py
```

Send a goal:

```bash
ros2 topic pub --once /goal geometry_msgs/msg/Point "{x: 2.0, y: 0.0, z: 0.0}"
```

The node runs a bidirectional RRT on an inflated occupancy grid, connects the two
trees when a collision-free segment is found, simplifies the path locally, and
then follows the simplified waypoints.
