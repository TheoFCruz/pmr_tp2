# Bidirectional 2D Geometric RRT Implementation Plan

## Goal

Implement a bidirectional 2D geometric RRT planner that:

- Uses the robot's current position as the start configuration.
- Uses the received `/goal` point as the goal configuration.
- Grows two trees: one rooted at the start and one rooted at the goal.
- Connects both trees when possible.
- Reconstructs a path from start to goal.
- Publishes the resulting path for visualization.

Robot control/path following will be implemented later. This plan is only for making RRT build and publish a path.

---

## 1. RRT constants

Add RRT-specific constants to `rrt.cpp`:

```cpp
const int MAX_RRT_ITERATIONS = 1000;
const double STEP_SIZE = 0.3;
const double CONNECT_DISTANCE = 0.3;
const double GOAL_SAMPLE_PROBABILITY = 0.1;
const double COLLISION_CHECK_STEP = 0.05;
```

`GOAL_SAMPLE_PROBABILITY` controls target-biased sampling:

- The start tree sometimes samples the goal.
- The goal tree sometimes samples the robot's initial position.

---

## 2. Tree representation

Use a simple vector-backed tree:

```cpp
struct RRTNode
{
  Eigen::Vector2d position;
  int parent = -1;
};

using RRTTree = std::vector<RRTNode>;
```

Each node stores its position and the index of its parent node.

---

## 3. Sampling

Create a sampling helper:

```cpp
Eigen::Vector2d samplePoint(const Eigen::Vector2d &tree_target);
```

Behavior:

1. With probability `GOAL_SAMPLE_PROBABILITY`, return `tree_target`.
2. Otherwise, return a uniformly random point inside the map bounds.

Usage:

- When extending the start tree, pass the goal as `tree_target`.
- When extending the goal tree, pass the start position as `tree_target`.

---

## 4. Tree extension

Implement:

```cpp
int extendTree(
  RRTTree &tree,
  const Eigen::Vector2d &sample);
```

Steps:

1. Find the nearest tree node to the sampled point.
2. Steer from `q_near` toward the sample, capped by `STEP_SIZE`.
3. Run the straight-line local planner between `q_near` and `q_new`.
4. If the segment is collision-free, add `q_new` to the tree.
5. Return the new node index, or `-1` if extension failed.

---

## 5. Local planner / collision checking

Replace the current `isPointValid()` placeholder with a segment checker:

```cpp
bool isSegmentFree(
  const Eigen::Vector2d &from,
  const Eigen::Vector2d &to);
```

This is the local planner for the 2D geometric RRT.

Behavior:

1. Sample points along the straight segment every `COLLISION_CHECK_STEP` meters.
2. Convert each sampled point to a map cell.
3. Reject the segment if any sampled point is outside the map.
4. Reject the segment if any sampled point is inside an occupied cell.
5. Accept the segment otherwise.

Then `extendTree()` should use:

```cpp
if (!isSegmentFree(q_near, q_new)) return -1;
```

---

## 6. Tree connection

After adding a node to one tree, try to connect to the other tree:

```cpp
bool tryConnectTrees(
  const RRTTree &tree_a,
  const RRTTree &tree_b,
  int new_node_index,
  int &connection_index_b);
```

Steps:

1. Find the nearest node in `tree_b` to the newly added node in `tree_a`.
2. Check if the distance is less than or equal to `CONNECT_DISTANCE`.
3. Check whether the straight segment between both nodes is collision-free.
4. If valid, report success and return the connection index from `tree_b`.

---

## 7. Path reconstruction

When the trees connect, reconstruct the full path:

```cpp
std::vector<Eigen::Vector2d> buildPathFromTrees(
  const RRTTree &start_tree,
  int start_connection_index,
  const RRTTree &goal_tree,
  int goal_connection_index);
```

Path construction:

1. Trace the start-tree connection node back to the start root.
2. Reverse that partial path so it goes from start to connection.
3. Trace the goal-tree connection node back to the goal root.
4. Append the goal-tree path so the final path goes from connection to goal.

Final path direction:

```text
robot start -> ... -> connection -> ... -> goal
```

---

## 8. Publishing

After `runRRT()` returns a non-empty path, publish it:

```cpp
if (has_path)
{
  visualizer.publishPath(path, map_frame_id);
}
```

This part is already mostly present in `controlLoop()`.

---

## 9. Optional future improvement: nearest neighbor search

The first implementation can use brute-force nearest-neighbor search:

```cpp
int nearestNodeIndex(const RRTTree &tree, const Eigen::Vector2d &point);
```

Later, this can be replaced by a faster method without changing the planner structure.

Possible options:

- Spatial hash / grid buckets.
- Coordinate bins.
- Approximate nearest-neighbor search.
- KD-tree, if desired later.

For now, brute force is simple, reliable, and adequate for a basic RRT.
