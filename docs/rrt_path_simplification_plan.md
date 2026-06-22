# RRT Path Simplification Implementation Plan

## Goal

Add a post-processing step that simplifies the path returned by `runRRT()` while preserving the rough shape of the original RRT path.

The simplification should:

- Remove unnecessary intermediate waypoints.
- Keep all resulting path segments collision-free.
- Avoid overly aggressive global shortcutting.
- Use a local lookahead limit so the path still resembles the original route.

---

## 1. Design choice

Keep `runRRT()` focused only on planning and path construction. Add a separate function:

```cpp
std::vector<Eigen::Vector2d> simplifyPath(
  const std::vector<Eigen::Vector2d> &raw_path) const;
```

Then apply it after `runRRT()` succeeds:

```cpp
const auto raw_path = runRRT(robot_pos, goal);
path = simplifyPath(raw_path);
has_path = !path.empty();
```

This keeps the code structure clean:

- `runRRT()` generates a valid path.
- `simplifyPath()` improves that path.
- `controlLoop()` decides when to run planning and post-processing.

---

## 2. Local lookahead shortcutting

Use bounded line-of-sight pruning instead of full line-of-sight pruning.

Full pruning would try to connect each waypoint to the farthest possible future waypoint. That can erase too much of the path shape, especially in open areas.

Instead, limit the search to a small number of future nodes:

```cpp
const std::size_t MAX_SIMPLIFY_LOOKAHEAD = 5;
```

For each current waypoint `i`, try to jump ahead at most 5 path nodes, choosing the farthest locally reachable waypoint whose segment is valid.

Example:

```text
raw:        A -> B -> C -> D -> E -> F -> G -> H
lookahead: 5
possible:  A --------------------> F -> G -> H
```

If `A -> F` is collision-free, keep `F` and remove `B, C, D, E`. If not, try `A -> E`, then `A -> D`, etc.

---

## 3. Collision safety

Reuse the existing segment checker:

```cpp
bool isSegmentValid(
  const Eigen::Vector2d &start,
  const Eigen::Vector2d &end) const;
```

Because the occupancy grid is already inflated by `ROBOT_RADIUS`, a simplified segment is safe if `isSegmentValid()` returns true.

No new collision-checking logic is needed.

---

## 4. Proposed algorithm

```cpp
std::vector<Eigen::Vector2d> simplifyPath(
  const std::vector<Eigen::Vector2d> &raw_path) const
{
  if (raw_path.size() <= 2) return raw_path;

  std::vector<Eigen::Vector2d> simplified_path;
  simplified_path.push_back(raw_path.front());

  std::size_t current_index = 0;

  while (current_index < raw_path.size() - 1)
  {
    std::size_t best_next_index = current_index + 1;

    const std::size_t max_next_index = std::min(
      raw_path.size() - 1,
      current_index + MAX_SIMPLIFY_LOOKAHEAD);

    for (std::size_t candidate_index = max_next_index;
         candidate_index > current_index + 1;
         --candidate_index)
    {
      if (isSegmentValid(raw_path[current_index], raw_path[candidate_index]))
      {
        best_next_index = candidate_index;
        break;
      }
    }

    simplified_path.push_back(raw_path[best_next_index]);
    current_index = best_next_index;
  }

  return simplified_path;
}
```

Notes:

- The function always keeps the first and last path points.
- If no shortcut is valid, it falls back to the next original waypoint.
- The path remains valid because every accepted shortcut is checked with `isSegmentValid()`.

---

## 5. Integration point

Update the path planning block in `controlLoop()`.

Current shape:

```cpp
if (!has_path)
{
  path = runRRT(robot_pos, goal);
  has_path = !path.empty();
  waypoint_i = 0;
  if (has_path) visualizer.publishLineStrip("rrt_path", path, map_frame_id);
}
```

Proposed shape:

```cpp
if (!has_path)
{
  const auto raw_path = runRRT(robot_pos, goal);
  path = simplifyPath(raw_path);
  has_path = !path.empty();
  waypoint_i = 0;
  if (has_path) visualizer.publishLineStrip("rrt_path", path, map_frame_id);
}
```

Optionally, publish both paths for debugging:

```cpp
if (!raw_path.empty()) visualizer.publishLineStrip("rrt_raw_path", raw_path, map_frame_id);
if (has_path) visualizer.publishLineStrip("rrt_path", path, map_frame_id);
```

---

## 6. Constants

Add a new class constant near the other RRT parameters:

```cpp
const std::size_t MAX_SIMPLIFY_LOOKAHEAD = 5;
```

This is a node-count limit, not a grid-cell limit.

With the current `STEP_SIZE = 0.1`, a lookahead of 5 roughly allows local shortcuts across about `0.5 m` of original path length, depending on the path geometry.

---

## 7. Expected result

The simplified path should:

- Have fewer waypoints than the raw RRT path.
- Remove small zig-zags caused by random tree growth.
- Preserve the general route selected by RRT.
- Avoid cutting directly across large open spaces unless that shortcut occurs within the local lookahead window.

This is a conservative first smoothing step. If more smoothness is needed later, a second post-processing stage could add curve fitting or corner rounding, but those generated curves would need their own collision validation.

---

## 8. Validation checklist

After implementing, test with several goals and verify:

1. The node still publishes a valid path.
2. The simplified path never crosses inflated obstacles.
3. The number of path points decreases compared to the raw path.
4. The simplified path keeps the same rough shape as the raw RRT path.
5. Changing `MAX_SIMPLIFY_LOOKAHEAD` has the expected effect:
   - Lower values preserve more detail.
   - Higher values produce more aggressive shortcutting.
