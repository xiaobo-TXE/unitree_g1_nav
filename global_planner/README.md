# global_planner

A lightweight ROS 2 global planner for a legged robot on indoor, outdoor and
natural terrain. It directly consumes a static `PointXYZI` PCD and normalizes
different intensity scales to an internal traversability range of `[0, 1]`.
It does not depend on PCT Planner.

## Design

The PCD is voxelized once at startup. Only traversable surface voxels are kept,
and multiple heights in the same XY column remain separate. Low-traversability
voxels inflate nearby surfaces by the configured robot radius, but only within
the configured vertical band so separate floors do not erode each other.
Weighted A* creates neighbors lazily and rejects transitions that exceed the
configured horizontal distance, step height or slope. This avoids both a dense
3D array and a stored edge graph.

At startup, the planner labels connected terrain components using the same
motion constraints. Requests whose endpoints belong to different components
fail immediately. During search, generation-stamped open/closed arrays avoid
full-map resets and guarantee that each node is formally expanded at most once.

After search, a terrain-validated line-of-sight pass removes grid zigzags. Every
shortcut is sampled against the inflated traversable surface and checked with
the same step and slope limits. Stairs are not flattened when their height
profile differs from the shortcut by more than `smoothing_height_tolerance`.

## Topics

| Direction | Topic | Type |
|---|---|---|
| Input | `/initialpose` | `geometry_msgs/msg/PoseWithCovarianceStamped` (test start) |
| Input | `/lio/odom` | `nav_msgs/msg/Odometry` (live robot start) |
| Input | `/goal_3d` | `geometry_msgs/msg/PoseStamped` |
| Output | `/global_path` | `nav_msgs/msg/Path` |
| Output | `/global_planner/map` | `sensor_msgs/msg/PointCloud2` |
| Output | `/global_planner/raw_map` | `sensor_msgs/msg/PointCloud2` |

The planner continuously updates its live start from odometry. `/initialpose`
remains available for tests when odometry is absent; if both are publishing,
the most recently received valid pose is used when the goal arrives. Poses in
another TF frame are transformed to `map`, so the odometry frame must have a
valid TF connection to the configured map frame. The published path keeps a
snapped safe approach point and then appends the exact commanded goal position
and orientation. With `path_z_offset: 0.0`, its final pose equals `/goal_3d`;
with a non-zero offset, only the final Z is deliberately changed by that amount.
The topic is transient-local for RViz.

At startup, the planner also publishes the voxelized, inflated traversable map
that is actually used by A* on `/global_planner/map`. Its `intensity` field is
the normalized traversability in `[0, 1]`. The map uses transient-local QoS, so
RViz can subscribe after startup. Set `publish_map: false` to disable it or
change `map_topic` to remap the output.

The unmodified points loaded from the PCD are published separately on
`/global_planner/raw_map`, retaining their original `x`, `y`, `z` and
`intensity` values. Configure this output with `publish_raw_map` and
`raw_map_topic`. Both map topics publish once at startup and use transient-local
QoS rather than periodically retransmitting an unchanged static map.

## Build and run

```bash
cd /home/jiewang/nav_ros2_ws
source /opt/ros/foxy/setup.bash
colcon build --packages-select global_planner
source install/setup.bash
ros2 launch global_planner global_planner.launch.py
```

Override the map when needed:

```bash
ros2 launch global_planner global_planner.launch.py pcd_file:=/absolute/map.pcd
```

## Important tuning

- `resolution_xy`: lower values preserve narrow passages but use more memory.
- `intensity_normalization_mode`: `auto_minmax` detects the finite input range,
  `manual_minmax` uses configured bounds, and `legacy_255` preserves the old
  fixed 0--255 behavior.
- `intensity_semantics`: use `traversability` when high values are better, or
  `cost` when low values are better. Auto range detection cannot infer this
  direction reliably.
- `intensity_input_min` / `intensity_input_max`: bounds used by
  `manual_minmax`; values outside them are clamped.
- `normalized_traversability_threshold`: minimum accepted internal
  traversability in `[0, 1]` for auto and manual modes.
- `min_traversability`: old raw threshold used only by `legacy_255`.
- `robot_radius`: horizontal inflation radius around low-traversability voxels.
- `inflation_vertical_tolerance`: limits inflation to a nearby height layer.
- `max_step_height`: largest adjacent height change the robot can traverse.
- `max_slope_degrees`: hard slope limit, including stair connections.
- `snap_radius` and `snap_vertical_tolerance`: start/goal projection limits.
- `path_z_offset`: height added to every published path pose.
- `heuristic_weight`: `1.0` is ordinary A*; a slightly larger value is faster
  but may return a mildly longer route.
- `enable_connectivity_precheck`: rejects disconnected endpoints before A*.
- `enable_path_smoothing`: enables terrain-validated line-of-sight smoothing.

The defaults are an initial baseline, not certified hardware limits. Validate
step, slope and clearance settings on the physical robot before autonomous use.
