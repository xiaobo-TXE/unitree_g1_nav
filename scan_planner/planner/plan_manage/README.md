# Terminal tracking handoff

For real-robot closed-loop reference-path navigation (`navi_mode:=3`),
`run.launch.py` starts both trackers and `velocity_selector` by default:

```bash
source install/setup.bash
ros2 launch scan_planner run.launch.py is_real_world:=true navi_mode:=3 \
  controller_mode:=closed_loop enable_terminal_handoff:=true
```

The local controller publishes `/nav_cmd/local`, and `path_follow` publishes
`/nav_cmd/global`. Only `velocity_selector` publishes `/nav_cmd`. Do not start
an additional standalone tracker that also publishes `/nav_cmd`.

The selector measures the 2D straight-line distance from `/loc/robo/odom` to the
last `/global_path` pose, both in `map`. At a distance of **2 m or less** it
enables `path_follow` and keeps that choice until the terminal position/yaw or
frame changes. Repeated paths and changes to intermediate points do not reset
the choice. Empty or invalid paths stop output. A new goal starts selection again.

**The terminal segment follows the global path directly, without local obstacle
avoidance or collision stopping.** Keep that segment clear. Local obstacle-related
stop commands are ignored after handoff, as are all other local commands.

During a handoff the selector clears buffered velocities, sends a stop and waits
for `/path_follower/set_enabled` acknowledgement of the current goal before
accepting a fresh selected velocity. Missing/invalid localization, a frame mismatch
or a selected velocity older than 0.3 s stops output without switching sources.
These input ages use monotonic receive time, not message timestamps.

Configure `switch_distance`, `cmd_timeout`, and `odom_timeout` under
`velocity_selector` in `config/controllers.yaml`. The terminal follower retains
`goal_align_distance=0.12 m` and `goal_tolerance=0.2 m`. The local controller
retains `max_vy=0.3 m/s`. Look for `[SELECTOR]` and `[PF_DIAG]` logs.

To use the original local-only connection:

```bash
ros2 launch scan_planner run.launch.py enable_terminal_handoff:=false
```

Simulation, open-loop control and other navigation modes retain their original
launch behavior. Standalone `path_follow` remains enabled by default; its
`~/set_enabled` service accepts `std_srvs/SetBool` and returns the acknowledged
goal identity as a JSON array `[frame, x, y, yaw]` (or `null` when no goal exists).

Tests include state-machine checks, launch wiring and live ROS selector/follower
interaction. Build and run them with:

```bash
colcon build --packages-up-to scan_planner --cmake-args -DBUILD_TESTING=ON
source install/setup.bash
colcon test --packages-select path_follow scan_planner
colcon test-result --verbose
```
