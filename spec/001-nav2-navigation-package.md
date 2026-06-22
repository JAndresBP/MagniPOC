# Spec 001 — Nav2 Navigation Package (`magni_navigation`)

Date: 2026-06-22

Status: Draft

Owner: ROS2Architect → ROS2Developer

---

## 1. Goal 🎯

Add autonomous navigation to MAGNI by integrating the **ROS 2 Nav2** stack in a new
package `src/magni_navigation`, mirroring the structure and conventions of the existing
`magni_mapping` package. The package must let an operator send a goal pose (via RViz or
the `/navigate_to_pose` action) and have MAGNI plan and drive to it while avoiding
obstacles, in **simulation first** (Webots), then on the real robot.

This spec is simulation-first and reuses the same sensors/frames already wired for
Cartographer (`/scan`, `/odom`, `base_link`/`odom`/`map`).

---

## 2. Context & motivation 🧭

- Mapping (Cartographer 2D) is already integrated via `magni_mapping`; the natural next
  capability is goal-directed navigation on top of that map.
- The devcontainer already installs **`ros-jazzy-nav2-bringup`** (see
  `.devcontainer/Dockerfile`), so Nav2 binaries are available without new system deps in
  the dev image — only `package.xml` declarations and (for the robot image) Dockerfile
  additions are required.
- Nav2 consumes exactly the topics/frames the project already produces, so integration is
  primarily configuration + launch wiring, not new C++.

---

## 3. Requirements (Actor / Event / Environment / Artifact / Response / Restriction)

Using the project's ROS2Architect requirement framework:

- **Actor:** Operator (RViz "Nav2 Goal" tool) or an external node calling the
  `nav2_msgs/action/NavigateToPose` action.
- **Event:** A goal pose is published / the action is invoked.
- **Environment:** MAGNI running in Webots (`break_room.wbt`) with `use_sim_time:=true`;
  later the real robot. A map is available either from a saved pbstream/occupancy grid
  (localization mode) or being built live (SLAM mode).
- **Artifact:** Goal `geometry_msgs/PoseStamped`; output velocity commands on `/cmd_vel`;
  planned path on `/plan`; costmaps on `/global_costmap/*`, `/local_costmap/*`.
- **Response:** The robot plans a collision-free path and drives to the goal within
  tolerance, recovering from local blockages via Nav2 behavior trees.
- **Restriction:** 2D planar navigation only (diff-drive kinematics); must publish a
  **stamped** velocity command to match `ubiquity_velocity_controller`; must not require
  IMU; `published_frame`/TF chain must remain consistent with Cartographer's conventions.

---

## 4. Topics, frames & TF 🔁

Inputs/outputs (all standard Nav2, aligned with existing stack):

| Interface | Type | Topic / Action | Source |
| --- | --- | --- | --- |
| Laser scan | `sensor_msgs/LaserScan` | `/scan` | Webots / YDLidar |
| Odometry | `nav_msgs/Odometry` | `/odom` | `ubiquity_velocity_controller` |
| Velocity cmd | `geometry_msgs/TwistStamped` | `/cmd_vel` | Nav2 controller server |
| Map | `nav_msgs/OccupancyGrid` | `/map` | map_server (loc) or Cartographer (SLAM) |
| Goal | action | `/navigate_to_pose` | Operator / RViz |
| Path | `nav_msgs/Path` | `/plan` | Nav2 planner |

- **TF chain:** `map → odom → base_link → <sensor links>` (laser). In localization mode
  AMCL provides `map → odom`; in SLAM mode Cartographer provides it. `odom → base_link`
  comes from `ubiquity_velocity_controller`; static links from `robot_state_publisher`.
- **Stamped cmd_vel:** Nav2 on Jazzy must publish `TwistStamped` (set
  `enable_stamped_cmd_vel: true`) because the controller and teleop in this repo already
  use stamped twists (`run-teleop-keyboard.sh` uses `-p stamped:=true`; the joy launch
  uses `publish_stamped_twist:=true`).
- Propagate `use_sim_time:=true` to **all** Nav2 nodes in simulation.

---

## 5. Package design 📦

New package `src/magni_navigation` (`ament_cmake`, mirroring `magni_mapping`):

```
src/magni_navigation/
  CMakeLists.txt           # installs config + launch (same as magni_mapping)
  package.xml              # deps below
  README.md                # run instructions + caveats
  config/
    nav2_params.yaml       # full Nav2 param set (AMCL, planner, controller, costmaps, BT, behaviors)
    nav2_default_view.rviz # optional RViz layout with Nav2 panels (or reuse control_station rviz)
  launch/
    navigation.launch.py   # brings up Nav2 + optional localization (AMCL) + map_server
  maps/
    break_room.yaml        # occupancy-grid map metadata for localization mode
    break_room.pgm         # saved map image (produced by magni_mapping run)
  test/
    test_nav2_config.py    # config-validation test (mirror test_cartographer_config.py)
```

### `package.xml` dependencies

Build/exec depends (all available as `ros-jazzy-*`):
`nav2_bringup`, `nav2_common`, `nav2_lifecycle_manager`, `nav2_map_server`,
`nav2_amcl`, `nav2_planner`, `nav2_controller`, `nav2_behaviors`,
`nav2_bt_navigator`, `nav2_costmap_2d`, plus `rclcpp`, `nav_msgs`, `sensor_msgs`,
`geometry_msgs`, `tf2_ros`. Test deps: `ament_lint_auto`, `ament_lint_common`.

### `CMakeLists.txt`

Identical pattern to `magni_mapping`: `find_package(ament_cmake REQUIRED)` +
`install(DIRECTORY config launch maps DESTINATION share/${PROJECT_NAME})` +
`ament_package()`.

### `launch/navigation.launch.py`

Python launch following repo conventions (`DeclareLaunchArgument` +
`IfCondition(LaunchConfiguration(...))`):

- Args:
  - `use_sim_time` (default `true`)
  - `slam` (default `false`) — when `true`, skip AMCL + map_server and rely on
    `magni_mapping` providing `map → odom`; when `false`, start `map_server` + `amcl`.
  - `map` (default `<pkg>/maps/break_room.yaml`) — map file for localization mode.
  - `params_file` (default `<pkg>/config/nav2_params.yaml`).
  - `autostart` (default `true`) — lifecycle auto-transition.
- Prefer **including** the upstream `nav2_bringup/launch/bringup_launch.py` (or
  `navigation_launch.py` + `localization_launch.py`) with our `params_file`/`map`,
  rather than re-declaring every node — keeps us aligned with upstream lifecycle wiring.

### `config/nav2_params.yaml` — key settings for MAGNI

- `amcl`: diff-drive motion model, `base_frame_id: base_link`, `odom_frame_id: odom`,
  `global_frame_id: map`, scan topic `/scan`.
- `controller_server`: DWB or MPPI/RPP controller tuned for diff drive; publish
  `TwistStamped` (`enable_stamped_cmd_vel: true`).
- `planner_server`: `NavfnPlanner` (or Smac 2D) with `GridBased` plugin.
- `local_costmap` / `global_costmap`: `robot_radius` from MAGNI footprint
  (`magni_description`), `obstacle_layer` using `/scan`, `inflation_layer`,
  `static_layer` (global only).
- `bt_navigator`: default `navigate_to_pose` + `navigate_through_poses` BTs.
- `behavior_server`: spin / backup / wait recoveries.

---

## 6. Integration with existing bringup 🔌

1. **Simulation** — add a `navigation` arg to
   `src/magni_webots/launch/magni_sim_bringup.launch.py` (alongside existing `rviz` and
   `mapping` args), conditionally including `magni_navigation/launch/navigation.launch.py`
   with `use_sim_time:=true`. Extend `scripts/launch-webots-simulation.sh` with a
   `--navigation` flag (parsed like `--rviz` / `--mapping`, appending `navigation:=true`).
2. **Real robot** — add an `enable_navigation` arg to
   `src/magni_bringup/launch/magni_bringup.launch.py` to include navigation alongside
   motors + lidar.
3. **Build wiring** — add `src/magni_navigation` to the `--base-paths` list in
   `scripts/build-simulation.sh`.
4. **Robot image** — add the `ros-jazzy-nav2-*` runtime packages to the runtime stage of
   the top-level `Dockerfile` (the devcontainer already has `nav2-bringup`; the arm64
   robot image does not).

Modes:
- **SLAM + navigation** (explore unknown space): `mapping:=true navigation:=true slam:=true`.
- **Localization + navigation** (known map): `navigation:=true slam:=false` with a saved
  `maps/break_room.yaml`.

---

## 7. Implementation tasks (todo) ✅

1. Scaffold `src/magni_navigation` (`package.xml`, `CMakeLists.txt`, dirs).
2. Author `config/nav2_params.yaml` tuned for diff-drive + `/scan` + stamped cmd_vel.
3. Write `launch/navigation.launch.py` with `slam` / `map` / `params_file` / `use_sim_time`.
4. Save an initial map from a `magni_mapping` run into `maps/break_room.{yaml,pgm}`.
5. Wire `navigation` arg into `magni_sim_bringup.launch.py` + `--navigation` in the script.
6. Wire `enable_navigation` into `magni_bringup.launch.py` (real robot).
7. Add `magni_navigation` to `build-simulation.sh` and Nav2 deps to the robot `Dockerfile`.
8. Add `test/test_nav2_config.py` (param/launch validation, mirror mapping test).
9. Extend `magni_integration_tests` with a navigation smoke test (send a goal, assert the
   `navigate_to_pose` action returns success and `base_link` reaches goal tolerance).
10. Add a `nav2-smoke-test.yml` workflow (mirror `mapping-smoke-test.yml`, self-hosted GPU).
11. Write `README.md` and record **ADR 004 — Nav2 for autonomous navigation** in
    `architecture/adr/`, plus a `architecture/nav2_integration_design.md` design doc.

---

## 8. Testing & acceptance criteria 🧪

Simulation smoke test (manual + CI):

1. `./scripts/launch-webots-simulation.sh --rviz --navigation` (localization mode with the
   saved map) — Nav2 lifecycle nodes reach `active`.
2. AMCL converges: `map → odom` TF present; `/global_costmap/costmap` and
   `/local_costmap/costmap` published.
3. Send a goal (RViz "Nav2 Goal" or `ros2 action send_goal /navigate_to_pose ...`).
4. **Pass criteria:** `/plan` is non-empty; `/cmd_vel` (TwistStamped) is published; the
   robot moves toward and reaches the goal within position/yaw tolerance; the
   `navigate_to_pose` action returns `SUCCEEDED`; no `TF_SELF_TRANSFORM` warnings.
5. SLAM-mode check: `--mapping --navigation` with `slam:=true` navigates while mapping.

Validation commands (per project conventions):
`ros2 lifecycle list`, `ros2 run tf2_tools view_frames`, `ros2 topic echo /cmd_vel --once`,
`ros2 action list`.

---

## 9. CI integration 🤖

- New manual workflow `nav2-smoke-test.yml` (`workflow_dispatch`, self-hosted GPU runner)
  modeled on `mapping-smoke-test.yml`: build the GPU sim image, run the navigation
  integration test in-container, upload logs/path artifacts.
- The integration test must be headless (`use_webots_gui:=false`) and goal-driven via the
  action API so it runs without an operator.

---

## 10. Risks & follow-ups ⚠️

- **cmd_vel type mismatch:** if Nav2 publishes unstamped `Twist`, the controller ignores
  it — verify `enable_stamped_cmd_vel: true` end-to-end.
- **Costmap footprint/inflation** must match MAGNI's real footprint from
  `magni_description`, or the planner will be over/under-conservative.
- **Odometry quality on hardware:** noisy odom degrades AMCL/controller; revisit IMU only
  if needed (consistent with ADR 002's IMU-optional stance).
- **Lifecycle/launch race conditions:** ensure `robot_state_publisher` and odom are up
  before Nav2 activates; reuse upstream `lifecycle_manager` autostart ordering.
- **Map provenance:** localization mode depends on a committed map; keep `maps/` in sync
  with the latest `magni_mapping` output.

---

## 11. References

- `src/magni_mapping/` — package layout, launch, and test patterns to mirror.
- `architecture/cartographer_design.md`, `architecture/adr/002-cartographer-2d.md` — sensor
  topics, frames, and the simulation-first methodology this spec follows.
- `.devcontainer/Dockerfile` — confirms `ros-jazzy-nav2-bringup` is preinstalled in dev.
- Nav2 docs: https://docs.nav2.org/
