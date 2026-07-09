# ADR 004 — Nav2 Navigation (SLAM mode) for MAGNI

Date: 2026-07-08

Status: Proposed

## Context
MAGNI needs autonomous point-to-point navigation. We already have a working, REP-105-compliant
localization/mapping stack: Cartographer (`src/magni_mapping`) publishes `/map` and broadcasts
`map`→`odom`, while `ros2_control`'s `diff_drive_controller` (`src/ubiquity_motor_ros2`) broadcasts
`odom`→`base_footprint` and publishes `/odom`. Nav2 packages (`ros-jazzy-navigation2`,
`ros-jazzy-nav2-bringup`) are already installed via apt but unused. Rather than duplicating
localization with AMCL against a static map file, we can run Nav2 in "SLAM mode": reuse the live
Cartographer map/TF and only add Nav2's planning/control/behavior layer on top.

## Decision
Add a new `magni_navigation` package (mirroring `magni_mapping`'s structure) that includes
`nav2_bringup`'s `navigation_launch.py` — not `bringup_launch.py`, `slam_launch.py`, or
`localization_launch.py` — since we neither run AMCL nor a static map file. A custom
`nav2_params.yaml` configures `global_frame: map` / `robot_base_frame: base_footprint` throughout,
`RegulatedPurePursuitController` for `controller_server` (simpler/more predictable than stock MPPI
for a first integration), `NavfnPlanner` for `planner_server`, and sets `enable_stamped_cmd_vel:
true` on every node in the pipeline that touches `/cmd_vel` (`controller_server`,
`behavior_server`, `velocity_smoother`, `collision_monitor` — traced via `navigation_launch.py`'s
remap chain: controller_server/behavior_server → `cmd_vel_nav`; velocity_smoother reads
`cmd_vel_nav`, writes `cmd_vel_smoothed`; collision_monitor reads `cmd_vel_smoothed`, writes the
final bare `/cmd_vel`) to match the project's existing `TwistStamped`-only `cmd_vel` convention
(see `ubiquity_motor_ros2`'s `diff_drive_controller` and `magni_teleop`'s
`publish_stamped_twist`).

## Consequences
- New package `src/magni_navigation` (`package.xml`, `CMakeLists.txt`, `config/nav2_params.yaml`,
  `launch/navigation.launch.py`).
- `src/magni_webots/launch/magni_sim_bringup.launch.py` gains a `navigation` launch argument
  (default `false`), conditionally including `magni_navigation`'s launch file.
- `scripts/launch-webots-simulation.sh` gains a `--nav2` flag that also forces `--mapping` on,
  since Nav2 in this mode depends on Cartographer's `/map`.
- `src/magni_control_station/config/basicconf.rviz` gains Global/Local Costmap and Global/Local
  Plan displays (topics `/global_costmap/costmap`, `/local_costmap/costmap`, `/plan`,
  `/local_plan`); the existing `2D Goal Pose` tool already publishes to `/goal_pose`, which
  `bt_navigator` already subscribes to directly — no RViz tool changes needed.
- New smoke test `src/magni_integration_tests/tests/test_navigation_smoke.py` and
  `scripts/test-navigation.sh`, following `test_speed_response.py`'s conventions, driving a
  `NavigateToPose` goal and checking the result against `TestSupervisor`'s ground-truth pose.
- No AMCL, no map file, no `map_server`/`map_saver` anywhere in this workspace.
- `enable_stamped_cmd_vel: true` must be kept in sync across `controller_server`,
  `behavior_server`, `velocity_smoother`, and `collision_monitor` in `nav2_params.yaml` — losing
  this on any one of them will silently break `/cmd_vel` delivery to the robot exactly as
  previously discovered with the plain teleop stack.
- Real-hardware LIDAR frame mismatch (`laser` in sim vs `laser_frame` + driver-provided static
  transform on hardware) is pre-existing and out of scope; Nav2's costmap sensor sources read
  `frame_id` from the `LaserScan` message itself, so no additional handling is required here.
- `robot_radius: 0.30` in `nav2_params.yaml` is a placeholder loosely derived from
  `wheel_separation: 0.33`; revisit against the real chassis footprint (a `footprint` polygon may
  fit better given the casters extend further front/back than the drive-wheel track width).
