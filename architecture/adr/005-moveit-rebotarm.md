# ADR 005 — MoveIt 2 for the reBot Arm

Date: 2026-07-09

Status: Proposed

## Context
The reBot Arm (Seeed B601-DM + gripper) is mounted on MAGNI and simulated (`ros2_control` wiring
in `src/magni_description/urdf/rebotarm.control.xacro` + `reBot_B601_DM_with_gripper.xacro`,
driven by `arm_controller`/`gripper_controller`, two `JointTrajectoryController`s spawned by
`src/magni_webots/launch/magni_spawn.launch.py`). There was no motion planning layer — driving the
arm meant hand-authoring joint trajectories.

The `src/rebotarm_ros2` git submodule already ships a complete MoveIt 2 config package,
`rebotarm_moveit_config` (SRDF, kinematics.yaml, joint_limits.yaml, moveit_controllers.yaml,
ompl_planning.yaml, a `demo.launch.py`). It's written for a **standalone** arm: its own xacro
entry point, its own mock `ros2_control_node`, a `world`→`base_link` virtual joint, and a
`moveit_controllers.yaml` targeting a controller named `rebotarm_controller` — none of which match
how the arm is actually wired up on MAGNI.

## Decision
Add a new sibling package `src/magni_moveit_config` (mirroring `magni_mapping`/
`magni_navigation`) that adapts the vendored SRDF/kinematics/joint_limits/ompl/controllers config
rather than editing the submodule in place, and points `move_group` at MAGNI's actual composed
URDF and already-running controllers instead of the vendored mock-hardware demo launch:

- `config/magni_rebotarm.srdf` — adapted from the vendored `rebotarm.srdf`: chain
  `base_link="rebotarm_base_link"` (MAGNI's copy of the arm xacro renames the root link to avoid
  colliding with MAGNI's own `base_link`), `tip_link="gripper_link"` (not `gripper_tcp` — that
  frame doesn't exist in MAGNI's vendored arm xacro; fabricating an unmeasured offset for it was
  rejected in favor of using the real last link directly), `virtual_joint` child_link
  `base_footprint` (MAGNI's actual URDF root, not `base_link` which sits partway down the tree).
- `config/moveit_controllers.yaml` — controller renamed `rebotarm_controller` → `arm_controller`
  to match what `rebotarm_controllers.yaml` actually spawns; `gripper_controller` name already
  matched.
- `config/kinematics.yaml`, `joint_limits.yaml`, `ompl_planning.yaml` — copied verbatim (group and
  joint names already match MAGNI's).
- `launch/move_group.launch.py` — starts only `move_group` + optional RViz (MoveIt motion
  planning plugin); does **not** start `ros2_control_node` or controller spawners, since
  `magni_spawn.launch.py`'s existing chain already brings up `arm_controller`/`gripper_controller`
  and `robot_state_publisher`. `robot_description` is built via `MoveItConfigsBuilder` from
  MAGNI's own `magni_description/urdf/magni.urdf.xacro`, using the same xacro mappings
  `magni_spawn.launch.py`'s `get_robot_spawner()` uses, so the planning model never drifts from
  the simulated one.
- A `publish_world_tf` arg (default `true`) adds a static `world`→`base_footprint` transform:
  needed for a stationary-arm MoveIt session since nothing else roots `base_footprint` to a fixed
  frame in that case. **This conflicts with `--mapping`/`--navigation`**, where `base_footprint`
  is already rooted via `odom` (from the diff-drive controller) — a TF frame can't have two
  parents. Set `publish_world_tf:=false` if composing MoveIt with mapping/navigation.
- Wired into `magni_sim_bringup.launch.py` via a `moveit` arg (default `false`) and a separate
  `moveit_rviz` arg — deliberately not reusing the existing `rviz` arg, since that launches the
  generic teleop/mapping/nav2 `basicconf.rviz`, a different RViz config than MoveIt's motion
  planning view. `scripts/launch-webots-simulation.sh --moveit` validates arm-family/arm-installed
  since this SRDF is rebotarm-specific (franka not supported).
- A `magni_integration_tests` smoke test (`test_moveit_smoke.py`) sends the arm a joint-space
  goal via a raw `moveit_msgs/action/MoveGroup` action call and verifies `/joint_states` settles
  there, mirroring the launch_testing conventions already used for mapping/Nav2. **Not** using the
  newer `moveit_py` Python API: `MoveItPy(config_dict=...)` fails with "Failed to load planning
  pipelines from parameter server" in this environment, and the same failure persists even
  routing the exact same config through a real `launch_ros.actions.Node` (`--params-file`) instead
  of `config_dict` — while `move_group` itself loads the identical config without issue. Root
  cause not fully isolated; raw `MoveGroup` action calls (the same pattern already used for
  Nav2's `NavigateToPose`) sidestep it entirely and are well-defined/stable enough to trust.

## Consequences
- New package `src/magni_moveit_config` (`package.xml`, `CMakeLists.txt`, `config/`, `launch/`).
- `.devcontainer/Dockerfile` gains `ros-jazzy-moveit` + `ros-jazzy-moveit-py` (the latter installed
  for completeness/future use even though the smoke test ended up not using it — see above).
- `src/magni_webots/launch/magni_sim_bringup.launch.py` and
  `scripts/launch-webots-simulation.sh` gain `moveit`/`--moveit` (+ `moveit_rviz`) support.
- Franka arm family is **not** supported by this MoveIt config — the SRDF's joint/group names are
  specific to the reBot Arm.
- No `gripper_tcp` frame — planning targets `gripper_link` directly. Adding a proper TCP frame
  (matching the physical grasp point between the fingers) is a good follow-up for precision
  pick-and-place, not a blocker for general arm motion planning.
- The hand-adapted `disable_collisions` matrix carries over the vendored arm-internal pairs
  as-is, but has **not** been verified against the real composed MAGNI+arm geometry (e.g. whether
  `link1`/`link2` need pairs disabled against `torso_frame`). Regenerate/verify this using
  MoveIt's self-collision matrix tooling against the actual composed URDF before relying on it for
  anything safety-critical.
- Whole-body planning while the mobile base also drives around (integrating with Nav2) is future
  work — today's static `world` TF assumes a stationary base, matching the "control the arm"
  scope of this ADR.
