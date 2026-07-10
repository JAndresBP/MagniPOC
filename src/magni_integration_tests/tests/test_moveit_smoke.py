import os
import time
import unittest

import launch
import launch.actions
import launch_testing
import launch_testing.actions
import rclpy
import rclpy.node
from rclpy.action import ActionClient
from ament_index_python.packages import get_package_share_directory

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
from sensor_msgs.msg import JointState

ARM_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
# Safely within every arm joint's actual limits and clearly different from the arm's all-zero
# spawn pose, so reaching it actually proves MoveIt planned+executed a real motion rather than a
# no-op. joint2/joint3 specifically have an upper limit of only 0.02 rad (padded from the
# upstream 0 -- see reBot_B601_DM_with_gripper.xacro's joint2/joint3 comments), so their targets
# must be negative, unlike the other four joints.
TARGET_POSITIONS = [0.3, -0.3, -0.3, 0.3, 0.3, 0.3]
JOINT_TOLERANCE_RAD = 0.05
JOINT_STATES_WAIT_SEC = 60.0
MOVE_ACTION_WAIT_SEC = 60.0
PLAN_AND_EXECUTE_TIMEOUT_SEC = 60.0


@launch_testing.ready_to_test_action_timeout(120.0)
def generate_test_description():
    pkg_webots = get_package_share_directory('magni_webots')
    pkg_moveit = get_package_share_directory('magni_moveit_config')

    webots_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(pkg_webots, 'launch', 'magni_spawn.launch.py')
        )
    )

    move_group_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit, 'launch', 'move_group.launch.py')
        ),
        launch_arguments={'use_rviz': 'false', 'publish_world_tf': 'true'}.items(),
    )

    # Controller bring-up (magni_spawn.launch.py's sequential chain) + move_group startup takes
    # a while.
    ready_timer = launch.actions.TimerAction(period=25.0, actions=[launch_testing.actions.ReadyToTest()])

    ld = launch.LaunchDescription([
        webots_launch,
        move_group_launch,
        ready_timer,
    ])

    return ld, {}


class TestMoveitSmoke(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_moveit_smoke')
        self.latest_joint_state = None
        self.joint_state_sub = self.node.create_subscription(
            JointState, '/joint_states', self._on_joint_state, 10)
        # move_group's action server, default name "move_action" (moveit_msgs/action/MoveGroup) --
        # raw action goal used instead of the moveit_py Python API: moveit_py's MoveItPy(...) does
        # not reliably load the "ompl" planning pipeline config in this environment (fails with
        # "Failed to load planning pipelines from parameter server" both in-process via
        # config_dict and via a real launch_ros Node with --params-file), even though move_group
        # itself loads the identical config fine -- see architecture/adr/005-moveit-rebotarm.md.
        self.move_action_client = ActionClient(self.node, MoveGroup, 'move_action')

    def tearDown(self):
        self.node.destroy_subscription(self.joint_state_sub)
        self.move_action_client.destroy()
        self.node.destroy_node()

    def _on_joint_state(self, msg):
        self.latest_joint_state = msg

    def _wait_for_arm_joint_states(self, timeout_sec):
        deadline = time.time() + timeout_sec
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=1.0)
            msg = self.latest_joint_state
            if msg is not None and all(name in msg.name for name in ARM_JOINTS):
                return msg
        return None

    def _current_arm_positions(self):
        msg = self.latest_joint_state
        name_to_position = dict(zip(msg.name, msg.position))
        return [name_to_position[name] for name in ARM_JOINTS]

    def test_moveit_plans_and_executes_to_target(self):
        self.assertIsNotNone(
            self._wait_for_arm_joint_states(JOINT_STATES_WAIT_SEC),
            'No /joint_states message containing the arm joints within timeout -- did '
            'arm_controller/joint_state_broadcaster activate? (magni_spawn.launch.py spawns '
            'controllers in sequence and this can take a while.)')

        self.assertTrue(
            self.move_action_client.wait_for_server(timeout_sec=MOVE_ACTION_WAIT_SEC),
            'move_action action server not available -- did move_group finish starting up?')

        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = 'arm'
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 10.0

        constraints = Constraints()
        for joint_name, target in zip(ARM_JOINTS, TARGET_POSITIONS):
            constraints.joint_constraints.append(JointConstraint(
                joint_name=joint_name,
                position=target,
                tolerance_above=0.01,
                tolerance_below=0.01,
                weight=1.0,
            ))
        goal_msg.request.goal_constraints = [constraints]
        goal_msg.planning_options.plan_only = False

        send_goal_future = self.move_action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=10.0)
        goal_handle = send_goal_future.result()
        self.assertIsNotNone(goal_handle, 'send_goal_async did not complete')
        self.assertTrue(goal_handle.accepted, 'MoveGroup goal was rejected')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=PLAN_AND_EXECUTE_TIMEOUT_SEC)
        self.assertTrue(result_future.done(), 'MoveGroup action did not finish within timeout')

        result = result_future.result().result
        self.assertEqual(
            result.error_code.val, 1,  # moveit_msgs/MoveItErrorCodes.SUCCESS
            f'MoveGroup did not succeed, error_code={result.error_code.val}')

        final_positions = self._current_arm_positions()
        self.assertTrue(
            all(abs(p - t) < JOINT_TOLERANCE_RAD for p, t in zip(final_positions, TARGET_POSITIONS)),
            f'Arm joints ended at {final_positions}, expected near {TARGET_POSITIONS} '
            f'(tolerance {JOINT_TOLERANCE_RAD} rad) -- MoveIt reported success but the arm did '
            'not actually reach the target.')


# No post-shutdown exit-code assertion here -- see test_mapping_smoke.py's/test_navigation_smoke.
# py's identical rationale: with the full magni driver (arm/gripper controllers) and move_group
# all tearing down together, some node reproducibly exits with a different incidental nonzero/
# crash code every run. The functional assertion above already confirms MoveIt actually worked.
