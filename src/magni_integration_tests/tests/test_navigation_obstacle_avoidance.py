import math
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

from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose

# break_room.wbt's "cabinet(4)" (translation 1.4 2.5 0) sits (within ~1cm) directly on the
# straight line from the robot's spawn point (0, 0) to GOAL below -- confirmed live via /scan
# (an obstacle at ~2.2-2.5m in the 60-70deg bearing range matches this cabinet's near edge). A
# controller with no obstacle avoidance would drive straight through it.
#
# GOAL_X/GOAL_Y can't simply be "far past the obstacle": Cartographer's global costmap only
# covers its initial submap extent before the robot has explored (confirmed live at cold start in
# this world: x in [-2.90, 3.05], y in [-2.40, 3.75] -- a goal outside that is rejected by
# planner_server with "Goal Coordinates ... was outside bounds"). GOAL is chosen to sit just
# beyond the obstacle along the same line while staying comfortably inside those initial bounds.
OBSTACLE_X = 1.4
OBSTACLE_Y = 2.5
# Half-diagonal of the cabinet's footprint (~0.5m half-width x ~0.25m half-depth) plus the
# configured robot_radius (0.30, see nav2_params.yaml) is close to 0.9m -- comfortably clearing
# this threshold requires a real avoidance maneuver, while a straight-line collision path would
# pass within ~0m of the obstacle center.
MIN_CLEARANCE_M = 0.6

GOAL_X = 1.8
GOAL_Y = 3.2
GOAL_TOLERANCE_M = 0.5  # coarse: matches nav2's own xy_goal_tolerance (0.25) plus margin
NAV_ACTION_WAIT_SEC = 60.0
# Same rationale as test_navigation_smoke.py: collision_monitor's disabled-by-default
# FootprintApproach polygon aside, a cluttered world plus a longer/curved avoidance path takes
# longer than the open tech_office goal.
NAV_RESULT_TIMEOUT_SEC = 180.0


@launch_testing.ready_to_test_action_timeout(120.0)
def generate_test_description():
    pkg_webots = get_package_share_directory('magni_webots')
    pkg_mapping = get_package_share_directory('magni_mapping')
    pkg_navigation = get_package_share_directory('magni_navigation')

    webots_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(pkg_webots, 'launch', 'magni_spawn.launch.py')
        ),
        launch_arguments={'world': 'break_room.wbt'}.items(),
    )

    test_supervisor_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(pkg_webots, 'launch', 'test_supervisor.launch.py')
        ),
        launch_arguments={'track_nodes': 'magni'}.items(),
    )

    mapping_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(pkg_mapping, 'launch', 'mapping.launch.py')
        ), launch_arguments={'use_sim_time': 'true'}.items()
    )

    navigation_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, 'launch', 'navigation.launch.py')
        ), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Nav2's 10-node lifecycle bring-up is heavier than mapping alone -- give it more settle time.
    ready_timer = launch.actions.TimerAction(period=25.0, actions=[launch_testing.actions.ReadyToTest()])

    ld = launch.LaunchDescription([
        webots_launch,
        test_supervisor_launch,
        mapping_launch,
        navigation_launch,
        ready_timer,
    ])

    return ld, {}


class TestNavigationObstacleAvoidance(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_navigation_obstacle_avoidance')
        self.latest_pose = None
        # Every ground-truth pose sample seen during the whole navigation attempt, not just the
        # latest -- needed to check the path's closest approach to the obstacle, not just where
        # the robot ended up.
        self.pose_samples = []
        self.pose_sub = self.node.create_subscription(
            PoseStamped, '/test_supervisor/magni/pose', self._on_pose, 10)
        self.nav_client = ActionClient(self.node, NavigateToPose, 'navigate_to_pose')

    def tearDown(self):
        self.node.destroy_subscription(self.pose_sub)
        self.nav_client.destroy()
        self.node.destroy_node()

    def _on_pose(self, msg):
        self.latest_pose = msg
        self.pose_samples.append((msg.pose.position.x, msg.pose.position.y))

    def _wait_for_pose(self, timeout_sec):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and self.latest_pose is None:
            rclpy.spin_once(self.node, timeout_sec=1.0)
        return self.latest_pose

    def test_navigate_to_pose_avoids_static_obstacle(self):
        self.assertIsNotNone(
            self._wait_for_pose(30.0),
            'No pose received on /test_supervisor/magni/pose within timeout -- '
            'is the TestSupervisor plugin tracking "magni"?')

        self.assertTrue(
            self.nav_client.wait_for_server(timeout_sec=NAV_ACTION_WAIT_SEC),
            'navigate_to_pose action server not available -- did the Nav2 lifecycle nodes '
            'reach the active state? (check `ros2 lifecycle list`, especially route_server/'
            'docking_server)')

        # bt_navigator's action server can appear before every other lifecycle node's own action
        # server (e.g. planner_server's compute_path_to_pose) has finished being discovered over
        # the ROS graph -- give that a moment to settle before sending the first goal.
        deadline = time.time() + 5.0
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.5)

        # Reset samples so only the actual navigation attempt (not the settle wait above) counts
        # towards the closest-approach check below.
        self.pose_samples = []

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.node.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = GOAL_X
        goal_msg.pose.pose.position.y = GOAL_Y
        goal_msg.pose.pose.orientation.w = 1.0

        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.node, send_goal_future, timeout_sec=10.0)
        goal_handle = send_goal_future.result()
        self.assertIsNotNone(goal_handle, 'send_goal_async did not complete')
        self.assertTrue(goal_handle.accepted, 'NavigateToPose goal was rejected')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.node, result_future, timeout_sec=NAV_RESULT_TIMEOUT_SEC)
        self.assertTrue(result_future.done(), 'NavigateToPose did not finish within timeout')

        final_pose = self.latest_pose
        distance_to_goal = math.hypot(
            final_pose.pose.position.x - GOAL_X,
            final_pose.pose.position.y - GOAL_Y)
        self.assertLess(
            distance_to_goal, GOAL_TOLERANCE_M,
            f'Robot ended {distance_to_goal:.3f}m from goal ({GOAL_X}, {GOAL_Y}) -- Nav2 did not '
            'actually drive the robot there.')

        self.assertGreater(
            len(self.pose_samples), 0,
            'No ground-truth pose samples were recorded during the navigation attempt.')
        closest_approach = min(
            math.hypot(x - OBSTACLE_X, y - OBSTACLE_Y) for x, y in self.pose_samples)
        self.assertGreaterEqual(
            closest_approach, MIN_CLEARANCE_M,
            f'Robot came within {closest_approach:.3f}m of the obstacle at '
            f'({OBSTACLE_X}, {OBSTACLE_Y}) -- expected at least {MIN_CLEARANCE_M}m of clearance. '
            'This suggests the robot drove through/into the static obstacle instead of '
            'routing around it.')


# No post-shutdown exit-code assertion here -- see test_navigation_smoke.py's comment: with
# Cartographer, all ten Nav2 lifecycle nodes, and the full magni driver all tearing down together
# under load, some node reproducibly exits with a different incidental nonzero/crash code every
# run. The functional assertions above already confirm navigation and obstacle avoidance worked.
