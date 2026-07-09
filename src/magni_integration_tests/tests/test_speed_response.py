import math
import os
import signal
import time
import unittest

import launch
import launch.actions
import launch_testing
import launch_testing.actions
import rclpy
import rclpy.node
from ament_index_python.packages import get_package_share_directory

from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry

# ubiquity_velocity_controller's cmd_vel_timeout (see ubiquity_motor_ros2/cfg/conf.yaml) stops the
# robot if it doesn't see a fresh command within this window, so cmd_vel must be republished faster
# than this for the whole drive duration.
CMD_VEL_PUBLISH_RATE_HZ = 10.0
DRIVE_LINEAR_SPEED = 0.2
DRIVE_DURATION_SEC = 3.0
MIN_EXPECTED_DISPLACEMENT_M = 0.05


@launch_testing.ready_to_test_action_timeout(90.0)
def generate_test_description():
    pkg_webots = get_package_share_directory('magni_webots')

    webots_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(pkg_webots, 'launch', 'magni_spawn.launch.py')
        )
    )

    test_supervisor_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(pkg_webots, 'launch', 'test_supervisor.launch.py')
        ),
        launch_arguments={'track_nodes': 'magni'}.items(),
    )

    # Give Webots, the magni spawn/controllers, and the test supervisor time to come up.
    ready_timer = launch.actions.TimerAction(period=15.0, actions=[launch_testing.actions.ReadyToTest()])

    ld = launch.LaunchDescription([
        webots_launch,
        test_supervisor_launch,
        ready_timer,
    ])

    return ld, {}


class TestSpeedResponse(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_speed_response')
        self.latest_pose = None
        self.odom_received = False
        self.pose_sub = self.node.create_subscription(
            PoseStamped, '/test_supervisor/magni/pose',
            self._on_pose, 10)
        self.odom_sub = self.node.create_subscription(
            Odometry, '/odom', self._on_odom, 10)

    def tearDown(self):
        self.node.destroy_subscription(self.pose_sub)
        self.node.destroy_subscription(self.odom_sub)
        self.node.destroy_node()

    def _on_pose(self, msg):
        self.latest_pose = msg

    def _on_odom(self, msg):
        self.odom_received = True

    def _wait_for_pose(self, timeout_sec):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and self.latest_pose is None:
            rclpy.spin_once(self.node, timeout_sec=1.0)
        return self.latest_pose

    def _wait_for_odom(self, timeout_sec):
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not self.odom_received:
            rclpy.spin_once(self.node, timeout_sec=1.0)
        return self.odom_received

    def test_forward_speed_moves_robot(self):
        # /odom is only published once ubiquity_velocity_controller has been loaded, configured
        # and activated -- magni_spawn.launch.py spawns controllers in sequence (velocity, then
        # joint_state_broadcaster, then arm, then gripper), which can take longer than the sim's
        # own startup, so cmd_vel published before this is silently dropped.
        self.assertTrue(
            self._wait_for_odom(60.0),
            'No /odom message received within timeout -- ubiquity_velocity_controller does not '
            'appear to be active.')

        initial_pose = self._wait_for_pose(30.0)
        self.assertIsNotNone(
            initial_pose,
            'No pose received on /test_supervisor/magni/pose within timeout -- '
            'is the TestSupervisor plugin tracking "magni"?')
        start_x = initial_pose.pose.position.x
        start_y = initial_pose.pose.position.y

        # ubiquity_velocity_controller expects TwistStamped, not plain Twist -- see
        # scripts/run-teleop-keyboard.sh's `-p stamped:=true` and joy_teleop.launch.py's
        # `publish_stamped_twist`.
        cmd_vel_pub = self.node.create_publisher(TwistStamped, '/cmd_vel', 10)
        try:
            deadline = time.time() + DRIVE_DURATION_SEC
            period = 1.0 / CMD_VEL_PUBLISH_RATE_HZ
            while time.time() < deadline:
                msg = TwistStamped()
                msg.header.stamp = self.node.get_clock().now().to_msg()
                msg.twist.linear.x = DRIVE_LINEAR_SPEED
                cmd_vel_pub.publish(msg)
                rclpy.spin_once(self.node, timeout_sec=period)

            # Stop the robot.
            stop_msg = TwistStamped()
            stop_msg.header.stamp = self.node.get_clock().now().to_msg()
            cmd_vel_pub.publish(stop_msg)
        finally:
            self.node.destroy_publisher(cmd_vel_pub)

        # Let the supervisor publish a fresh pose after the drive.
        deadline = time.time() + 5.0
        while time.time() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.5)

        final_pose = self.latest_pose
        end_x = final_pose.pose.position.x
        end_y = final_pose.pose.position.y

        displacement = math.hypot(end_x - start_x, end_y - start_y)
        self.assertGreater(
            displacement, MIN_EXPECTED_DISPLACEMENT_M,
            f'Robot only moved {displacement:.3f}m after driving forward for '
            f'{DRIVE_DURATION_SEC}s at {DRIVE_LINEAR_SPEED}m/s -- cmd_vel does not '
            'appear to be reaching the robot.')


@launch_testing.post_shutdown_test()
class TestSpeedResponsePostShutdown(unittest.TestCase):
    def test_exit_codes(self, proc_info):
        # Webots, Ros2Supervisor and the various controller/robot nodes are long-running and are
        # expected to be terminated at teardown once the test finishes, not to exit on their own:
        # plain rclpy nodes exit -SIGINT, while Cyberbotics' webots-controller binary (used for
        # the magni/NodeRemover/TestSupervisor extern controllers) exits 243 on termination.
        launch_testing.asserts.assertExitCodes(proc_info, allowable_exit_codes=[0, -signal.SIGINT, 243])
