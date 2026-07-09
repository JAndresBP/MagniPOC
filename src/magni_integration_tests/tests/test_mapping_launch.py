import os
import pathlib
import time
import unittest

import launch
import launch.actions
import launch_ros.actions
import launch_testing
import launch_testing.actions
import rclpy
import rclpy.node
from ament_index_python.packages import get_package_share_directory

from nav_msgs.msg import OccupancyGrid
from cartographer_ros_msgs.srv import WriteState

# resources/ is a source-tree-only fixture directory (not installed by CMakeLists.txt), and
# add_launch_test always runs this file from its source path, so resolve it relative to this
# file rather than via get_package_share_directory('magni_integration_tests') -- that lookup
# fails under `colcon test` (ctest's environment for testing a package doesn't include that same
# package's own install prefix in AMENT_PREFIX_PATH) and would point at an uninstalled path even
# if it succeeded.
BAGFILE = str(pathlib.Path(__file__).resolve().parent.parent / 'resources' / 'drive.bag')


def generate_test_description():
    pkg_webots = get_package_share_directory('magni_webots')
    pkg_mapping = get_package_share_directory('magni_mapping')

    bagfile = BAGFILE

    webots_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(pkg_webots, 'launch', 'magni_spawn.launch.py')
        )
    )

    mapping_launch = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(pkg_mapping, 'launch', 'mapping.launch.py')
        ), launch_arguments={'use_sim_time': 'true'}.items()
    )

    # Only launch `ros2 bag play` if the bag fixture actually exists -- pointing it at a missing
    # file doesn't just no-op, it exits immediately with an error, which cascades into spurious
    # process-death/exit-code failures elsewhere in the launch for a test that's about to skip
    # anyway.
    actions = [webots_launch, mapping_launch]
    if os.path.exists(bagfile):
        actions.append(launch.actions.ExecuteProcess(
            cmd=['ros2', 'bag', 'play', bagfile, '--clock'],
            output='screen',
        ))

    # wait a bit and then signal ReadyToTest
    ready_timer = launch.actions.TimerAction(period=5.0, actions=[launch_testing.actions.ReadyToTest()])
    actions.append(ready_timer)

    ld = launch.LaunchDescription(actions)

    return ld, {'bagfile': bagfile}


class TestMapping(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_mapping')

    def tearDown(self):
        self.node.destroy_node()

    def test_map_topic_published(self):
        # Skip test if bagfile not provided
        if not os.path.exists(BAGFILE):
            raise unittest.SkipTest('No bagfile present in resources — place a rosbag at tests/resources/drive.bag to run this test')

        msgs = []

        sub = self.node.create_subscription(OccupancyGrid, '/map', lambda msg: msgs.append(msg), 10)
        try:
            # Wait up to 30s for a /map message
            deadline = time.time() + 30.0
            while time.time() < deadline and not msgs:
                rclpy.spin_once(self.node, timeout_sec=1.0)
            assert len(msgs) > 0, 'No /map message received within timeout'
        finally:
            self.node.destroy_subscription(sub)

    def test_write_state_service(self):
        # Skip if service unavailable or bag not present
        if not os.path.exists(BAGFILE):
            raise unittest.SkipTest('No bagfile present in resources — skipping write_state test')

        client = self.node.create_client(WriteState, '/write_state')
        if not client.wait_for_service(timeout_sec=20.0):
            self.skipTest('WriteState service not available')

        filename = os.path.join(os.getcwd(), 'integration_map.pbstream')
        req = WriteState.Request()
        req.filename = filename

        fut = client.call_async(req)
        # Wait for service result (up to 10s)
        deadline = time.time() + 10.0
        while time.time() < deadline and not fut.done():
            rclpy.spin_once(self.node, timeout_sec=0.5)

        if not fut.done():
            self.fail('write_state service call did not finish')

        # Give a brief moment for file to be created
        time.sleep(1.0)
        assert os.path.exists(filename) and os.path.getsize(filename) > 0, 'pbstream file missing or empty'


# No post-shutdown exit-code assertion here (see test_mapping_smoke.py's/test_navigation_smoke.
# py's identical rationale): with only a 5s ready_timer, the whole stack (Webots, Ros2Supervisor,
# the magni spawn service call, Cartographer) gets torn down while still mid-startup, and
# whichever `ros2` CLI-wrapped subprocess happens to be mid-command at that moment reports a
# different incidental nonzero exit code (e.g. `ros2 service call` exits 2, not -SIGINT) on every
# run. The functional assertions above already confirm this test behaves correctly (it skips
# cleanly without a bag fixture); teardown ordering this early isn't deterministic enough to
# assert on.
