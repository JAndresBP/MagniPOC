"""
Spawns a supervisor-only "TestSupervisor" robot that publishes ground-truth
PoseStamped for a configurable list of tracked Webots node names (see
magni_webots::TestSupervisor / resource/test_supervisor.urdf.xacro).

Test-only: not included by magni_sim_bringup.launch.py or magni_bringup.launch.py,
so normal teleop/bringup runs are unaffected. Include this *after*
magni_webots/launch/magni_spawn.launch.py, which brings up Ros2Supervisor
(providing the /Ros2Supervisor/spawn_node_from_string service this depends on).
"""
import tempfile
import xacro
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_path
from webots_ros2_driver.webots_controller import WebotsController


def _spawn_test_supervisor(context, *args, **kwargs):
    track_nodes = LaunchConfiguration('track_nodes').perform(context)

    test_supervisor_xacro_path = get_package_share_path('magni_webots') / 'resource' / 'test_supervisor.urdf.xacro'
    test_supervisor_description = xacro.process_file(
        str(test_supervisor_xacro_path),
        mappings={'track_nodes': track_nodes},
    ).toxml()

    # WebotsNode's "pass robot_description as an inline string" code path is deprecated and
    # doesn't reliably carry custom <plugin> attributes through -- write the processed xacro to
    # a temp file and pass that path instead, same as the merged controller params file in
    # magni_spawn.launch.py.
    test_supervisor_description_file = tempfile.NamedTemporaryFile(
        mode='w', prefix='test_supervisor_', suffix='.urdf', delete=False
    )
    test_supervisor_description_file.write(test_supervisor_description)
    test_supervisor_description_file.close()
    test_supervisor_description = test_supervisor_description_file.name

    # Mirrors how magni_spawn.launch.py spawns the "NodeRemover" supervisor robot.
    supervisor_robot_description = (
        'data: Robot { supervisor TRUE name "TestSupervisor" controller "<extern>"}'
    )

    spawn_supervisor = ExecuteProcess(
        cmd=[
            'ros2', 'service', 'call',
            '/Ros2Supervisor/spawn_node_from_string',
            'webots_ros2_msgs/srv/SpawnNodeFromString',
            supervisor_robot_description,
        ],
        output='log',
    )

    test_supervisor_controller = WebotsController(
        robot_name='TestSupervisor',
        parameters=[{'robot_description': test_supervisor_description}],
    )

    return [spawn_supervisor, test_supervisor_controller]


def generate_launch_description():
    track_nodes_arg = DeclareLaunchArgument(
        'track_nodes',
        default_value='magni',
        description='Comma-separated Webots node names to publish ground-truth PoseStamped for',
    )

    # Give Ros2Supervisor (started by magni_spawn.launch.py's WebotsLauncher) a
    # moment to come up before calling its spawn service.
    delayed_spawn = TimerAction(period=3.0, actions=[OpaqueFunction(function=_spawn_test_supervisor)])

    return LaunchDescription([
        track_nodes_arg,
        delayed_spawn,
    ])
