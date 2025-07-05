import os
import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, RegisterEventHandler
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.utils import controller_url_prefix

def generate_launch_description():
    # Package shares
    magni_webots_pkg_share = get_package_share_directory('magni_webots')
    magni_description_pkg_share = get_package_share_directory('magni_description')

    # 1. Include description_launch.py from magni_description
    # This will handle xacro processing to URDF and start robot_state_publisher, joint_state_publisher
    magni_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(magni_description_pkg_share, 'launch', 'description_launch.py')
        ),
        # Pass use_sim_time:=True if the description_launch.py supports it
        # and if it's necessary for its nodes.
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # 2. Launch Webots simulation environment
    world_file = os.path.join(magni_webots_pkg_share, 'worlds', 'break_room.wbt')

    webots_launcher = WebotsLauncher(
        world=world_file,
        # Set to false if Ros2Supervisor is started manually or if not needed immediately
        # However, for spawning URDFs, the supervisor is essential.
        # Ros2Supervisor is typically started by WebotsLauncher by default.
        ros2_supervisor=True
    )

    # 3. Spawn the Magni robot using URDF
    # The URDF content is published as a string on the /robot_description topic
    # by the included magni_description_launch.
    # We need to call the /spawn_urdf_robot service provided by the Ros2Supervisor.
    # This can be done using an ExecuteProcess action that calls `ros2 service call`.
    # However, timing is critical: the service call must happen after the supervisor is ready.

    # A common pattern is to use a delayed action or an event handler.
    # Let's try to use a Node that represents the robot driver, which Webots will then link.
    # If the robot is spawned via supervisor, its controller needs to be set to <extern>
    # and a corresponding ROS 2 node (driver) must be running.

    # The webots_ros2_driver package has a utility `spawn_robot_from_urdf`
    # which is a script that calls the service.
    # `webots_ros2_driver.webots_controller.Ros2Controller` can be used if the robot is already in the world.
    # For spawning, `Ros2Supervisor.spawn_robot_from_urdf` is the service.

    # Option A: Use `ExecuteProcess` to call the service. This is a bit clunky for getting URDF.
    # Better: `webots_ros2_driver.urdf_spawner.UrdfSpawner` (if such a node/utility exists directly for launch)
    # Or a custom script/node that waits for supervisor and /robot_description and then calls.

    # Let's define the spawner as a Node that is part of webots_ros2_driver, if available,
    # or construct the service call.
    # The `webots_ros2_driver` provides `robot_description_spawner` which is a C++ node.
    # It subscribes to `/robot_description` and calls the spawn service.

    # This spawner node will take the URDF from /robot_description
    # and spawn a robot named "Magni".
    robot_spawner_node = Node(
        package='webots_ros2_driver',
        executable='robot_description_spawner', # This is the C++ node
        output='screen',
        name='magni_urdf_spawner',
        namespace='Magni', # The robot's namespace
        parameters=[{
            'robot_name': 'Magni',
            'relative_path_prefix': magni_description_pkg_share, # For resolving package:// in URDF meshes
            # 'spawn_rotation': '0 0 1 0', # Example: SFrotation for initial orientation if needed
            # 'spawn_translation': '0 0 0.1', # Example: SFvec3f for initial position if needed
            'custom_data': '', # Optional
            'robot_description_topic': '/robot_description', # Default, but good to be explicit
        }],
        # Ensures this runs after Webots (and supervisor) is likely up.
        # This is not a strict guarantee, a more robust way is needed if this fails.
        # Could use an event handler for when the supervisor service is available.
    )

    # The robot once spawned will need its driver.
    # If the spawner sets the controller to "<extern>", WebotsLauncher might auto-start a driver.
    # Or, we need to explicitly start the driver node.
    # The `robot_description_spawner` sets the controller to "<extern>" by default.
    # The `WebotsLauncher` should then automatically start a driver node for "Magni".
    # The parameters for this auto-started driver can be specified in `WebotsLauncher`
    # via `controller_configurations`.

    # For example, if we need to pass specific params to the Magni driver:
    # magni_driver_params = os.path.join(magni_webots_pkg_share, 'config', 'magni_driver.yaml') # if you have one
    # webots_launcher.controller_configurations = {
    #     'Magni': { # Name of the robot in Webots
    #         'parameters': {
    #             'robot_description': '/robot_description', # Path or topic
    #             'use_sim_time': True,
    #             # ... other params for Magni's specific driver ...
    #         }
    #     }
    # }
    # For now, assume default driver parameters are okay or handled by the spawner/auto-driver.

    return LaunchDescription([
        webots_launcher,  # Starts Webots and the Ros2Supervisor
        magni_description_launch, # Starts robot_state_publisher etc. and publishes /robot_description

        # Delay the spawner until the supervisor is likely ready.
        # A more robust solution would be to wait for the spawn service to be available.
        # One way is to use RegisterEventHandler with OnProcessStart for the supervisor node.
        # The supervisor node is started by WebotsLauncher, its name is usually 'Ros2Supervisor'.
        # However, Ros2Supervisor is a C++ node, not a launch action we directly add here.

        # Simpler approach: launch the spawner, it will retry until supervisor is available.
        robot_spawner_node,

        # Gracefully terminate Webots when the spawner exits (e.g. if it errors out or completes)
        # or when the main launch is asked to shut down.
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots_launcher,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
    ])
