import os
import tempfile
import launch
import xacro
import yaml
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument, ExecuteProcess
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.urdf_spawner import URDFSpawner,get_webots_driver_node
from webots_ros2_driver.webots_controller import WebotsController
from webots_ros2_driver.wait_for_controller_connection import WaitForControllerConnection
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition


def _deep_merge(base, overlay):
    merged = dict(base)
    for key, value in overlay.items():
        if key in merged and isinstance(merged[key], dict) and isinstance(value, dict):
            merged[key] = _deep_merge(merged[key], value)
        else:
            merged[key] = value
    return merged


def get_robot_spawner(*args):
    use_sim_time = LaunchConfiguration("use_sim_time")
        
    magni_description_pkg_share = get_package_share_directory('magni_description')
    magni_xacro_path = get_package_share_path('magni_description') / 'urdf' / 'magni.urdf.xacro'
    # xacro is processed immediately (no LaunchContext exists yet at this point in
    # the launch description, so LaunchConfiguration substitutions can't be used
    # here) -- read overrides from the environment instead, set by the calling
    # shell script (see scripts/launch-webots-simulation.sh).
    arm_family = os.environ.get('MAGNI_ARM_FAMILY', 'rebotarm')

    magni_description = xacro.process_file(magni_xacro_path, mappings = {
        'use_mock_hardware': 'True',
        'arm_installed': os.environ.get('MAGNI_ARM_INSTALLED', 'true'),
        'arm_family': arm_family,
        'robot_type': os.environ.get('MAGNI_ARM_ROBOT_TYPE', 'fr3'),
        'arm_hand': os.environ.get('MAGNI_ARM_HAND', 'true'),
        'torso_installed': os.environ.get('MAGNI_TORSO_INSTALLED', 'true'),
        'torso_left_accessory': os.environ.get('MAGNI_TORSO_LEFT', 'tray'),
        'torso_head_accessory': os.environ.get('MAGNI_TORSO_HEAD', 'screen'),
    }).toxml()

    magni_webots_pkg_share = get_package_share_directory('magni_webots')
    magni_wb_xacro_path = os.path.join(magni_webots_pkg_share,'resource', 'magni_wb.urdf.xacro')
    node_remover_xacro_path = os.path.join(magni_webots_pkg_share,'resource', 'node_remover.urdf.xacro')
    # Only one arm is ever mounted at a time, so only that arm's controller yaml
    # gets merged in -- the other family's joints don't exist in the URDF and a
    # controller referencing missing joints would fail to activate.
    arm_controllers_file = os.path.join(
        magni_webots_pkg_share, 'resource',
        'rebotarm_controllers.yaml' if arm_family == 'rebotarm' else 'franka_arm_controllers.yaml'
    )

    ubiquity_motor_pkg_share = get_package_share_directory('ubiquity_motor_ros2')
    robot_controllers = os.path.join(ubiquity_motor_pkg_share, "cfg", "conf.yaml")

    # WebotsController only ever emits a single "--params-file" flag no matter how
    # many file paths are in `parameters=[...]` (see webots_ros2_driver's
    # WebotsController), so passing both yaml files separately produces a
    # malformed command line and the controller process dies on startup. Merge
    # them into one generated file instead.
    merged_controller_params = {}
    for params_path in (robot_controllers, arm_controllers_file):
        with open(params_path) as f:
            data = yaml.safe_load(f) or {}
        merged_controller_params = _deep_merge(merged_controller_params, data)

    merged_params_file = tempfile.NamedTemporaryFile(
        mode='w', prefix='magni_webots_controllers_', suffix='.yaml', delete=False
    )
    yaml.safe_dump(merged_controller_params, merged_params_file)
    merged_params_file.close()
    robot_controllers = merged_params_file.name

    robot_spawner_node = URDFSpawner(
        name='magni',
        robot_description=magni_description,
        relative_path_prefix=os.path.join(magni_description_pkg_share),
        translation="0 0 0"
    )
    
    magni_driver = WebotsController(
        robot_name='magni',
        parameters=[
            {
                'robot_description': magni_wb_xacro_path,
                'use_sim_time': use_sim_time
            },
            robot_controllers,
        ],
        remappings=[
            ('/ubiquity_velocity_controller/cmd_vel', '/cmd_vel'),  # Remap the cmd_vel topic
            ('/ubiquity_velocity_controller/odom', '/odom')         # Remap odom
        ]
    )
    
    magni_description_launch = os.path.join(
        magni_description_pkg_share,
        'launch',
        'description.launch.py'
    )
    
    magni_description_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(magni_description_launch),
        launch_arguments={
            'use_mock_hardware': use_sim_time,
            'arm_installed': os.environ.get('MAGNI_ARM_INSTALLED', 'true'),
            'arm_family': arm_family,
            'robot_type': os.environ.get('MAGNI_ARM_ROBOT_TYPE', 'fr3'),
            'arm_hand': os.environ.get('MAGNI_ARM_HAND', 'true'),
            'torso_installed': os.environ.get('MAGNI_TORSO_INSTALLED', 'true'),
            'torso_left_accessory': os.environ.get('MAGNI_TORSO_LEFT', 'tray'),
            'torso_head_accessory': os.environ.get('MAGNI_TORSO_HEAD', 'screen'),
        }.items()
    )
     
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"]
    )
    
    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["ubiquity_velocity_controller"],
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["arm_controller"],
        parameters=[
            {'use_sim_time': use_sim_time},
        ],
    )

    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delay_arm_controller_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[arm_controller_spawner],
        )
    )

    # Only the reBot arm's gripper is wired up as a separate controller so far --
    # gripper_controller.ros__parameters only exists in rebotarm_controllers.yaml.
    extra_actions = []
    if arm_family == 'rebotarm':
        gripper_controller_spawner = Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=["gripper_controller"],
            parameters=[
                {'use_sim_time': use_sim_time},
            ],
        )
        extra_actions.append(RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[gripper_controller_spawner],
            )
        ))

    waiting_nodes = WaitForControllerConnection(
        target_driver=magni_driver, nodes_to_start=robot_controller_spawner
    )
    
    
    # spawn supervisor logic copied from https://github.com/Ekumen-OS/andino_webots/blob/humble/andino_webots/launch/remove_nodes.launch.py
    supervisor_robot_description = """
        data: Robot { supervisor TRUE name "NodeRemover" controller "<extern>"}
        """
    
    command = [
        "ros2",
        "service",
        "call",
        "/Ros2Supervisor/spawn_node_from_string",
        "webots_ros2_msgs/srv/SpawnNodeFromString",
        supervisor_robot_description,
    ]
    
    spawn_supervisor = ExecuteProcess(
        cmd=command,
        output="log"
    )
    
    
    node_remover_controller = WebotsController(
        robot_name="NodeRemover",
        parameters=[
            {
                'robot_description': node_remover_xacro_path
            },
        ]
    )
    
    return [
        robot_spawner_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                target_action=robot_spawner_node,
                on_stdout=lambda event: get_webots_driver_node(
                    event, [
                        magni_driver,
                        spawn_supervisor,
                        node_remover_controller
                    ]
                ),
            )
        ),
        waiting_nodes,
        delay_joint_state_broadcaster_after_robot_controller_spawner,
        delay_arm_controller_after_joint_state_broadcaster_spawner,
        magni_description_include
    ] + extra_actions

def generate_launch_description():
    use_webots_gui_arg = DeclareLaunchArgument("use_webots_gui", default_value='false')
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true",)
    
    # Package shares
    magni_webots_pkg_share = get_package_share_directory('magni_webots')
    
    # Launch Webots simulation environment
    world_file = os.path.join(magni_webots_pkg_share, 'worlds', 'tech_office.wbt')

    webots_launcher = WebotsLauncher(
        world=world_file,
        gui=LaunchConfiguration("use_webots_gui"),
        # Set to false if Ros2Supervisor is started manually or if not needed immediately
        # However, for spawning URDFs, the supervisor is essential.
        # Ros2Supervisor is typically started by WebotsLauncher by default.
        ros2_supervisor=True   
    )
    
    reset_handler = launch.actions.RegisterEventHandler(
        event_handler=launch.event_handlers.OnProcessExit(
            target_action=webots_launcher._supervisor,
            on_exit=get_robot_spawner
        )
    )

    teleop_twist_joy_pkg_share = get_package_share_directory('teleop_twist_joy')
    teleop_twist_joy_launch = os.path.join(
        teleop_twist_joy_pkg_share,
        'launch',
        'teleop-launch.py'
    )
    
    teleop_twist_joy_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_twist_joy_launch),
        launch_arguments={
            'joy_config': 'ps3',
            'publish_stamped_twist': 'True'
        }.items()
    )
    
    return LaunchDescription([
        use_webots_gui_arg,
        use_sim_time_arg,
        webots_launcher,
        webots_launcher._supervisor,

        # Gracefully terminate Webots when the spawner exits (e.g. if it errors out or completes)
        # or when the main launch is asked to shut down.
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessExit(
                target_action=webots_launcher,
                on_exit=[launch.actions.EmitEvent(event=launch.events.Shutdown())],
            )
        ),
        reset_handler,
        teleop_twist_joy_include,
    ] + get_robot_spawner())
