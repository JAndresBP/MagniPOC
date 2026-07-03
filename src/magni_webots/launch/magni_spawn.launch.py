import os
import launch
import xacro
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

def get_robot_spawner(*args):
    use_sim_time = LaunchConfiguration("use_sim_time")
        
    magni_description_pkg_share = get_package_share_directory('magni_description')
    magni_xacro_path = get_package_share_path('magni_description') / 'urdf' / 'magni.urdf.xacro'
    magni_description = xacro.process_file(magni_xacro_path, mappings = {'use_mock_hardware': 'True'}).toxml()
    
    magni_webots_pkg_share = get_package_share_directory('magni_webots')
    magni_wb_xacro_path = os.path.join(magni_webots_pkg_share,'resource', 'magni_wb.urdf.xacro')
    node_remover_xacro_path = os.path.join(magni_webots_pkg_share,'resource', 'node_remover.urdf.xacro')
    
    ubiquity_motor_pkg_share = get_package_share_directory('ubiquity_motor_ros2')
    robot_controllers = os.path.join(ubiquity_motor_pkg_share, "cfg", "conf.yaml")
    
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
            'use_mock_hardware': use_sim_time
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
    
    delay_joint_state_broadcaster_after_robot_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=robot_controller_spawner,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )
    
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
        magni_description_include
    ]    

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
