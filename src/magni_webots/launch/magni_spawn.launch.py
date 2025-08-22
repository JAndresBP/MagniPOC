import os
import launch
import xacro
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from webots_ros2_driver.webots_launcher import WebotsLauncher
from webots_ros2_driver.urdf_spawner import URDFSpawner,get_webots_driver_node
from webots_ros2_driver.webots_controller import WebotsController


def get_robot_spawner(*args):
    #magni_description_pkg_share = get_package_share_directory('magni_description')
    # magni_xacro_path = get_package_share_path('magni_description') / 'urdf' / 'magni.urdf.xacro'
    # magni_description = xacro.process_file(magni_xacro_path).toxml()
    magni_webots_pkg_share = get_package_share_directory('magni_webots')
    magni_xacro_path = os.path.join(magni_webots_pkg_share,'resource', 'magni_wb.urdf.xacro')
    magni_description = xacro.process_file(magni_xacro_path).toxml()
    
    robot_spawner_node = URDFSpawner(
        name='magni',
        robot_description=magni_description,
        relative_path_prefix=os.path.join(magni_webots_pkg_share),
        translation="0 0 0.1"
    )
    
    magni_driver = WebotsController(
        robot_name='magni',
        parameters=[
            {'robot_description': magni_xacro_path},
        ]
    )
    
    return [
        robot_spawner_node,
        launch.actions.RegisterEventHandler(
            event_handler=launch.event_handlers.OnProcessIO(
                target_action=robot_spawner_node,
                on_stdout=lambda event: get_webots_driver_node(
                    event, [
                        magni_driver
                    ]
                ),
            )
        )
    ]    

def generate_launch_description():
    # Package shares
    magni_webots_pkg_share = get_package_share_directory('magni_webots')
    
    # Launch Webots simulation environment
    world_file = os.path.join(magni_webots_pkg_share, 'worlds', 'break_room.wbt')

    webots_launcher = WebotsLauncher(
        world=world_file,
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

    return LaunchDescription([
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
    ] + get_robot_spawner())
