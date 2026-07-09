import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')

    pkg_share = get_package_share_directory('magni_navigation')
    default_params_file = os.path.join(pkg_share, 'config', 'nav2_params.yaml')

    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    navigation_launch_file = os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')

    # SLAM mode: Cartographer (magni_mapping) already owns /map and map->odom, so we include only
    # nav2_bringup's navigation_launch.py (controller/planner/behavior/bt_navigator/velocity
    # smoother/collision monitor) -- not bringup_launch.py/slam_launch.py/localization_launch.py,
    # which would pull in map_server/AMCL/a second SLAM stack we don't want.
    navigation_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(navigation_launch_file),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file,
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('autostart', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=default_params_file),
        navigation_launcher,
    ])
