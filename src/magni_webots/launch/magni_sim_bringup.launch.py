import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description() -> LaunchDescription:
    # Launch argument to enable RViz (default: false)
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Whether to launch RViz'
    )

    # Launch argument to enable mapping (default: false)
    mapping_arg = DeclareLaunchArgument(
        'mapping',
        default_value='false',
        description='Whether to launch mapping subsystem'
    )

    # Launch argument to enable Nav2 navigation (default: false). SLAM mode: relies on
    # magni_mapping's Cartographer for /map + map->odom, so it needs mapping:=true too.
    navigation_arg = DeclareLaunchArgument(
        'navigation',
        default_value='false',
        description='Whether to launch Nav2 navigation subsystem (requires mapping)'
    )

    rviz_config = os.path.join(
        get_package_share_directory('magni_control_station'),
        'config',
        'basicconf.rviz'
    )
    
    magni_webots_pkg_share = get_package_share_directory('magni_webots')
    magni_webots_launch = os.path.join(
        magni_webots_pkg_share,
        'launch',
        'magni_spawn.launch.py'
    )
    
    magni_mapping_pkg_share = get_package_share_directory('magni_mapping')
    magni_mapping_launch = os.path.join(
        magni_mapping_pkg_share,
        'launch',
        'mapping.launch.py'
    )

    magni_navigation_pkg_share = get_package_share_directory('magni_navigation')
    magni_navigation_launch = os.path.join(
        magni_navigation_pkg_share,
        'launch',
        'navigation.launch.py'
    )

    webots_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            magni_webots_launch
        )
    )

    mapping_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            magni_mapping_launch
        ),
        condition=IfCondition(LaunchConfiguration('mapping'))
    )

    navigation_launcher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            magni_navigation_launch
        ),
        condition=IfCondition(LaunchConfiguration('navigation'))
    )

    # RViz node, launched only if "rviz" is true
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    ld = LaunchDescription()
    ld.add_action(rviz_arg)
    ld.add_action(mapping_arg)
    ld.add_action(navigation_arg)
    ld.add_action(webots_launcher)
    ld.add_action(mapping_launcher)
    ld.add_action(navigation_launcher)
    ld.add_action(rviz_node)
    return ld
