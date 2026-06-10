import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    teleop_arg = LaunchConfiguration('teleop')
    mapping_arg = LaunchConfiguration('mapping')
    rviz_arg = LaunchConfiguration('rviz')

    args = [
        DeclareLaunchArgument('teleop', default_value='false', description='Launch teleop'),
        DeclareLaunchArgument('mapping', default_value='true', description='Launch mapping'),
        DeclareLaunchArgument('rviz', default_value='true', description='Launch RViz'),
    ]

    # Include teleop
    teleop_share = get_package_share_directory('magni_teleop')
    teleop_launch = os.path.join(teleop_share, 'launch', 'joy_teleop.launch.py')
    include_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch),
        condition=IfCondition(teleop_arg)
    )

    # Include mapping
    mapping_share = get_package_share_directory('magni_mapping')
    mapping_launch = os.path.join(mapping_share, 'launch', 'mapping.launch.py')
    include_mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(mapping_launch),
        condition=IfCondition(mapping_arg)
    )

    # RViz using magni_control_station config
    rviz_config = os.path.join(get_package_share_directory('magni_control_station'), 'config', 'control_station.rviz')
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['--display-config=' + rviz_config],
        condition=IfCondition(rviz_arg)
    )

    ld = LaunchDescription(args)
    ld.add_action(include_teleop)
    ld.add_action(include_mapping)
    ld.add_action(rviz_node)

    return ld
