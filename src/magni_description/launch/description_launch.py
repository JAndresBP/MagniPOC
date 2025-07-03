from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time',default_value='false',choices=['true','false'], description='sim time'),
    DeclareLaunchArgument('tower_installed',default_value='false',choices=['true','false'], description='tower'),
    DeclareLaunchArgument('shell_installed',default_value='false',choices=['true','false'], description='shell'),
    DeclareLaunchArgument('sonars_installed',default_value='false',choices=['true','false'], description='sonars'),
    DeclareLaunchArgument('camera_extrinsics_file',default_value='extrinsics/camera_extrinsics_forward.yaml', description='path to camera extrinsics file'),
    DeclareLaunchArgument('lidar_extrinsics_file',default_value='extrinsics/lidar_extrinsics_top_plate_center.yaml', description='path to lidar extrinsics file')
]

def generate_launch_description():
    path_to_urdf = get_package_share_path('magni_description') / 'urdf' / 'magni.urdf.xacro'
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', str(path_to_urdf), ' ',
                            ' tower_installed:=', LaunchConfiguration('tower_installed'),
                            ' shell_installed:=', LaunchConfiguration('shell_installed'),
                            ' sonars_installed:=', LaunchConfiguration('sonars_installed'),
                            ' lidar_extrinsics_file:=', LaunchConfiguration('lidar_extrinsics_file'),
                            ' camera_extrinsics_file:=', LaunchConfiguration('camera_extrinsics_file')
                            ]),
                value_type=str
            )
        }])
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{
            'use_gui': False
        }])
    
    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    return ld