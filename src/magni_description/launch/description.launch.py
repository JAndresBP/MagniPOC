from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument
from launch.substitutions.launch_configuration import LaunchConfiguration

ARGUMENTS = [
    DeclareLaunchArgument('use_mock_hardware',default_value='false',choices=['true','false'], description='use_mock_hardware'),
    DeclareLaunchArgument('tower_installed',default_value='false',choices=['true','false'], description='tower'),
    DeclareLaunchArgument('shell_installed',default_value='false',choices=['true','false'], description='shell'),
    DeclareLaunchArgument('sonars_installed',default_value='false',choices=['true','false'], description='sonars'),
    DeclareLaunchArgument('camera_extrinsics_file',default_value='extrinsics/camera_extrinsics_forward.yaml', description='path to camera extrinsics file'),
    DeclareLaunchArgument('lidar_extrinsics_file',default_value='extrinsics/lidar_extrinsics_top_plate_center.yaml', description='path to lidar extrinsics file'),
    DeclareLaunchArgument('arm_installed',default_value='false',choices=['true','false'], description='mount an arm'),
    DeclareLaunchArgument('arm_family',default_value='rebotarm',choices=['rebotarm','franka'], description='which arm to mount'),
    DeclareLaunchArgument('robot_type',default_value='fr3', description='franka arm model, only used when arm_family=franka (fr3, fer, fp3, ...)'),
    DeclareLaunchArgument('arm_hand',default_value='true',choices=['true','false'], description='mount the franka hand gripper, only used when arm_family=franka')
]

def generate_launch_description():
    path_to_urdf = get_package_share_path('magni_description') / 'urdf' / 'magni.urdf.xacro'
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output="both",
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', str(path_to_urdf), ' ',
                            ' use_mock_hardware:=', LaunchConfiguration('use_mock_hardware'),
                            ' tower_installed:=', LaunchConfiguration('tower_installed'),
                            ' shell_installed:=', LaunchConfiguration('shell_installed'),
                            ' sonars_installed:=', LaunchConfiguration('sonars_installed'),
                            ' lidar_extrinsics_file:=', LaunchConfiguration('lidar_extrinsics_file'),
                            ' camera_extrinsics_file:=', LaunchConfiguration('camera_extrinsics_file'),
                            ' arm_installed:=', LaunchConfiguration('arm_installed'),
                            ' arm_family:=', LaunchConfiguration('arm_family'),
                            ' robot_type:=', LaunchConfiguration('robot_type'),
                            ' arm_hand:=', LaunchConfiguration('arm_hand')
                            ]),
                value_type=str
            ),
            'use_sim_time': LaunchConfiguration('use_mock_hardware')
        }])
    
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'use_sim_time': LaunchConfiguration('use_mock_hardware')}]
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher)
    ld.add_action(joint_state_publisher)
    return ld