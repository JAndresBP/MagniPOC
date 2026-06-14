import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition
from launch.substitutions import TextSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description() -> LaunchDescription:
	# Launch arguments
	enable_lidar = LaunchConfiguration('enable_lidar')
	teleop_arg = LaunchConfiguration('teleop')

	arguments = [
		DeclareLaunchArgument('enable_lidar', default_value='true',description='Whether to launch LIDAR driver'),
		DeclareLaunchArgument('teleop', default_value='true', description='Launch teleop')
	]

	# Include the motors launch from ubiquity_motor_ros2
	ubiq_share = get_package_share_directory('ubiquity_motor_ros2')
	motors_launch_path = os.path.join(ubiq_share, 'launch', 'motors.launch.py')
	include_motors = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(motors_launch_path)
	)

	# Include ydlidar launch if enabled
	ydlidar_share = get_package_share_directory('ydlidar_ros2_driver')
	ydlidar_launch_path = os.path.join(ydlidar_share, 'launch', 'ydlidar_launch.py')
	# Use local copy of ydlidar params in magni_bringup/config/ydlidar.yaml
	local_params = os.path.join(get_package_share_directory('magni_bringup'), 'config', 'ydlidar.yaml')
	include_ydlidar = IncludeLaunchDescription(
		PythonLaunchDescriptionSource(ydlidar_launch_path),
		launch_arguments={
			'params_file': TextSubstitution(text=local_params),
		}.items(),
		condition=IfCondition(enable_lidar)
	)
    
	teleop_share = get_package_share_directory('magni_teleop')
	teleop_launch = os.path.join(teleop_share, 'launch', 'joy_teleop.launch.py')
	include_teleop = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(teleop_launch),
        condition=IfCondition(teleop_arg)
    )

	ld = LaunchDescription(arguments)
	ld.add_action(include_motors)
	ld.add_action(include_ydlidar)
	ld.add_action(include_teleop)

	return ld

