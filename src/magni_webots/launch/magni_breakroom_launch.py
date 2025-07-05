from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os

ARGUMENTS = [
    DeclareLaunchArgument('use_sim_time',default_value='false',choices=['true','false'], description='sim time'),
    DeclareLaunchArgument('tower_installed',default_value='false',choices=['true','false'], description='tower'),
    DeclareLaunchArgument('shell_installed',default_value='false',choices=['true','false'], description='shell'),
    DeclareLaunchArgument('sonars_installed',default_value='false',choices=['true','false'], description='sonars'),
    DeclareLaunchArgument('camera_extrinsics_file',default_value='extrinsics/camera_extrinsics_forward.yaml', description='path to camera extrinsics file'),
    DeclareLaunchArgument('lidar_extrinsics_file',default_value='extrinsics/lidar_extrinsics_top_plate_center.yaml', description='path to lidar extrinsics file')
]

def generate_launch_description():
    magni_description_pkg = get_package_share_directory('magni_description')
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(magni_description_pkg, 'launch', 'description_launch.py')),
        launch_arguments={
            
        }
    )
    
    
    ld = LaunchDescription()
    return ld