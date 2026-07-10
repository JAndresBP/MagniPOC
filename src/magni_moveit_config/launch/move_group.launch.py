import os
from importlib.machinery import SourceFileLoader

from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder

moveit_parameters = SourceFileLoader(
    "moveit_launch_common",
    os.path.join(os.path.dirname(__file__), "moveit_launch_common.py"),
).load_module().moveit_parameters


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument("use_sim_time", default_value="true")
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Start RViz with the MoveIt motion planning plugin",
    )
    # Adds a static world->base_footprint transform, needed because nothing else publishes a
    # fixed root for base_footprint when running MoveIt on a stationary arm without mapping/Nav2
    # also active. If composing with --mapping/--navigation (where base_footprint is already
    # rooted via odom), set this to false -- base_footprint can't have two parents in TF.
    publish_world_tf_arg = DeclareLaunchArgument(
        "publish_world_tf",
        default_value="true",
        description="Publish a static world->base_footprint transform (disable if mapping/navigation already root base_footprint via odom)",
    )

    use_sim_time = LaunchConfiguration("use_sim_time")

    magni_description_xacro_path = str(
        get_package_share_path("magni_description") / "urdf" / "magni.urdf.xacro"
    )

    # Mirrors magni_spawn.launch.py's get_robot_spawner() mappings exactly, so the planning
    # model never drifts from the simulated one. This MoveIt config is rebotarm-specific (SRDF
    # joint/group names assume the rebotarm, not franka).
    moveit_config = (
        MoveItConfigsBuilder("magni_rebotarm", package_name="magni_moveit_config")
        .robot_description(
            file_path=magni_description_xacro_path,
            mappings={
                "use_mock_hardware": "True",
                "arm_installed": "true",
                "arm_family": "rebotarm",
                "torso_installed": "true",
                "torso_left_accessory": "tray",
                "torso_head_accessory": "screen",
            },
        )
        .robot_description_semantic(file_path="config/magni_rebotarm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )
    moveit_params = moveit_parameters(moveit_config)

    # move_group only -- no ros2_control_node or controller spawners here: MAGNI's own
    # magni_spawn.launch.py chain already brings up arm_controller/gripper_controller, and
    # robot_state_publisher is already running via magni_description's own launch include.
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_params, {"use_sim_time": use_sim_time}],
    )

    rviz_config = str(get_package_share_path("magni_moveit_config") / "config" / "moveit.rviz")
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        parameters=[moveit_params, {"use_sim_time": use_sim_time}],
    )

    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_footprint"],
        condition=IfCondition(LaunchConfiguration("publish_world_tf")),
    )

    return LaunchDescription([
        use_sim_time_arg,
        use_rviz_arg,
        publish_world_tf_arg,
        move_group_node,
        rviz_node,
        static_tf_node,
    ])
