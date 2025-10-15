from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder
import os


def generate_launch_description():
    # Declare arguments
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )

    # Build MoveIt configuration
    moveit_config = (
        MoveItConfigsBuilder("NEXTAGE", package_name="kawada_moveit_config")
        .robot_description(file_path="config/NEXTAGE.urdf.xacro")
        .robot_description_semantic(file_path="config/NEXTAGE.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )

    # Get package path
    pkg_share = FindPackageShare("kawada_moveit_config").find("kawada_moveit_config")
    rviz_config_file = PathJoinSubstitution([pkg_share, "config", LaunchConfiguration("rviz_config")])

    # Static TF (world to base_link)
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "CHEST_JOINT0_Link"],
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # Joint State Publisher (for manual joint control)
    joint_state_publisher = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "publish_robot_description_semantic": True,
                "moveit_controller_manager": "moveit_simple_controller_manager/MoveItSimpleControllerManager",
            },
        ],
    )

    # RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
    )

    # Delay move_group after robot_state_publisher starts
    delay_move_group = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=robot_state_publisher,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[move_group_node],
                ),
            ],
        )
    )

    # Delay rviz after move_group starts
    delay_rviz = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=move_group_node,
            on_start=[
                TimerAction(
                    period=2.0,
                    actions=[rviz_node],
                ),
            ],
        )
    )

    return LaunchDescription([
        rviz_config_arg,
        static_tf,
        robot_state_publisher,
        joint_state_publisher,
        delay_move_group,
        delay_rviz,
    ])



