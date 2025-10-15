import os
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory
from launch.substitutions import PathJoinSubstitution


def load_file(file_path):
    try:
        with open(file_path, 'r') as file:
            return file.read()
    except EnvironmentError:
        return None


def load_yaml(file_path):
    try:
        with open(file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # Get package directories
    moveit_config_pkg = get_package_share_directory('kawada_moveit_config')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    
    # Paths
    urdf_file = os.path.join(moveit_config_pkg, 'config', 'NEXTAGE.urdf.xacro')
    world_file = os.path.join(moveit_config_pkg, 'worlds', 'nextage_workspace.sdf')
    
    # Process URDF with xacro
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 1. Launch Gazebo with custom world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r -v 4 {world_file}'}.items()
    )

    # 2. Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description, {'use_sim_time': True}],
    )

    # 3. Spawn robot in Gazebo
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'nextage',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.8',
        ],
        parameters=[{'use_sim_time': True}],
    )

    # Timing logic
    delayed_spawn = TimerAction(period=3.0, actions=[spawn_entity])

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        delayed_spawn,
    ])



