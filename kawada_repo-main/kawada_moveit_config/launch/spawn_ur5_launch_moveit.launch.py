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
    srdf_file = os.path.join(moveit_config_pkg, 'config', 'NEXTAGE.srdf')
    kinematics_file = os.path.join(moveit_config_pkg, 'config', 'kinematics.yaml')
    ompl_config_file = os.path.join(moveit_config_pkg, 'config', 'ompl_planning.yaml')
    controllers_file = os.path.join(moveit_config_pkg, 'config', 'moveit_controllers.yaml')
    ros_controllers_file = os.path.join(moveit_config_pkg, 'config', 'ros_controllers.yaml')
    rviz_config_file = os.path.join(moveit_config_pkg, 'config', 'moveit.rviz')
    world_file = os.path.join(moveit_config_pkg, 'worlds', 'nextage_workspace.sdf')
    
    # Process URDF with xacro - critical for Gazebo
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}
    
    # Load other files
    robot_description_semantic = {'robot_description_semantic': load_file(srdf_file)}
    kinematics_config = load_yaml(kinematics_file)
    ompl_config = load_yaml(ompl_config_file)
    controllers_yaml = load_yaml(controllers_file)

    # MoveIt controller manager setup
    moveit_controllers = {
        'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
        'moveit_simple_controller_manager.controller_names': controllers_yaml.get('controller_names', ['joint_trajectory_controller']),
    }
    for ctrl_name in moveit_controllers['moveit_simple_controller_manager.controller_names']:
        ctrl_cfg = controllers_yaml.get(ctrl_name, {})
        prefix = f'moveit_simple_controller_manager.{ctrl_name}'
        moveit_controllers[f'{prefix}.type'] = ctrl_cfg.get('type', 'FollowJointTrajectory')
        moveit_controllers[f'{prefix}.action_ns'] = ctrl_cfg.get('action_ns', 'follow_joint_trajectory')
        moveit_controllers[f'{prefix}.default'] = ctrl_cfg.get('default', True)
        moveit_controllers[f'{prefix}.joints'] = ctrl_cfg.get('joints', [])

    trajectory_execution = {
        'moveit_manage_controllers': True,
        'trajectory_execution.allowed_execution_duration_scaling': 1.2,
        'trajectory_execution.allowed_goal_duration_margin': 0.5,
        'trajectory_execution.allowed_start_tolerance': 0.01
    }
    
    planning_scene_monitor_config = {
        'publish_planning_scene': True,
        'publish_geometry_updates': True,
        'publish_state_updates': True,
        'publish_transforms_updates': True,
        'publish_robot_description_semantic': True,
    }

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

    # 3. Spawn robot in Gazebo (on the robot table at elevated position)
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'nextage',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.8',  # Elevated to be on top of the table
        ],
        parameters=[{'use_sim_time': True}],
    )

    # 4. ros2_control_node (needed for controllers!)
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, ros_controllers_file],
        output="screen",
    )

    # 5. Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    right_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    left_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["left_arm_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    right_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["right_gripper_controller", "--controller-manager", "/controller_manager"],
        output="screen",
    )

    # 6. MoveGroup Node
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {
                'robot_description_kinematics': kinematics_config,
                'ompl': ompl_config,
                'planning_pipelines': ['ompl'],
                'use_sim_time': True,
            },
            moveit_controllers,
            trajectory_execution,
            planning_scene_monitor_config,
        ],
    )

    # 7. RViz Node
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=['-d', rviz_config_file],
        parameters=[
            robot_description,
            robot_description_semantic,
            {'robot_description_kinematics': kinematics_config, 'use_sim_time': True},
        ],
    )

    # 8. Camera Bridges - Bridge Gazebo camera topics to ROS 2
    head_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/head_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/head_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/head_camera/image', '/head_camera/image_raw'),
        ]
    )

    rgbd_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/rgbd_camera/image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/depth_image@sensor_msgs/msg/Image[ignition.msgs.Image',
            '/rgbd_camera/points@sensor_msgs/msg/PointCloud2[ignition.msgs.PointCloudPacked',
            '/rgbd_camera/camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo',
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
        remappings=[
            ('/rgbd_camera/image', '/rgbd_camera/image_raw'),
            ('/rgbd_camera/depth_image', '/rgbd_camera/depth/image_raw'),
        ]
    )

    # Timing logic
    delayed_spawn = TimerAction(period=5.0, actions=[spawn_entity])
    delay_joint_state_broadcaster = RegisterEventHandler(
        OnProcessStart(
            target_action=ros2_control_node,
            on_start=[TimerAction(period=2.0, actions=[joint_state_broadcaster_spawner])],
        )
    )
    delay_right_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[TimerAction(period=1.0, actions=[right_arm_controller_spawner])],
        )
    )

    delay_left_arm_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[TimerAction(period=1.0, actions=[left_arm_controller_spawner])],
        )
    )
    
    delay_gripper_controller = RegisterEventHandler(
        OnProcessStart(
            target_action=joint_state_broadcaster_spawner,
            on_start=[TimerAction(period=1.0, actions=[right_gripper_controller_spawner])],
        )
    )
    
    delayed_moveit = TimerAction(period=10.0, actions=[move_group_node, rviz_node])

    return LaunchDescription([
        gz_sim,
        robot_state_publisher,
        ros2_control_node,
        delayed_spawn,
        delay_joint_state_broadcaster,
        delay_right_arm_controller,
        delay_left_arm_controller,
        delay_gripper_controller,
        delayed_moveit,
        head_camera_bridge,
        rgbd_camera_bridge,
    ])
