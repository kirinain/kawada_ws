#!/usr/bin/env python3
"""Pick Red Cube using MoveIt Action Interface"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped, Pose
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from shape_msgs.msg import SolidPrimitive
from builtin_interfaces.msg import Duration
import time
import math

CUBE_X = 0.35
CUBE_Y = -0.2
CUBE_Z = 0.05
PRE_GRASP_OFFSET_Z = 0.15
APPROACH_OFFSET_Z = 0.05
LIFT_OFFSET_Z = 0.20


class PickCubeCartesian(Node):
    def __init__(self):
        super().__init__('pick_cube_cartesian')

        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self.gripper_pub = self.create_publisher(JointTrajectory, '/right_gripper_controller/joint_trajectory', 10)
        self.current_joint_state = None
        self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)

        self.get_logger().info("Waiting for MoveGroup server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Connected to MoveGroup!")

        while self.current_joint_state is None:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.get_logger().info("Received joint states.")

    def joint_cb(self, msg):
        self.current_joint_state = msg
    
    def control_gripper(self, position, duration_sec=1.0):
        traj = JointTrajectory()
        traj.joint_names = ['right_gripper_left_finger_joint']
        point = JointTrajectoryPoint()
        point.positions = [position]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        traj.points.append(point)
        self.gripper_pub.publish(traj)
        self.get_logger().info(f"Gripper: {position}")
        time.sleep(duration_sec + 0.5)

    def make_pose(self, x, y, z, qx=0.0, qy=0.0, qz=0.0, qw=1.0):
        pose = PoseStamped()
        pose.header.frame_id = "world"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = float(z)
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)
        return pose

    def move_to_pose(self, pose, desc="motion"):
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "right_arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.3
        goal_msg.request.max_acceleration_scaling_factor = 0.3
        goal_msg.request.start_state.joint_state = self.current_joint_state
        
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "world"
        pos_constraint.link_name = "RARM_JOINT5_Link"
        pos_constraint.constraint_region.primitives.append(SolidPrimitive())
        pos_constraint.constraint_region.primitives[0].type = SolidPrimitive.SPHERE
        pos_constraint.constraint_region.primitives[0].dimensions = [0.001]
        pos_constraint.constraint_region.primitive_poses.append(Pose())
        pos_constraint.constraint_region.primitive_poses[0].position.x = pose.pose.position.x
        pos_constraint.constraint_region.primitive_poses[0].position.y = pose.pose.position.y
        pos_constraint.constraint_region.primitive_poses[0].position.z = pose.pose.position.z
        pos_constraint.weight = 1.0
        
        orient_constraint = OrientationConstraint()
        orient_constraint.header.frame_id = "world"
        orient_constraint.link_name = "RARM_JOINT5_Link"
        orient_constraint.orientation = pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.5
        orient_constraint.absolute_y_axis_tolerance = 0.5
        orient_constraint.absolute_z_axis_tolerance = 0.5
        orient_constraint.weight = 1.0
        
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        constraints.orientation_constraints.append(orient_constraint)
        goal_msg.request.goal_constraints.append(constraints)

        goal_msg.request.workspace_parameters.header.frame_id = "world"
        goal_msg.request.workspace_parameters.min_corner.x = -2.0
        goal_msg.request.workspace_parameters.max_corner.x = 2.0
        goal_msg.request.workspace_parameters.min_corner.y = -2.0
        goal_msg.request.workspace_parameters.max_corner.y = 2.0
        goal_msg.request.workspace_parameters.min_corner.z = -0.5
        goal_msg.request.workspace_parameters.max_corner.z = 2.0

        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.get_logger().info(f"Sending {desc} goal...")
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"{desc} rejected!")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        if result.result.error_code.val == 1:
            self.get_logger().info(f"✓ {desc} SUCCESS")
            return True
        else:
            self.get_logger().error(f"✗ {desc} FAILED ({result.result.error_code.val})")
            return False

    def execute_pick(self):
        qx = math.sqrt(2)/2
        qy = 0.0
        qz = 0.0
        qw = math.sqrt(2)/2
        
        self.get_logger().info("Opening gripper...")
        self.control_gripper(0.07, 1.0)
        
        self.get_logger().info(f"Pre-grasp: ({CUBE_X}, {CUBE_Y}, {CUBE_Z + PRE_GRASP_OFFSET_Z})")
        pre_pose = self.make_pose(CUBE_X, CUBE_Y, CUBE_Z + PRE_GRASP_OFFSET_Z, qx, qy, qz, qw)
        if not self.move_to_pose(pre_pose, "Pre-grasp"):
            return False
        time.sleep(1.0)

        self.get_logger().info(f"Approach: ({CUBE_X}, {CUBE_Y}, {CUBE_Z + APPROACH_OFFSET_Z})")
        approach_pose = self.make_pose(CUBE_X, CUBE_Y, CUBE_Z + APPROACH_OFFSET_Z, qx, qy, qz, qw)
        if not self.move_to_pose(approach_pose, "Approach"):
            return False
        time.sleep(1.0)

        self.get_logger().info("Closing gripper...")
        self.control_gripper(0.045, 1.5)
        time.sleep(0.5)
        self.control_gripper(0.04, 1.5)

        self.get_logger().info(f"Lift: ({CUBE_X}, {CUBE_Y}, {CUBE_Z + LIFT_OFFSET_Z})")
        lift_pose = self.make_pose(CUBE_X, CUBE_Y, CUBE_Z + LIFT_OFFSET_Z, qx, qy, qz, qw)
        if not self.move_to_pose(lift_pose, "Lift"):
            self.get_logger().warn("Lift failed, but gripper may have cube")
        
        time.sleep(2.0)
        self.get_logger().info("✓ Pick sequence completed!")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PickCubeCartesian()
    node.execute_pick()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
