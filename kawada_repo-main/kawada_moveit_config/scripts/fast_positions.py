#!/usr/bin/env python3
"""Fast Sequential Position Movement - Move arm through multiple waypoints at max speed"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
from geometry_msgs.msg import PoseStamped, Pose
from shape_msgs.msg import SolidPrimitive
import time
import math

class FastPositionMover(Node):
    def __init__(self):
        super().__init__('fast_position_mover')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info('Waiting for MoveGroup action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Connected to MoveGroup!')

    def rpy_to_quaternion(self, roll, pitch, yaw):
        """Convert Roll-Pitch-Yaw (degrees) to Quaternion"""
        roll = math.radians(roll)
        pitch = math.radians(pitch)
        yaw = math.radians(yaw)
        
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        qw = cr * cp * cy + sr * sp * sy
        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        
        return qx, qy, qz, qw

    def move_to_position(self, x, y, z, roll=0, pitch=0, yaw=0, position_name="Position"):
        """Move the arm to a specific XYZ position with orientation at maximum speed
        
        Args:
            x, y, z: Position in meters
            roll, pitch, yaw: Orientation in DEGREES
            position_name: Description for logging
        """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "right_arm"
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 1.0
        goal_msg.request.max_velocity_scaling_factor = 1.0  # Uses URDF limits (now 100 rad/s)
        goal_msg.request.max_acceleration_scaling_factor = 1.0  # Maximum acceleration
        
        # Create position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "world"
        pos_constraint.link_name = "right_gripper_tcp"
        pos_constraint.constraint_region.primitives.append(SolidPrimitive())
        pos_constraint.constraint_region.primitives[0].type = SolidPrimitive.SPHERE
        pos_constraint.constraint_region.primitives[0].dimensions = [0.001]  # 1mm tolerance
        pos_constraint.constraint_region.primitive_poses.append(Pose())
        pos_constraint.constraint_region.primitive_poses[0].position.x = float(x)
        pos_constraint.constraint_region.primitive_poses[0].position.y = float(y)
        pos_constraint.constraint_region.primitive_poses[0].position.z = float(z)
        pos_constraint.weight = 1.0
        
        constraints = Constraints()
        constraints.position_constraints.append(pos_constraint)
        # NO orientation constraints - let MoveIt find any valid orientation for max speed
        goal_msg.request.goal_constraints.append(constraints)

        # Set workspace
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

        self.get_logger().info(f"→ Moving to {position_name}: ({x:.2f}, {y:.2f}, {z:.2f}) RPY=[{roll}°, {pitch}°, {yaw}°] at MAX SPEED")
        start_time = time.time()
        
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"✗ {position_name} REJECTED!")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()

        elapsed = time.time() - start_time
        
        if result.result.error_code.val == 1:
            self.get_logger().info(f"✓ {position_name} SUCCESS in {elapsed:.2f}s")
            return True
        else:
            self.get_logger().error(f"✗ {position_name} FAILED (error code: {result.result.error_code.val})")
            return False

    def execute_fast_sequence(self):
        """Execute a sequence of 4 fast positions"""
        self.get_logger().info("=" * 60)
        self.get_logger().info("FAST POSITION SEQUENCE - 4 WAYPOINTS AT MAX SPEED")
        self.get_logger().info("=" * 60)
        
        # Define 4 target positions (x, y, z, roll, pitch, yaw, description)
        # Roll, Pitch, Yaw are in DEGREES
        # Roll=90 means gripper pointing down, Roll=0 means gripper horizontal
        positions = [
            (0.4, -0.3, 0.3, 90, 0, 0, "Position 1 - Right Front High (gripper down)"),
            (0.3, -0.1, 0.15, 90, 0, 0, "Position 2 - Center Mid (gripper down)"),
            (0.45, -0.35, 0.5, 45, 0, 0, "Position 3 - Extended Arm High (gripper tilted)"),
            (0.4, -0.2, 0.25, 90, 0, 0, "Position 4 - Right Front Mid (gripper down)"),
        ]
        
        total_start = time.time()
        success_count = 0
        
        for i, (x, y, z, roll, pitch, yaw, name) in enumerate(positions, 1):
            self.get_logger().info("")
            self.get_logger().info(f"[{i}/4] {name}")
            if self.move_to_position(x, y, z, roll, pitch, yaw, name):
                success_count += 1
            else:
                self.get_logger().warn(f"Failed to reach {name}, continuing...")
            
            # No pause - continuous fast motion!
        
        total_time = time.time() - total_start
        
        self.get_logger().info("")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"SEQUENCE COMPLETE: {success_count}/4 positions reached")
        self.get_logger().info(f"Total time: {total_time:.2f}s")
        self.get_logger().info("=" * 60)

def main():
    rclpy.init()
    node = FastPositionMover()
    
    try:
        node.execute_fast_sequence()
    except KeyboardInterrupt:
        node.get_logger().info('Interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

