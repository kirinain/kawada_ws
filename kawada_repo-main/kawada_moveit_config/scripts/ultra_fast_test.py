#!/usr/bin/env python3
"""
Ultra-Fast Joint Movement Test
Tests the 100 rad/s velocity limits by commanding joint space goals directly
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint
import time

class UltraFastJointMover(Node):
    def __init__(self):
        super().__init__('ultra_fast_joint_mover')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self.get_logger().info("Waiting for MoveGroup action server...")
        self._action_client.wait_for_server()
        self.get_logger().info("Connected to MoveGroup!")

    def move_to_joint_positions(self, joint_values, position_name="Position"):
        """Move arm to specific joint positions at MAX SPEED
        
        Args:
            joint_values: List of 6 joint angles in radians [J0, J1, J2, J3, J4, J5]
            position_name: Description for logging
        """
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "right_arm"
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 2.0
        goal_msg.request.max_velocity_scaling_factor = 1.0  # 100% of 100 rad/s = 100 rad/s!
        goal_msg.request.max_acceleration_scaling_factor = 1.0  # 100% of 200 rad/s²
        
        # Create joint constraints
        constraints = Constraints()
        joint_names = [
            "RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2",
            "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5"
        ]
        
        for joint_name, joint_value in zip(joint_names, joint_values):
            jc = JointConstraint()
            jc.joint_name = joint_name
            jc.position = float(joint_value)
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        
        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options.plan_only = False
        goal_msg.planning_options.planning_scene_diff.is_diff = True
        goal_msg.planning_options.planning_scene_diff.robot_state.is_diff = True

        self.get_logger().info(f"Moving to {position_name}: {[f'{v:.2f}' for v in joint_values]}")
        start_time = time.time()
        
        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future)
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error(f"{position_name} REJECTED!")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        
        result = result_future.result().result
        elapsed = time.time() - start_time

        if result.error_code.val == 1:
            self.get_logger().info(f"{position_name} reached in {elapsed:.2f}s")
            return True
        else:
            self.get_logger().error(f"{position_name} FAILED (error code: {result.error_code.val})")
            return False

def main(args=None):
    rclpy.init(args=args)
    mover = UltraFastJointMover()

    # Define 5 joint space positions for ultra-fast motion (safe, tested values)
    positions = [
        ([0.3, -0.2, 0.3, 0.2, 0.3, 0.0], "Position 1 - Right Reach"),
        ([-0.3, 0.3, -0.2, 0.3, -0.3, 0.5], "Position 2 - Left Reach"),
        ([0.0, 0.4, 0.2, 0.4, 0.0, -0.5], "Position 3 - Up High"),
        ([0.5, -0.3, 0.5, -0.2, 0.5, 1.0], "Position 4 - Wide Right"),
        ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "Position 5 - Home"),
    ]

    mover.get_logger().info("ULTRA-FAST JOINT MOVEMENT TEST")

    start_total = time.time()
    success_count = 0

    for i, (joint_values, name) in enumerate(positions, 1):
        mover.get_logger().info(f"[{i}/{len(positions)}] {name}")
        if mover.move_to_joint_positions(joint_values, name):
            success_count += 1
        mover.get_logger().info("")

    total_time = time.time() - start_total
    mover.get_logger().info(f"SEQUENCE COMPLETE: {success_count}/{len(positions)} positions reached")
    mover.get_logger().info(f"Total time: {total_time:.2f}s")

    mover.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

