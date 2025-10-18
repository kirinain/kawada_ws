#!/usr/bin/env python3
"""
Direct Speed Test - Bypass MoveIt planning, send trajectories directly to controller
Tests actual 100 rad/s execution speed
"""

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import time

class DirectSpeedTest(Node):
    def __init__(self):
        super().__init__('direct_speed_test')
        self.publisher = self.create_publisher(
            JointTrajectory,
            '/right_arm_controller/joint_trajectory',
            10
        )
        time.sleep(1)  # Let publisher initialize
        self.get_logger().info("Direct trajectory publisher ready!")

    def move_to_joints(self, joint_values, duration_sec=1.0, position_name="Position"):
        """Send joint trajectory directly to controller
        
        Args:
            joint_values: List of 6 joint angles [J0, J1, J2, J3, J4, J5]
            duration_sec: Time to reach position (lower = faster)
            position_name: Description for logging
        """
        msg = JointTrajectory()
        msg.joint_names = [
            "RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2",
            "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5"
        ]
        
        point = JointTrajectoryPoint()
        point.positions = [float(v) for v in joint_values]
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=int((duration_sec % 1) * 1e9))
        
        msg.points.append(point)
        
        self.get_logger().info(f"Sending {position_name} in {duration_sec:.2f}s: {[f'{v:.2f}' for v in joint_values]}")
        start = time.time()
        self.publisher.publish(msg)
        time.sleep(duration_sec + 0.5)
        elapsed = time.time() - start
        self.get_logger().info(f"{position_name} completed in {elapsed:.2f}s")
        return True

def main(args=None):
    rclpy.init(args=args)
    tester = DirectSpeedTest()

    # Progressive speed test: Same movement, decreasing time
    positions = [
        ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5, "Home"),
        ([0.5, -0.3, 0.4, 0.3, -0.5, 0.8], 1.0, "Position 1 (1.0s)"),
        ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0, "Back Home (1.0s)"),
        ([0.5, -0.3, 0.4, 0.3, -0.5, 0.8], 0.5, "Position 1 (0.5s) - FASTER"),
        ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.5, "Back Home (0.5s) - FASTER"),
        ([0.5, -0.3, 0.4, 0.3, -0.5, 0.8], 0.2, "Position 1 (0.2s) - ULTRA FAST!"),
        ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.2, "Back Home (0.2s) - ULTRA FAST!"),
        ([0.5, -0.3, 0.4, 0.3, -0.5, 0.8], 0.1, "Position 1 (0.1s) - BLAZING!"),
        ([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], 0.1, "Back Home (0.1s) - BLAZING!"),
    ]

    tester.get_logger().info("DIRECT SPEED TEST - Testing controller execution speed")

    for i, (joints, duration, name) in enumerate(positions, 1):
        tester.get_logger().info(f"[{i}/{len(positions)}] {name}")
        tester.move_to_joints(joints, duration, name)
        tester.get_logger().info("")

    tester.get_logger().info("SPEED TEST COMPLETE!")

    tester.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

