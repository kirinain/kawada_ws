#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time

class CubeGrabber(Node):
    def __init__(self):
        super().__init__('cube_grabber')
        
        # Publishers for both arms
        self.right_joint_pub = self.create_publisher(
            JointState, '/right_arm/joint_commands', 10)
        self.left_joint_pub = self.create_publisher(
            JointState, '/left_arm/joint_commands', 10)
        
        self.right_cartesian_pub = self.create_publisher(
            Twist, '/right_arm/cartesian_command', 10)
        self.left_cartesian_pub = self.create_publisher(
            Twist, '/left_arm/cartesian_command', 10)
        
        self.get_logger().info("Cube grabber node started!")
        self.get_logger().info("Use the following methods to grab cubes:")
        self.get_logger().info("1. Joint control: self.move_to_cube_joints()")
        self.get_logger().info("2. Cartesian control: self.move_to_cube_cartesian()")
        
    def move_to_cube_joints(self, arm='right'):
        """Move arm to cube using joint commands"""
        self.get_logger().info(f"Moving {arm} arm to cube using joint commands...")
        
        msg = JointState()
        
        if arm == 'right':
            msg.name = ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']
            # Joint positions to reach the cube (red cube at x=0.8, y=-0.4, z=0.875)
            msg.position = [0.0, 0.5, -0.8, 0.3, 0.5, 0.0]  # Adjusted for right arm
            publisher = self.right_joint_pub
        else:
            msg.name = ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']
            # Joint positions to reach the cube
            msg.position = [0.0, -0.5, 0.8, -0.3, -0.5, 0.0]  # Adjusted for left arm
            publisher = self.left_joint_pub
            
        msg.velocity = [0.0] * 6
        msg.effort = [0.0] * 6
        
        # Publish the command
        publisher.publish(msg)
        self.get_logger().info(f"Published joint command for {arm} arm: {msg.position}")
        
    def move_to_cube_cartesian(self, arm='right'):
        """Move arm to cube using cartesian commands"""
        self.get_logger().info(f"Moving {arm} arm to cube using cartesian commands...")
        
        msg = Twist()
        
        if arm == 'right':
            # Move towards the cube (positive X direction)
            msg.linear.x = 0.3   # Move forward
            msg.linear.y = -0.2  # Move left (towards cube)
            msg.linear.z = 0.1   # Move up slightly
            publisher = self.right_cartesian_pub
        else:
            # Move towards the cube (positive X direction)
            msg.linear.x = 0.3   # Move forward
            msg.linear.y = 0.2   # Move right (towards cube)
            msg.linear.z = 0.1   # Move up slightly
            publisher = self.left_cartesian_pub
            
        # No rotation for now
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        
        # Publish the command
        publisher.publish(msg)
        self.get_logger().info(f"Published cartesian command for {arm} arm: "
                             f"lin=({msg.linear.x}, {msg.linear.y}, {msg.linear.z})")
        
    def grab_sequence(self, arm='right'):
        """Complete grab sequence"""
        self.get_logger().info(f"Starting grab sequence for {arm} arm...")
        
        # Step 1: Move to cube position
        self.get_logger().info("Step 1: Moving to cube position...")
        self.move_to_cube_joints(arm)
        time.sleep(2)
        
        # Step 2: Fine adjustment with cartesian control
        self.get_logger().info("Step 2: Fine adjustment...")
        self.move_to_cube_cartesian(arm)
        time.sleep(1)
        
        # Step 3: "Grab" motion (close gripper simulation)
        self.get_logger().info("Step 3: Grabbing motion...")
        if arm == 'right':
            msg = JointState()
            msg.name = ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']
            msg.position = [0.0, 0.6, -0.9, 0.4, 0.6, 0.0]  # Slightly closer
            self.right_joint_pub.publish(msg)
        else:
            msg = JointState()
            msg.name = ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']
            msg.position = [0.0, -0.6, 0.9, -0.4, -0.6, 0.0]  # Slightly closer
            self.left_joint_pub.publish(msg)
            
        self.get_logger().info("Grab sequence completed!")
        
    def demo_both_methods(self):
        """Demonstrate both control methods"""
        self.get_logger().info("=== DEMO: Both Control Methods ===")
        
        # Method 1: Joint control
        self.get_logger().info("\n--- Method 1: Joint Control ---")
        self.move_to_cube_joints('right')
        time.sleep(3)
        
        # Method 2: Cartesian control
        self.get_logger().info("\n--- Method 2: Cartesian Control ---")
        self.move_to_cube_cartesian('left')
        time.sleep(3)
        
        self.get_logger().info("\nDemo completed!")

def main(args=None):
    rclpy.init(args=args)
    
    node = CubeGrabber()
    
    # Wait a bit for the system to be ready
    time.sleep(2)
    
    # Run the demo
    node.demo_both_methods()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
