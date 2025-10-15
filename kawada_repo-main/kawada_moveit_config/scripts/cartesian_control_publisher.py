#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
import math
import time

class CartesianControlPublisher(Node):
    def __init__(self):
        super().__init__('cartesian_control_publisher')
        
        # Determine which arm based on node name
        node_name = self.get_name()
        if 'right' in node_name:
            self.arm_prefix = 'right_arm'
            self.joint_prefix = 'RARM'
        elif 'left' in node_name:
            self.arm_prefix = 'left_arm'
            self.joint_prefix = 'LARM'
        else:
            self.get_logger().error("Cannot determine arm type from node name")
            return
            
        # Publisher for cartesian commands
        self.cartesian_pub = self.create_publisher(
            Twist,
            f'/{self.arm_prefix}/cartesian_command',
            10
        )
        
        # Publisher for joint commands (alternative control method)
        self.joint_pub = self.create_publisher(
            JointState,
            f'/{self.arm_prefix}/joint_commands',
            10
        )
        
        # Timer for periodic commands
        self.timer = self.create_timer(2.0, self.publish_cartesian_command)
        self.joint_timer = self.create_timer(3.0, self.publish_joint_command)
        
        self.counter = 0
        self.get_logger().info(f"Cartesian control publisher started for {self.arm_prefix}")
        
    def publish_cartesian_command(self):
        """Publish cartesian velocity commands for end-effector control"""
        msg = Twist()
        
        # Simple circular motion pattern
        self.counter += 1
        angle = self.counter * 0.1
        
        # Linear velocities (in base frame)
        msg.linear.x = 0.02 * math.sin(angle)  # Forward/backward
        msg.linear.y = 0.02 * math.cos(angle)  # Left/right
        msg.linear.z = 0.01 * math.sin(angle * 0.5)  # Up/down
        
        # Angular velocities
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.1 * math.sin(angle * 0.3)  # Rotation around Z
        
        self.cartesian_pub.publish(msg)
        self.get_logger().info(f"Published cartesian command for {self.arm_prefix}: "
                             f"lin=({msg.linear.x:.3f}, {msg.linear.y:.3f}, {msg.linear.z:.3f}), "
                             f"ang=({msg.angular.x:.3f}, {msg.angular.y:.3f}, {msg.angular.z:.3f})")
    
    def publish_joint_command(self):
        """Publish joint position commands as alternative control method"""
        msg = JointState()
        
        # Joint names for the arm
        joint_names = [f'{self.joint_prefix}_JOINT{i}' for i in range(6)]
        msg.name = joint_names
        
        # Simple sinusoidal joint movements
        self.counter += 1
        angle = self.counter * 0.05
        
        positions = []
        for i in range(6):
            # Different frequencies for each joint
            pos = 0.2 * math.sin(angle + i * 0.5)
            positions.append(pos)
        
        msg.position = positions
        msg.velocity = [0.0] * 6  # No velocity commands
        msg.effort = [0.0] * 6    # No effort commands
        
        self.joint_pub.publish(msg)
        self.get_logger().info(f"Published joint command for {self.arm_prefix}: "
                             f"positions={[f'{p:.3f}' for p in positions]}")

def main(args=None):
    rclpy.init(args=args)
    
    node = CartesianControlPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
