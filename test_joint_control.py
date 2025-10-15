#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class SimpleJointControl(Node):
    def __init__(self):
        super().__init__('simple_joint_control')
        
        # Publisher for joint commands
        self.joint_pub = self.create_publisher(
            JointState, '/joint_states', 10)
        
        self.get_logger().info("Simple joint control started!")
        
    def move_right_arm_to_cube(self):
        """Move right arm to cube position"""
        self.get_logger().info("Moving right arm to cube...")
        
        msg = JointState()
        msg.name = ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']
        msg.position = [0.0, 0.5, -0.8, 0.3, 0.5, 0.0]
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Publish multiple times to ensure it's received
        for i in range(10):
            self.joint_pub.publish(msg)
            time.sleep(0.1)
            
        self.get_logger().info("Right arm command sent!")
        
    def move_left_arm_to_cube(self):
        """Move left arm to cube position"""
        self.get_logger().info("Moving left arm to cube...")
        
        msg = JointState()
        msg.name = ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']
        msg.position = [0.0, -0.5, 0.8, -0.3, -0.5, 0.0]
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Publish multiple times to ensure it's received
        for i in range(10):
            self.joint_pub.publish(msg)
            time.sleep(0.1)
            
        self.get_logger().info("Left arm command sent!")

def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleJointControl()
    
    # Wait a bit
    time.sleep(2)
    
    # Move right arm
    node.move_right_arm_to_cube()
    time.sleep(3)
    
    # Move left arm
    node.move_left_arm_to_cube()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
