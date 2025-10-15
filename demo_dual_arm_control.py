#!/usr/bin/env python3
"""
Demo script showing dual-arm control capabilities.
Demonstrates both joint-level and Cartesian space control.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist
import time


class DualArmDemo(Node):
    def __init__(self):
        super().__init__('dual_arm_demo')
        
        # Publishers for joint control
        self.right_joint_pub = self.create_publisher(JointState, '/right_arm/joint_commands', 10)
        self.left_joint_pub = self.create_publisher(JointState, '/left_arm/joint_commands', 10)
        
        # Publishers for Cartesian control
        self.right_cart_pub = self.create_publisher(Twist, '/right_arm/cartesian_command', 10)
        self.left_cart_pub = self.create_publisher(Twist, '/left_arm/cartesian_command', 10)
        
        time.sleep(1)  # Let publishers initialize
        
    def demo_joint_control(self):
        """Demonstrate joint-level control of both arms"""
        print("\n" + "="*60)
        print("DEMO 1: Joint-Level Control")
        print("="*60)
        print("Moving both arms using direct joint commands...")
        
        # Home position
        print("\n1. Moving to HOME position...")
        right_cmd = JointState()
        right_cmd.name = ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 
                          'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']
        right_cmd.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        left_cmd = JointState()
        left_cmd.name = ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2',
                         'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']
        left_cmd.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        for _ in range(10):
            self.right_joint_pub.publish(right_cmd)
            self.left_joint_pub.publish(left_cmd)
            time.sleep(0.1)
        
        print("   ✅ Both arms at HOME")
        time.sleep(2)
        
        # Forward reach position
        print("\n2. Moving to REACH FORWARD position...")
        right_cmd.position = [0.0, 0.5, -0.8, 0.3, 0.0, 0.0]
        left_cmd.position = [0.0, 0.5, -0.8, 0.3, 0.0, 0.0]
        
        for _ in range(10):
            self.right_joint_pub.publish(right_cmd)
            self.left_joint_pub.publish(left_cmd)
            time.sleep(0.1)
        
        print("   ✅ Both arms reaching forward")
        time.sleep(2)
        
        # Spread arms
        print("\n3. Moving to SPREAD position...")
        right_cmd.position = [0.5, 0.3, -0.5, 0.2, 0.0, 0.0]
        left_cmd.position = [-0.5, 0.3, -0.5, 0.2, 0.0, 0.0]
        
        for _ in range(10):
            self.right_joint_pub.publish(right_cmd)
            self.left_joint_pub.publish(left_cmd)
            time.sleep(0.1)
        
        print("   ✅ Arms spread apart")
        time.sleep(2)
        
        # Back to home
        print("\n4. Returning to HOME...")
        right_cmd.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        left_cmd.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        for _ in range(10):
            self.right_joint_pub.publish(right_cmd)
            self.left_joint_pub.publish(left_cmd)
            time.sleep(0.1)
        
        print("   ✅ Back to HOME")
        print("\n✅ Joint-level control demonstration complete!")
        
    def demo_cartesian_control(self):
        """Demonstrate Cartesian space control"""
        print("\n" + "="*60)
        print("DEMO 2: Cartesian Space Control")
        print("="*60)
        print("Moving end-effectors in Cartesian space...")
        
        # Move forward
        print("\n1. Moving end-effectors FORWARD...")
        right_cmd = Twist()
        right_cmd.linear.x = 0.3
        right_cmd.linear.y = 0.0
        right_cmd.linear.z = 0.0
        
        left_cmd = Twist()
        left_cmd.linear.x = 0.3
        left_cmd.linear.y = 0.0
        left_cmd.linear.z = 0.0
        
        for _ in range(10):
            self.right_cart_pub.publish(right_cmd)
            self.left_cart_pub.publish(left_cmd)
            time.sleep(0.1)
        
        print("   ✅ Moved forward in Cartesian space")
        time.sleep(2)
        
        # Move sideways
        print("\n2. Moving end-effectors SIDEWAYS (opposite directions)...")
        right_cmd.linear.x = 0.0
        right_cmd.linear.y = 0.2
        right_cmd.linear.z = 0.0
        
        left_cmd.linear.x = 0.0
        left_cmd.linear.y = -0.2
        left_cmd.linear.z = 0.0
        
        for _ in range(10):
            self.right_cart_pub.publish(right_cmd)
            self.left_cart_pub.publish(left_cmd)
            time.sleep(0.1)
        
        print("   ✅ Moved sideways in opposite directions")
        time.sleep(2)
        
        # Move up
        print("\n3. Moving end-effectors UP...")
        right_cmd.linear.x = 0.0
        right_cmd.linear.y = 0.0
        right_cmd.linear.z = 0.15
        
        left_cmd.linear.x = 0.0
        left_cmd.linear.y = 0.0
        left_cmd.linear.z = 0.15
        
        for _ in range(10):
            self.right_cart_pub.publish(right_cmd)
            self.left_cart_pub.publish(left_cmd)
            time.sleep(0.1)
        
        print("   ✅ Moved up in Cartesian space")
        time.sleep(2)
        
        print("\n✅ Cartesian control demonstration complete!")
        
    def demo_coordinated_motion(self):
        """Demonstrate coordinated dual-arm motion"""
        print("\n" + "="*60)
        print("DEMO 3: Coordinated Dual-Arm Motion")
        print("="*60)
        print("Demonstrating symmetric and coordinated movements...")
        
        # Symmetric reaching
        print("\n1. Symmetric reach towards objects...")
        right_cmd = JointState()
        right_cmd.name = ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 
                          'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']
        right_cmd.position = [0.3, 0.6, -0.9, 0.4, 0.2, 0.0]
        
        left_cmd = JointState()
        left_cmd.name = ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2',
                         'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']
        left_cmd.position = [-0.3, 0.6, -0.9, 0.4, 0.2, 0.0]
        
        for _ in range(10):
            self.right_joint_pub.publish(right_cmd)
            self.left_joint_pub.publish(left_cmd)
            time.sleep(0.1)
        
        print("   ✅ Both arms reaching symmetrically")
        time.sleep(2)
        
        # Mirror movements
        print("\n2. Mirror movements...")
        right_cmd.position = [0.5, 0.4, -0.6, 0.3, 0.1, 0.0]
        left_cmd.position = [-0.5, 0.4, -0.6, 0.3, -0.1, 0.0]
        
        for _ in range(10):
            self.right_joint_pub.publish(right_cmd)
            self.left_joint_pub.publish(left_cmd)
            time.sleep(0.1)
        
        print("   ✅ Arms in mirrored configuration")
        time.sleep(2)
        
        # Return to home
        print("\n3. Returning both arms to HOME...")
        right_cmd.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        left_cmd.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        for _ in range(10):
            self.right_joint_pub.publish(right_cmd)
            self.left_joint_pub.publish(left_cmd)
            time.sleep(0.1)
        
        print("   ✅ Back to HOME position")
        print("\n✅ Coordinated motion demonstration complete!")
        
    def run_all_demos(self):
        """Run all demonstration sequences"""
        print("\n" + "="*70)
        print(" DUAL-ARM CONTROL DEMONSTRATION")
        print("="*70)
        print("\nThis demo will show:")
        print("  1. Joint-level control of both arms")
        print("  2. Cartesian space control")
        print("  3. Coordinated dual-arm movements")
        print("\nWatch the robot in Gazebo/RViz!")
        print("\nStarting in 3 seconds...")
        time.sleep(3)
        
        self.demo_joint_control()
        time.sleep(2)
        
        self.demo_cartesian_control()
        time.sleep(2)
        
        self.demo_coordinated_motion()
        
        print("\n" + "="*70)
        print(" DEMONSTRATION COMPLETE!")
        print("="*70)
        print("\n✨ All control modes demonstrated successfully!")
        print("\nYou can now:")
        print("  - Control arms via /right_arm/joint_commands and /left_arm/joint_commands")
        print("  - Use Cartesian control via /right_arm/cartesian_command and /left_arm/cartesian_command")
        print("  - Plan with MoveIt using RViz interface")
        print("\n")


def main():
    rclpy.init()
    
    demo = DualArmDemo()
    
    print("Waiting for system to be ready...")
    time.sleep(2)
    
    demo.run_all_demos()
    
    demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

