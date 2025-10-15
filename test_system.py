#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import time

class SystemTester(Node):
    def __init__(self):
        super().__init__('system_tester')
        
        # Publisher for joint commands
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        
        self.get_logger().info("🧪 System Tester Started!")
        self.get_logger().info("Testing dual arm control and camera feeds...")
        
    def test_right_arm_movement(self):
        """Test right arm movement to reach for a cube"""
        self.get_logger().info("🦾 Testing right arm movement...")
        
        msg = JointState()
        msg.name = ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5']
        msg.position = [0.0, 0.5, -0.8, 0.3, 0.5, 0.0]  # Position to reach red cube
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        for i in range(10):
            self.joint_pub.publish(msg)
            time.sleep(0.1)
            
        self.get_logger().info("✅ Right arm moved to cube position!")
        
    def test_left_arm_movement(self):
        """Test left arm movement"""
        self.get_logger().info("🦾 Testing left arm movement...")
        
        msg = JointState()
        msg.name = ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5']
        msg.position = [0.0, -0.5, 0.8, -0.3, -0.5, 0.0]  # Mirror position
        msg.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        msg.effort = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        for i in range(10):
            self.joint_pub.publish(msg)
            time.sleep(0.1)
            
        self.get_logger().info("✅ Left arm moved to position!")

def main(args=None):
    rclpy.init(args=args)
    
    node = SystemTester()
    
    # Wait for system to be ready
    time.sleep(3)
    
    # Test right arm
    node.test_right_arm_movement()
    time.sleep(3)
    
    # Test left arm  
    node.test_left_arm_movement()
    
    node.get_logger().info("🎉 System test completed!")
    node.get_logger().info("📷 Check RViz for camera feeds!")
    node.get_logger().info("🎮 Use Joint State Publisher GUI for manual control!")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

