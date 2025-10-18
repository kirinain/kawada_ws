#!/usr/bin/env python3
"""Gripper Diagnostics: Monitor gripper state, forces, and contacts"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ContactsState
import time

class GripperDiagnostics(Node):
    def __init__(self):
        super().__init__('gripper_diagnostics')
        
        # Subscribe to joint states for gripper position/effort
        self.create_subscription(JointState, '/joint_states', self.joint_callback, 10)
        
        # Subscribe to contact sensor (we'll need to add this to the gripper)
        self.create_subscription(ContactsState, '/gripper_contacts', self.contact_callback, 10)
        
        self.gripper_state = {}
        self.contact_info = []
        
        self.get_logger().info("Gripper Diagnostics Started")
        self.get_logger().info("=" * 60)
        
        # Create timer for periodic status updates
        self.create_timer(1.0, self.print_status)
        
    def joint_callback(self, msg):
        """Track gripper joint state"""
        for i, name in enumerate(msg.name):
            if 'gripper' in name:
                self.gripper_state[name] = {
                    'position': msg.position[i] if i < len(msg.position) else 0.0,
                    'velocity': msg.velocity[i] if i < len(msg.velocity) else 0.0,
                    'effort': msg.effort[i] if i < len(msg.effort) else 0.0
                }
    
    def contact_callback(self, msg):
        """Track contact information"""
        self.contact_info = []
        for state in msg.states:
            contact = {
                'collision1': state.collision1_name,
                'collision2': state.collision2_name,
                'total_wrench': state.total_wrench,
                'contact_count': len(state.contact_positions)
            }
            self.contact_info.append(contact)
    
    def print_status(self):
        """Print diagnostic status"""
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("GRIPPER STATUS:")
        self.get_logger().info("-" * 60)
        
        if not self.gripper_state:
            self.get_logger().warn("No gripper joint data received!")
            return
        
        for joint_name, data in self.gripper_state.items():
            pos_mm = data['position'] * 1000  # Convert to mm
            opening_cm = data['position'] * 2 * 100  # Total opening in cm
            effort = data['effort']
            
            self.get_logger().info(f"{joint_name}:")
            self.get_logger().info(f"  Position: {pos_mm:.1f} mm (Total opening: {opening_cm:.1f} cm)")
            self.get_logger().info(f"  Effort:   {effort:.2f} N")
            self.get_logger().info(f"  Velocity: {data['velocity']:.4f} m/s")
            
            # Check if gripper is applying force
            if abs(effort) > 1.0:
                self.get_logger().info(f"  ⚠️  FORCE DETECTED: {effort:.2f} N")
            
            # Check if gripper is moving
            if abs(data['velocity']) > 0.001:
                self.get_logger().info(f"  🔄 MOVING")
            else:
                self.get_logger().info(f"  ✓ STATIONARY")
        
        # Contact information
        if self.contact_info:
            self.get_logger().info("-" * 60)
            self.get_logger().info(f"CONTACTS: {len(self.contact_info)} detected")
            for i, contact in enumerate(self.contact_info):
                self.get_logger().info(f"  Contact {i+1}:")
                self.get_logger().info(f"    {contact['collision1']} <-> {contact['collision2']}")
                self.get_logger().info(f"    Force: {contact['total_wrench'].force}")
                self.get_logger().info(f"    Points: {contact['contact_count']}")
        else:
            self.get_logger().info("-" * 60)
            self.get_logger().info("CONTACTS: None (sensor may not be configured)")
        
        self.get_logger().info("=" * 60)


def main(args=None):
    rclpy.init(args=args)
    node = GripperDiagnostics()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()


