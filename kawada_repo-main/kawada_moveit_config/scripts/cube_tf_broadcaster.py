#!/usr/bin/env python3
"""
Static TF broadcaster for the red cube.
Publishes the transform from world to red_cube frame.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
import tf2_ros


class CubeTFBroadcaster(Node):
    def __init__(self):
        super().__init__('cube_tf_broadcaster')
        
        self.tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        
        # Publish static transform for red cube
        # Gazebo coordinates: cube at (0.35, -0.2, 0.85), robot base at (0, 0, 0.8)
        # MoveIt world frame: robot base at (0, 0, 0), so cube is at (0.35, -0.2, 0.05)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'world'
        transform.child_frame_id = 'red_cube'
        
        transform.transform.translation.x = 0.35
        transform.transform.translation.y = -0.2
        transform.transform.translation.z = 0.05  # Relative to robot base (0.85 - 0.8)
        
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(transform)
        self.get_logger().info("Published static TF: world -> red_cube")


def main():
    rclpy.init()
    node = CubeTFBroadcaster()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()




