#!/bin/zsh

# Script to check camera topics and help debug camera feeds
echo "📷 Checking Camera Topics and Status..."
echo ""

# Change to workspace directory
cd /home/kiti/kawada

# Source the workspace (ignore zsh completion warnings)
source install/setup.zsh 2>/dev/null || source install/setup.bash

echo "Available camera-related topics:"
ros2 topic list 2>/dev/null | grep -E "(camera|image|depth)" || echo "No camera topics found"

echo ""
echo "Camera topic details:"
echo "Head Camera:"
ros2 topic info /head_camera/image_raw 2>/dev/null || echo "  ❌ /head_camera/image_raw not available"

echo "RGBD Camera Color:"
ros2 topic info /rgbd_camera/image_raw 2>/dev/null || echo "  ❌ /rgbd_camera/image_raw not available"

echo "RGBD Camera Depth:"
ros2 topic info /rgbd_camera/depth/image_raw 2>/dev/null || echo "  ❌ /rgbd_camera/depth/image_raw not available"

echo "RGBD Point Cloud:"
ros2 topic info /rgbd_camera/points 2>/dev/null || echo "  ❌ /rgbd_camera/points not available"

echo ""
echo "🎯 To see camera feeds in RViz:"
echo "1. Launch the robot: ./launch_robot.sh"
echo "2. In RViz, check the 'Displays' panel"
echo "3. Look for these displays:"
echo "   - Head Camera"
echo "   - RGBD Camera - Color" 
echo "   - RGBD Camera - Depth"
echo "   - RGBD Point Cloud"