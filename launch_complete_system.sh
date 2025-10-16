#!/bin/zsh

# Complete system launch script for NEXTAGE robot with cameras and dual arm control
echo " Launching Complete NEXTAGE System..."
echo " Including: Gazebo + MoveIt + RViz + Cameras + Dual Arm Controllers"
echo ""

# Change to workspace directory
cd /home/kiti/kawada

# Source the workspace (ignore zsh completion warnings)
echo " Sourcing workspace..."
source install/setup.zsh 2>/dev/null || source install/setup.bash

echo ""
echo " Starting complete system..."
echo "   - Gazebo simulation with robot and cubes"
echo "   - MoveIt motion planning"
echo "   - RViz visualization with camera feeds"
echo "   - Dual arm controllers"
echo ""

# Launch the complete system
ros2 launch kawada_moveit_config spawn_ur5_launch_moveit.launch.py

echo ""
echo " System launched! You should see:"
echo "   - Gazebo window with robot and colored cubes"
echo "   - RViz window with MoveIt interface"
echo "   - Camera feeds in RViz displays panel"
echo "   - Joint State Publisher GUI for manual control"
echo ""
echo " Camera feeds available in RViz:"
echo "   - Head Camera"
echo "   - RGBD Camera - Color"
echo "   - RGBD Camera - Depth" 
echo "   - RGBD Point Cloud"
echo ""
echo " Control options:"
echo "   - Use Joint State Publisher GUI sliders"
echo "   - Use RViz interactive markers"
echo "   - Use MoveIt planning groups: right_arm, left_arm, both_arms"
echo ""
echo "Press Ctrl+C to stop all systems"

