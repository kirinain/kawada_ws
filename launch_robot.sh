#!/bin/zsh

# Helper script to launch the robot with cameras in Gazebo and RViz
# Usage: ./launch_robot.sh

echo " Launching NEXTAGE Robot with Dual Arm Control and Cameras..."
echo ""

# Change to workspace directory
cd /home/kiti/kawada

# Source the ROS 2 workspace
# Note: There might be some zsh completion warnings, but they're harmless
source install/setup.zsh 2>/dev/null || source install/setup.bash

# Launch the robot
echo "Starting Gazebo, RViz, MoveIt, and Camera Feeds..."
ros2 launch kawada_moveit_config spawn_ur5_launch_moveit.launch.py

# Note: The script will run until you press Ctrl+C


