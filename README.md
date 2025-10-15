# NEXTAGE Dual-Arm Robot - ROS 2 & MoveIt Configuration

A complete dual-arm manipulation system for the NEXTAGE robot with ROS 2 Humble, Gazebo Fortress, and MoveIt 2.

## System Overview

- **Robot**: NEXTAGE dual-arm humanoid robot (6-DOF per arm, 12-DOF total)
- **ROS Version**: ROS 2 Humble
- **Simulator**: Gazebo Fortress (Ignition)
- **Motion Planning**: MoveIt 2 with OMPL
- **Control Modes**: Joint-level and Cartesian space control

## Prerequisites & Installation


### 1. Install ROS 2 Humble

Follow the official installation guide: [ROS 2 Humble Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Setup sources
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
```

### 2. Install Gazebo Fortress

Follow the official installation guide: [Gazebo Fortress Installation](https://gazebosim.org/docs/fortress/install_ubuntu)

```bash
# Install Gazebo Fortress
sudo apt-get update
sudo apt-get install lsb-release wget gnupg

sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

sudo apt-get update
sudo apt-get install ignition-fortress
```

### 3. Install ROS 2 - Gazebo Bridge

```bash
sudo apt install ros-humble-ros-gz
```

### 4. Install MoveIt 2

Follow the official installation guide: [MoveIt 2 Installation](https://moveit.ros.org/install-moveit2/binary/)

```bash
sudo apt install ros-humble-moveit
```

### 5. Install Additional Dependencies

```bash
# ROS 2 Control packages
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers ros-humble-gz-ros2-control

# MoveIt plugins and tools
sudo apt install ros-humble-moveit-planners-ompl ros-humble-moveit-simple-controller-manager

# Visualization and tools
sudo apt install ros-humble-rviz2 ros-humble-xacro ros-humble-joint-state-publisher-gui

# Python dependencies
sudo apt install python3-colcon-common-extensions
```

### 6. Setup ROS 2 Environment

Add to your `~/.bashrc` or `~/.zshrc`:

```bash
source /opt/ros/humble/setup.bash  # or setup.zsh for zsh
```

Then source it:
```bash
source ~/.bashrc  # or ~/.zshrc
```

## Package Structure

```
kawada/
├── kawada_repo-main/
│   ├── kawada_moveit_config/          
│   │   ├── config/                     
│   │   ├── launch/                     
│   │   ├── worlds/                     
│   │   └── scripts/                    
│   └── nextage_fillie_open_description/  
├── test_dual_arm_system.py            
├── demo_dual_arm_control.py           
└── SYSTEM_GUIDE.md                    
```

## Quick Start

### 1. Build the Workspace

```bash
cd /home/kiti/kawada
colcon build
source install/setup.bash  # or setup.zsh for zsh
```

### 2. Launch the System

```bash
ros2 launch kawada_moveit_config simple_dual_arm.launch.py
```

This launches:
- Gazebo with robot and workspace
- MoveIt for motion planning
- RViz for visualization
- All controllers

### 3. Verify Installation

```bash
python3 test_dual_arm_system.py
```

## Control Methods

### Joint-Level Control

**Right Arm:**
```bash
ros2 topic pub /right_arm/joint_commands sensor_msgs/msg/JointState "{
  name: ['RARM_JOINT0', 'RARM_JOINT1', 'RARM_JOINT2', 'RARM_JOINT3', 'RARM_JOINT4', 'RARM_JOINT5'],
  position: [0.0, 0.5, -0.8, 0.3, 0.5, 0.0]
}" --once
```

**Left Arm:**
```bash
ros2 topic pub /left_arm/joint_commands sensor_msgs/msg/JointState "{
  name: ['LARM_JOINT0', 'LARM_JOINT1', 'LARM_JOINT2', 'LARM_JOINT3', 'LARM_JOINT4', 'LARM_JOINT5'],
  position: [0.0, 0.5, -0.8, 0.3, 0.5, 0.0]
}" --once
```

### Cartesian Space Control

**Right Arm:**
```bash
ros2 topic pub /right_arm/cartesian_command geometry_msgs/msg/Twist "{
  linear: {x: 0.3, y: 0.2, z: 0.1}
}" --once
```

**Left Arm:**
```bash
ros2 topic pub /left_arm/cartesian_command geometry_msgs/msg/Twist "{
  linear: {x: 0.3, y: -0.2, z: 0.1}
}" --once
```

### MoveIt Planning (via RViz)

1. Select planning group: `right_arm`, `left_arm`, or `both_arms`
2. Drag interactive marker to desired position
3. Click "Plan" → "Execute"

## Testing & Demos

### Comprehensive System Test
```bash
python3 test_dual_arm_system.py
```

### Live Control Demonstration
```bash
python3 demo_dual_arm_control.py
```

## Available Topics

### Control:
- `/right_arm/joint_commands` - Right arm joint control
- `/left_arm/joint_commands` - Left arm joint control
- `/right_arm/cartesian_command` - Right arm Cartesian control
- `/left_arm/cartesian_command` - Left arm Cartesian control

### State:
- `/joint_states` - All joint states
- `/right_arm_controller/state` - Right arm controller state
- `/left_arm_controller/state` - Left arm controller state

### MoveIt:
- `/move_group/*` - Planning and execution
- `/display_planned_path` - Trajectory visualization

## 🔧 Configuration Files

| File | Purpose |
|------|---------|
| `config/NEXTAGE.srdf` | Robot semantic description |
| `config/ros_controllers.yaml` | ROS 2 controllers |
| `config/moveit_controllers.yaml` | MoveIt controllers |
| `config/kinematics.yaml` | Kinematics solvers |
| `config/NEXTAGE.urdf.xacro` | Robot URDF |
| `worlds/simple_workspace.sdf` | Gazebo world |

## 📝 Robot Specifications

### Degrees of Freedom:
- **Right Arm**: 6 DOF
- **Left Arm**: 6 DOF
- **Total**: 12 DOF

### Joint Names:
- Right: `RARM_JOINT0` to `RARM_JOINT5`
- Left: `LARM_JOINT0` to `LARM_JOINT5`

### Planning Groups:
- `right_arm` - Right arm only
- `left_arm` - Left arm only
- `both_arms` - Coordinated dual-arm

### End Effectors:
- `right_hand` - Right arm end effector
- `left_hand` - Left arm end effector


