# NEXTAGE Dual-Arm Robot - ROS 2 & MoveIt Configuration

A complete dual-arm manipulation system for the NEXTAGE robot with ROS 2 Humble, Gazebo Fortress, and MoveIt 2.

## 🤖 System Overview

- **Robot**: NEXTAGE dual-arm humanoid robot (6-DOF per arm, 12-DOF total)
- **ROS Version**: ROS 2 Humble
- **Simulator**: Gazebo Fortress (Ignition)
- **Motion Planning**: MoveIt 2 with OMPL
- **Control Modes**: Joint-level and Cartesian space control

## ✅ Features

- ✅ Dual-arm coordination (independent or simultaneous control)
- ✅ Joint-level control via ROS 2 topics
- ✅ Cartesian space control for end-effectors
- ✅ MoveIt motion planning with collision avoidance
- ✅ Gazebo simulation with physics
- ✅ Pick-and-place ready workspace (tables + 5 objects)
- ✅ RViz visualization

## 📦 Package Structure

```
kawada/
├── kawada_repo-main/
│   ├── kawada_moveit_config/          # MoveIt configuration
│   │   ├── config/                     # SRDF, controllers, kinematics
│   │   ├── launch/                     # Launch files
│   │   ├── worlds/                     # Gazebo worlds
│   │   └── scripts/                    # Python control scripts
│   └── nextage_fillie_open_description/  # Robot URDF
├── test_dual_arm_system.py            # System verification script
├── demo_dual_arm_control.py           # Control demonstration
└── SYSTEM_GUIDE.md                    # Complete usage guide
```

## 🚀 Quick Start

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

Expected output: `🎉 ALL TESTS PASSED! System is fully operational.`

## 🎮 Control Methods

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

## 🌍 Workspace Setup

### Scene Objects:
- **Robot Table** (gray) at origin - robot base
- **Work Table** (brown) at x=0.9m - object placement
- **5 Colored Cubes** on work table:
  - Red (y=-0.4m)
  - Blue (y=-0.2m)
  - Green (y=0.0m)
  - Yellow (y=0.2m)
  - Orange (y=0.4m)

All objects at z=0.85m, within arm reach (~0.9m distance)

## 🧪 Testing & Demos

### Comprehensive System Test
```bash
python3 test_dual_arm_system.py
```

### Live Control Demonstration
```bash
python3 demo_dual_arm_control.py
```

## 📊 Available Topics

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

## 🛠️ Troubleshooting

### Kill All Processes:
```bash
pkill -9 -f gazebo; pkill -9 -f rviz; pkill -9 -f ros2
```

### Relaunch:
```bash
ros2 launch kawada_moveit_config simple_dual_arm.launch.py
```

### Check System Status:
```bash
ros2 node list
ros2 topic list
ros2 control list_controllers
```

## 📚 Documentation

- **System Guide**: `SYSTEM_GUIDE.md` - Complete usage manual
- **Test Script**: `test_dual_arm_system.py` - Automated verification
- **Demo Script**: `demo_dual_arm_control.py` - Control examples

## 🎯 Use Cases

- Dual-arm manipulation tasks
- Pick-and-place operations
- Coordinated bimanual assembly
- Motion planning research
- Robot learning experiments

## 🤝 Contributing

1. Make changes to configuration or launch files
2. Test with `python3 test_dual_arm_system.py`
3. Verify in simulation
4. Commit changes

## 📄 License

See individual package licenses in `kawada_repo-main/`

## 🆘 Support

For issues or questions, refer to `SYSTEM_GUIDE.md` or run the test script for diagnostics.

---

**System Status**: ✅ Fully Operational  
**Last Tested**: October 2025  
**ROS 2 Version**: Humble  
**Gazebo Version**: Fortress
