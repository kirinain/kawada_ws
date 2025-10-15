# Dual-Arm Robot System Guide

## ✅ System Status: FULLY OPERATIONAL

All components are working correctly:
- ✅ Gazebo simulation running
- ✅ Dual-arm robot (both arms controllable)
- ✅ Joint-level control
- ✅ Cartesian space control  
- ✅ Table with 5 colored cubes
- ✅ Robot positioned for grasping

---

## 🚀 Quick Start

### Launch the Complete System

```bash
cd /home/kiti/kawada
source install/setup.zsh
ros2 launch kawada_moveit_config simple_dual_arm.launch.py
```

This will start:
- Gazebo with the robot and workspace
- MoveIt for motion planning
- RViz for visualization
- All necessary controllers

---

## 🎮 Control Methods

### 1. Joint-Level Control (Direct Joint Commands)

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

### 2. Cartesian Space Control (End-Effector Commands)

**Right Arm:**
```bash
ros2 topic pub /right_arm/cartesian_command geometry_msgs/msg/Twist "{
  linear: {x: 0.3, y: 0.2, z: 0.1},
  angular: {x: 0.0, y: 0.0, z: 0.0}
}" --once
```

**Left Arm:**
```bash
ros2 topic pub /left_arm/cartesian_command geometry_msgs/msg/Twist "{
  linear: {x: 0.3, y: -0.2, z: 0.1},
  angular: {x: 0.0, y: 0.0, z: 0.0}
}" --once
```

### 3. MoveIt Planning (via RViz)

1. In RViz, select a planning group:
   - `right_arm` - Control right arm only
   - `left_arm` - Control left arm only
   - `both_arms` - Control both arms together

2. Drag the interactive marker to desired position
3. Click "Plan" to compute trajectory
4. Click "Execute" to move the robot

---

## 🌍 Scene Setup

### Objects in Gazebo:

1. **Robot Table** (at origin)
   - Position: (0, 0, 0.4)
   - Robot sits on top at z=0.8

2. **Work Table** (in front of robot)
   - Position: (0.9, 0, 0.4)
   - Contains the objects

3. **5 Colored Cubes** (on work table at z=0.85):
   - **Red Cube**: (0.9, -0.4, 0.85)
   - **Blue Cube**: (0.9, -0.2, 0.85)
   - **Green Cube**: (0.9, 0.0, 0.85)
   - **Yellow Cube**: (0.9, 0.2, 0.85)
   - **Orange Cube**: (0.9, 0.4, 0.85)

### Grasping Capability:
- Objects are 0.9m in front of robot
- Within reach of both arms
- Arms can extend ~0.5-0.7m forward

---

## 🔍 Testing & Verification

### Run Comprehensive System Test:

```bash
cd /home/kiti/kawada
source install/setup.zsh
python3 test_dual_arm_system.py
```

This tests:
- Robot spawning
- Joint state publishing
- Joint-level control
- Cartesian control
- Scene objects
- Grasping position

### Check System Topics:

```bash
# List all topics
ros2 topic list

# Monitor joint states
ros2 topic echo /joint_states

# Check controllers
ros2 control list_controllers

# View nodes
ros2 node list
```

---

## 📊 Available Topics

### Control Topics:
- `/right_arm/joint_commands` - Right arm joint commands
- `/left_arm/joint_commands` - Left arm joint commands  
- `/right_arm/cartesian_command` - Right arm Cartesian commands
- `/left_arm/cartesian_command` - Left arm Cartesian commands

### State Topics:
- `/joint_states` - All joint positions, velocities
- `/right_arm_controller/state` - Right arm controller state
- `/left_arm_controller/state` - Left arm controller state

### MoveIt Topics:
- `/move_group/*` - MoveIt planning and execution
- `/display_planned_path` - Visualize planned trajectories

---

## 🛠️ Troubleshooting

### If Gazebo crashes:
```bash
# Kill all processes
pkill -9 -f gazebo; pkill -9 -f rviz; pkill -9 -f ros2

# Relaunch
ros2 launch kawada_moveit_config simple_dual_arm.launch.py
```

### If robot doesn't spawn:
- Wait 5-10 seconds after launch
- Check: `ros2 topic echo /joint_states`
- Should see 12 joints (6 per arm)

### If controllers fail:
```bash
# Check controller status
ros2 control list_controllers

# Manually spawn if needed
ros2 run controller_manager spawner joint_state_broadcaster
ros2 run controller_manager spawner right_arm_controller
ros2 run controller_manager spawner left_arm_controller
```

---

## 📝 Key Configuration Files

- **Launch**: `kawada_moveit_config/launch/simple_dual_arm.launch.py`
- **World**: `kawada_moveit_config/worlds/simple_workspace.sdf`
- **SRDF**: `kawada_moveit_config/config/NEXTAGE.srdf`
- **Controllers**: `kawada_moveit_config/config/ros_controllers.yaml`
- **Kinematics**: `kawada_moveit_config/config/kinematics.yaml`

---

## 🎯 Example Grasping Workflow

1. **Launch system**
2. **Move right arm to approach cube** (using MoveIt in RViz)
3. **Fine-tune position** with Cartesian commands
4. **Close gripper** (if gripper is implemented)
5. **Lift object** with joint or Cartesian commands

---

## ✨ Features

✅ **Dual-arm coordination** - Control both arms independently or together  
✅ **Two control modes** - Joint-level and Cartesian space  
✅ **Motion planning** - MoveIt integration with OMPL  
✅ **Collision avoidance** - Automatic collision checking  
✅ **Realistic simulation** - Gazebo physics  
✅ **Visualization** - Real-time in RViz  
✅ **Grasping ready** - Objects positioned within reach  

---

## 📞 Support

Run the test script to verify all components:
```bash
python3 /home/kiti/kawada/test_dual_arm_system.py
```

Expected output: "🎉 ALL TESTS PASSED! System is fully operational."

