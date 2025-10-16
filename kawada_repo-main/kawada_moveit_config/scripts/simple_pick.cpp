/**
 * Simple Pick - Direct Joint Control
 * Simple script to move the right arm using joint trajectory controller
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>

using namespace std::chrono_literals;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("simple_pick_node");
  
  RCLCPP_INFO(node->get_logger(), "==========================================");
  RCLCPP_INFO(node->get_logger(), " Simple Pick - Direct Joint Control");
  RCLCPP_INFO(node->get_logger(), "==========================================");
  
  // Create publishers for right arm and gripper controllers
  auto arm_publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/right_arm_controller/joint_trajectory", 10);
  
  auto gripper_publisher = node->create_publisher<trajectory_msgs::msg::JointTrajectory>(
    "/right_gripper_controller/joint_trajectory", 10);
  
  // Wait for connections
  rclcpp::sleep_for(1s);
  RCLCPP_INFO(node->get_logger(), "Publishers ready");
  
  // Define joint names
  std::vector<std::string> joint_names = {
    "RARM_JOINT0", "RARM_JOINT1", "RARM_JOINT2",
    "RARM_JOINT3", "RARM_JOINT4", "RARM_JOINT5"
  };
  
  // Red cube CONFIRMED POSITION: x=0.35, y=-0.2, z=0.875
  // TESTED joint configuration that TOUCHES the cube: [-0.2, 0.4, -0.2, 0.3, 0.0, 0.0]
  
  // ========== VERY FAST MOTION WITH MULTIPLE WAYPOINTS ==========
  
  // Open gripper FIRST (parallel with arm movement)
  RCLCPP_INFO(node->get_logger(), "[1/9] Opening gripper...");
  trajectory_msgs::msg::JointTrajectory gripper_open;
  gripper_open.joint_names = {"right_gripper_left_finger_joint"};
  trajectory_msgs::msg::JointTrajectoryPoint gripper_open_point;
  gripper_open_point.positions = {0.04};  // Fully open
  gripper_open_point.time_from_start = rclcpp::Duration(0, 400000000);  // 0.4s - VERY FAST
  gripper_open.points.push_back(gripper_open_point);
  gripper_publisher->publish(gripper_open);
  rclcpp::sleep_for(500ms);
  
  // Waypoint 6e: BIG FORWARD push to center around cube
  RCLCPP_INFO(node->get_logger(), "    Big FORWARD push to center on cube...");
  trajectory_msgs::msg::JointTrajectory traj6e;
  traj6e.joint_names = joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint point6e;
  // Significantly more forward (JOINT2 more negative)
  point6e.positions = {-0.2, 0.52, -0.45, 0.44, 0.0, 0.0};
  point6e.time_from_start = rclcpp::Duration(0, 500000000);  // 0.5s
  traj6e.points.push_back(point6e);
  arm_publisher->publish(traj6e);
  rclcpp::sleep_for(600ms);
  
  // Waypoint 6f: DEEPER DOWN to cradle the cube
  RCLCPP_INFO(node->get_logger(), "    Going further DOWN to cradle...");
  trajectory_msgs::msg::JointTrajectory traj6f;
  traj6f.joint_names = joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint point6f;
  // Lower more by increasing JOINT1/JOINT3 and tiny extra forward
  point6f.positions = {-0.2, 0.56, -0.47, 0.50, 0.0, 0.0};
  point6f.time_from_start = rclcpp::Duration(0, 500000000);  // 0.5s
  traj6f.points.push_back(point6f);
  arm_publisher->publish(traj6f);
  rclcpp::sleep_for(700ms);

  // Waypoint 1: Start moving shoulder out
  RCLCPP_INFO(node->get_logger(), "[2/9] Moving shoulder to reach zone...");
  trajectory_msgs::msg::JointTrajectory traj1;
  traj1.joint_names = joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint point1;
  point1.positions = {-0.2, 0.1, -0.1, 0.1, -0.3, 0.0};
  point1.time_from_start = rclcpp::Duration(0, 800000000);  // 0.8s - VERY FAST
  traj1.points.push_back(point1);
  arm_publisher->publish(traj1);
  rclcpp::sleep_for(900ms);
  
  // Waypoint 2: Extend arm toward cube area
  RCLCPP_INFO(node->get_logger(), "[3/9] Extending toward cube area...");
  trajectory_msgs::msg::JointTrajectory traj2;
  traj2.joint_names = joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint point2;
  point2.positions = {-0.2, 0.2, -0.15, 0.15, -0.25, 0.0};
  point2.time_from_start = rclcpp::Duration(0, 600000000);  // 0.6s - VERY FAST
  traj2.points.push_back(point2);
  arm_publisher->publish(traj2);
  rclcpp::sleep_for(700ms);
  
  // Waypoint 6d: Final micro-adjust ~2cm forward, ~2cm down
  RCLCPP_INFO(node->get_logger(), "    Micro-adjust +2cm forward, +2cm down...");
  trajectory_msgs::msg::JointTrajectory traj6d;
  traj6d.joint_names = joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint point6d;
  // Slight forward (JOINT2 more negative ~0.02) and lower (JOINT1/JOINT3 +0.02)
  point6d.positions = {-0.2, 0.52, -0.36, 0.44, 0.0, 0.0};
  point6d.time_from_start = rclcpp::Duration(0, 400000000);  // 0.4s
  traj6d.points.push_back(point6d);
  arm_publisher->publish(traj6d);
  rclcpp::sleep_for(500ms);
  
  // Waypoint 3: High pre-grasp (well above cube)
  RCLCPP_INFO(node->get_logger(), "[4/9] Moving to high pre-grasp...");
  trajectory_msgs::msg::JointTrajectory traj3;
  traj3.joint_names = joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint point3;
  point3.positions = {-0.2, 0.25, -0.2, 0.18, -0.2, 0.0};
  point3.time_from_start = rclcpp::Duration(0, 600000000);  // 0.6s - VERY FAST
  traj3.points.push_back(point3);
  arm_publisher->publish(traj3);
  rclcpp::sleep_for(700ms);
  
  // Waypoint 4: Low pre-grasp (closer to cube)
  RCLCPP_INFO(node->get_logger(), "[5/9] Lowering to pre-grasp position...");
  trajectory_msgs::msg::JointTrajectory traj4;
  traj4.joint_names = joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint point4;
  point4.positions = {-0.2, 0.3, -0.2, 0.22, -0.15, 0.0};
  point4.time_from_start = rclcpp::Duration(0, 500000000);  // 0.5s - VERY FAST
  traj4.points.push_back(point4);
  arm_publisher->publish(traj4);
  rclcpp::sleep_for(600ms);
  
  // Waypoint 6c: Micro-center FORWARD and DOWN for better enclosure
  RCLCPP_INFO(node->get_logger(), "    Micro-centering FORWARD and DOWN...");
  trajectory_msgs::msg::JointTrajectory traj6c;
  traj6c.joint_names = joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint point6c;
  // Slightly more forward (JOINT2 more negative) and lower (JOINT1/JOINT3 higher)
  point6c.positions = {-0.2, 0.50, -0.34, 0.42, 0.0, 0.0};
  point6c.time_from_start = rclcpp::Duration(0, 500000000);  // 0.5s
  traj6c.points.push_back(point6c);
  arm_publisher->publish(traj6c);
  rclcpp::sleep_for(700ms);
  
  // Waypoint 5: Final approach
  RCLCPP_INFO(node->get_logger(), "[6/9] Final approach to cube...");
  trajectory_msgs::msg::JointTrajectory traj5;
  traj5.joint_names = joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint point5;
  point5.positions = {-0.2, 0.35, -0.2, 0.27, -0.05, 0.0};
  point5.time_from_start = rclcpp::Duration(0, 500000000);  // 0.5s - VERY FAST
  traj5.points.push_back(point5);
  arm_publisher->publish(traj5);
  rclcpp::sleep_for(600ms);
  
  // Waypoint 6: GRASP POSITION - Touch the cube (TESTED POSITION)
  RCLCPP_INFO(node->get_logger(), "[7/9] GRASP position - touching cube!");
  trajectory_msgs::msg::JointTrajectory traj6;
  traj6.joint_names = joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint point6;
  // EXACT position that touched the cube from testing
  point6.positions = {-0.2, 0.4, -0.2, 0.3, 0.0, 0.0};
  point6.time_from_start = rclcpp::Duration(0, 500000000);  // 0.5s - VERY FAST
  traj6.points.push_back(point6);
  arm_publisher->publish(traj6);
  rclcpp::sleep_for(600ms);
  
  // Waypoint 6b: Move MUCH MORE FORWARD and DOWN to reach cube
  RCLCPP_INFO(node->get_logger(), "    Extending FORWARD and DOWN to cube...");
  trajectory_msgs::msg::JointTrajectory traj6b;
  traj6b.joint_names = joint_names;
  trajectory_msgs::msg::JointTrajectoryPoint point6b;
  // Go MUCH MORE FORWARD (decrease JOINT2 more) and DOWN (increase JOINT1 and JOINT3)
  point6b.positions = {-0.2, 0.48, -0.3, 0.38, 0.0, 0.0};
  point6b.time_from_start = rclcpp::Duration(0, 500000000);  // 0.5s
  traj6b.points.push_back(point6b);
  arm_publisher->publish(traj6b);
  rclcpp::sleep_for(600ms);
  
  // Waypoint 7: CLOSE GRIPPER to grasp cube
  RCLCPP_INFO(node->get_logger(), "[8/9] CLOSING GRIPPER - grasping cube!");
  trajectory_msgs::msg::JointTrajectory gripper_close;
  gripper_close.joint_names = {"right_gripper_left_finger_joint"};
  trajectory_msgs::msg::JointTrajectoryPoint gripper_close_point;
  gripper_close_point.positions = {0.004};  // Close gently first
  gripper_close_point.time_from_start = rclcpp::Duration(0, 800000000);  // 0.8s - slower for firm contact
  gripper_close.points.push_back(gripper_close_point);
  gripper_publisher->publish(gripper_close);
  rclcpp::sleep_for(900ms);  // Allow physics to settle on contact

  // Extra squeeze to secure grasp
  trajectory_msgs::msg::JointTrajectory gripper_squeeze;
  gripper_squeeze.joint_names = {"right_gripper_left_finger_joint"};
  trajectory_msgs::msg::JointTrajectoryPoint gripper_squeeze_point;
  gripper_squeeze_point.positions = {0.001};  // Tighten grip
  gripper_squeeze_point.time_from_start = rclcpp::Duration(0, 700000000);  // 0.7s
  gripper_squeeze.points.push_back(gripper_squeeze_point);
  gripper_publisher->publish(gripper_squeeze);
  rclcpp::sleep_for(900ms);
  
  // Hold position without lifting
  RCLCPP_INFO(node->get_logger(), "Holding grip without lifting...");
  rclcpp::sleep_for(2s);
  
  RCLCPP_INFO(node->get_logger(), "==========================================");
  RCLCPP_INFO(node->get_logger(), " Motion sequence completed!");
  RCLCPP_INFO(node->get_logger(), "==========================================");
  
  rclcpp::shutdown();
  return 0;
}

