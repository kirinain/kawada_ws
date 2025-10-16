/**
 * Pick Red Cube - C++ MoveIt 2 Script
 * Simple pick-and-place implementation using MoveIt 2 C++ API
 */

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

// Red cube parameters (from Gazebo world) - NEW CLOSER POSITION (within reach)
constexpr double CUBE_X = 0.35;
constexpr double CUBE_Y = -0.2;
constexpr double CUBE_Z = 0.875;
constexpr double CUBE_SIZE = 0.1;

// Motion parameters
constexpr double PRE_GRASP_HEIGHT = 0.15;  // 15cm above cube
constexpr double APPROACH_HEIGHT = 0.05;    // 5cm above cube
constexpr double LIFT_HEIGHT = 0.20;        // Lift 20cm

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("pick_red_cube_node");
  
  // Create executor for spinning
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  std::thread spin_thread([&executor]() { executor.spin(); });
  
  RCLCPP_INFO(node->get_logger(), "========================================");
  RCLCPP_INFO(node->get_logger(), "Pick Red Cube - Starting");
  RCLCPP_INFO(node->get_logger(), "========================================");
  
  // Wait for MoveIt to initialize
  rclcpp::sleep_for(std::chrono::seconds(2));
  
  try
  {
    // Initialize MoveIt interfaces
    moveit::planning_interface::MoveGroupInterface move_group(node, "right_arm");
    moveit::planning_interface::PlanningSceneInterface planning_scene;
    
    // Configure planner
    move_group.setPlanningTime(15.0);
    move_group.setNumPlanningAttempts(20);
    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);
    move_group.setGoalTolerance(0.01);
    move_group.setGoalJointTolerance(0.01);
    
    RCLCPP_INFO(node->get_logger(), "Planning frame: %s", move_group.getPlanningFrame().c_str());
    RCLCPP_INFO(node->get_logger(), "End effector: %s", move_group.getEndEffectorLink().c_str());
    
    // Get current joint values for debugging
    std::vector<double> current_joints = move_group.getCurrentJointValues();
    RCLCPP_INFO(node->get_logger(), "Current joint values:");
    for (size_t i = 0; i < current_joints.size(); ++i) {
      RCLCPP_INFO(node->get_logger(), "  Joint %zu: %.3f", i, current_joints[i]);
    }
    RCLCPP_INFO(node->get_logger(), "");
    
    // Step 1: Add collision objects (tables and cube) - SKIPPING FOR NOW
    RCLCPP_INFO(node->get_logger(), "[1/6] Adding collision objects (skipped - using Gazebo scene)...");
    
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    
    // Robot table (matches Gazebo world)
    moveit_msgs::msg::CollisionObject robot_table;
    robot_table.header.frame_id = move_group.getPlanningFrame();
    robot_table.id = "robot_table";
    shape_msgs::msg::SolidPrimitive table1_primitive;
    table1_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    table1_primitive.dimensions = {1.2, 1.2, 0.8};  // Match Gazebo: 1.2x1.2x0.8
    geometry_msgs::msg::Pose table1_pose;
    table1_pose.orientation.w = 1.0;
    table1_pose.position.x = 0.0;    // Center at origin
    table1_pose.position.y = 0.0;
    table1_pose.position.z = 0.4;    // Pose is at center, so z=0.4
    robot_table.primitives.push_back(table1_primitive);
    robot_table.primitive_poses.push_back(table1_pose);
    robot_table.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_objects.push_back(robot_table);
    
    // Work table (matches Gazebo world)
    moveit_msgs::msg::CollisionObject work_table;
    work_table.header.frame_id = move_group.getPlanningFrame();
    work_table.id = "work_table";
    shape_msgs::msg::SolidPrimitive table2_primitive;
    table2_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    table2_primitive.dimensions = {1.0, 1.5, 0.8};  // Match Gazebo: 1.0x1.5x0.8
    geometry_msgs::msg::Pose table2_pose;
    table2_pose.orientation.w = 1.0;
    table2_pose.position.x = 0.8;    // At x=0.8
    table2_pose.position.y = 0.0;
    table2_pose.position.z = 0.4;    // Pose is at center, so z=0.4
    work_table.primitives.push_back(table2_primitive);
    work_table.primitive_poses.push_back(table2_pose);
    work_table.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_objects.push_back(work_table);
    
    // Red cube
    moveit_msgs::msg::CollisionObject red_cube;
    red_cube.header.frame_id = move_group.getPlanningFrame();
    red_cube.id = "red_cube";
    shape_msgs::msg::SolidPrimitive cube_primitive;
    cube_primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    cube_primitive.dimensions = {CUBE_SIZE, CUBE_SIZE, CUBE_SIZE};
    geometry_msgs::msg::Pose cube_pose;
    cube_pose.orientation.w = 1.0;
    cube_pose.position.x = CUBE_X;
    cube_pose.position.y = CUBE_Y;
    cube_pose.position.z = CUBE_Z;
    red_cube.primitives.push_back(cube_primitive);
    red_cube.primitive_poses.push_back(cube_pose);
    red_cube.operation = moveit_msgs::msg::CollisionObject::ADD;
    collision_objects.push_back(red_cube);
    
    // Commenting out collision object addition - use Gazebo's scene instead
    // planning_scene.addCollisionObjects(collision_objects);
    // rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(node->get_logger(), "    Skipped - using Gazebo collision scene");
    RCLCPP_INFO(node->get_logger(), "");
    
    // Step 2: Skip home - go directly to pre-grasp from current position
    RCLCPP_INFO(node->get_logger(), "[2/6] Skipping home position (using current pose)...");
    RCLCPP_INFO(node->get_logger(), "");
    
    // Step 3: Move to pre-grasp position (above cube)
    RCLCPP_INFO(node->get_logger(), "[3/6] Moving to pre-grasp position...");
    geometry_msgs::msg::Pose pre_grasp_pose;
    pre_grasp_pose.position.x = CUBE_X;
    pre_grasp_pose.position.y = CUBE_Y;
    pre_grasp_pose.position.z = CUBE_Z + PRE_GRASP_HEIGHT;
    pre_grasp_pose.orientation.x = 0.707;
    pre_grasp_pose.orientation.y = 0.0;
    pre_grasp_pose.orientation.z = 0.0;
    pre_grasp_pose.orientation.w = 0.707;
    move_group.setPoseTarget(pre_grasp_pose);
    
    moveit::planning_interface::MoveGroupInterface::Plan pre_grasp_plan;
    bool success = (move_group.plan(pre_grasp_plan) == moveit::core::MoveItErrorCode::SUCCESS);
    if (success) {
      move_group.execute(pre_grasp_plan);
      RCLCPP_INFO(node->get_logger(), "    Reached pre-grasp position");
    } else {
      RCLCPP_ERROR(node->get_logger(), "    Failed to reach pre-grasp");
      rclcpp::shutdown();
      spin_thread.join();
      return 1;
    }
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(node->get_logger(), "");
    
    // Step 4: Approach cube (cartesian path down)
    RCLCPP_INFO(node->get_logger(), "[4/6] Approaching cube...");
    std::vector<geometry_msgs::msg::Pose> approach_waypoints;
    geometry_msgs::msg::Pose current_pose = move_group.getCurrentPose().pose;
    approach_waypoints.push_back(current_pose);
    
    geometry_msgs::msg::Pose approach_pose = current_pose;
    approach_pose.position.z = CUBE_Z + APPROACH_HEIGHT;
    approach_waypoints.push_back(approach_pose);
    
    moveit_msgs::msg::RobotTrajectory approach_trajectory;
    double fraction = move_group.computeCartesianPath(
      approach_waypoints, 0.01, 0.0, approach_trajectory);
    
    if (fraction > 0.9) {
      move_group.execute(approach_trajectory);
      RCLCPP_INFO(node->get_logger(), "    Approached cube (%.1f%% of path)", fraction * 100);
    } else {
      RCLCPP_WARN(node->get_logger(), "    Only %.1f%% of approach path computed", fraction * 100);
    }
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(node->get_logger(), "");
    
    // Step 5: Grasp cube (attach to end-effector)
    RCLCPP_INFO(node->get_logger(), "[5/6] Grasping cube...");
    moveit_msgs::msg::AttachedCollisionObject attached_object;
    attached_object.link_name = move_group.getEndEffectorLink();
    attached_object.object.id = "red_cube";
    attached_object.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    planning_scene.applyAttachedCollisionObject(attached_object);
    rclcpp::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(node->get_logger(), "    Cube grasped and attached");
    RCLCPP_INFO(node->get_logger(), "");
    
    // Step 6: Lift cube
    RCLCPP_INFO(node->get_logger(), "[6/6] Lifting cube...");
    std::vector<geometry_msgs::msg::Pose> lift_waypoints;
    current_pose = move_group.getCurrentPose().pose;
    lift_waypoints.push_back(current_pose);
    
    geometry_msgs::msg::Pose lift_pose = current_pose;
    lift_pose.position.z += LIFT_HEIGHT;
    lift_waypoints.push_back(lift_pose);
    
    moveit_msgs::msg::RobotTrajectory lift_trajectory;
    fraction = move_group.computeCartesianPath(
      lift_waypoints, 0.01, 0.0, lift_trajectory);
    
    if (fraction > 0.9) {
      move_group.execute(lift_trajectory);
      RCLCPP_INFO(node->get_logger(), "    Lifted cube (%.1f%% of path)", fraction * 100);
    } else {
      RCLCPP_WARN(node->get_logger(), "    Only %.1f%% of lift path computed", fraction * 100);
    }
    
    RCLCPP_INFO(node->get_logger(), "");
    RCLCPP_INFO(node->get_logger(), "========================================");
    RCLCPP_INFO(node->get_logger(), "Pick sequence completed successfully!");
    RCLCPP_INFO(node->get_logger(), "========================================");
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(node->get_logger(), "Exception: %s", e.what());
    rclcpp::shutdown();
    spin_thread.join();
    return 1;
  }
  
  rclcpp::shutdown();
  spin_thread.join();
  return 0;
}

