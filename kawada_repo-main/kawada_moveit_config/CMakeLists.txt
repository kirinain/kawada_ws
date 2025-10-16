cmake_minimum_required(VERSION 3.22)
project(kawada_moveit_config)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

# Build pick_red_cube executable (MoveIt version)
add_executable(pick_red_cube scripts/pick_red_cube.cpp)
ament_target_dependencies(pick_red_cube
  rclcpp
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  shape_msgs
)

# Build simple_pick executable (direct joint control)
find_package(trajectory_msgs REQUIRED)
add_executable(simple_pick scripts/simple_pick.cpp)
ament_target_dependencies(simple_pick
  rclcpp
  trajectory_msgs
)

install(TARGETS pick_red_cube simple_pick DESTINATION lib/${PROJECT_NAME})

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "setup_assistant.launch" EXCLUDE)

install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
  )

install(
  DIRECTORY worlds
  DESTINATION share/${PROJECT_NAME}
  )

install(FILES .setup_assistant DESTINATION share/${PROJECT_NAME})

install(PROGRAMS
  scripts/cartesian_control_publisher.py
  scripts/grab_cube.py
  DESTINATION lib/${PROJECT_NAME}
)
