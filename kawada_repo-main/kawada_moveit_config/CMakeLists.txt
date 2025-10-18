cmake_minimum_required(VERSION 3.22)
project(kawada_moveit_config)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

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
  scripts/pick_cube_moveit_py.py
  scripts/cube_tf_broadcaster.py
  scripts/gripper_diagnostics.py
  scripts/fast_positions.py
  scripts/ultra_fast_test.py
  DESTINATION lib/${PROJECT_NAME}
)
