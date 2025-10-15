cmake_minimum_required(VERSION 3.22)
project(kawada_moveit_config)

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
  scripts/cartesian_control_publisher.py
  scripts/grab_cube.py
  DESTINATION lib/${PROJECT_NAME}
)
