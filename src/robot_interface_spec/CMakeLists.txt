cmake_minimum_required(VERSION 3.10)
project(robot_interface_spec)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(FILES SPEC.md
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/check_interface.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
