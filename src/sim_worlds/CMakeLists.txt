cmake_minimum_required(VERSION 3.10)
project(sim_worlds)

find_package(ament_cmake REQUIRED)

install(DIRECTORY worlds maps launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/generate_worlds.py
  scripts/dynamic_obstacle_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
