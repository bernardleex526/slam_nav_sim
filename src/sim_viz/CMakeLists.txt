cmake_minimum_required(VERSION 3.10)
project(sim_viz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY rviz launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  tools/gen_rviz_configs.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
