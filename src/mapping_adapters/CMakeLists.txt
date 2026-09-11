cmake_minimum_required(VERSION 3.10)
project(mapping_adapters)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  nodes/mapping_contract_adapter.py
  nodes/points_to_scan.py
  nodes/cloud_downsampler.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
