cmake_minimum_required(VERSION 3.10)
project(sim_metrics)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  nodes/metrics_collector.py
  scripts/evaluate_vs_gt.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
