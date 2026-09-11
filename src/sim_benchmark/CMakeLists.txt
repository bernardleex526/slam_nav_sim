cmake_minimum_required(VERSION 3.10)
project(sim_benchmark)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/benchmark_runner.py
  scripts/stage_a_runner.py
  scripts/stage_b_runner.py
  scripts/exploration_runner.py
  scripts/exploration_routes.py
  scripts/nav_diag.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
