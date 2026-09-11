cmake_minimum_required(VERSION 3.10)
project(nav2_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  tools/compose_nav2_params.py
  DESTINATION lib/${PROJECT_NAME}
)

# 同时装一份到 share，便于阅读与作为路径兜底
install(FILES tools/compose_nav2_params.py
  DESTINATION share/${PROJECT_NAME}/tools
)

ament_package()
