cmake_minimum_required(VERSION 3.13)
project(robot_descriptions)
if(CMAKE_VERSION VERSION_GREATER_EQUAL "4.0")
  # Ubuntu 22.04 Gazebo dependencies predate CMake 4's removed policies.
  set(CMAKE_POLICY_VERSION_MINIMUM 3.5)
endif()

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GAZEBO REQUIRED IMPORTED_TARGET gazebo)

add_library(sim_planar_constraint SHARED plugins/planar_constraint.cpp)
target_compile_features(sim_planar_constraint PRIVATE cxx_std_17)
target_link_libraries(sim_planar_constraint PkgConfig::GAZEBO)
install(TARGETS sim_planar_constraint LIBRARY DESTINATION lib)
find_package(gazebo_ros REQUIRED)
find_package(std_msgs REQUIRED)
add_library(sim_contact_monitor SHARED plugins/contact_monitor.cpp)
target_compile_features(sim_contact_monitor PRIVATE cxx_std_17)
target_link_libraries(sim_contact_monitor PkgConfig::GAZEBO)
ament_target_dependencies(sim_contact_monitor gazebo_ros std_msgs)
install(TARGETS sim_contact_monitor LIBRARY DESTINATION lib)

install(DIRECTORY config xacro launch
  DESTINATION share/${PROJECT_NAME}
)

# rigidify_joints.py 走 share 路径被 launch 的 Command 直接调用（管道），
# 不能放 lib/ 下用 ros2 run，所以这里单独装到 share/scripts。
install(DIRECTORY scripts
  DESTINATION share/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.py"
)

install(PROGRAMS
  scripts/sim_joint_state_bridge.py
  scripts/gimbal_stabilizer.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
