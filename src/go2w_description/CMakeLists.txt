cmake_minimum_required(VERSION 3.10)
project(go2w_description)

# -----------------------------------------------------------------------------
# 宇树 Go2W 轮足机器人描述包（ROS2 化）
# -----------------------------------------------------------------------------
# 上游仓库 unitreerobotics/unitree_ros 是 ROS1 catkin 包，这里做了最小改动迁移到
# ament，只保留描述资产（urdf / dae / config）。
#
# 为什么不用上游的 launch：
#   它们拉起的是 ROS1 的 robot_state_publisher / joint_state_publisher_gui，
#   在 ROS2 里根本跑不起来。本平台统一由 robot_descriptions 包负责加载描述、
#   注入传感器与驱动插件、发布 TF（见 xacro/external/go2w/robot.urdf.xacro）。
# -----------------------------------------------------------------------------

find_package(ament_cmake REQUIRED)

# urdf 里引用的是 package://go2w_description/dae/xxx.dae，
# 所以 dae 目录必须原样安装，不能改名。
install(DIRECTORY dae urdf config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
