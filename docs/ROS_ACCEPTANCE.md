# Linux 实跑验收与后续矩阵

本轮已在 Ubuntu VM 完成环境安装、原生 C++ 插件构建、真实接触验证、独立参考健康导航及三轮完整参考／生成地图对照。详细结果见 [实跑报告](ROS_EXECUTION.md)。Docker 镜像尚未构建，完整矩阵的失败项仍阻断整体通过。以下保留部署／后续回归清单；ROS 故障注入全矩阵、多种子、回环和公开包回放尚未执行。原生部署默认 `rmw_cyclonedds_cpp`，并记录实际 RMW；此前 Fast DDS 启动中断证据也保留。Ubuntu 24.04、ROS 2 Jazzy 与 Gazebo Harmonic 的组合按 [Gazebo 官方兼容表](https://gazebosim.org/docs/harmonic/ros_installation/) 固定。

1. 执行 `docker build -f deploy/Dockerfile -t robot-validation:jazzy .`。核对接触监测插件 CMake、Jazzy vendor 库和软件渲染。镜像内保存 `deploy/apt-resolved.txt` 与 `python-resolved.txt`；验收通过后记录镜像 digest，锁定构建产物。
2. 执行 `./deploy/run.sh doctor`，确认 ROS 2、Gazebo、rclpy 与本地依赖存在。
3. 执行 `./deploy/run.sh sim --scenario healthy --output /results/first-healthy`。核对时钟推进、激光频率、TF、四个碰撞体计数和连续接触帧。若渲染失败或监测缺失，报告应为环境错误／证据不足，不能通过。
4. 对照生成地图原点与初始世界位姿；核对 reference／generated 两个阶段保存的地图哈希及实际 `/map` 栅格匹配。检查每阶段的规划路径、实际真值轨迹和事件图。
5. 独立运行 `sim --navigation-only --scenario demo`。健康目标应到达；阻挡移除后恢复；不可达目标应在时限内受控失败并停止。narrow 已增加固定途经点和实际穿越判据；每次仍须核对穿越检查，不能只看动作成功。
6. 完整运行 `sim --scenario demo`，再执行扫描断流、轮速偏置、地图导出失败和缺真值对照。保留每次非零状态、部分报告、时间段和日志。每个故障与同种子基线配对。
7. 使用 `configs/demo.yaml` 与 `configs/loop-off.yaml` 在有回访路线运行开关对照。当前只有开关配置和轨迹，未接入内部候选／接受事件，不足以归因回环失败原因。
8. 对至少三个固定种子复跑。中途发送 SIGINT、制造墙钟超时，核对进程台账中的进程组全部退出；无需停止全局 ROS daemon。
9. 用 `bag --backend ros2 --scenario public` 验证原始传感器回放。该包没有独立真值，预期不能给精度通过。随后接入具有明确独立真值和许可的公开二维激光包，完成原计划未覆盖的那部分验收。

参考接口：

- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox/blob/ros2/README.md)：建图、回环开关与 ROS 接口。
- [Nav2 Jazzy 源码](https://github.com/ros-navigation/navigation2/tree/94d1c2a345dda6ff637e6a72c9101d927ee99f19)：启动和参数接口审查基准；运行使用镜像 apt 实际安装版本并记录版本清单。
- [Gazebo DiffDrive](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1DiffDrive.html)、[独立位姿发布器](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1OdometryPublisher.html)：轮速估计与世界真值分离。
- [Gazebo 原生接触系统源码](https://github.com/gazebosim/gz-sim/blob/gz-sim8/src/systems/contact/Contact.cc)：原生发布只在有接触时发生；本项目补充连续空帧与监测体数量证据。
- [evo 指标及对齐](https://github.com/MichaelGrupp/evo/wiki/Metrics)：ATE／RPE 和不修正尺度的刚体对齐。

## v0.1.3 最新回归

原临时阻挡两项已修复；锁定原 005 生成地图后，完整 8 场景通过，详见 [BLOCK_RECOVERY_FIX.md](BLOCK_RECOVERY_FIX.md)。保留了 fresh SLAM 退化及中间失败报告。本次通过的是历史地图上的导航回归，不等于本轮重新建图通过。

实际执行的 pytest 契约为 `configs/ros-block-recovery.case.json`。复跑到新目录可使用 `deploy/vm-run.sh sim --navigation-only --generated-map results/ros2-demo-005/map.yaml --scenario demo --output /mnt/robot-validation/results/block-recovery-new --timeout 1800`。
