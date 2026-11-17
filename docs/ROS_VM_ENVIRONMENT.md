# 本机 ROS 2 验证环境

本轮在 Apple Silicon macOS 上使用隔离的 Ubuntu 24.04 arm64 虚拟机运行，宿主机不需要安装 ROS 或 Docker。

- 虚拟机：Lima 2.3.0，VZ，名称 `rv`，4 核／6 GiB 内存／40 GiB 稀疏磁盘。
- 运行文件：当前工作区 `work/ros2-runtime/`。源码挂载到 guest `/mnt/robot-validation`，不挂载用户主目录，不转发 SSH agent。
- 镜像：Ubuntu 24.04 minimal，固定发布 20261105，SHA256 `3dc8370d923978d4b610fa7964d1b36ba040eefb43198809ca5bc8ac51777b18`。
- ROS 软件源引导：`ros2-apt-source` 1.3.0，安装前核对 SHA256。依赖包由 apt 校验，实际版本清单保存在 `results/ros2-environment/`。
- 仿真：ROS 2 Jazzy、Gazebo Harmonic，软件渲染；接触监测插件在 Linux 内编译。部署默认采用 Cyclone DDS（`rmw_cyclonedds_cpp`），实际实现记录在每轮 provenance。

安装脚本 `deploy/setup-ubuntu.sh` 适用于专用 Ubuntu 24.04 环境。已准备的本机虚拟机可以这样使用：

```bash
# 在插件根目录执行；VM 停止时会先启动。
./deploy/vm-run.sh doctor
./deploy/vm-run.sh sim --scenario healthy \
  --output /mnt/robot-validation/results/ros2-healthy-new --timeout 1800

# 完整矩阵：现场建图，随后对照参考地图／生成地图的导航。
./deploy/vm-run.sh sim --scenario demo \
  --output /mnt/robot-validation/results/ros2-demo-new --timeout 3600
```

每次输出目录必须为空。`--timeout` 是整轮墙钟运行预算，不改变场景中的仿真时间限额、终点容差或零碰撞要求。是否验收通过以本轮 JSON／HTML 的实际证据为准，环境准备完成本身不代表导航通过。

如果移动了工作区，设置 `ROBOT_VM_RUNTIME` 为实际的 `work/ros2-runtime` 绝对路径；VM 中的源码挂载也须对应新的宿主目录。运行完成后可用该目录中的 `vm.sh stop rv` 停止 VM，保留磁盘及安装结果。

环境来源：[Lima 安装方式](https://lima-vm.io/docs/installation/)、[Lima VZ 驱动](https://lima-vm.io/docs/config/vmtype/vz/)、[ROS 2 Jazzy Ubuntu 安装](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)。镜像及 Lima 下载文件均已按发布元数据校验。

连续重启验证中，Fast DDS 路径曾出现 lifecycle 服务响应发送超时和 `rclpy` 创建节点阻塞。已保留部分报告与线程栈，并切换中间件做对照；这不等于已证明 Fast DDS 内部根因。选择方法遵循 [ROS 2 Jazzy 官方 RMW 说明](https://github.com/ros2/ros2_documentation/blob/jazzy/source/How-To-Guides/Working-with-multiple-RMW-implementations.rst)。需要对照时可使用 `RMW_IMPLEMENTATION=rmw_fastrtps_cpp ./deploy/vm-run.sh ...`；VM 入口只转发这一项显式覆盖，所有本轮节点继承相同实现，不操作全局 ROS daemon。

收尾已核对 268 个本轮进程组，无残留运行进程；VM 已停止以释放资源，磁盘和安装环境保留，`deploy/vm-run.sh` 会按需重新启动。见 [收尾记录](../results/ros2-environment/final-cleanup.json)。

本轮收尾核对 213 个进程组，无残留运行进程；VM 已停止释放资源，磁盘和环境保留，运行入口可按需重启。见 [收尾记录](../results/block-recovery-environment/final-cleanup.json)。
