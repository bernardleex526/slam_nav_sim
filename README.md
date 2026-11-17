# SLAM／导航验证插件

按 `slam-navigation-validation-plan.md` 第 1、2 步实现的首版工具。本次依照后续确认，交付 **工具、本地验证和 ROS 2 部署包**。它是可安装的 Python／pytest 插件，命令为 `robot-validate`；未注册到 Codex 插件市场。

**v0.1.3 已修复原临时阻挡恢复两项失败。** 锁定原 `ros2-demo-005` 生成地图，与参考地图的 8 个导航场景全部通过原门槛；实际受阻、移除、恢复及穿闸有独立时序证据。本轮是历史地图导航回归，未重新建图；另一次 fresh SLAM 退化仍保留失败。详见 [修复记录](docs/BLOCK_RECOVERY_FIX.md) 和 [图形汇总](results/block-recovery-index.html)。

最新完成度检查见 [PLUGIN_COMPLETION_AUDIT.md](PLUGIN_COMPLETION_AUDIT.md)。仓库包含源码、固定地图及文档直接链接的验收证据；本机 VM、缓存、编译目录和未链接的运行日志保留在本地。

## 已交付

- ROS 1／ROS 2 SQLite bag 读取、公开激光日志转换、传感器自身的点到线配准＋BreezySLAM 地图积分、PGM＋YAML 地图、TUM 轨迹；旧 RMHC 匹配器保留为对照。
- evo ATE／RPE、独立参考与双向配对覆盖检查、禁止尺度修正；地图覆盖、墙体双向距离与误判空闲比例。
- 逐阶段日志、独立结果目录、校验值、可追溯配置、超时／中断清理；只管理本轮启动的进程组。
- 单份离线 HTML（内嵌图片）、JSON、严格 JUnit XML；pytest 可运行同一用例契约。
- Ubuntu 24.04／ROS 2 Jazzy／Gazebo Harmonic Dockerfile，固定世界，SLAM Toolbox、Nav2、传感器适配和连续接触监测插件源码。
- ROS 仿真编排：建图导出 → 重新启动同世界 → 参考地图导航 → 本轮生成地图导航。加载地图时比较实际 `/map` 内容、分辨率和原点，并记录两次地图哈希。

## 在本机安装与试用

Python 3.9–3.12，需本地 C 编译器（macOS 的 Xcode Command Line Tools 或 Linux build-essential）。第三方 BreezySLAM 源码和公开日志已随附，Python 依赖安装需要网络。

```bash
cd robot-validation-plugin
./scripts/bootstrap.sh

# 公开真实激光日志 → ROS 1 bag → 局部配准＋BreezySLAM 地图 → 报告。
# 预期退出 2：该公开日志没有独立真值，不能宣布精度通过。
.venv/bin/robot-validate bag --scenario public --output results/public --open

# 本地射线仿真建图；不运行 Gazebo／Nav2。
# 预期退出 0：本地建图通过；导航／系统集成模块仍显示证据不足。
.venv/bin/robot-validate sim --backend local --scenario demo --output results/local --open

# 扫描断流故障，预期退出 1，报告保留约 8.1 秒缺口。
.venv/bin/robot-validate sim --backend local --fault scan-gap --output results/gap

# 可重现的旧匹配器失败对照（仍采用相同验收门槛）。
.venv/bin/robot-validate sim --backend local --config configs/local-rmhc.yaml --output results/legacy

# 工具代码和验收规则测试；这些测试通过不代表被测算法全部通过。
.venv/bin/python -m pytest -q --junitxml=results/tool-tests.xml

# 重跑全部本地对照并生成汇总 HTML；输出目录必须为空。
.venv/bin/python scripts/local_regression.py --output results/new-run
```

每次 `--output` 必须为空或不存在，防止把旧地图和新结果混在一起。省略时自动创建带时间戳的目录。可选故障：`scan-gap`、`odometry-bias`、`map-export`、`no-truth`。`--generated-map` 仅用于 ROS 2 的 `--navigation-only`，接受 trinary 8 位灰度 PGM；复制地图并记录来源哈希，不把历史建图标为本轮已执行。

退出码：`0` 已请求的必测项通过，`1` 验收失败，`2` 必测证据不足，`3` 环境错误，`124` 超时，`130` 用户中断。报告的三个模块分别判定；未执行模块保持“证据不足”。

## 在 Linux 部署 ROS 2 仿真

本机已使用隔离的 Ubuntu 24.04 arm64 VM 完成安装、C++ 接触插件编译、真实接触验证和闭环导航。Docker 镜像仍未构建；实际验证的是原生 Linux 部署路径。已有 VM 的入口如下，详细配置见 [本机环境](docs/ROS_VM_ENVIRONMENT.md)：

```bash
./deploy/vm-run.sh doctor
# 固定原生成地图，复跑此次修复的两图导航对照。
./deploy/vm-run.sh sim --navigation-only --scenario demo \
  --generated-map results/ros2-demo-005/map.yaml \
  --output /mnt/robot-validation/results/block-recovery-new --timeout 1800

# 重新建图并导航：质量仍按原阈值独立判定，可能暴露建图波动。
./deploy/vm-run.sh sim --scenario demo --config configs/demo.yaml \
  --output /mnt/robot-validation/results/ros2-demo-new --timeout 3600
```

在独立 Ubuntu 24.04 主机，可先执行 `bash deploy/setup-ubuntu.sh`，再用 `deploy/native-run.sh` 运行。

在已有 Docker 的 Linux／Linux VM 中，从插件根目录运行：

```bash
# 首次会构建环境，之后复用缓存；结果写入 results/docker/。
./deploy/run.sh

# 独立导航：不依赖 SLAM／地图导出成功。
./deploy/run.sh sim --navigation-only --scenario demo --output /results/nav-only

# 先用单个短场景验证环境，再跑完整矩阵。
./deploy/run.sh sim --scenario healthy --output /results/healthy

# 公开日志经 rosbag2 时钟回放，再运行 SLAM Toolbox。
./deploy/run.sh bag --backend ros2 --scenario public --output /results/public-ros2

# 回环关闭对照：配置开关有记录，候选／拒绝／接受事件仍需专门适配。
./deploy/run.sh sim --config /opt/robot-validation/configs/loop-off.yaml --output /results/loop-off
```

容器采用无界面软件渲染，世界不依赖在线 Fuel 模型。默认完整 `demo` 含正常到达、临时阻挡移除、不可达目标和窄通道路线；每种场景重启同一世界，保持种子、初始位姿和导航参数一致。`narrow` 使用固定途经点并核对独立实际轨迹确实穿过 0.8 米通道，避免绕行到达被当作通道验收。

接触插件遍历机器人全部碰撞体，在每个物理步收集接触，以 50 Hz 发布包含空帧的观测。验收要求监测到四个碰撞体、全程覆盖率至少 95%、最大观测间隔和末帧年龄均不超过 0.5 秒。原生 Gazebo 接触话题只在有接触时发布，因此不能直接据“零消息”判无碰撞。

Linux 已执行与待执行项目在 [部署检查单](docs/ROS_ACCEPTANCE.md)。若已在 Ubuntu 安装 ROS 2，可构建 `deploy/contact_monitor` 后设置 `GZ_SIM_SYSTEM_PLUGIN_PATH`，再运行原生命令 `robot-validate sim --scenario demo --open`。

## pytest 批跑与自己的算法

```bash
.venv/bin/python -m pytest -q --robot-case configs/public.case.json --junitxml=results/pytest-batch.xml
```

用例 JSON 声明运行请求、结果目录、期望退出码和关键判据。故障测试需要明确预期失败，不能默认把所有非零退出当成功。[接口约定](docs/ADAPTERS.md) 说明传感器、参考来源和坐标系要求。

ROS 算法可通过 `configs/custom-algorithm.example.yaml` 的 `algorithm.launch_argv` 接入。算法须使用 `/scan`、`odom→base_link`，发布 `/map` 和 `map→base_link` 变换；所有节点使用仿真时钟。当前录包适配器只覆盖 `LaserScan + TwistStamped` 轮速输入，其他话题／消息类型需要小型适配器，不能直接假定兼容。

## 结果解读

当前随附 `results/` 包含 macOS 本地建图证据和本机 Ubuntu VM 的 ROS 实跑证据；旧结果保留在 `results/history/v0.1.0/`。`results/index.html` 保留本地报告，`results/ros2-index.html` 为新的真实 ROS 导航汇总。原射线基线仍含 4% 轮速比例偏差和持续转角偏差。v0.1.1 只修改算法适配，未去掉这些噪声／偏差，也没有把真值或参考地图送入算法。精确前后指标在 `results/comparison.json` 和汇总 HTML 中列出。

默认本地位姿估计已从纯 RMHC 改为本项目实现的点到线局部配准；BreezySLAM 仍负责栅格积分。定位器只利用原始扫描、轮速先验和历史估计构成的子地图，使用相同的边缘束排除配置。原生三射线扩展改为每个观测束一条射线，修正概率／灰度换算及半格原点。没有扩大墙体宽度、缩小评分区域或拟合地图到参考图。

三倍轮速偏置现在能被激光匹配纠正，输出可通过原门槛；这不代表故障输入消失。断流和导出失败仍非零退出，缺真值仍不能验收 ATE。NaN、负无穷和越界读数不会生成空闲空间；只有明确的正无穷无返回值可以形成清空射线。

新本地配准入口只接受 `time_increment=0` 的瞬时或已去畸变扫描；有束间时间的扫描须预先去畸变或使用 ROS 后端。没有把原本未知的公开日志采样时序当作已校准数据。

默认门槛是示例门槛，尚未由业务误差容忍度或盲测锁定。不能将工具单元测试通过、轨迹通过、地图通过、导航通过视为同一件事。

第三方来源和许可证见 [THIRD_PARTY.md](THIRD_PARTY.md)。
