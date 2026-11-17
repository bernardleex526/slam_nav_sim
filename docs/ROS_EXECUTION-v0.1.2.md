# ROS 2／Gazebo 闭环导航实跑报告

2026-11-16，Asia/Shanghai。插件版本 v0.1.2。

已完成 Ubuntu VM、ROS 2 Jazzy、Gazebo Harmonic 和接触监测插件的准备，并完成三轮完整参考／生成地图闭环导航对照。本轮最终验收结论为 **失败**：建图通过、参考地图导航失败、生成地图集成失败。保留所有失败结果，没有降低原验收阈值。

主要证据：[本轮完整 HTML](../results/ros2-demo-005/report.html)、[导航对照 JSON](../results/ros2-demo-005/navigation-comparison.json)、[JUnit](../results/ros2-demo-005/junit.xml)、[汇总 HTML](../results/ros2-index-v0.1.2.html)。

## 本轮地图质量

| 指标 | 实测 | 原门槛 | 结论 |
|---|---|---|---|
| ATE RMSE | 0.056 米 | ≤ 0.75 米 | 通过 |
| 地图覆盖率 | 95.228 % | ≥ 70 % | 通过 |
| 误判空闲 | 9.880 % | ≤ 10 % | 通过 |
| 墙体 P95 | 0.160 米 | ≤ 0.5 米 | 通过 |

原配置、默认阈值与评分函数三个文件 SHA256 均未变化，见 [不变性检查](../results/ros2-environment/acceptance-integrity.json)。轨迹双向配对覆盖至少 95%，配对时间差上限 0.03 秒；地图采用声明原点，不拟合地图到参考图。

## 相同条件下的导航对照

| 场景 | 地图 | 结论 | 动作状态 | 位置误差 m | 朝向误差 rad | 接触次数 | 用时 s | 补充证据 |
|---|---|---|---|---|---|---|---|---|
| 正常到达 | 参考 | 通过 | succeeded | 0.117 | 0.052 | 0 | 17.74 |  |
| 正常到达 | 生成 | 通过 | succeeded | 0.036 | 0.014 | 0 | 17.84 |  |
| 临时阻挡恢复 | 参考 | 失败 | succeeded | 0.147 | 0.096 | 0 | 40.68 | 受阻／移除／恢复：否/是/是 |
| 临时阻挡恢复 | 生成 | 失败 | succeeded | 0.135 | 0.117 | 0 | 40.99 | 受阻／移除／恢复：否/是/是 |
| 不可达目标 | 参考 | 通过 | aborted | — | — | 0 | 18.88 | 停止速度 0.00000 m/s |
| 不可达目标 | 生成 | 通过 | aborted | — | — | 0 | 18.84 | 停止速度 0.00000 m/s |
| 窄通道 | 参考 | 通过 | succeeded | 0.104 | 0.130 | 0 | 33.55 | 实际穿越：通过 |
| 窄通道 | 生成 | 通过 | succeeded | 0.027 | 0.114 | 0 | 34.67 | 实际穿越：通过 |

终点位置 ≤0.25 m、朝向 ≤0.25 rad，接触次数必须为 0。不可达目标须自行 abort/cancel 并停止（≤0.02 m/s），限时 60 s；其余场景限时 120 s。临时阻挡须同时记录受阻、移除及恢复移动，单纯绕行到达不能通过。窄通道在原最终目标前加入固定途经点，共用原 120 s 预算，并独立检查实际穿越；旧候选路线的到达结果不能证明窄通道覆盖。

每个阶段都重启同一世界、核对初始位姿、等待实际 `/map` 栅格内容／分辨率／原点与请求文件一致，随后才发送目标。参考／生成地图使用相同种子、机器人和 Nav2 参数；生成地图就是本轮现场建图输出。

- 世界 SHA256：`79ffaf4b39b349a9a6bcc47cb198387c5098f6a470431a4a3294c21e348e8290`。
- Nav2 参数 SHA256：`906200f686cd9e93e09b38592d4c8929b23312fac7f485c06660a83f4a597898`。
- 种子：`9999`；Nav2 内部随机过程不保证逐字节确定性。
- 生成地图 PGM SHA256：`ac287241f03a8dfd04ec2b5b98c8b6584fa2539bcd42c417ad72924b2d37fbd8`。
- 接触流覆盖和四个碰撞体观测完整性见各场景 `navigation.json`；本轮原始时间序列已[独立复算核对](../results/ros2-telemetry-audit.json)。零碰撞来自连续空帧观测，不是零消息推断。

## 环境与已修复问题

实际环境为 Lima 2.3.0／Ubuntu 24.04 arm64，4 核、6 GiB RAM、40 GiB 稀疏磁盘；ROS 2 Jazzy，Gazebo 8.15.0，Nav2 1.3.13，SLAM Toolbox 2.8.5，Cyclone DDS 0.10.5／RMW 2.2.4，Python 3.12.3。宿主机只挂载插件目录。安装脚本、运行入口与 [环境说明](ROS_VM_ENVIRONMENT.md) 已保存；[apt 清单](../results/ros2-environment/apt-resolved.txt) 和 [Python 清单](../results/ros2-environment/python-resolved.txt) 记录实际版本。Docker 镜像仍未构建，本次实际执行方式是原生 Linux VM。

连续接触插件已编译并实测：基线监测四个碰撞体、零非地板接触；两次障碍注入均检出接触（初轮 80 条记录，Cyclone 复验 48 条），见 [初轮验证](../results/ros2-contact-sanity-001/contact-sanity.json) 和 [Cyclone 复验](../results/ros2-contact-sanity-002/contact-sanity.json)。

本轮运行暴露并修复了以下问题：

1. 轮速转向低估导致固定建图路线撞墙。独立诊断记录轮速角度 1.5582 rad、真值 1.7390 rad。路线改用 SLAM 的地图坐标估计反馈，并在估计过期时停止；固定路线和速度不变，真值只用于评估。
2. Nav2 Jazzy 启动表达式需要 `True/False` 布尔字面量；修正启动参数大小写。
3. 动作端点出现早于 lifecycle active；增加生命周期就绪等待，逐场景保存 `readiness.json`。
4. 原真值 10 Hz 降采样与估计采样相位错开约 40 ms，错误触发 30 ms 配对容差。改为以估计采样器相位建立规则参考网格、从独立 50 Hz 真值插值；保留完整真值时段、检查缺口，不移动时钟或拟合位姿。
5. VM 中 Nav2 行为树动作确认曾超过上游默认 20 ms，导致提前 abort。将内部确认等待改为 200 ms，两张地图一致；60／120 s 验收时限不变。
6. Fast DDS 环境在连续重启中出现服务响应发送超时和节点构造阻塞。切换部署默认中间件为 `rmw_cyclonedds_cpp`，所有节点一致，实际实现写入 provenance；原失败完整保留。
7. Cyclone 首次动作请求附近出现约 0.9 秒真值／接触接收缺口，证据支持与端点发现未稳定有关，原标准判证据不足。增加目标发出前的 1.5 秒回调处理及新鲜接触流检查，不截短目标执行观察窗、不补齐缺失数据；另存动作调用耗时和原始接触时间序列。
8. 原 narrow 最终目标允许绕行，第一轮两条轨迹都未进入通道。新增固定途经点和连续穿越检查，避免把普通到达当作窄通道通过。

## 保留的历史与限制

| 运行 | 实际观察 |
|---|---|
| `ros2-healthy-001` | 原轮速路线撞墙，建图中止；原始事件保留 |
| `ros2-healthy-002` | 地图三项通过；旧参考采样导致 ATE 证据不足，随后 Nav2 布尔启动失败 |
| `ros2-healthy-003` | ATE 通过，误判空闲 27.42% 失败；随后遇到 lifecycle inactive 拒绝目标 |
| `ros2-nav-only-001` | 独立参考地图健康导航通过，位置误差 0.063 m、朝向 0.063 rad、零接触 |
| `ros2-demo-001` | 首轮 8 个导航阶段全部执行；地图误判空闲 10.39% 失败，临时阻挡恢复未通过；narrow 仅到达，实际未穿越 |
| `ros2-demo-002` | 地图通过、参考窄通道穿越通过；生成地图临时阻挡阶段尚未发送目标即因 DDS lifecycle 响应丢失而中断，部分报告保留 |
| `ros2-demo-003` | 未并发其他测试，仍在第七个导航阶段创建节点时卡住；线程栈定位至 rclpy Node 构造，诊断中断并保留退出 130 报告 |
| `ros2-demo-004` | Cyclone DDS 完整跑完 8 阶段；参考不可达／窄通道因初次动作请求附近约 1 秒观测缺口判证据不足 |
| `ros2-discovery-check-001` | 增加发送目标前的发现稳定等待，短场景通过；最大观测间隔 0.02 s |
| `ros2-demo-005` | 本报告主结果，8 个导航阶段及实际窄通道覆盖检查全部执行 |

[第一轮完整报告](../results/ros2-demo-001/report.html) 与其他阶段失败报告均保留。第二轮遇到 DDS 服务响应发送超时；第三轮未并发其他测试，仍在创建 ROS 节点时卡住，见 [线程栈](../results/ros2-environment/demo-003-stack.txt)。第四轮统一采用 Cyclone DDS 后完整运行，但初次动作请求附近的观测缺口仍被严格阻断；增加发送目标前 1.5 s 的发现稳定等待和接触流新鲜检查后，短场景最大间隔为 0.02 s，第五轮再跑完整矩阵。采用 Cyclone DDS 是经本轮实测的环境规避措施，不声称已确定 Fast DDS 的内部根因或证明长期稳定。相同固定场景多次建图质量已有波动，不能从一次通过推断稳定达标；本地 v0.1.1 配准器与 ROS SLAM Toolbox 是不同实现，本地通过不保证 ROS 地图通过。当前失败项：`REFERENCE_TEMPORARY-BLOCK, GENERATED_TEMPORARY-BLOCK`。

本轮没有完成 ROS 故障注入全矩阵、多种子统计、ROS 公开包回放、回环内部事件归因或真机验收。原计划中第三方工具试接、独立真值公开包、三维和人工提效等工作仍未完成，不能宣布原计划整体完成。

## 复跑

从插件根目录执行：

```bash
./deploy/vm-run.sh doctor
./deploy/vm-run.sh sim --scenario demo --config configs/demo.yaml \
  --output /mnt/robot-validation/results/ros2-demo-new --timeout 3600
```

输出目录必须为空。VM 停止时运行入口会先启动；3600 秒仅是整轮墙钟预算。单独参考地图入口为 `sim --navigation-only --scenario healthy`。严格健康验收契约为 `configs/ros-healthy.case.json`；该契约要求整轮通过，不把算法失败当成功。工具回归和浏览器验证结果见 [验证记录](VERIFICATION.md)。

收尾已核对 268 个本轮进程组，无残留运行进程；VM 已停止以释放资源，磁盘和安装环境保留，`deploy/vm-run.sh` 会按需重新启动。见 [收尾记录](../results/ros2-environment/final-cleanup.json)。
