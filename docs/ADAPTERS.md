# 适配接口与判定约定

## 数据

本地包输入要求 `sensor_msgs/msg/LaserScan` 和 `geometry_msgs/msg/TwistStamped`。默认话题 `/scan`、`/wheel_twist`；本地后端可配置名称。扫描必须角度居中，束数、视场角须与 `bag.laser` 一致；其他格式明确拒绝。轮速每个样本积分到下一次扫描，扫描掉帧期间的轮速不会丢弃。

ROS 包回放当前固定白名单 `/scan /wheel_twist`，排除旧地图、估计里程计和 TF。自有 ROS 包需先适配至这两个话题；不支持仅改本地 `bag.scan_topic` 就改变 ROS 回放。回放开始前检查 SLAM 扫描订阅和轮速适配订阅；回放时启用 `/clock`，结束后等待 `ros.settle_s` 再导出。

自定义消息、QoS、外参和传感器时延必须在接入时验证。当前无自动时间同步估计、外参标定、RTK 或双目适配。

## 算法

ROS 配置 `algorithm.launch_argv` 是命令参数数组，不通过 shell 展开。待测节点遵守 `/map`（OccupancyGrid，可供 transient-local 订阅）、`map→odom→base_link` TF、激光 frame `laser` 和 `use_sim_time=true`。仿真标准激光外参是 base_link 上方 0.16 米；随附公开日志的前向外参为 0.145 米。

采集器读取 TF 估计轨迹。仿真真值来自独立 Gazebo OdometryPublisher，经 `/ground_truth` 只进入评估节点，不发布到算法使用的 TF／里程计。建图路线控制使用 SLAM 输出的地图坐标位姿，修正轮速转向误差；估计超过 0.5 秒未更新即停止。固定路线及速度不变，原始轮速另存为 `wheel-odometry.tum`。

当前算法导出接口是 OccupancyGrid 和 TF；其他接口需实现适配器。原始点云、内部关键帧和回环事件暂未接入。

## 真值与地图

参考轨迹采用 TUM 格式，时间戳严格递增，四元数为单位长度。配置 `reference.trajectory` 为执行主机可读路径，`reference.independent: true` 只能用于与算法融合输入独立的参考。缺失、非独立、覆盖不足或对齐退化都显示“证据不足”。双向配对覆盖均至少 95%，时间差上限 0.03 秒；米制尺度不做修正。

ROS 仿真保留 50 Hz 原始独立真值，以估计采样器的 10 Hz 时钟相位生成规则参考网格，再由原始真值插值。参考时段覆盖完整采集区间（沿用首尾各 0.2 秒边缘排除），不会截到估计轨迹的开始／结束时刻。真值相邻样本间隔超过原 0.03 秒配对容差时不插值，并独立检查参考可用率。此步骤不移动任何时钟、不根据位姿误差选时间偏移，元数据记录于 `metrics.reference_sampling`。

地图使用 ROS PGM／YAML 语义：未知 −1、空闲 0、障碍 100。PGM 行朝下，内部栅格 Y 朝上；写入／读取时显式翻转。地图比较在声明的原点、旋转和分辨率下采样，不拟合地图变换。轨迹 SE(3) 对齐不会偷偷用于地图或导航，从而避免掩盖同世界坐标偏差。

覆盖分母为参考的固定可观察区域；模拟实体内部标未知。墙体 P95 为双向最近邻距离合并分布的 95 分位；误判空闲比例以参考障碍为分母，未知不计作空闲。该实现是可测试的首版基础评分，不声称等价于 nav_explore_eval。

## 验收结果

`result.json` 的 `checks` 包括 `id / section / status / value / threshold / reason / window_s / evidence / required`。四种状态为 `pass / fail / insufficient / environment_error`。整体状态仅聚合已请求的必测项；每个模块另外判定。必测证据不足在 JUnit 中作为失败，非必测未实现项为 skipped。

导航成功同时依赖动作结果、独立终点位置和朝向、接触观测完整性、时限。不可达目标需要 abort/cancel 后实际停止；运行器超时强制取消记为 timeout，不能冒充受控失败。临时阻挡要求记录受阻、移除和恢复移动。

发送导航目标前，除 TF 和动作端点外，还等待 AMCL、控制器、规划器、行为树导航器、速度平滑器和碰撞监测器的 lifecycle 状态为 active；每个场景的 `readiness.json` 保存观测时间，避免端点已存在但节点仍 inactive 时被拒绝。随后继续处理回调 1.5 秒以等待 DDS 端点发现稳定，并确认接触和碰撞体计数新鲜后才开始目标执行窗。原 0.5 秒观测缺口门槛不变；动作调用的同步耗时存为 `action_request_timing`，不等于目标确认总耗时。每阶段另存 `contact-telemetry.json`，便于复算覆盖和最大间隔。

Nav2 行为树动作请求的确认等待为 200 毫秒，适配本机软件渲染 VM；这是内部通信参数，场景的 60／120 秒时限保持不变。两张地图使用同一份 Nav2 参数并记录哈希。`narrow` 在原最终目标前增加两个固定途经点，整条路线共用一个 120 秒预算和连续碰撞观测，另外要求独立实际轨迹连续从南向北穿过 x=(5.15, 5.95)、y=[2.0, 3.2] 的通道。绕行到达不能代替通道覆盖。

每轮任务新建结果目录和独立进程组。ROS 使用进程锁分配的 domain 以及唯一 Gazebo partition；只在隔离容器／专用测试环境使用。该 domain 分配只能避免本工具实例相互冲突，不能检测其他程序自行占用同一 domain。


## v0.1.1 本地匹配与栅格修复

默认 `algorithm.local_matcher: point_to_line` 使用本项目的传感器自身局部配准，随后调用 BreezySLAM 的 native 地图积分；`rmhc` 可用于旧匹配器对照。局部配准从历史估计扫描构造子地图，不接受参考地图或真值。逐帧状态、对应点数、残差和轮速先验修正量见 `mapping-diagnostics.json`。

定位与地图积分均遵守 `laser.margin`。新本地匹配器要求扫描 `time_increment=0`；非零束间时间会被保留并明确拒绝，须先去畸变或改用 ROS 后端。NaN／负无穷／越界测量不生成自由空间，正无穷才是显式无返回清空射线。

本地地图像素分类按 YAML 已声明的 0.65／0.196 概率门槛换算，原点计入 native 像素中心与 ROS 左下角之间的半格差。射线 span 为 1，避免在深度边界扩展出未观测射线。所有验收阈值和评分范围保持 v0.1.0 不变。

## v0.1.3 临时闸门与固定地图回归

临时障碍为中心 `(3,2.5)`、尺寸 `0.5×9.2×1.5 m` 的闸门。起步时隐藏于地面下；AMCL 估计到达插入区后出现，真实激光触发 Nav2 `FrontHold`。调度等待轮速低于原 0.05 m/s 并连续停留 3 秒后移除；不使用真值调度、不直接发送停车命令。独立真值在完成后核验至少 2 秒持续受阻、移除后的新鲜前进速度与 0.1 米位移，以及实际穿闸。`motion.json` 保留时间、位置、朝向和速度，`collision-monitor.json` 保留真实防护状态。

`FrontHold` 限制平移、保留纯角速度；标准 RotationShim 包装原 MPPI，末端采用零线速度转向，只在新目标做起始对齐。原 `FootprintApproach` 保留。Nav2 内部位置容差仍 0.25 m，内部朝向目标收紧至 0.15 rad；独立验收位置／朝向仍为原 0.25 m／0.25 rad。两图共用相同配置，实际节点参数另存 `controller-configuration.json`。

`sim --navigation-only --generated-map PATH` 用已有生成地图与参考地图做相同导航对照，不执行 SLAM，也不重新判定来源建图。仅支持 trinary 的 8 位灰度 PGM；raw、scale、alpha 等语义不支持的输入会被拒绝。源文件不修改，复制后检查几何和全部栅格，来源与输出哈希及 `mapping_executed_this_run=false` 写入 provenance。pytest 契约中的生成地图路径相对契约文件解析。
