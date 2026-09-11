# 可交付范围与复验说明

## 当前实现

- WSL Ubuntu 22.04、ROS 2 Humble、Gazebo Classic。
- 模型入口：`wheeled_quadruped`、`quadruped`、宇树开源 `go2w`、独立通用 `agv`。
- 理想平面运动、激光/IMU/RGB-D、真实 Gazebo 接触事件、动态障碍。
- SLAM Toolbox 建图；GT 辅助确定性安全覆盖路径；原图保留、质量评分、选图。
- AMCL 固定地图定位；NavFn/SMAC 2D/SMAC Hybrid 与 DWB/MPPI/RPP 组合对比。
- 固定目标、冻结地图哈希、完整目标状态、轨迹、ATE/RPE、接触次数、控制平滑度和路径效率。
- 每轮状态留档；失败停止；原子落盘；分批全量汇总；机器可执行验收。

## 必须保留的边界

1. 当前为理想平面机器人算法测试平台，不是步态、足地接触、轮胎打滑或真实 M20 Pro 动力学仿真器。
2. Go2W 使用开源模型；AGV 为自建通用模型。没有 M20 Pro 厂商 URDF，不能把近似模型标成原厂型号。
3. FAST-LIO、LIO-SAM、RTAB-Map 只保留适配扩展入口，未包含在本次算法实装验收中。缺包会明确失败。
4. 建图使用 GT 辅助覆盖路径，不声称未知环境自主探索。复杂环境的窄迷宫可能超出允许原地转向的保守工作范围。
5. `verify_delivery.py` 分开报告安全工作范围验收与全环境覆盖，二者不可互换。全环境覆盖失败仍会显式保留。
6. 实验每组一个 repeat、五个连续目标，只能作这次固定测试任务的比较。失败可能影响后续起点，不提供统计显著性结论。
7. 平台功能验收不要求所有算法目标全部成功；算法失败和接触事件必须如实计入结果，而不是删掉。

## 复验

先在 WSL 中进入本 `sim_ws`。source ROS 脚本前不要启用 `set -u`。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
python3 -m unittest discover -s tools -p 'test_*.py'
```

从头执行正式矩阵，时长可能超过一小时：

```bash
bash tools/run_delivery.sh
```

已有产物只做验收，不重跑：

```bash
python3 tools/resummarize_stage_a.py --config src/sim_benchmark/config/delivery_stage_a.yaml
python3 tools/verify_delivery.py
python3 tools/build_delivery_report.py --output /absolute/path/to/delivery
```

正式结果位于 `~/.ros/sim_delivery_stage_a` 与 `~/.ros/sim_delivery_stage_b`。
短程模型接口与接触验收位于 `~/.ros/sim_delivery_checks`。

单独使用模型：

```bash
ros2 launch sim_bringup sim.launch.py robot:=agv world:=simple headless:=true use_gimbal:=false
```

批量导航统一使用 `rmw_cyclonedds_cpp`，规避本机 Fast DDS 生命周期服务偶发丢响应问题。
并行仿真必须同时隔离 `ROS_DOMAIN_ID` 和 `GAZEBO_MASTER_URI`。不要在其他实验运行时执行旧 `clean_sim.sh`。

## 地图验收口径

- 所有声明的覆盖路点必须走完，timeout、跳点、终止不能冒充 done。
- 原始地图不裁剪；场外占据比例小于 0.3，10cm 容差精确率至少 0.7。
- 以机器人 footprint 外接圆加 0.035m 余量，定义可原地转向的保守连通工作区；该区域中心格观测覆盖至少 95%。
- 同时独立报告整个 GT 自由空间覆盖是否达到 85%。未达到时必须显示“全环境覆盖未通过”，不能删除或隐藏窄通道。
- 工作区定义是保守运行约束，不是严格 SE(2) 可达性证明。更窄区域的通过性需要不同姿态空间规划器和单独验收。

## 导航验收口径

- 8 组配置各 5 个目标，共 40 次尝试；状态与采集器目标计数一致。
- 同一世界和 repeat 的目标序列、地图像素哈希、机器人与 DDS 实现一致。
- 无基础设施启动错误；关键指标落盘；每个世界至少存在成功导航证据。
- 显式报告失败状态、持续接触次数和耗时，不将较短的失败耗时解释为更好的导航性能。
- action 成功还需真值到达误差不超过 0.5m；独立导出每个目标结束时的 GT 误差。
