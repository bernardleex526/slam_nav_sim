# 机器人仿真接口规范 v0.1

> 本规范是整个仿真测试环境的**地基**。任何机器人模型、任何建图算法、任何导航配置，
> 只要 conform 这套契约，就能互相替换，而算法侧**零改动**。
>
> 落点对应需求 #1（任意 URDF 可投入仿真）与需求 #6（换机器人只改参数）。

---

## 1. 设计原则

1. **契约优先于实现**。算法包不关心机器人长什么样，只关心话题和 frame 是否存在。
2. **机器人是数据，不是代码**。换机器人 = 换一个 yaml + 一个 xacro 入口，不改 launch 逻辑。
3. **仿真与真机同构**。同一套 frame / 话题在 Gazebo 和真机上完全一致，bag 可以互放。
4. **可诊断**。里程计来源、传感器档位、地图难度都是显式开关，不进混。

---

## 2. Frame 契约（TF 树）

### 2.1 必需 frame

| Frame | 语义 | 谁发布 | 备注 |
|-------|------|--------|------|
| `map` | 全局固定坐标系 | SLAM / AMCL | 建图算法输出 |
| `odom` | 连续无跳变坐标系 | 里程计源 | GT 或 LIO |
| `base_footprint` | 机器人在**地面投影**的原点 | 里程计源 / robot_state_publisher | 必须贴地，z=0 |
| `base_link` | 机器人本体原点 | robot_state_publisher | 与 `base_footprint` 固定变换 |
| `imu_link` | IMU 测量原点 | robot_state_publisher | **必须与雷达 frame 重合或已知固定偏移** |
| `lidar_link` | 3D 雷达原点 | robot_state_publisher | MID360 内置 IMU 时 = `imu_link` |
| `lidar_stabilized` | 稳定云台 frame | `gimbal_stabilizer` 节点 | 可选，补偿躯干俯仰抖动 |
| `camera_link` | RGB-D 相机原点 | robot_state_publisher | D435 |
| `camera_optical_frame` | 相机光学坐标系 | robot_state_publisher | **z 朝前、x 朝右、y 朝下**，ROS 约定 |

### 2.2 坐标系方向约定

- 所有机器人本体 frame 遵守 **REP-103**：x 朝前、y 朝左、z 朝上。
- `base_footprint` 的 z 分量恒为 0（贴地投影），任意时刻 `base_footprint → base_link` 只允许
  z 方向平移（腿式平台车身高度变化时 z 可变，但不允许 roll/pitch 累积漂移）。
- 相机 optical frame **必须**遵守 ROS 相机约定（z 前、x 右、y 下），否则视觉 SLAM 会静默出错。

### 2.3 TF 发布责任划分

```
map ──(SLAM/AMCL)──> odom ──(里程计源)──> base_footprint ──(URDF)──> base_link ──(URDF)──> 所有传感器 frame
```

- `map → odom`：**只有**建图/定位算法发布。
- `odom → base_footprint`：**只有**里程计源发布（Gazebo 真值或 LIO 估计）。
- `base_footprint → 传感器`：**只有** `robot_state_publisher` 发布。

> ⚠️ 违反此划分会导致 TF 树出现双父节点，rviz2 报 "TF 多重父节点" 且无任何显式错误。

---

## 3. 话题契约

### 3.1 传感器输入（仿真 → 算法）

| 话题 | 类型 | 频率 | 说明 |
|------|------|------|------|
| `/points` | `sensor_msgs/PointCloud2` | 10 Hz | 3D 雷达原始点云（`lidar_link` 或 `lidar_stabilized`） |
| `/points_filtered` | `sensor_msgs/PointCloud2` | 10 Hz | 去畸变/去地面后的点云（可选中间产物） |
| `/scan` | `sensor_msgs/LaserScan` | 10 Hz | 由 3D 点云切出的虚拟 2D 扫描，给 2D SLAM 用 |
| `/imu` | `sensor_msgs/Imu` | 200 Hz | **必须有 `angular_velocity` 和 `linear_acceleration`，协方差字段不能全零** |
| `/camera/image_raw` | `sensor_msgs/Image` | 30 Hz | RGB |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | 30 Hz | 内参 |
| `/camera/depth/image_raw` | `sensor_msgs/Image` | 30 Hz | 深度（`32FC1`，单位 m） |
| `/camera/points` | `sensor_msgs/PointCloud2` | 30 Hz | 深度反投影点云 |

> 🚨 协方差全零是 LIO 类算法最常见的静默失效源：算法内部会用协方差加权，
> 全零会导致优化器认为 IMU 绝对可信，进而把点云权重压到 0。本环境的传感器宏
> **必须**填合理的协方差对角阵。

### 3.2 里程计与状态

| 话题 | 类型 | 说明 |
|------|------|------|
| `/odom` | `nav_msgs/Odometry` | 里程计输出，来源由 `odom_source` 决定（`gt` / `lio`） |
| `/ground_truth/odom` | `nav_msgs/Odometry` | **始终发布**的 Gazebo 真值，仅用于评测，算法不得消费 |
| `/joint_states` | `sensor_msgs/JointState` | 关节状态，供 `robot_state_publisher` 用 |

### 3.3 建图输出（算法 → 环境）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/map` | `nav_msgs/OccupancyGrid` | 2D 占据栅格（2D SLAM / 点云投影后） |
| `/map_cloud` | `sensor_msgs/PointCloud2` | 3D 点云地图（LIO 类算法输出的场地图） |
| `/map_cloud_downsampled` | `sensor_msgs/PointCloud2` | 降采样后的地图点云，供可视化对比稀疏度 |

> 统一约定：**2D 地图走 `/map`，3D 地图走 `/map_cloud`**。
> 3D LIO 如果只能出点云，由适配节点负责投影出 `/map`；2D SLAM 如果只能出栅格，
> `/map_cloud` 留空即可。

### 3.4 导航接口（Nav2）

| 话题 | 类型 | 说明 |
|------|------|------|
| `/cmd_vel` | `geometry_msgs/Twist` | 速度指令（Nav2 输出 → 仿真底盘输入） |
| `/plan` | `nav_msgs/Path` | 全局路径 |
| `/local_plan` | `nav_msgs/Path` | 局部路径 |
| `/goal_pose` | `geometry_msgs/PoseStamped` | 导航目标点 |
| `/navigate_to_pose/_action/status` | `action_msgs/GoalStatusArray` | 导航任务状态，评测用 |

### 3.5 评测专用话题

| 话题 | 类型 | 说明 |
|------|------|------|
| `/eval/gt_path` | `nav_msgs/Path` | 真值轨迹（从 `/ground_truth/odom` 累积） |
| `/eval/est_path` | `nav_msgs/Path` | 估计轨迹（从 `/odom` 累积） |
| `/eval/metrics` | `std_msgs/String`（JSON） | 实时指标，供 rviz2 文本显示 / 记录 |

---

## 4. 参数契约

### 4.1 机器人描述包必须提供的参数

每个机器人 profile 在 `robot_descriptions/config/robot_params.yaml` 下有一个命名空间：

```yaml
<robot_name>:
  description_package: robot_descriptions
  description_xacro: xacro/wheeled_quadruped/robot.urdf.xacro
  robot_type: wheeled_quadruped        # quadruped | wheeled_quadruped | agv_diff | agv_ackermann
  base_frame: base_footprint
  footprint:                           # 导航用，单位 m
    shape: polygon                     # polygon | circle
    radius: 0.32                       # circle 时使用
    vertices: "[[0.45,0.30],[0.45,-0.30],[-0.45,-0.30],[-0.45,0.30]]"  # polygon 时使用
  sensor_profile: lidar3d_imu          # 见 4.2
  max_vel: {x: 1.2, y: 0.0, theta: 1.8}
  acc_lim: {x: 1.5, y: 0.0, theta: 2.5}
  odom_source: gt                      # gt | lio
```

> **`footprint` 是导航参数适配的关键**。四足和轮足的外形差异直接决定 costmap 的
> 膨胀半径和局部规划器的可行性，改这里就能让 Nav2 参数跟着走。

### 4.2 传感器档位（`sensor_profile`）

| Profile | 传感器组合 | 目标算法 |
|---------|-----------|---------|
| `lidar3d_imu` | 3D 激光 + IMU | FAST-LIO2 / LIO-SAM / Point-LIO |
| `lidar3d_imu_cam` | 3D 激光 + IMU + RGB-D | 多传感器融合 / RTAB-Map |
| `mid360_imu` | MID360（非重复扫描）+ IMU | 贴近真机的 LIO 测试 |
| `mid360_imu_cam` | MID360 + IMU + RGB-D | 真机全配置 |
| `cam_only` | 仅 RGB-D | 纯视觉 SLAM（VIO） |

档位在 **xacro 编译期**决定模型里挂哪些传感器，在 **launch 运行期**由
`sim.launch.py` 的 `sensor_profile:=` 参数覆盖。

### 4.3 仿真侧必须提供的参数

| 参数 | 取值 | 说明 |
|------|------|------|
| `robot` | `quadruped` / `wheeled_quadruped` | 机器人 profile 名 |
| `world` | `simple` / `medium` / `complex` | 地图难度 |
| `mapping` | `fast_lio` / `lio_sam` / `rtabmap` / `none` | 建图算法 |
| `navigation` | `nav2` / `none` | 是否起导航栈 |
| `odom_source` | `gt` / `lio` | 里程计来源 |
| `global_planner` | `navfn` / `smac_2d` / `smac_hybrid` | 全局规划器 |
| `local_planner` | `dwb` / `rpp` / `mppi` / `teb` | 局部规划器 |
| `use_gimbal_stabilizer` | `true` / `false` | 稳定云台 |
| `headless` | `true` / `false` | 无头模式（批量跑测试用） |

---

## 5. 新机器人接入流程（Checklist）

1. 在 `robot_descriptions/xacro/<type>/` 下新增 `robot.urdf.xacro`。
2. **强制** include `common/sensors.xacro` 并调用 `sensors_profile` 宏，保证 frame 命名一致。
3. 在 `robot_descriptions/config/robot_params.yaml` 加一个 profile 段。
4. 在 `nav2_config/config/<robot>/` 下加一份 `nav2_params.yaml`。
5. 验证：
   ```bash
   ros2 launch robot_descriptions check_interface.launch.py robot:=<name>
   ```
   该 launch 会检查所有必需 frame 和话题是否存在，缺失项直接报错。

**不需要改任何算法代码、任何 launch 逻辑。** 这就是本规范存在的意义。
