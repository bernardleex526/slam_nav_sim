#!/usr/bin/env bash
# =============================================================================
# clean_sim.sh —— 仿真环境彻底清场
# -----------------------------------------------------------------------------
# 为什么需要这个脚本（血泪教训）：
#
#   Windows 侧的 TaskStop / taskkill 只能杀掉 WSL 里那层 `wsl.exe` 包装进程，
#   WSL 内部的 `ros2 launch` 进程树【会逃逸成活孤儿】。于是：
#
#     $ ros2 node list
#     /gimbal_stabilizer        ← ×3
#     /points_to_scan           ← ×2
#     /mapping_contract_adapter ← ×2
#     /metrics_collector        ← ×2
#     WARNING: nodes in the graph that share an exact name ...
#     $ ros2 topic info /map
#     Publisher count: 2        ← 两个 slam_toolbox 在抢发地图
#
#   后果（实测）：
#     · 三路 gimbal_stabilizer 抢发 lidar_stabilized TF → 雷达外参随机跳
#     · 两路 points_to_scan 抢发 /scan → slam_toolbox 收到交错的两套扫描
#     · 两个 slam_toolbox 抢发 map→odom 与 /map
#     ⇒ 建图结果不可复现、产生【场外幽灵墙】、探索行为时好时坏
#
#   这些"幽灵"节点不占 CPU 也不报错，只是安静地往话题里灌数据，
#   极难从日志发现。所以【每轮实验前必须跑这个脚本】。
#
# ⚠️ 2026-09-11 实测补充（这次翻车才发现的）：
#   用 `ps -eo pid,ppid,etimes,cmd | grep sim_ws/install` 一查，抓到 6 个
#   PPID=1 的孤儿，最老的已经活了 14.1 小时：
#
#       48815  14.1h  sim_joint_state_bridge
#       63738  12.6h  sim_joint_state_bridge
#       63944  12.6h  mapping_contract_adapter     ← 就是它污染 /map
#       64489  12.3h  sim_joint_state_bridge
#       64643  12.3h  mapping_contract_adapter     ← 还有它
#       38338  14.9h  dynamic_obstacle_driver
#
#   所以"孤儿清单"必须包含全部自研节点，尤其是：
#     mapping_contract_adapter / sim_joint_state_bridge / dynamic_obstacle_driver
#   旧版本漏了这几个 → 清场永远"看着干净"，但串扰一直在。
#
#   另外：FastDDS 的共享内存段在进程被 -9 杀掉时不会自然回收，
#   残留段会让新起的节点读到过期端点信息。一并清掉。
#
# 用法：
#     bash tools/clean_sim.sh          # 清场 + 复核
#     bash tools/clean_sim.sh --soft   # 只 SIGTERM，等优雅退出（调试用）
# =============================================================================

SOFT=0
[ "${1:-}" = "--soft" ] && SOFT=1
if [ "$SOFT" = "1" ]; then
    KILLFLAG="-15"
    echo "[clean_sim] 软清场模式（SIGTERM）"
else
    KILLFLAG="-9"
fi

# ---------------------------------------------------------------------------
# 待清进程清单。
#
# ⚠️ 每个模式都用 `pkill -f` 全命令行匹配，且必须写【足够specific】的字符串：
#    写 "nav2" 会误伤用户自己开的编辑器/浏览器（命令行里带 nav2 字样）；
#    写 "mapping" 会误伤 clean_sim 自己。所以按【可执行文件名】写。
# ---------------------------------------------------------------------------
PATTERNS=(
    "ros2 launch sim_bringup"
    "ros2 launch mapping_adapters"
    "ros2 launch robot_descriptions"
    "ros2 launch sim_worlds"
    "ros2 launch nav2_config"
    "ros2 launch sim_metrics"
    "gzserver"
    "gzclient"
    "spawn_entity"
    "slam_toolbox_node"
    "exploration_runner"
    "navigation_runner"
    "stage_a_runner"
    "stage_b_runner"
    "benchmark_runner"
    "robot_state_publisher"
    "gimbal_stabilizer"
    "points_to_scan"
    "cloud_downsampler"
    "mapping_contract_adapter"
    "metrics_collector"
    "sim_joint_state_bridge"
    "dynamic_obstacle_driver"
    "static_transform_publisher"
    "amcl"
    "controller_server"
    "planner_server"
    "bt_navigator"
    "behavior_server"
    "waypoint_follower"
    "velocity_smoother"
    "map_server"
    "lifecycle_manager"
    "nav2_"
    "ros2 run sim_benchmark"
    "ros2 run sim_metrics"
    "ros2 run sim_worlds"
    "ros2 run robot_descriptions"
    "ros2 run mapping_adapters"
)

echo "[clean_sim] 1/4 杀 ROS / Gazebo 进程树（$(( ${#PATTERNS[@]} )) 类模式）..."
for p in "${PATTERNS[@]}"; do
    pkill $KILLFLAG -f "$p" 2>/dev/null
done

# 兜底：任何从 sim_ws/install 里跑出来的自研节点（覆盖上面漏网的）
pkill $KILLFLAG -f "sim_ws/install/" 2>/dev/null

echo "[clean_sim] 2/4 清 FastDDS 共享内存 ..."
rm -rf /dev/shm/fastrtps_* 2>/dev/null
rm -rf /dev/shm/fastdds_*  2>/dev/null
rm -rf /dev/shm/sem.fastrtps_* 2>/dev/null

echo "[clean_sim] 3/4 等待进程退出 ..."
sleep 3

echo "[clean_sim] 4/4 复核 ..."
# ⚠️ 计数陷阱：pgrep -f <pat> 会把【本脚本自己的命令行】也匹配进去
#    （因为命令行里就写着这些关键词），于是永远显示"还有残留"，
#    让人误以为清场失败、反复重跑。必须用 ps 排除掉自己和 grep。
#
# ⚠️ 另一个更隐蔽的陷阱：复核用的 —— 模式 —— 必须和上面 kill 的一致，
#    否则会出现"杀了但没查"（看着干净，其实漏网）或"查了但没杀"。
#    所以这里直接复用同一个 PATTERNS 数组。
left=0
for pat in "${PATTERNS[@]}"; do
    n=$(ps -eo pid=,cmd= 2>/dev/null \
        | grep -F "$pat" \
        | grep -v grep \
        | grep -v "clean_sim.sh" \
        | grep -v "ps -eo" \
        | wc -l)
    if [ "$n" -gt 0 ]; then
        echo "  ⚠️  仍有 $n 个匹配 '$pat' 的进程："
        ps -eo pid=,ppid=,etimes=,cmd= 2>/dev/null | grep -F "$pat" | grep -v grep \
            | grep -v "clean_sim.sh" | head -3 | sed 's/^/       /'
        left=$((left + n))
    fi
done

# 最后一层：直接扫 sim_ws/install 路径，能抓到所有自研节点（最重要的一道网）
n_inst=$(ps -eo pid=,cmd= 2>/dev/null \
    | grep -F "sim_ws/install/" \
    | grep -v grep | grep -v "clean_sim.sh" | wc -l)
if [ "$n_inst" -gt 0 ]; then
    echo "  ⚠️  仍有 $n_inst 个 sim_ws/install 下的自研节点存活："
    ps -eo pid=,ppid=,etimes=,cmd= 2>/dev/null | grep -F "sim_ws/install/" \
        | grep -v grep | grep -v "clean_sim.sh" | head -8 | sed 's/^/       /'
    left=$((left + n_inst))
fi

# 报告孤儿（PPID=1 说明父进程已死、已被 init 收养 —— 这类最危险）
n_orph=$(ps -eo ppid=,cmd= 2>/dev/null \
    | awk '$1==1' | grep -F "sim_ws/install/" | grep -v grep \
    | grep -v "clean_sim.sh" | wc -l)
if [ "$n_orph" -gt 0 ]; then
    echo "  ❗ 其中 $n_orph 个是 PPID=1 的孤儿进程（父进程已死，最隐蔽）："
    ps -eo pid=,ppid=,etimes=,cmd= 2>/dev/null | awk '$2==1' \
        | grep -F "sim_ws/install/" | grep -v grep | head -8 | sed 's/^/       /'
fi

if [ "$left" -eq 0 ]; then
    echo "[clean_sim] ✅ 环境干净，可以开始实验"
else
    echo "[clean_sim] ❌ 仍有 $left 个残留，请手动检查"
    exit 1
fi
