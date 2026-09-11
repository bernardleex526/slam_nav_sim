#!/usr/bin/env bash
# =============================================================================
# smoke_sim.sh —— 短时仿真存活体检（5 分钟）
# -----------------------------------------------------------------------------
# 动机（2026-09-11 翻车）：
#   上一轮 simple 实验跑了 20 分钟，日志里一切正常，但最后发现：
#     · gzserver 在启动约 50s 后就【无声消失】了
#     · 仿真节点（slam_toolbox 等）还活着，但拿不到任何传感器数据
#     · exploration_runner 因为 /odom 停更而空转 20 分钟
#     · 最终既没有地图、也没有报错 —— 典型的"静默失败"
#   而 launch.log 里【没有】"process has died"，说明 launch 也没察觉。
#
#   所以：不要再拿 20 分钟的长实验去赌。先用这个 5 分钟的体检，
#   专门盯三件事：
#     ① gzserver 是否全程存活（每 10s 采样一次 PID）
#     ② 关键传感器话题是否在持续更新（/points /scan /odom 频率）
#     ③ 自研节点是否出现重名（串扰排查）
#   三项全过，再跑正式实验。
#
# 用法：
#     bash tools/smoke_sim.sh            # 默认 simple，300s
#     bash tools/smoke_sim.sh medium 420
# =============================================================================

WORLD="${1:-simple}"
DURATION="${2:-300}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="/tmp/smoke_${WORLD}_$(date +%H%M%S)"
mkdir -p "$OUT"

echo "=========================================================="
echo "[smoke] 环境=$WORLD  时长=${DURATION}s  输出=$OUT"
echo "=========================================================="

cd "$WS" || exit 1
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash >/dev/null 2>&1
# shellcheck disable=SC1091
source install/setup.bash >/dev/null 2>&1

cleanup() {
    echo "[smoke] 收尾：杀进程组 ..."
    [ -n "${SIM_PID:-}" ] && kill -INT -"$SIM_PID" 2>/dev/null
    sleep 3
    pkill -9 -f "ros2 launch sim_bringup" 2>/dev/null
    pkill -9 -f gzserver 2>/dev/null
    pkill -9 -f sim_ws/install 2>/dev/null
    sleep 1
    echo "[smoke] 已清场"
}
trap cleanup EXIT

echo "[smoke] 启动仿真（navigation:=none, mapping:=slam_toolbox, 机器人静止）..."
setsid ros2 launch sim_bringup sim.launch.py \
    robot:=wheeled_quadruped world:="$WORLD" \
    mapping:=slam_toolbox navigation:=none \
    odom_source:=gt use_gimbal:=true headless:=true \
    metrics:=true run_name:="smoke_${WORLD}" viz:=false \
    > "$OUT/sim.log" 2>&1 &
SIM_PID=$!
echo "[smoke] launch PID=$SIM_PID"

# ---- 等 gzserver 起来 ----
for _ in $(seq 1 60); do
    if pgrep -f "gzserver" >/dev/null 2>&1; then break; fi
    sleep 2
done
sleep 20   # 让 spawn + 传感器稳定

echo "[smoke] 开始存活监测 ..."
echo "elapsed,gzserver_alive,n_ros2launch,n_self_nodes" > "$OUT/liveness.csv"
GZ_GONE_AT=""
T0=$(date +%s)
while true; do
    EL=$(( $(date +%s) - T0 ))
    [ "$EL" -ge "$DURATION" ] && break

    GZ=$(pgrep -c -f "gzserver" 2>/dev/null || echo 0)
    LAU=$(pgrep -c -f "ros2 launch sim_bringup" 2>/dev/null || echo 0)
    SELF=$(ps -eo cmd= 2>/dev/null | grep -c "sim_ws/install/" || echo 0)
    echo "$EL,$GZ,$LAU,$SELF" >> "$OUT/liveness.csv"

    # 第一次发现 gzserver 没了就记下来（这是关键证据）
    if [ "$GZ" -eq 0 ] && [ -z "$GZ_GONE_AT" ]; then
        GZ_GONE_AT="$EL"
        echo "  ❌ [${EL}s] gzserver 消失了！"
        echo "     最后 15 行 sim.log："
        tail -15 "$OUT/sim.log" | sed 's/^/       /'
    fi
    # 每 60s 报一次
    if [ $((EL % 60)) -lt 5 ] && [ "$EL" -gt 0 ]; then
        echo "  [${EL}s] gzserver=$GZ  ros2launch=$LAU  自研节点=$SELF"
    fi
    sleep 5
done

echo ""
echo "=========================================================="
echo "[smoke] 结果汇总"
echo "=========================================================="

# ---- ① gzserver 存活 ----
GZ_MIN=$(awk -F, 'NR>1 && $2==0' "$OUT/liveness.csv" | wc -l)
GZ_TOT=$(awk 'NR>1' "$OUT/liveness.csv" | wc -l)
if [ -z "$GZ_GONE_AT" ]; then
    echo "① gzserver 存活：✅ 全程存活（$GZ_TOT 次采样全部在线）"
else
    echo "① gzserver 存活：❌ 在 ${GZ_GONE_AT}s 消失（$GZ_MIN/$GZ_TOT 次采样离线）"
fi

# ---- ② 话题频率 ----
echo "② 关键话题频率（5s 采样）："
for t in /points /scan /odom /map /ground_truth/odom; do
    R=$(timeout 12 ros2 topic hz "$t" --window 20 2>/dev/null \
        | grep -m1 "average rate" || echo "")
    if [ -n "$R" ]; then
        echo "     $t  ->  $R"
    else
        echo "     $t  ->  ❌ 无数据"
    fi
done

# ---- ③ 重名节点（串扰）----
echo "③ 节点重名检查（串扰）："
NODES=$(timeout 20 ros2 node list 2>/dev/null | sort)
DUP=$(echo "$NODES" | uniq -d)
if [ -n "$DUP" ]; then
    echo "     ❌ 发现重名节点："
    echo "$DUP" | sed 's/^/        /'
else
    echo "     ✅ 无重名节点"
fi
echo "$NODES" > "$OUT/nodes.txt"

PUB=$(timeout 15 ros2 topic info /map 2>/dev/null | sed -n 's/^Publisher count: //p')
echo "     /map publisher count = ${PUB:-未知} $([ "${PUB:-1}" = "1" ] && echo ✅ || echo '❌ 应为 1')"

# ---- ④ 关键 frame 与 slam 输出（2026-09-11 新增，这两项曾静默失效）----
#
#   gimbal_stabilizer 崩过一次，结果 lidar_stabilized 这个 frame 根本不存在，
#   而 scan/points 仍带着它发出去 → slam_toolbox 的 MessageFilter 把每一帧都
#   丢掉 → 地图永远空。症状极隐蔽：话题有数据、节点活着、日志无 ERROR。
#   必须显式查这两项，否则体检会给出"全绿"的假结论。
echo "④ 关键 frame 与 slam 产出："
FRAMES=$(timeout 25 ros2 run tf2_ros tf2_echo base_link lidar_stabilized 2>&1 \
    | grep -m1 "Translation" || echo "")
if [ -n "$FRAMES" ]; then
    echo "     base_link -> lidar_stabilized  ✅ 存在"
else
    echo "     base_link -> lidar_stabilized  ❌ 不存在（scan 的 frame_id 解析不了，"
    echo "         slam_toolbox 会丢光所有帧 —— 检查 gimbal_stabilizer 是否崩溃）"
fi

MAPF=$(timeout 25 ros2 run tf2_ros tf2_echo map odom 2>&1 \
    | grep -m1 "Translation" || echo "")
if [ -n "$MAPF" ]; then
    echo "     map -> odom                    ✅ 存在"
else
    echo "     map -> odom                    ❌ 不存在（说明 slam_toolbox 没在正常工作）"
fi

MAPHZ=$(timeout 20 ros2 topic hz /map 2>/dev/null | grep -m1 "average rate" || echo "")
if [ -n "$MAPHZ" ]; then
    echo "     /map 发布                      ✅ $MAPHZ"
else
    echo "     /map 发布                      ⚠️  未观测到（可能是 TRANSIENT_LOCAL 只发一次）"
fi

# ---- ⑤ slam_toolbox 是否在建图（自查脚本）----
echo "⑤ slam_toolbox 健康检查："
timeout 40 python3 tools/slam_health.py 20 2>&1 | sed 's/^/     /'

echo ""
echo "[smoke] 详细数据：$OUT"
echo "[smoke]   liveness.csv  逐次采样"
echo "[smoke]   sim.log       仿真日志"
echo "[smoke]   nodes.txt     节点清单"
