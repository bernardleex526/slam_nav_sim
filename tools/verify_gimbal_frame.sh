#!/usr/bin/env bash
# =============================================================================
# verify_gimbal_frame.sh —— 30 秒极速验证 lidar_stabilized frame 是否发布
# -----------------------------------------------------------------------------
# 动机（2026-09-11）：
#   gimbal_stabilizer 因 API 误用（.is_zero / .sec / .nanosec）连续翻车，
#   每轮验证都用 3~5 分钟的 smoke，反馈太慢。
#
#   但「frame 是否发布」这件事其实【几秒内就能判定】：
#   gimbal 以 100Hz 发布 TF，只要节点逻辑正常，启动 10s 内必然可见。
#
#   所以这个脚本做最小验证：启动仿真 → 等 35s → 只查一件事：
#     · base_link -> lidar_stabilized 是否存在
#     · 若存在，slam_toolbox 是否开始产出 map
#   不做话题频率统计、不做存活监测，把反馈时间压到 1 分钟以内。
#
# 用法：bash tools/verify_gimbal_frame.sh [world] [wait_sec]
#
# 注意：本脚本【自己写日志文件】，调用方不要再套 `| tail -N` ——
#       SIGPIPE 会在脚本跑到一半时把它杀掉，表现为"目录建好了但 sim.log 是空的"。
#       上一次还踩了 `exec > >(tee ...)` 的坑：进程替换在这个环境下会让脚本
#       在 source ROS setup 时直接以 exit 1 退出，连第一行日志都来不及写。
#       现在改为最朴素的重定向：所有输出进 $OUT/report.txt，结束时原样打出来。
# =============================================================================
set -u

WORLD="${1:-simple}"
WAIT="${2:-40}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="/tmp/vgimbal_${WORLD}_$(date +%H%M%S)"
REPORT="$OUT/report.txt"
mkdir -p "$OUT"

# ==== 从这里开始，所有输出都进 report.txt（结束时原样打出来）====
exec >>"$REPORT" 2>&1

echo "=========================================================="
echo "[vgimbal] 环境=$WORLD  观察 ${WAIT}s  输出=$OUT"
echo "=========================================================="

cd "$WS" || { echo "[vgimbal] ❌ 进不了 $WS"; exit 1; }

# ⚠️ source ROS 环境时必须临时关掉 set -u：
#   ROS 的 setup.bash 会引用若干未定义变量（AMENT_* 等），
#   在 set -u 下 source 会以 "unbound variable" 直接退出，
#   症状是脚本打完标题就消失、exit 1、一个字的报错都看不到。
#   （smoke_sim.sh 没踩这个坑，因为它压根没用 set -u。）
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash >/dev/null 2>&1
# shellcheck disable=SC1091
source install/setup.bash >/dev/null 2>&1
set -u
echo "[vgimbal] ROS 环境已加载"

SIM_PID=""

cleanup() {
    # 注意：不要用 `kill -INT -$SIM_PID`（负 PID = 打进程组）。
    #   $! 拿到的是 setsid 包装进程的 PID，setsid 会立刻 fork 并退出，
    #   那个 PID（及其进程组）早就没了，kill 会失败。改用 pkill 按模式兜底，
    #   这也是 clean_sim.sh 里验证过的做法。
    echo "[vgimbal] 收尾：清理进程 ..."
    pkill -9 -f "ros2 launch sim_bringup" 2>/dev/null
    pkill -9 -f gzserver 2>/dev/null
    pkill -9 -f sim_ws/install 2>/dev/null
    sleep 1
}
trap cleanup EXIT

# nohup 让仿真脱离本脚本的 SIGHUP，避免调用方管道断开时被连带杀死。
# 收尾统一由 cleanup() 的 pkill 负责，所以这里不需要 setsid。
nohup ros2 launch sim_bringup sim.launch.py \
    world:="$WORLD" navigation:=none mapping:=slam_toolbox \
    headless:=true > "$OUT/sim.log" 2>&1 &
SIM_PID=$!
echo "[vgimbal] launch PID=$SIM_PID，等待仿真就绪 ..."

READY=0
for i in $(seq 1 "$WAIT"); do
    sleep 1
    if grep -q "稳定云台已启用" "$OUT/sim.log" 2>/dev/null; then
        READY=1
        echo "[vgimbal] 云台节点已启动（${i}s），再等 8s 让 TF 发出来 ..."
        sleep 8
        break
    fi
done
[ "$READY" = "0" ] && echo "[vgimbal] ⚠️  ${WAIT}s 内没看到云台启动日志，仍继续检查"

echo
echo "--- ① gimbal_stabilizer 日志（前 10 条相关行）---"
grep -E "gimbal_stabilizer\]" "$OUT/sim.log" 2>/dev/null | head -10

echo
echo "--- ② TF 树中是否存在 lidar_stabilized ---"
timeout 10 ros2 run tf2_ros tf2_echo base_link lidar_stabilized \
    > "$OUT/tf_echo.txt" 2>&1
echo "   （完整输出见 $OUT/tf_echo.txt）"
if grep -q "Translation" "$OUT/tf_echo.txt" 2>/dev/null; then
    echo "   ✅ base_link -> lidar_stabilized 存在！"
    grep -m3 -E "Translation|Rotation" "$OUT/tf_echo.txt" | sed 's/^/     /'
else
    echo "   ❌ base_link -> lidar_stabilized 不存在"
    grep -m3 -iE "does not exist|Invalid frame|Exception" "$OUT/tf_echo.txt" \
        | sed 's/^/     /'
fi

echo
echo "--- ③ slam_toolbox 是否还在丢帧 ---"
# ⚠️ grep -c 无匹配时返回码非 0，但同时会打印 "0"。
#    若写 $(grep -c ... || echo 0) 会得到 "0\n0" 两行，
#    后面拿去做数值比较就会报 "需要整数表达式"。
DROP=$(grep -c "Message Filter dropping message" "$OUT/sim.log" 2>/dev/null)
DONE=$(grep -c "Registering sensor\|Update Map\|scan matching" "$OUT/sim.log" 2>/dev/null)
DROP=${DROP:-0}
DONE=${DONE:-0}
echo "   MessageFilter 丢帧行数 = $DROP"
echo "   slam 处理痕迹行数     = $DONE"
if [ "$DROP" -gt 0 ]; then
    echo "   ⚠️  仍在丢帧（若丢帧行数 < 5 且随时间停止，可能是启动瞬间的暂态，看下面的 map）"
else
    echo "   ✅ 无丢帧"
fi

echo
echo "--- ④ /map 是否在产出 ---"
timeout 15 python3 - <<'PY' 2>&1 | tail -12
import time
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid

rclpy.init()
n = Node("vg_map_probe")
n.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

box = {"n": 0, "info": None}

def cb(m):
    box["n"] += 1
    if box["info"] is None:
        box["info"] = (m.info.width, m.info.height,
                       m.header.stamp.sec, m.header.stamp.nanosec)

qos = QoSProfile(depth=10,
                 durability=DurabilityPolicy.TRANSIENT_LOCAL,
                 reliability=ReliabilityPolicy.RELIABLE)
n.create_subscription(OccupancyGrid, "/map", cb, qos)

# 挂死保护：外层 timeout 15s 之外，再用【墙钟】兜底。
# 不能依赖仿真时钟计算超时——万一 /clock 没收到，now() 恒为 0，循环永不退出。
t0 = time.monotonic()
while rclpy.ok() and time.monotonic() - t0 < 10.0:
    rclpy.spin_once(n, timeout_sec=0.2)
print(f"   /map 收到 {box['n']} 帧")
if box["info"]:
    w, h, s, ns = box["info"]
    print(f"   首帧 {w}x{h} 格，时戳 {s}.{ns:09d}")
n.destroy_node()
rclpy.shutdown()
PY

echo
echo "=========================================================="
echo "[vgimbal] 日志：$OUT/sim.log"
echo "=========================================================="
