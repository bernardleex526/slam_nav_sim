#!/usr/bin/env bash
# =============================================================================
# tf_probe_run.sh —— 短时探索 + map->odom 审计
# -----------------------------------------------------------------------------
# 想一次回答的问题：
#   机器人用 /odom 看始终在场地内（20x20m），但保存的地图却长到 y=-59m。
#   到底是 map->odom 在漂，还是别的东西？
#
# 做法：起一套仿真（simple + slam_toolbox + 割草机探索），同时用
#       tools/tf_audit.py 记录 map->odom / map->base 序列，跑 N 秒后分析。
#
# 用法：
#     bash tools/tf_probe_run.sh [秒数]      # 默认 240
# =============================================================================
DUR="${1:-240}"
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OUT="/tmp/tf_probe_$(date +%H%M%S)"
mkdir -p "$OUT"

echo "=========================================================="
echo "[probe] 时长 ${DUR}s  输出 $OUT"
echo "=========================================================="

cd "$WS" || exit 1
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash >/dev/null 2>&1
# shellcheck disable=SC1091
source install/setup.bash >/dev/null 2>&1

cleanup() {
    echo "[probe] 收尾 ..."
    pkill -9 -f "ros2 launch sim_bringup" 2>/dev/null
    pkill -9 -f gzserver 2>/dev/null
    pkill -9 -f tf_audit 2>/dev/null
    pkill -9 -f exploration_runner 2>/dev/null
    pkill -9 -f sim_ws/install 2>/dev/null
    sleep 2
    echo "[probe] 已清场"
}
trap cleanup EXIT

echo "[probe] 起仿真（simple + slam_toolbox + 无导航）..."
setsid ros2 launch sim_bringup sim.launch.py \
    robot:=wheeled_quadruped world:=simple \
    mapping:=slam_toolbox navigation:=none \
    odom_source:=gt use_gimbal:=true headless:=true \
    metrics:=true run_name:=tf_probe viz:=false \
    > "$OUT/sim.log" 2>&1 &
sleep 5

# 等就绪
for _ in $(seq 1 60); do
    if grep -q "Registering sensor" "$OUT/sim.log" 2>/dev/null; then break; fi
    sleep 2
done
sleep 12
echo "[probe] 仿真就绪，开始审计 TF ..."

# 探索（后台）
setsid ros2 run sim_benchmark exploration_runner.py --ros-args \
    -p world:=simple -p lane_spacing:=1.200 -p speed:=0.500 \
    -p clearance:=0.450 -p max_duration:=600.0 \
    -p traj_dir:="$OUT" > "$OUT/explore.log" 2>&1 &
EXPL=$!

# TF 审计
python3 tools/tf_audit.py --duration "$DUR" --out "$OUT/tf.csv" \
    > "$OUT/audit.log" 2>&1
kill -INT $EXPL 2>/dev/null

echo ""
echo "=========================================================="
echo "[probe] map->odom 审计"
echo "=========================================================="
python3 tools/tf_audit.py --analyze "$OUT/tf.csv"

# 顺手存图看尺寸
echo ""
echo "[probe] 保存当前地图 ..."
timeout 60 ros2 run nav2_map_server map_saver_cli \
    -f "$OUT/map" --ros-args -p save_map_timeout:=20.0 > /dev/null 2>&1
if [ -f "$OUT/map.yaml" ]; then
    echo "地图尺寸："
    grep -E "^origin|^resolution" "$OUT/map.yaml" | sed 's/^/   /'
    python3 - "$OUT/map.yaml" <<'PYEOF'
import sys, yaml, os
try:
    from PIL import Image
    m = yaml.safe_load(open(sys.argv[1]))
    p = os.path.join(os.path.dirname(sys.argv[1]), m["image"])
    im = Image.open(p)
    r = m["resolution"]; ox, oy = m["origin"][0], m["origin"][1]
    print(f"   像素 {im.size[0]}x{im.size[1]}  ->  "
          f"世界 x[{ox:.2f},{ox+im.size[0]*r:.2f}] "
          f"y[{oy:.2f},{oy+im.size[1]*r:.2f}]")
except Exception as e:
    print("   读取地图尺寸失败:", e)
PYEOF
fi

echo ""
echo "[probe] 详细输出：$OUT"
