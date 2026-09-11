#!/usr/bin/env bash
# 导航栈集成测试：slam_toolbox 建图 + Nav2 导航
OUT=/tmp/slam_nav_sim_nav_test.txt
: > "$OUT"; exec >> "$OUT" 2>&1

source /opt/ros/humble/setup.bash
WS="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$WS"; source install/setup.bash

echo "########## A. 参数合成器在真实环境跑一遍 ##########"
echo "  已安装的可执行:"
ls install/nav2_config/lib/nav2_config/ 2>/dev/null | sed 's/^/    /'
echo "  --- 运行 compose（smac_hybrid + mppi）---"
timeout -k 3 60 ros2 run nav2_config compose_nav2_params.py \
  --robot wheeled_quadruped --global-planner smac_hybrid --local-planner mppi \
  --odom-source gt --localization slam --out /tmp/nav2_real.yaml 2>&1 | sed 's/^/  /'
echo "  --- 校验产出 ---"
python3 - <<'PY'
import yaml
try:
    d = yaml.safe_load(open('/tmp/nav2_real.yaml'))
    ps = d['planner_server']['ros__parameters']
    cs = d['controller_server']['ros__parameters']
    lc = d['local_costmap']['local_costmap']['ros__parameters']
    print(f"  顶层节点 {len(d)} 个: {', '.join(list(d)[:6])} ...")
    print(f"  planner_plugins   : {ps.get('planner_plugins')}")
    print(f"  controller_plugins: {cs.get('controller_plugins')}")
    print(f"  footprint(local)  : {lc.get('footprint')}")
    print(f"  inflation_radius  : {lc['inflation_layer']['inflation_radius']}")
    print(f"  amcl.enabled      : {d['amcl']['ros__parameters']['enabled']}  (slam 模式应为 False)")
    resid = [k for k in ('GridBased','FollowPath','navfn','dwb') if k in ps or k in cs]
    print(f"  参数残留          : {resid if resid else '无'}")
except Exception as e:
    print("  解析失败:", e)
PY

echo
echo "########## B. 启动 slam_toolbox + Nav2 ##########"
timeout -k 3 20 ros2 daemon stop  >/dev/null 2>&1; sleep 2
timeout -k 3 20 ros2 daemon start >/dev/null 2>&1; sleep 2
rm -f /dev/shm/fastrtps_* 2>/dev/null
pkill -9 -f '[g]zserver' 2>/dev/null; sleep 2

LOG=/tmp/nav_test.log
setsid ros2 launch sim_bringup sim.launch.py \
  world:=simple headless:=true mapping:=slam_toolbox navigation:=nav2 \
  localization:=slam odom_source:=gt global_planner:=smac_2d local_planner:=dwb \
  viz:=false metrics:=true use_gimbal:=true > "$LOG" 2>&1 &
LPID=$!
cleanup() {
  kill -9 -- -"$LPID" 2>/dev/null
  pkill -9 -f '[g]zserver' 2>/dev/null
  pkill -9 -f '[r]obot_state_publisher' 2>/dev/null
  pkill -9 -f '[n]av2' 2>/dev/null
  pkill -9 -f '[s]lam_toolbox' 2>/dev/null
  pkill -9 -f '[m]etrics_collector' 2>/dev/null
  pkill -9 -f '[m]apping_contract_adapter' 2>/dev/null
  pkill -9 -f '[p]oints_to_scan' 2>/dev/null
  pkill -9 -f '[c]loud_downsampler' 2>/dev/null
  pkill -9 -f '[g]imbal_stabilizer' 2>/dev/null
  pkill -9 -f '[s]im_joint_state_bridge' 2>/dev/null
  sleep 2
}
trap cleanup EXIT

for i in $(seq 1 8); do
  timeout -k 3 30 ros2 topic echo /clock --once >/dev/null 2>&1 && break
done
echo "  /clock OK，等 45 秒让建图与导航栈起来…"
sleep 45

echo
echo "--- 关键话题 ---"
for t in /scan /points /map /odom /ground_truth/odom /map_cloud /cmd_vel /plan /global_costmap/costmap /local_costmap/costmap /eval/gt_path /eval/metrics; do
  if ros2 topic list 2>/dev/null | grep -qx "$t"; then
    P=$(ros2 topic info "$t" 2>/dev/null | grep -oP 'Publisher count: \K\d+')
    printf "  存在 %-30s 发布者=%s\n" "$t" "${P:-0}"
  else
    printf "  缺失 %-30s\n" "$t"
  fi
done

echo
echo "--- /scan 与 /map 是否有数据 ---"
for t in /scan /map /map_cloud; do
  if timeout -k 3 15 ros2 topic echo "$t" --once >/dev/null 2>&1; then echo "  OK   $t"; else echo "  FAIL $t"; fi
done

echo
echo "--- 导航 action server ---"
timeout -k 3 20 ros2 action list 2>/dev/null | grep -i navigate | sed 's/^/  /'

echo
echo "--- 节点清单（应含 nav2 全家 + slam_toolbox + 适配层）---"
timeout -k 3 20 ros2 node list --no-daemon 2>/dev/null | sort | sed 's/^/  /'

echo
echo "--- 指标采集是否在出数据 ---"
timeout -k 3 15 ros2 topic echo /eval/metrics --once 2>/dev/null | head -18 | sed 's/^/  /'

echo
echo "--- 日志 ERROR（排除已知无害）---"
grep -E "\[ERROR\]" "$LOG" 2>/dev/null | grep -viE "does not exist" | head -12 | sed 's/^/  /'
echo "  (以上为全部)"

echo
echo "########## 结束 ##########"
