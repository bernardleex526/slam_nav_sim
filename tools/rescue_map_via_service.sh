#!/usr/bin/env bash
# =============================================================================
# rescue_map_via_service.sh —— 用 slam_toolbox 的 SaveMap 服务把正在运行
# 那轮的地图抢救出来。
# =============================================================================
#
# 为什么需要它（一次性救急脚本）：
#   stage_a_runner 原本只用 map_saver_cli 存图，而它靠**订阅 /map 话题**拿图；
#   /map 是 transient_local，时机错开就一路卡到超时，实验白跑（实测过一轮，
#   探索 18 分钟的产物全丢）。slam_toolbox 自带的 SaveMap 服务直接序列化
#   内部地图，没有时序问题，实测 1 秒出图、result=0。
#
#   本脚本是给【已经跑了很久的那一轮】救急用的：stage_a_runner 是 Python，
#   启动时就把源码读进内存，改源码对运行中的进程无效。好在 main loop 有窗口：
#       探索结束 → sleep 3s → map_saver 失败(≤90s) → kill_group(sim)
#   探索结束后还有约 90 秒可以抢存。
#
#   ⚠️ 后续轮次不需要它：stage_a_runner 已内置「SaveMap 优先 + CLI 兜底」。
#
#【v2 为什么改用 PID 监听而不是 grep 日志】
#   v1 用 `grep '探索完成|超时'` 扫整份日志 —— 但 stage_a_runner 是
#   【多个实验往同一个日志文件里追加】，第一轮留下的"探索超时"会让守护刚
#   启动 3 秒就误触发，那时探索才跑到 91%，抢出来的是张残缺地图。
#   改盯**探索进程本身**：进程退出 = 这一轮真的结束了，语义没有歧义。
#
# 用法：
#   bash tools/rescue_map_via_service.sh <exploration_runner的PID> <run目录>
# =============================================================================
set -uo pipefail

PID="${1:?用法: rescue_map_via_service.sh <探索进程PID> <产物run目录>}"
RUN_DIR="${2:?缺少产物 run 目录}"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"

# ⚠️ source ROS setup.bash 会引用未定义变量，`set -u` 下脚本会直接退出 1
# 且不打任何报错（踩过，很难查）。source 前后必须临时关掉 nounset。
set +u
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source "${ROOT}/install/setup.bash" >/dev/null 2>&1
set -u

echo "[$(date '+%H:%M:%S')] 监听探索进程 PID=$PID  目标目录：$RUN_DIR"

# 阶段一：等探索进程真的退出
n=0
while kill -0 "$PID" 2>/dev/null; do
    sleep 5
    n=$((n + 5))
    if [ "$n" -ge 3600 ]; then
        echo "[$(date '+%H:%M:%S')] 监听超时（探索跑了 1 小时还没结束）"
        exit 1
    fi
done
echo "[$(date '+%H:%M:%S')] 探索进程已退出，准备抢存"

# 给 slam_toolbox 一点时间把最后一帧并进图
sleep 5

# 阶段二：反复抢存。
# 为什么抢多次而不是一次：stm之后 sim 大约还有 60~90 秒才被 kill_group，
# 期间 slam 仍在更新（有时还会做最后的回环优化）。多抢几次，最后成功的
# 那张就是内容最新的；下一次重/覆盖不会丢失信息，失败了也不影响别人。
mkdir -p "$RUN_DIR" 2>/dev/null
for i in $(seq 1 8); do
    timeout 40 ros2 service call /slam_toolbox/save_map \
        slam_toolbox/srv/SaveMap \
        "{name: {data: '${RUN_DIR}/map'}}" \
        > "${RUN_DIR}/rescue_savemap.log" 2>&1
    if [ -f "${RUN_DIR}/map.yaml" ]; then
        sz=$(stat -c %s "${RUN_DIR}/map.pgm" 2>/dev/null || echo 0)
        echo "[$(date '+%H:%M:%S')] ✅ 第 $i 次抢存成功（map.pgm ${sz} bytes）"
    else
        echo "[$(date '+%H:%M:%S')] ❌ 第 $i 次抢存未出图"
    fi
    sleep 12
done

echo "[$(date '+%H:%M:%S')] 抢存结束，最终产物："
ls -la "${RUN_DIR}/map.pgm" "${RUN_DIR}/map.yaml" 2>/dev/null || echo "  （无产物）"
