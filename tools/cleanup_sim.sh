#!/usr/bin/env bash
# 清理仿真残留进程。
# 为什么写成脚本文件而不是一条 bash -lc：
#   Bash 工具会解析嵌套引号，且 pkill -f 的模式字符串本身会出现在
#   命令行里被自己匹配到，导致 pkill 杀掉调用它的 shell（Exit 15）。
#   写成文件后，模式字符串只出现在脚本内，配合 [x] 括号技巧双重保险。
set -u

for pat in "[g]zserver" "[g]zclient" "[s]ync_slam_toolbox" "[a]sync_slam_toolbox" \
           "[r]obot_state_publisher" "[s]pawn_entity" "[p]oints_to_scan" \
           "[m]apping_contract_adapter" "[c]loud_downsampler" \
           "[c]ontroller_server" "[p]lanner_server" "[b]t_navigator" \
           "[m]etrics_collector" "[e]xploration_runner" "[m]ap_saver" \
           "[a]mcl" "[l]ifecycle_manager" "[v]elocity_smoother" \
           "[b]ehavior_server" "[w]aypoint_follower" "[r]obot_interface"; do
  pkill -f "$pat" 2>/dev/null || true
done

sleep 2

# 顽固进程强杀
for pat in "[g]zserver" "[g]zclient"; do
  pkill -9 -f "$pat" 2>/dev/null || true
done

sleep 1
n_gz=$(pgrep -f "[g]zserver" | wc -l)
n_slam=$(pgrep -f "[s]lam_toolbox" | wc -l)
echo "清理完成：gzserver=$n_gz slam_toolbox=$n_slam"
