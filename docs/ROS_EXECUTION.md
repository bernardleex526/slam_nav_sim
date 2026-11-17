# 当前 ROS 导航验收

v0.1.3 已完成原临时阻挡恢复修复。参考地图与原 005 生成地图的 8 个导航场景全部通过原验收门槛，实际受阻、移除、恢复及穿闸时序已独立复算。

本轮复用历史生成地图，未重新建图。另一次 fresh SLAM 地图退化和中间导航回归失败均保留；本结论不代表 fresh SLAM 稳定性或原计划整体通过。

- [本轮修复与完整证据](BLOCK_RECOVERY_FIX.md)
- [图形汇总](../results/block-recovery-index.html)
- [完整实跑报告](../results/block-recovery-final-004/report.html)
- [v0.1.2 实跑历史](ROS_EXECUTION-v0.1.2.md)
- [部署与复跑环境](ROS_VM_ENVIRONMENT.md)
