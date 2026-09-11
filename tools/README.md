# 工具使用

推荐入口与命令以仓库根目录 [README](../README.md) 为准。

- `run_delivery.sh`：完整正式矩阵，耗时长。
- `test_*.py`：自动回归；使用 unittest discover，不会启动 Gazebo。
- `review_*smoke.py`：有时间上限的真实 Gazebo 集成检查。
- `verify_delivery.py`：严格检查已有阶段 A/B 产物，不启动仿真。
- `build_delivery_report.py`：根据实际落盘结果生成报告与图表。
- `resummarize_stage_a.py`：重建阶段 A 全量汇总。

旧 `clean_sim.sh`、`cleanup_sim.sh` 和部分 `check_*` / `quick_*` shell 脚本包含全局 pkill 或清场逻辑，只能在没有其他实验时阅读后使用，不能用作并行实验入口。
