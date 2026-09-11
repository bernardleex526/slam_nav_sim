# 实测材料

日期：2026-09-11。数据来自本次实际仿真，而非预期结果或示例造数。

- `results.json`：三环境地图与轨迹指标、8 组目标序列/结果/地图哈希、成功与失败目标的 GT 终点误差。
- `navigation_summary.csv`：便于分析的导航指标表。
- `*_mapping.png`：GT、未裁剪 SLAM 地图、双轨迹对比。
- `navigation_success.png`：8 组固定目标任务成功率。
- `maps/<world>/map.yaml` 与 `map.pgm`：便于快速体验的实际基线地图。
- `test_results.txt`、`runtime_versions.txt`：历史回归与运行环境记录。

本目录是精简历史证据，不包含全部原始 CSV、rosbag、完整日志或开发者的绝对路径。fresh clone 后 `verify_delivery.py` 不会读取这些材料来伪装成本机新实验通过；必须先运行 A/B 矩阵生成自己的产物。

安全工作范围验收通过，但全环境覆盖未通过：complex 全图覆盖率只有 69.89%。导航 40 次尝试中 27 次成功，medium 两种默认组合均只有 1/5 成功。平均耗时包含失败，不能单独当作性能优劣依据。

地图不得用于真实机器人安全决策；没有真实动力学、步态或硬件标定的认证。
