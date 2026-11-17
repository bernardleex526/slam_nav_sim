# 验证记录

2026-11-16，macOS Apple Silicon／Python 3.9.6，以及本机 Ubuntu 24.04 arm64 VM／Python 3.12.3。

## 本轮 v0.1.3

- Linux 与 macOS 各 **74 项工具测试通过**，结果位于 `results/block-recovery-environment/tool-tests-linux.xml` 和 `tool-tests-macos.xml`；新增覆盖闸门拓扑、停顿新鲜度、恢复先后、地图复用和不支持语义拒绝。
- 真实 ROS／Gazebo pytest 契约 **1 项通过，270.35 秒**，对应两图各四场景及窄通道实际穿越检查；见 `ros-contract-complete.xml`。当前回归复用原 005 生成地图，未执行新的 SLAM。
- 两项临时阻挡分别有 **2.86／2.84 秒**独立连续受阻证据，随后移除、恢复、穿闸，零接触到达；八个导航场景全部通过原数值门槛。
- 真实 Nav2 防护边界对照：旧 STOP 配置“阻止前进／允许安全旋转／清障后前进”为通过／失败／通过；最终配置三项均通过。见 `results/front-rotation-before-002/result.json` 与 `front-rotation-after-final/result.json`。
- 世界拓扑回归先在旧障碍上失败、修复后通过。仅延时插入、全宽闸门、首次完整导航及重新建图的中间失败均保留，没有改判或删掉。
- 原阈值文件、评分函数、参考地图和复用生成地图哈希不变；静态世界去除闸门后相同。新增闸门几何和导航配置明确记录，两图使用完全相同的新配置。
- 原始接触序列、四碰撞体计数、独立速度／位姿及实际 Nav2 防护状态已复算核对，见 `results/block-recovery-audit.json`。实际运行节点参数另存每个阶段的 `controller-configuration.json`。
- Chromium Headless 离线打开、内嵌时序图、八个导航叠图、证据展开、筛选和手机无横向溢出均通过；截图和 JSON 在 `results/block-recovery-preview.png`、`block-recovery-route-preview.png`、`block-recovery-browser-qa.json`。
- 两路独立审查关闭了末端转向锁死和复用地图 raw／scale／alpha 语义风险。源图仅支持明确的 trinary 8 位灰度 PGM；不支持输入会拒绝。详见 `ROS_REVIEW.md`。

本轮不宣称 fresh SLAM 稳定性通过；002 重建图误判空闲 30.51%、墙体 P95 1.16 m 的失败继续保留。完整结论见 [BLOCK_RECOVERY_FIX.md](BLOCK_RECOVERY_FIX.md)。

## 历史 v0.1.2

- macOS 最终 64 项工具测试通过，见 `results/ros2-environment/tool-tests-macos-final.xml`／`.log`；Ubuntu 最终 64 项通过，见 `tool-tests-linux-final.xml`／`.log`。覆盖原链路和本轮路线反馈、参考采样相位、缺失参考、窄通道绕行／缺口、接触自检异常等行为。
- macOS 初次 58 项运行中曾出现一次进程清理 `killpg` PermissionError（57 通过、1 失败）。单独三项进程测试、随后完整 58 项和最终 64 项均通过；没有吞掉权限错误或放宽断言，根因尚未确认，原失败日志保留为 `tool-tests-macos.log`。
- Ruff、格式检查、Python 编译、shell 语法、Git 差异空白检查通过。
- 原阈值配置、默认阈值模型及评分函数 SHA256 未变，见 `results/ros2-environment/acceptance-integrity.json`。
- 接触 C++ 插件已在 Linux 编译；真实障碍注入验证通过，基线零接触、四个碰撞体、两次正例分别记录 80／48 条接触。
- 完成独立参考地图健康导航及三轮完整参考／生成地图各四场景对照。中间 `ros2-demo-002` 在第六个导航阶段启动时遭遇 DDS lifecycle 响应发送超时，未发送目标即中断；部分报告原样保留。第三轮不并发工具测试仍在第七个导航阶段创建 ROS 节点时卡住；保留线程栈并诊断中断（退出 130）。第四轮统一切换 Cyclone DDS 后完整重跑，但两个参考场景因约 1 秒观测缺口被阻断；增加动作发出前发现稳定等待及接触流检查后，短场景 maxgap 恢复 0.02 s，第五轮再次完整重跑。环境规避通过本轮验证，Fast DDS 内部根因及长期稳定性仍不能断言。
- 最终真实矩阵保持非零退出：临时阻挡绕行到达仍缺少受阻后恢复证据，不能凭动作成功通过。实际地图与各场景指标见 [实跑报告](ROS_EXECUTION.md)。本轮地图通过不能代替跨次稳定性结论。
- Chromium Headless 实际验证离线 `file://` 页面、8 个导航叠图、失败展开、状态筛选、地图图层开关、手机宽度无横向溢出和无脚本错误，见 `results/ros2-browser-qa.json`。PNG 嵌入图片解码和报告相对链接另行核对。同一恢复计数的重复反馈在显示层收起，避免数百个相同计数标记遮挡实际轨迹；原始 events.json、判定 JSON 和 JUnit 保持不变。
- 两路独立审查分别检查 Standards 与 Spec。关闭接触自检异常仍可能报告 pass 的问题，新增导出和清理异常回归，详情见 [ROS_REVIEW.md](ROS_REVIEW.md)。

严格健康 pytest 契约 `configs/ros-healthy.case.json` 已提供；本轮真实场景通过 CLI 执行，未另外重复运行该 pytest 契约。工具测试通过不代表被测导航矩阵整体通过。

## 历史 v0.1.1 证据

原型 8 项、本地插件 51 项测试通过，8 份本地案例报告、同种子逐字节一致性和噪声种子 614／1009 结果保持保留。相关证据在 `results/test-suite.xml`、`browser-qa.json`、`html-integrity.json`、`holdouts.json` 和 `history/v0.1.0/`。本地地图修复详见 [MAP_QUALITY_FIX.md](MAP_QUALITY_FIX.md)。

Docker 镜像仍未构建；本轮验证的是原生 Linux VM。ROS 故障注入全矩阵、多种子统计、公开包回放、回环内部事件归因、真机以及原计划其他扩展仍待执行。
