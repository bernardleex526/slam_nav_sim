#!/usr/bin/env python3
"""Run local acceptance and fault controls into a new report directory."""

import argparse
from html import escape
import json
from pathlib import Path
import shutil

from robot_validate.model import dump_json, load_config, sha256
from robot_validate.runner import supervised_run

LABELS = {
    "pass": "通过",
    "fail": "失败",
    "insufficient": "证据不足",
    "environment_error": "环境错误",
}
METRICS = {
    "ATE": "轨迹 ATE RMSE（米）",
    "MAP_COVERAGE": "地图覆盖率",
    "FALSE_FREE": "误判空闲比例",
    "WALL_P95": "墙体 P95（米）",
}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--baseline", type=Path)
    args = parser.parse_args()
    root = args.output.resolve()
    if root.exists() and any(p.name != "history" for p in root.iterdir()):
        parser.error("output must be empty (a history/ archive is allowed)")
    root.mkdir(parents=True, exist_ok=True)
    cases = [
        ("local-baseline", "demo", "none", "local", 0, "point_to_line"),
        ("public-bag", "public", "none", "local", 2, "point_to_line"),
        ("scan-gap", "demo", "scan-gap", "local", 1, "point_to_line"),
        ("odometry-bias", "demo", "odometry-bias", "local", 0, "point_to_line"),
        ("no-truth", "demo", "no-truth", "local", 2, "point_to_line"),
        ("map-export", "demo", "map-export", "local", 1, "point_to_line"),
        ("legacy-rmhc", "demo", "none", "local", 1, "rmhc"),
    ]
    if not shutil.which("ros2") or not shutil.which("gz"):
        cases.append(("ros2-preflight", "demo", "none", "ros2", 3, None))
    summary = []
    for name, scenario, fault, backend, expected, matcher in cases:
        cfg = load_config()
        if matcher:
            cfg["algorithm"] = {"local_matcher": matcher}
        request = dict(
            scenario=scenario,
            backend=backend,
            config=cfg,
            fault=fault,
            bag=None,
            mode="bag" if scenario == "public" else "sim",
        )
        code, report = supervised_run(request, root / name, timeout=300)
        result = json.loads(report.with_name("result.json").read_text())
        summary.append(
            dict(
                case=name,
                status=result["status"],
                exit_code=code,
                expected_exit=expected,
                report=f"{name}/report.html",
                checks=[
                    dict(id=c["id"], status=c["status"], value=c["value"], threshold=c["threshold"])
                    for c in result["checks"]
                    if c["required"]
                ],
            )
        )
        print(f"{name}: {code} (expected {expected})", flush=True)
    dump_json(root / "summary.json", summary)
    comparisons = []
    lock = {}
    if args.baseline:
        before_root = args.baseline.resolve() / "local-baseline"
        before = json.loads((before_root / "result.json").read_text())
        after = json.loads((root / "local-baseline/result.json").read_text())
        old = {c["id"]: c for c in before["checks"]}
        new = {c["id"]: c for c in after["checks"]}
        for name in METRICS:
            comparisons.append(
                dict(
                    metric=name,
                    before=old[name]["value"],
                    after=new[name]["value"],
                    threshold=new[name]["threshold"],
                    status=new[name]["status"],
                )
            )
        lock = dict(
            input_bag_identical=sha256(before_root / "input.bag")
            == sha256(root / "local-baseline/input.bag"),
            reference_map_identical=sha256(before_root / "reference-map.pgm")
            == sha256(root / "local-baseline/reference-map.pgm"),
            thresholds_identical=before["provenance"]["config"]["thresholds"]
            == after["provenance"]["config"]["thresholds"],
        )
        if not all(lock.values()):
            raise AssertionError(f"Baseline lock changed: {lock}")
    dump_json(root / "comparison.json", dict(unchanged=lock, metrics=comparisons))
    table = "".join(
        f"<tr><td>{escape(METRICS[c['metric']])}</td><td>{c['before']:.6f}</td><td>{c['after']:.6f}</td><td>{'≥' if c['metric'] == 'MAP_COVERAGE' else '≤'} {c['threshold']}</td><td>{LABELS[c['status']]}</td></tr>"
        for c in comparisons
    )
    rows = "".join(
        f'<tr><td><a href="{escape(r["report"])}">{escape(r["case"])}</a></td><td>{LABELS[r["status"]]}</td><td>{r["exit_code"]}</td><td>'
        + escape("；".join(c["id"] + ": " + LABELS[c["status"]] for c in r["checks"]))
        + "</td></tr>"
        for r in summary
    )
    html = (
        """<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>地图质量修复验证</title><style>body{font:16px/1.7 system-ui,sans-serif;background:#f3f5f7;color:#233b4c;margin:0}main{max-width:1120px;margin:30px auto;padding:24px}h1{font-size:32px}.scroll{overflow:auto}table{width:100%;border-collapse:collapse;background:white}th,td{text-align:left;padding:13px;border-bottom:1px solid #dbe4eb}a{color:#136c9b}small{color:#506979}.note{background:#e8f6f0;padding:18px;border-left:4px solid #18876d}.pending{background:#fff6e6;padding:16px;border-left:4px solid #d69c3d}</style><main><small>ROBOT VALIDATION · v0.1.1 · 2026-11-16</small><h1>地图质量修复验证</h1><p class="note">本地基线建图通过。原始输入包、参考地图和验收阈值保持不变；算法只使用传感器观测，不读取评估真值。</p><h2>修复前后</h2><div class="scroll"><table><tr><th>指标</th><th>修复前</th><th>修复后</th><th>原门槛</th><th>结论</th></tr>"""
        + table
        + """</table></div><p>修复包括：扫描与子地图的点到线配准、真实射线积分、占据概率换算、半格原点修正。新增定位器为本项目实现，地图积分继续复用 BreezySLAM。</p><p class="pending">以下仅为本地二维建图与工具验证。ROS 2／Gazebo 镜像未构建和实跑；导航与系统集成仍待验收。</p><h2>正常与故障对照</h2><div class="scroll"><table><tr><th>案例／报告</th><th>结论</th><th>退出码</th><th>必测项</th></tr>"""
        + rows
        + """</table></div><p>轮速放大故障由激光匹配纠正，因此输出质量通过；不能据此认为轮速输入没有故障。扫描断流、导出失败和缺真值仍按证据判定。旧 RMHC 匹配器仍作为失败对照保留。</p><p><a href="comparison.json">前后指标及不变性检查</a> · <a href="summary.json">批次 JSON</a> · <a href="test-suite.xml">工具测试 JUnit</a> · <a href="holdouts.json">未参与诊断的噪声种子</a> · <a href="history/v0.1.0/index.html">修复前报告</a></p></main></html>"""
    )
    baseline_note = f"本地基线建图：{LABELS[summary[0]['status']]}。"
    baseline_note += (
        "原始输入包、参考地图和验收阈值保持不变。"
        if lock
        else "未指定旧基线，本轮不作前后不变性断言。"
    )
    baseline_note += "算法只使用传感器观测，不读取评估真值。"
    html = html.replace(
        "本地基线建图通过。原始输入包、参考地图和验收阈值保持不变；算法只使用传感器观测，不读取评估真值。",
        baseline_note,
    )
    html = html.replace(
        "ROS 2／Gazebo 镜像未构建和实跑；导航与系统集成仍待验收。",
        "本批未执行 ROS 2／Gazebo 闭环导航；不构成导航和系统集成验收。",
    )
    html = html.replace(
        "轮速放大故障由激光匹配纠正，因此输出质量通过；不能据此认为轮速输入没有故障。",
        "轮速偏置案例若输出通过，表示此例下激光匹配纠正了偏置，不能据此认为轮速输入没有故障。",
    )
    for filename, label in [
        ("test-suite.xml", "工具测试 JUnit"),
        ("holdouts.json", "未参与诊断的噪声种子"),
        ("history/v0.1.0/index.html", "修复前报告"),
    ]:
        if not (root / filename).exists():
            html = html.replace(f' · <a href="{filename}">{label}</a>', "")
    (root / "index.html").write_text(html)
    return 0 if all(r["exit_code"] == r["expected_exit"] for r in summary) else 1


if __name__ == "__main__":
    raise SystemExit(main())
