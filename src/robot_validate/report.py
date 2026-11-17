import base64
from html import escape
from io import BytesIO
import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np
from PIL import Image

from .metrics import sample_grid, world_coordinates
from .data import read_tum
from .slam import Grid
from .model import dump_json

LABELS = {
    "pass": "通过",
    "fail": "失败",
    "insufficient": "证据不足",
    "environment_error": "环境错误",
}


def map_figure(generated, reference=None, estimated=None, truth=None, overlays=(), markers=()):
    """Embed images in a single HTML; no external fonts, scripts, or image dependencies."""
    if reference is not None:
        corners = world_coordinates(reference)
        x0, y0 = corners.min(axis=0) - reference.resolution
        x1, y1 = corners.max(axis=0) + reference.resolution
    elif estimated is not None:
        x0, y0 = estimated[:, 1:3].min(axis=0) - 4
        x1, y1 = estimated[:, 1:3].max(axis=0) + 4
    else:
        x0, y0 = generated.origin[:2]
        x1 = x0 + generated.cells.shape[1] * generated.resolution
        y1 = y0 + generated.cells.shape[0] * generated.resolution
    width, height = 900, max(350, min(850, int(900 * (y1 - y0) / (x1 - x0))))
    xx, yy = np.meshgrid(np.linspace(x0, x1, width), np.linspace(y1, y0, height))
    points = np.column_stack((xx.ravel(), yy.ravel()))

    def raster(grid, color=None):
        values = sample_grid(grid, points).reshape(height, width)
        pixels = np.zeros((height, width, 4), np.uint8)
        if color is None:
            pixels[:] = [225, 231, 237, 255]
            pixels[values == 0] = [250, 251, 252, 255]
            pixels[values == 100] = [44, 58, 71, 255]
        else:
            pixels[values == 100] = color
        stream = BytesIO()
        Image.fromarray(pixels).save(stream, format="PNG")
        return base64.b64encode(stream.getvalue()).decode()

    layers = [
        f'<image width="{width}" height="{height}" href="data:image/png;base64,{raster(generated)}"/>'
    ]
    if reference is not None:
        layers.append(
            f'<image class="reference" width="{width}" height="{height}" href="data:image/png;base64,{raster(reference, [0, 174, 180, 170])}"/>'
        )
    for points_, color, label in ((estimated, "#da6a24", "estimated"), (truth, "#1571c4", "truth")):
        if points_ is None:
            continue
        coords = " ".join(
            f"{(x - x0) / (x1 - x0) * width:.2f},{(y1 - y) / (y1 - y0) * height:.2f}"
            for x, y in points_[:, 1:3]
        )
        layers.append(
            f'<polyline class="{label}" fill="none" stroke="{color}" stroke-width="2.5" points="{coords}"/>'
        )
    for path, color in overlays:
        coords = " ".join(
            f"{(x - x0) / (x1 - x0) * width:.2f},{(y1 - y) / (y1 - y0) * height:.2f}"
            for x, y in path
        )
        layers.append(
            f'<polyline fill="none" stroke="{color}" stroke-width="2" stroke-dasharray="5 4" points="{coords}"/>'
        )
    for x, y, label, color in markers:
        sx, sy = (x - x0) / (x1 - x0) * width, (y1 - y) / (y1 - y0) * height
        layers.append(
            f'<circle cx="{sx:.2f}" cy="{sy:.2f}" r="6" fill="{color}"><title>{escape(label)}</title></circle>'
        )
    return f'<svg role="img" aria-label="地图和轨迹叠图" viewBox="0 0 {width} {height}">{"".join(layers)}</svg>'


def compact_recovery_feedback(events):
    """Display cumulative recovery counters once; keep the raw event file unchanged."""
    displayed = []
    for event in events:
        if (
            displayed
            and event["kind"] == "recovery"
            and event.get("count") is not None
            and displayed[-1]["kind"] == "recovery"
            and displayed[-1].get("count") == event["count"]
        ):
            previous = displayed[-1]
            previous["feedback_samples"] = previous.get("feedback_samples", 1) + 1
            previous["last_feedback_s"] = event["time_s"]
        else:
            displayed.append(dict(event))
    return displayed


def navigation_panels(output):
    comparison = output / "navigation-comparison.json"
    if not comparison.exists():
        return ""
    panels = []
    for record in json.loads(comparison.read_text()):
        source, name = record["map_source"], record["name"]
        phase = output / source / name
        map_path = output / ("map.yaml" if source == "generated" else "reference-map.yaml")
        truth_path = phase / "ground-truth-raw.tum"
        truth = read_tum(truth_path) if truth_path.exists() else None
        paths = (
            json.loads((phase / "paths.json").read_text())
            if (phase / "paths.json").exists()
            else []
        )
        events = (
            json.loads((phase / "events.json").read_text())
            if (phase / "events.json").exists()
            else []
        )
        events = compact_recovery_feedback(events)
        overlays = [(p["points"], "#8b5fbf") for p in paths[:1] if p["points"]]
        markers = [(*record["goal"][:2], "目标", "#1e9a77")]
        for event in events:
            if truth is not None and event["kind"] in ("collision", "recovery"):
                pose = truth[np.argmin(abs(truth[:, 0] - event["time_s"]))]
                markers.append((*pose[1:3], event["kind"], "#d33e36"))
        plot = map_figure(
            Grid.load(map_path),
            Grid.load(output / "reference-map.yaml"),
            truth=truth,
            overlays=overlays,
            markers=markers,
        )
        timeline = []
        for event in events:
            detail = str(event.get("count", event.get("goal", "")))
            if event.get("feedback_samples"):
                detail += (
                    f"（同一恢复计数反馈 {event['feedback_samples']} 次，"
                    f"末次 {event['last_feedback_s']:.2f}s；原始记录见 JSON）"
                )
            timeline.append(
                f"<li>{event['time_s']:.2f}s · {escape(event['kind'])} · {escape(detail)}</li>"
            )
        timeline = "".join(timeline)
        panels.append(
            f'<section class="panel"><h2>导航 · {escape(source)} / {escape(name)}</h2><p>动作：{escape(record["action_status"])} · 地图校验值：<code>{escape(record["map_sha256"][:16])}</code></p><p class="legend">紫色虚线：规划路径 · 蓝色：独立实际轨迹 · 绿色：目标 · 红色：碰撞／恢复计数变化</p>{plot}<h3>事件时间线</h3><p><a href="{escape(source)}/{escape(name)}/events.json">完整原始事件 JSON</a>（同一恢复计数的重复反馈已收起）</p><ol>{timeline or "<li>没有记录到恢复／碰撞事件；观测完整性见验收项。</li>"}</ol></section>'
        )
    return "".join(panels)


def trajectory_plot(result):
    data = result.metrics.get("trajectory", {})
    times, errors = data.get("error_times", []), data.get("errors_m", [])
    if len(times) < 2:
        return ""
    width, height = 900, 180
    upper = max(
        max(errors) * 1.1,
        result.provenance.get("config", {}).get("thresholds", {}).get("ate_rmse_m", 0.75),
    )
    points = " ".join(
        f"{(t - times[0]) / (times[-1] - times[0]) * width:.2f},{height - e / upper * height:.2f}"
        for t, e in zip(times, errors)
    )
    return f'<section class="panel"><h2>逐时刻轨迹误差</h2><p>纵轴：0–{upper:.2f} 米 · 横轴：{times[0]:.2f}–{times[-1]:.2f} 秒。ATE 门槛用于全程 RMSE。</p><svg role="img" aria-label="轨迹误差时间曲线" viewBox="0 0 {width} {height}"><polyline fill="none" stroke="#da6a24" stroke-width="2" points="{points}"/></svg></section>'


def write_report(result, output, figure=""):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    data = result.as_dict()
    dump_json(output / "result.json", data)
    suite = ET.Element("testsuite", name=result.scenario, tests=str(len(result.checks)))
    failures = errors = skipped = 0
    for check in result.checks:
        item = ET.SubElement(suite, "testcase", name=check.id, classname=check.section)
        if check.status == "environment_error":
            ET.SubElement(item, "error", message=check.reason).text = str(check.value)
            errors += 1
        elif check.status == "fail" or (check.status == "insufficient" and check.required):
            ET.SubElement(
                item, "failure", message=f"{check.status}: {check.reason}"
            ).text = json.dumps(check.__dict__, ensure_ascii=False)
            failures += 1
        elif check.status == "insufficient":
            ET.SubElement(item, "skipped", message=check.reason)
            skipped += 1
    suite.attrib.update(failures=str(failures), errors=str(errors), skipped=str(skipped))
    ET.ElementTree(suite).write(output / "junit.xml", encoding="utf-8", xml_declaration=True)

    def pretty(value):
        return escape(json.dumps(value, ensure_ascii=False, indent=2))

    cards = "".join(
        f'<div class="card {state}"><span>{name}</span><strong>{LABELS[state]}</strong></div>'
        for section, name in (
            ("mapping", "建图"),
            ("navigation", "导航"),
            ("integration", "系统集成"),
        )
        for state in [data["sections"][section]]
    )
    rows = []
    for check in result.checks:
        links = " ".join(
            f'<a href="{escape(p, quote=True)}">{escape(p)}</a>' for p in check.evidence
        )
        detail = f"<p>{escape(check.reason)}</p><pre>实测值: {pretty(check.value)}\n阈值: {pretty(check.threshold)}\n时间（秒）: {pretty(check.window_s)}</pre><p>{links}</p>"
        rows.append(
            f'<details class="check {check.status}" data-state="{check.status}"><summary><b>{escape(check.id)}</b><span>{LABELS[check.status]}{" · 非必测" if not check.required else ""}</span></summary>{detail}</details>'
        )
    logs = []
    for path in sorted(output.rglob("*.log")):
        relative = path.relative_to(output).as_posix()
        excerpt = path.read_text(errors="replace")[-16000:]
        logs.append(
            f"<details><summary>{escape(relative)}</summary><pre>{escape(excerpt)}</pre></details>"
        )
    stages = "".join(
        f"<li><b>{escape(s['name'])}</b> · {escape(s['status'])} · {s.get('elapsed_s', 0):.2f}s</li>"
        for s in result.stages
    )
    page = f"""<!doctype html><html lang="zh-CN"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>SLAM 验证 · {escape(result.scenario)}</title><style>
*{{box-sizing:border-box}}body{{margin:0;background:#f3f5f7;color:#1e3040;font:15px system-ui,sans-serif}}main{{max-width:1180px;margin:auto;padding:42px 28px}}h1{{font-size:32px;margin:8px 0}}h2{{font-size:21px;margin-top:30px}}.eyebrow{{letter-spacing:2px;font-size:12px;color:#597286}}.meta{{color:#536a7b;line-height:1.8}}.cards{{display:grid;grid-template-columns:repeat(3,1fr);gap:14px;margin:24px 0}}.card{{background:white;border-radius:10px;padding:20px;border-top:4px solid #94a3b8}}.card span{{display:block;color:#52667a}}.card strong{{display:block;font-size:25px;margin-top:8px}}.pass{{border-color:#18876d}}.fail{{border-color:#ce4f36}}.insufficient{{border-color:#cf9a2e}}.environment_error{{border-color:#8a61ad}}.panel{{background:white;border-radius:12px;padding:24px;margin-top:18px}}svg{{width:100%;height:auto;border:1px solid #e4eaf0}}.check{{border-left:4px solid;margin:9px 0;background:#f8fafc;padding:12px 16px;border-radius:4px}}summary{{cursor:pointer}}summary span{{float:right}}pre{{white-space:pre-wrap;overflow-wrap:anywhere;font:12px/1.6 ui-monospace,monospace;background:#edf2f6;padding:14px;border-radius:6px}}a{{color:#096ba4}}button{{padding:7px 13px;background:#fff;border:1px solid #b6c4d0;border-radius:5px;cursor:pointer}}label{{margin-right:18px}}.legend{{color:#546b7b;line-height:2}}@media(max-width:650px){{main{{padding:20px 12px}}.cards{{grid-template-columns:1fr}}summary span{{float:none;display:block}}}}
</style><main><div class="eyebrow">ROBOT VALIDATION / EVIDENCE REPORT</div><h1>{escape(result.scenario)} · {LABELS[result.status]}</h1>
<p class="meta">后端：{escape(result.backend)} · 随机种子：{result.seed} · <a href="result.json">JSON</a> · <a href="junit.xml">JUnit XML</a><br>本地射线示例仅验证建图与评估工具。ROS 2 / Gazebo 闭环验收以对应后端的实际证据为准。</p>
<div class="cards">{cards}</div><section class="panel"><h2>地图与轨迹</h2><div class="legend">深灰：生成地图障碍 · 青色：参考墙体 · 橙色：算法轨迹 · 蓝色：独立真值</div>
<p><label><input type="checkbox" checked onchange="document.querySelectorAll('.reference').forEach(x=>x.style.display=this.checked?'':'none')">参考地图</label><label><input type="checkbox" checked onchange="document.querySelectorAll('.truth').forEach(x=>x.style.display=this.checked?'':'none')">参考轨迹</label></p>{figure or "<p>本阶段未生成可显示的地图。查看阶段记录与日志。</p>"}</section>
{trajectory_plot(result)}{navigation_panels(output)}
<section class="panel"><h2>验收明细</h2><button onclick="document.querySelectorAll('.check').forEach(x=>x.hidden=x.dataset.state==='pass')">只看未通过项</button> <button onclick="document.querySelectorAll('.check').forEach(x=>x.hidden=false)">显示全部</button>{"".join(rows)}</section>
<section class="panel"><h2>阶段与故障时间</h2><ol>{stages}</ol><details><summary>指标与对齐信息</summary><pre>{pretty(result.metrics)}</pre></details><details><summary>来源与运行条件</summary><pre>{pretty(result.provenance)}</pre></details>{"".join(logs)}</section></main></html>"""
    (output / "report.html").write_text(page)
    return output / "report.html"
