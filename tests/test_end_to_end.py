import json
import signal
import subprocess
import sys
import time
import xml.etree.ElementTree as ET
import pytest
from robot_validate.model import load_config
from robot_validate.runner import supervised_run


@pytest.mark.parametrize(
    "scenario,fault,code,check,state",
    [
        ("public", "none", 2, "ATE", "insufficient"),
        ("demo", "none", 0, "ATE", "pass"),
        ("demo", "scan-gap", 1, "SCAN_GAP", "fail"),
        ("demo", "odometry-bias", 0, "ATE", "pass"),
        ("demo", "no-truth", 2, "ATE", "insufficient"),
        ("demo", "map-export", 1, "MAP_EXPORT_ERROR", "fail"),
    ],
)
def test_sensor_to_report_contract(tmp_path, scenario, fault, code, check, state):
    request = dict(
        scenario=scenario, backend="local", config=load_config(), fault=fault, bag=None, mode="sim"
    )
    exit_code, report = supervised_run(request, tmp_path / "result", 30)
    assert exit_code == code
    result = json.loads(report.with_name("result.json").read_text())
    assert next(c for c in result["checks"] if c["id"] == check)["status"] == state
    assert result["sections"]["navigation"] == "insufficient"
    assert result["sections"]["integration"] == "insufficient"
    html = report.read_text()
    assert "<html" in html and "验收明细" in html
    assert "<script src=" not in html and '<img src="http' not in html
    suite = ET.parse(report.with_name("junit.xml")).getroot()
    assert (int(suite.get("failures")) == 0) if code == 0 else (int(suite.get("failures")) >= 1)
    if fault == "map-export":
        assert report.with_name("estimated.tum").exists()
        assert not report.with_name("map.yaml").exists()
    else:
        assert "data:image/png;base64," in html


def test_timeout_preserves_partial_report(tmp_path):
    request = dict(scenario="demo", backend="local", config=load_config(), fault="none", bag=None)
    code, report = supervised_run(request, tmp_path / "timeout", 0.001)
    assert code == 124
    data = json.loads(report.with_name("result.json").read_text())
    assert data["checks"][-1]["id"] == "TIMEOUT"
    assert data["exit_code"] == 124
    assert report.exists()


def test_original_matcher_remains_a_failing_map_control(tmp_path):
    config = load_config()
    config["algorithm"] = {"local_matcher": "rmhc"}
    code, report = supervised_run(
        dict(scenario="demo", backend="local", config=config, fault="none", bag=None, mode="sim"),
        tmp_path / "legacy",
    )
    data = json.loads(report.with_name("result.json").read_text())
    assert code == 1
    assert next(c for c in data["checks"] if c["id"] == "FALSE_FREE")["status"] == "fail"


def test_user_interrupt_preserves_report(tmp_path):
    output = tmp_path / "interrupt"
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "robot_validate.cli",
            "sim",
            "--backend",
            "local",
            "--output",
            str(output),
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )
    deadline = time.monotonic() + 10
    while not (output / "processes.jsonl").exists() and time.monotonic() < deadline:
        time.sleep(0.02)
    assert (output / "processes.jsonl").exists()
    process.send_signal(signal.SIGINT)
    stdout, _ = process.communicate(timeout=10)
    assert process.returncode == 130, stdout
    data = json.loads((output / "result.json").read_text())
    assert data["checks"][-1]["id"] == "INTERRUPTED"
    assert data["exit_code"] == 130


def test_refuse_overwrite(tmp_path):
    (tmp_path / "existing.txt").write_text("preserve")
    with pytest.raises(ValueError, match="empty"):
        supervised_run({}, tmp_path)
    assert (tmp_path / "existing.txt").read_text() == "preserve"


def test_pytest_plugin_runs_case_contract(tmp_path):
    case = tmp_path / "case.json"
    case.write_text(
        json.dumps(
            dict(
                request=dict(
                    scenario="public", backend="local", fault="none", bag=None, mode="bag"
                ),
                output="artifact",
                expected_exit=2,
                expected_checks={"ATE": "insufficient"},
            )
        )
    )
    run = subprocess.run(
        [sys.executable, "-m", "pytest", "-q", str(tmp_path), "--robot-case", str(case)],
        capture_output=True,
        text=True,
        timeout=30,
    )
    assert run.returncode == 0, run.stdout + run.stderr
    assert (tmp_path / "artifact/report.html").exists()


def test_public_reference_independence_is_respected(tmp_path):
    import numpy as np
    from robot_validate.data import write_tum

    write_tum(tmp_path / "fused.tum", np.array([[0, 0, 0, 0], [1, 1, 0, 0], [2, 1, 1, 0]], float))
    cfg = load_config()
    cfg["reference"] = dict(trajectory=str(tmp_path / "fused.tum"), independent=False)
    _, report = supervised_run(
        dict(scenario="public", backend="local", config=cfg, fault="none", bag=None),
        tmp_path / "run",
    )
    data = json.loads(report.with_name("result.json").read_text())
    check = next(c for c in data["checks"] if c["id"] == "ATE")
    assert check["status"] == "insufficient"
    assert "不独立" in check["reason"]
