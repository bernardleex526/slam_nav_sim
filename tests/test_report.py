import numpy as np
from robot_validate.data import write_tum
from robot_validate.model import Result, Check, dump_json
from robot_validate.report import write_report
from robot_validate.slam import Grid


def test_navigation_artifacts_render_with_events_and_goal(tmp_path):
    grid = Grid(np.array([[0, 0, 0], [100, 100, 100]], np.int8), 1, (-3, -3, 0))
    grid.save(tmp_path, "reference-map")
    grid.save(tmp_path)
    phase = tmp_path / "reference" / "healthy"
    phase.mkdir(parents=True)
    write_tum(phase / "ground-truth-raw.tum", np.array([[0, -2, -2, 0], [1, -1, -2, 0]], float))
    dump_json(phase / "paths.json", [dict(points=[[-2, -2], [-1, -2]])])
    dump_json(phase / "events.json", [dict(kind="recovery", time_s=0.5, count=1)])
    dump_json(
        tmp_path / "navigation-comparison.json",
        [
            dict(
                map_source="reference",
                name="healthy",
                goal=[-1, -2, 0],
                map_sha256="abc",
                action_status="succeeded",
            )
        ],
    )
    result = Result(
        "report-fixture", "unit-test", 1, checks=[Check("render", "navigation", "pass")]
    )
    report = write_report(result, tmp_path).read_text()
    assert "导航 · reference / healthy" in report
    assert "recovery" in report and "<circle" in report
    assert 'stroke-dasharray="5 4"' in report


def test_cli_rejects_unsupported_fault_before_claiming_injection():
    import pytest
    from robot_validate.cli import main

    with pytest.raises(SystemExit) as error:
        main(["sim", "--navigation-only", "--fault", "scan-gap"])
    assert error.value.code == 2
