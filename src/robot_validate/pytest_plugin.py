"""One pytest entry point for JSON case contracts, both bag and simulation."""

import json
from pathlib import Path
import pytest
from .model import load_config
from .runner import supervised_run


def pytest_addoption(parser):
    group = parser.getgroup("robot validation")
    group.addoption("--robot-case", action="append", default=[], help="JSON case contract path")


def pytest_collection_modifyitems(session, config, items):
    for case in config.getoption("--robot-case"):
        items.append(RobotCase.from_parent(session, name=Path(case).stem, case_path=case))


class RobotCase(pytest.Item):
    def __init__(self, *args, case_path, **kwargs):
        super().__init__(*args, **kwargs)
        self.case_path = Path(case_path).resolve()

    def runtest(self):
        contract = json.loads(self.case_path.read_text())
        request = contract["request"]
        config_path = request.pop("config_path", None)
        request["config"] = load_config(
            self.case_path.parent / config_path if config_path else None
        )
        if request.get("generated_map"):
            request["generated_map"] = str(
                (self.case_path.parent / request["generated_map"]).resolve()
            )
        output = self.case_path.parent / contract["output"]
        code, report = supervised_run(request, output, contract.get("timeout_s", 300))
        expected = contract.get("expected_exit", 0)
        assert code == expected, f"exit {code}, expected {expected}; report: {report}"
        data = json.loads((output / "result.json").read_text())
        statuses = {c["id"]: c["status"] for c in data["checks"]}
        for check, status in contract.get("expected_checks", {}).items():
            assert statuses.get(check) == status, f"{check}: {statuses.get(check)} != {status}"

    def reportinfo(self):
        return self.case_path, 0, self.name
