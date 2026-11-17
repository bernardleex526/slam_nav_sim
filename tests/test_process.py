import os
import subprocess
import sys
import time
import pytest
from robot_validate.process import ProcessScope
from robot_validate.process import cleanup_ledger
import json


def test_timeout_cleans_owned_process_and_keeps_unrelated(tmp_path):
    unrelated = subprocess.Popen([sys.executable, "-c", "import time; time.sleep(60)"])
    try:
        with ProcessScope(tmp_path) as scope:
            child = scope.start("sleep", [sys.executable, "-c", "import time; time.sleep(60)"])
            with pytest.raises(subprocess.TimeoutExpired):
                scope.wait(child, 0.1)
        assert child.poll() is not None
        assert unrelated.poll() is None
    finally:
        unrelated.terminate()
        unrelated.wait()


def test_descendant_cleaned_after_leader_exits(tmp_path):
    pidfile = tmp_path / "pid"
    code = "import subprocess,sys; from pathlib import Path; p=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)']); Path(sys.argv[1]).write_text(str(p.pid))"
    with ProcessScope(tmp_path) as scope:
        scope.wait(scope.start("parent", [sys.executable, "-c", code, str(pidfile)]), 5)
    pid = int(pidfile.read_text())
    for _ in range(30):
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            break
        time.sleep(0.05)
    else:
        state = subprocess.check_output(["ps", "-o", "stat=", "-p", str(pid)], text=True)
        assert "Z" in state  # terminated but awaiting OS reaping


def test_ledger_cleans_group_when_session_leader_already_exited(tmp_path):
    pidfile = tmp_path / "child.pid"
    code = "import subprocess,sys; from pathlib import Path; p=subprocess.Popen([sys.executable,'-c','import time; time.sleep(60)']); Path(sys.argv[1]).write_text(str(p.pid))"
    leader = subprocess.Popen([sys.executable, "-c", code, str(pidfile)], start_new_session=True)
    leader.wait(timeout=5)
    ledger = tmp_path / "processes.jsonl"
    ledger.write_text(json.dumps({"pid": leader.pid}) + "\n")
    cleanup_ledger(ledger)
    pid = int(pidfile.read_text())
    for _ in range(30):
        try:
            os.kill(pid, 0)
        except ProcessLookupError:
            return
        time.sleep(0.05)
    state = subprocess.check_output(["ps", "-o", "stat=", "-p", str(pid)], text=True)
    assert "Z" in state
