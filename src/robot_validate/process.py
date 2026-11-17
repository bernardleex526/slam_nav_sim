"""Own process groups only: no global pkill, no ROS daemon shutdown."""

import os
import json
from pathlib import Path
import signal
import subprocess
import time


class ProcessScope:
    def __init__(self, directory, env=None):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=True)
        self.env = dict(os.environ, **(env or {}))
        self.children = []

    def start(self, name, argv):
        log = (self.directory / f"{name}.log").open("wb")
        try:
            process = subprocess.Popen(
                argv, stdout=log, stderr=subprocess.STDOUT, env=self.env, start_new_session=True
            )
        except BaseException:
            log.close()
            raise
        self.children.append((process, log))
        ledger = self.env.get("ROBOT_VALIDATE_PROCESS_LEDGER")
        if ledger:
            descriptor = os.open(ledger, os.O_CREAT | os.O_WRONLY | os.O_APPEND, 0o600)
            try:
                os.write(
                    descriptor, (json.dumps({"pid": process.pid, "argv": argv}) + "\n").encode()
                )
            finally:
                os.close(descriptor)
        return process

    def wait(self, process, timeout):
        return process.wait(timeout=timeout)

    @staticmethod
    def _signal(process, sig):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            pass

    def close(self):
        # Signal whole groups even when their leader has already exited.
        for sig, grace in ((signal.SIGINT, 1.0), (signal.SIGTERM, 1.0), (signal.SIGKILL, 0)):
            for process, _ in reversed(self.children):
                self._signal(process, sig)
            deadline = time.monotonic() + grace
            for process, _ in self.children:
                try:
                    process.wait(timeout=max(0, deadline - time.monotonic()))
                except subprocess.TimeoutExpired:
                    pass
        for process, log in self.children:
            process.wait()
            log.close()
        self.children.clear()

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()


def cleanup_ledger(path):
    """The outer supervisor also owns child groups created by a killed worker."""
    path = Path(path)
    if not path.exists():
        return
    for line in path.read_text().splitlines():
        pid = json.loads(line)["pid"]
        try:
            os.killpg(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    time.sleep(0.1)
    for line in path.read_text().splitlines():
        pid = json.loads(line)["pid"]
        try:
            os.killpg(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
