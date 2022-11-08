from __future__ import annotations

import subprocess
import time
from typing import Generator

import pytest
import rosgraph
import rospy


class _Roscore:
    def __init__(self, print_stdout: bool = True, autostart: bool = False):
        self._process = None
        self._stdout = print_stdout
        if autostart:
            self.run()

    def run(self) -> None:
        if self._stdout:
            self._process = subprocess.Popen(["roscore"])
        else:
            self._process = subprocess.Popen(["roscore"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

        while not self.is_running():
            time.sleep(0.01)

    def terminate(self) -> None:
        if self._process is None:
            raise rospy.exceptions.ROSInitException("roscore is not running.")
        self._process.terminate()
        self._process.wait()

    @classmethod
    def is_running(cls) -> bool:
        return rosgraph.masterapi.is_online()


@pytest.fixture(scope="session", autouse=True)
def fixture_session() -> Generator:
    roscore_process = _Roscore(print_stdout=False, autostart=True)
    rospy.init_node("pytest")
    yield
    rospy.signal_shutdown("finished pytest")
    roscore_process.terminate()
