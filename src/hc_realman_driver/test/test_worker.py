import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/hc_realman_driver"))

from hc_realman_driver.config import load_config
from hc_realman_driver.sdk import ArmSnapshot
from hc_realman_driver.worker import ControllerWorker


class FakeSession:
    def __init__(self):
        self.commands = []

    def command_arm(self, values):
        self.commands.append(tuple(values))


class WorkerSafetyTest(unittest.TestCase):
    def make_worker(self):
        arm = load_config(str(ROOT / "config/hardware.yaml"))["parsed_arms"][0]
        worker = ControllerWorker(
            arm, None, True, True, False, 100.0, 50.0, 20.0, 0.25,
            115200, 24, (80, 80, 80, 80), 45.0, 120.0, 200,
        )
        worker.session = FakeSession()
        worker._state.arm = ArmSnapshot((0.0,) * 6, (), None)
        return worker

    def test_first_large_target_is_acceleration_bounded(self):
        worker = self.make_worker()
        worker._send_smoothed_arm((90.0,) * 6, 10.0)
        command = worker.session.commands[-1]
        self.assertTrue(all(0.0 < value < 0.001 for value in command))

    def test_configured_joint_limit_is_enforced(self):
        worker = self.make_worker()
        with self.assertRaises(ValueError):
            worker.set_arm_target((999.0,) * 6)
