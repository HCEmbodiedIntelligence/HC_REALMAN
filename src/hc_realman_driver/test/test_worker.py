import pathlib
import sys
import time
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/hc_realman_driver"))

from hc_realman_driver.config import load_config
from hc_realman_driver.sdk import ArmSnapshot, RealManSession, arm_has_fault
from hc_realman_driver.worker import ControllerWorker


class FakeSession:
    def __init__(self):
        self.commands = []
        self.connected = False

    def connect(self):
        self.connected = True

    def start_realtime_state(self, handler, target_ip, port, cycle_ms):
        del target_ip, port, cycle_ms
        handler(ArmSnapshot((0.0,) * 6, (), {"err_len": 1, "err": ["0"]}),
                time.monotonic())

    def disconnect(self):
        self.connected = False

    def slow_stop(self):
        return None

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

    def test_small_step_has_no_exponential_tail_or_overshoot(self):
        worker = self.make_worker()
        now = 10.0
        for _ in range(30):
            worker._send_smoothed_arm((1.0,) * 6, now)
            now += 0.01
        command = worker.session.commands[-1]
        self.assertTrue(all(abs(value - 1.0) < 0.01 for value in command))
        self.assertTrue(all(0.0 <= value <= 1.0 for sent in worker.session.commands for value in sent))

    def test_configured_joint_limit_is_enforced(self):
        worker = self.make_worker()
        with self.assertRaises(ValueError):
            worker.set_arm_target((999.0,) * 6)

    def test_clear_targets_defers_hold_to_command_thread(self):
        worker = self.make_worker()
        worker.session.connected = True
        worker._streaming = True
        worker.clear_targets()
        self.assertEqual(worker.session.commands, [])
        self.assertTrue(worker._hold_requested)

    def test_command_loop_runs_without_tcp_state_polling(self):
        worker = self.make_worker()
        worker.set_arm_target((1.0,) * 6)
        worker.set_motion_gate(True)
        worker.start()
        time.sleep(0.12)
        worker.stop()
        self.assertGreaterEqual(len(worker.session.commands), 7)

    def test_hold_resets_command_gap_between_streams(self):
        worker = self.make_worker()
        worker.session.connected = True
        worker._streaming = True
        worker._last_send_stamp = 1.0
        worker._hold_once()
        self.assertEqual(worker._last_send_stamp, 0.0)


class ArmFaultTest(unittest.TestCase):
    def test_api2_zero_sentinel_is_not_a_fault(self):
        self.assertFalse(arm_has_fault({"err_len": 1, "err": ["0"]}))
        self.assertFalse(arm_has_fault({"err_len": 1, "err": [0]}))

    def test_nonzero_or_missing_error_code_is_a_fault(self):
        self.assertTrue(arm_has_fault({"err_len": 1, "err": ["0x10"]}))
        self.assertTrue(arm_has_fault({"err_len": 1, "err": []}))

    def test_empty_error_payload_is_not_a_fault(self):
        self.assertFalse(arm_has_fault(None))
        self.assertFalse(arm_has_fault({"err_len": 0, "err": []}))


class ArmReadTest(unittest.TestCase):
    def test_transient_read_error_is_retried_once(self):
        class FakeArm:
            def __init__(self):
                self.calls = 0

            def rm_get_current_arm_state(self):
                self.calls += 1
                if self.calls == 1:
                    return -2, {}
                return 0, {"joint": [0.0] * 6, "pose": [], "err": ["0"]}

        arm_cfg = load_config(str(ROOT / "config/hardware.yaml"))["parsed_arms"][0]
        session = RealManSession(arm_cfg, None, True)
        session._arm = FakeArm()
        state = session.read_arm()
        self.assertEqual(session._arm.calls, 2)
        self.assertEqual(state.joint_deg, (0.0,) * 6)

    def test_canfd_stream_uses_qualified_follow_mode(self):
        class FakeArm:
            def __init__(self):
                self.call = None

            def rm_movej_canfd(self, joints, follow, **options):
                self.call = (joints, follow, options)
                return 0

        arm_cfg = load_config(str(ROOT / "config/hardware.yaml"))["parsed_arms"][0]
        session = RealManSession(arm_cfg, None, True)
        session._arm = FakeArm()
        session.command_arm((1.0, 2.0, 3.0, 4.0, 5.0, 6.0))
        self.assertEqual(session._arm.call[0], [1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 0.0])
        self.assertIs(session._arm.call[1], arm_cfg.high_follow)
        self.assertEqual(session._arm.call[2], {})
