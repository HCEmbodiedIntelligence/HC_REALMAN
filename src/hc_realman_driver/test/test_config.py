import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/hc_realman_driver"))

from hc_realman_driver.config import load_config


class ConfigTest(unittest.TestCase):
    def test_default_hardware_config_is_valid(self):
        config = load_config(str(ROOT / "config/hardware.yaml"))
        self.assertEqual(len(config["parsed_arms"]), 2)
        self.assertTrue(all(len(arm.joints) == 6 for arm in config["parsed_arms"]))
        self.assertEqual([hand.device_id for hand in config["parsed_hands"]], [1, 1])
        self.assertEqual(config["hands"]["baudrate"], 115200)
