import pathlib
import sys
import unittest

ROOT = pathlib.Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src/hc_realman_driver"))

from hc_realman_driver.protocol import (decode_registers, encode_registers,
                                        normalized_to_register)


class ProtocolTest(unittest.TestCase):
    def test_command_register_layout(self):
        self.assertEqual(encode_registers(127, 10, 20, 30, 40),
                         [127, 10, 20, 30, 40, 1])

    def test_normalized_mapping_and_status(self):
        self.assertEqual(normalized_to_register(0.0), 0)
        self.assertEqual(normalized_to_register(0.5), 128)
        self.assertEqual(normalized_to_register(1.0), 255)
        status = decode_registers([0, 1, 64, 12, 34])
        self.assertEqual(status.fault, 0)
        self.assertEqual(status.state, 1)
        self.assertEqual(status.normalized_position, 64 / 255)

    def test_invalid_values_are_rejected(self):
        with self.assertRaises(ValueError):
            normalized_to_register(-0.01)
        with self.assertRaises(ValueError):
            encode_registers(256, 0, 0, 0, 0)
        with self.assertRaises(ValueError):
            decode_registers([0, 1])
