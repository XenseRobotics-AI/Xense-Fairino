import os
from pathlib import Path
import sys
import tempfile
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts/dxhand"))
import ports  # noqa: E402

A, B = "304F38533035", "3050383D3035"


class PortsTest(unittest.TestCase):
    def setUp(self):
        tmp = tempfile.TemporaryDirectory()
        self.addCleanup(tmp.cleanup)
        self.root = Path(tmp.name)
        self.by_id = self.root / "by-id"
        self.by_id.mkdir()
        self.store = self.root / "hand_ports.json"
        self.probed = []

    def attach(self, serial):
        dev = self.root / ("tty" + serial)
        dev.touch()
        link = self.by_id / f"usb-STMicroelectronics_STM32_Virtual_ComPort_{serial}-if00"
        link.symlink_to(dev)
        return str(link)

    def probe(self, replies):
        def run(path):
            self.probed.append(path)
            return replies.get(path)
        return run

    def resolve(self, wanted, replies):
        return ports.resolve(wanted, self.store, self.probe(replies), self.by_id)

    def test_two_hands_identified_by_firmware_then_remembered(self):
        a, b = self.attach(A), self.attach(B)
        got = self.resolve({"left": "auto", "right": "auto"}, {a: 1, b: 2})
        self.assertEqual(got, {"left": b, "right": a})
        self.assertEqual(ports.load(self.store), {A: "right", B: "left"})
        self.probed.clear()
        self.assertEqual(self.resolve({"left": "auto", "right": "auto"}, {}), got)
        self.assertEqual(self.probed, [])  # Remembered: firmware silence is fine.

    def test_single_unknown_hand_is_used(self):
        a = self.attach(A)
        self.assertEqual(self.resolve({"right": "auto"}, {}), {"right": a})

    def test_two_unknown_silent_hands_are_refused(self):
        self.attach(A), self.attach(B)
        with self.assertRaisesRegex(RuntimeError, "无法自动识别"):
            self.resolve({"right": "auto"}, {})

    def test_single_device_known_as_other_side_is_refused(self):
        self.attach(A)
        ports.remember(self.store, A, "left")
        with self.assertRaisesRegex(RuntimeError, "无法自动识别"):
            self.resolve({"right": "auto"}, {})

    def test_explicit_port_is_excluded_from_auto(self):
        a, b = self.attach(A), self.attach(B)
        got = self.resolve({"left": a, "right": "auto"}, {})
        self.assertEqual(got, {"left": a, "right": b})

    def test_no_devices(self):
        with self.assertRaisesRegex(RuntimeError, "未找到"):
            self.resolve({"right": "auto"}, {})


if __name__ == "__main__":
    unittest.main()
