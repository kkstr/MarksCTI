"""
Unit tests for the pure control/state logic in tyre_inflator.py.

The module imports hardware libraries (gpiozero, smbus2, luma, serial,
pynmea2, PIL) at import time, none of which need to be installed to test the
logic. We stub them in sys.modules before importing so the tests run on any
PC with no Raspberry Pi hardware or driver libraries present.

Run with:  python3 -m unittest discover -s tests
"""

import os
import sys
import types
import unittest
from unittest.mock import MagicMock

# --- Stub out hardware modules before importing the script under test ---
_HW_MODULES = [
    "gpiozero", "smbus2",
    "luma", "luma.core", "luma.core.interface", "luma.core.interface.serial",
    "luma.core.render", "luma.oled", "luma.oled.device",
    "serial", "pynmea2",
    "PIL", "PIL.Image", "PIL.ImageFont",
]
for _name in _HW_MODULES:
    sys.modules.setdefault(_name, MagicMock(name=_name))

# pynmea2.ParseError must be a real exception class (used in an except clause).
sys.modules["pynmea2"].ParseError = type("ParseError", (Exception,), {})

# Make the repo root importable.
sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import tyre_inflator as ti  # noqa: E402


class TestLockoutState(unittest.TestCase):
    def test_no_fix_is_failsafe_stage2(self):
        lk = ti.LockoutState()
        lk.update(None, gps_ok=False)
        self.assertEqual(lk.stage(), 2)
        self.assertFalse(lk.gps_ok())

    def test_speed_maps_to_stages(self):
        lk = ti.LockoutState()
        lk.update(0.0, True);    self.assertEqual(lk.stage(), 0)
        lk.update(19.9, True);   self.assertEqual(lk.stage(), 0)
        lk.update(20.0, True);   self.assertEqual(lk.stage(), 1)
        lk.update(80.0, True);   self.assertEqual(lk.stage(), 1)
        lk.update(80.1, True);   self.assertEqual(lk.stage(), 2)

    def test_gps_ok_but_none_speed_is_failsafe(self):
        lk = ti.LockoutState()
        lk.update(None, True)
        self.assertEqual(lk.stage(), 2)


class TestPresetManager(unittest.TestCase):
    def setUp(self):
        self.path = os.path.join(
            os.path.dirname(os.path.abspath(__file__)), "_test_presets.json")
        self._cleanup()
        self.pm = ti.PresetManager(path=self.path)

    def tearDown(self):
        if self.pm._save_timer:
            self.pm._save_timer.cancel()
        self._cleanup()

    def _cleanup(self):
        for p in (self.path, self.path + ".tmp"):
            if os.path.exists(p):
                os.remove(p)

    def test_defaults_loaded_when_no_file(self):
        self.assertEqual(self.pm.get_target("front"), ti.DEFAULT_PRESETS["ROAD"]["front"])

    def test_adjust_clamps_to_bounds(self):
        for _ in range(200):
            self.pm.adjust(ti.PSI_STEP)
        self.assertLessEqual(self.pm.get_target("front"), ti.MAX_PSI)
        for _ in range(400):
            self.pm.adjust(-ti.PSI_STEP)
        self.assertGreaterEqual(self.pm.get_target("front"), ti.MIN_PSI)

    def test_cycle_skips_custom_when_restricted(self):
        # Walk the restricted order a full lap; CUSTOM must never appear.
        seen = set()
        for _ in range(len(ti.STAGE1_ALLOWED_PRESETS) + 1):
            self.pm.cycle_preset(allowed=ti.STAGE1_ALLOWED_PRESETS)
            seen.add(self.pm.active_name())
        self.assertNotIn("CUSTOM", seen)

    def test_enforce_allowed_moves_off_custom(self):
        while self.pm.active_name() != "CUSTOM":
            self.pm.cycle_preset()
        self.pm.enforce_allowed(ti.STAGE1_ALLOWED_PRESETS)
        self.assertIn(self.pm.active_name(), ti.STAGE1_ALLOWED_PRESETS)

    def test_flush_writes_atomically(self):
        self.pm.adjust(ti.PSI_STEP)
        self.pm.flush()
        self.assertTrue(os.path.exists(self.path))
        self.assertFalse(os.path.exists(self.path + ".tmp"))


class TestWarningManager(unittest.TestCase):
    def setUp(self):
        self.wm = ti.WarningManager()

    def test_puncture_set_and_clear(self):
        self.wm.set_puncture("FL", True)
        self.assertTrue(self.wm.has_puncture("FL"))
        self.assertIn(("PUNCTURE", "FL"), self.wm.get_unpaused_active())
        self.wm.set_puncture("FL", False)
        self.assertFalse(self.wm.has_puncture("FL"))
        self.assertEqual(self.wm.get_unpaused_active(), [])

    def test_pause_silences_then_highlights(self):
        self.wm.set_puncture("FL", True)
        self.assertTrue(self.wm.pause_all())
        self.assertEqual(self.wm.get_unpaused_active(), [])   # no longer flashing
        names, comp = self.wm.get_paused_highlights()
        self.assertIn("FL", names)
        self.assertFalse(comp)

    def test_overpressure_and_deflate_stuck_are_distinct_kinds(self):
        self.wm.set_overpressure("FL", True)
        self.wm.set_deflate_stuck("RR", True)
        active = self.wm.get_unpaused_active()
        self.assertIn(("OVERPRESSURE", "FL"), active)
        self.assertIn(("DEFLATE_STUCK", "RR"), active)

    def test_compressor_failure_clears_when_a_corner_is_healthy(self):
        self.wm.set_compressor_failure(True)
        # FL has no puncture -> air is flowing somewhere -> should clear.
        self.wm.maybe_clear_compressor_failure(["FL"])
        self.assertEqual(self.wm.get_unpaused_active(), [])

    def test_compressor_failure_persists_when_all_punctured(self):
        self.wm.set_puncture("FL", True)
        self.wm.set_compressor_failure(True)
        self.wm.maybe_clear_compressor_failure(["FL"])
        self.assertIn(("COMPRESSOR_FAILURE", None), self.wm.get_unpaused_active())


if __name__ == "__main__":
    unittest.main()
