"""Import and resource-path checks after moving hardware code into packages."""
import argparse
import math
import os
from pathlib import Path
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from motor_control.xm430.axis import SuspensionAxis, ShapeConfig
from motor_control.xm430.config import SuspensionConfig, config_from_args
from motor_control.xm430.link import open_motor
from motor_control.vesc.xerun import CONFIG_DIR, DEFAULT_CONFIG_PATH


class PackageTests(unittest.TestCase):
    def test_profiles_and_vesc_config_resolve_from_another_directory(self):
        previous = Path.cwd()
        try:
            os.chdir(ROOT / 'tests')
            args = argparse.Namespace(profile='medium', set=[], port='', log_dir=None)
            cfg = config_from_args(args)
            self.assertIsInstance(cfg, SuspensionConfig)
            self.assertEqual(CONFIG_DIR, ROOT / 'config')
            self.assertEqual(DEFAULT_CONFIG_PATH, ROOT / 'config/front.json')
            self.assertTrue(DEFAULT_CONFIG_PATH.is_file())
        finally:
            os.chdir(previous)

    def test_xm430_mock_imports_without_ros_or_hardware_sdk(self):
        driver, motor, _ = open_motor(SuspensionConfig(), mock=True, verbose=False)
        try:
            self.assertEqual(driver.__name__, 'motor_control.xm430.mock')
            self.assertTrue(math.isfinite(motor.read().position_rad))
            self.assertEqual(ShapeConfig.neutral_alpha_deg, 150)
            self.assertEqual(SuspensionAxis.__module__, 'motor_control.xm430.axis')
        finally:
            motor.shutdown()


if __name__ == '__main__':
    unittest.main()
