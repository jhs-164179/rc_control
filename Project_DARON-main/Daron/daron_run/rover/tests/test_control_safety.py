"""Hardware-free regression tests: python -m unittest discover -s tests -v."""
import ast
import math
from pathlib import Path
import sys
import threading
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from motor_control.l12.driver import L12Pair, L12Error, neutral_position, solve_steering
from rc_control.rc_controller import RCCommand, RCController
from rc_control.ppm_decoder import PPMDecoder


class SteeringTests(unittest.TestCase):
    def test_invalid_commands_neutralize_both_actuators(self):
        pair = L12Pair(simulate=True)
        for bad in (float('nan'), float('inf'), -float('inf'), 80, -80):
            pair.set_steering(15, -30)
            with self.assertRaises(L12Error):
                pair.set_steering(bad, -30)
            self.assertAlmostEqual(pair.front_mm, neutral_position())
            self.assertAlmostEqual(pair.rear_mm, neutral_position())

    def test_invalid_lengths_neutralize_pair(self):
        pair = L12Pair(simulate=True)
        for bad in (float('nan'), float('inf'), -1, 31):
            pair.set_steering(15, -30)
            with self.assertRaises(L12Error):
                pair.set_positions(10, bad)
            self.assertEqual((pair.front_mm, pair.rear_mm), (neutral_position(),) * 2)

    def test_inversion_and_zero_curvature(self):
        self.assertEqual(solve_steering(0, -30).curvature_per_m, 0)
        a = solve_steering(20, -30, True)
        b = solve_steering(-20, -30, False)
        self.assertAlmostEqual(a.front_mm, b.front_mm)
        self.assertAlmostEqual(a.rear_mm, b.rear_mm)


class RCTests(unittest.TestCase):
    def enter_rc(self, ctrl, throttle=1500):
        for i in range(3):
            cmd = ctrl.update([2000, throttle, 1500, 1500], now=i * .05, frame_time=i)
        return cmd

    def test_signal_loss_keeps_rc_authority_and_requires_neutral(self):
        c = RCController()
        self.assertTrue(self.enter_rc(c).armed)
        self.assertGreater(c.update([2000, 2000, 1800, 1500], now=.2).drive_rpm, 0)
        lost = c.update(None, now=.25)
        self.assertEqual(lost.source, 'rc')
        self.assertEqual((lost.drive_rpm, lost.steer_deg), (0, 0))
        self.assertFalse(c.update([2000, 2000, 1800, 1500], now=.3).armed)
        self.assertTrue(c.update([2000, 1500, 1500, 1500], now=.35).armed)

    def test_same_frame_does_not_confirm_mode(self):
        c = RCController()
        for _ in range(10):
            cmd = c.update([2000, 1500, 1500, 1500], frame_time=1)
        self.assertEqual(cmd.source, 'wifi')
        c.update([2000, 1500, 1500, 1500], frame_time=2)
        self.assertEqual(c.update([2000, 1500, 1500, 1500], frame_time=3).source, 'rc')

    def test_invalid_channel_stops_and_disarms(self):
        for bad in (float('nan'), float('inf'), 500, 2500):
            c = RCController()
            self.enter_rc(c)
            cmd = c.update([2000, 1600, bad, 1500])
            self.assertFalse(cmd.connected)
            self.assertFalse(cmd.armed)
            self.assertEqual(cmd.steer_deg, 0)

    def test_pose_rate_and_release(self):
        c = RCController()
        self.enter_rc(c)
        cmd = c.update([2000, 1500, 1500, 2000], now=.2)
        self.assertGreater(cmd.articulation_deg, 0)
        self.assertTrue(c.update([2000, 1500, 1500, 1500], now=.25).hold_pose)
        self.assertFalse(c.update([2000, 1500, 1500, 1500], now=.30).hold_pose)


class DecoderTests(unittest.TestCase):
    def test_malformed_frame_invalidates_last_good_frame(self):
        d = PPMDecoder.__new__(PPMDecoder)
        d._lock = threading.Lock()
        d._latest = None
        d._last_frame_at = None
        d._pending = []
        d._synced = False
        d._valid_count = d._invalid_count = 0
        d.num_channels, d.sync_us, d.min_us, d.max_us = 4, 3000, 800, 2200
        now = time.monotonic()
        for pulse in (4000, 2000, 1500, 1500, 1500, 4000):
            d._process(pulse, now)
        self.assertIsNotNone(d.get_channels())
        d._process(500, now)
        self.assertIsNone(d.get_channels())


class CallbackTests(unittest.TestCase):
    """Exercise actual callback bodies without importing unavailable ROS libraries."""
    def test_bad_ros_commands_replace_previous_steering(self):
        tree = ast.parse((ROOT / 'rover_node/node.py').read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RoverNode')
        methods = [n for n in cls.body if isinstance(n, ast.FunctionDef)
                   and n.name in ('_on_steer', '_on_auto_steer')]
        for method in methods:
            method.args.args[1].annotation = None
        ns = {'math': math, 'time': time}
        exec(compile(ast.Module(body=methods, type_ignores=[]), '<callbacks>', 'exec'), ns)
        for method, attr in (('_on_steer', '_steer_cmd'), ('_on_auto_steer', '_auto_steer')):
            for bad in (float('nan'), float('inf'), -21, 21):
                node = SimpleNamespace(_rc=RCCommand(), max_steer=20, _touch=Mock(),
                                       get_logger=Mock(return_value=Mock()))
                setattr(node, attr, 15)
                ns[method](node, SimpleNamespace(data=bad))
                self.assertEqual(getattr(node, attr), 0)
                node.get_logger().error.assert_called_once()

    def test_rc_source_switch_clears_ros_commands_and_refreshes_watchdog(self):
        tree = ast.parse((ROOT / 'rover_node/node.py').read_text(encoding='utf-8'))
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RoverNode')
        method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == '_update_rc_input')
        ns = {'math': math, 'time': time}
        exec(compile(ast.Module(body=[method], type_ignores=[]), '<rc-adapter>', 'exec'), ns)
        node = SimpleNamespace(
            ppm=Mock(), axis=Mock(), art_zero=0, rc_controller=RCController(),
            max_steer=20, _rc=RCCommand(),
            _rc_pose_active=False, _touch=Mock(),
            get_logger=Mock(return_value=Mock()), _steer_cmd=15, _auto_rpm=100,
            _auto_steer=15, _drive_cmd=1, _jog_cmd=1, _auto_rpm_t=1, _auto_steer_t=1)
        node.axis.snapshot.return_value = dict(position_deg=-30, ready=True,
                                              drivable=True, torque_on=True)
        for i in range(3):
            node.ppm.snapshot.return_value = ([2000, 1500, 1500, 1500], i)
            ns['_update_rc_input'](node)
        self.assertEqual(node._rc.source, 'rc')
        self.assertEqual((node._steer_cmd, node._auto_rpm, node._drive_cmd), (0, 0, 0))
        self.assertEqual(node._auto_rpm_t, 0)
        node._touch.assert_called_once()
        node.ppm.snapshot.return_value = (None, 3)
        ns['_update_rc_input'](node)
        self.assertEqual(node._rc.source, 'rc')
        self.assertEqual((node._rc.drive_rpm, node._rc.steer_deg), (0, 0))


if __name__ == '__main__':
    unittest.main()
