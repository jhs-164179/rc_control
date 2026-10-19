#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS-free RC command generator. Does NOT touch motors or GPIO.

CH1: low Wi-Fi, high RC. CH2: proportional VESC speed.
CH3: proportional L12 vehicle steering angle.
CH4: proportional articulation ANGULAR VELOCITY (not +/-1 jog).
Neutral CH4 holds current pose. Pose target stays within zero +/- allowed travel.
"""
from dataclasses import dataclass
import math
import time


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))


def axis_value(us, deadband=50, center=1500, low=1000, high=2000):
    """Deadband compensated, continuous mapping [-1,1]."""
    if not math.isfinite(float(us)):
        return 0.0
    u = _clamp(float(us), low, high)
    if abs(u-center) <= deadband:
        return 0.0
    if u > center:
        return _clamp((u-center-deadband)/(high-center-deadband), 0, 1)
    return -_clamp((center-u-deadband)/(center-low-deadband), 0, 1)


@dataclass(frozen=True)
class RCCommand:
    source: str = 'wifi'
    connected: bool = False
    armed: bool = False
    drive_rpm: float = 0.0
    steer_deg: float = 0.0
    articulation_deg: float | None = None   # Motor-relative angle theta [deg], not alpha
    hold_pose: bool = False
    channels: tuple | None = None


class RCController:
    def __init__(self, *, max_rpm=100.0, max_steer_deg=20.0,
                 max_articulation_rate_dps=12.0, articulation_limit_deg=30.0,
                 drive_deadband_us=50, steer_deadband_us=50,
                 pose_deadband_us=90, pose_command_step_deg=0.75,
                 source_confirm_frames=3, min_us=850, max_us=2150,
                 max_rpm_slew_per_s=150.0):
        self.max_rpm = float(max_rpm)
        self.max_steer_deg = float(max_steer_deg)
        self.max_articulation_rate_dps = float(max_articulation_rate_dps)
        self.articulation_limit_deg = float(articulation_limit_deg)
        self.drive_deadband_us = int(drive_deadband_us)
        self.steer_deadband_us = int(steer_deadband_us)
        self.pose_deadband_us = int(pose_deadband_us)
        self.pose_command_step_deg = float(pose_command_step_deg)
        self.source_confirm_frames = int(source_confirm_frames)
        self.min_us, self.max_us = int(min_us), int(max_us)
        self.max_rpm_slew_per_s = float(max_rpm_slew_per_s)
        positive = (self.max_rpm, self.max_steer_deg, self.max_articulation_rate_dps,
                    self.articulation_limit_deg, self.pose_command_step_deg)
        if not all(math.isfinite(v) and v > 0 for v in positive):
            raise ValueError('RC limits must be finite and positive')
        if (self.source_confirm_frames < 1 or not 0 < self.min_us < 1000
                or not 2000 < self.max_us or
                not all(0 <= v < 500 for v in (self.drive_deadband_us,
                            self.steer_deadband_us, self.pose_deadband_us)) or
                not math.isfinite(self.max_rpm_slew_per_s) or self.max_rpm_slew_per_s < 0):
            raise ValueError('Invalid RC channel, deadband or slew configuration')
        self._frame_time = None
        self.source = 'wifi'
        self._candidate = None
        self._count = 0
        self.reset_motion()

    def reset_motion(self):
        self._armed = False
        self._last_t = None
        self._rpm = 0.0
        self._pose_target = None
        self._pose_last_sent = None

    def update(self, channels, *, motor_theta_deg=0.0, drivable=True, now=None, frame_time=None):
        """Return commands. Caller enforces hardware faults and VESC pair safety.

        channels must come from PPMDecoder.get_channels(timeout_s=0.3).
        motor_theta_deg = measured XM430 position minus calibrated zero, DEGREES.
        """
        now = time.monotonic() if now is None else float(now)
        valid = (channels is not None and len(channels) >= 4 and
                 all(isinstance(x, (int, float)) and math.isfinite(x) and
                     self.min_us <= x <= self.max_us for x in channels[:4]))
        if not valid:
            # Do not hand RC authority over to Wi-Fi on signal loss.
            self.reset_motion()
            self._candidate, self._count = None, 0
            self._frame_time = None
            return RCCommand(source=self.source, connected=False)

        ch = tuple(int(x) for x in channels[:4])
        ch1, ch2, ch3, ch4 = ch
        requested = 'rc' if ch1 >= 1800 else 'wifi' if ch1 <= 1200 else None
        fresh_frame = frame_time is None or frame_time != self._frame_time
        self._frame_time = frame_time
        if fresh_frame:
            self._count = self._count + 1 if requested == self._candidate else 1
            self._candidate = requested
            if requested is None:
                self._count = 0
        if requested and self._count >= self.source_confirm_frames and requested != self.source:
            self.source = requested
            self.reset_motion()

        if self.source != 'rc' or not drivable or not math.isfinite(motor_theta_deg):
            self.reset_motion()
            return RCCommand(source=self.source, connected=True, channels=ch)
        if not self._armed and abs(ch2-1500) <= self.drive_deadband_us:
            self._armed = True
            self._last_t = now
        if not self._armed:
            return RCCommand(source='rc', connected=True, channels=ch)

        dt = 0.0 if self._last_t is None else _clamp(now-self._last_t, 0.0, 0.1)
        self._last_t = now
        rpm_goal = axis_value(ch2, self.drive_deadband_us) * self.max_rpm
        if self.max_rpm_slew_per_s > 0 and dt > 0:
            delta = self.max_rpm_slew_per_s * dt
            self._rpm = _clamp(rpm_goal, self._rpm-delta, self._rpm+delta)
        elif dt > 0:
            self._rpm = rpm_goal
        steer = axis_value(ch3, self.steer_deadband_us) * self.max_steer_deg

        pose_rate = axis_value(ch4, self.pose_deadband_us) * self.max_articulation_rate_dps
        target = None
        hold = False
        if self._pose_target is None:
            # No movement on entry: initialize to measured pose, not to theta=0.
            self._pose_target = _clamp(float(motor_theta_deg),
                                       -self.articulation_limit_deg, self.articulation_limit_deg)
        if pose_rate:
            self._pose_target = _clamp(self._pose_target + pose_rate*dt,
                                       -self.articulation_limit_deg, self.articulation_limit_deg)
            if (self._pose_last_sent is None or
                abs(self._pose_target-self._pose_last_sent) >= self.pose_command_step_deg):
                target = self._pose_target
                self._pose_last_sent = target
        elif self._pose_last_sent is not None:
            hold = True  # On CH4 return to neutral, hold actual position via existing axis.
            self._pose_target = None
            self._pose_last_sent = None
        return RCCommand(source='rc', connected=True, armed=True,
                         drive_rpm=self._rpm, steer_deg=steer,
                         articulation_deg=target, hold_pose=hold, channels=ch)
