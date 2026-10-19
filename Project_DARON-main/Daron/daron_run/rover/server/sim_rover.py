#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_rover.py — 하드웨어 없이 서버 경로 추종 루프를 돌려 보는 운동학 시뮬 로버

radxa.py 대신 띄운다. path_follower 가 보내는 자율 지령을 받아 움직이고,
맵 위치(localization 흉내)와 휠 오도메트리를 발행한다. 운동학 모델은
path_tracking.SimRover (셀프테스트와 같은 것).

구독  /rover/auto/drive_rpm, /rover/auto/steer_cmd   (auto_timeout 동안 없으면 정지)
발행  /localization/pose  (map [m], frame 'map', header.stamp = 지금, pose_hz, 잡음)
      /rover/odom         (odom_hz)
      /rover/diagnostics  (wheelbase_mm 만)

실행
----
  python3 sim_rover.py --ros-args -p wheel_diameter_m:=0.1 -p gear_ratio:=10.0
  python3 path_follower.py --ros-args --params-file config/follower.yaml \\
          -p wheel_diameter_m:=0.1 -p gear_ratio:=10.0 -p min_rpm:=100.0 -p max_rpm:=2000.0 \\
          -p start_enabled:=true
  python3 send_path_csv.py path_detour.csv     # 또는 python3 send_path.py --shape s_curve
"""

from __future__ import annotations

import math
import random
import sys
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from path_tracking import DEFAULT_WHEELBASE_M, SimRover, TrackerConfig


def _quat(yaw):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class SimRoverNode(Node):

    def __init__(self):
        super().__init__('sim_rover')
        self.declare_parameters('', [
            ('x0', 0.0), ('y0', 0.0), ('yaw0_deg', 0.0),      # map [m]
            ('wheel_diameter_m', 0.0), ('gear_ratio', 0.0),
            ('wheelbase_m', DEFAULT_WHEELBASE_M),
            ('pose_hz', 10.0), ('pose_noise_m', 0.01), ('pose_noise_deg', 1.0),
            ('odom_hz', 20.0),
            ('auto_timeout', 0.5),
        ])
        g = lambda n: self.get_parameter(n).value
        k = TrackerConfig(wheel_diameter_m=g('wheel_diameter_m'),
                          gear_ratio=g('gear_ratio')).rpm_per_mps
        if k is None:
            raise SystemExit("wheel_diameter_m / gear_ratio 를 주십시오 (follower 와 같은 값).")
        self.sim = SimRover(g('x0'), g('y0'), math.radians(g('yaw0_deg')),
                            wheelbase_m=g('wheelbase_m'), rpm_per_mps=k)
        self.noise_m, self.noise_yaw = g('pose_noise_m'), math.radians(g('pose_noise_deg'))
        self.auto_timeout = g('auto_timeout')
        self.rpm, self.steer, self.t_cmd = 0.0, 0.0, 0.0

        self.create_subscription(Float64, '/rover/auto/drive_rpm', self._on_rpm, 10)
        self.create_subscription(Float64, '/rover/auto/steer_cmd', self._on_steer, 10)
        self.pub_pose = self.create_publisher(PoseStamped, '/localization/pose', 10)
        self.pub_odom = self.create_publisher(Odometry, '/rover/odom', 10)
        self.pub_diag = self.create_publisher(DiagnosticArray, '/rover/diagnostics', 10)

        self._t = time.monotonic()
        self.create_timer(0.005, self._sim_step)
        self.create_timer(1.0 / g('pose_hz'), self._pub_pose)
        self.create_timer(1.0 / g('odom_hz'), self._pub_odom)
        self.create_timer(0.1, self._pub_diag)
        self.get_logger().info(f"시뮬 로버 시작 ({g('x0'):.2f}, {g('y0'):.2f}, "
                               f"{g('yaw0_deg'):.1f} deg), rpm/(m/s) {k:.1f}")

    def _on_rpm(self, msg):
        self.rpm, self.t_cmd = float(msg.data), time.monotonic()

    def _on_steer(self, msg):
        self.steer, self.t_cmd = float(msg.data), time.monotonic()

    def _sim_step(self):
        now = time.monotonic()
        dt, self._t = now - self._t, now
        stale = now - self.t_cmd > self.auto_timeout
        self.sim.step(0.0 if stale else self.rpm, 0.0 if stale else self.steer, dt)

    def _pub_pose(self):
        s = self.sim
        x = s.x + random.gauss(0, self.noise_m)
        y = s.y + random.gauss(0, self.noise_m)
        yaw = s.yaw + random.gauss(0, self.noise_yaw)
        m = PoseStamped()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = 'map'
        m.pose.position.x, m.pose.position.y = x, y
        q = _quat(yaw)
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
        self.pub_pose.publish(m)

    def _pub_odom(self):
        s = self.sim
        m = Odometry()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id, m.child_frame_id = 'odom', 'base_link'
        m.pose.pose.position.x, m.pose.pose.position.y = s.odom[0], s.odom[1]
        q = _quat(s.odom[2])
        o = m.pose.pose.orientation
        o.x, o.y, o.z, o.w = q
        m.twist.twist.linear.x = s.v
        m.twist.twist.angular.z = s.v * math.tan(math.radians(s.steer)) / s.L
        self.pub_odom.publish(m)

    def _pub_diag(self):
        st = DiagnosticStatus(name='sim_rover', message='SIM', level=DiagnosticStatus.OK)
        st.values = [KeyValue(key='wheelbase_mm', value=f"{self.sim.L * 1e3:.2f}")]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.pub_diag.publish(arr)


def main(argv=None):
    rclpy.init(args=argv)
    node = SimRoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
