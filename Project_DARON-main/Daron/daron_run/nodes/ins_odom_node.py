#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
ins_odom_node — INS 단독 이중적분 궤적 (ZUPT 없음) → nav_msgs/Odometry.
배치 경로: ~/daron_ros/ins_odom_node.py
작성: 2026-09-21
환경: 시스템 python3 (numpy 있음). 계산은 lib/ins_daron.py — 논문용 `~/analysis_ins_only.py` 의
      실시간 이식본(ksy_test 에서 가져옴, 같은 bag 이면 1e-6 m 로 일치).

[무엇인가]
  /imu/data (CV7 raw, 논문 INS 선과 같은 입력) 를 받아
    정지 static_seconds(기본 15 s) 로 자이로 bias·|g|·초기 자세 → 자이로 직접 적분 자세 →
    중력 제거 → 사다리꼴 이중적분. ZUPT 없음 — 오차는 t² 로 발산한다(그게 논문 기준).
  world 프레임: 중력 = +Z, x = 정지 시 IMU x 의 수평 투영(yaw 관측 불가 → 0).
  VSLAM odom(첫 락 = 원점, base_link 전방 = x) 과 축이 맞으므로 RViz 에서 겹쳐 그린다.
  bag 앞에 정지 15 s 가 없으면 캘리브레이션이 늦어지거나(정지 샘플 부족 시 계속 모음) 틀어진다 — 경고만.

[출력]
  /ins/odometry  frame odom / child base_link. 캘리브 중에는 pose 공분산 1e6.
  /ins/status    1 Hz 문자열
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib.ins_daron import DaronReckoner       # noqa: E402
from lib.rotations import mat_to_quat         # noqa: E402

import rclpy                                  # noqa: E402
from rcl_interfaces.msg import ParameterDescriptor  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node                   # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from nav_msgs.msg import Odometry             # noqa: E402
from sensor_msgs.msg import Imu               # noqa: E402
from std_msgs.msg import String               # noqa: E402

# /imu/data 발행자 실측 QoS: RELIABLE / VOLATILE, depth 100 (bag metadata).
IMU_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=100,
                     reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


class InsOdomNode(Node):
    def __init__(self):
        super().__init__('ins_odom_node')
        dp = self.declare_parameter
        dp('imu_topic', '/imu/data', ParameterDescriptor(description='raw IMU (논문 INS 선과 동일)'))
        dp('odom_topic', '/ins/odometry')
        dp('status_topic', '/ins/status')
        dp('odom_frame_id', 'odom')
        dp('child_frame_id', 'base_link')
        dp('static_seconds', 15.0, ParameterDescriptor(description='기동 후 정지 캘리브레이션 창 [s] (논문 --static-sec)'))
        dp('max_dt', 0.1)
        dp('min_static_samples', 200)
        g = lambda n: self.get_parameter(n).value  # noqa: E731

        self._frame, self._child = str(g('odom_frame_id')), str(g('child_frame_id'))
        self._dr = DaronReckoner(static_s=float(g('static_seconds')), max_dt=float(g('max_dt')),
                                 min_static_samples=int(g('min_static_samples')))
        self._pub = self.create_publisher(Odometry, str(g('odom_topic')), 10)
        self._status_pub = self.create_publisher(String, str(g('status_topic')), 10)
        self.create_subscription(Imu, str(g('imu_topic')), self._on_imu, IMU_QOS)
        self.create_timer(1.0, self._status)
        self._n = 0
        self.get_logger().info(
            f'ins_odom_node: {g("imu_topic")} -> {g("odom_topic")} static={self._dr.static_s:.0f}s '
            f'(ZUPT 없음, 논문 analysis_ins_only.py 와 동일) use_sim_time={g("use_sim_time")}')

    def _on_imu(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        was = self._dr.calibrated
        R, tracked = self._dr.step(
            t, (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z),
            (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z))
        if tracked and not was:
            self.get_logger().info('calib: ' + self._dr.calib_summary())
        self._n += 1

        out = Odometry()
        out.header.stamp, out.header.frame_id, out.child_frame_id = msg.header.stamp, self._frame, self._child
        p, o = out.pose.pose.position, out.pose.pose.orientation
        p.x, p.y, p.z = (float(v) for v in self._dr.p)
        o.x, o.y, o.z, o.w = (float(v) for v in mat_to_quat(R))
        cov = [0.0] * 36
        for i in range(6):
            cov[i * 7] = 0.5 if tracked else 1e6
        out.pose.covariance = cov
        vb = R.T @ self._dr.v
        out.twist.twist.linear.x, out.twist.twist.linear.y, out.twist.twist.linear.z = (float(v) for v in vb)
        out.twist.twist.angular = msg.angular_velocity
        self._pub.publish(out)

    def _status(self):
        self._status_pub.publish(String(data=self._dr.status('ins')))


def main():
    rclpy.init()
    node = None
    try:
        node = InsOdomNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
