#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
imu_recalib_relay — 전복(중력 반전) 감지 후 IMU 피치를 재보정해 재발행하는 릴레이.
배치 경로: ~/daron_ros/imu_recalib_relay.py
작성: 2026-09-21
환경: 시스템 python3 (imu_cov_relay.py 와 동일). 계산은 lib/recalib.py (ROS 없음, tests/ 로 검증).

[배관]
  /ekf/imu/data → imu_cov_relay → /imu/data_fixed → [이 노드] → /imu/data_recalib → EKF-B
                                        └────────────────────────────────────→ EKF-A
  이벤트 전에는 /imu/data_recalib 이 /imu/data_fixed 와 값이 같다(R_corr = I). 그래서
  EKF-A/B 는 이벤트 전까지 같은 궤적이고, 갈라지는 만큼이 recalib 효과다.

[무엇을 바꾸나]
  (a) 중력 기준 장착 정렬 (body_align, 기본 true — 260921 저녁 추가)
      시작 정지 창의 중력 g0 로 장착 자세 R_base←imu = initial_attitude(g0) 를 잡고
        R_world_base(t) = R_z(-yaw0) · R_world_imu(t) · R_corrᵀ · R_base←imuᵀ
      로 **base_link 자세**를 만들어 frame_id=base_link 로 낸다(가속도·각속도도 base 축으로 회전).
      EKF-B 는 imu0_relative:=false 로 받는다.
      왜: robot_localization 의 imu0_relative 는 첫 자세를 R_imu(0)⁻¹·R_imu(t) 로 **world 축(왼쪽 곱)**
      에서 뺀다. IMU 가 base 에 -13.3° 피치로 달려 있어 정자세에선 무해하지만 180° 롤 전복 뒤엔
      그 오프셋 부호가 뒤집혀 pitch 가 2×13.3=27° 틀린다(viz2_rollover 실측: VSLAM -14° vs EKF-A +13.5°,
      후진 1.4 m × sin27° ≈ 0.63 m 가 z 로 샘 = EKF-A z +0.65 m). 장착을 body 축(오른쪽 곱)에서
      빼면 뒤집혀도 맞는다. yaw 는 world 축 회전이므로 왼쪽 곱으로 0 을 맞추는 게 옳다.
  (b) 전복 뒤 피치 Δθ (lib/recalib.py) — **기본은 기록만(apply_dpitch:=false)**.
      260921 실측: (a) 만으로 EKF-B pitch 가 VSLAM 과 1° 안에서 맞았고(t=58 s: -18.6 vs -19.6),
      Δθ=-12.8° 를 얹자 -25.6 으로 12° 반대로 넘어갔다. 중력으로 잰 Δθ 는 "장착 변화"가 아니라
      "재구성 뒤 몸체가 실제로 기운 자세"이고 그건 AHRS 자세가 이미 담고 있다 — 이중 계산.
      IMU 와 카메라가 다른 세그먼트에 있어 장착이 실제로 돌아가는 구조가 되면 그때 켠다.
  body_align:=false 면 (b) 만 하고 frame_id/relative 는 원래대로(초기 구현, 27° 오차 절반만 잡힘).
  [261008] mount_source — (a) 의 R_base←imu 를 어디서 얻나
      g0  (옛 동작) 시작 정지 창 중력 g0 로 잡음 = "출발 = 수평" 가정. 기울어진 곳에서 출발하면
          그 경사가 장착각으로 흡수돼 world 가 출발 경사만큼 기운다(real_output 약 10° → SLAM 과 5.5° 어긋남).
      tf  (설계값) launch 가 URDF base_to_imu rpy 를 imu_roll/pitch/yaw 로 넘긴다(기본 mount:=level → 0,0,0).
          출발 기울기는 CV7 AHRS 자세에 그대로 남는다 → world z = 중력, yaw0 만 0 으로.
      두 모드 모두 첫 출력 자세 R_world←base(t0)(yaw 0)를 start_attitude_topic 으로 한 번 낸다(TRANSIENT_LOCAL).
      SLAM 노드가 이걸 odom 원점 자세로 써서 SLAM·EKF 가 같은 중력 기준에서 출발한다(mount_source:=tf 일 때만 켬).
  recalib_apply:=false 면 (a)(b) 다 끄고 감지·이벤트만 내며 메시지는 그대로 통과(대조군).

[출력]
  /ins/recalib_event   std_msgs/String  이벤트마다 1회 (RecalibEvent.summary())
  /ins/recalib_marker  visualization_msgs/Marker  이벤트 위치(pose_topic 의 마지막 pose)에 구 + 텍스트
  /ins/recalib_status  std_msgs/String  1 Hz 상태
  이벤트 판정은 전부 header stamp 기준이라 bag 배속(0.3x)과 무관하다.

사용 예:
  /usr/bin/python3 ~/daron_ros/imu_recalib_relay.py --ros-args \
      -p input_topic:=/imu/data_fixed -p output_topic:=/imu/data_recalib \
      -p recalib_apply:=true -p flip_ratio:=0.5 -p hold_s:=0.5 -p static_s:=1.0
"""

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib.ins_daron import initial_attitude   # noqa: E402
from lib.recalib import RollRecalib          # noqa: E402
from lib.rotations import mat_to_quat, quat_to_mat  # noqa: E402

import rclpy                                  # noqa: E402
from rcl_interfaces.msg import ParameterDescriptor  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node                   # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from geometry_msgs.msg import Point, QuaternionStamped  # noqa: E402
from nav_msgs.msg import Odometry             # noqa: E402
from sensor_msgs.msg import Imu               # noqa: E402
from std_msgs.msg import String               # noqa: E402
from visualization_msgs.msg import Marker     # noqa: E402

# imu_cov_relay.RELAY_QOS 와 동일 (발행자 실측 RELIABLE/VOLATILE).
RELAY_QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                       reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)


def _rpy_to_mat(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF rpy (고정축 X→Y→Z) → 회전행렬 R = Rz·Ry·Rx (parent ← child)."""
    cr, sr, cp, sp, cy, sy = np.cos(roll), np.sin(roll), np.cos(pitch), np.sin(pitch), np.cos(yaw), np.sin(yaw)
    Rx = np.array([[1.0, 0.0, 0.0], [0.0, cr, -sr], [0.0, sr, cr]])
    Ry = np.array([[cp, 0.0, sp], [0.0, 1.0, 0.0], [-sp, 0.0, cp]])
    Rz = np.array([[cy, -sy, 0.0], [sy, cy, 0.0], [0.0, 0.0, 1.0]])
    return Rz @ Ry @ Rx


def _rotate_cov(cov9, R: np.ndarray):
    C = np.array(cov9, float).reshape(3, 3)
    return [float(v) for v in (R @ C @ R.T).reshape(9)]


class ImuRecalibRelay(Node):
    def __init__(self):
        super().__init__('imu_recalib_relay')
        dp = self.declare_parameter
        dp('input_topic', '/imu/data_fixed', ParameterDescriptor(description='구독 Imu (공분산 채워진 것)'))
        dp('output_topic', '/imu/data_recalib', ParameterDescriptor(description='재발행 Imu (EKF-B imu0)'))
        dp('event_topic', '/ins/recalib_event')
        dp('marker_topic', '/ins/recalib_marker')
        dp('status_topic', '/ins/recalib_status')
        dp('pose_topic', '/odometry/filtered_recalib',
           ParameterDescriptor(description='마커 위치용 Odometry. 없으면 원점에 그린다.'))
        dp('marker_frame_id', 'odom')
        dp('recalib_apply', True, ParameterDescriptor(description='false 면 감지만 하고 IMU 는 그대로 통과(대조군)'))
        dp('body_align', True, ParameterDescriptor(
            description='시작 중력으로 장착 자세를 잡아 base_link 자세로 재발행(frame_id=base_link). '
                        'EKF-B 는 imu0_relative:=false 여야 한다.'))
        dp('body_frame_id', 'base_link')
        dp('mount_source', 'g0', ParameterDescriptor(
            description='[261008] body_align 장착 자세 출처: g0(시작 중력, 출발=수평 가정 — 옛 동작) / '
                        'tf(imu_roll/pitch/yaw = URDF base_to_imu, 출발 기울기 보존)'))
        dp('imu_roll', 0.0)
        dp('imu_pitch', 0.0)
        dp('imu_yaw', 0.0)
        dp('start_attitude_topic', '/ins/start_attitude', ParameterDescriptor(
            description='[261008] 첫 출력 base 자세(world←base, yaw 0)를 한 번 발행. 빈 문자열 = 안 냄'))
        dp('apply_dpitch', False, ParameterDescriptor(
            description='전복 뒤 정지 창 중력으로 잰 Δθ(피치)를 실제로 적용할지. 기본 false = 기록만. '
                        '260921 실측: body_align 만으로 EKF-B pitch 가 VSLAM 과 1° 안에서 맞고, Δθ 를 얹으면 '
                        '12° 반대로 넘어간다(Δθ 는 장착 변화가 아니라 재구성 뒤 몸체 자세 — AHRS 가 이미 반영).'))
        # 판정 파라미터 — 전부 DOUBLE. launch 에서 '0.5' 처럼 소수점 있는 문자열로 넘길 것.
        dp('flip_ratio', 0.5)
        dp('hold_s', 0.5)
        dp('static_s', 1.0)
        dp('gyro_thr', 0.10)
        dp('acc_std_thr', 0.15)
        dp('max_wait_s', 10.0)
        dp('relaxed_acc_std_max', 1.0)

        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self._in_topic, self._out_topic = str(g('input_topic')), str(g('output_topic'))
        if self._in_topic == self._out_topic:
            raise ValueError(f'input_topic 과 output_topic 이 같습니다: {self._in_topic}')
        self._apply = bool(g('recalib_apply'))
        self._body_align = bool(g('body_align')) and self._apply
        self._apply_dpitch = bool(g('apply_dpitch')) and self._apply
        self._body_frame = str(g('body_frame_id'))
        self._mount_source = str(g('mount_source')).strip().lower()
        if self._mount_source not in ('g0', 'tf'):
            raise ValueError(f"mount_source 는 g0/tf 여야 합니다: {self._mount_source!r}")
        self._R_base_imu = None      # base <- imu (g0 또는 URDF 로 결정)
        if self._body_align and self._mount_source == 'tf':
            self._R_base_imu = _rpy_to_mat(float(g('imu_roll')), float(g('imu_pitch')), float(g('imu_yaw')))
        self._start_topic = str(g('start_attitude_topic')).strip()
        self._start_sent = False
        self._Rz_yaw0 = None         # world yaw 0 맞춤 (첫 메시지)
        self._rc = RollRecalib(
            flip_ratio=float(g('flip_ratio')), hold_s=float(g('hold_s')), static_s=float(g('static_s')),
            gyro_thr=float(g('gyro_thr')), acc_std_thr=float(g('acc_std_thr')),
            max_wait_s=float(g('max_wait_s')), relaxed_acc_std_max=float(g('relaxed_acc_std_max')),
            enabled=self._apply_dpitch)
        self._marker_frame = str(g('marker_frame_id'))
        self._last_pos = None
        self._marker_id = 0
        self._count = 0
        self._last_state_logged = None

        self._pub = self.create_publisher(Imu, self._out_topic, RELAY_QOS)
        self._event_pub = self.create_publisher(String, str(g('event_topic')), 10)
        self._marker_pub = self.create_publisher(Marker, str(g('marker_topic')), 10)
        self._status_pub = self.create_publisher(String, str(g('status_topic')), 10)
        self._start_pub = None
        if self._start_topic and self._body_align:
            latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self._start_pub = self.create_publisher(QuaternionStamped, self._start_topic, latched)
        self.create_subscription(Imu, self._in_topic, self._on_imu, RELAY_QOS)
        self.create_subscription(Odometry, str(g('pose_topic')), self._on_pose, 10)
        self.create_timer(1.0, self._status)

        self.get_logger().info(
            f'imu_recalib_relay: {self._in_topic} -> {self._out_topic} apply={self._apply} '
            f'flip_ratio={self._rc.flip_ratio} hold_s={self._rc.hold_s} static_s={self._rc.static_s} '
            f'gyro_thr={self._rc.gyro_thr} acc_std_thr={self._rc.acc_std_thr} '
            f'max_wait_s={self._rc.max_wait_s} body_align={self._body_align} apply_dpitch={self._apply_dpitch} '
            f'mount_source={self._mount_source} start_attitude_topic={self._start_topic or "(off)"} '
            f'use_sim_time={g("use_sim_time")}')

    # ── 콜백 ─────────────────────────────────────────────────────────
    def _on_pose(self, msg: Odometry):
        p = msg.pose.pose.position
        self._last_pos = (p.x, p.y, p.z)

    def _on_imu(self, msg: Imu):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        a = (msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z)
        w = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        R, event = self._rc.step(t, a, w)

        if self._rc.state != self._last_state_logged and self._rc.g0 is not None:
            self.get_logger().info(f'state -> {self._rc.state} (t={t:.2f}) '
                                   f'g0=({self._rc.g0[0]:+.2f},{self._rc.g0[1]:+.2f},{self._rc.g0[2]:+.2f})')
            self._last_state_logged = self._rc.state
        if event is not None:
            self._emit_event(msg, event)

        if self._body_align and self._R_base_imu is None and self._rc.g0 is None:
            # [g0] 장착 자세(g0)가 잡히기 전(첫 정지 창 ~1 s)에는 내보내지 않는다 — imu_link 자세를 그대로
            # 내면 EKF-B 가 그걸로 초기화돼 정렬된 뒤 자세가 한 번 튄다. [tf] 는 기동 때 이미 정해져 기다리지 않는다.
            return
        if self._body_align and self._rc.g0 is not None and self._R_base_imu is None:
            self._R_base_imu = initial_attitude(self._rc.g0)          # base <- imu (시작 = 수평 가정)
            pitch = np.degrees(np.arcsin(-self._R_base_imu[2, 0]))
            self.get_logger().info(f'body_align: 장착 자세 from g0 — imu pitch {pitch:+.1f}deg (URDF -17.4 대신 실측 사용)')

        if self._body_align and self._R_base_imu is not None:
            # (a) base_link 자세: 장착은 body 축(오른쪽 곱), yaw 0 은 world 축(왼쪽 곱)
            q = msg.orientation
            Rwb = quat_to_mat(q.x, q.y, q.z, q.w) @ R.T @ self._R_base_imu.T
            if self._Rz_yaw0 is None:
                yaw0 = np.arctan2(Rwb[1, 0], Rwb[0, 0])
                c, s_ = np.cos(-yaw0), np.sin(-yaw0)
                self._Rz_yaw0 = np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]])
            Rwb = self._Rz_yaw0 @ Rwb
            q.x, q.y, q.z, q.w = (float(v) for v in mat_to_quat(Rwb))
            if not self._start_sent:
                self._send_start(msg, Rwb)
            Rv = self._R_base_imu @ R                                    # 벡터: imu -> base
            a2, w2 = Rv @ np.asarray(a), Rv @ np.asarray(w)
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (float(v) for v in a2)
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (float(v) for v in w2)
            msg.orientation_covariance = _rotate_cov(msg.orientation_covariance, Rv)
            msg.angular_velocity_covariance = _rotate_cov(msg.angular_velocity_covariance, Rv)
            msg.linear_acceleration_covariance = _rotate_cov(msg.linear_acceleration_covariance, Rv)
            msg.header.frame_id = self._body_frame
        elif self._apply and self._rc.n_events > 0:
            # (b) 만: imu 프레임 그대로 두고 R_corr 만 적용 (body_align:=false)
            q = msg.orientation
            Rwb = quat_to_mat(q.x, q.y, q.z, q.w) @ R.T
            q.x, q.y, q.z, q.w = (float(v) for v in mat_to_quat(Rwb))
            a2, w2 = R @ np.asarray(a), R @ np.asarray(w)
            msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z = (float(v) for v in a2)
            msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z = (float(v) for v in w2)
            msg.orientation_covariance = _rotate_cov(msg.orientation_covariance, R)
            msg.angular_velocity_covariance = _rotate_cov(msg.angular_velocity_covariance, R)
            msg.linear_acceleration_covariance = _rotate_cov(msg.linear_acceleration_covariance, R)
        self._pub.publish(msg)
        self._count += 1
        if self._count == 1:
            self.get_logger().info(f'첫 메시지 릴레이: frame_id={msg.header.frame_id!r}')

    def _send_start(self, imu: Imu, Rwb: np.ndarray):
        """[261008] 첫 출력 자세 = 출발 자세(world←base, yaw 0). SLAM 노드가 odom 원점 자세로 쓴다."""
        self._start_sent = True
        roll = np.degrees(np.arctan2(Rwb[2, 1], Rwb[2, 2]))
        pitch = np.degrees(np.arcsin(-np.clip(Rwb[2, 0], -1.0, 1.0)))
        tilt = np.degrees(np.arccos(np.clip(Rwb[2, 2], -1.0, 1.0)))
        self.get_logger().info(f'출발 자세 (world←base, mount_source={self._mount_source}): '
                               f'roll {roll:+.2f}° pitch {pitch:+.2f}° (기울기 {tilt:.2f}°)')
        if self._start_pub is None:
            return
        m = QuaternionStamped()
        m.header.stamp = imu.header.stamp
        m.header.frame_id = 'odom'
        m.quaternion.x, m.quaternion.y, m.quaternion.z, m.quaternion.w = (float(v) for v in mat_to_quat(Rwb))
        self._start_pub.publish(m)

    def _emit_event(self, imu: Imu, ev):
        text = ev.summary() + (' APPLIED' if self._apply_dpitch and ev.quality != 'none' else ' LOGGED-ONLY')
        if ev.quality == 'strict':
            self.get_logger().info('RECALIB ' + text)
        else:
            self.get_logger().warning('RECALIB ' + text)
        self._event_pub.publish(String(data=text))

        pos = self._last_pos or (0.0, 0.0, 0.0)
        for kind in (Marker.SPHERE, Marker.TEXT_VIEW_FACING):
            m = Marker()
            m.header.stamp = imu.header.stamp
            m.header.frame_id = self._marker_frame
            m.ns = 'recalib'
            m.id = self._marker_id
            self._marker_id += 1
            m.type = kind
            m.action = Marker.ADD
            m.pose.position = Point(x=pos[0], y=pos[1], z=pos[2] + (0.25 if kind == Marker.TEXT_VIEW_FACING else 0.0))
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.12
            m.color.a = 1.0
            m.color.r, m.color.g, m.color.b = (1.0, 0.2, 0.2) if ev.state == 'inverted' else (0.2, 0.6, 1.0)
            if kind == Marker.TEXT_VIEW_FACING:
                m.text = f'recalib #{self._rc.n_events} {ev.state} dpitch={np.degrees(ev.delta_pitch_rad):+.1f}deg ({ev.quality})'
            self._marker_pub.publish(m)

    def _status(self):
        self._status_pub.publish(String(data=self._rc.status() + f' relayed={self._count}'))


def main():
    rclpy.init()
    node = None
    rc = 0
    try:
        node = ImuRecalibRelay()
        rclpy.spin(node)
    except ValueError as exc:
        rclpy.logging.get_logger('imu_recalib_relay').error(str(exc))
        rc = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
