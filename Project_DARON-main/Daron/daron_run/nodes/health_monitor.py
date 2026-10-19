#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
health_monitor — SLAM / INS health 판정 + EKF-C 입력 공분산 릴레이 + 재정렬(set_pose).
배치 경로: ~/Daron/daron_run/nodes/health_monitor.py
[261015 시계 분리] 이벤트는 기록 즉시 수준에 반영(릴레이는 hm.levels() 를 메시지마다 읽는다), 생존 판정은
  수신 시각(rx), 프레임 공백은 영상 끊김(video_cut, 끊긴 동안만 FAIL). 검증: local_test/health_v2/ (git 제외).
작성: 2026-09-29
환경: 시스템 python3 (numpy). 판정 계산은 lib/health.py (ROS 없음, tests/test_health.py, tools/health_offline.py).

[배관]
  /ekf/imu/data  ──(진단만)──────────────────────────► [이 노드] ── /health/state (JSON, 10 Hz)
  /slam/status   ──(진단만)──────────────────────────►     │      ── /health/marker (RViz 텍스트)
  /imu/data_recalib ─► 공분산 × 1/h_INS², FAIL 이면 버림 ─► /imu/data_h      ─► EKF-C
  /slam/odometry    ─► 위치 × 1/h_SLAM², 자세는 INS 상태로 ─► /slam/odometry_h ─► EKF-C
  /odometry/filtered_health ─► 재정렬 판정 ─► /ekf_filter_node_health/set_pose

[SLAM 자세 공분산 — INS 상태에 따라 반대로 움직인다]
  INS OK       → slam_ori_var_ins_ok (1.0)       IMU 가 자세를 주도 (EKF-C 는 SLAM 자세를 각속도로 받는다)
  INS DEGRADED → slam_ori_var_ins_degraded (0.1)
  INS FAIL     → 원래 분산(0.01) × 1/h_SLAM²       SLAM 이 넘겨받는다 — real_output 처럼 IMU 가 끊기면
                                                   EKF-A 는 자세가 제멋대로 돌아 발산했다(260929 분석)

[재정렬] 조건을 모두 만족할 때만:
  (1) 최근 realign_window_s 안에 health 이벤트(INS 나 SLAM 이 OK 아님)가 있었고
  (2) 지금 SLAM 이 OK 이고
  (3) EKF-C 위치와 목표의 차이 > realign_thresh_m (0.10 m, 260929 결정)
  목표 = 마지막 정상 시점의 EKF pose · (그때 SLAM pose)⁻¹ · 지금 SLAM pose
       (SLAM 절대 위치가 아니다 — 기존 맵과 이어지게 하려고).
  둘 다 정상일 때는 재정렬하지 않는다(평소 EKF–SLAM 차이는 공분산이 처리). 로버를 세울 필요 없음:
  set_pose 는 속도를 0 으로 두고 differential 기준을 지워 SLAM 한 샘플(33 ms)을 건너뛸 뿐이다.

[SLAM 점프 제거 — 260929 저녁 추가]
  cuVSLAM 이 LOST 없이 'OK' 로 위치를 튀기는 경우가 있다(실시간 책상 테스트: 2 s 사이 6 m). EKF 는 SLAM 을
  differential 로 받으므로 그 메시지를 버려도 다음 메시지와의 차이에 점프가 실린다. 그래서 릴레이가
  JumpRemover(프레임당 slam_step_fail_m=0.10 m 초과)로 점프를 지운 연속 포즈를 /slam/odometry_h 로 낸다.
  판정은 원시 포즈로 한다(점프 = SLAM FAIL → 그동안 릴레이는 버림). 재정렬 목표·맵 ANCHOR 도 이 연속 포즈 기준.
  EKF-A/B 는 원시 /slam/odometry 를 받으므로 같은 점프를 그대로 삼킨다(differential 은 pose_rejection 이 안 걸림).

[장착 회전 R_base_imu] extrinsic_mode:
  online (기본) 주행 초반 데이터로 추정(estimate_extrinsic). 추정 전에는 교차검사(회전 비교)를 보류한다.
                URDF 값(-17.4° 피치)과 데이터 추정(bag 별 -12~-16°)이 달라, URDF 로 바로 비교하면 오경보가 난다.
                extrinsic_fallback_s 안에 추정이 안 되면(회전이 부족한 직진 등) TF(URDF) 값으로 교차검사를 켠다.
  tf            처음부터 TF(base_link ← IMU frame_id) 값.

사용 예 (보통은 daron_viz.launch.py health:=true 가 띄운다):
  /usr/bin/python3 ~/Daron/daron_run/nodes/health_monitor.py --ros-args -p use_sim_time:=true
"""

import json
import math
import sys
from collections import deque
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from lib.health import (FAIL, LEVEL_HEALTH, LEVEL_NAME, OK, DEGRADED, HealthMonitor, HealthParams, JumpRemover,  # noqa: E402
                        cov_scale, estimate_extrinsic, quat_to_mat, rot_angle_deg)
from lib.rotations import mat_to_quat  # noqa: E402

import rclpy  # noqa: E402
import tf2_ros  # noqa: E402
from geometry_msgs.msg import PoseWithCovarianceStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from rclpy.executors import ExternalShutdownException  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from rclpy.time import Time  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402

# imu_cov_relay 와 같은 QoS (입력 발행자들이 RELIABLE / VOLATILE)
QOS = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                 reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
DIAG3 = (0, 4, 8)                   # 3x3 row-major 대각
POS6 = (0, 7, 14)                   # 6x6 pose 공분산의 x,y,z 대각
ORI6 = (21, 28, 35)                 # roll,pitch,yaw 대각
LEVEL_RGB = {OK: (0.2, 0.9, 0.2), DEGRADED: (1.0, 0.8, 0.0), FAIL: (1.0, 0.15, 0.15)}


def _stamp(h):
    return h.stamp.sec + h.stamp.nanosec * 1e-9


def _T(pos, quat):
    T = np.eye(4)
    T[:3, :3] = quat_to_mat(quat)
    T[:3, 3] = pos
    return T


def _Tinv(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T @ T[:3, 3]
    return Ti


class HealthMonitorNode(Node):
    def __init__(self):
        super().__init__('health_monitor')
        P = self.declare_parameter
        P('imu_diag_topic', '/ekf/imu/data')
        P('imu_in_topic', '/imu/data_recalib')
        P('imu_out_topic', '/imu/data_h')
        P('slam_in_topic', '/slam/odometry')
        P('slam_out_topic', '/slam/odometry_h')
        P('slam_status_topic', '/slam/status')
        P('ekf_topic', '/odometry/filtered_health')
        P('set_pose_topic', '/ekf_filter_node_health/set_pose')
        P('rate_hz', 10.0)
        P('slam_ori_var_ins_ok', 1.0)
        P('slam_ori_var_ins_degraded', 0.1)
        P('realign_enable', True)
        P('realign_thresh_m', 0.10)
        P('realign_window_s', 30.0)
        P('realign_min_interval_s', 5.0)
        P('extrinsic_mode', 'online')
        P('extrinsic_after_s', 30.0)
        P('extrinsic_min_rot_deg', 20.0)
        P('extrinsic_min_corr', 0.9)
        P('extrinsic_fallback_s', 120.0)
        P('body_frame', 'base_link')
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.p = {n: g(n) for n in (
            'imu_diag_topic', 'imu_in_topic', 'imu_out_topic', 'slam_in_topic', 'slam_out_topic',
            'slam_status_topic', 'ekf_topic', 'set_pose_topic', 'rate_hz', 'slam_ori_var_ins_ok',
            'slam_ori_var_ins_degraded', 'realign_enable', 'realign_thresh_m', 'realign_window_s',
            'realign_min_interval_s', 'extrinsic_mode', 'extrinsic_after_s', 'extrinsic_min_rot_deg',
            'extrinsic_min_corr', 'extrinsic_fallback_s', 'body_frame')}
        # rclpy 는 DOUBLE 로 선언한 파라미터에 정수를 주면 죽는다(260911) — 여기선 float() 로만 읽는다.
        for k in ('rate_hz', 'slam_ori_var_ins_ok', 'slam_ori_var_ins_degraded', 'realign_thresh_m',
                  'realign_window_s', 'realign_min_interval_s', 'extrinsic_after_s',
                  'extrinsic_min_rot_deg', 'extrinsic_min_corr', 'extrinsic_fallback_s'):
            self.p[k] = float(self.p[k])
        if self.p['extrinsic_mode'] not in ('online', 'tf'):
            raise ValueError(f"extrinsic_mode 는 online/tf 여야 합니다: {self.p['extrinsic_mode']!r}")

        self.hp = HealthParams()
        self.hm = HealthMonitor(self.hp)
        self.hm.cross_enabled = False               # 장착 회전이 정해질 때까지 교차검사 보류
        self.rep = None
        self._prev_levels = None
        self._last_log = -1e9
        self.jr = JumpRemover(self.hp.slam_step_fail_m)

        # 장착 회전
        self._ext_state = 'waiting'                 # waiting → online | tf
        self._imu_frame = None
        self._ext_imu = deque()                     # (t, ω)
        self._ext_slam = deque()                    # (t, R)
        self._ext_next_try = None
        self._slam_t0 = None
        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)

        # 재정렬
        self._slam_T = None                         # (t, T_odom_base)
        self._ekf_T = None
        self._anchor = None                         # (T_ekf, T_slam) 마지막 정상 시점
        self._event_until = -1.0
        self._last_realign = -1e9
        self.realign_count = 0

        # 통계
        self._n = {'imu_in': 0, 'imu_drop': 0, 'slam_in': 0, 'slam_drop': 0}

        p = self.p
        self.pub_imu = self.create_publisher(Imu, p['imu_out_topic'], QOS)
        self.pub_slam = self.create_publisher(Odometry, p['slam_out_topic'], QOS)
        self.pub_state = self.create_publisher(String, '/health/state', 10)
        self.pub_marker = self.create_publisher(Marker, '/health/marker', 1)
        self.pub_set_pose = self.create_publisher(PoseWithCovarianceStamped, p['set_pose_topic'], 10)
        self.create_subscription(Imu, p['imu_diag_topic'], self.on_imu_diag, QOS)
        self.create_subscription(Imu, p['imu_in_topic'], self.on_imu_relay, QOS)
        self.create_subscription(Odometry, p['slam_in_topic'], self.on_slam, QOS)
        self.create_subscription(String, p['slam_status_topic'], self.on_slam_status, 10)
        self.create_subscription(Odometry, p['ekf_topic'], self.on_ekf, QOS)
        self.create_timer(1.0 / p['rate_hz'], self.on_timer)
        self.get_logger().info(
            f"health_monitor: diag {p['imu_diag_topic']} + {p['slam_status_topic']} | relay "
            f"{p['imu_in_topic']}→{p['imu_out_topic']}, {p['slam_in_topic']}→{p['slam_out_topic']} | "
            f"realign {'on' if p['realign_enable'] else 'off'} >{p['realign_thresh_m']} m → {p['set_pose_topic']} | "
            f"extrinsic {p['extrinsic_mode']}")

    # ------------------------------------------------ 입력: 진단
    def on_imu_diag(self, m: Imu):
        t = _stamp(m.header)
        w, a, q = m.angular_velocity, m.linear_acceleration, m.orientation
        gyro = (w.x, w.y, w.z)
        self.hm.imu_sample(t, gyro, (a.x, a.y, a.z), (q.x, q.y, q.z, q.w),
                           rx=self.get_clock().now().nanoseconds * 1e-9)   # 수신 시각 (261015)
        if self._imu_frame is None:
            self._imu_frame = m.header.frame_id
        if self._ext_state == 'waiting':
            self._ext_imu.append((t, gyro))
            while self._ext_imu and self._ext_imu[0][0] < t - 150.0:
                self._ext_imu.popleft()

    def on_slam_status(self, m: String):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        self.hm.slam_status(d.get('stamp_ns', 0) * 1e-9, d.get('features_l'), d.get('relock_jump_m'))

    def on_ekf(self, m: Odometry):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self._ekf_T = (_stamp(m.header), _T((p.x, p.y, p.z), (q.x, q.y, q.z, q.w)))

    # ------------------------------------------------ 릴레이
    def _levels(self):
        # 이벤트 직후 escalate 된 지금 수준 — 다음 evaluate(최대 0.1 s)를 기다리지 않는다 (261015)
        ins, slam = self.hm.levels()
        return ins, slam, LEVEL_HEALTH[ins], LEVEL_HEALTH[slam]

    def on_imu_relay(self, m: Imu):
        self._n['imu_in'] += 1
        ins, _, h_ins, _ = self._levels()
        k = cov_scale(h_ins)
        if ins == FAIL or k is None:
            self._n['imu_drop'] += 1
            return
        for cov in (m.orientation_covariance, m.angular_velocity_covariance, m.linear_acceleration_covariance):
            for i in DIAG3:
                cov[i] = cov[i] * k
        self.pub_imu.publish(m)

    def on_slam(self, m: Odometry):
        t = _stamp(m.header)
        p, q = m.pose.pose.position, m.pose.pose.orientation
        pos, quat = (p.x, p.y, p.z), (q.x, q.y, q.z, q.w)
        var0 = m.pose.covariance[0]
        self.hm.slam_sample(t, pos, quat, var0, rx=self.get_clock().now().nanoseconds * 1e-9)
        tracked = var0 < self.hp.lost_var
        if self._slam_t0 is None:
            self._slam_t0 = t
        T_out = None
        if tracked:
            T_out, jumped = self.jr.apply(_T(pos, quat))
            if jumped is not None:
                self.get_logger().warning(
                    f"SLAM 점프 {jumped:.3f} m 제거 (#{self.jr.count}, 누적 {self.jr.total_m:.2f} m) — "
                    "/slam/odometry_h 는 직전 포즈에서 이어서 낸다")
            self._slam_T = (t, T_out)
            if self._ext_state == 'waiting':
                self._ext_slam.append((t, quat_to_mat(quat)))
                while self._ext_slam and self._ext_slam[0][0] < t - 150.0:
                    self._ext_slam.popleft()

        self._n['slam_in'] += 1
        ins, slam, _, h_slam = self._levels()
        k = cov_scale(h_slam)
        if slam == FAIL or k is None or not tracked:
            self._n['slam_drop'] += 1
            return
        # 점프 제거한 연속 포즈로 바꿔 낸다 (점프가 없으면 T_fix = I 라 원시와 같다)
        pp = m.pose.pose
        pp.position.x, pp.position.y, pp.position.z = (float(v) for v in T_out[:3, 3])
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = mat_to_quat(T_out[:3, :3])
        c = m.pose.covariance
        for i in POS6:
            c[i] = c[i] * k
        for i in ORI6:
            if ins == OK:
                c[i] = self.p['slam_ori_var_ins_ok']
            elif ins == DEGRADED:
                c[i] = self.p['slam_ori_var_ins_degraded']
            else:
                c[i] = c[i] * k
        self.pub_slam.publish(m)

    # ------------------------------------------------ 주기
    def on_timer(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        if now <= 0.0:
            return                                    # sim time 에서 /clock 이 아직 없음
        self._try_extrinsic(now)
        rep = self.hm.evaluate(now)
        self.rep = rep
        # 수준이 바뀌면 즉시, 수준은 같고 사유만 바뀌면 1 s 에 한 번 (같은 DEGRADED 안에서 jump 등이 묻히지 않게)
        levels = (rep.ins_level, rep.slam_level, tuple(rep.ins_reasons), tuple(rep.slam_reasons))
        if levels != self._prev_levels:
            if self._prev_levels is None or levels[:2] != self._prev_levels[:2] or now - self._last_log >= 1.0:
                self._log_change(rep)
                self._last_log = now
                self._prev_levels = levels
        if self.p['realign_enable']:
            self._realign(now, rep)
        self._publish_state(rep)

    def _log_change(self, rep):
        msg = f"health: {rep.summary()}"
        # rclpy 는 호출 지점마다 severity 를 캐싱한다(260911) — 수위별로 다른 줄에서 부른다.
        if rep.ins_level == FAIL or rep.slam_level == FAIL:
            self.get_logger().error(msg)
        elif rep.ins_level == DEGRADED or rep.slam_level == DEGRADED:
            self.get_logger().warning(msg)
        else:
            self.get_logger().info(msg)

    # ------------------------------------------------ 장착 회전
    def _tf_extrinsic(self):
        if not self._imu_frame:
            return None
        try:
            tf = self.tf_buf.lookup_transform(self.p['body_frame'], self._imu_frame, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            return None
        q = tf.transform.rotation
        return quat_to_mat((q.x, q.y, q.z, q.w))

    def _set_extrinsic(self, R, how):
        self.hm.R_bi = R
        self.hm.cross_enabled = True
        self._ext_state = how
        self._ext_imu.clear()
        self._ext_slam.clear()
        tfR = self._tf_extrinsic()
        diff = f", URDF 와 {rot_angle_deg(tfR.T @ R):.1f}° 차이" if tfR is not None and how == 'online' else ''
        self.get_logger().info(f"장착 회전 R_base_imu 확정 ({how}){diff} — 회전 교차검사 시작")

    def _try_extrinsic(self, now):
        if self._ext_state != 'waiting' or self._slam_t0 is None:
            return
        if self.p['extrinsic_mode'] == 'tf':
            R = self._tf_extrinsic()
            if R is not None:
                self._set_extrinsic(R, 'tf')
            return
        span = now - self._slam_t0
        if span >= self.p['extrinsic_fallback_s']:
            R = self._tf_extrinsic()
            if R is not None:
                self.get_logger().warning(
                    f"장착 회전 온라인 추정 실패({self.p['extrinsic_fallback_s']:.0f} s 안에 회전 부족) — "
                    "TF(URDF) 값으로 교차검사를 켭니다. URDF 오차만큼 회전 잔차가 커질 수 있습니다.")
                self._set_extrinsic(R, 'tf')
            return
        if span < self.p['extrinsic_after_s'] or (self._ext_next_try and now < self._ext_next_try):
            return
        self._ext_next_try = now + 10.0
        if len(self._ext_slam) < 100 or len(self._ext_imu) < 300:
            return
        Rs = [r for _, r in self._ext_slam]
        # 누적 회전은 15 프레임(0.5 s) 간격으로, 1° 넘는 것만 더한다 — 프레임마다 더하면 정지 중 떨림이 쌓여
        # 정지 95 s 만으로 '66°' 가 됐다(260929 실시간). 그러면 거의 정지 데이터로 추정하게 된다.
        total = 0.0
        for k in range(0, len(Rs) - 15, 15):
            a = rot_angle_deg(Rs[k].T @ Rs[k + 15])
            if a > 1.0:
                total += a
        if total < self.p['extrinsic_min_rot_deg']:
            return
        st = np.array([t for t, _ in self._ext_slam])
        it = np.array([t for t, _ in self._ext_imu])
        iw = np.array([w for _, w in self._ext_imu])
        # 끊기거나 몰린 구간이 섞이면 추정이 망가진다 — 첫 공백 전까지만
        gaps = np.where(np.diff(it) > self.hp.gap_warn_s)[0]
        if len(gaps):
            cut = it[gaps[0]]
            keep_i, keep_s = it <= cut, st <= cut
            it, iw, st = it[keep_i], iw[keep_i], st[keep_s]
            Rs = [r for r, k in zip(Rs, keep_s) if k]
            if len(st) < 100:
                return
        R, off, corr, rms = estimate_extrinsic(st, Rs, it, iw)
        if corr >= self.p['extrinsic_min_corr']:
            self.get_logger().info(f"장착 회전 추정: |ω| corr {corr:.3f}, 잔차 {rms:.2f} deg/s, "
                                   f"시간 오프셋 {off * 1000:.0f} ms(적용 안 함), 누적 회전 {total:.0f}°")
            self._set_extrinsic(R, 'online')

    # ------------------------------------------------ 재정렬
    def _realign(self, now, rep):
        all_ok = rep.ins_level == OK and rep.slam_level == OK
        if not all_ok:
            self._event_until = now + self.p['realign_window_s']
        if self._ekf_T is None or self._slam_T is None:
            return
        if all_ok and now >= self._event_until:
            self._anchor = (self._ekf_T[1].copy(), self._slam_T[1].copy())   # 마지막 정상 시점
            return
        if self._anchor is None or rep.slam_level != OK or now >= self._event_until:
            return
        if now - self._last_realign < self.p['realign_min_interval_s']:
            return
        T_e0, T_s0 = self._anchor
        target = T_e0 @ _Tinv(T_s0) @ self._slam_T[1]
        d = float(np.linalg.norm(self._ekf_T[1][:3, 3] - target[:3, 3]))
        if d <= self.p['realign_thresh_m']:
            return
        msg = PoseWithCovarianceStamped()
        t = self._slam_T[0]
        msg.header.stamp.sec = int(t)
        msg.header.stamp.nanosec = int(round((t - int(t)) * 1e9)) % 1_000_000_000
        msg.header.frame_id = 'odom'
        pp = msg.pose.pose
        pp.position.x, pp.position.y, pp.position.z = (float(v) for v in target[:3, 3])
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = mat_to_quat(target[:3, :3])
        for i in POS6 + ORI6:
            msg.pose.covariance[i] = 1e-4
        self.pub_set_pose.publish(msg)
        self._last_realign = now
        self.realign_count += 1
        self.get_logger().warning(
            f"REALIGN #{self.realign_count}: EKF-C 가 목표(정상 시점 EKF + SLAM 이동분)에서 {d:.3f} m "
            f"벗어나 set_pose (INS={LEVEL_NAME[rep.ins_level]}, SLAM={LEVEL_NAME[rep.slam_level]})")

    # ------------------------------------------------ 출력
    def _publish_state(self, rep):
        st = {
            't': rep.t, 'ins': LEVEL_NAME[rep.ins_level], 'slam': LEVEL_NAME[rep.slam_level],
            'ins_health': rep.ins_health, 'slam_health': rep.slam_health,
            'ins_reasons': rep.ins_reasons, 'slam_reasons': rep.slam_reasons,
            'rot_resid_deg': {f'{k:g}s': round(v, 3) for k, v in rep.rot_resid_deg.items()},
            'cross_check': self._ext_state, 'realign_count': self.realign_count,
            'slam_jumps_removed': self.jr.count,
            'relay': dict(self._n),
        }
        self.pub_state.publish(String(data=json.dumps(st)))

        mk = Marker()
        mk.header.frame_id = self.p['body_frame']
        mk.header.stamp = self.get_clock().now().to_msg()
        mk.ns, mk.id = 'health', 0
        mk.type, mk.action = Marker.TEXT_VIEW_FACING, Marker.ADD
        mk.pose.position.z = 0.35
        mk.pose.orientation.w = 1.0
        mk.scale.z = 0.06
        worst = max(rep.ins_level, rep.slam_level)
        mk.color.r, mk.color.g, mk.color.b = LEVEL_RGB[worst]
        mk.color.a = 1.0
        why = ','.join(rep.ins_reasons + rep.slam_reasons)
        mk.text = (f"INS {LEVEL_NAME[rep.ins_level]} | SLAM {LEVEL_NAME[rep.slam_level]}"
                   + (f"\n{why}" if why else '') + (f"\nrealign {self.realign_count}" if self.realign_count else ''))
        self.pub_marker.publish(mk)


def main():
    rclpy.init()
    node = None
    rc = 0
    try:
        node = HealthMonitorNode()
        rclpy.spin(node)
    except ValueError as exc:
        rclpy.logging.get_logger('health_monitor').error(str(exc))
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
