#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
path_follower.py — 서버 쪽 경로 추종 노드 (ROS 2). Stanley 계열 곡률 제어 (CLAUDE.md)

경로와 맵 위 로버 위치(축 가운데 기준)를 받아 곡률 feedforward + Stanley 피드백으로
곡률 명령을 만들고, 로버에 모터 rpm 과 차량 유효조향각을 발행한다. 제어 로직은
follower_core.py / stanley_core.py / path_cte.py 에 있고, 이 노드는 토픽 입출력·
파라미터·로그만 맡는다. 제어는 고정 주기 타이머에서만 계산한다 (콜백은 저장만).
이전 PID 판은 daron_run/backup/path_follower.py.bak_261011_pre-stanley.

구독
----
  /planner/path            nav_msgs/Path              경로. map [m], 점 간격 ~5 cm, 축 가운데 기준. 빈 경로 = 정지
  /localization/pose       geometry_msgs/PoseStamped  map [m] 로버 위치 + yaw (축 가운데). header.stamp 로 staleness

  입력은 둘 다 map 좌표 [m] 다 (픽셀 아님. follow_test_paths_261011/README.md).
  경로 frame_id 가 map_frame('map') 이 아니면 버린다.
  /rover/odom              nav_msgs/Odometry          휠 오도메트리 (속도 피드백, pose 사이 보간)
  /rover/diagnostics       diagnostic_msgs/DiagnosticArray   wheelbase_mm (굴절각에 따른 휠베이스)
  ~/enable                 std_msgs/Bool              추종 시작/정지

발행
----
  /rover/auto/drive_rpm    std_msgs/Float64    모터 rpm. + 전진 (후진 안 함)
  /rover/auto/steer_cmd    std_msgs/Float64    차량 유효조향각 [deg]. + 좌회전
  /follower/debug          std_msgs/Float32MultiArray   CLAUDE.md §4.8 순서 (follower_core.DEBUG_FIELDS)
  ~/status                 diagnostic_msgs/DiagnosticArray   상태·오차 (state_hz)

  /follower/debug 의 [0] t 는 노드 시작 후 경과 시간 [s] 이다. float32 로는 epoch 초를
  담을 수 없어서다 (해상도 128 s). [1] state 코드는 follower_core.STATE_CODE.

pose staleness
--------------
  pose_stamp_source = header (기본): now - header.stamp. 지상국과 로버의 시간 동기화(chrony)가 전제.
                      stamp 가 0 인 메시지는 수신 시각으로 판단한다.
  pose_stamp_source = receive      : 동기화가 안 됐을 때. 마지막 수신 후 경과 시간으로 판단한다
                      (이 경우 DDS 전송 지연 자체는 보이지 않는다).

실행
----
  python3 path_follower.py --ros-args --params-file config/follower.yaml
  ros2 topic pub --once /path_follower/enable std_msgs/Bool "{data: true}"

추종 파라미터(FollowerParams 의 필드)는 전부 같은 이름의 ROS 파라미터이고
런타임에 바꿀 수 있다:  ros2 param set /path_follower k_e 2.0
"""

from __future__ import annotations

import csv
import dataclasses
import math
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rcl_interfaces.msg import SetParametersResult

from std_msgs.msg import Bool, Float64, Float32MultiArray
from nav_msgs.msg import Odometry, Path as PathMsg
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from follower_core import (DEBUG_FIELDS, HOLD_DEVIATION, STOP_STALE, Follower, FollowerOutput,
                           FollowerParams, speed_to_rpm)
from path_tracking import DEFAULT_WHEELBASE_M, PoseEstimator, TrackerConfig

_LEVEL = {STOP_STALE: DiagnosticStatus.WARN, HOLD_DEVIATION: DiagnosticStatus.ERROR}

_CSV_FIELDS = list(DEBUG_FIELDS) + ["enabled", "x", "y", "yaw_deg", "rpm", "steer_deg",
                                    "wheelbase_m", "message"]


def _yaw(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')

        # ---- 추종 파라미터: FollowerParams 필드를 그대로 ROS 파라미터로 ----
        self.p = FollowerParams()
        self._p_fields = {f.name for f in dataclasses.fields(FollowerParams)}
        self.declare_parameters('', [(f.name, getattr(self.p, f.name))
                                     for f in dataclasses.fields(FollowerParams)])
        for n in self._p_fields:
            setattr(self.p, n, self.get_parameter(n).value)

        # ---- 노드 파라미터 (기존 인터페이스 유지) ----
        self.declare_parameters('', [
            ('path_topic', '/planner/path'),
            ('pose_topic', '/localization/pose'),
            ('odom_topic', '/rover/odom'),
            ('debug_topic', '/follower/debug'),
            ('rover', '/rover'),
            ('map_frame', 'map'),             # 경로 frame_id 검증
            ('pose_stamp_source', 'header'),  # header | receive
            ('control_hz', 50.0),
            ('state_hz', 10.0),
            ('odom_timeout_s', 0.5),          # odom 이 이보다 오래되면 속도 피드백·보간 안 씀
            ('use_odom_propagation', True),
            ('start_enabled', False),         # 기동하자마자 추종할지. 기본은 enable 토픽을 기다린다
            # [261016] 로버 진단(/rover/diagnostics, 10 Hz)이 이만큼 안 오면 추종을 스스로 끈다 = 긴 WiFi 끊김·로버 재시작 뒤엔
            # 확인 없이 다시 출발하지 않는다(제어창 [재개]). 짧은 끊김은 STOP_STALE 로 서 있다가 자동으로 이어 간다. 0 = 끔
            ('link_resume_max_s', 3.0),
            ('allow_reverse', False),         # 전진만 지원. true 여도 후진하지 않는다
            ('log_dir', 'logs'),              # '' = 로그 끔
            # ---- rpm 변환 (기존 그대로) ----
            ('wheel_diameter_m', 0.0), ('gear_ratio', 0.0),
            ('min_rpm', 0.0), ('max_rpm', 500.0),
        ])
        g = lambda n: self.get_parameter(n).value

        self.map_frame = str(g('map_frame'))
        self.stamp_src = str(g('pose_stamp_source'))
        self.odom_timeout = float(g('odom_timeout_s'))
        self.enabled = bool(g('start_enabled'))
        self.link_max_s = float(g('link_resume_max_s'))
        self._diag_rx = None              # [261016] 마지막 /rover/diagnostics 수신 (monotonic)
        self._t_enabled = time.monotonic()  # 켠 시각 — 켠 뒤 link_resume_max_s 안에 진단이 와야 한다
        self._stop_reason = ''            # 'link_lost' = 링크 끊김으로 스스로 끔 (제어창 [재개] 표시용)
        self.tc = TrackerConfig(wheel_diameter_m=g('wheel_diameter_m'), gear_ratio=g('gear_ratio'),
                                min_rpm=g('min_rpm'), max_rpm=g('max_rpm'))

        self.f = Follower(self.p)
        self.est = PoseEstimator(use_odom=bool(g('use_odom_propagation')))
        self.wheelbase_m = DEFAULT_WHEELBASE_M
        self._out = FollowerOutput()
        self._pose = None
        self._pose_stamp = None           # rclpy Time (header.stamp 또는 수신 시각)
        self._t_prev = None
        self._t0 = time.monotonic()
        self._was_enabled = self.enabled
        self._rpm = 0.0

        for e in self._validate():
            self.get_logger().error(f"설정: {e}")
        if bool(g('allow_reverse')):
            self.get_logger().warn("allow_reverse=true 지만 이 추종기는 전진만 합니다.")

        # ---- 인터페이스 ----
        rover = str(g('rover')).rstrip('/')
        self.pub_rpm = self.create_publisher(Float64, f'{rover}/auto/drive_rpm', 10)
        self.pub_steer = self.create_publisher(Float64, f'{rover}/auto/steer_cmd', 10)
        self.pub_debug = self.create_publisher(Float32MultiArray, g('debug_topic'), 10)
        self.pub_status = self.create_publisher(DiagnosticArray, '~/status', 10)

        self.create_subscription(PathMsg, g('path_topic'), self._on_path, 10)
        self.create_subscription(PoseStamped, g('pose_topic'), self._on_pose, 10)
        self.create_subscription(Odometry, g('odom_topic'), self._on_odom, 20)
        self.create_subscription(DiagnosticArray, f'{rover}/diagnostics', self._on_rover_diag, 10)
        self.create_subscription(Bool, '~/enable', self._on_enable, 10)
        self.add_on_set_parameters_callback(self._on_set_parameters)

        self._csv = self._open_log(str(g('log_dir') or ''))
        self.create_timer(1.0 / float(g('control_hz')), self._control_step)
        self.create_timer(1.0 / float(g('state_hz')), self._publish_status)

        rpm_k = self.tc.rpm_per_mps
        self.get_logger().info(
            f"준비. 경로 {g('path_topic')} ({self.map_frame}, m), "
            f"pose {g('pose_topic')} (m, stamp={self.stamp_src}), "
            f"지령 {rover}/auto/*, k_e={self.p.k_e:g}, kappa_max={self.p.kappa_max:g}, "
            f"rpm 범위 [{self.tc.min_rpm:g}, {self.tc.max_rpm:g}], "
            f"rpm/(m/s) {'미설정' if rpm_k is None else f'{rpm_k:.1f}'}, "
            f"{'추종 ON' if self.enabled else '추종 OFF (~/enable 대기)'}")

    # ------------------------------------------------------------
    def _validate(self):
        errs = self.tc.validate()
        if self.stamp_src not in ('header', 'receive'):
            errs.append("pose_stamp_source 는 header | receive")
        if self.p.gain_scale != 1.0:
            errs.append(f"gain_scale={self.p.gain_scale} — 이번 범위에서는 1.0 고정입니다.")
        if not 0.5 <= self.p.understeer_gain <= 2.0:
            errs.append(f"understeer_gain={self.p.understeer_gain} — 0.5 ~ 2.0 (1 = 보정 안 함). "
                        "0 이하면 1 로 씁니다.")
        if not 0 < self.p.v_min <= self.p.v_max_mps:
            errs.append("0 < v_min <= v_max_mps 이어야 합니다.")
        if not self.p.e1 < self.p.e2:
            errs.append("e1 < e2 이어야 합니다.")
        return errs

    def _open_log(self, log_dir):
        if not log_dir:
            return None
        if not os.path.isabs(log_dir):
            log_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), log_dir)
        os.makedirs(log_dir, exist_ok=True)
        path = os.path.join(log_dir, time.strftime('follow_%Y%m%d_%H%M%S.csv'))
        f = open(path, 'w', newline='')
        w = csv.DictWriter(f, fieldnames=_CSV_FIELDS)
        w.writeheader()
        self.get_logger().info(f"로그: {path}")
        return f, w

    # ------------------------------------------------------------
    # 구독 (저장만 한다. 제어 계산은 타이머에서)
    # ------------------------------------------------------------
    def _on_path(self, msg: PathMsg):
        if not msg.poses:                 # 빈 경로 = 목표 도착. 정지 쪽이므로 frame 은 보지 않는다
            self.get_logger().info(f"경로 수신: {self.f.submit_path([])}")
            return
        if msg.header.frame_id != self.map_frame:
            self.get_logger().warn(f"경로 버림: frame_id '{msg.header.frame_id}' != "
                                   f"'{self.map_frame}' (기존 경로 유지)")
            return
        # orientation(yaw) 은 쓰지 않는다. 접선·곡률은 x, y 로 다시 구한다
        pts = [(ps.pose.position.x, ps.pose.position.y) for ps in msg.poses]
        try:
            info = self.f.submit_path(pts)
        except ValueError as e:
            self.get_logger().warn(f"경로 버림: {e} (기존 경로 유지)")
            return
        self.get_logger().info(f"경로 수신: {info}")

    def _on_pose(self, msg: PoseStamped):
        x, y, yaw = msg.pose.position.x, msg.pose.position.y, _yaw(msg.pose.orientation)
        if not all(math.isfinite(v) for v in (x, y, yaw)):
            return
        stamp = Time.from_msg(msg.header.stamp)
        if self.stamp_src != 'header' or stamp.nanoseconds == 0:
            stamp = self.get_clock().now()
        self._pose_stamp = stamp
        self.est.on_map_pose(x, y, yaw, time.monotonic())

    def _on_odom(self, msg: Odometry):
        p = msg.pose.pose
        self.est.on_odom(p.position.x, p.position.y, _yaw(p.orientation),
                         msg.twist.twist.linear.x, time.monotonic())

    def _on_rover_diag(self, msg: DiagnosticArray):
        self._diag_rx = time.monotonic()
        for st in msg.status:
            for kv in st.values:
                if kv.key == 'wheelbase_mm':
                    try:
                        v = float(kv.value)
                    except ValueError:
                        return
                    if v > 0:
                        self.wheelbase_m = v * 1e-3
                    return

    def _on_enable(self, msg: Bool):
        v = bool(msg.data)
        if v != self.enabled:
            self.get_logger().info(f"추종 {'ON' if v else 'OFF'}")
            if v:
                self._t_enabled = time.monotonic()
                self._stop_reason = ''
        self.enabled = v

    def _on_set_parameters(self, params):
        for prm in params:
            if prm.name in self._p_fields:
                try:
                    cur = getattr(self.p, prm.name)
                    setattr(self.p, prm.name, type(cur)(prm.value))
                except (TypeError, ValueError) as e:
                    return SetParametersResult(successful=False, reason=f"{prm.name}: {e}")
            elif prm.name == 'odom_timeout_s':
                self.odom_timeout = float(prm.value)
            elif prm.name == 'link_resume_max_s':
                self.link_max_s = float(prm.value)
        errs = self._validate()
        if errs:
            self.get_logger().warn("; ".join(errs))
        return SetParametersResult(successful=True)

    # ------------------------------------------------------------
    # 제어 (고정 주기)
    # ------------------------------------------------------------
    def _pose_age(self) -> float:
        if self._pose_stamp is None:
            return math.inf
        age = (self.get_clock().now() - self._pose_stamp).nanoseconds * 1e-9
        if age < -0.05:
            self.get_logger().warn(f"pose stamp 가 {-age:.2f} s 미래입니다. 시간 동기화를 확인하세요.",
                                   throttle_duration_sec=5.0)
        return age

    def _control_step(self):
        now = time.monotonic()
        dt = 0.0 if self._t_prev is None else now - self._t_prev
        self._t_prev = now

        odom_ok = self.est.odom_fresh(now, self.odom_timeout)
        pose = None
        if self.est.map_pose is not None:
            pose = self.est.estimate() if odom_ok else self.est.map_pose
        self._pose = pose

        if self.enabled and self.link_max_s > 0:
            gap = now - max(self._diag_rx or -1e9, self._t_enabled)
            if gap > self.link_max_s:
                self.enabled = False
                self._stop_reason = 'link_lost'
                self.get_logger().warn(f"로버 진단 {gap:.1f} s 끊김 (> {self.link_max_s:.1f} s) → 추종 끔. "
                                       "링크가 돌아오면 확인 뒤 제어창 [재개] (긴 끊김·로버 재시작 뒤 무확인 출발 방지)")

        out = self.f.step(dt, pose, self._pose_age(), self.est.odom_v if odom_ok else None,
                          self.enabled, self.wheelbase_m)
        self._out = out
        if out.state == STOP_STALE:
            self.get_logger().warn(out.message, throttle_duration_sec=2.0)
        elif out.state == HOLD_DEVIATION:
            self.get_logger().error(out.message, throttle_duration_sec=2.0)
        t = now - self._t0
        self.pub_debug.publish(Float32MultiArray(data=out.debug_array(t)))

        if not self.enabled:
            self._rpm = 0.0
            if self._was_enabled:
                self._send(0.0, 0.0)      # 끄는 순간 한 번 0 을 보내고 발행을 멈춘다
                self._was_enabled = False
            return
        self._was_enabled = True

        if self.tc.rpm_per_mps is None:
            self.get_logger().error("wheel_diameter_m / gear_ratio 미설정 — 구동하지 않습니다.",
                                    throttle_duration_sec=5.0)
        self._rpm = speed_to_rpm(out.v_cmd, self.tc.rpm_per_mps, self.tc.min_rpm, self.tc.max_rpm)
        self._send(self._rpm, math.degrees(out.delta_cmd))
        self._log(t)

    def _send(self, rpm, steer_deg):
        self.pub_rpm.publish(Float64(data=float(rpm)))
        self.pub_steer.publish(Float64(data=float(steer_deg)))

    def _log(self, t):
        if not self._csv:
            return
        o, p = self._out, self._pose
        row = dict(zip(DEBUG_FIELDS, o.debug_array(t)))
        row.update(state=o.state, enabled=self.enabled,
                   x=p[0] if p else '', y=p[1] if p else '',
                   yaw_deg=math.degrees(p[2]) if p else '', rpm=self._rpm,
                   steer_deg=math.degrees(o.delta_cmd), wheelbase_m=self.wheelbase_m,
                   message=o.message)
        self._csv[1].writerow(row)

    def _publish_status(self):
        o = self._out
        st = DiagnosticStatus()
        st.name = f"{self.get_name()}: follower"
        st.level = _LEVEL.get(o.state, DiagnosticStatus.OK)
        st.message = f"{o.state} {o.message}".strip()
        f = lambda v, d=3: f"{v:.{d}f}" if isinstance(v, (int, float)) and math.isfinite(v) else ''
        p = self._pose
        st.values = [
            KeyValue(key='state', value=o.state),
            KeyValue(key='enabled', value=str(self.enabled)),
            KeyValue(key='stop_reason', value=self._stop_reason),       # [261016] 'link_lost' = 링크 끊김으로 스스로 끔
            KeyValue(key='rover_diag_age_s', value=f(time.monotonic() - self._diag_rx, 1) if self._diag_rx else ''),
            KeyValue(key='path_id', value=str(o.path_id)),
            KeyValue(key='remain_m', value=f(o.remaining)),
            KeyValue(key='x', value=f(p[0]) if p else ''),
            KeyValue(key='y', value=f(p[1]) if p else ''),
            KeyValue(key='yaw_deg', value=f(math.degrees(p[2]), 1) if p else ''),
            KeyValue(key='pose_age_s', value=f(o.pose_age, 3)),
            KeyValue(key='e_y_m', value=f(o.e_y)),
            KeyValue(key='e_th_deg', value=f(math.degrees(o.e_th), 2)),
            KeyValue(key='kappa_ref', value=f(o.kappa_ref)),
            KeyValue(key='kappa_cmd', value=f(o.kappa_cmd)),
            KeyValue(key='saturated', value=str(o.saturated)),
            KeyValue(key='I', value=f(o.I, 4)),
            KeyValue(key='v_cmd', value=f(o.v_cmd)),
            KeyValue(key='v_meas', value=f(o.v_meas)),
            KeyValue(key='rpm', value=f(self._rpm, 1)),
            KeyValue(key='steer_deg', value=f(math.degrees(o.delta_cmd), 2)),
            KeyValue(key='wheelbase_m', value=f(self.wheelbase_m)),
        ]
        arr = DiagnosticArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.status = [st]
        self.pub_status.publish(arr)

    def destroy_node(self):
        try:
            self._send(0.0, 0.0)
        except Exception:
            pass
        if self._csv:
            self._csv[0].close()
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = PathFollower()
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
