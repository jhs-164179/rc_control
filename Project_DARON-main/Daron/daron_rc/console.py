#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
console.py — 지상국 조종 콘솔 (pygame 창). RC 4채널 판 (daron_rc, 261015).

daron_run/rover/console.py 에 RC 패널(창 오른쪽)과 X / R 키를 더한 것이다. 키보드 조종·로그·
화면 왼쪽은 원본과 같다. RC 를 안 켠 radxa.py 에 붙여도 원본처럼 돈다 (RC 패널만 'disabled').

RC 4채널 (radxa.py 를 rc_enable:=true 로 띄웠을 때)
---------------------------------------------------
  X         SW STOP  /rover/control_mode/set 'STOP'. 로버가 래치 정지 (CH1 을 움직여도 안 풀림)
  R         RESUME   'RC'. 래치 해제 + 재arm 요청. 다시 움직이려면 CH2 중립을 0.5 s 유지
  RC 모드에서는 W/S/A/D/Q/E 가 모터로 가지 않는다 (로버가 무시). 1/2/0 형상, SPACE 전복, P 자율 허용은 그대로 먹는다.
  AUTO 는 CH1 스위치로만 들어간다. 이 콘솔의 P(자율 허용)는 그 위의 추가 조건이다.
  자율 허용은 꺼진 채(AUTO PAUSED)로 시작한다 — 원본 console.py 와 다르다 (-p auto_enable_start:=true 로 원본처럼).
  오른쪽 패널: SOURCE(STOP/RC/AUTO), 링크, arm, CH1~CH4 막대(µs), dry 일 때 '켜졌다면 낼 지령'(would),
  지금 왜 안 움직이는지(reason). drive CSV 끝에 rc_* 열이 붙는다.
  ⚠ X 는 소프트웨어 정지다. 물리 E-stop 이 아니며 Wi-Fi 가 끊기면 전달되지 않는다.

조작
----
  W / S     전진 / 후진   누르는 동안 가속, 떼면 감속 정지 (front+back 동시)
  A / D     조향 좌 / 우   누르는 동안 최대 차량 유효조향각, 떼면 중립
  Q / E     굴절 CW / CCW  누르는 동안 회전, 떼면 그 자리 정지
                           Q = 굴절각 감소 (굴절부 올라감), E = 굴절각 증가 (굴절부 내려감)
  SPACE     전복 토글      한 번 누를 때마다 정상 <-> 전복
  1         중립 모드      굴절각 150 deg 로 편다 (조향도 중립으로)
  2         수평 모드      굴절각 180 deg
  0         수납 모드      노드가 수납 상태로 켜졌을 때의 자세로 접는다
  P         자율 허용 토글 (AUTO ON <-> PAUSED). 멈추면 로버가 자율 지령을 무시한다
  X / R     [RC] SW STOP 래치 / 해제 (위 'RC 4채널')
  ESC       종료

  1/2/0 은 누른 순간 한 번만 보낸다. 노드가 큰 전류로 천천히 옮기며, 옮기는
  동안과 수납 상태에서는 구동·조향·굴절 조그가 막힌다. 노드가 수납 상태
  (굴절각 250 deg 이상)로 켜졌으면 1 이나 2 를 누를 때까지 아무것도 움직이지 않는다.

  A+D 또는 Q+E 를 동시에 누르면 해당 축은 정지(중립)한다.

자율주행과 같이 쓸 때
---------------------
  지상국 path_follower 가 rpm/조향을 보내는 동안에도 키보드가 우선한다.
  W/S/A/D 를 누르는 순간 수동으로 넘어가고, 키를 뗀 뒤 1 s(노드의
  manual_override_hold_s) 가 지나면 자율로 돌아간다. 완전히 멈추려면 P.
  형상 전환 중이거나 수납 상태면 자율 지령도 막힌다.

전복 플래그
-----------
  굴절각의 부호와는 별개의 독립 상태다. 로버가 뒤집혀 있으면 위에서 본
  회전 방향이 반대가 되므로, 노드가 유효조향각의 부호를 뒤집어
  조종 감각을 유지한다. 창 상단에 현재 상태가 표시된다.

조향각
------
  보내는 값은 '차량 유효조향각'(전/후방 조향을 합쳐 차량이 실제로 도는 각도)이다.
  노드가 각 축의 유효조향각으로 환산한다 (tan Delta_a = tan Delta_v / 2).
  기본 최대값 43.003 deg 는 기계 한계인 축 25 deg 와 같은 최소 회전반경이다.

서스펜션
--------
  XM430 은 굴절 관절인 동시에 서스펜션이다. 상단 배지에 현재 상태가 뜬다.
      HOLD  ·   밴드 안. 낮은 유지 전류로 물렁하게 받치는 중
      COMPLY ↓  외력이 축을 밀어내는 중. 양보하는 중
      RECOVER ↑ 외력이 사라져 큰 전류로 복귀하는 중
      YIELD !   계속 눌려 있어 복원을 포기하고 대기 (모터 보호)
      CMD   →   조그/절대 지령으로 이동 중
  복원이 한 번 끝날 때마다 터미널에 t90/오버슈트/잔류가 한 줄 찍힌다.

로그
----
  logs/xm430_YYYYmmdd_HHMMSS.csv   XM430 + 서스펜션 상태
  logs/l12_YYYYmmdd_HHMMSS.csv     조향 해 (차량/축 유효조향각, 곡률, 캐스터각,
                                   휠 조향각, 인출 길이)
  logs/drive_YYYYmmdd_HHMMSS.csv   구동모터 요약 (front/back 한 행에 나란히)
  세 파일 모두 노드의 상태 발행 주기(state_hz, 기본 10 Hz)에 맞춰 기록된다.

  구동모터의 '정본' 로그는 RADXA 쪽 VescDriver 의 TelemetryLogger 가
  전체 속도로 남긴다. 여기 drive CSV 는 다른 두 로그와 같은 시각에
  맞춰 비교하기 위한 요약본이다.

  서스펜션도 마찬가지다. 여기 CSV 는 10 Hz 라 복원 지표(t90)를 뽑을 수 없다.
  정본은 RADXA 의 logs/rover_*.csv (susp_local_log, 100 Hz) 이고, 그쪽은
  analyze.py 로 바로 읽힌다:  python3 analyze.py summary "logs/rover_*.csv"

실행
----
  python3 console.py
  python3 console.py --ros-args -p log_dir:=/data/logs -p max_steer_deg:=30.0

의존성
------
  pip install pygame
  창이 떠야 키 이벤트를 받는다. 헤드리스 환경에서는 동작하지 않는다.
  WSL2 는 WSLg 가 있으면 그대로 뜬다. SSH 라면 X11 forwarding(ssh -X) 필요.
"""

from __future__ import annotations

import datetime
import math
import os
import sys
import threading
import time

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64, String, Int8, Empty, Bool
from sensor_msgs.msg import JointState
from diagnostic_msgs.msg import DiagnosticArray

if not os.environ.get("XDG_RUNTIME_DIR"):
    os.environ["XDG_RUNTIME_DIR"] = "/tmp"

import pygame

TELEOP_HZ = 10.0          # 노드의 teleop_timeout(0.5 s)보다 충분히 빨라야 한다
HEARTBEAT_HZ = 4.0
FPS = 60                  # 창 갱신 및 키 폴링 주기
STALE_S = 2.0

LEFT_W = 700                     # 원본 콘솔 영역 폭
RC_W = 320                       # RC 패널 (오른쪽, daron_rc 261015)
WIN_W, WIN_H = LEFT_W + RC_W, 1010   # 배터리 두 줄 (261010) + MODE 배지·자율 세 줄 (261011)

# 차량 유효조향각 기본 최대값 [deg]. 기계 한계(축 25 deg)와 같은 최소 회전반경.
# 지상국에 l12_driver 가 있으면 그 값을 쓰고, 없으면 같은 식으로 계산한다.
# 노드도 자체 한계로 다시 clamp 하므로 여기 값이 더 커도 안전하다.
try:
    from l12_driver import MAX_STEER_DEG as DEFAULT_MAX_STEER_DEG
except ImportError:
    _AXLE_STEER_LIMIT_DEG = 25.0      # l12_driver.AXLE_STEER_LIMIT_DEG 와 같아야 한다
    DEFAULT_MAX_STEER_DEG = math.degrees(
        math.atan(2.0 * math.tan(math.radians(_AXLE_STEER_LIMIT_DEG))))

XM430_HEADER = [
    "iso_time", "t_mono", "state", "position_rad", "position_deg",
    "motor_deg", "goal_deg", "error_deg", "articulation_deg", "alpha_eff_deg",
    "inverted", "velocity_rad_s", "current_mA", "torque_mNm",
    "temperature_C", "hw_error", "loop_hz", "jog",
    # ---- 서스펜션 (정본은 RADXA 의 100 Hz rover_*.csv) ----
    "susp_state", "goal_current_lsb", "goal_current_mNm", "susp_hz",
    "susp_hz_target", "outside_travel_deg", "recoveries",
    "last_t90_ms", "last_overshoot_pct", "last_osc", "last_residual_deg",
    "last_outcome",
]

SUSP_FIELDS = [
    "susp_state", "goal_current_lsb", "goal_current_mNm", "susp_hz",
    "susp_hz_target", "outside_travel_deg", "recoveries",
    "last_t90_ms", "last_overshoot_pct", "last_osc", "last_residual_deg",
    "last_outcome",
]

# 서스펜션 상태 표시 기호. run_suspension.py 와 같게 둔다.
STATE_MARK = {"HOLD": "·", "COMPLY": "↓", "RECOVER": "↑", "YIELD": "!",
              "CMD": "→", "SHAPE": "⇄", "LOCK": "■"}

L12_HEADER = [
    "iso_time", "t_mono", "inverted", "motor_deg",
    "articulation_deg", "alpha_eff_deg",
    "steer_cmd_deg", "steer_eff_deg", "steer_limit_deg",
    "axle_front_deg", "axle_rear_deg", "axle_limit_deg",
    "wheelbase_mm", "curvature_per_m",
    "gamma_front_deg", "gamma_rear_deg",
    "delta_front_deg", "delta_rear_deg",
    "l12_front_mm", "l12_rear_mm", "l12_output",
]

# 구동모터는 front/back 두 채널이다. 공통 지령 다음에 역할별 컬럼을 나란히 둔다.
# 한 행에 양쪽을 담으므로 전/후 전류 분담이나 ERPM 대칭성을 바로 비교할 수 있다.
DRIVE_ROLES = ("front", "back")
DRIVE_PER_ROLE = (
    "enabled", "sign", "target", "rpm", "erpm", "voltage_v",
    "current_a", "input_a", "iq_a", "torque_est_nm", "temp_c", "fault", "error",
    "battery_pct",
)
# 배터리 요약 (노드가 front/back 중 낮은 쪽으로 낸 값). 맨 뒤에 둬서 기존 열 위치를 안 바꾼다.
BATTERY_FIELDS = ("battery_pct", "battery_voltage_v", "battery_wh")
# 자율 지령·오도메트리 (261011). 역시 맨 뒤에 둔다.
AUTO_FIELDS = ("drive_mode", "auto_enable", "auto_rpm", "auto_steer_deg",
               "odom_x", "odom_y", "odom_yaw_deg", "odom_v")
# RC 4채널 (daron_rc 261015). 맨 뒤에 둔다 — 앞 열은 원본 console.py 와 같다.
RC_FIELDS = ("rc_source", "rc_switch", "rc_link", "rc_armed", "rc_auto_ready",
             "rc_latched", "rc_would_mode", "rc_would_steer_deg", "rc_would_drive",
             "rc_would_jog", "rc_ch1", "rc_ch2", "rc_ch3", "rc_ch4", "rc_reason")
DRIVE_HEADER = (
    ["iso_time", "t_mono", "inverted", "drive_cmd", "drive_applied", "drive_armed"]
    + [f"{r}_{f}" for r in DRIVE_ROLES for f in DRIVE_PER_ROLE]
    + list(BATTERY_FIELDS) + list(AUTO_FIELDS) + list(RC_FIELDS)
)


def battery_low(d: dict) -> bool:
    """노드가 낸 battery_pct 가 battery_warn_pct 미만인가 (모르면 False)."""
    try:
        return float(d.get('battery_pct')) < float(d.get('battery_warn_pct') or 20.0)
    except (TypeError, ValueError):
        return False


def battery_text(d: dict) -> str:
    """'87.5 %  11.92 V  (front)' — 잔량을 모르면 전압만, 둘 다 모르면 '?'."""
    pct, volt = d.get('battery_pct', ''), d.get('battery_voltage_v', '')
    parts = ([f'{pct} %'] if pct else []) + ([f'{volt} V'] if volt else [])
    if not parts:
        return '? (VESC 응답 없음)'
    src = d.get('battery_source', '')
    return '  '.join(parts) + (f'  ({src})' if src else '')


# ============================================================
# 콘솔 노드
# ============================================================
class Console(Node):

    def __init__(self):
        super().__init__('console')
        self.declare_parameter('target', '/rover')
        self.declare_parameter('joint_name', 'articulation')
        self.declare_parameter('max_steer_deg', DEFAULT_MAX_STEER_DEG)
        self.declare_parameter('log_dir', 'logs')
        # RC 판: 자율 허용을 끈 채 시작한다 (원본은 true). CH1 AUTO + 이 값 + 추종기 지령이 모두 있어야 자율.
        self.declare_parameter('auto_enable_start', False)

        tgt = self.get_parameter('target').value.rstrip('/')
        self.target = tgt
        self.joint_name = self.get_parameter('joint_name').value
        self.max_steer = float(self.get_parameter('max_steer_deg').value)

        self.pub_steer = self.create_publisher(Float64, f'{tgt}/steer_cmd', 10)
        self.pub_jog = self.create_publisher(Int8, f'{tgt}/articulation_cmd', 10)
        self.pub_drive = self.create_publisher(Int8, f'{tgt}/drive_cmd', 10)
        self.pub_inv = self.create_publisher(Bool, f'{tgt}/inverted', 10)
        self.pub_cmd = self.create_publisher(String, f'{tgt}/command', 10)
        self.pub_auto = self.create_publisher(Bool, f'{tgt}/auto_enable', 10)
        self.pub_hb = self.create_publisher(Empty, f'{tgt}/heartbeat', 10)
        self.pub_mode = self.create_publisher(String, f'{tgt}/control_mode/set', 10)

        self.create_subscription(JointState, '/joint_states', self._on_js, 10)
        self.create_subscription(DiagnosticArray, f'{tgt}/diagnostics',
                                 self._on_diag, 10)

        self._lock = threading.Lock()
        self._recoveries = 0          # 복원 횟수. 늘어나면 터미널에 한 줄 찍는다
        self._js = None
        self._diag = {}
        self._diag_msg = ''
        self._last_rx = 0.0

        # 키 상태는 pygame 루프가 갱신하고 타이머 스레드가 읽는다
        self.keys = {k: False for k in 'wasdqe'}
        self.inverted = False
        # P 키로 토글. 노드는 false 면 자율 지령을 무시한다
        self.auto_enable = bool(self.get_parameter('auto_enable_start').value)
        self.rows_xm430 = 0
        self.rows_l12 = 0

        # ---------- 로그 ----------
        log_dir = self.get_parameter('log_dir').value
        os.makedirs(log_dir, exist_ok=True)
        ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
        self.xm430_path = os.path.join(log_dir, f'xm430_{ts}.csv')
        self.l12_path = os.path.join(log_dir, f'l12_{ts}.csv')
        self.drive_path = os.path.join(log_dir, f'drive_{ts}.csv')
        self._log_xm430 = open(self.xm430_path, 'w', buffering=1, encoding='utf-8')
        self._log_l12 = open(self.l12_path, 'w', buffering=1, encoding='utf-8')
        self._log_drive = open(self.drive_path, 'w', buffering=1, encoding='utf-8')
        self._log_xm430.write(','.join(XM430_HEADER) + '\n')
        self._log_l12.write(','.join(L12_HEADER) + '\n')
        self._log_drive.write(','.join(DRIVE_HEADER) + '\n')
        self._t0 = time.monotonic()

        self.create_timer(1.0 / TELEOP_HZ, self._send_teleop)
        self.create_timer(1.0 / HEARTBEAT_HZ, lambda: self.pub_hb.publish(Empty()))

        print(f"대상 노드 : {tgt}")
        print(f"조향 지령 : +-{self.max_steer:.3f} deg (차량 유효조향각)")
        print(f"로그      : {self.xm430_path}")
        print(f"          : {self.l12_path}")
        print(f"          : {self.drive_path}")

    # ---------- 송신 ----------
    def _send_teleop(self):
        """조향각·조그·전복을 10 Hz 로 계속 보낸다.

        노드의 텔레옵 워치독이 이 흐름을 감시한다. 끊기면 노드가 스스로
        중립으로 돌아가므로, 링크가 죽어도 액추에이터가 꺾인 채 남지 않는다.
        """
        with self._lock:
            k = dict(self.keys)
            inv = self.inverted
            auto = self.auto_enable

        # A + D 동시 -> 중립.  Q + E 동시 -> 정지.
        steer = 0.0
        if k['a'] != k['d']:
            steer = self.max_steer if k['a'] else -self.max_steer
        jog = 0
        if k['q'] != k['e']:
            # q = CW = 위치 감소 = -1,  e = CCW = 위치 증가 = +1
            jog = -1 if k['q'] else +1
        # W + S 동시 -> 정지.  전복 보정은 노드가 한다(조종자 기준으로 보낸다).
        drive = 0
        if k['w'] != k['s']:
            drive = +1 if k['w'] else -1

        self.pub_steer.publish(Float64(data=steer))
        self.pub_jog.publish(Int8(data=jog))
        self.pub_drive.publish(Int8(data=drive))
        self.pub_inv.publish(Bool(data=inv))
        self.pub_auto.publish(Bool(data=auto))

    # ---------- 수신 ----------
    def _on_js(self, msg: JointState):
        if self.joint_name not in msg.name:
            return
        i = msg.name.index(self.joint_name)
        with self._lock:
            self._js = (msg.position[i] if i < len(msg.position) else 0.0,
                        msg.velocity[i] if i < len(msg.velocity) else 0.0,
                        msg.effort[i] if i < len(msg.effort) else 0.0)
            self._last_rx = time.monotonic()

    def _on_diag(self, msg: DiagnosticArray):
        """상태 수신 시점에 두 CSV 에 한 줄씩 쓴다."""
        if not msg.status:
            return
        st = msg.status[0]
        with self._lock:
            self._diag = {kv.key: kv.value for kv in st.values}
            self._diag_msg = st.message
            js = self._js
            d = dict(self._diag)

        # 복원이 끝날 때마다 한 줄. 지표 자체는 노드가 계산한 것을 받아 적는다.
        try:
            n = int(d.get('recoveries') or 0)
        except ValueError:
            n = self._recoveries
        if n > self._recoveries:
            self._recoveries = n
            way = '-CW ' if (d.get('last_dir') or '').startswith('-') else '+CCW'
            t90 = d.get('last_t90_ms') or '--'
            print(f"복원 #{n:<3d} {way}  변위 {d.get('last_err0_deg', '?')} deg"
                  f"  t90 {t90} ms  OS {d.get('last_overshoot_pct', '?')} %"
                  f"  진동 {d.get('last_osc', '?')}"
                  f"  잔류 {d.get('last_residual_deg', '?')} deg"
                  f"  {d.get('last_outcome', '')}")

        if js is None:
            return

        pos, vel, _eff = js
        now_iso = datetime.datetime.now().isoformat(timespec='milliseconds')
        t = f"{time.monotonic() - self._t0:.4f}"

        xm = [
            now_iso, t, d.get('state', ''),
            f"{pos:.6f}", f"{math.degrees(pos):.4f}",
            d.get('motor_deg', ''), d.get('goal_deg', ''), d.get('error_deg', ''),
            d.get('articulation_deg', ''), d.get('alpha_eff_deg', ''),
            d.get('inverted', ''), f"{vel:.6f}",
            d.get('current_mA', ''), d.get('torque_mNm', ''),
            d.get('temperature_C', ''), d.get('hw_error', ''),
            d.get('loop_hz', ''), d.get('jog', ''),
        ] + [d.get(k, '') for k in SUSP_FIELDS]
        l12 = [
            now_iso, t, d.get('inverted', ''), d.get('motor_deg', ''),
            d.get('articulation_deg', ''), d.get('alpha_eff_deg', ''),
            d.get('steer_cmd_deg', ''), d.get('steer_eff_deg', ''),
            d.get('steer_limit_deg', ''),
            d.get('axle_front_deg', ''), d.get('axle_rear_deg', ''),
            d.get('axle_limit_deg', ''),
            d.get('wheelbase_mm', ''), d.get('curvature_per_m', ''),
            d.get('gamma_front_deg', ''), d.get('gamma_rear_deg', ''),
            d.get('delta_front_deg', ''), d.get('delta_rear_deg', ''),
            d.get('l12_front_mm', ''), d.get('l12_rear_mm', ''),
            d.get('l12_output', ''),
        ]
        def clean(v):
            return str(v).replace(',', ';')
        drive = [
            now_iso, t, d.get('inverted', ''),
            d.get('drive_cmd', ''), d.get('drive_applied', ''), d.get('drive_armed', ''),
        ]
        for role in DRIVE_ROLES:
            for field in DRIVE_PER_ROLE:
                drive.append(clean(d.get(f'drive_{role}_{field}', '')))
        drive += [clean(d.get(k, '')) for k in BATTERY_FIELDS + AUTO_FIELDS + RC_FIELDS]
        try:
            self._log_xm430.write(','.join(xm) + '\n')
            self._log_l12.write(','.join(l12) + '\n')
            self._log_drive.write(','.join(drive) + '\n')
            self.rows_xm430 += 1
            self.rows_l12 += 1
        except Exception as e:
            print(f"[WARN] 로그 기록 실패: {e!r}")

    def send_shape(self, cmd: str):
        """형상 모드 지령 (neutral / flat / stow). 키를 누른 순간 한 번만."""
        self.pub_cmd.publish(String(data=cmd))
        print(f"형상 모드 -> {cmd}")

    def send_control_mode(self, req: str):
        """RC 중재 요청 (STOP = 래치 정지, RC = 래치 해제·재arm). 누른 순간 한 번.
        전달 여부는 RC 패널의 LATCHED 표시(로버 진단)로 확인한다."""
        self.pub_mode.publish(String(data=req))
        print(f"RC 요청 -> {req}")

    def snapshot(self):
        with self._lock:
            return (self._js, dict(self._diag), self._diag_msg,
                    time.monotonic() - self._last_rx if self._last_rx else 1e9)

    # ---------- 종료 ----------
    def shutdown(self):
        try:
            self.pub_steer.publish(Float64(data=0.0))
            self.pub_jog.publish(Int8(data=0))
            self.pub_drive.publish(Int8(data=0))
            self.pub_cmd.publish(String(data='stop'))
            time.sleep(0.3)
        except Exception:
            pass
        for f in (self._log_xm430, self._log_l12, self._log_drive):
            try:
                f.close()
            except Exception:
                pass
        print(f"\n로그 저장: {self.xm430_path} ({self.rows_xm430} rows)")
        print(f"          {self.l12_path} ({self.rows_l12} rows)")
        print(f"          {self.drive_path} ({self.rows_l12} rows)")


# ============================================================
# pygame 창
# ============================================================
COL_BG = (18, 18, 22)
COL_TXT = (225, 225, 230)
COL_DIM = (120, 120, 130)
COL_ON = (90, 200, 120)
COL_OFF = (55, 55, 62)
COL_WARN = (235, 170, 60)
COL_ERR = (230, 90, 90)


def _low_hz(actual, target, floor=0.8) -> bool:
    """
    서스펜션 루프가 설정 주기의 floor 배 아래로 떨어졌는가.

    기준(target)은 노드가 진단으로 같이 보낸다. 여기서 100 Hz 를 가정하면
    susp_hz 를 낮춰 띄운 경우 늘 경고가 켜진다.
    """
    try:
        a, t = float(actual), float(target)
    except (TypeError, ValueError):
        return False
    return 0.0 < a < t * floor


def _font(size, bold=False):
    return pygame.font.SysFont('dejavusansmono,couriernew,monospace', size, bold=bold)


# ============================================================
# RC 패널 (오른쪽). 글자는 영문만 — 위 폰트에 한글 글리프가 없다.
# ============================================================
RC_BAR_LO, RC_BAR_HI = 900, 2100          # 막대 눈금 범위 [µs]
RC_NAMES = ('CH1 MODE', 'CH2 DRIVE', 'CH3 STEER', 'CH4 XM430')


def _int(v, default=None):
    try:
        return int(float(v))
    except (TypeError, ValueError):
        return default


def draw_rc_panel(screen, fonts, d: dict, live: bool, x0: int, w: int):
    """진단의 rc_* 값으로 RC 상태를 그린다. live=False 면 (링크 없음/STALE) 값 대신 '--'."""
    f_big, f, f_small = fonts
    x, y = x0 + 14, 14
    pygame.draw.line(screen, COL_OFF, (x0, 0), (x0, WIN_H), 2)

    def text(s, font=f_small, col=COL_TXT, dx=0):
        nonlocal y
        screen.blit(font.render(s, True, col), (x + dx, y))
        y += font.get_linesize() + 2

    title = f_big.render('RC 4CH', True, COL_TXT)
    screen.blit(title, (x, y))
    if not live:
        screen.blit(f_small.render('no rover diagnostics', True, COL_DIM), (x + 100, y + 5))
        return
    if d.get('rc_enabled') != 'True':
        y += 34
        text('disabled on rover', f, COL_DIM)
        text('(radxa.py rc_enable:=false)', f_small, COL_DIM)
        text('X / R keys do nothing', f_small, COL_DIM)
        return
    if 'rc_source' not in d:
        y += 34
        text('waiting for arbiter...', f, COL_DIM)
        return

    dry = d.get('rc_actuation') != 'True'
    tag = f_big.render('DRY' if dry else 'LIVE', True, COL_WARN if dry else COL_ERR)
    screen.blit(tag, (x0 + w - tag.get_width() - 14, y))
    y += 34

    # SOURCE 배지
    src = d.get('rc_source', '?')
    armed = d.get('rc_armed') == 'True'
    ready = d.get('rc_auto_ready') == 'True'
    scol = {'STOP': COL_ERR, 'RC': COL_ON if armed else COL_WARN,
            'AUTO': COL_ON if ready else COL_WARN}.get(src, COL_DIM)
    box = pygame.Rect(x, y, w - 28, 40)
    pygame.draw.rect(screen, scol, box, width=2, border_radius=6)
    s = f_big.render(f'SOURCE {src}', True, scol)
    screen.blit(s, (box.centerx - s.get_width() // 2, box.y + 9))
    y += 48

    if d.get('rc_latched') == 'True':
        box = pygame.Rect(x, y, w - 28, 26)
        pygame.draw.rect(screen, COL_ERR, box, border_radius=4)
        s = f.render('SW STOP LATCHED - R to resume', True, COL_BG)
        screen.blit(s, (box.centerx - s.get_width() // 2, box.y + 4))
        y += 32

    link = d.get('rc_link') == 'True'
    text(f"link   {'OK' if link else 'NO SIGNAL'}", f, COL_ON if link else COL_ERR)
    text(f"CH1 -> {d.get('rc_switch', '?')}", f)
    text(f"armed {'YES' if armed else 'no '}   auto ready {'YES' if ready else 'no'}", f,
         COL_TXT)
    am, wm = d.get('drive_mode', '?'), d.get('rc_would_mode', '?')
    text(f"active {am}   would {wm}", f, COL_WARN if dry and wm != 'idle' else COL_TXT)
    y += 6

    # CH1~CH4 막대
    center = _int(d.get('rc_center_us'), 1500)
    lo, hi = _int(d.get('rc_mode_low_us'), 1300), _int(d.get('rc_mode_high_us'), 1700)
    bw = w - 28
    bx = x

    def px(us):
        us = max(RC_BAR_LO, min(RC_BAR_HI, us))
        return bx + int((us - RC_BAR_LO) / (RC_BAR_HI - RC_BAR_LO) * bw)

    for i, name in enumerate(RC_NAMES):
        v = _int(d.get(f'rc_ch{i + 1}'))
        screen.blit(f_small.render(name, True, COL_DIM), (x, y))
        vs = f.render('----' if v is None else f'{v:4d} us', True, COL_TXT if v else COL_DIM)
        screen.blit(vs, (x0 + w - vs.get_width() - 14, y - 2))
        y += 18
        bar = pygame.Rect(bx, y, bw, 14)
        pygame.draw.rect(screen, COL_OFF, bar, border_radius=3)
        if i == 0:      # CH1: RC | STOP | AUTO 구간
            pygame.draw.rect(screen, (60, 40, 40), pygame.Rect(px(lo), y, px(hi) - px(lo), 14))
            for t in (lo, hi):
                pygame.draw.line(screen, COL_DIM, (px(t), y - 2), (px(t), y + 15), 1)
        pygame.draw.line(screen, COL_DIM, (px(center), y - 2), (px(center), y + 15), 1)
        if v is not None:
            a, b = sorted((px(center), px(v)))
            pygame.draw.rect(screen, COL_ON if link else COL_DIM,
                             pygame.Rect(a, y + 3, max(2, b - a), 8))
        y += 22

    # 켜졌다면 낼 지령 (dry 확인용) — RC 수동 arm 상태에서만 0 이 아님
    y += 2
    text(f"steer {d.get('rc_would_steer_deg', '?'):>7} deg", f)
    jog = d.get('rc_would_jog', '?')
    ch4 = '' if d.get('rc_ch4_enabled') == 'True' else ' (CH4 off)'
    text(f"drive {d.get('rc_would_drive', '?'):>3}   jog {jog:>3}{ch4}", f)
    y += 6

    # 왜 안 움직이는가
    reason = d.get('rc_reason', '')
    words, line = reason.split(), ''
    for wd in words:
        if f.size(line + ' ' + wd)[0] > w - 28 and line:
            text(line, f, COL_WARN)
            line = wd
        else:
            line = (line + ' ' + wd).strip()
    if line:
        text(line, f, COL_WARN)
    y += 10
    text('X  SW STOP (latch)', f_small, COL_DIM)
    text('R  resume / re-arm request', f_small, COL_DIM)
    text('WASD/QE ignored while RC on', f_small, COL_DIM)
    text('P  auto_enable (AUTO also needs CH1)', f_small, COL_DIM)


def run_ui(node: Console) -> None:
    pygame.init()
    pygame.display.set_caption('Rover Console')
    screen = pygame.display.set_mode((WIN_W, WIN_H))
    clock = pygame.time.Clock()
    f_big = _font(20, bold=True)
    f = _font(15)
    f_small = _font(13)

    key_map = {
        pygame.K_w: 'w', pygame.K_a: 'a', pygame.K_s: 's',
        pygame.K_d: 'd', pygame.K_q: 'q', pygame.K_e: 'e',
    }
    layout = [('W', 'w'), ('A', 'a'), ('S', 's'), ('D', 'd'), ('Q', 'q'), ('E', 'e')]
    shape_keys = {
        pygame.K_1: 'neutral', pygame.K_KP1: 'neutral',
        pygame.K_2: 'flat', pygame.K_KP2: 'flat',
        pygame.K_0: 'stow', pygame.K_KP0: 'stow',
    }

    running = True
    while running:
        # ---------- 이벤트 ----------
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                running = False
            elif ev.type == pygame.KEYDOWN:
                if ev.key == pygame.K_ESCAPE:
                    running = False
                elif ev.key == pygame.K_SPACE:
                    # 스페이스바는 '눌린 순간'에만 토글한다 (autorepeat 무시)
                    with node._lock:
                        node.inverted = not node.inverted
                elif ev.key in shape_keys:
                    node.send_shape(shape_keys[ev.key])
                elif ev.key == pygame.K_p:
                    with node._lock:
                        node.auto_enable = not node.auto_enable
                        print(f"자율 {'허용' if node.auto_enable else '일시정지'}")
                elif ev.key == pygame.K_x:
                    node.send_control_mode('STOP')
                elif ev.key == pygame.K_r:
                    node.send_control_mode('RC')
                elif ev.key in key_map:
                    with node._lock:
                        node.keys[key_map[ev.key]] = True
            elif ev.type == pygame.KEYUP:
                if ev.key in key_map:
                    with node._lock:
                        node.keys[key_map[ev.key]] = False

        # 창이 포커스를 잃으면 KEYUP 을 놓칠 수 있다. 눌림 상태를 매 프레임 동기화.
        pressed = pygame.key.get_pressed()
        with node._lock:
            for code, name in key_map.items():
                node.keys[name] = bool(pressed[code])
            keys = dict(node.keys)
            inverted = node.inverted
            auto_enable = node.auto_enable

        js, d, dmsg, age = node.snapshot()

        # ---------- 렌더 ----------
        screen.fill(COL_BG)
        y = 14

        # 링크 상태
        if js is None:
            screen.blit(f_big.render('NO LINK', True, COL_ERR), (16, y))
            screen.blit(f_small.render(
                f'{node.target} 노드와 ROS_DOMAIN_ID 를 확인하세요', True, COL_DIM),
                (140, y + 5))
        elif age > STALE_S:
            screen.blit(f_big.render(f'STALE {age:.1f}s', True, COL_WARN), (16, y))
        else:
            state = d.get('state', '?')
            col = COL_ERR if dmsg and dmsg not in (state,) else COL_ON
            if state == 'YIELD':
                col = COL_WARN
            mark = STATE_MARK.get(state, ' ')
            screen.blit(f_big.render(f'{mark} {state}', True, col), (16, y))
            if dmsg and dmsg != state:
                screen.blit(f_small.render(dmsg, True, COL_WARN), (140, y + 5))

        # 전복 배지
        badge = 'INVERTED' if inverted else 'NORMAL'
        bcol = COL_WARN if inverted else COL_ON
        surf = f_big.render(badge, True, bcol)
        screen.blit(surf, (LEFT_W - surf.get_width() - 16, y))
        y += 34

        # 키 표시
        x = 16
        for label, name in layout:
            on = keys[name]
            rect = pygame.Rect(x, y, 34, 30)
            pygame.draw.rect(screen, COL_ON if on else COL_OFF, rect, border_radius=5)
            t = f.render(label, True, COL_BG if on else COL_DIM)
            screen.blit(t, (x + 17 - t.get_width() // 2, y + 7))
            x += 40
        screen.blit(f_small.render('W/S 구동  A/D 조향  Q/E 굴절  '
                                   'SPACE 전복  P 자율  ESC 종료', True, COL_DIM),
                    (x + 8, y + 9))
        y += 22
        screen.blit(f_small.render('1 중립(150)  2 수평(180)  0 수납', True, COL_DIM),
                    (x + 8, y + 9))
        y += 30

        # 지령 소스 배지 (MANUAL / AUTO / IDLE) + 자율 허용 상태
        mode = (d.get('drive_mode') or '?').upper() if js is not None else '--'
        mcol = {'AUTO': COL_ON, 'MANUAL': COL_TXT, 'IDLE': COL_DIM}.get(mode, COL_DIM)
        screen.blit(f_big.render(f'MODE {mode}', True, mcol), (16, y))
        acol = COL_ON if auto_enable else COL_WARN
        surf = f_big.render('AUTO ON' if auto_enable else 'AUTO PAUSED', True, acol)
        screen.blit(surf, (LEFT_W - surf.get_width() - 16, y))
        y += 34

        # 텔레메트리
        def row(label, value, unit='', warn=False):
            nonlocal y
            screen.blit(f_small.render(label, True, COL_DIM), (18, y))
            screen.blit(f.render(f'{value} {unit}'.rstrip(), True,
                                 COL_WARN if warn else COL_TXT), (200, y - 2))
            y += 21

        if js is not None and age <= STALE_S:
            target = d.get('shape_target', '')
            drivable = d.get('drivable', 'True') == 'True'
            row('Battery', battery_text(d), '', warn=battery_low(d))
            row('Shape Mode', d.get('shape_mode', '?') +
                (f' -> {target} (moving)' if target else '') +
                ('' if drivable else '   [drive locked]'), '',
                warn=not drivable)
            row('Articulation Angle', d.get('articulation_deg', '?'), 'deg')
            row('XM430 Theta', d.get('motor_deg', '?'), 'deg')
            row('Articulation Angle Err', d.get('error_deg', '?'), 'deg')
            row('XM430 Current', d.get('current_mA', '?'), 'mA')
            row('XM430 Torque (Estimated)', d.get('torque_mNm', '?'), 'mN.m')
            # ---- 서스펜션 ----
            out_travel = d.get('outside_travel_deg', '0')
            try:
                over = float(out_travel) > 0.0
            except ValueError:
                over = False
            row('Susp Current Cap', f"{d.get('goal_current_lsb', '?')} LSB / "
                                    f"{d.get('goal_current_mNm', '?')}", 'mN.m')
            row('Susp Loop', f"{d.get('susp_hz', '?')} / "
                             f"{d.get('susp_hz_target', '?')}", 'Hz',
                warn=_low_hz(d.get('susp_hz'), d.get('susp_hz_target')))
            row('Susp Outside Travel', out_travel, 'deg', warn=over)
            row('Recoveries', d.get('recoveries', '0'), '')
            row('Last Recovery', f"t90 {d.get('last_t90_ms', '--')}ms  "
                                 f"OS {d.get('last_overshoot_pct', '--')}%  "
                                 f"osc {d.get('last_osc', '--')}  "
                                 f"{d.get('last_outcome', '')}", '',
                warn=(d.get('last_outcome') in ('timeout', 'reimpact')))
            row('Steer (Vehicle)', f"{d.get('steer_eff_deg', '?')} / "
                              f"{d.get('steer_limit_deg', '?')}", 'deg')
            row('Steer F/R (Axle)', f"{d.get('axle_front_deg', '?')} / "
                              f"{d.get('axle_rear_deg', '?')}", 'deg')
            row('Curvature', d.get('curvature_per_m', '?'), '1/m')
            row('Wheel Steering Angle F/R', f"{d.get('delta_front_deg', '?')} / "
                                 f"{d.get('delta_rear_deg', '?')}", 'deg')
            out = d.get('l12_output', '?')
            row('L12 Front/Rear', f"{d.get('l12_front_mm', '?')} / "
                           f"{d.get('l12_rear_mm', '?')}", 'mm')
            row('L12 Output', 'ON' if out == 'True' else
                            ('OFF (hold)' if out == 'False' else out), '',
                warn=(out == 'False'))
            armed = d.get('drive_armed', '?')
            row('Drive Command', f"{d.get('drive_cmd', '?')} -> "
                                 f"{d.get('drive_applied', '?')}  [{armed}]", '',
                warn=(armed != 'True'))
            row('Auto Cmd rpm/steer', f"{d.get('auto_rpm', '?')} / "
                                      f"{d.get('auto_steer_deg', '?')}", 'deg')
            row('Odom x/y/yaw', f"{d.get('odom_x', '?')} / {d.get('odom_y', '?')} / "
                                f"{d.get('odom_yaw_deg', '?')}", '')
            row('Odom v', d.get('odom_v', '') or '(no wheel params)', 'm/s')
            # front / back 을 한 줄에 나란히 둔다. 두 값이 크게 어긋나면
            # 한쪽만 물려 있거나 direction_sign 이 잘못된 것이다.
            def pair(field, digits=None):
                f_ = d.get(f'drive_front_{field}', '?')
                b_ = d.get(f'drive_back_{field}', '?')
                return f"{f_} / {b_}"
            row('Drive ON  F/B', pair('enabled'), '',
                warn=('False' in pair('enabled')))
            row('Drive RPM  F/B', pair('rpm'), '')
            row('Drive ERPM F/B', pair('erpm'), '')
            row('Drive I    F/B', pair('current_a'), 'A')
            row('Drive Iq   F/B', pair('iq_a'), 'A')
            row('Drive Tq   F/B', pair('torque_est_nm'), 'N.m')
            row('Drive V    F/B', pair('voltage_v'), 'V')
            row('Battery    F/B', pair('battery_pct'), '%')
            row('Drive degC F/B', pair('temp_c'), '')
            for role in ('front', 'back'):
                err = (d.get(f'drive_{role}_error', '')
                       or d.get(f'drive_{role}_fault', ''))
                if err and err not in ('FAULT_CODE_NONE', 'NONE', 'None'):   # [261017] radxa 는 FAULT_NAMES[0]='NONE' 을 보낸다
                    row(f'Drive Error ({role})', err[:32], '', warn=True)
            row('Temperature', f"{d.get('temperature_C', '?')}C", '')
            row('Loop', f"{d.get('loop_hz', '?')}Hz", '')
            row('Log', f"{node.rows_xm430} rows")

        draw_rc_panel(screen, (f_big, f, f_small), d,
                      js is not None and age <= STALE_S, LEFT_W, RC_W)

        pygame.display.flip()
        clock.tick(FPS)

    pygame.quit()


# ============================================================
def main(argv=None):
    rclpy.init(args=argv)
    try:
        node = Console()
    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        rclpy.shutdown()
        return 1

    spin = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin.start()
    try:
        run_ui(node)
    except KeyboardInterrupt:
        print("\n중단됨")
    finally:
        node.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        print("종료")
    return 0


if __name__ == '__main__':
    sys.exit(main())
