#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
rover_teleop — 제어창의 로버 키보드 조종 (ROS 쪽, Qt 없음). 작성 261010.

제어팀 console.py (~/docker_settings/workspace/DARON_keyboard_ctrl, 10/9 02:09판) 의 지령 규약을 그대로 옮긴 것.
그 파일은 import 하지 않는다 (모듈 맨 위에서 pygame 을 불러옴) — 제어팀 파일은 수정 금지, 규약이 바뀌면 여기를 맞춘다.
[261010 밤] 제어팀 코드를 daron_run/rover/ 로 합침 (RADXA 는 launch/daron_rover.launch.py). 같이 바뀐 것:
  - 굴절각 부호 정정: alpha = 180 + theta (굴절각이 커질수록 굴절부가 내려옴). 진단 articulation_deg 가 이 값이다.
    지령(Q = -1 CW, E = +1 CCW)은 그대로 — Q 가 굴절각 감소, E 가 증가.
  - 배터리: 진단 battery_pct / battery_voltage_v / battery_wh / battery_source / battery_warn_pct,
    drive_<role>_battery_pct. 잔량은 VESC battery_level (front/back 중 낮은 쪽). CSV 는 console.py 와 같이 drive 끝 열.

  보냄 (<ns> = rover_ns, 기본 /rover)        받음
    <ns>/steer_cmd         Float64  차량 유효조향각 [deg], + = 좌     /joint_states        JointState (articulation)
    <ns>/articulation_cmd  Int8     +1 CCW(E) / -1 CW(Q) / 0          <ns>/diagnostics     DiagnosticArray (status[0].values, 배터리 포함)
    <ns>/drive_cmd         Int8     +1 전진(W) / -1 후진(S) / 0
    <ns>/inverted          Bool     전복 (SPACE 토글)
    <ns>/command           String   neutral / flat / stow (1/2/0, 누른 순간 한 번) · stop (창 닫을 때)
    <ns>/heartbeat         Empty    4 Hz

console.py 와 다른 점
  - **조종 ON(수동 버튼)일 때만 보낸다.** OFF 면 아무것도 안 보냄 → console.py 를 같이 켜 둬도 지령이 번갈아 들어가지 않는다
    (특히 inverted 가 엇갈리면 조향 부호가 뒤집힘). OFF 로 바뀌는 순간 0 지령을 한 번 보내고, 그 뒤는 로버 텔레옵 워치독
    (teleop_timeout 0.5 s → 조향 중립·구동 0·조그 정지) 이 맡는다.
  - 송신 주기(10 Hz)는 Qt 타이머가 tick() 을 부르는 것으로 돈다. 창이 굳으면 송신도 멈춰 워치독이 걸린다
    (ROS 타이머로 돌리면 굳은 창의 마지막 키 상태를 계속 보낼 수 있음). 키가 바뀌면 주기를 기다리지 않고 바로 보낸다.
  - ON 으로 켤 때 전복 상태는 로버가 진단으로 알려 준 값(inverted)에서 시작한다 (그동안 console.py 로 바꿨을 수 있으니).
  - CSV 3종(xm430_/l12_/drive_<시각>.csv, 형식은 console.py 와 같음)은 **처음 ON 을 켤 때** 만들어 그 뒤 진단마다 한 줄.
    폴더(session_dir)도 그때 만든다 — waypoint_seq.JsonlLog 와 같은 이유 (daron_viz 녹화가 '폴더 있음'으로 막히지 않게).
  - 복원 이벤트 한 줄은 터미널 print 대신 노드 로거로.
"""
from __future__ import annotations

import datetime
import math
import os
import threading
import time

from diagnostic_msgs.msg import DiagnosticArray
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Empty, Float64, Int8, String

TELEOP_HZ = 10.0          # 로버 teleop_timeout(0.5 s)보다 충분히 빨라야 한다 (console.py 와 같음)
HEARTBEAT_HZ = 4.0
STALE_S = 2.0
# 차량 유효조향각 최대 [deg] = 축 25 deg 와 같은 최소 회전반경 (l12_driver.MAX_STEER_DEG, tan Δv = 2 tan Δa). 로버도 다시 clamp.
AXLE_STEER_LIMIT_DEG = 25.0
DEFAULT_MAX_STEER_DEG = 20.0  # Same operational limit as rover/l12_driver.py
SHAPE_CMDS = ('neutral', 'flat', 'stow')

# ---- CSV 헤더: console.py 와 같게 (analyze·비교 스크립트가 같은 열을 읽게)
SUSP_FIELDS = [
    "susp_state", "goal_current_lsb", "goal_current_mNm", "susp_hz",
    "susp_hz_target", "outside_travel_deg", "recoveries",
    "last_t90_ms", "last_overshoot_pct", "last_osc", "last_residual_deg",
    "last_outcome",
]
XM430_HEADER = [
    "iso_time", "t_mono", "state", "position_rad", "position_deg",
    "motor_deg", "goal_deg", "error_deg", "articulation_deg", "alpha_eff_deg",
    "inverted", "velocity_rad_s", "current_mA", "torque_mNm",
    "temperature_C", "hw_error", "loop_hz", "jog",
] + SUSP_FIELDS
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
DRIVE_ROLES = ("front", "back")
DRIVE_PER_ROLE = (
    "enabled", "sign", "target", "rpm", "erpm", "voltage_v",
    "current_a", "input_a", "iq_a", "torque_est_nm", "temp_c", "fault", "error",
    "battery_pct",
)
BATTERY_FIELDS = ("battery_pct", "battery_voltage_v", "battery_wh")    # console.py 와 같이 맨 뒤
DRIVE_HEADER = (
    ["iso_time", "t_mono", "inverted", "drive_cmd", "drive_applied", "drive_armed"]
    + [f"{r}_{f}" for r in DRIVE_ROLES for f in DRIVE_PER_ROLE]
    + list(BATTERY_FIELDS)
)
BATTERY_WARN_DEFAULT = 20.0     # 로버 battery_warn_pct 기본값 (진단에 없을 때만)
# 서스펜션 상태 표시 기호 (console.py STATE_MARK 와 같음)
STATE_MARK = {"HOLD": "·", "COMPLY": "↓", "RECOVER": "↑", "YIELD": "!",
              "CMD": "→", "SHAPE": "⇄", "LOCK": "■"}


# ============================================================ 순수 함수 (단위 테스트 대상)
def teleop_cmd(keys: dict, max_steer: float):
    """키 눌림 → (steer_deg, jog, drive). console.py _send_teleop 과 같은 규칙.
    A+D / Q+E / W+S 를 같이 누르면 그 축은 중립·정지. q = CW = -1, e = CCW = +1. 전복 보정은 로버가 한다."""
    steer = 0.0
    if keys.get('a') != keys.get('d'):
        steer = max_steer if keys.get('a') else -max_steer
    jog = 0
    if keys.get('q') != keys.get('e'):
        jog = -1 if keys.get('q') else +1
    drive = 0
    if keys.get('w') != keys.get('s'):
        drive = +1 if keys.get('w') else -1
    return steer, jog, drive


def low_hz(actual, target, floor=0.8) -> bool:
    """서스펜션 루프가 설정 주기의 floor 배 아래인가 (기준은 로버가 진단으로 보낸 susp_hz_target, console._low_hz 와 같음)."""
    try:
        a, t = float(actual), float(target)
    except (TypeError, ValueError):
        return False
    return 0.0 < a < t * floor


def battery_state(d: dict):
    """진단 → (pct 또는 None, 전압 문자열, level). level: 'ok' / 'low' / 'unknown'.
    low 기준은 로버가 진단으로 보낸 battery_warn_pct (console.battery_low 와 같음)."""
    volt = d.get('battery_voltage_v', '') or ''
    try:
        pct = float(d.get('battery_pct'))
    except (TypeError, ValueError):
        return None, volt, 'unknown'
    try:
        warn = float(d.get('battery_warn_pct') or BATTERY_WARN_DEFAULT)
    except ValueError:
        warn = BATTERY_WARN_DEFAULT
    return pct, volt, ('low' if pct < warn else 'ok')


def battery_text(d: dict) -> str:
    """'82.3 %  11.80 V  (back)' — console.battery_text 와 같다."""
    pct, volt = d.get('battery_pct', ''), d.get('battery_voltage_v', '')
    parts = ([f'{pct} %'] if pct else []) + ([f'{volt} V'] if volt else [])
    if not parts:
        return '? (VESC 응답 없음)'
    src = d.get('battery_source', '')
    return '  '.join(parts) + (f'  ({src})' if src else '')


def drive_errors(d: dict):
    """[(role, 에러 문자열)] — FAULT_CODE_NONE / None / 빈 값은 빼고."""
    out = []
    for role in DRIVE_ROLES:
        err = d.get(f'drive_{role}_error', '') or d.get(f'drive_{role}_fault', '')
        if err and err not in ('FAULT_CODE_NONE', 'None', 'NONE'):
            out.append((role, err[:32]))
    return out


def soc_temp_text(d: dict):
    """[261015] RADXA SoC 온도 줄 → (값, warn). warn = 커널 스로틀 온도(passive trip) 이상.
    키가 없으면 '?' (로버 radxa.py 가 261015 이전 판), 빈 값이면 'n/a' (sysfs 를 못 읽음)."""
    soc, trip = d.get('soc_temp_C'), d.get('soc_trip_C', '')
    if soc is None:
        return '?', False
    if soc == '':
        return 'n/a', False
    text = f"{soc}C" + (f"  (throttle {trip}C)" if trip else '')
    try:
        return text, bool(trip) and float(soc) >= float(trip)
    except ValueError:
        return text, False


def detail_rows(d: dict, rows_logged: int = 0):
    """console.py 창의 텔레메트리 줄 전부 → [(이름, 값, warn)]. 제어창 [상세] 창이 그대로 그린다."""
    g = lambda k: d.get(k, '?')  # noqa: E731

    def pair(field):
        return f"{g(f'drive_front_{field}')} / {g(f'drive_back_{field}')}"
    target = d.get('shape_target', '')
    drivable = d.get('drivable', 'True') == 'True'
    out_travel = d.get('outside_travel_deg', '0')
    try:
        over = float(out_travel) > 0.0
    except ValueError:
        over = False
    out = d.get('l12_output', '?')
    armed = d.get('drive_armed', '?')
    rows = [
        ('Battery', battery_text(d), battery_state(d)[2] == 'low'),
        ('Shape Mode', g('shape_mode') + (f' -> {target} (moving)' if target else '')
         + ('' if drivable else '   [drive locked]'), not drivable),
        ('Articulation Angle', f"{g('articulation_deg')} deg", False),
        ('XM430 Theta', f"{g('motor_deg')} deg", False),
        ('Articulation Angle Err', f"{g('error_deg')} deg", False),
        ('XM430 Current', f"{g('current_mA')} mA", False),
        ('XM430 Torque (Estimated)', f"{g('torque_mNm')} mN.m", False),
        ('Susp Current Cap', f"{g('goal_current_lsb')} LSB / {g('goal_current_mNm')} mN.m", False),
        ('Susp Loop', f"{g('susp_hz')} / {g('susp_hz_target')} Hz", low_hz(d.get('susp_hz'), d.get('susp_hz_target'))),
        ('Susp Outside Travel', f'{out_travel} deg', over),
        ('Recoveries', d.get('recoveries', '0'), False),
        ('Last Recovery', f"t90 {d.get('last_t90_ms', '--')}ms  OS {d.get('last_overshoot_pct', '--')}%  "
                          f"osc {d.get('last_osc', '--')}  {d.get('last_outcome', '')}",
         d.get('last_outcome') in ('timeout', 'reimpact')),
        ('Steer (Vehicle)', f"{g('steer_eff_deg')} / {g('steer_limit_deg')} deg", False),
        ('Steer F/R (Axle)', f"{g('axle_front_deg')} / {g('axle_rear_deg')} deg", False),
        ('Curvature', f"{g('curvature_per_m')} 1/m", False),
        ('Wheel Steering Angle F/R', f"{g('delta_front_deg')} / {g('delta_rear_deg')} deg", False),
        ('L12 Front/Rear', f"{g('l12_front_mm')} / {g('l12_rear_mm')} mm", False),
        ('L12 Output', 'ON' if out == 'True' else ('OFF (hold)' if out == 'False' else out), out == 'False'),
        ('Drive Command', f"{g('drive_cmd')} -> {g('drive_applied')}  [{armed}]", armed != 'True'),
        ('Drive ON  F/B', pair('enabled'), 'False' in pair('enabled')),
        ('Drive RPM  F/B', pair('rpm'), False),
        ('Drive ERPM F/B', pair('erpm'), False),
        ('Drive I    F/B', pair('current_a') + ' A', False),
        ('Drive Iq   F/B', pair('iq_a') + ' A', False),
        ('Drive Tq   F/B', pair('torque_est_nm') + ' N.m', False),
        ('Drive V    F/B', pair('voltage_v') + ' V', False),
        ('Battery    F/B', pair('battery_pct') + ' %', False),
        ('Drive degC F/B', pair('temp_c'), False),
    ]
    rows += [(f'Drive Error ({role})', err, True) for role, err in drive_errors(d)]
    rows += [('Temperature', f"{g('temperature_C')}C", False),
             ('RADXA SoC Temp', *soc_temp_text(d)),
             ('Loop', f"{g('loop_hz')}Hz", False),
             ('Log', f'{rows_logged} rows', False)]
    return rows


# ============================================================ CSV (console.py 와 같은 형식)
class RoverCsv:
    """처음 open() 때 폴더·파일 3개를 만든다. 쓰기 실패는 예외를 올리지 않고 error 에 남긴다."""

    def __init__(self, log_dir):
        self.log_dir = log_dir
        self.paths = None
        self.rows = 0
        self.error = ''
        self._f = None
        self._t0 = time.monotonic()

    def open(self):
        if self._f is not None:
            return True
        try:
            os.makedirs(self.log_dir, exist_ok=True)
            ts = datetime.datetime.now().strftime('%Y%m%d_%H%M%S')
            self.paths = [os.path.join(self.log_dir, f'{k}_{ts}.csv') for k in ('xm430', 'l12', 'drive')]
            self._f = [open(p, 'w', buffering=1, encoding='utf-8') for p in self.paths]
            for f, h in zip(self._f, (XM430_HEADER, L12_HEADER, DRIVE_HEADER)):
                f.write(','.join(h) + '\n')
            self._t0 = time.monotonic()
            return True
        except OSError as e:
            self.error = f'CSV 열기 실패: {e}'
            self._f = None
            return False

    def write(self, js, d: dict):
        """console._on_diag 와 같은 열. js = (pos, vel, eff) 가 아직 없으면 console 처럼 안 쓴다."""
        if self._f is None or js is None:
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
        l12 = [now_iso, t, d.get('inverted', '')] + [d.get(k, '') for k in L12_HEADER[3:]]
        drive = [now_iso, t] + [d.get(k, '') for k in DRIVE_HEADER[2:6]]
        for role in DRIVE_ROLES:
            for field in DRIVE_PER_ROLE:
                drive.append(str(d.get(f'drive_{role}_{field}', '')).replace(',', ';'))
        drive += [str(d.get(k, '')).replace(',', ';') for k in BATTERY_FIELDS]
        try:
            for f, row in zip(self._f, (xm, l12, drive)):
                f.write(','.join(str(v) for v in row) + '\n')
            self.rows += 1
        except OSError as e:
            self.error = f'CSV 기록 실패: {e}'

    def close(self):
        for f in self._f or []:
            try:
                f.close()
            except OSError:
                pass
        self._f = None


# ============================================================ ROS 연결
class RoverLink:
    """제어창 노드(GroundNode)에 퍼블리셔·구독을 붙인다. 구독 콜백은 spin 스레드, 나머지는 Qt 스레드에서 부른다."""

    def __init__(self, node, ns='/rover', max_steer=DEFAULT_MAX_STEER_DEG, log_dir='.', joint_name='articulation'):
        self.node = node
        self.ns = ns.rstrip('/') or '/rover'
        self.max_steer = float(max_steer)
        self.joint_name = joint_name
        ns = self.ns
        self.pub_steer = node.create_publisher(Float64, f'{ns}/steer_cmd', 10)
        self.pub_jog = node.create_publisher(Int8, f'{ns}/articulation_cmd', 10)
        self.pub_drive = node.create_publisher(Int8, f'{ns}/drive_cmd', 10)
        self.pub_inv = node.create_publisher(Bool, f'{ns}/inverted', 10)
        self.pub_cmd = node.create_publisher(String, f'{ns}/command', 10)
        self.pub_hb = node.create_publisher(Empty, f'{ns}/heartbeat', 10)
        node.create_subscription(JointState, '/joint_states', self._on_js, 10)
        node.create_subscription(DiagnosticArray, f'{ns}/diagnostics', self._on_diag, 10)

        self._lock = threading.Lock()
        self._js = None
        self._diag = {}
        self._diag_msg = ''
        self._last_rx = 0.0
        self._recoveries = None       # 처음 받은 값은 기준만 (이미 지난 복원은 안 찍음)
        self.enabled = False
        self.keys = {k: False for k in 'wasdqe'}
        self.inverted = False
        self.last_sent = None         # (steer, jog, drive, inverted)
        self._last_send_t = 0.0
        self._last_hb_t = 0.0
        self.csv = RoverCsv(log_dir)
        node.get_logger().info(f'[261010] 로버 조종 {ns}/* (수동 버튼 ON 일 때만 송신), 조향 ±{self.max_steer:.3f}°, '
                               f'CSV {log_dir} (처음 ON 때 생성)')

    # ---------- 수신 (spin 스레드)
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
        if not msg.status:
            return
        st = msg.status[0]
        d = {kv.key: kv.value for kv in st.values}
        with self._lock:
            self._diag, self._diag_msg = d, st.message
            js = self._js
            self.csv.write(js, d)
        try:
            n = int(d.get('recoveries') or 0)
        except ValueError:
            return
        if self._recoveries is not None and n > self._recoveries:
            way = '-CW ' if (d.get('last_dir') or '').startswith('-') else '+CCW'
            self.node.get_logger().info(
                f"복원 #{n:<3d} {way}  변위 {d.get('last_err0_deg', '?')} deg  t90 {d.get('last_t90_ms') or '--'} ms  "
                f"OS {d.get('last_overshoot_pct', '?')} %  진동 {d.get('last_osc', '?')}  "
                f"잔류 {d.get('last_residual_deg', '?')} deg  {d.get('last_outcome', '')}")
        self._recoveries = n

    def snapshot(self):
        """→ (js, diag dict, diag message, 마지막 /joint_states 이후 초). console.snapshot 과 같다."""
        with self._lock:
            return (self._js, dict(self._diag), self._diag_msg,
                    time.monotonic() - self._last_rx if self._last_rx else 1e9)

    def link_ok(self):
        js, _d, _m, age = self.snapshot()
        return js is not None and age <= STALE_S

    # ---------- 송신 (Qt 스레드)
    def set_enabled(self, on: bool):
        on = bool(on)
        if on == self.enabled:
            return
        if on:
            _js, d, _m, _age = self.snapshot()
            if self.link_ok() and d.get('inverted') in ('True', 'False'):
                self.inverted = d['inverted'] == 'True'          # 로버가 알고 있는 전복 상태에서 시작
            with self._lock:
                self.csv.open()
            self.release_all()
            self.enabled = True
            self._send(force=True)
            self.node.get_logger().info(f'로버 조종 ON (전복 {"ON" if self.inverted else "OFF"})')
        else:
            self.release_all()
            self._send(force=True)                               # 0 지령 한 번 → 이후 침묵, 로버 워치독이 맡음
            self.enabled = False
            self.last_sent = None
            self.node.get_logger().info('로버 조종 OFF — 송신 중지 (로버 텔레옵 워치독 0.5 s)')

    def release_all(self):
        for k in self.keys:
            self.keys[k] = False

    def release_keys(self):
        """눌린 키를 한꺼번에 떼고 0 지령을 한 번에 보낸다 (하나씩 떼면 W+A 일 때 'drive 0, steer 43' 이 먼저 나감)."""
        if any(self.keys.values()):
            self.release_all()
            self._send(force=True)

    def set_key(self, name, down):
        if name in self.keys and self.keys[name] != bool(down):
            self.keys[name] = bool(down)
            self._send(force=True)                               # 바뀐 순간 바로 (주기 기다리지 않음)

    def toggle_inverted(self):
        if not self.enabled:
            return
        self.inverted = not self.inverted
        self.node.get_logger().info(f'전복 {"ON (조향 반전)" if self.inverted else "OFF (정상)"}')
        self._send(force=True)

    def shape(self, cmd):
        if not self.enabled or cmd not in SHAPE_CMDS:
            return
        self.pub_cmd.publish(String(data=cmd))
        self.node.get_logger().info(f'형상 모드 -> {cmd}')

    def tick(self):
        """Qt 타이머에서 부른다 (100 ms). ON 일 때만 10 Hz 지령 + 4 Hz heartbeat."""
        if not self.enabled:
            return
        now = time.monotonic()
        if now - self._last_send_t >= 1.0 / TELEOP_HZ - 0.02:
            self._send()
        if now - self._last_hb_t >= 1.0 / HEARTBEAT_HZ - 0.02:
            self._last_hb_t = now
            self.pub_hb.publish(Empty())

    def _send(self, force=False):
        if not self.enabled:
            return
        steer, jog, drive = teleop_cmd(self.keys, self.max_steer)
        self.pub_steer.publish(Float64(data=float(steer)))
        self.pub_jog.publish(Int8(data=jog))
        self.pub_drive.publish(Int8(data=drive))
        self.pub_inv.publish(Bool(data=self.inverted))
        self.last_sent = (steer, jog, drive, self.inverted)
        self._last_send_t = time.monotonic()

    def shutdown(self):
        """창 닫을 때. ON 이었으면 console.shutdown 처럼 0 지령 + stop. OFF 였으면 아무것도 안 보냄 (console.py 사용 중일 수 있음)."""
        if self.enabled:
            try:
                self.pub_steer.publish(Float64(data=0.0))
                self.pub_jog.publish(Int8(data=0))
                self.pub_drive.publish(Int8(data=0))
                self.pub_cmd.publish(String(data='stop'))
                time.sleep(0.3)
            except Exception:  # noqa: BLE001 — 종료 중 context 내려감
                pass
            self.enabled = False
        with self._lock:
            self.csv.close()
