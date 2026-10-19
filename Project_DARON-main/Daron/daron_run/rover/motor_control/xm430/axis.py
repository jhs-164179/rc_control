"""XM430 articulation and suspension worker; independent of ROS."""
from __future__ import annotations
import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from .link import open_motor
from .config import SuspensionConfig, lsb_to_mnm, unwrap_zero_deg
from .suspension import DEG, HOLD, RateLoop, SuspensionController, YIELD
from .log import Recorder
from motor_control.l12.driver import articulation_deg, motor_angle_from_articulation

STOW, NEUTRAL, FLAT = "STOW", "NEUTRAL", "FLAT"


SHAPE_MOVE, SHAPE_HOLD = "SHAPE", "LOCK"


CURRENT_LIMIT_MAX = 1193          # XM430-W210 Current Limit(38) 최대값


@dataclass
class ShapeConfig:
    """
    형상 전환 설정. 구동 값의 기본은 xm430_angle_hold/hold_angle.py 를
    --hold-current 600 으로 돌렸을 때와 같다 (이동 전류 = 고정 전류,
    20 deg/s, 40 deg/s^2, P800 I0 D0, 도착 밴드 1 deg).
    """
    neutral_alpha_deg: float = 150.0
    flat_alpha_deg: float = 180.0
    stow_detect_alpha_deg: float = 250.0
    # 수납 상태로 켜지지 않았을 때 0 번의 목표 굴절각. NaN 이면 0 번을 거부한다.
    stow_alpha_deg: float = float('nan')
    hold_current: int = 600
    move_current: int = 600
    velocity_dps: float = 20.0
    accel_dpss: float = 40.0
    p_gain: int = 800
    i_gain: int = 0
    d_gain: int = 0
    band_deg: float = 1.0
    slip_deg: float = 3.0
    timeout_margin_s: float = 3.0


def move_time_s(dist_deg: float, v: float, a: float) -> float:
    """사다리꼴(또는 삼각) 프로파일의 이동 시간. hold_angle.py 와 같다."""
    d = abs(dist_deg)
    if d > v * v / a:
        return d / v + v / a
    return 2.0 * math.sqrt(d / a)


class SuspensionAxis(threading.Thread):
    """
    XM430 한 축을 susp_hz (기본 100 Hz) 로 돌린다.

    시리얼 포트는 이 스레드만 만진다. ROS 콜백/타이머는 request() 로 의도만
    큐에 넣고 snapshot() 으로 상태를 읽는다. 이렇게 하지 않으면 콜백 지터와
    sysfs 쓰기가 서스펜션 주기에 그대로 섞인다 (복원 검출이 한 박자 늦는다).

    조그와 서스펜션의 관계
    ----------------------
    조그를 누르는 동안은 ctrl.command() 로 CMD 상태(moving 전류)에서 한계까지
    이동하고, 키를 떼면 그 자리에서 hold_here() 를 불러 그 위치가 새 기준
    자세(home)가 된다. 목표를 자르는 가동 범위 창(set_travel_limit)은 기계
    원점에 고정되어 있으므로 home 이 움직여도 한계 밖으로는 못 나간다.
    """

    SLOW_POLL_S = 2.0            # Hardware Error / Temperature 폴링 간격

    def __init__(self, cfg: SuspensionConfig, *, mock: bool = False,
                 logger=None, log_dir: str = "",
                 art_zero_deg: float = 0.0, jog_limit_deg: float = 45.0,
                 shape: ShapeConfig = None):
        super().__init__(name="susp-axis", daemon=True)
        self.cfg = cfg
        self.log = logger
        self.art_zero_deg = float(art_zero_deg)
        self.art_zero_profile_deg = self.art_zero_deg   # 회전 맞추기 전 값 (로그용)
        self.jog_limit_deg = float(jog_limit_deg)

        # 형상 모드. _shape 가 None 이면 서스펜션 상태기계가 축을 잡고 있고,
        # SHAPE_MOVE / SHAPE_HOLD 면 형상 제어가 레지스터를 직접 쓴다.
        self.shape = shape or ShapeConfig()
        self.mode = ""                 # '' = 형상 지령 전(시작 자세). STOW/NEUTRAL/FLAT
        self.stow_rad = None           # 수납 위치 [rad, 엔코더 절대]. 모르면 None
        self._shape = None
        self._shape_target = ""
        self._shape_goal = 0.0         # [rad, 엔코더 절대]. 가동 범위 창을 쓰지 않는다
        self._shape_gc = 0
        self._t_shape = 0.0
        self._t_shape_limit = 0.0
        self._shape_slipped = False

        self.dxl, self.motor, self._info = open_motor(
            cfg, mock=mock, port=cfg.port, verbose=False)
        self.ctrl = SuspensionController(self.dxl, self.motor, cfg)
        self.ctrl.event_cb = self._on_event

        self._q = deque()
        self._qlock = threading.Lock()
        self._lock = threading.Lock()
        self._stop_evt = threading.Event()   # [261017] _stop 이면 threading.Thread._stop 을 가려 join() 이 TypeError

        self._jog_cmd = 0
        self._jog_applied = 0
        self._torque_on = True
        self._pos_rad = 0.0            # 마지막으로 읽은 위치 (요청 처리에 쓴다)
        self._recoveries = 0
        self._last_event = {}
        self._fails = 0
        self._t_slow = 0.0
        self._hw_error = 0
        self._temperature = 0
        self._rec = None
        self._log_dir = log_dir
        self._snap = {
            'state': HOLD, 'position_rad': 0.0, 'position_deg': 0.0,
            'velocity_rad_s': 0.0, 'current_a': 0.0, 'goal_rad': 0.0,
            'goal_deg': 0.0, 'goal_current': 0, 'hz': 0.0, 'hw_error': 0,
            'temperature': 0, 'fault': None, 'outside_deg': 0.0,
            'recoveries': 0, 'last_event': {}, 'torque_on': True,
            'moving': False, 'ready': False,
            'mode': '', 'shape_target': '', 'drivable': True,
        }

    # --------------------------------------------------------
    # 준비 / 종료
    # --------------------------------------------------------
    def prepare(self):
        """
        스레드를 띄우기 전에 부른다. Goal Current/Position 을 안전한 값으로
        맞춰 두고 가동 범위 창을 건다.
        """
        fb = self.ctrl.attach(self._info)
        self._pos_rad = fb.position_rad
        # 원점을 지금 읽힌 엔코더 회전에 맞춘다. 판단·목표·가동 범위가 전부
        # art_zero_deg 를 기준으로 하므로 창을 걸기 전에 한다.
        self.art_zero_profile_deg = self.art_zero_deg
        self.set_art_zero(self.art_zero_deg)
        if self.art_zero_deg != self.art_zero_profile_deg:
            self._note(f"굴절 원점 {self.art_zero_profile_deg:+.2f} -> "
                       f"{self.art_zero_deg:+.2f} deg (엔코더 "
                       f"{fb.position_rad * DEG:+.1f} deg 와 같은 회전으로 옮김)")
        self.apply_travel_window()
        if self._startup_stowed(fb.position_rad):
            # 수납 상태로 켜졌다. 형상을 바꾸지 않고 그 자리를 강한 전류로 붙든다.
            # 이 위치가 0 번(수납)의 목표가 된다.
            self.stow_rad = fb.position_rad
            self.mode = STOW
            self._shape_hold(fb.position_rad, time.monotonic())
        else:
            # 기본 자세는 굴절 원점(180)이 아니라 중립(150)이다. 1 번과 같은
            # 형상 전환으로 옮기고, 도착하면 서스펜션이 그 자세를 home 으로 잡는다.
            self._shape_move(NEUTRAL,
                             self.alpha_to_rad(self.shape.neutral_alpha_deg),
                             time.monotonic())
            if not math.isnan(self.shape.stow_alpha_deg):
                self.stow_rad = self.alpha_to_rad(self.shape.stow_alpha_deg)
        if self._log_dir:
            self._rec = Recorder(
                self._log_dir, "rover",
                {"mode": "rover", "config": self.cfg.to_dict(),
                 "articulation_zero_deg": self.art_zero_deg,
                 "articulation_zero_profile_deg": self.art_zero_profile_deg,
                 "jog_limit_deg": self.jog_limit_deg,
                 "start_mode": STOW if self.mode == STOW else NEUTRAL})
        self._update_snapshot(time.monotonic(), fb, 0.0)
        return fb

    def apply_travel_window(self):
        """
        목표를 자르는 창: 굴절 원점 ± jog_limit_deg.

        중심은 기계 원점에 고정한다. home 은 조그를 따라 움직이지만 이 창은
        움직이지 않으므로, 조그를 반복해도 축이 기계 한계 밖으로 끌려가지 않는다.
        서스펜션이 외력에 밀려 이 밖으로 나가는 것은 막지 않는다 (막을 수도 없고,
        막겠다고 실행을 멈추면 토크가 풀려 차체가 주저앉는다). 범위 밖 목표가
        경계로 잘릴 뿐이다.
        """
        lim = self.jog_limit_deg
        self.cfg.travel_limit_deg = lim
        self.ctrl.set_travel_limit(math.radians(self.art_zero_deg),
                                   math.radians(lim))
        return lim

    @property
    def log_path(self):
        return self._rec.path if self._rec else None

    def stop(self, *, torque_off: bool = False, timeout: float = 2.0):
        """
        스레드를 세우고 포트를 정리한다. torque_off=False (기본) 면 토크를
        끄지 않고 유지 전류를 건 채 포트만 닫는다. 서보가 마지막 지령을
        붙들고 있으므로 차체가 주저앉지 않는다 (run_suspension.py 와 같다).
        """
        self._stop_evt.set()
        if self.is_alive():
            self.join(timeout)
        try:
            if self._shape is None:
                self.ctrl.force_goal_current(self.cfg.current_holding)
            else:
                # 수납/형상 전환 중이면 큰 전류를 그대로 남긴다 (hold_angle.py 와 같다).
                self._write_gc(self.shape.hold_current)
        except Exception:
            pass
        if self._rec:
            try:
                self._rec.update_meta(
                    finished=time.strftime("%Y-%m-%d %H:%M:%S"),
                    recoveries=self._recoveries)
                self._rec.close()
            except Exception:
                pass
        try:
            if torque_off:
                self.motor.shutdown()
            else:
                self.motor.port.closePort()
        except Exception:
            pass

    # --------------------------------------------------------
    # 요청 큐 — ROS 스레드에서 부른다
    # --------------------------------------------------------
    def request(self, kind: str, **kw):
        with self._qlock:
            self._q.append((kind, kw))

    def snapshot(self) -> dict:
        with self._lock:
            return dict(self._snap)

    # --------------------------------------------------------
    # 복원 이벤트
    # --------------------------------------------------------
    def _on_event(self, ev):
        t90 = ev.t90_s
        self._recoveries += 1
        self._last_event = {
            'n': self._recoveries,
            'dir': ev.direction,
            'err0_deg': ev.err0_deg,
            't90_ms': float('nan') if t90 != t90 else t90 * 1000.0,
            'overshoot_pct': ev.overshoot_pct,
            'osc': ev.zero_crossings,
            'residual_deg': ev.residual_deg,
            'outcome': ev.outcome,
        }
        if self.log:
            t = self._last_event['t90_ms']
            self.log.info(
                f"복원 #{self._recoveries} "
                f"{'+CCW' if ev.direction > 0 else '-CW'} "
                f"변위 {ev.err0_deg:+.2f}deg  "
                f"t90 {'--' if t != t else f'{t:.0f}ms'}  "
                f"OS {ev.overshoot_pct:.1f}%  진동 {ev.zero_crossings}  "
                f"잔류 {ev.residual_deg:+.2f}deg  {ev.outcome}")

    # --------------------------------------------------------
    # 루프
    # --------------------------------------------------------
    def run(self):
        loop = RateLoop(self.cfg.control_hz)
        t0 = time.monotonic()
        while not self._stop_evt.is_set():
            now = loop.wait()
            fault = None
            try:
                self._drain(now)
            except Exception as e:
                fault = f"요청 처리 실패: {type(e).__name__}: {e}"
                self._warn(fault)

            try:
                fb = self.motor.read()
                self._fails = 0
            except Exception as e:
                self._fails += 1
                self._fault_only(f"{type(e).__name__}: {e}")
                if self._fails in (1, 20) or self._fails % 200 == 0:
                    self._warn(f"XM430 읽기 실패 x{self._fails}: {e}")
                continue
            self._pos_rad = fb.position_rad

            if self._torque_on:
                try:
                    if self._shape is not None:
                        self._shape_update(fb, now)
                    else:
                        self._apply_jog(fb, now)
                        self.ctrl.update(fb, now)
                except Exception as e:
                    fault = f"{type(e).__name__}: {e}"
                    self._warn(f"서스펜션 갱신 실패: {e}")

            if self._rec:
                try:
                    state, goal_deg, gc = self._view()
                    self._rec.write(now - t0, state, self._jog_applied,
                                    goal_deg, fb.position_rad * DEG,
                                    fb.velocity_rad_s * DEG, fb.current_a,
                                    gc, self.ctrl.take_tag())
                except Exception:
                    pass
            else:
                self.ctrl.take_tag()

            if now - self._t_slow >= self.SLOW_POLL_S:
                self._t_slow = now
                try:
                    self._hw_error = self.motor.hardware_error()
                    self._temperature = self.motor.temperature()
                    if self._hw_error:
                        self._warn("Hardware Error Status(70) = "
                                   f"0x{self._hw_error:02X}")
                except Exception:
                    pass

            self._update_snapshot(now, fb, loop.hz_actual, fault)

    def _warn(self, msg: str):
        if self.log:
            self.log.error(msg, throttle_duration_sec=2.0)

    def _note(self, msg: str):
        if self.log:
            self.log.info(msg)

    def _fault_only(self, msg: str):
        with self._lock:
            self._snap['fault'] = msg

    def _view(self):
        """(상태, 목표 [deg], Goal Current [LSB]) — 지금 축을 잡고 있는 쪽 기준."""
        if self._shape is None:
            return self.ctrl.state, self.ctrl.goal_deg, self.ctrl.goal_current
        return self._shape, self._shape_goal * DEG, self._shape_gc

    def _update_snapshot(self, now, fb, hz, fault=None):
        state, goal_deg, gc = self._view()
        with self._lock:
            self._snap.update(
                state=state,
                position_rad=fb.position_rad,
                position_deg=fb.position_rad * DEG,
                velocity_rad_s=fb.velocity_rad_s,
                current_a=fb.current_a,
                moving=bool(fb.moving),
                goal_rad=math.radians(goal_deg),
                goal_deg=goal_deg,
                goal_current=gc,
                mode=self.mode,
                shape_target=self._shape_target if self._shape == SHAPE_MOVE else '',
                drivable=self._shape is None,
                hz=hz,
                hw_error=self._hw_error,
                temperature=self._temperature,
                fault=fault,
                outside_deg=self.ctrl.outside_travel(fb.position_rad) * DEG,
                recoveries=self._recoveries,
                last_event=dict(self._last_event),
                torque_on=self._torque_on,
                ready=True)

    # --------------------------------------------------------
    # 조그
    # --------------------------------------------------------
    def _apply_jog(self, fb, now):
        """
        방향이 바뀔 때만 쓴다. 매 주기 Goal Position 을 쓰면 궤적이 계속
        재생성되어 끊긴 동작이 된다.
        """
        if self._jog_cmd == self._jog_applied:
            return
        if self._jog_cmd:
            target = self.art_zero_deg + self._jog_cmd * self.jog_limit_deg
            self.ctrl.command(math.radians(target), now)
        else:
            # 키를 뗀 자리가 새 기준 자세가 된다.
            self.ctrl.hold_here(fb.position_rad, now)
        self._jog_applied = self._jog_cmd

    # --------------------------------------------------------
    # 형상 모드 (수납 / 중립 / 수평) — hold_angle.py 방식
    # --------------------------------------------------------
    def alpha_deg(self, position_rad: float) -> float:
        """엔코더 절대각 -> 굴절각 alpha = 180 + theta [deg]."""
        return articulation_deg(position_rad * DEG - self.art_zero_deg)

    def alpha_to_rad(self, alpha_deg: float) -> float:
        """굴절각 -> 엔코더 절대각 [rad]."""
        return math.radians(self.art_zero_deg
                            + motor_angle_from_articulation(alpha_deg))

    def set_art_zero(self, zero_deg: float):
        """
        굴절 원점을 정한다. 지금 위치와 같은 회전으로 옮겨서(unwrap_zero_deg)
        cfg.home_deg 에도 같이 쓴다 (노드와 같은 cfg 객체다).
        """
        self.art_zero_deg = unwrap_zero_deg(float(zero_deg), self._pos_rad * DEG)
        self.cfg.home_deg = self.art_zero_deg

    def _startup_stowed(self, position_rad: float) -> bool:
        # prepare() 가 원점을 이 위치와 같은 회전으로 옮겨 두었으므로
        # theta 는 [-180, 180), 굴절각은 [0, 360) 이다.
        return self.alpha_deg(position_rad) >= self.shape.stow_detect_alpha_deg

    def _write_gc(self, lsb: int):
        self.motor.w2(self.dxl.ADDR_GOAL_CURRENT, int(lsb), "Goal Current(102)")
        self._shape_gc = int(lsb)

    def _write_goal(self, rad: float):
        # 가동 범위 창을 거치지 않는다. 수납 위치는 원래 그 창 밖이다.
        self.motor.w4(self.dxl.ADDR_GOAL_POSITION,
                      int(round(rad * self.dxl.PULSE_PER_RAD)), "Goal Position(116)")

    def _shape_take(self, now):
        """서스펜션 상태기계에서 축을 넘겨받고 게인·궤적을 형상 전환용으로 바꾼다."""
        if self._shape is not None:
            return
        s = self.shape
        self.ctrl.suspend(now)
        self.motor.set_gains(s.p_gain, s.i_gain, s.d_gain)
        self.motor.set_profile(s.velocity_dps, s.accel_dpss)
        self._jog_cmd = self._jog_applied = 0

    def _shape_hold(self, goal_rad: float, now: float):
        """그 자리를 큰 전류로 붙든다 (hold_angle.py 의 HOLD)."""
        self._shape_take(now)
        self._shape = SHAPE_HOLD
        self._shape_goal = goal_rad
        self._shape_slipped = False
        self._write_goal(goal_rad)
        self._write_gc(self.shape.hold_current)

    def _shape_move(self, target: str, goal_rad: float, now: float):
        """target 모드의 자세로 천천히 옮긴다 (hold_angle.py 의 MOVE)."""
        s = self.shape
        dist = (goal_rad - self._pos_rad) * DEG
        if abs(dist) > 180.0:
            self._warn(f"{target}: 이동 거리 {dist:+.1f} deg 가 180 deg 를 넘습니다. "
                       f"엔코더가 한 바퀴 어긋났을 수 있어 거부합니다.")
            return
        self._shape_take(now)
        self._shape = SHAPE_MOVE
        self._shape_target = target
        self._shape_goal = goal_rad
        self._write_gc(s.move_current)
        self._write_goal(goal_rad)
        t = move_time_s(dist, s.velocity_dps, s.accel_dpss)
        self._t_shape = now
        self._t_shape_limit = t * 1.5 + s.timeout_margin_s
        self._note(f"형상 {self.mode or 'START'} -> {target}: 굴절각 "
                   f"{self.alpha_deg(self._pos_rad):.1f} -> "
                   f"{self.alpha_deg(goal_rad):.1f} deg (거리 {dist:+.1f} deg, "
                   f"예상 {t:.1f} s, {s.move_current} LSB)")

    def _shape_update(self, fb, now):
        s = self.shape
        err = (self._shape_goal - fb.position_rad) * DEG
        if self._shape == SHAPE_MOVE:
            arrived = (not fb.moving) and abs(err) < s.band_deg
            timed_out = now - self._t_shape > self._t_shape_limit
            if not (arrived or timed_out):
                return
            target = self._shape_target
            alpha = self.alpha_deg(fb.position_rad)
            if arrived:
                self._note(f"형상 {target} 도착: 굴절각 {alpha:.1f} deg "
                           f"(오차 {err:+.2f} deg, {now - self._t_shape:.1f} s)")
            else:
                self._warn(f"형상 {target}: {self._t_shape_limit:.1f} s 안에 도착하지 "
                           f"못했습니다. 굴절각 {alpha:.1f} deg (오차 {err:+.2f} deg), "
                           f"전류 {fb.current_a * 1000:+.0f} mA.")
            self.mode = target
            self._shape_target = ""
            if target == STOW:
                self._shape_hold(self._shape_goal, now)
            else:
                # 펴진 자세에서는 서스펜션이 다시 축을 잡는다. 그 자세가 home.
                self._shape = None
                self.ctrl.resume(self._shape_goal, now)
        elif self._shape == SHAPE_HOLD:
            if abs(err) > s.slip_deg and not self._shape_slipped:
                self._shape_slipped = True
                self._warn(f"고정 중 밀림: 오차 {err:+.2f} deg > {s.slip_deg} deg. "
                           f"shape_hold_current({self._shape_gc} LSB)가 모자랄 수 있습니다.")
            elif abs(err) < 0.5 * s.slip_deg and not fb.moving:
                self._shape_slipped = False

    def _request_shape(self, mode: str, now: float):
        if mode == STOW:
            if self.stow_rad is None:
                self._warn("수납 위치를 모릅니다. 수납 상태로 켜지 않았다면 "
                           "-p stow_alpha_deg:=<굴절각> 으로 정해 주십시오.")
                return
            goal = self.stow_rad
        elif mode == NEUTRAL:
            goal = self.alpha_to_rad(self.shape.neutral_alpha_deg)
        elif mode == FLAT:
            goal = self.alpha_to_rad(self.shape.flat_alpha_deg)
        else:
            self._warn(f"모르는 형상 모드: {mode!r}")
            return
        self._shape_move(mode, goal, now)

    # --------------------------------------------------------
    # 요청 처리 (모터 접근은 전부 여기서)
    # --------------------------------------------------------
    def _drain(self, now):
        while True:
            with self._qlock:
                if not self._q:
                    return
                kind, kw = self._q.popleft()
            self._handle(kind, kw, now)

    def _handle(self, kind, kw, now):
        ctrl, cfg = self.ctrl, self.cfg

        if kind == 'shape':
            self._request_shape(kw['mode'], now)
            return

        # 형상 제어가 축을 잡고 있는 동안(전환 중 / 수납)은 서스펜션 쪽 지령을
        # 막는다. 게인·궤적은 cfg 에만 적어 두면 축을 돌려줄 때 반영된다.
        if self._shape is not None:
            if kind == 'jog':
                self._jog_cmd = self._jog_applied = 0
            elif kind in ('goal_abs', 'goal_rel', 'home', 'home_here'):
                self._warn(f"형상 {self._shape} 중에는 '{kind}' 를 무시합니다. "
                           f"1(중립) 또는 2(수평)로 먼저 펴십시오.")
            elif kind == 'stop_here':
                if self._shape == SHAPE_MOVE:
                    self._shape_target = ""
                    self._shape_hold(self._pos_rad, now)
                    # 감속하며 조금 지나쳤다 돌아오는 것은 밀림이 아니다.
                    self._shape_slipped = True
                    self._note("형상 전환을 멈추고 그 자리에 고정합니다.")
            elif kind == 'torque':
                on = bool(kw['value'])
                self.motor.torque(on)
                self._torque_on = on
                if on:
                    self._shape_hold(self._pos_rad, now)
            elif kind == 'gains':
                for k, v in kw.items():
                    setattr(cfg, f"{k}_gain", int(v))
            elif kind == 'profile':
                if kw.get('velocity_dps') is not None:
                    cfg.profile_velocity_dps = float(kw['velocity_dps'])
                if kw.get('accel_dpss') is not None:
                    cfg.profile_accel_dpss = float(kw['accel_dpss'])
            elif kind == 'currents':
                for name, value in kw.items():
                    setattr(cfg, f"current_{name}", int(value))
            elif kind == 'cfg':
                for name, value in kw.items():
                    setattr(cfg, name, value)
            elif kind == 'travel':
                if 'art_zero_deg' in kw:
                    self.set_art_zero(kw['art_zero_deg'])
                if 'jog_limit_deg' in kw:
                    self.jog_limit_deg = float(kw['jog_limit_deg'])
                self.apply_travel_window()
            return

        if kind == 'jog':
            self._jog_cmd = int(kw['value'])

        elif kind == 'goal_abs':
            self._jog_cmd = self._jog_applied = 0
            ctrl.command(float(kw['rad']), now)

        elif kind == 'goal_rel':
            self._jog_cmd = self._jog_applied = 0
            ctrl.command(self._pos_rad + float(kw['rad']), now)

        elif kind == 'home':
            self._jog_cmd = self._jog_applied = 0
            ctrl.command(math.radians(self.art_zero_deg), now)

        elif kind == 'home_here':
            self._jog_cmd = self._jog_applied = 0
            ctrl.hold_here(self._pos_rad, now)

        elif kind == 'stop_here':
            self._jog_cmd = self._jog_applied = 0
            ctrl.hold_here(self._pos_rad, now)

        elif kind == 'torque':
            on = bool(kw['value'])
            self.motor.torque(on)
            self._torque_on = on
            self._jog_cmd = self._jog_applied = 0
            if on:
                ctrl.invalidate_current_cache()
                ctrl.hold_here(self._pos_rad, now)

        elif kind == 'gains':
            ctrl.set_gains(**kw)

        elif kind == 'profile':
            ctrl.set_profile(**kw)

        elif kind == 'currents':
            for name, value in kw.items():
                setattr(cfg, f"current_{name}", int(value))
            ctrl.invalidate_current_cache()

        elif kind == 'cfg':
            for name, value in kw.items():
                setattr(cfg, name, value)
            ctrl.invalidate_current_cache()

        elif kind == 'travel':
            if 'art_zero_deg' in kw:
                self.set_art_zero(kw['art_zero_deg'])
            if 'jog_limit_deg' in kw:
                self.jog_limit_deg = float(kw['jog_limit_deg'])
            self.apply_travel_window()

