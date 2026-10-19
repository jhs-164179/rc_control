#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
susp_controller.py — XM430 을 '서스펜션'으로 쓰기 위한 상태기계.

왜 dxl_driver.XM430.update() 를 쓰지 않는가
---------------------------------------------
드라이버의 update() 는 굴절 관절용이다. 외란 복귀를 '부드럽게'(recover <
moving) 만드는 것이 목적이고, 재기입 간격도 0.4 s 로 느긋하다.
서스펜션은 요구가 정반대다: 눌릴 때는 약하게(유지 토크 낮음), 외력이
사라진 순간에는 세게(복원 빠름). 그래서 상태기계만 새로 짠다.
드라이버의 나머지(포트/configure/sync read/게인·프로파일 기록)는 그대로 쓴다.

핵심 아이디어
-------------
Goal Current(102) 는 RAM 이라 매 주기 바꿀 수 있다. Current-based Position
Control 에서 이 값은 '부호 없는 대칭 토크 상한'이다. 따라서

    낮은 유지 토크  ==  HOLD 상태의 작은 Goal Current
    빠른 복원       ==  RECOVER 상태의 큰 Goal Current + Goal Position 재기입

두 요구가 서로 다른 상태에 배치되므로 동시에 만족할 수 있다.
Goal Position 재기입이 필요한 이유: 재기입해야 내부 궤적 생성기가 다시
돌아 Profile Velocity/Acceleration 의 사다리꼴을 탄다. 재기입하지 않으면
순수 PID 외란 응답이 되어 프로파일이 개입하지 못한다.

상태
----
  HOLD     밴드 안. 유지 전류(낮음). 서스펜션이 '물렁한' 구간.
  COMPLY   외력이 축을 밀어내는 중. 더 낮은 전류로 양보한다.
  RECOVER  외력이 사라짐. 방향 보정된 큰 전류 + 궤적 재기동으로 복귀.
  YIELD    복원이 timeout 까지 안 끝남(계속 눌려 있음). 모터 보호를 위해
           유지 전류로 내려가 대기하다가 주기적으로 복원을 재시도한다.
  CMD      실험용 지령 이동. moving 전류로 확실히 목표에 보낸다.

방향 비대칭
-----------
Goal Current 는 양방향에 같은 크기의 상한을 건다. 그런데 실제 순 토크는

    tau_net = KT*I_cap - sign(v)*tau_friction - tau_gravity(theta)

이라 중력 부하가 있으면 방향에 따라 2*tau_gravity 만큼 차이난다.
그래서 복원 진입 시점에 방향을 보고 I_cap 을 보정한다. 보정값은
asymmetry.py 가 측정으로 구한다 (추정하지 않는다).
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import List, Optional

from motor_control.xm430.metrics import recovery_metrics
from motor_control.xm430.config import DIR_NEG, DIR_POS, SuspensionConfig, clamp

HOLD, COMPLY, RECOVER, YIELD, CMD = "HOLD", "COMPLY", "RECOVER", "YIELD", "CMD"

DEG = 180.0 / math.pi


@dataclass
class RecoveryEvent:
    """복원 1회의 요약. 방향 비대칭은 이걸 방향별로 모아서 본다."""
    t_start: float = 0.0
    direction: int = 0
    err0_deg: float = 0.0            # 복원 시작 시점의 오차 (초기 변위)
    goal_current: int = 0
    t90_s: float = float("nan")      # 오차가 10 % 로 줄 때까지
    t_settle_s: float = float("nan") # settled 밴드에 들어와 머물 때까지
    peak_vel_dps: float = 0.0
    mean_vel_dps: float = 0.0
    overshoot_pct: float = 0.0
    zero_crossings: int = 0
    peak_current_mA: float = 0.0
    residual_deg: float = 0.0        # 복원이 도달한 평형점의 잔류 오차(정적 처짐)
    outcome: str = ""                # settled | residual | timeout | reimpact
    metrics: dict = field(default_factory=dict)

    def header(self):
        return ("dir  err0[deg]  I[LSB]  t90[ms]  settle[ms]  "
                "vpk[deg/s]  OS[%]  osc  Ipk[mA]  결과")

    def line(self):
        f = lambda v, s: ("  --  " if math.isnan(v) else f"{v * s:6.0f}")
        d = "+CCW" if self.direction > 0 else "-CW "
        return (f"{d} {self.err0_deg:+8.2f}  {self.goal_current:5d}  "
                f"{f(self.t90_s, 1000)}  {f(self.t_settle_s, 1000)}    "
                f"{self.peak_vel_dps:8.1f}  {self.overshoot_pct:5.1f}  "
                f"{self.zero_crossings:3d}  {self.residual_deg:+7.2f}  "
                f"{self.peak_current_mA:6.0f}  {self.outcome}")


class RateLoop:
    """고정 주기 루프. 실제 달성 주기를 같이 알려준다 (FTDI latency 진단용)."""

    def __init__(self, hz: float):
        self.period = 1.0 / float(hz)
        self._next = time.monotonic()
        self._last = self._next
        self.hz_actual = float(hz)
        self.overruns = 0

    def wait(self) -> float:
        self._next += self.period
        now = time.monotonic()
        if now < self._next:
            time.sleep(self._next - now)
            now = time.monotonic()
        else:
            self.overruns += 1
            if now - self._next > 0.5:      # 크게 밀렸으면 따라잡기를 포기한다
                self._next = now
        dt = now - self._last
        self._last = now
        if dt > 0:
            self.hz_actual += 0.1 * (1.0 / dt - self.hz_actual)
        return now


class SuspensionController:
    def __init__(self, dxl, motor, cfg: SuspensionConfig):
        self.dxl = dxl
        self.motor = motor
        self.cfg = cfg
        self.state = HOLD
        self._goal = cfg.home_rad
        self._gc = None                  # Goal Current 캐시. 바뀔 때만 쓴다.
        self._prof_vel_lsb = None
        self._vel_limit = 330
        self._pulse_per_rad = 1.0 / dxl.RAD_PER_PULSE

        self._t_release: Optional[float] = None
        self._t_settle: Optional[float] = None
        self._t_state = 0.0
        self._t_reissue = 0.0
        self._t_last = time.monotonic()
        self._vfb = 0.0                  # 속도 피드백 적분기 [LSB]
        # 마지막 복원이 도달한 평형점의 잔류 오차. 하중을 받는 서스펜션은
        # 낮은 유지 토크 때문에 정적 처짐이 남는 것이 정상이고, 이걸 기억하지
        # 않으면 HOLD 가 그 처짐을 매번 새 외란으로 착각해 무한히 재복원한다.
        self._resid = 0.0
        self._gc_base = 0                # 현재 복원의 기준 전류

        self._tags: List[str] = []
        # 가동 범위. 0 이면 제한 없음. '이 밖을 지령하지 않는다'는 뜻이지
        # '이 밖으로 나가면 멈춘다'는 뜻이 아니다.
        self._tl_center = 0.0
        self._tl_limit = 0.0
        self._tl_warned = 0.0
        self._ev: Optional[RecoveryEvent] = None
        self._ev_err0 = 0.0
        self._ev_samples: List[tuple] = []
        self.events: List[RecoveryEvent] = []
        self.event_cb = None             # 완료된 RecoveryEvent 를 받는 콜백

    # ------------------------------------------------------------------
    # 초기화
    # ------------------------------------------------------------------
    def attach(self, info: dict = None):
        """configure() 직후에 부른다. 속도 한계를 캐시해 이후 왕복을 줄인다."""
        if info and "velocity_limit" in info:
            self._vel_limit = int(info["velocity_limit"])
        else:
            self._vel_limit = int(self.motor.r4(self.dxl.ADDR_VELOCITY_LIMIT))
        fb = self.motor.read()
        self._write_goal_position(self._apply_goal(fb.position_rad))
        self._set_gc(self.cfg.current_holding)
        self.state = HOLD
        self._t_state = time.monotonic()
        return fb

    # ------------------------------------------------------------------
    # 가동 범위
    # ------------------------------------------------------------------
    def set_travel_limit(self, center_rad: float, limit_rad: float):
        """
        home 기준 ±limit 을 '지령 가능한 범위'로 정한다.

        이 밖으로 나가는 것을 막지는 않는다. 서스펜션은 외력에 눌리는 것이
        일이므로, 큰 충격에 경계를 넘는 일은 정상이다. 그때 실행을 멈추면
        오히려 토크가 풀려 차체가 주저앉는다. 대신 목표를 경계로 잘라
        '경계로 되돌리는' 지령이 걸린다.
        """
        self._tl_center = float(center_rad)
        self._tl_limit = max(0.0, float(limit_rad))

    def clamp_goal(self, rad: float) -> float:
        """목표를 가동 범위 안으로 자른다. 잘랐으면 가끔 알려준다."""
        if self._tl_limit <= 0.0:
            return rad
        lo = self._tl_center - self._tl_limit
        hi = self._tl_center + self._tl_limit
        out = clamp(rad, lo, hi)
        if abs(out - rad) > 1e-6:
            now = time.monotonic()
            if now - self._tl_warned > 2.0:
                self._tl_warned = now
                print(f"[ctrl] 목표 {rad * DEG:+.1f}° 는 가동 범위 "
                      f"({lo * DEG:+.1f}° ~ {hi * DEG:+.1f}°) 밖입니다. "
                      f"{out * DEG:+.1f}° 로 자릅니다.")
        return out

    def _apply_goal(self, rad: float) -> float:
        self._goal = self.clamp_goal(rad)
        return self._goal

    @property
    def travel_limit_rad(self) -> float:
        return self._tl_limit

    def outside_travel(self, position_rad: float) -> float:
        """가동 범위를 얼마나 벗어나 있는가 [rad]. 안이면 0."""
        if self._tl_limit <= 0.0:
            return 0.0
        d = abs(position_rad - self._tl_center) - self._tl_limit
        return max(0.0, d)

    # ------------------------------------------------------------------
    # 이벤트 태그 — 로거가 CSV event 열에 받아 적는다
    # ------------------------------------------------------------------
    def _tag(self, name: str):
        self._tags.append(name)

    def take_tag(self) -> str:
        if not self._tags:
            return ""
        out = "|".join(self._tags)
        self._tags.clear()
        return out

    # ------------------------------------------------------------------
    # 저수준 (드라이버의 공개 API 만 쓴다)
    # ------------------------------------------------------------------
    def _set_gc(self, raw: int):
        raw = int(clamp(round(raw), 1, self.cfg.current_limit))
        if raw == self._gc:
            return
        self.motor.w2(self.dxl.ADDR_GOAL_CURRENT, raw, "Goal Current(102)")
        self._gc = raw

    def _write_goal_position(self, rad: float):
        # 마지막 관문. 어떤 경로로 들어오든 범위 밖은 레지스터에 쓰지 않는다.
        rad = self.clamp_goal(rad)
        self.motor.w4(self.dxl.ADDR_GOAL_POSITION,
                      int(round(rad * self._pulse_per_rad)), "Goal Position(116)")

    def _set_profile_velocity(self, dps: float):
        lsb = int(clamp(round(dps / self.dxl.DPS_PER_VEL_LSB), 1, self._vel_limit))
        if lsb == self._prof_vel_lsb:
            return
        self.motor.w4(self.dxl.ADDR_PROFILE_VELOCITY, lsb, "Profile Velocity(112)")
        self._prof_vel_lsb = lsb

    def set_profile(self, velocity_dps=None, accel_dpss=None):
        """런타임 재튜닝. 둘 다 RAM 이라 Torque ON 상태에서 바로 먹는다."""
        if velocity_dps is not None:
            self.cfg.profile_velocity_dps = float(velocity_dps)
            self._set_profile_velocity(velocity_dps)
        if accel_dpss is not None:
            self.cfg.profile_accel_dpss = float(accel_dpss)
            lsb = max(1, int(round(accel_dpss / self.dxl.DPSS_PER_ACC_LSB)))
            self.motor.w4(self.dxl.ADDR_PROFILE_ACCELERATION, lsb,
                          "Profile Acceleration(108)")

    def set_gains(self, p=None, i=None, d=None):
        if p is not None:
            self.cfg.p_gain = int(p)
        if i is not None:
            self.cfg.i_gain = int(i)
        if d is not None:
            self.cfg.d_gain = int(d)
        self.motor.set_gains(p, i, d)

    def invalidate_current_cache(self):
        """전류 레벨을 바꾼 뒤 부른다. 다음 주기에 강제로 다시 쓰게 한다."""
        self._gc = None

    # ------------------------------------------------------------------
    # 목표 조작
    # ------------------------------------------------------------------
    @property
    def goal_rad(self) -> float:
        return self._goal

    @property
    def goal_deg(self) -> float:
        return self._goal * DEG

    def set_home(self, rad: float):
        """서스펜션이 되돌아갈 기준 위치를 옮긴다 (상태는 건드리지 않는다)."""
        self._write_goal_position(self._apply_goal(rad))

    def command(self, rad: float, now: float = None):
        """실험용 지령 이동. moving 전류로 확실히 보낸다."""
        now = now or time.monotonic()
        self._apply_goal(rad)
        self._set_gc(self.cfg.current_moving)
        self._set_profile_velocity(self.cfg.profile_velocity_dps)
        self._write_goal_position(self._goal)
        self._enter(CMD, now)

    def arm_recovery(self, goal_rad: float = None, now: float = None) -> float:
        """
        실험용. 초기 변위를 만들어 둔 뒤 이걸 부르면 즉시 RECOVER 로 들어간다.
        Goal Current 를 먼저 복원값으로 바꾸고 그 다음 Goal Position 을 쓰므로,
        지령 이동용 moving 전류가 복원 구간에 새어 들어오지 않는다.
        """
        now = now or time.monotonic()
        if goal_rad is not None:
            self._apply_goal(goal_rad)
        fb = self.motor.read()
        err = self._goal - fb.position_rad
        self._enter_recover(now, err, DIR_POS if err >= 0 else DIR_NEG)
        return err

    def force_goal_current(self, raw: int):
        """상태기계를 거치지 않고 Goal Current 를 직접 건다 (sag/breakaway 전용)."""
        self._set_gc(raw)

    def hold_here(self, position_rad: float, now: float = None):
        now = now or time.monotonic()
        self._write_goal_position(self._apply_goal(position_rad))
        self._set_gc(self.cfg.current_holding)
        self._enter(HOLD, now)

    # ------------------------------------------------------------------
    # 축 넘겨주기 — 상태기계 밖에서 레지스터를 직접 쓰는 쪽(radxa.py 형상 전환)
    # ------------------------------------------------------------------
    def suspend(self, now: float = None):
        """
        상태기계가 축을 놓는다. 이후 update() 를 부르지 않는 동안 다른 쪽이
        Goal Current/Position, 게인, 궤적을 마음대로 써도 된다.
        진행 중인 복원 이벤트는 닫고, 레지스터 캐시는 믿을 수 없게 되므로 비운다.
        """
        if self._ev is not None:
            self._finish_event(now or time.monotonic(), "aborted")
        self._gc = None
        self._prof_vel_lsb = None

    def resume(self, position_rad: float, now: float = None):
        """
        suspend() 뒤에 축을 돌려받는다. 게인·궤적을 cfg 값으로 다시 쓰고
        position_rad 를 기준 자세로 HOLD 에 들어간다.
        """
        cfg = self.cfg
        self._gc = None
        self._prof_vel_lsb = None
        self._resid = 0.0
        self.motor.set_gains(cfg.p_gain, cfg.i_gain, cfg.d_gain)
        self.set_profile(cfg.profile_velocity_dps, cfg.profile_accel_dpss)
        self.hold_here(position_rad, now)

    # ------------------------------------------------------------------
    # 상태 전환
    # ------------------------------------------------------------------
    def _enter(self, state: str, now: float):
        if state != self.state:
            self.state = state
            self._t_state = now
            self._t_release = None
            self._t_settle = None

    def _enter_comply(self, now: float):
        if self.state == RECOVER and self._ev is not None:
            self._finish_event(now, "reimpact")
        self._enter(COMPLY, now)
        self._tag("COMPLY")
        self._set_gc(self.cfg.current_comply)

    def _enter_recover(self, now: float, err: float, direction: int):
        cfg = self.cfg
        self._enter(RECOVER, now)
        gc = cfg.recover_current(direction)
        self._gc_base = gc
        self._vfb = 0.0
        if cfg.dir_profile_active():
            self._set_profile_velocity(cfg.recover_profile_velocity(direction))
        self._set_gc(gc)
        # 궤적 생성기 재기동. 이게 없으면 프로파일이 개입하지 못한다.
        self._write_goal_position(self._goal)
        self._t_reissue = now
        self._tag("RECOVER_START")
        self._ev = RecoveryEvent(t_start=now, direction=direction,
                                 err0_deg=err * DEG, goal_current=gc)
        self._ev_err0 = abs(err)
        self._ev_samples = [(now, err, 0.0, 0.0)]

    def _finish_event(self, now: float, outcome: str):
        """지표 계산은 analyze.recovery_metrics 한 곳에만 둔다."""
        ev, s = self._ev, self._ev_samples
        self._ev, self._ev_samples = None, []
        self._tag(f"RECOVER_END:{outcome}")
        if ev is None or len(s) < 3:
            return
        m = recovery_metrics([x[0] for x in s],
                             [x[1] * DEG for x in s],
                             [x[2] * DEG for x in s],
                             [x[3] * 1000.0 for x in s],
                             self.cfg.settled_band_deg)
        ev.outcome = outcome
        ev.t90_s = m["t90_s"]
        ev.t_settle_s = m["t_settle_s"]
        ev.peak_vel_dps = m["peak_vel_dps"]
        ev.mean_vel_dps = m["mean_vel_dps"]
        ev.overshoot_pct = m["overshoot_pct"]
        ev.zero_crossings = m["osc"]
        ev.peak_current_mA = m["peak_mA"]
        ev.residual_deg = m["residual_deg"]
        ev.metrics = m
        self._resid = abs(m["residual_deg"]) / DEG if m["residual_deg"] == m["residual_deg"] else 0.0
        self.events.append(ev)
        if self.event_cb:
            self.event_cb(ev)

    # ------------------------------------------------------------------
    # 주기 갱신
    # ------------------------------------------------------------------
    def _converged(self, fb, a: float, rv: float, settled: float) -> bool:
        """
        '더 이상 움직이지 않는다' 판정.

        밴드 안에 들어오는 것만을 완료로 보면, 정적 처짐이 밴드보다 큰 순간
        복원은 영원히 끝나지 않는다(그리고 모터는 계속 밀어붙인다). 속도가
        죽었으면 그 지점이 이 전류로 도달할 수 있는 평형점이다. 거기서
        멈추는 것이 맞다.

        Moving(122) 플래그는 판정에 쓰지 않는다. 궤적 생성기가 목표 근처에서
        미세 리밋사이클에 빠지면 플래그가 계속 서 있을 수 있다. 대신 진입
        직후 구간을 제외하고(_recover_converged) settle_dwell_s 동안 속도가
        죽어 있을 것을 요구해, 오버슈트 반환점의 순간 0 속도를 걸러낸다.
        """
        return a < settled or abs(fb.velocity_rad_s) < rv

    def _recover_converged(self, fb, a, rv, settled, now) -> bool:
        # 진입 직후에는 Moving 플래그가 아직 서지 않았을 수 있다.
        if now - self._t_state < 0.05:
            return False
        return self._converged(fb, a, rv, settled)

    def update(self, fb, now: float) -> str:
        cfg = self.cfg
        dt = max(now - self._t_last, 1e-4)
        self._t_last = now

        err = self._goal - fb.position_rad
        a = abs(err)
        direction = DIR_POS if err >= 0 else DIR_NEG
        vel = fb.velocity_rad_s
        approach = vel * direction          # >0 이면 목표로 다가오는 중
        rv = cfg.release_vel_rad_s
        settled = cfg.settled_band_rad
        comply_band = cfg.comply_band_rad

        # 복원 이벤트 표본은 상태와 무관하게 먼저 쌓는다
        if self._ev is not None:
            self._ev_samples.append((now, err, vel, fb.current_a))
            if math.isnan(self._ev.t90_s) and a <= 0.1 * max(self._ev_err0, 1e-9):
                self._ev.t90_s = now - self._ev.t_start

        st = self.state

        if st == CMD:
            if self._recover_converged(fb, a, rv, settled, now):
                if self._t_settle is None:
                    self._t_settle = now
                elif now - self._t_settle >= cfg.settle_dwell_s:
                    self._enter(HOLD, now)
                    self._set_gc(cfg.current_holding)
            else:
                self._t_settle = None
            return self.state

        if st == HOLD:
            # 이미 알고 있는 정적 처짐은 새 외란이 아니다. 그 위로 반 밴드
            # 이상 더 벌어졌을 때만 다시 복원을 건다.
            trigger = max(settled, self._resid + 0.5 * comply_band)
            if a >= comply_band and approach < -rv:
                self._enter_comply(now)
            elif a >= trigger:
                self._enter_recover(now, err, direction)
            else:
                # 처짐이 스스로 줄었으면(외력이 사라졌으면) 기준도 같이 내린다.
                # 이게 없으면 한 번 크게 눌린 뒤 영구히 둔감해진다.
                if a < settled:
                    self._resid = 0.0
                elif a < self._resid:
                    self._resid = a
                self._set_gc(cfg.current_holding)
            return self.state

        if st == COMPLY:
            self._set_gc(cfg.current_comply)
            pushed_away = approach < -rv
            if pushed_away:
                self._t_release = None
            else:
                # 외력이 멈췄거나 이미 되돌아오기 시작했다
                if self._t_release is None:
                    self._t_release = now
                elif now - self._t_release >= cfg.release_dwell_s:
                    if a < settled:
                        self._enter(HOLD, now)
                        self._set_gc(cfg.current_holding)
                    else:
                        self._enter_recover(now, err, direction)
            return self.state

        if st == RECOVER:
            # 복원 중 새 충격
            if a >= comply_band and a > self._ev_err0 * cfg.reimpact_ratio \
                    and approach < -rv:
                self._enter_comply(now)
                return self.state
            if self._recover_converged(fb, a, rv, settled, now):
                if self._t_settle is None:
                    self._t_settle = now
                elif now - self._t_settle >= cfg.settle_dwell_s:
                    self._finish_event(now, "settled" if a < settled else "residual")
                    self._enter(HOLD, now)
                    self._set_gc(cfg.current_holding)
                    return self.state
            else:
                self._t_settle = None

            if now - self._t_state >= cfg.recover_timeout_s:
                # 계속 눌려 있다. 모터를 계속 밀어붙이면 발열만 쌓인다.
                self._finish_event(now, "timeout")
                self._enter(YIELD, now)
                self._tag("YIELD")
                self._set_gc(cfg.current_holding)
                return self.state

            # Goal Position 재기입은 복원 진입에서 딱 한 번만 한다.
            # 평형점에 도달한 뒤 또 쓰면 궤적 생성기가 다시 돌아 Moving 이
            # 서고, 수렴 판정이 영원히 리셋된다(= 복원이 끝나지 않는다).
            if cfg.vel_fb_enable and a > settled * 1.5:
                self._velocity_feedback(vel, dt)
            return self.state

        if st == YIELD:
            self._set_gc(cfg.current_holding)
            if a < max(settled, self._resid + 0.5 * comply_band):
                self._enter(HOLD, now)
            elif now - self._t_state >= cfg.yield_retry_s:
                if approach >= -rv:
                    self._enter_recover(now, err, direction)
                else:
                    self._enter_comply(now)
            return self.state

        return self.state

    def _velocity_feedback(self, vel_rad_s: float, dt: float):
        """
        L3. 복원 '속도'를 목표에 맞추도록 전류 상한을 적분 조정한다.

        Present Velocity 는 이미 같은 sync read 안에 있어 추가 통신이 없다.
        비용은 Goal Current 재기입(57600 baud 왕복 ≈ 4 ms)이므로,
        데드밴드 밖에서만 실제로 쓴다. 서보 내부 1 kHz 루프를 바깥에서
        다시 감싸는 구조라 게인은 낮게, 폭은 좁게 두는 것이 안전하다.
        """
        cfg = self.cfg
        e = cfg.vel_fb_target_dps - abs(vel_rad_s) * DEG
        self._vfb = clamp(self._vfb + cfg.vel_fb_gain * e * dt,
                          -cfg.vel_fb_span_lsb, cfg.vel_fb_span_lsb)
        target = self._gc_base + self._vfb
        if self._gc is None or abs(target - self._gc) >= cfg.vel_fb_deadband_lsb:
            self._set_gc(target)

    # ------------------------------------------------------------------
    # 편의
    # ------------------------------------------------------------------
    @property
    def goal_current(self) -> int:
        return self._gc or 0

    def goto_blocking(self, rad: float, timeout: float = 4.0,
                      recorder=None, t0: float = None, hz: float = None):
        """지령 이동을 마칠 때까지 돈다. 실험의 초기조건을 만들 때 쓴다."""
        loop = RateLoop(hz or self.cfg.control_hz)
        t0 = t0 if t0 is not None else time.monotonic()
        start = time.monotonic()
        self.command(rad, start)
        while True:
            now = loop.wait()
            fb = self.motor.read()
            self.update(fb, now)
            if recorder:
                recorder.write(now - t0, self.state, 0, self.goal_deg,
                               fb.position_rad * DEG, fb.velocity_rad_s * DEG,
                               fb.current_a, self.goal_current, self.take_tag())
            if self.state == HOLD:
                return True
            if now - start > timeout:
                return False
