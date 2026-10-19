#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
follower_core.py — 경로추종 한 주기 (ROS 없음). CLAUDE.md §3.1 순서를 그대로 따른다.

path_follower.py(ROS 노드)는 토픽 입출력만 맡고 이 객체를 감싼다. sim_offline.py 와
tests/ 도 같은 객체를 쓴다.

    f = Follower(FollowerParams())
    f.submit_path(points)          # 경로 콜백에서. 검증·전처리만 하고 pending 에 둔다
    f.submit_path([])              # 빈 경로 = 목표 도착, 정지
    out = f.step(dt, pose, pose_age, v_meas, enabled, wheelbase_m)   # 고정 주기 타이머에서

한 주기
-------
 1) pending 경로를 active 로 교체 (원자적). kappa_prev, I 는 그대로 (bumpless)
 2) enable / 경로 유무 / pose staleness
 3) CTE -> e_y, e_th, kappa_ref, remaining
 4) 5) StanleyCurvature: 제어법칙 -> 포화 -> 조건부 적분 -> rate 제한
 6) kappa -> delta (understeer_gain 배로 더 꺾는다)
 7) 속도 명령 + a_max rate 제한
 8) 안전 로직: 정지 조건이면 v = 0 (rate 제한 없이 즉시), 조향은 마지막 값 유지

상태
----
  IDLE            enable off, 또는 경로가 아직 없음
  TRACKING        정상
  STOP_GOAL       빈 경로 수신.             -> 점이 있는 새 경로
  STOP_END        remaining < end_tol.     -> 새 경로
  STOP_STALE      pose 가 pose_timeout 보다 오래됨 (적분 안 함). -> 새 pose
  HOLD_DEVIATION  |e_y| > e_abort, |e_th| > e_th_abort_deg, 또는 pose 점프(jump_m / jump_deg) [261016].  -> 새 경로

I 리셋: 빈 경로, enable off, 정지(IDLE/STOP_GOAL/STOP_END/HOLD_DEVIATION) 후 재출발.
        STOP_STALE 에서 돌아올 때와 경로 교체 때는 리셋하지 않는다.
kappa_prev 는 enable off 때만 0 으로 (로버가 auto_timeout 뒤 조향을 중립으로 돌리므로).
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple

from path_cte import PreparedPath, cte, prepare_path, wrap
from stanley_core import Params, StanleyCurvature, kappa_to_delta, speed_command, understeer_gain

IDLE = "IDLE"
TRACKING = "TRACKING"
STOP_GOAL = "STOP_GOAL"
STOP_END = "STOP_END"
STOP_STALE = "STOP_STALE"
HOLD_DEVIATION = "HOLD_DEVIATION"

STATE_CODE = {IDLE: 0, TRACKING: 1, STOP_GOAL: 2, STOP_END: 3, STOP_STALE: 4, HOLD_DEVIATION: 5}
_RESTART_FROM = (IDLE, STOP_GOAL, STOP_END, HOLD_DEVIATION)

# /follower/debug 순서 (CLAUDE.md §4.8). 바꾸면 문서 표도 같이 고친다.
DEBUG_FIELDS = ("t", "state", "path_id", "e_y", "e_th", "kappa_ref", "kappa_fb",
                "kappa_raw", "kappa_cmd", "delta_cmd", "saturated", "I", "v_cmd",
                "v_meas", "remaining", "pose_age")


@dataclass
class FollowerParams(Params):
    """CLAUDE.md §5. Params(k_e, k_th_ratio, k_i, I_max, e_I_max, kappa_max, kappa_rate,
    gain_scale) 를 그대로 물려받아 StanleyCurvature 에 이 객체를 넘긴다."""
    delta_max_deg: float = 20.0
    preview_m: float = 0.05
    v_max_mps: float = 0.15          # = v_nom (기존 파라미터 이름 유지)
    v_min: float = 0.05
    e1: float = 0.05
    e2: float = 0.15
    curv_slow_ratio: float = 0.8
    curv_slow_factor: float = 0.6
    a_max: float = 0.3
    end_tol: float = 0.02
    e_abort: float = 0.30
    pose_timeout_s: float = 0.3
    new_path_search_m: float = 0.5
    # [261016] pose 점프·방향 이탈 → HOLD_DEVIATION (새 경로 대기). 10/14 로그: 26 cm·58° 점프 뒤 풀락으로 0.84 s 주행.
    #   추종 로그 TRACKING 38k 행(50 Hz): 주기 사이 정상 p99.9 3.2 cm·3.6°, 점프 9~26 cm 5 회 → 8 cm / 15° 는 그 5 회만 잡는다.
    jump_m: float = 0.08
    jump_deg: float = 15.0
    #   Stanley 는 |e_th| < 90° 전제(±180° 근처에선 조향 부호가 떨린다). 로그 TRACKING 최대 56.7°.
    e_th_abort_deg: float = 75.0


@dataclass
class FollowerOutput:
    state: str = IDLE
    v_cmd: float = 0.0
    kappa_cmd: float = 0.0
    delta_cmd: float = 0.0           # [rad] 차량 유효조향각
    path_id: int = 0
    e_y: float = math.nan
    e_th: float = math.nan
    kappa_ref: float = math.nan
    kappa_fb: float = math.nan
    kappa_raw: float = math.nan
    saturated: bool = False
    I: float = 0.0
    v_meas: float = math.nan
    remaining: float = math.nan
    pose_age: float = math.nan
    message: str = ""

    def debug_array(self, t: float):
        d = dict(self.__dict__, t=t, state=STATE_CODE[self.state],
                 saturated=1.0 if self.saturated else 0.0)
        return [float(d[k]) for k in DEBUG_FIELDS]


_EMPTY = object()      # pending 표식: 빈 경로


class Follower:

    def __init__(self, p: FollowerParams):
        self.p = p
        self.ctrl = StanleyCurvature(p)
        self.active: Optional[PreparedPath] = None
        self._pending = None
        self.path_id = 0
        self.state = IDLE
        self.v_prev = 0.0
        self._enabled = False
        self._last_pose = None                   # [261016] 점프 감시용 — 신선한 pose 마다 갱신 (상태와 무관)

    # ------------------------------------------------------------
    def submit_path(self, points: Sequence[Tuple[float, float]]) -> str:
        """경로 콜백에서 부른다. 좌표는 map [m], frame 검증은 호출자가 한다.
        반환: 로그용 한 줄. 비유한 좌표면 ValueError (기존 경로 유지)."""
        for px, py in points:
            if not (math.isfinite(px) and math.isfinite(py)):
                raise ValueError("비유한 좌표가 있습니다")
        path = prepare_path(points, self.p.new_path_search_m) if len(points) else None
        self._pending = path if path is not None else _EMPTY
        if path is None:
            return f"빈 경로 (입력 {len(points)} 점) -> 정지"
        return f"입력 {len(points)} 점 -> {len(path)} 점, 길이 {path.length:.2f} m"

    # ------------------------------------------------------------
    def _swap_pending(self):
        pend, self._pending = self._pending, None
        if pend is None:
            return
        if pend is _EMPTY:
            self.active = None
            self.state = STOP_GOAL
            self.ctrl.reset_integrator()
            return
        self.active = pend                       # kappa_prev, I 는 유지 (bumpless)
        self.path_id += 1
        if self.state in _RESTART_FROM and self._enabled:
            self.ctrl.reset_integrator()
            self.state = TRACKING

    def _stop(self, out: FollowerOutput, state: str, wheelbase_m, message="") -> FollowerOutput:
        """v = 0 (즉시), 조향은 마지막 명령 유지."""
        self.state = state
        self.v_prev = 0.0
        out.state, out.v_cmd, out.message = state, 0.0, message
        out.kappa_cmd = self.ctrl.kappa_prev
        out.delta_cmd = self._delta(self.ctrl.kappa_prev, wheelbase_m)
        out.I = self.ctrl.I
        return out

    def _delta(self, kappa, wheelbase_m):
        """kappa = 실제로 돌 곡률. [261015] 언더스티어만큼 조향 지령 곡률을 키운다."""
        return kappa_to_delta(kappa * understeer_gain(self.p), self.p.kappa_max,
                              self.p.delta_max_deg, wheelbase_m)

    # ------------------------------------------------------------
    def step(self, dt: float, pose: Optional[Tuple[float, float, float]], pose_age: float,
             v_meas: Optional[float], enabled: bool,
             wheelbase_m: Optional[float] = None) -> FollowerOutput:
        """
        pose     : (x, y, psi) map [m, rad], 축 가운데 기준. None = 없음
        pose_age : now - pose.stamp [s]
        v_meas   : odom 속도 [m/s]. None 이면 직전 v_cmd 를 쓴다 (적분의 ds 용)
        """
        p = self.p
        dt = max(0.0, min(dt, 0.1))              # 타이머가 밀려도 rate 제한이 한 번에 풀리지 않게

        # 1) 경로 교체 — enable 처리보다 먼저 해서 꺼져 있는 동안 온 경로도 최신으로 둔다
        if not enabled and self._enabled:        # enable off: 전부 리셋
            self.ctrl.reset()
            self.v_prev = 0.0
        if enabled and not self._enabled and self.active is not None:
            self.active.need_init = True         # 꺼져 있는 동안 로버가 옮겨졌을 수 있다
            self.state = IDLE                    # 아래에서 재출발 처리
        self._enabled = enabled
        self._swap_pending()

        out = FollowerOutput(path_id=self.path_id, pose_age=pose_age,
                             v_meas=math.nan if v_meas is None else v_meas)

        # [261016] pose 점프: 직전 신선한 pose 와 비교. 정지·대기 중에도 갱신해 두어 재출발 때 오검출하지 않게 한다.
        jump = None
        if pose is not None and math.isfinite(pose_age) and pose_age <= p.pose_timeout_s:
            if self._last_pose is not None:
                dxy = math.hypot(pose[0] - self._last_pose[0], pose[1] - self._last_pose[1])
                dpsi = abs(wrap(pose[2] - self._last_pose[2]))
                if dxy > p.jump_m or dpsi > math.radians(p.jump_deg):
                    jump = (dxy, dpsi)
            self._last_pose = (pose[0], pose[1], pose[2])

        # 2) enable / 경로
        if not enabled:
            return self._stop(out, IDLE, wheelbase_m, "비활성")
        if self.active is None:
            st = STOP_GOAL if self.state == STOP_GOAL else IDLE
            return self._stop(out, st, wheelbase_m, "목표 도착" if st == STOP_GOAL else "경로 대기")
        if self.state == IDLE:
            self.ctrl.reset_integrator()
            self.state = TRACKING

        # 2) pose staleness
        stale = pose is None or not math.isfinite(pose_age) or pose_age > p.pose_timeout_s
        if stale:
            msg = "pose 없음" if pose is None else f"pose 오래됨 {pose_age:.2f} s"
            # STOP_END / HOLD_DEVIATION 은 새 경로로만 풀리므로 상태를 덮어쓰지 않는다
            st = STOP_STALE if self.state in (TRACKING, STOP_STALE) else self.state
            return self._stop(out, st, wheelbase_m, msg)
        if self.state == STOP_STALE:
            self.state = TRACKING                # I 는 유지

        # 3) CTE (정지 상태에서도 진단용으로 계산)
        e_y, e_th, kappa_ref, remaining = cte(self.active, pose[0], pose[1], pose[2], p.preview_m)
        out.e_y, out.e_th, out.kappa_ref, out.remaining = e_y, e_th, kappa_ref, remaining

        if self.state in (STOP_END, HOLD_DEVIATION):
            return self._stop(out, self.state, wheelbase_m, "새 경로 대기")

        # 8) 안전 판정 (정지 조건이면 이번 주기 제어 상태를 건드리지 않는다)
        if jump is not None:
            return self._stop(out, HOLD_DEVIATION, wheelbase_m,
                              f"pose 점프 {jump[0] * 100:.0f} cm / {math.degrees(jump[1]):.0f}° -> 새 경로 대기")
        if abs(e_y) > p.e_abort:
            return self._stop(out, HOLD_DEVIATION, wheelbase_m,
                              f"이탈 |e_y|={abs(e_y):.2f} m > {p.e_abort:.2f} m")
        if abs(e_th) > math.radians(p.e_th_abort_deg):
            return self._stop(out, HOLD_DEVIATION, wheelbase_m,
                              f"방향 이탈 |e_th|={math.degrees(abs(e_th)):.0f}° > {p.e_th_abort_deg:.0f}°")
        if remaining < p.end_tol:
            return self._stop(out, STOP_END, wheelbase_m, "경로 끝")

        # 4) 5) 조향
        v_for_ds = self.v_prev if v_meas is None else abs(v_meas)
        kappa_cmd, info = self.ctrl.step(e_y, e_th, kappa_ref, v_for_ds, dt)
        out.kappa_cmd = kappa_cmd
        out.kappa_fb, out.kappa_raw = info["kappa_fb"], info["kappa_raw"]
        out.saturated, out.I = info["saturated"], info["I"]
        # 6)
        out.delta_cmd = self._delta(kappa_cmd, wheelbase_m)

        # 7) 속도
        v_tgt = speed_command(e_y, kappa_ref, out.saturated, v_nom=p.v_max_mps, v_min=p.v_min,
                              e1=p.e1, e2=p.e2, kappa_max=p.kappa_max / understeer_gain(p),
                              curv_slow_ratio=p.curv_slow_ratio,
                              curv_slow_factor=p.curv_slow_factor)
        dv = p.a_max * dt
        self.v_prev = max(0.0, self.v_prev + max(-dv, min(dv, v_tgt - self.v_prev)))
        out.v_cmd = self.v_prev
        out.state = self.state = TRACKING
        return out


def speed_to_rpm(v: float, rpm_per_mps: Optional[float], min_rpm: float, max_rpm: float) -> float:
    """기존 path_tracking 의 rpm 변환 (피드포워드 부분). 전진만 하므로 rpm >= 0."""
    if rpm_per_mps is None or v <= 1e-4:
        return 0.0
    return max(min_rpm, min(max_rpm, v * rpm_per_mps))
