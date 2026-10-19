#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
path_tracking.py — 경로 추종 제어의 순수 로직 (ROS 없음)

path_follower.py(ROS 노드)가 이 모듈을 감싼다. 하드웨어도 ROS 도 없이 import
해서 검증할 수 있도록 표준 라이브러리만 쓴다.

    python3 path_tracking.py          # 폐루프 셀프테스트 (직선 / S자 / 후진 커스프)

--------------------------------------------------------------------------
좌표 규약
--------------------------------------------------------------------------
    제어는 전부 미터 단위 오른손 좌표계(map)에서 한다. x 오른쪽, y 위쪽,
    yaw 는 x 축에서 반시계 + [rad].

    경로와 위치는 '5 cm 픽셀' 로 들어올 수 있다. 픽셀 (col, row) 은
        x = origin_x + col * res
        y = origin_y + row * res      (pixel_y_down = False)
        y = origin_y - row * res      (pixel_y_down = True, 이미지 규약)
    로 바꾼다. pixel_y_down 이면 y 축이 뒤집히므로 yaw 부호도 뒤집는다.

--------------------------------------------------------------------------
경로 전처리  (Path.build)
--------------------------------------------------------------------------
 1) 중복점 제거
 2) 커스프 분할: 연속한 두 구간의 방향이 cusp_angle_deg 넘게 꺾이면 그 점에서
    경로를 나눈다. 커스프 = 진행 방향이 뒤집히는 점 (전진 -> 후진 전환 지점).
    격자 경로의 계단은 최대 90 deg 라서 기본 120 deg 에 걸리지 않는다.
 3) 구간마다 이동평균으로 다듬는다 (양 끝점은 고정). 8-연결 격자 경로는
    계단처럼 꺾이므로 그대로 쓰면 헤딩·곡률이 톱니가 된다.
 4) ds 간격으로 재샘플링하고 점마다 접선 헤딩 psi, 곡률 kappa, 누적거리 s 를 구한다.

--------------------------------------------------------------------------
조향 (PID + 피드포워드)
--------------------------------------------------------------------------
    e_y   : 경로 기준 횡오차 [m]. 로버가 경로의 왼쪽이면 +.
    e_psi : 헤딩오차 = wrap(psi_path - psi_rover) [rad]. 경로가 반시계 쪽이면 +.

    Delta_v = atan(L * kappa)                       피드포워드 (경로 곡률)
            - (Kp * e_y + Ki * int(e_y) + Kd * de_y/dt)   횡오차 PID
            + Kpsi * e_psi                          헤딩 항

    Delta_v 는 radxa.py 의 steer_cmd 와 같은 '차량 유효조향각' 이다.
    곡률 = tan(Delta_v) / L 이므로 피드포워드가 경로 곡률을 그대로 낸다.
    L 은 굴절각에 따라 바뀐다 (로버 diagnostics 의 wheelbase_mm).

    de_y/dt 는 두 방식 중 고른다.
        'analytic' : de_y/dt = -v * sin(e_psi)   (운동학 관계. pose 잡음에 강함)
        'numeric'  : e_y 의 수치미분 + 1차 저역통과
    적분은 |e_y| < i_zone_m 일 때만 하고, 한계(i_limit_deg)로 자르고, 조향이 포화된
    방향으로는 더 쌓지 않는다.

후진
----
    구간에 들어갈 때 로버 헤딩과 구간 시작 접선이 90 deg 넘게 어긋나면 그 구간을
    후진으로 추종한다. 후진 중에는 가상 헤딩 psi + pi 로 오차를 계산하고 조향
    부호를 뒤집는다. 가상 차량(앞을 보고 |v| 로 전진)의 yaw rate 가
    |v| * kappa' 이고 실제 차량은 v * kappa = -|v| * kappa 이므로 kappa = -kappa'.
    이 로버는 전후방 역상 대칭 조향(Delta_a_f = -Delta_a_r)이라 앞뒤가 같은
    기구학을 가지므로 그대로 성립한다.

--------------------------------------------------------------------------
속도 (피드포워드 + PI)
--------------------------------------------------------------------------
    v_ref = min(v_max,
                sqrt(a_lat_max / |kappa_ahead|),      곡선 감속
                sqrt(2 * a_dec * 남은거리))           끝점·커스프 앞 감속
            * (횡오차·헤딩오차가 크면 줄이는 배율)
    rpm   = v_ref * rpm_per_mps  +  PI(v_ref - |v_odom|)
    rpm_per_mps = 60 * gear_ratio / (pi * wheel_diameter)     [모터 rpm / (m/s)]

    |rpm| <= max_rpm 으로 자르고, 0 < |rpm| < min_rpm 이면 min_rpm 으로 올린다
    (저속에서 모터가 못 도는 구간). 정지 지령(v_ref = 0)은 0 그대로다.
"""

from __future__ import annotations

import math
import os
import sys
from dataclasses import dataclass, field
from typing import List, Optional, Sequence, Tuple

# l12_driver 는 상위 폴더에 있다. 순수 계산 부분만 쓴다 (하드웨어 import 없음).
_HERE = os.path.dirname(os.path.abspath(__file__))
if os.path.dirname(_HERE) not in sys.path:
    sys.path.insert(0, os.path.dirname(_HERE))
from motor_control.l12.driver import DEFAULT_STEER_LIMIT_DEG as MAX_STEER_DEG, DEFAULT_ARTICULATION, wheelbase_mm  # noqa: E402

__all__ = [
    "TrackerConfig", "Path", "PathTracker", "TrackerOutput", "PoseEstimator",
    "SimRover", "pixel_to_map", "map_to_pixel", "wrap",
    "IDLE", "TRACK", "CUSP", "DONE", "ERROR",
]

# 상태
IDLE = "IDLE"      # 경로 없음 / 비활성
TRACK = "TRACK"    # 추종 중
CUSP = "CUSP"      # 커스프에서 정지 대기 (방향 전환 전)
DONE = "DONE"      # 끝점 도달
ERROR = "ERROR"    # 경로 이탈 등으로 정지

DEFAULT_WHEELBASE_M = wheelbase_mm(DEFAULT_ARTICULATION) * 1e-3


def wrap(a: float) -> float:
    """각도를 (-pi, pi] 로."""
    return math.atan2(math.sin(a), math.cos(a))


def _clamp(v: float, lo: float, hi: float) -> float:
    return lo if v < lo else hi if v > hi else v


# =========================================================================
# 설정
# =========================================================================
@dataclass
class TrackerConfig:
    # ---- 좌표 ----
    resolution_m: float = 0.05        # 1 픽셀 [m]
    origin_x_m: float = 0.0           # 픽셀 (0,0) 의 map 좌표 [m]
    origin_y_m: float = 0.0
    pixel_y_down: bool = True         # 이미지 규약 (row 증가 = 아래)

    # ---- 경로 전처리 ----
    smooth_window: int = 7            # 이동평균 점 수 (홀수). 1 = 끔
    resample_ds_m: float = 0.05
    cusp_angle_deg: float = 120.0
    curvature_window: int = 5         # 곡률 이동평균 점 수

    # ---- 조향 PID ----
    kp_deg_per_m: float = 250.0       # 횡오차 P  [deg / m]
    ki_deg_per_m_s: float = 5.0       # 횡오차 I  [deg / (m.s)]
    kd_deg_per_mps: float = 0.0       # 횡오차 D  [deg / (m/s)]
    k_heading: float = 1.5            # 헤딩 항   [deg / deg]
    d_mode: str = "analytic"          # 'analytic' | 'numeric'
    d_filter_tau_s: float = 0.15      # numeric 미분 저역통과 시정수
    i_limit_deg: float = 5.0          # 적분 항 크기 한계 [deg]
    i_zone_m: float = 0.10            # |e_y| 가 이보다 작을 때만 적분 (큰 초기오차로 쌓이지 않게)
    use_feedforward: bool = True
    max_steer_deg: float = MAX_STEER_DEG
    steer_rate_dps: float = 90.0      # 조향 지령 변화율 한계 [deg/s]. 0 = 없음

    # ---- 속도 ----
    wheel_diameter_m: float = 0.0     # 0 = 미설정 (나중에 실측값)
    gear_ratio: float = 0.0           # 모터 회전 / 바퀴 회전. 0 = 미설정
    min_rpm: float = 0.0              # 모터 rpm. 이보다 작은 0 아닌 지령은 올린다
    max_rpm: float = 500.0            # 모터 rpm. VESC config 의 max_speed 이하로
    v_max_mps: float = 0.30
    a_lat_max_mps2: float = 0.30      # 곡선 횡가속도 한계
    a_acc_mps2: float = 0.30          # 가속 한계
    a_dec_mps2: float = 0.30          # 끝점·커스프 앞 감속
    preview_m: float = 0.50           # 곡률 선행 확인 거리
    cte_slow_m: float = 0.20          # 이 횡오차에서 속도 배율이 최소가 된다
    heading_slow_deg: float = 45.0    # 이 헤딩오차에서 속도 배율이 최소가 된다
    slow_scale_min: float = 0.3
    kp_v_rpm_per_mps: float = 200.0   # 속도 PI (odom 있을 때만)
    ki_v_rpm_per_m: float = 100.0
    i_v_limit_rpm: float = 100.0

    # ---- 종료 / 안전 ----
    allow_reverse: bool = True
    goal_tol_m: float = 0.05          # 끝점·커스프 도달 판정 거리 (1 픽셀)
    max_cte_m: float = 0.60           # 이보다 벗어나면 ERROR 정지
    stop_speed_mps: float = 0.02      # 커스프에서 '멈췄다' 판정 속도
    cusp_wait_s: float = 0.5          # odom 이 없을 때 커스프 대기 시간
    search_back_m: float = 0.20       # 최근접점 검색 창 (뒤)
    search_ahead_m: float = 1.00      # 최근접점 검색 창 (앞)

    @property
    def rpm_per_mps(self) -> Optional[float]:
        """모터 rpm / (m/s). 바퀴 지름·기어비가 없으면 None."""
        if self.wheel_diameter_m > 0 and self.gear_ratio > 0:
            return 60.0 * self.gear_ratio / (math.pi * self.wheel_diameter_m)
        return None

    def validate(self) -> List[str]:
        """문제 목록. 비어 있으면 주행 가능."""
        errs = []
        if self.rpm_per_mps is None:
            errs.append("wheel_diameter_m / gear_ratio 미설정 (0). rpm 을 계산할 수 없습니다.")
        if not 0 <= self.min_rpm <= self.max_rpm:
            errs.append(f"0 <= min_rpm({self.min_rpm}) <= max_rpm({self.max_rpm}) 이어야 합니다.")
        if self.d_mode not in ("analytic", "numeric"):
            errs.append("d_mode 는 analytic | numeric")
        if self.resolution_m <= 0 or self.resample_ds_m <= 0:
            errs.append("resolution_m, resample_ds_m 은 양수")
        return errs


# =========================================================================
# 픽셀 <-> map
# =========================================================================
def pixel_to_map(col: float, row: float, cfg: TrackerConfig) -> Tuple[float, float]:
    sy = -1.0 if cfg.pixel_y_down else 1.0
    return (cfg.origin_x_m + col * cfg.resolution_m,
            cfg.origin_y_m + sy * row * cfg.resolution_m)


def map_to_pixel(x: float, y: float, cfg: TrackerConfig) -> Tuple[float, float]:
    sy = -1.0 if cfg.pixel_y_down else 1.0
    return ((x - cfg.origin_x_m) / cfg.resolution_m,
            sy * (y - cfg.origin_y_m) / cfg.resolution_m)


def yaw_pixel_to_map(yaw: float, cfg: TrackerConfig) -> float:
    return -yaw if cfg.pixel_y_down else yaw


# =========================================================================
# 경로
# =========================================================================
def _moving_average(pts: List[Tuple[float, float]], w: int) -> List[Tuple[float, float]]:
    """양 끝점을 고정한 이동평균. 끝 근처는 창을 대칭으로 줄인다."""
    n = len(pts)
    if w <= 1 or n < 3:
        return list(pts)
    h = w // 2
    out = [pts[0]]
    for i in range(1, n - 1):
        k = min(h, i, n - 1 - i)
        xs = [p[0] for p in pts[i - k:i + k + 1]]
        ys = [p[1] for p in pts[i - k:i + k + 1]]
        out.append((sum(xs) / len(xs), sum(ys) / len(ys)))
    out.append(pts[-1])
    return out


def _resample(pts: List[Tuple[float, float]], ds: float) -> List[Tuple[float, float]]:
    """폴리라인을 호길이 ds 간격으로 다시 찍는다. 끝점은 항상 포함."""
    if len(pts) < 2:
        return list(pts)
    out = [pts[0]]
    carry = 0.0                        # 다음 샘플까지 남은 거리
    for (x0, y0), (x1, y1) in zip(pts, pts[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg <= 0:
            continue
        d = ds - carry
        while d <= seg + 1e-12:
            t = d / seg
            out.append((x0 + t * (x1 - x0), y0 + t * (y1 - y0)))
            d += ds
        carry = seg - (d - ds)
    if math.hypot(out[-1][0] - pts[-1][0], out[-1][1] - pts[-1][1]) > 1e-3 * ds:
        out.append(pts[-1])
    return out


@dataclass
class Segment:
    start: int          # Path 배열에서의 시작 인덱스 (포함)
    end: int            # 끝 인덱스 (포함)


@dataclass
class Path:
    x: List[float] = field(default_factory=list)
    y: List[float] = field(default_factory=list)
    psi: List[float] = field(default_factory=list)     # 접선 헤딩 [rad]
    kappa: List[float] = field(default_factory=list)   # 곡률 [1/m], 좌회전 +
    s: List[float] = field(default_factory=list)       # 구간 안 누적거리 [m]
    segments: List[Segment] = field(default_factory=list)

    def __len__(self):
        return len(self.x)

    @classmethod
    def build(cls, pts_m: Sequence[Tuple[float, float]], cfg: TrackerConfig) -> "Path":
        # 1) 중복점 제거
        pts: List[Tuple[float, float]] = []
        for p in pts_m:
            if not pts or math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) > 1e-6:
                pts.append((float(p[0]), float(p[1])))
        path = cls()
        if len(pts) < 2:
            return path

        # 2) 커스프 분할
        lim = math.radians(cfg.cusp_angle_deg)
        pieces, cur = [], [pts[0], pts[1]]
        for i in range(1, len(pts) - 1):
            a = math.atan2(pts[i][1] - pts[i - 1][1], pts[i][0] - pts[i - 1][0])
            b = math.atan2(pts[i + 1][1] - pts[i][1], pts[i + 1][0] - pts[i][0])
            if abs(wrap(b - a)) > lim:
                pieces.append(cur)
                cur = [pts[i]]
            cur.append(pts[i + 1])
        pieces.append(cur)

        # 3) 4) 구간마다 다듬기 + 재샘플링 + 헤딩/곡률
        for piece in pieces:
            if len(piece) < 2:
                continue
            fine = _resample(piece, cfg.resample_ds_m / 5.0)
            # 창 크기는 원래 점 간격 기준이므로 촘촘해진 만큼 늘린다
            smooth = _moving_average(fine, max(1, cfg.smooth_window) * 5 | 1)
            rs = _resample(smooth, cfg.resample_ds_m)
            if len(rs) < 2:
                continue
            n = len(rs)
            psi = []
            for i in range(n):
                j0, j1 = max(0, i - 1), min(n - 1, i + 1)
                psi.append(math.atan2(rs[j1][1] - rs[j0][1], rs[j1][0] - rs[j0][0]))
            kap = []
            for i in range(n):
                j0, j1 = max(0, i - 1), min(n - 1, i + 1)
                d = math.hypot(rs[j1][0] - rs[j0][0], rs[j1][1] - rs[j0][1])
                kap.append(wrap(psi[j1] - psi[j0]) / d if d > 0 else 0.0)
            w = max(1, cfg.curvature_window)
            h = w // 2
            kap = [sum(kap[max(0, i - h):i + h + 1]) / len(kap[max(0, i - h):i + h + 1])
                   for i in range(n)]
            base = len(path.x)
            acc = 0.0
            for i in range(n):
                if i > 0:
                    acc += math.hypot(rs[i][0] - rs[i - 1][0], rs[i][1] - rs[i - 1][1])
                path.x.append(rs[i][0])
                path.y.append(rs[i][1])
                path.psi.append(psi[i])
                path.kappa.append(kap[i])
                path.s.append(acc)
            path.segments.append(Segment(base, base + n - 1))
        return path

    @property
    def length(self) -> float:
        return sum(self.s[sg.end] for sg in self.segments)


# =========================================================================
# 위치 추정: 맵 pose + 그 뒤의 odom 변화량
# =========================================================================
def _se2_between(a, b):
    """a 기준 b 의 상대 변환 (dx, dy, dyaw) — a^-1 * b."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    c, s = math.cos(a[2]), math.sin(a[2])
    return (c * dx + s * dy, -s * dx + c * dy, wrap(b[2] - a[2]))


def _se2_compose(a, d):
    """a * d."""
    c, s = math.cos(a[2]), math.sin(a[2])
    return (a[0] + c * d[0] - s * d[1], a[1] + s * d[0] + c * d[1], wrap(a[2] + d[2]))


class PoseEstimator:
    """
    맵 pose 는 느리거나 늦게 올 수 있다. 마지막 맵 pose 에, 그것을 받은 이후
    odom 이 움직인 만큼(SE2 상대 변환)을 더해 현재 위치로 쓴다.
    odom 이 없으면 맵 pose 를 그대로 쓴다.
    """

    def __init__(self, use_odom: bool = True):
        self.use_odom = use_odom
        self.map_pose = None       # (x, y, yaw)
        self.map_t = None
        self.odom_at_map = None    # 맵 pose 를 받은 순간의 odom
        self.odom = None
        self.odom_t = None
        self.odom_v = None         # 전진 + [m/s]

    def on_map_pose(self, x, y, yaw, t):
        self.map_pose = (x, y, yaw)
        self.map_t = t
        self.odom_at_map = self.odom

    def on_odom(self, x, y, yaw, v, t):
        self.odom = (x, y, yaw)
        self.odom_v = v
        self.odom_t = t

    def age(self, now) -> float:
        return float('inf') if self.map_t is None else now - self.map_t

    def odom_fresh(self, now, timeout) -> bool:
        return self.odom_t is not None and now - self.odom_t <= timeout

    def estimate(self):
        if self.map_pose is None:
            return None
        if self.use_odom and self.odom is not None and self.odom_at_map is not None:
            return _se2_compose(self.map_pose, _se2_between(self.odom_at_map, self.odom))
        return self.map_pose


# =========================================================================
# 추종기
# =========================================================================
@dataclass
class TrackerOutput:
    rpm: float = 0.0            # 모터 rpm, 부호 = 전/후진
    steer_deg: float = 0.0      # 차량 유효조향각
    state: str = IDLE
    message: str = ""
    # --- 진단 ---
    segment: int = -1
    n_segments: int = 0
    reverse: bool = False
    index: int = -1
    progress: float = 0.0       # 0~1 (전체 경로 기준)
    remain_m: float = 0.0       # 현재 구간 끝까지
    e_y: float = 0.0
    e_psi_deg: float = 0.0
    kappa: float = 0.0
    v_ref: float = 0.0
    v_meas: float = float('nan')
    steer_ff: float = 0.0
    steer_p: float = 0.0
    steer_i: float = 0.0
    steer_d: float = 0.0
    steer_h: float = 0.0
    rpm_ff: float = 0.0
    rpm_fb: float = 0.0


class PathTracker:
    """
    step() 을 일정 주기로 부른다. 상태를 가지는 것은 이 객체 하나다
    (진행 인덱스, 구간, 방향, 적분기, 슬루 상태).
    """

    def __init__(self, cfg: TrackerConfig):
        self.cfg = cfg
        self.path: Optional[Path] = None
        self.reset()

    # ------------------------------------------------------------
    def reset(self):
        self.state = IDLE
        self.message = ""
        self.seg = 0
        self.idx = 0
        self.reverse = False
        self._seg_entered = False
        self._cusp_t = None
        self._i_y = 0.0
        self._d_prev_e = None
        self._d_filt = 0.0
        self._i_v = 0.0
        self._v_cmd = 0.0
        self._steer = 0.0

    def set_path(self, path: Path):
        self.path = path
        self.reset()
        if path is None or not path.segments:
            self.state, self.message = IDLE, "경로가 비어 있습니다."
            return
        self.state = TRACK
        self.idx = path.segments[0].start

    def _enter_segment(self, k: int, pose):
        """구간 k 에 들어간다. 방향을 여기서 한 번 정하고 구간 동안 유지한다."""
        sg = self.path.segments[k]
        self.seg, self.idx = k, sg.start
        diff = abs(wrap(self.path.psi[sg.start] - pose[2]))
        self.reverse = self.cfg.allow_reverse and diff > math.pi / 2
        self._seg_entered = True
        self._i_y, self._d_prev_e, self._d_filt, self._i_v = 0.0, None, 0.0, 0.0

    # ------------------------------------------------------------
    def _nearest(self, px, py) -> int:
        p, sg, c = self.path, self.path.segments[self.seg], self.cfg
        ds = max(c.resample_ds_m, 1e-3)
        lo = max(sg.start, self.idx - int(c.search_back_m / ds))
        hi = min(sg.end, self.idx + int(c.search_ahead_m / ds))
        best, bi = float('inf'), self.idx
        for i in range(lo, hi + 1):
            d = (p.x[i] - px) ** 2 + (p.y[i] - py) ** 2
            if d < best:
                best, bi = d, i
        return bi

    # ------------------------------------------------------------
    def step(self, pose, dt: float, now: float, *, wheelbase_m: float = DEFAULT_WHEELBASE_M,
             v_meas: Optional[float] = None) -> TrackerOutput:
        """
        pose    : (x, y, yaw) map [m, rad]. None 이면 정지.
        v_meas  : odom 속도 [m/s] (전진 +). None 이면 속도 피드백 없이 피드포워드만.
        """
        c = self.cfg
        out = TrackerOutput(state=self.state, message=self.message)
        out.v_meas = float('nan') if v_meas is None else v_meas
        p = self.path

        if self.state in (IDLE, DONE, ERROR) or p is None or not p.segments:
            return self._stop(out, dt)
        if pose is None:
            out.message = "위치 없음"
            return self._stop(out, dt)
        if c.rpm_per_mps is None:
            self.state = ERROR
            self.message = out.message = "wheel_diameter_m / gear_ratio 미설정"
            out.state = ERROR
            return self._stop(out, dt)

        if not self._seg_entered:
            self._enter_segment(self.seg, pose)

        # --- 커스프: 멈출 때까지 기다렸다가 다음 구간으로 ---
        if self.state == CUSP:
            stopped = (abs(v_meas) < c.stop_speed_mps if v_meas is not None
                       else now - self._cusp_t >= c.cusp_wait_s)
            if stopped and self._v_cmd <= 1e-6:
                self._enter_segment(self.seg + 1, pose)
                self.state = TRACK
            else:
                out.message = "커스프 정지 대기"
                self._fill_diag(out)
                return self._stop(out, dt, hold_steer=True)

        # --- 투영 ---
        self.idx = i = self._nearest(pose[0], pose[1])
        sg = p.segments[self.seg]
        psi_p, kap = p.psi[i], p.kappa[i]
        dx, dy = pose[0] - p.x[i], pose[1] - p.y[i]
        e_y = -dx * math.sin(psi_p) + dy * math.cos(psi_p)       # 좌 +
        along = dx * math.cos(psi_p) + dy * math.sin(psi_p)      # 진행방향 +
        yaw_eff = wrap(pose[2] + math.pi) if self.reverse else pose[2]
        e_psi = wrap(psi_p - yaw_eff)
        remain = max(0.0, p.s[sg.end] - p.s[i] - along)

        out.e_y, out.e_psi_deg, out.kappa = e_y, math.degrees(e_psi), kap
        out.remain_m = remain

        # --- 이탈 ---
        if abs(e_y) > c.max_cte_m:
            self.state = ERROR
            self.message = f"경로 이탈 |e_y|={abs(e_y):.2f} m > {c.max_cte_m:.2f} m"
            out.state, out.message = ERROR, self.message
            self._fill_diag(out)
            return self._stop(out, dt)

        # --- 구간 끝 ---
        last = self.seg == len(p.segments) - 1
        end_d = math.hypot(pose[0] - p.x[sg.end], pose[1] - p.y[sg.end])
        if (i == sg.end and along >= 0.0) or end_d < c.goal_tol_m:
            if last:
                self.state, self.message = DONE, "끝점 도달"
                out.state, out.message = DONE, self.message
                self._fill_diag(out)
                return self._stop(out, dt, hold_steer=True)
            self.state, self._cusp_t = CUSP, now
            out.state, out.message = CUSP, "커스프 도달"
            self._fill_diag(out)
            return self._stop(out, dt, hold_steer=True)

        # --- 조향 ---
        v_abs = abs(v_meas) if v_meas is not None else self._v_cmd
        ff = math.degrees(math.atan(wheelbase_m * kap)) if c.use_feedforward else 0.0
        if c.d_mode == "analytic":
            de = -v_abs * math.sin(e_psi)
        else:
            raw = 0.0 if self._d_prev_e is None or dt <= 0 else (e_y - self._d_prev_e) / dt
            a = dt / (c.d_filter_tau_s + dt) if dt > 0 else 1.0
            self._d_filt += a * (raw - self._d_filt)
            de = self._d_filt
        self._d_prev_e = e_y
        p_term = -c.kp_deg_per_m * e_y
        d_term = -c.kd_deg_per_mps * de
        h_term = c.k_heading * math.degrees(e_psi)
        i_term = -c.ki_deg_per_m_s * self._i_y
        virt = ff + p_term + i_term + d_term + h_term
        sat = abs(virt) >= c.max_steer_deg
        # anti-windup: 포화 방향으로는 적분하지 않는다
        if (c.ki_deg_per_m_s > 0 and dt > 0 and self._v_cmd > 1e-3
                and abs(e_y) < c.i_zone_m):
            grow = -c.ki_deg_per_m_s * e_y * dt           # 이번 주기 적분 항 변화 [deg]
            if not (sat and grow * virt > 0):
                self._i_y += e_y * dt
                lim = c.i_limit_deg / c.ki_deg_per_m_s
                self._i_y = _clamp(self._i_y, -lim, lim)
        virt = _clamp(virt, -c.max_steer_deg, c.max_steer_deg)
        target = -virt if self.reverse else virt
        if c.steer_rate_dps > 0 and dt > 0:
            step = c.steer_rate_dps * dt
            target = _clamp(target, self._steer - step, self._steer + step)
        self._steer = target

        out.steer_ff, out.steer_p, out.steer_i = ff, p_term, i_term
        out.steer_d, out.steer_h = d_term, h_term

        # --- 속도 ---
        k_ahead = 0.0
        j = i
        while j <= sg.end and p.s[j] - p.s[i] <= c.preview_m:
            k_ahead = max(k_ahead, abs(p.kappa[j]))
            j += 1
        v_ref = c.v_max_mps
        if k_ahead > 1e-6:
            v_ref = min(v_ref, math.sqrt(c.a_lat_max_mps2 / k_ahead))
        v_ref = min(v_ref, math.sqrt(2.0 * c.a_dec_mps2 * max(0.0, remain - 0.5 * c.goal_tol_m)))
        scale = min(1.0 - abs(e_y) / max(c.cte_slow_m, 1e-6) * (1 - c.slow_scale_min),
                    1.0 - abs(math.degrees(e_psi)) / max(c.heading_slow_deg, 1e-6)
                    * (1 - c.slow_scale_min))
        v_ref *= _clamp(scale, c.slow_scale_min, 1.0)
        # 가속 한계 (감속은 위의 프로파일이 이미 만든다)
        if dt > 0:
            v_ref = min(v_ref, self._v_cmd + c.a_acc_mps2 * dt)
        self._v_cmd = v_ref
        out.v_ref = v_ref

        self._speed_to_rpm(out, v_ref, v_meas, dt)
        out.steer_deg = self._steer
        out.state = self.state
        self._fill_diag(out)
        return out

    # ------------------------------------------------------------
    def _speed_to_rpm(self, out: TrackerOutput, v_ref, v_meas, dt):
        c = self.cfg
        k = c.rpm_per_mps
        if v_ref <= 1e-4:
            self._i_v = 0.0
            out.rpm = 0.0
            return
        ff = v_ref * k
        fb = 0.0
        if v_meas is not None:
            err = v_ref - abs(v_meas)
            self._i_v = _clamp(self._i_v + c.ki_v_rpm_per_m * err * dt,
                               -c.i_v_limit_rpm, c.i_v_limit_rpm)
            fb = c.kp_v_rpm_per_mps * err + self._i_v
        mag = _clamp(ff + fb, 0.0, c.max_rpm)
        if mag < c.min_rpm:
            mag = c.min_rpm
        out.rpm_ff, out.rpm_fb = ff, fb
        out.rpm = -mag if self.reverse else mag

    def _stop(self, out: TrackerOutput, dt, hold_steer=False) -> TrackerOutput:
        self._v_cmd = 0.0
        self._i_v = 0.0
        out.rpm = 0.0
        out.v_ref = 0.0
        out.steer_deg = self._steer if hold_steer else 0.0
        if not hold_steer:
            self._steer = 0.0
        if not out.state:
            out.state = self.state
        return out

    def _fill_diag(self, out: TrackerOutput):
        p = self.path
        out.segment, out.n_segments = self.seg, len(p.segments) if p else 0
        out.reverse, out.index = self.reverse, self.idx
        if p and p.segments:
            done = sum(p.s[sg.end] for sg in p.segments[:self.seg])
            sg = p.segments[self.seg]
            done += p.s[min(max(self.idx, sg.start), sg.end)]
            out.progress = done / p.length if p.length > 0 else 0.0


# =========================================================================
# 시뮬레이션 로버 (셀프테스트와 sim_rover.py 가 같이 쓴다)
# =========================================================================
class SimRover:
    """
    휠베이스 중점 기준 운동학 모델.
        yaw_rate = v * tan(Delta_v) / L
    조향은 L12 가 느린 것을 흉내 내는 변화율 한계 + 1차 지연, 속도는 1차 지연.
    turn_ratio: [261015] 언더스티어 (실제/지령 곡률, 1 = 없음). yaw_rate 에 곱한다
    """

    def __init__(self, x=0.0, y=0.0, yaw=0.0, *, wheelbase_m=DEFAULT_WHEELBASE_M,
                 rpm_per_mps=1000.0, steer_tau_s=0.15, steer_rate_dps=120.0,
                 speed_tau_s=0.2, max_steer_deg=MAX_STEER_DEG, turn_ratio=1.0):
        self.x, self.y, self.yaw = x, y, yaw
        self.L = wheelbase_m
        self.k = rpm_per_mps
        self.steer_tau, self.steer_rate = steer_tau_s, steer_rate_dps
        self.speed_tau = speed_tau_s
        self.max_steer = max_steer_deg
        self.turn_ratio = turn_ratio
        self.steer = 0.0        # 실제 조향 [deg]
        self.v = 0.0            # 실제 속도 [m/s]
        self.odom = (0.0, 0.0, 0.0)

    def step(self, rpm_cmd, steer_cmd_deg, dt):
        tgt = _clamp(steer_cmd_deg, -self.max_steer, self.max_steer)
        d = (tgt - self.steer) * min(1.0, dt / self.steer_tau)
        lim = self.steer_rate * dt
        self.steer += _clamp(d, -lim, lim)
        v_tgt = rpm_cmd / self.k
        self.v += (v_tgt - self.v) * min(1.0, dt / self.speed_tau)
        w = self.turn_ratio * self.v * math.tan(math.radians(self.steer)) / self.L
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt
        self.yaw = wrap(self.yaw + w * dt)
        ox, oy, oyaw = self.odom
        self.odom = (ox + self.v * math.cos(oyaw) * dt,
                     oy + self.v * math.sin(oyaw) * dt, wrap(oyaw + w * dt))
        return w


# =========================================================================
# 셀프테스트
# =========================================================================
def _rasterize(pts_m, cfg: TrackerConfig):
    """연속 곡선 -> 5 cm 픽셀 경로 (계단). 실제 격자 플래너 출력을 흉내 낸다."""
    px = []
    for x, y in pts_m:
        c, r = map_to_pixel(x, y, cfg)
        cell = (round(c), round(r))
        if not px or px[-1] != cell:
            px.append(cell)
    return px


def _run_case(name, pts_m, start, cfg, *, pose_period=0.1, pose_noise=0.01,
              seconds=60.0, seed=1, settle_m=1.0):
    import random
    rnd = random.Random(seed)
    px = _rasterize(pts_m, cfg)
    path = Path.build([pixel_to_map(c, r, cfg) for c, r in px], cfg)
    trk = PathTracker(cfg)
    trk.set_path(path)
    rover = SimRover(*start, rpm_per_mps=cfg.rpm_per_mps)
    est = PoseEstimator()
    dt, t, t_pose, dist = 0.05, 0.0, -1.0, 0.0
    ctes, max_after, reversed_seen = [], 0.0, False
    out = TrackerOutput()
    while t < seconds:
        # 시뮬은 1 ms 로 적분, 제어는 dt 마다
        for _ in range(int(dt / 0.001)):
            rover.step(out.rpm, out.steer_deg, 0.001)
        t += dt
        dist += abs(rover.v) * dt
        est.on_odom(*rover.odom, rover.v, t)
        if t - t_pose >= pose_period:
            est.on_map_pose(rover.x + rnd.gauss(0, pose_noise),
                            rover.y + rnd.gauss(0, pose_noise),
                            rover.yaw + rnd.gauss(0, math.radians(1.0)), t)
            t_pose = t
        out = trk.step(est.estimate(), dt, t, v_meas=rover.v)
        reversed_seen |= out.reverse and out.rpm < 0
        if out.state == TRACK:
            ctes.append(out.e_y)
            if dist > settle_m:
                max_after = max(max_after, abs(out.e_y))
        if out.state in (DONE, ERROR):
            break
    rms = math.sqrt(sum(e * e for e in ctes) / len(ctes)) if ctes else float('nan')
    end_err = math.hypot(rover.x - path.x[-1], rover.y - path.y[-1])
    print(f"  {name:<14} {out.state:<6} t={t:5.1f}s  구간 {len(path.segments)}  "
          f"길이 {path.length:5.2f} m  RMS cte {rms * 100:5.1f} cm  "
          f"max cte({settle_m:.1f} m 후) {max_after * 100:5.1f} cm  끝점 오차 {end_err * 100:4.1f} cm"
          f"{'  [후진 사용]' if reversed_seen else ''}")
    return out.state, rms, max_after, end_err, reversed_seen


def _self_test():
    cfg = TrackerConfig(wheel_diameter_m=0.10, gear_ratio=10.0, min_rpm=100.0, max_rpm=2000.0)
    assert not cfg.validate(), cfg.validate()
    print(f"rpm_per_mps = {cfg.rpm_per_mps:.1f}, L = {DEFAULT_WHEELBASE_M * 1e3:.1f} mm")

    # 픽셀 변환 왕복
    for pydown in (True, False):
        c2 = TrackerConfig(origin_x_m=1.0, origin_y_m=-2.0, pixel_y_down=pydown)
        x, y = pixel_to_map(*map_to_pixel(0.37, 1.23, c2), c2)
        assert abs(x - 0.37) < 1e-9 and abs(y - 1.23) < 1e-9

    # 커스프 분할
    pts = [(i * 0.05, 0.0) for i in range(30)] + [(1.45 - i * 0.05, 0.3 * (i * 0.05) ** 2)
                                                  for i in range(1, 30)]
    pth = Path.build(pts, cfg)
    assert len(pth.segments) == 2, len(pth.segments)

    # 폐루프
    straight = [(i * 0.01, 0.0) for i in range(301)]
    s_curve = [(i * 0.01, 0.5 * math.sin(2 * math.pi * i * 0.01 / 3.0)) for i in range(451)]
    # 커스프: 직진 1.5 m -> 뒤로 곡선을 그리며 후진
    cusp = [(i * 0.01, 0.0) for i in range(151)]
    cusp += [(1.5 - 1.0 * math.sin(a), 1.0 - 1.0 * math.cos(a))
             for a in [k * 0.01 for k in range(1, 158)]]
    print("폐루프 (pose 10 Hz, 잡음 1 cm / 1 deg, 제어 20 Hz, 격자 경로):")
    r = _run_case("직선(오프셋)", straight, (0.0, 0.15, math.radians(15)), cfg)
    assert r[0] == DONE and r[2] < 0.05 and r[3] < 0.12, r
    r = _run_case("S자", s_curve, (0.0, 0.0, math.radians(45)), cfg)
    assert r[0] == DONE and r[2] < 0.08 and r[3] < 0.12, r
    r = _run_case("후진 커스프", cusp, (0.0, 0.0, 0.0), cfg)
    assert r[0] == DONE and r[4] and r[2] < 0.10 and r[3] < 0.12, r
    r = _run_case("시작부터 후진", [(-i * 0.01, 0.0) for i in range(201)],
                  (0.0, 0.05, 0.0), cfg)
    assert r[0] == DONE and r[4], r

    # 미설정이면 주행 거부
    t2 = PathTracker(TrackerConfig())
    t2.set_path(Path.build(straight, cfg))
    o = t2.step((0.0, 0.0, 0.0), 0.05, 0.0)
    assert o.state == ERROR and o.rpm == 0.0
    print("OK")


if __name__ == "__main__":
    _self_test()
