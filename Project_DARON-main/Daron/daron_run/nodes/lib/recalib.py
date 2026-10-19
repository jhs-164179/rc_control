"""전복(중력 반전) 감지 + 피치 재보정 — ROS 의존 없음.

노드(``imu_recalib_relay.py``)는 IMU 샘플마다 :meth:`RollRecalib.step` 을 부르고,
돌려받은 ``R_corr`` (3x3, corrected_body <- measured_body) 를 orientation·각속도·가속도에
적용해 재발행한다. 여기에는 판정·상태·계산만 있고 토픽은 없다.

[동작]
  1. 기준 중력 g0: 기동 뒤 첫 ``static_s`` 창(정지 조건 충족)의 가속도 평균.
     z 축 값이 아니라 **벡터** — IMU 가 base_link 에 -17.4° 피치로 달려 있어 z 만 보면 이미 기울어 있다.
  2. 전복 감지: s = a·ĝ0/|g0|.  s < -flip_ratio 가 hold_s 연속 → INVERTED,
     s > +flip_ratio 가 hold_s 연속 → UPRIGHT. 그 사이(-r..+r)는 판정 보류(히스테리시스).
  3. 재보정: 상태가 바뀐 뒤 정지 창(길이 static_s, 평균|ω| < gyro_thr, std|a| < acc_std_thr)
     에서 g1 을 재고, 기대값 e = R_x(180°)·ĝ0 (INVERTED) 또는 ĝ0 (UPRIGHT) 와의 차이를
     **Y 축 회전 Δθ 하나**로 풀어 R_corr = R_y(Δθ) 로 둔다. 롤 방향 잔차는 적용하지 않고
     ``last_event`` 에 남긴다(재구성이 피치만 바꾼다는 가정의 검증용).
     Δθ 는 항상 **g0 기준** 으로 새로 계산한다(직전 보정에 누적하지 않음).
  4. 정지 창이 max_wait_s 안에 안 오면 그동안 본 창 중 가장 조용한 창으로 계산한다
     (quality='relaxed'). 그것마저 std|a| > relaxed_acc_std_max 면 보정하지 않는다(quality='none').

     근거(viz2_rollover 실측, /ekf/imu/data): 전복 51.5~53.5 s → 재구성 54~61 s(피치 각속도) →
     61.0~62.0 s 만 준정지(|ω|max 0.15~0.28, std|a| 0.10) → 62 s 부터 후진. 전복 뒤 2 s 짜리
     엄격한 정지 창은 끝(86 s)까지 없다. 그래서 창 기본값은 1 s 이고 완화 폴백이 필요하다.

[시간] 전부 header stamp 기준 초(float). bag 배속과 무관.
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

UPRIGHT = "upright"
INVERTED = "inverted"
UNKNOWN = "unknown"


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def pitch_only_correction(g_meas: np.ndarray, g_expect: np.ndarray) -> tuple[float, float]:
    """R_y(Δθ)·ĝ_meas 가 ĝ_expect 에 가장 가까워지는 Δθ [rad] 와, 적용 후 남는 각도 잔차 [rad].

    Y 축 회전은 x-z 평면 안의 회전이므로 두 벡터의 x-z 성분 각도 차로 닫힌 해가 나온다.
    """
    u = np.array([g_meas[0], g_meas[2]])
    v = np.array([g_expect[0], g_expect[2]])
    if np.linalg.norm(u) < 1e-9 or np.linalg.norm(v) < 1e-9:
        return 0.0, float("nan")
    # rot_y(θ) 는 (x, z) → (x cosθ + z sinθ, -x sinθ + z cosθ): (x,z) 평면에서 -θ 회전.
    phi = math.atan2(u[0] * v[1] - u[1] * v[0], float(u @ v))
    theta = -phi
    a = rot_y(theta) @ (g_meas / np.linalg.norm(g_meas))
    b = g_expect / np.linalg.norm(g_expect)
    resid = math.acos(max(-1.0, min(1.0, float(a @ b))))
    return theta, resid


@dataclass
class RecalibEvent:
    t: float
    state: str                 # UPRIGHT | INVERTED
    delta_pitch_rad: float
    residual_rad: float        # 피치만 보정한 뒤 남는 각도(≈롤 잔차)
    quality: str               # strict | relaxed | none
    window_std_acc: float
    window_mean_gyro: float
    g0: tuple
    g1: tuple
    s_at_flip: float

    def summary(self) -> str:
        return (f"t={self.t:.2f}s state={self.state} dpitch={math.degrees(self.delta_pitch_rad):+.2f}deg "
                f"resid={math.degrees(self.residual_rad):.2f}deg quality={self.quality} "
                f"win(std|a|={self.window_std_acc:.3f} mean|w|={self.window_mean_gyro:.3f}) "
                f"g0=({self.g0[0]:+.2f},{self.g0[1]:+.2f},{self.g0[2]:+.2f}) "
                f"g1=({self.g1[0]:+.2f},{self.g1[1]:+.2f},{self.g1[2]:+.2f})")


@dataclass
class RollRecalib:
    flip_ratio: float = 0.5          # s 문턱 (g0 방향 투영, 무차원)
    hold_s: float = 0.5              # 문턱 밖 연속 유지 시간
    static_s: float = 1.0            # 정지 창 길이
    gyro_thr: float = 0.10           # 정지 창 평균 |ω| [rad/s]
    acc_std_thr: float = 0.15        # 정지 창 std|a| [m/s²]
    max_wait_s: float = 10.0         # 상태 변경 뒤 정지 창 대기 한도 → 완화 폴백
    relaxed_acc_std_max: float = 1.0 # 완화 폴백도 이보다 시끄러우면 보정 안 함
    enabled: bool = True             # False 면 감지·로그만 하고 R_corr 는 항등 유지

    # ── 상태 ──
    g0: np.ndarray | None = None
    state: str = UNKNOWN
    R_corr: np.ndarray = field(default_factory=lambda: np.eye(3))
    delta_pitch: float = 0.0
    last_event: RecalibEvent | None = None
    n_events: int = 0
    n: int = 0
    _buf: deque = field(default_factory=deque)          # (t, a, w) 최근 static_s 창
    _cand_state: str = UNKNOWN                            # 문턱 밖에 머무는 후보 상태
    _cand_since: float | None = None
    _pending_since: float | None = None                   # 상태 확정 후 정지 창 대기 시작
    _pending_s: float = 0.0
    _best: tuple | None = None                            # (score, t, g_mean, std, gyro)

    # ── 내부 ──
    def _window_stats(self):
        a = np.array([b[1] for b in self._buf])
        w = np.array([b[2] for b in self._buf])
        return a.mean(0), float(np.linalg.norm(a, axis=1).std()), float(np.linalg.norm(w, axis=1).mean())

    def _window_full(self, t: float) -> bool:
        # 버퍼는 span > static_s 인 샘플을 버리므로 span 은 static_s 에 못 미친다(부동소수 포함).
        # 90% 이상 차 있으면 창으로 인정한다.
        return len(self._buf) >= 2 and (t - self._buf[0][0]) >= 0.9 * self.static_s

    def _is_static(self, std_acc: float, mean_gyro: float) -> bool:
        return std_acc < self.acc_std_thr and mean_gyro < self.gyro_thr

    def _apply(self, t: float, g1: np.ndarray, std_acc: float, mean_gyro: float, quality: str, s: float):
        g0n = self.g0 / np.linalg.norm(self.g0)
        expect = rot_x(math.pi) @ g0n if self.state == INVERTED else g0n
        theta, resid = pitch_only_correction(g1, expect)
        if quality == "none":
            theta = 0.0
        self.delta_pitch = theta
        self.R_corr = rot_y(theta) if self.enabled else np.eye(3)
        self.last_event = RecalibEvent(
            t=t, state=self.state, delta_pitch_rad=theta, residual_rad=resid, quality=quality,
            window_std_acc=std_acc, window_mean_gyro=mean_gyro,
            g0=tuple(float(v) for v in self.g0), g1=tuple(float(v) for v in g1), s_at_flip=s)
        self.n_events += 1
        self._pending_since = None
        self._best = None
        return self.last_event

    # ── 공개 ──
    def step(self, t: float, acc, gyro) -> tuple[np.ndarray, RecalibEvent | None]:
        """한 샘플. (R_corr, 이번 샘플에서 난 이벤트 또는 None)."""
        acc = np.asarray(acc, float)
        gyro = np.asarray(gyro, float)
        self.n += 1
        self._buf.append((t, acc, gyro))
        while self._buf and (t - self._buf[0][0]) > self.static_s:
            self._buf.popleft()

        # 1. 기준 중력: 첫 정지 창
        if self.g0 is None:
            if self._window_full(t):
                g_mean, std_acc, mean_gyro = self._window_stats()
                if self._is_static(std_acc, mean_gyro) and np.linalg.norm(g_mean) > 5.0:
                    self.g0 = g_mean
                    self.state = UPRIGHT
                    self._cand_state = UPRIGHT
                    self._cand_since = t
            return self.R_corr, None

        # 2. 전복 감지 (히스테리시스)
        s = float(acc @ self.g0) / float(self.g0 @ self.g0)
        if s < -self.flip_ratio:
            cand = INVERTED
        elif s > self.flip_ratio:
            cand = UPRIGHT
        else:
            cand = UNKNOWN
        if cand != self._cand_state:
            self._cand_state, self._cand_since = cand, t
        if (cand != UNKNOWN and cand != self.state
                and self._cand_since is not None and (t - self._cand_since) >= self.hold_s):
            self.state = cand
            self._pending_since = t
            self._pending_s = s
            self._best = None

        # 3. 상태 변경 뒤 정지 창 대기 → 재보정
        event = None
        if self._pending_since is not None and self._window_full(t):
            g_mean, std_acc, mean_gyro = self._window_stats()
            # 창이 이전 상태의 샘플을 물고 있지 않도록: 창 시작이 상태 변경 이후여야 한다.
            if self._buf[0][0] >= self._pending_since:
                score = std_acc + 3.0 * mean_gyro       # 3 rad/s ≈ 1 m/s² 로 환산해 합산
                if self._best is None or score < self._best[0]:
                    self._best = (score, t, g_mean, std_acc, mean_gyro)
                if self._is_static(std_acc, mean_gyro):
                    event = self._apply(t, g_mean, std_acc, mean_gyro, "strict", self._pending_s)
                elif (t - self._pending_since) >= self.max_wait_s:
                    _, _, bg, bstd, bgyro = self._best
                    quality = "relaxed" if bstd <= self.relaxed_acc_std_max else "none"
                    event = self._apply(t, bg, bstd, bgyro, quality, self._pending_s)
        return self.R_corr, event

    def status(self) -> str:
        pend = "" if self._pending_since is None else " (정지 창 대기)"
        return (f"state={self.state}{pend} dpitch={math.degrees(self.delta_pitch):+.2f}deg "
                f"events={self.n_events} n={self.n}")
