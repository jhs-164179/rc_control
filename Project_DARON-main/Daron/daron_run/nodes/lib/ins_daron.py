"""INS 단독 이중적분 — `~/analysis_ins_only.py`(DARON, 논문 INS 선) 의 실시간 이식.

ROS 의존 없음. 노드는 샘플마다 :meth:`DaronReckoner.step` 을 부르고 ``p``/``R`` 을 발행한다.

원본(오프라인)과 **같은 데이터면 같은 결과**가 나오도록 계산 순서를 그대로 옮겼다:

1. 정지 구간(``static_s``, 기본 15 s) 샘플 평균으로 자이로 bias ``bg`` 와 body 중력 ``g_body``.
2. 초기 자세 R0: body 의 중력 방향을 world +Z 에 맞춘다 (yaw 는 관측 불가 → 0).
3. 자세는 **드라이버 쿼터니언을 쓰지 않고** bias 뺀 자이로를 직접 적분한다 (body 증분, 우측 곱).
4. world 가속도 ``aw = R·acc − [0,0,|g|]`` — |g| 는 상수가 아니라 정지 구간 실측값.
5. 정지 구간 ``aw`` 평균을 잔여 가속도 bias ``ba_w`` 로 빼고, 속도·위치를 **사다리꼴**로 이중적분.
6. ZUPT 없음. 정지해도 속도를 0 으로 끊지 않는다 — 오차는 t² 로 단조 발산한다(그게 논문 기준).

원본은 bag 전체를 읽고 나서 계산하지만 정지 구간이 맨 앞이라 인과적이다: 정지 구간 동안은
샘플을 모아 두고, 창이 차는 순간 1·2·5 를 계산한 뒤 모아 둔 샘플을 같은 식으로 되감아 적분하고,
그 뒤로는 샘플이 올 때마다 3~5 를 한 번씩 한다. 되감기 때문에 정지 구간 끝의 위치가 원본의
``[SELF-CHECK] 정지 구간 끝 INS 위치`` 와 같은 값이 된다.

원본과 다른 점은 ``max_dt`` 하나 — 샘플 간격이 이보다 크면(토픽 끊김) 그 구간은 적분하지 않는다.
원본은 그런 구간도 그대로 적분한다. 끊김이 없는 bag 에서는 동일하다.
"""
from __future__ import annotations

import math

import numpy as np


def _rodrigues(w: np.ndarray, dt: float) -> np.ndarray:
    """회전벡터 w·dt 의 회전행렬 (원본의 axis-angle 쿼터니언 증분과 같은 양)."""
    ang = float(np.linalg.norm(w)) * dt
    if ang <= 1e-12:
        return np.eye(3)
    k = w / np.linalg.norm(w)
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(ang) * K + (1.0 - math.cos(ang)) * (K @ K)


def initial_attitude(g_body: np.ndarray) -> np.ndarray:
    """body 중력 벡터로 world←body 회전행렬. 원본 analysis_ins_only.py 의 R0 와 같은 구성."""
    zb = g_body / np.linalg.norm(g_body)
    tmp = np.array([1.0, 0.0, 0.0])
    if abs(float(np.dot(tmp, zb))) > 0.9:
        tmp = np.array([0.0, 1.0, 0.0])
    xb = tmp - np.dot(tmp, zb) * zb
    xb /= np.linalg.norm(xb)
    yb = np.cross(zb, xb)
    return np.column_stack([xb, yb, zb]).T      # world <- body


class DaronReckoner:
    """자이로 적분 자세 + 중력 제거 이중적분, ZUPT 없음.

    Parameters
    ----------
    static_s
        기동 후 정지로 간주하는 시간 [s]. 원본 ``--static-sec`` (기본 15).
    max_dt
        이보다 긴 샘플 간격은 적분하지 않는다 [s].
    min_static_samples
        정지 구간 샘플이 이보다 적으면 캘리브레이션을 거부한다 (원본 200).
    """

    def __init__(self, static_s: float = 15.0, max_dt: float = 0.1, min_static_samples: int = 200):
        self.static_s = float(static_s)
        self.max_dt = float(max_dt)
        self.min_static_samples = int(min_static_samples)
        self.reset()

    def reset(self) -> None:
        self.t0 = None
        self.calibrated = False
        self._buf: list[tuple[float, np.ndarray, np.ndarray]] = []   # 정지 구간 (t, gyro, acc)
        self.bg = np.zeros(3)          # 자이로 bias [rad/s]
        self.g_mag = float("nan")      # |g| 실측 [m/s²]
        self.ba_w = np.zeros(3)        # 잔여 가속도 bias (world) [m/s²]
        self.R = np.eye(3)             # world <- body
        self.v = np.zeros(3)
        self.p = np.zeros(3)
        self._last_t = None
        self._last_aw = None           # 사다리꼴용 직전 aw_c
        self.n = 0
        self.static_end_error = float("nan")   # 정지 구간 끝 |p| (원본 SELF-CHECK)
        # DeadReckoner(ZUPT) 와 같은 이름의 필드 — 노드가 두 적분기를 같은 코드로 다루기 위해.
        self.stationary = False
        self.stop_error = None
        self.last_stop_error = float("nan")
        self.zupt_n = 0

    # ── 내부 ──────────────────────────────────────────────────────
    def _integrate(self, t: float, gyro: np.ndarray, acc: np.ndarray) -> None:
        """3~5: 자세 전파 → world 가속도 → 사다리꼴 이중적분. 캘리브레이션 뒤에만 부른다."""
        dt = 0.0 if self._last_t is None else t - self._last_t
        self._last_t = t
        if dt <= 0.0 or dt > self.max_dt:
            dt = 0.0
        if dt > 0.0:
            self.R = self.R @ _rodrigues(gyro - self.bg, dt)
        aw_c = self.R @ acc - np.array([0.0, 0.0, self.g_mag]) - self.ba_w
        if dt > 0.0 and self._last_aw is not None:
            v_prev = self.v.copy()
            self.v = self.v + 0.5 * (aw_c + self._last_aw) * dt
            self.p = self.p + 0.5 * (self.v + v_prev) * dt
        self._last_aw = aw_c

    def _calibrate(self) -> None:
        """1·2·5 를 모아 둔 정지 샘플로 계산하고, 같은 샘플을 되감아 적분한다."""
        ts = np.array([b[0] for b in self._buf])
        gyr = np.array([b[1] for b in self._buf])
        acc = np.array([b[2] for b in self._buf])
        self.bg = gyr.mean(0)
        g_body = acc.mean(0)
        self.g_mag = float(np.linalg.norm(g_body))
        R0 = initial_attitude(g_body)
        # 원본 1차 통과: R0 에서 자이로로 자세를 전파하며 정지 구간 world 가속도를 구해 그 평균이 ba_w
        R, aw = R0.copy(), np.zeros((len(ts), 3))
        for i in range(len(ts)):
            if i > 0:
                dt = ts[i] - ts[i - 1]
                if 0.0 < dt <= self.max_dt:
                    R = R @ _rodrigues(gyr[i] - self.bg, dt)
            aw[i] = R @ acc[i] - np.array([0.0, 0.0, self.g_mag])
        self.ba_w = aw.mean(0)
        # 되감기: 정지 구간도 원본처럼 t=0 부터 적분한다 (자세는 R0 부터 다시)
        self.R, self.v, self.p = R0.copy(), np.zeros(3), np.zeros(3)
        self._last_t, self._last_aw = None, None
        for t, g, a in self._buf:
            self._integrate(t, g, a)
        self.static_end_error = float(np.linalg.norm(self.p))
        self.calibrated = True
        self._buf = []

    # ── 공개 ──────────────────────────────────────────────────────
    def step(self, t: float, gyro, acc):
        """한 샘플. ``(R, tracked)`` — tracked=False 는 아직 정지 캘리브레이션 중."""
        gyro, acc = np.asarray(gyro, float), np.asarray(acc, float)
        self.n += 1
        if self.t0 is None:
            self.t0 = t
        if not self.calibrated:
            # 원본과 같이 정지 구간은 `t - t0 < static_s` 인 샘플 **만** (경계 샘플은 주행 구간).
            if t - self.t0 < self.static_s:
                self._buf.append((t, gyro, acc))
                return self.R, False
            if len(self._buf) < self.min_static_samples:
                # 원본은 여기서 [FATAL] 로 종료한다. 실시간에서는 정지 구간을 늘려 계속 모은다.
                self._buf.append((t, gyro, acc))
                return self.R, False
            self._calibrate()
        self._integrate(t, gyro, acc)
        return self.R, True

    def calib_summary(self) -> str:
        return (f"static={self.static_s:.0f}s bias_gyro=({np.degrees(self.bg[0]):+.3f},{np.degrees(self.bg[1]):+.3f},"
                f"{np.degrees(self.bg[2]):+.3f})°/s |g|={self.g_mag:.4f} "
                f"ba_w=({self.ba_w[0]:+.4f},{self.ba_w[1]:+.4f},{self.ba_w[2]:+.4f}) "
                f"정지끝|p|={self.static_end_error:.3f}m")

    def status(self, name: str = "") -> str:
        phase = "INTEGRATING" if self.calibrated else f"CALIB({self.static_s:.0f}s 정지)"
        return (f"{name}: n={self.n} {phase} zupt=none |v|={float(np.linalg.norm(self.v)):.2f}m/s "
                f"p=({self.p[0]:+.2f},{self.p[1]:+.2f},{self.p[2]:+.2f})")
