"""SLAM / INS health 판정 — ROS 의존 없음 (numpy 만).

노드(``health_monitor.py``, 예정)는 메시지마다 ``imu_sample`` / ``slam_sample`` 을, 주기적으로
``evaluate(now)`` 를 부르고 돌려받은 :class:`HealthReport` 를 발행한다. 여기에는 판정·상태·계산만 있다.
오프라인 검증: ``tools/health_offline.py`` (bag → 같은 코드 → 타임라인·오경보 집계).

[파이프라인] 센서별 자기진단 → 교차 일관성(둘 다 자기진단 통과 시에만) → 센서별 health(히스테리시스)
  → cov_scale(h) 로 EKF 입력 공분산 배율. FAIL 이면 None = 메시지를 버린다(260929 결정).

[INS 자기진단] 스트림이 1순위 (260929 real_output: CV7 스트림이 143 s 부터 끊기고 몰려오다 151.6 s 에 정지)
  - stamp 간격 > gap_warn_s → DEGRADED, > gap_fail_s → FAIL
  - 최근 burst_win_s 창 샘플 수 > burst_ratio × 기대치 → DEGRADED (몰아서 도착 = stamp 압축, 적분 불가)
  - 마지막 샘플 뒤 dead_s 동안 무수신 → FAIL (수신 시각 rx 와 evaluate 의 now — 같은 지상국 시계)
  - NaN / 가속도 포화 → FAIL
  - AHRS 자세 변화 vs bias 뺀 자이로 적분 (ahrs_win_s 창, IMU 자체 좌표계라 외부 캘리브 불필요) → DEGRADED
  - 자이로 bias: 기본(gyro_bias_mode='none')은 빼지 않는다 — 입력 /ekf/imu/data 는 CV7 필터가 이미
    bias 를 보정한 값이다. 기동 평균을 한 번 더 빼면 오히려 틀린다: viz2/sup bag 은 처음 15 s 가 정지가
    아니라(max|ω| 9~33°/s) 평균에 움직임이 섞여 10 s 창 잔차가 2~3.8° 로 부풀었다(빼지 않으면 ≤1.9°).
    원시 /imu/data 를 쓸 때만 'startup' — 기동 뒤 첫 bias_s 평균. 주행 중 정지 판정은 어느 쪽도 쓰지
    않는다(아주 느린 이동을 정지로 오판할 수 있음 — ZUPT 제외와 같은 이유, 260929 결정).

[SLAM 자기진단] LOST(pose 공분산 ≥ lost_var) → FAIL, 프레임 간 점프, features(있을 때만).
  프레임 공백은 품질로 보지 않는다(영상 끊김). slam_gap_fail_s 넘게 안 들어오면 그동안만 video_cut FAIL (261015).

[시계] 이벤트는 기록 즉시 수준에 반영(escalate)하고 evaluate 가 꺼내 쓴다 — 센서 stamp 를 판정 시계 now 와 빼지 않는다.
  생존(stall/dead/video_cut)은 수신 시각 rx 와 now, 회전 교차검사 창은 두 센서 stamp 의 min 에서 끝난다 (261015).
  첫 evaluate 뒤 first_sample_s 동안 샘플이 한 번도 없으면 never FAIL. 회전 창 끝점 SLAM 샘플이 rot_sample_age_s 보다
  낡았으면(끊김 직후) 그 창은 판정하지 않는다 (261016).

[교차 일관성] 자이로 적분 회전 vs SLAM 상대회전, 1 s 창(급격한 이상) + 10 s 창(미세 드리프트).
  창 안에 IMU 공백/burst 나 SLAM LOST 가 끼면 그 창은 판정하지 않는다(누구 잘못인지 모르므로).
  최근 persist_n 창 중 persist_k 개가 임계를 넘을 때만 위반. R_body_imu 와 time_offset_s 는
  외부에서 준다(노드: TF / 오프라인: :func:`estimate_extrinsic`).

[판정] 자기진단 FAIL 센서는 교차검사에서 빠진다. 둘 다 통과인데 회전 불일치 → SLAM 회전 DEGRADED
  (짧은 구간은 자이로를 더 믿는다). 불일치가 both_after_s 넘게 이어지면 INS 도 DEGRADED.
  health: OK=1.0, DEGRADED=0.5, FAIL=0.0. 나빠질 때는 즉시, 좋아질 때는 recover_s 연속 유지 후.

[시간] 판정 입력(이벤트·회전 창)은 header stamp 기준 초(float), 생존 판정만 수신 시각 rx 기준 — 위 [시계].
[임계치] 기본값은 전부 잠정값 — 정상 bag 분포로 다시 정한다 (tools/health_offline.py 가 분포를 출력).
"""
from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass, field

import numpy as np

OK, DEGRADED, FAIL = 0, 1, 2
LEVEL_NAME = {OK: "OK", DEGRADED: "DEGRADED", FAIL: "FAIL"}
LEVEL_HEALTH = {OK: 1.0, DEGRADED: 0.5, FAIL: 0.0}


# ---------------------------------------------------------------- SO(3) 헬퍼 (scipy 없이)
def so3_exp(v: np.ndarray) -> np.ndarray:
    """회전벡터 → 회전행렬 (Rodrigues)."""
    th = float(np.linalg.norm(v))
    if th < 1e-12:
        return np.eye(3)
    k = v / th
    K = np.array([[0.0, -k[2], k[1]], [k[2], 0.0, -k[0]], [-k[1], k[0], 0.0]])
    return np.eye(3) + math.sin(th) * K + (1.0 - math.cos(th)) * (K @ K)


def so3_log(R: np.ndarray) -> np.ndarray:
    """회전행렬 → 회전벡터. 180° 근처는 대각에서 축을 뽑는다."""
    c = max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))
    th = math.acos(c)
    if th < 1e-9:
        return np.zeros(3)
    if math.pi - th < 1e-4:
        # R + I = 2·k·kᵀ (θ≈π) → 대각이 가장 큰 열이 축에 비례. sin θ≈0 이라 아래 식은 불안정.
        i = int(np.argmax(np.diag(R)))
        col = R[:, i] + np.eye(3)[:, i]
        axis = col / np.linalg.norm(col)
        # 부호: 반대칭부(sin θ·K)가 남아 있으면 그 방향으로 맞춘다
        w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
        if float(w @ axis) < 0:
            axis = -axis
        return axis * th
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return w * (th / (2.0 * math.sin(th)))


def rot_angle_deg(R: np.ndarray) -> float:
    return math.degrees(math.acos(max(-1.0, min(1.0, (np.trace(R) - 1.0) / 2.0))))


def quat_to_mat(q) -> np.ndarray:
    """(x, y, z, w) → 3x3. lib/rotations.quat_to_mat 과 같은 규약 (인자만 시퀀스 하나)."""
    x, y, z, w = (float(v) for v in q)
    n = x * x + y * y + z * z + w * w
    if n < 1e-12:
        return np.eye(3)
    s = 2.0 / n
    return np.array([
        [1.0 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y)],
    ])


# ---------------------------------------------------------------- 파라미터
@dataclass
class HealthParams:
    # INS 스트림
    imu_nominal_dt: float = 0.01        # CV7 100 Hz
    gap_warn_s: float = 0.03            # 3 × 주기
    gap_fail_s: float = 0.2             # = ekf.yaml sensor_timeout
    dead_s: float = 0.5                 # 무수신 → FAIL
    burst_win_s: float = 0.2
    burst_ratio: float = 1.5            # 0.2 s 창에 20개 기대 → 30개 초과면 몰림
    acc_sat: float = 150.0              # |a| [m/s²] 이상이면 포화로 본다 (CV7 ±8 g ≈ 78 → 여유)
    # INS 내부 (AHRS vs 자이로)
    gyro_bias_mode: str = "none"        # 'none' (/ekf/imu/data, CV7 보정됨) | 'startup' (/imu/data 원시)
    bias_s: float = 15.0                # 'startup' 일 때 기동 뒤 정지 창
    ahrs_win_s: float = 1.0
    ahrs_warn_deg: float = 3.0          # 잠정
    # SLAM
    lost_var: float = 1.0               # pose cov[0] ≥ 이 값이면 LOST (TRACKED 0.01 / LOST 1e6)
    slam_gap_warn_s: float = 0.1        # 261015 부터 판정에 안 씀 (프레임 공백 = 영상 끊김, 품질 이벤트 아님)
    slam_gap_fail_s: float = 0.5        # 이만큼 SLAM 이 안 들어오면(수신 시각) video_cut — 끊긴 동안만 FAIL
    slam_step_warn_m: float = 0.05      # 프레임 간 점프. 정상 최대 8 mm (real_output)
    # 이 이상이면 FAIL — 로버(최고 ~7 cm/s)로는 불가능한 3 m/s 급. 260929 실시간 책상 테스트에서 cuVSLAM 이
    # LOST 없이 'OK' 로 2 s 사이 6 m 를 튀었고, DEGRADED(공분산 ×4)로만 두자 EKF-C 가 그대로 삼켜 발산했다.
    # 릴레이는 이 점프를 JumpRemover 로 스트림에서 지운다(differential 이라 메시지를 버리는 것만으론 안 됨).
    slam_step_fail_m: float = 0.10
    features_warn: int = 60             # 잠정. 정상 최소 183 (real_output 오프라인)
    slam_warmup_s: float = 3.0          # 첫 SLAM 메시지 뒤 이 시간은 gap/jump 를 보지 않는다
                                        # (정상 bag 11개 전부 기동 직후 gap 오경보 — 워밍업 프레임 간격)
    # 교차 (회전): 임계 = base_deg + ratio × (창 안 SLAM 회전량 deg).
    # 비례항 이유: 장착 회전 오차 δ 는 회전량에 비례하는 잔차를 만든다(bag 별 추정 피치가 -12~-16° 로 흔들림).
    # 260929 정상 bag 11개(+real_output, rollover) 분포로 정함:
    #   1 s: 회전 <5° 에서 p99 1.07°, 5~20° 에서 max 2.87° → 2.0 + 0.15·rot
    #   10 s: 회전량과 상관 없음(corr≈0), 정상 max 1.9° → 3.0 (real_output 3.9°, rollover 전복 4.7° 는
    #         자이로·AHRS 가 서로 0.4~1° 로 맞고 SLAM 만 다른 창 — SLAM 쪽 실제 이탈 후보)
    #   fail: 한 창이라도 fail_base + ratio·rot 를 넘으면 지속 확인 없이 SLAM FAIL (260929 책상 테스트 10 s 잔차 49°).
    #         정상 최대(1 s 2.5°, 전복 80° 회전 창 6°, 10 s 4.4°)와 거리를 둔 값.
    rot_windows: tuple = ((1.0, 0.5, 2.0, 0.15, 10.0), (10.0, 1.0, 3.0, 0.0, 15.0))   # (창 s, 간격 s, base, ratio, fail_base)
    persist_n: int = 3
    persist_k: int = 2
    both_after_s: float = 10.0          # 불일치가 이만큼 이어지면 INS 도 DEGRADED
    # [261016] 창 끝점의 SLAM 샘플이 이보다 오래면 그 창은 판정하지 않는다. 영상이 끊겼다 돌아온 직후 창 시작점이
    # 끊김 이전 샘플로 잡히면 SLAM 은 짧은 구간, 자이로는 창 전체 회전을 재서 회전 중 rot_fail 오경보가 났다(시뮬 14°/s).
    rot_sample_age_s: float = 0.2
    # [261016] evaluate 를 시작하고 이만큼 지나도 첫 샘플이 없으면 FAIL('never'). 예전엔 한 번도 안 오면 영원히 OK —
    # D405 가 USB 2.1 로 붙어 infra2 가 안 열리면 SLAM 0 Hz 인데 health 는 OK 였다.
    first_sample_s: float = 15.0
    # 히스테리시스
    recover_s: float = 2.0
    # 버퍼
    keep_s: float = 12.0


# ---------------------------------------------------------------- 결과
@dataclass
class HealthReport:
    t: float
    ins_level: int
    slam_level: int
    ins_health: float
    slam_health: float
    ins_reasons: list = field(default_factory=list)
    slam_reasons: list = field(default_factory=list)
    rot_resid_deg: dict = field(default_factory=dict)   # {창 s: 마지막 잔차} (판정 안 된 창은 없음)

    def summary(self) -> str:
        r = " ".join(f"{w:g}s={v:.2f}" for w, v in sorted(self.rot_resid_deg.items()))
        return (f"t={self.t:.2f} INS={LEVEL_NAME[self.ins_level]}({','.join(self.ins_reasons) or '-'}) "
                f"SLAM={LEVEL_NAME[self.slam_level]}({','.join(self.slam_reasons) or '-'}) rot[{r}]")


def cov_scale(h: float, h_min: float = 0.1) -> float | None:
    """EKF 입력 공분산 배율 = 1/h². FAIL(h≈0) 이면 None = 메시지를 넘기지 않는다."""
    if h <= 1e-6:
        return None
    return 1.0 / max(h, h_min) ** 2


# ---------------------------------------------------------------- 히스테리시스
class _Hysteresis:
    """나빠질 때는 즉시, 좋아질 때는 recover_s 동안 raw 가 계속 좋은 쪽이어야 내려온다."""

    def __init__(self, recover_s: float):
        self.recover_s = recover_s
        self.level = OK
        self._better_since = None

    def escalate(self, raw: int) -> None:
        """이벤트가 기록된 그 순간 나빠지는 쪽만 반영 (회복은 update 가 시계로 판단)."""
        if raw > self.level:
            self.level = raw
            self._better_since = None

    def update(self, t: float, raw: int) -> int:
        if raw >= self.level:
            self.level = raw
            self._better_since = None
        else:
            if self._better_since is None:
                self._better_since = t
            if t - self._better_since >= self.recover_s:
                self.level = raw
                self._better_since = None
        return self.level


# ---------------------------------------------------------------- 본체
class HealthMonitor:
    def __init__(self, params: HealthParams | None = None,
                 R_body_imu: np.ndarray | None = None, time_offset_s: float = 0.0):
        self.p = params or HealthParams()
        self.R_bi = np.eye(3) if R_body_imu is None else np.asarray(R_body_imu, float)
        self.time_offset_s = float(time_offset_s)
        self.cross_enabled = True                  # 노드: 장착 회전 추정 전에는 False (교차검사 보류)
        p = self.p
        # INS
        self._imu_t0 = None
        self._imu_last_t = None
        self._imu_last_rx = None                    # 수신 시각 (판정 시계) — 생존 판정용
        self._imu_stamps = deque()                  # burst 계산용 최근 stamp
        self._bias_acc = []
        self.gyro_bias = None
        self._gyro_R = np.eye(3)                    # IMU 좌표계 누적 자이로 자세
        self._imu_hist = deque()                    # (t, R_gyro_cum, R_ahrs|None, seg)
        self._seg = 0                               # 공백마다 증가 — 창이 두 seg 에 걸치면 무효
        self._imu_events = deque()                  # (t, level, reason) 최근 이벤트
        self._ahrs_next = None
        # SLAM
        self._slam_hist = deque()                   # (t, R, tracked)
        self._slam_last_t = None
        self._slam_last_rx = None
        self._slam_t0 = None
        self._slam_last_p = None
        self._slam_events = deque()
        # 교차
        self._rot_next = {w[0]: None for w in p.rot_windows}
        self._rot_flags = {w[0]: deque(maxlen=p.persist_n) for w in p.rot_windows}
        self._rot_last = {}
        self._rot_bad_since = None
        self.rot_log = {w[0]: [] for w in p.rot_windows}      # (t_end, resid_deg, slam_rot_deg) — 임계 산정용
        self.ahrs_log = []                                    # (t_end, resid_deg)
        # 출력 히스테리시스
        self._h_ins = _Hysteresis(p.recover_s)
        self._h_slam = _Hysteresis(p.recover_s)
        self._eval_t0 = None                        # 첫 evaluate 시각 — first_sample_s 기준 (261016)

    # ------------------------------------------------ 입력: IMU
    def imu_sample(self, t: float, gyro, acc, quat=None, rx: float | None = None) -> None:
        """t = header stamp [s]. gyro [rad/s], acc [m/s²] (IMU 좌표계), quat = AHRS (x,y,z,w) 또는 None.
        rx = 수신 시각 (evaluate 의 now 와 같은 시계). 없으면 t (오프라인: 지연 0)."""
        p = self.p
        t = float(t) + self.time_offset_s
        self._imu_last_rx = t if rx is None else float(rx)
        try:
            self._imu_sample(t, gyro, acc, quat)
        finally:
            self._escalate()

    def _imu_sample(self, t: float, gyro, acc, quat) -> None:
        p = self.p
        g = np.asarray(gyro, float)
        a = np.asarray(acc, float)
        if self._imu_t0 is None:
            self._imu_t0 = t
        if not (np.all(np.isfinite(g)) and np.all(np.isfinite(a))):
            self._event(self._imu_events, t, FAIL, "nan")
            return
        if np.linalg.norm(a) > p.acc_sat:
            self._event(self._imu_events, t, FAIL, "acc_sat")

        # 스트림: 간격
        new_seg = False
        if self._imu_last_t is not None:
            dt = t - self._imu_last_t
            if dt <= 0.0:
                self._event(self._imu_events, t, DEGRADED, "stamp_nonmono")
                return
            if dt > p.gap_fail_s:
                self._event(self._imu_events, t, FAIL, "gap")
                new_seg = True
            elif dt > p.gap_warn_s:
                self._event(self._imu_events, t, DEGRADED, "gap")
                new_seg = True
        # 스트림: 몰림
        self._imu_stamps.append(t)
        while self._imu_stamps and self._imu_stamps[0] < t - p.burst_win_s:
            self._imu_stamps.popleft()
        expect = p.burst_win_s / p.imu_nominal_dt
        if len(self._imu_stamps) > p.burst_ratio * expect:
            self._event(self._imu_events, t, DEGRADED, "burst")
            new_seg = True
        if new_seg:
            self._seg += 1

        # 자이로 bias: 기동 뒤 첫 bias_s
        if self.gyro_bias is None and p.gyro_bias_mode == "none":
            self.gyro_bias = np.zeros(3)
        if self.gyro_bias is None:
            if t - self._imu_t0 < p.bias_s:
                self._bias_acc.append(g)
            else:
                self.gyro_bias = np.mean(self._bias_acc, axis=0) if self._bias_acc else np.zeros(3)
        # 자이로 누적 (bias 확정 전에는 적분하지 않는다)
        if self.gyro_bias is not None and self._imu_last_t is not None and not new_seg:
            dt = t - self._imu_last_t
            self._gyro_R = self._gyro_R @ so3_exp((g - self.gyro_bias) * dt)
        R_ahrs = quat_to_mat(quat) if quat is not None else None
        if self.gyro_bias is not None:
            self._imu_hist.append((t, self._gyro_R.copy(), R_ahrs, self._seg))
            while self._imu_hist and self._imu_hist[0][0] < t - p.keep_s:
                self._imu_hist.popleft()
        self._imu_last_t = t
        self._check_ahrs(t)

    def _check_ahrs(self, t: float) -> None:
        p = self.p
        if self.gyro_bias is None or not self._imu_hist or self._imu_hist[-1][2] is None:
            return
        if self._ahrs_next is None:
            self._ahrs_next = t + p.ahrs_win_s
        if t < self._ahrs_next:
            return
        self._ahrs_next = t + p.ahrs_win_s / 2.0
        a = self._imu_at(t - p.ahrs_win_s)
        b = self._imu_hist[-1]
        if a is None or a[3] != b[3]:
            return                                   # 창 안에 공백/몰림 — 판정 안 함
        dG = a[1].T @ b[1]
        dA = a[2].T @ b[2]
        r = rot_angle_deg(dG.T @ dA)
        self.ahrs_log.append((t, r))
        if r > p.ahrs_warn_deg:
            self._event(self._imu_events, t, DEGRADED, "ahrs_vs_gyro")

    def _imu_at(self, t: float):
        """t 이하에서 가장 가까운 IMU 누적 샘플 (없으면 None)."""
        h = self._imu_hist
        if not h or h[0][0] > t:
            return None
        lo, hi = 0, len(h) - 1
        while lo < hi:
            mid = (lo + hi + 1) // 2
            if h[mid][0] <= t:
                lo = mid
            else:
                hi = mid - 1
        return h[lo]

    # ------------------------------------------------ 입력: SLAM
    def slam_sample(self, t: float, pos, quat, pose_var: float, features: int | None = None,
                    rx: float | None = None) -> None:
        """t = header stamp [s]. pos/quat = /slam/odometry pose (body = base_link), pose_var = cov[0].
        rx = 수신 시각 (evaluate 의 now 와 같은 시계). 없으면 t."""
        t = float(t)
        self._slam_last_rx = t if rx is None else float(rx)
        try:
            self._slam_sample(t, pos, quat, pose_var, features)
        finally:
            self._escalate()

    def _slam_sample(self, t: float, pos, quat, pose_var: float, features) -> None:
        p = self.p
        tracked = pose_var < p.lost_var
        pos = np.asarray(pos, float)
        if not tracked:
            self._event(self._slam_events, t, FAIL, "lost")
        if self._slam_t0 is None:
            self._slam_t0 = t
        warm = t - self._slam_t0 < p.slam_warmup_s
        if self._slam_last_t is not None and not warm:
            # 프레임 공백은 품질 이벤트로 보지 않는다 — RADXA 과열·WiFi 로 프레임이 빠지는 건 "영상이 끊긴 것"이지
            # SLAM 이 틀린 게 아니다(261015 실시간: 11 Hz 수신에서 gap DEGRADED 97%). 끊겨 있는 동안은 evaluate 가
            # 수신 시각으로 video_cut 을 내고, 다시 들어오면 바로 풀린다. 끊김 뒤 SLAM 이 실제로 틀어졌으면
            # 아래 jump / lost 가 잡는다.
            if tracked and self._slam_last_p is not None:
                step = float(np.linalg.norm(pos - self._slam_last_p))
                if step > p.slam_step_fail_m:
                    self._event(self._slam_events, t, FAIL, "jump")
                elif step > p.slam_step_warn_m:
                    self._event(self._slam_events, t, DEGRADED, "jump")
        if features is not None and features < p.features_warn:
            self._event(self._slam_events, t, DEGRADED, "features")
        self._slam_last_t = t
        self._slam_last_p = pos if tracked else self._slam_last_p
        self._slam_hist.append((t, quat_to_mat(quat), tracked))
        while self._slam_hist and self._slam_hist[0][0] < t - p.keep_s:
            self._slam_hist.popleft()

    def slam_status(self, t: float, features: int | None = None, relock_jump: float | None = None) -> None:
        """/slam/status (cuVSLAM 노드가 /slam/odometry 뒤에 내는 JSON) — 특징점 수·RELOCK.

        RELOCK 은 LOST(이미 FAIL) 에서 돌아온 순간이라 DEGRADED 로 표시하고 복귀는 히스테리시스가 맡는다.
        """
        p = self.p
        t = float(t)
        if self._slam_t0 is not None and t - self._slam_t0 < p.slam_warmup_s:
            return
        if features is not None and features < p.features_warn:
            self._event(self._slam_events, t, DEGRADED, "features")
        if relock_jump is not None:
            self._event(self._slam_events, t, DEGRADED, "relock")
        self._escalate()

    def _slam_at(self, t: float):
        h = self._slam_hist
        if not h or h[0][0] > t:
            return None
        best = None
        for s in reversed(h):
            if s[0] <= t:
                best = s
                break
        return best

    # ------------------------------------------------ 교차: 회전
    def _check_rotation(self, now: float, ins_raw: int, slam_raw: int) -> None:
        # 창 끝은 판정 시계(now)가 아니라 두 센서가 모두 도달한 센서 시각 — 둘은 같은 시계(RADXA)이고
        # 도착 지연이 달라(SLAM ~0.3 s, IMU ~0.03 s) now 로 자르면 끝점이 어긋난다.
        if self._imu_last_t is None or self._slam_last_t is None:
            return
        t_end = min(self._imu_last_t, self._slam_last_t)
        for w, hop, base, ratio, fail_base in self.p.rot_windows:
            if self._rot_next[w] is None:
                self._rot_next[w] = now + w
            if now < self._rot_next[w]:
                continue
            self._rot_next[w] = now + hop
            ok = self.cross_enabled and ins_raw == OK and slam_raw == OK
            res = self._rot_residual(t_end - w, t_end) if ok else None
            if res is None:
                continue                                  # 판정 불가 창 — 플래그 기록 안 함
            resid, rot = res
            self._rot_last[w] = resid
            self.rot_log[w].append((t_end, resid, rot))
            self._rot_flags[w].append(resid > base + ratio * rot)
            if resid > fail_base + ratio * rot:
                self._event(self._slam_events, t_end, FAIL, "rot_fail")

    def _rot_residual(self, t0: float, t1: float):
        s0, s1 = self._slam_at(t0), self._slam_at(t1)
        i0, i1 = self._imu_at(t0), self._imu_at(t1)
        if None in (s0, s1, i0, i1):
            return None
        if t0 - s0[0] > self.p.rot_sample_age_s or t1 - s1[0] > self.p.rot_sample_age_s:
            return None                                    # 끝점 SLAM 샘플이 낡음 (영상 끊김) — 판정 불가 창
        if not (s0[2] and s1[2]) or i0[3] != i1[3]:
            return None
        # 창 안에 LOST 가 끼었으면 무효
        if any((not s[2]) for s in self._slam_hist if t0 <= s[0] <= t1):
            return None
        dS = s0[1].T @ s1[1]                               # body(base) 상대회전
        dG_imu = i0[1].T @ i1[1]
        dG = self.R_bi @ dG_imu @ self.R_bi.T              # IMU → body
        return rot_angle_deg(dG.T @ dS), rot_angle_deg(dS)

    def _rot_violation(self) -> bool:
        k = self.p.persist_k
        return any(sum(f) >= k for f in self._rot_flags.values())

    # ------------------------------------------------ 이벤트·판정
    @staticmethod
    def _event(q: deque, t: float, level: int, reason: str) -> None:
        q.append((t, level, reason))

    @staticmethod
    def _peek(q: deque) -> int:
        return max((l for _, l, _ in q), default=OK)

    def _escalate(self) -> None:
        """새 이벤트를 그 자리에서 히스테리시스에 올린다 — 다음 evaluate(최대 1 주기)를 기다리지 않는다."""
        self._h_ins.escalate(self._peek(self._imu_events))
        self._h_slam.escalate(self._peek(self._slam_events))

    @staticmethod
    def _drain(q: deque):
        """지난 evaluate 뒤 쌓인 이벤트 중 가장 나쁜 수준과 사유. 꺼낸 이벤트는 지운다.
        시각으로 자르지 않는다 — 이벤트 stamp 는 센서 시계라 판정 시계(now)와 비교하면 도착 지연만큼 놓친다."""
        lvl, why = OK, []
        while q:
            _, l, r = q.popleft()
            if l > lvl:
                lvl = l
            if r not in why:
                why.append(r)
        return lvl, why

    def levels(self) -> tuple:
        """(ins_level, slam_level) 지금 값 — 이벤트 직후 escalate 가 반영된 값 (릴레이가 메시지마다 읽는다)."""
        return self._h_ins.level, self._h_slam.level

    def evaluate(self, now: float) -> HealthReport:
        """now = 현재 시각 [s] (노드: ROS 시계, 오프라인: 재생 시각). 주기 호출 (예: 10 Hz).
        이벤트는 지난 호출 뒤 쌓인 것을 꺼내 쓰고, 생존(stall/dead)은 수신 시각(rx)과 now 를 비교한다."""
        p = self.p
        if self._eval_t0 is None:
            self._eval_t0 = now
        never = now - self._eval_t0 > p.first_sample_s   # 이만큼 기다렸는데 첫 샘플이 없으면 미수신 FAIL (261016)
        ins_raw, ins_why = self._drain(self._imu_events)
        slam_raw, slam_why = self._drain(self._slam_events)
        if self._imu_last_rx is not None and now - self._imu_last_rx > p.gap_fail_s:
            # 공백은 다음 샘플이 와야 gap 이벤트가 되므로, 끊겨 있는 동안은 여기서 잡는다
            ins_raw, ins_why = FAIL, ins_why + ["dead" if now - self._imu_last_rx > p.dead_s else "stall"]
        elif self._imu_last_rx is None and never:
            ins_raw, ins_why = FAIL, ins_why + ["never"]
        # 영상 끊김: 히스테리시스에 넣지 않는다 — 끊긴 동안만 FAIL 로 알리고, 프레임이 다시 오면 바로 풀린다
        video_cut = self._slam_last_rx is not None and now - self._slam_last_rx > p.slam_gap_fail_s
        slam_never = self._slam_last_rx is None and never

        self._check_rotation(now, ins_raw, FAIL if video_cut else slam_raw)
        if ins_raw == OK and slam_raw == OK and self._rot_violation():
            slam_raw, slam_why = max(slam_raw, DEGRADED), slam_why + ["rot_vs_gyro"]
            if self._rot_bad_since is None:
                self._rot_bad_since = now
            if now - self._rot_bad_since >= p.both_after_s:
                ins_raw, ins_why = max(ins_raw, DEGRADED), ins_why + ["rot_vs_slam_persist"]
        else:
            self._rot_bad_since = None

        ins_lvl = self._h_ins.update(now, ins_raw)
        slam_lvl = self._h_slam.update(now, slam_raw)
        if video_cut or slam_never:
            slam_lvl, slam_why = FAIL, slam_why + ["video_cut" if video_cut else "never"]
        return HealthReport(now, ins_lvl, slam_lvl, LEVEL_HEALTH[ins_lvl], LEVEL_HEALTH[slam_lvl],
                            ins_why, slam_why, dict(self._rot_last))


# ---------------------------------------------------------------- SLAM 점프 제거 (릴레이용)
class JumpRemover:
    """SLAM 포즈 스트림에서 불가능한 점프를 지워 연속으로 만든다 (cuVSLAM RELOCK 재정렬과 같은 식).

    EKF 는 SLAM 을 differential 로 받으므로, 점프한 메시지를 버려도 다음 메시지와의 차이에 점프가 그대로 실린다.
    그래서 점프 순간에 보정 T_fix 를 갱신해 출력이 직전 출력에서 이어지게 한다:
        out = T_fix · raw,   점프 시  T_fix ← out_prev · raw⁻¹   (그 한 프레임의 움직임은 0 으로 취급)
    step_m 은 프레임 간 위치 변화 기준(HealthParams.slam_step_fail_m 과 같은 값).
    """

    def __init__(self, step_m: float):
        self.step_m = float(step_m)
        self.T_fix = np.eye(4)
        self._raw_prev = None
        self._out_prev = None
        self.count = 0
        self.total_m = 0.0

    def apply(self, T_raw: np.ndarray):
        """T_raw (4x4) → (T_out, 제거한 점프 크기[m] 또는 None)."""
        jumped = None
        if self._raw_prev is not None:
            step = float(np.linalg.norm(T_raw[:3, 3] - self._raw_prev[:3, 3]))
            if step > self.step_m:
                Ti = np.eye(4)
                Ti[:3, :3] = T_raw[:3, :3].T
                Ti[:3, 3] = -T_raw[:3, :3].T @ T_raw[:3, 3]
                self.T_fix = self._out_prev @ Ti
                self.count += 1
                self.total_m += step
                jumped = step
        T_out = self.T_fix @ T_raw
        self._raw_prev = T_raw
        self._out_prev = T_out
        return T_out, jumped


# ---------------------------------------------------------------- 외부 캘리브 추정 (오프라인/기동 시)
def estimate_extrinsic(slam_t, slam_R, imu_t, imu_w, offsets=np.arange(-0.1, 0.1001, 0.005),
                       smooth_slam=5, smooth_imu=15):
    """SLAM 각속도(body) 와 자이로(IMU) 를 맞춰 R_body_imu 와 시간 오프셋(IMU stamp 에 더할 값)을 추정.

    |ω| 상호상관으로 오프셋 → 직교 Procrustes 로 회전. 반환 (R_bi, offset_s, corr, resid_rms_deg_s).
    회전이 충분한 구간(주행)을 넣어야 한다. 정지 구간만이면 결과가 의미 없다.
    """
    slam_t = np.asarray(slam_t, float)
    imu_t = np.asarray(imu_t, float)
    imu_w = np.asarray(imu_w, float)
    dt = np.diff(slam_t)
    wc = np.array([so3_log(slam_R[k].T @ slam_R[k + 1]) / dt[k] if dt[k] > 0 else np.zeros(3)
                   for k in range(len(dt))])
    tc = (slam_t[:-1] + slam_t[1:]) / 2.0

    def sm(x, n):
        k = np.ones(n) / n
        return np.array([np.convolve(x[:, i], k, "same") for i in range(3)]).T

    wcs = sm(wc, smooth_slam)
    wis = sm(imu_w - imu_w[: max(1, len(imu_w) // 20)].mean(0), smooth_imu)
    m = (tc > imu_t[0] + 1.0) & (tc < imu_t[-1] - 1.0)

    def at(off):
        return np.array([np.interp(tc[m], imu_t + off, wis[:, i]) for i in range(3)]).T

    best = (0.0, -2.0)
    for off in offsets:
        c = np.corrcoef(np.linalg.norm(at(off), axis=1), np.linalg.norm(wcs[m], axis=1))[0, 1]
        if c > best[1]:
            best = (float(off), float(c))
    gi = at(best[0])
    H = gi.T @ wcs[m]
    U, _, Vt = np.linalg.svd(H)
    D = np.diag([1.0, 1.0, np.sign(np.linalg.det(Vt.T @ U.T))])
    R = Vt.T @ D @ U.T
    res = wcs[m] - gi @ R.T
    rms = math.degrees(math.sqrt(float((res ** 2).sum(1).mean())))
    return R, best[0], best[1], rms
