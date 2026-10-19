"""stanley_core.py — DARON 경로추종 제어 코어 (ROS 비의존)

CLAUDE.md §6 참조 구현. 바뀐 점은 kappa_to_delta 에 wheelbase_m 인자를 더한 것 하나다
(기존 kinematics 의 tan(Delta_v) = L * kappa, L 은 굴절각에 따른 휠베이스).

부호: kappa > 0 좌회전 / e_y > 0 로버가 경로 왼쪽 / e_th = psi - path_heading.
"""
import math
from dataclasses import dataclass
from typing import Optional


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class Params:
    k_e: float = 1.5          # [1/m]   주 게인
    k_th_ratio: float = 4.0   # k_th = ratio * k_e (4 -> 임계감쇠)
    k_i: float = 0.0          # [1/m^3] 적분 (기본 꺼짐)
    I_max: float = 0.5        # [1/m]
    e_I_max: float = 0.10     # [m]
    kappa_max: float = 1.0986  # [1/m]
    kappa_rate: float = 5.5   # [1/(m*s)]
    gain_scale: float = 1.0   # Fuzzy GS 자리 — 1.0 고정
    understeer_gain: float = 1.0  # [261015] 조향 지령 곡률 / 실제 곡률 (= 1 / 회전 비율). 1 = 보정 안 함


class StanleyCurvature:
    """부호: kappa>0 좌회전 / e_y>0 로버가 경로 왼쪽 / e_th = psi - path_heading."""

    def __init__(self, p: Params):
        self.p = p
        self.reset()

    def reset(self) -> None:
        """빈 경로, enable off, 정지 후 재출발에서 호출. 경로 교체 시에는 호출하지 않는다."""
        self.kappa_prev = 0.0
        self.I = 0.0

    def reset_integrator(self) -> None:
        self.I = 0.0

    def step(self, e_y: float, e_th: float, kappa_ref: float, v_meas: float, dt: float):
        p = self.p
        k_e = p.k_e * p.gain_scale
        k_th = p.k_th_ratio * k_e

        th_d = -math.atan(k_e * e_y)                     # 목표 접근각 (Stanley arctan 항)
        kappa_fb = k_th * wrap(th_d - e_th)
        kappa_raw = kappa_ref + kappa_fb - self.I

        # [261015] 언더스티어 보정: 여기 kappa 는 '실제로 돌 곡률'. 조향은 understeer_gain 배로 내므로
        # 실제로 낼 수 있는 곡률 한계·변화율은 그만큼 작다 (kappa_to_delta 에서 다시 곱한다)
        g = understeer_gain(p)
        kappa_max, kappa_rate = p.kappa_max / g, p.kappa_rate / g
        kappa_sat = max(-kappa_max, min(kappa_max, kappa_raw))
        saturated = kappa_sat != kappa_raw

        if p.k_i > 0.0 and not saturated and abs(e_y) < p.e_I_max:   # 조건부 적분 (호 길이 기준)
            self.I = max(-p.I_max, min(p.I_max, self.I + p.k_i * e_y * v_meas * dt))

        step_max = kappa_rate * dt                                  # rate 제한
        self.kappa_prev += max(-step_max, min(step_max, kappa_sat - self.kappa_prev))

        return self.kappa_prev, {
            "kappa_fb": kappa_fb, "kappa_raw": kappa_raw,
            "saturated": saturated, "I": self.I,
        }


def understeer_gain(p) -> float:
    """[261015] 잘못된 값(0 이하·비유한)이면 1 (보정 안 함)."""
    g = float(getattr(p, "understeer_gain", 1.0))
    return g if math.isfinite(g) and g > 0.0 else 1.0


def kappa_to_delta(kappa: float, kappa_max: float = 1.0986, delta_max_deg: float = 20.0,
                   wheelbase_m: Optional[float] = None) -> float:
    """kappa [1/m] -> 차량 유효조향각 delta [rad].

    wheelbase_m 을 주면 기존 kinematics(l12_driver: tan(Delta_v) = L * kappa)를 따른다.
    없으면 L_eff = tan(delta_max) / kappa_max (CLAUDE.md §4.5) 를 쓴다.
    """
    delta_max = math.radians(delta_max_deg)
    L = wheelbase_m if wheelbase_m and wheelbase_m > 0 else math.tan(delta_max) / kappa_max
    return max(-delta_max, min(delta_max, math.atan(L * kappa)))


def speed_command(e_y, kappa_ref, saturated, v_nom=0.15, v_min=0.05, e1=0.05, e2=0.15,
                  kappa_max=1.0986, curv_slow_ratio=0.8, curv_slow_factor=0.6):
    a = abs(e_y)
    if a <= e1:
        f_cte = 1.0
    elif a >= e2:
        f_cte = v_min / v_nom
    else:
        f_cte = 1.0 - (1.0 - v_min / v_nom) * (a - e1) / (e2 - e1)
    f_curv = curv_slow_factor if (abs(kappa_ref) > curv_slow_ratio * kappa_max or saturated) else 1.0
    return max(0.0, v_nom * min(f_cte, f_curv))   # 속도 rate 제한(a_max)은 노드에서 적용
