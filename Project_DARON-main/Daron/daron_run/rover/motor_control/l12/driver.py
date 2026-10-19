#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
l12_driver.py — Actuonix L12 리니어 액추에이터 한 쌍의 구동 + 조향 기구학

두 부분으로 나뉜다.
  [A] 조향 기구학 : 차량 유효조향각 + 굴절각 -> 전/후방 액추에이터 인출 길이 [mm]
                    (순수 계산. 하드웨어 없이 import 해서 검증 가능)
  [B] PWM 구동    : 인출 길이 [mm] -> sysfs PWM duty_cycle [ns]

핵심 API
--------
    from motor_control.l12.driver import L12Pair, articulation_state, max_effective_steer

    pair = L12Pair()                 # 기본 전방 pwm8, 후방 pwm9 (resolve_pwm_path)
    pair.initialize()                # 중립(16.0 mm)으로 초기화
    sol = pair.set_steering(steer_deg=+30.0, motor_angle_deg=30.0)
    pair.neutral()
    pair.close()

--------------------------------------------------------------------------
용어 (세 가지 조향각을 구분한다)
--------------------------------------------------------------------------
    Delta_v : 차량 유효조향각.  조종 지령. 전/후방 조향을 합친 결과로
              차량이 실제로 도는 각도 (전륜조향 등가 조향각).
    Delta_a : 축 유효조향각.    각 액슬의 지면 투영 조향각.
              전방 +Delta_a, 후방 -Delta_a (역상 대칭).
    delta   : 휠 조향각.        기울어진 조향축 둘레의 실제 회전각.

--------------------------------------------------------------------------
[A] 조향 기구학 — 계산 흐름
--------------------------------------------------------------------------
 1) 굴절각 alpha -> 휠베이스 L 과 전/후방 캐스터각
        L       = sqrt(x1^2 + x2^2 - 2*x1*x2*cos alpha)
        gamma_f = atan2(x2*sin alpha, x1 - x2*cos alpha)
        gamma_r = atan2(x1*sin alpha, x2 - x1*cos alpha)
    (|gamma_f| + |gamma_r| = pi - alpha 가 항등적으로 성립)

 2) 차량 유효조향각 Delta_v -> 축 유효조향각 Delta_a
    전후륜 조향 kinematic bicycle model 에서 기준점 궤적의 곡률은
        kappa = cos(beta) * (tan Delta_a_f - tan Delta_a_r) / L
    이다 (R. Rajamani, Vehicle Dynamics and Control, 2nd ed., Springer,
    2012, Sec. 2.2). 본 설계는 Delta_a_f = -Delta_a_r = Delta_a 이므로
    순간회전중심(ICR)이 휠베이스 중점의 법선 위에 놓이고, 중점에서
    beta = 0, kappa = 2 tan(Delta_a) / L 이 된다.
    Delta_v 를 '같은 휠베이스의 전륜조향 차량이 같은 곡률을 내는 조향각'
    (tan Delta_v = L * kappa) 으로 정의하면
        tan Delta_v = 2 tan Delta_a    <=>    Delta_a = atan( tan(Delta_v) / 2 )
    따라서 지령 Delta_v 는 회전반경과 1:1 로 대응한다.
        R_mid = L / tan(Delta_v)        (휠베이스 중점의 회전반경)
    단순 반분(Delta_a = Delta_v / 2)은 소각에서만 근사적으로 같고
    대각에서 곡률 관계가 어긋나므로 쓰지 않는다.

 3) 축 유효조향각 Delta_a -> 실제 휠 조향각 (조향축 둘레 회전각)
        tan Delta_a = cos gamma * tan delta   (기울어진 축 둘레 회전의 지면 투영)
        delta_f = +atan( tan Delta_a / cos gamma_f )
        delta_r = -atan( tan Delta_a / cos gamma_r )
    전/후방 캐스터각이 다르므로 |delta_f| != |delta_r| 이다.
    지면상에서 전후방이 크기 같고 방향 반대인 축 유효조향각을 갖도록 한 것이며,
    이때 순간회전중심은 휠베이스 중점의 법선상에 놓인다.

 4) 휠 조향각 -> 액추에이터 인출 길이 (오프셋 슬라이더-크랭크)
    조향축에 수직한 평면에서, 조향축을 원점, 액추에이터 축 방향을 x(인보드 +),
    그에 수직한 방향을 y 로 둔다.
        C : 너클 암 핀 (크랭크 끝단).  중립에서 C = (-u, e)
        B : 로드엔드 핀 (슬라이더).    중립에서 B = (l - u, e),  y = e 선상 이동
        e = 조향축 -> 액추에이터 축 수직거리
        u = 조향축 -> C 의 축방향 거리
        l = 커플러 길이 (= 이중 조인트 두 축 사이 거리)
    폐루프를 풀면
        g(delta) = e*(1 - cos delta) + u*sin delta          (커플러의 y 성분)
        x_B(delta) = -u*cos delta - e*sin delta + sqrt(l^2 - g^2)
    액추에이터 본체는 액슬에 고정되어 있으므로 인출 길이는 x_B 의 1차 함수이며
        E(delta) = K - x_B(delta),   K = 상수
    K 는 커플러 길이와 무관하다. 따라서 COUPLER_LENGTH 를 바꾸면 중립 인출
    E(0) = K + u - l 이 자동으로 갱신된다. K 는 실측 기준점 1개로 교정한다
    (CAD 실측: l = 8.475 mm 일 때 중립 인출 16.0 mm  ->  K = 11.9 mm).

 5) 전방과 후방 액슬은 동일 부품이며 전방 = 후방을 수직축 둘레로 180 deg
    회전시킨 것이다. 180 deg 회전은 reflection 이 아닌 proper rotation 이므로
    수직축 둘레의 회전 방향이 보존된다. 따라서 delta -> E 사상은 전/후방이
    완전히 동일한 함수이며, 위 E() 를 그대로 양쪽에 적용한다.

--------------------------------------------------------------------------
조향 한계
--------------------------------------------------------------------------
    최소 회전반경은 기계 구조로 정해진 축 유효조향각 한계로 결정된다.
        AXLE_STEER_LIMIT_DEG = 25.0                          (축 기준, 기계 한계)
        MAX_STEER_DEG = atan(2 tan 25 deg) = 43.003 deg      (차량 기준, 같은 R_min)
    실제 적용 한계는 축 기준으로 min(기계 한계, 행정 한계) 를 구한 뒤
    2) 의 식으로 차량 기준으로 환산한다. 현 설계 상수에서 행정 한계는
    축 기준 약 27.8 ~ 28.9 deg (theta = 0 ~ 45) 이므로 기계 한계가 지배한다.

--------------------------------------------------------------------------
굴절 상태와 전복
--------------------------------------------------------------------------
    XM430 각도 theta 와 굴절각 alpha 의 관계:  alpha = 180 + theta [deg]
        theta =   0  ->  alpha = 180   (평면 전개)
        theta = -30  ->  alpha = 150   (정상 굴절 주행. 굴절부가 위로 솟음)
        theta = +30  ->  alpha = 210   (반대로 굴절 = 전복 주행. 굴절부가 내려감)
    alpha 가 커질수록 굴절부가 내려오고, 더 키우면 앞바퀴가 들린다 (수납 쪽).
    (261010 실측으로 부호 정정. 이전 판은 alpha = 180 - theta 로 반대였다.)

    alpha > 180 은 반대 방향 굴절이다. _caster_angles 는
        gamma(360 - alpha) = -gamma(alpha)
    를 만족하고 하류에서 cos 만 쓰므로 캐스터각 '크기'는 |theta| 만으로 정해진다.
    즉 기구학에 넣을 굴절각은
        alpha_eff = 180 - |theta|
    이다. (theta = +45 -> alpha = 225 -> alpha_eff = 135, theta = -45 와 동일)

    전복 여부는 theta 의 부호와 '별개의' 상태 변수다. theta 는 링크 기하만
    결정하고, 로버가 뒤집혀 있는지는 조종자가 inverted 플래그로 알려준다.
    (theta 부호로 추론하면 같은 굴절 자세에서 전복 여부를 바꿀 수 없다.)

    전복 상태에서는 로버가 뒤집혀 있으므로 위에서 본 회전 방향이 반대다.
    같은 인출 조합이 반대 방향 선회를 만들기 때문에, 차량 유효조향각의 부호를
    뒤집어 넣어야 조종 감각이 유지된다.
        steer_sign = +1  (inverted = False)
        steer_sign = -1  (inverted = True)

--------------------------------------------------------------------------
부호 규약
--------------------------------------------------------------------------
    Delta_v > 0 : 좌회전 (조종자 기준).  전방 액추에이터 신장 + 후방 수축
    Delta_a     : Delta_v 와 같은 부호. 전방 +Delta_a, 후방 -Delta_a 로 적용
    delta > 0   : 위에서 봤을 때 반시계(좌회전) 방향 휠 회전
    E         : 액추에이터 인출 길이. 완전 수축 = 0, 완전 신장 = STROKE_MAX
    로드엔드 핀이 인보드(+x)로 이동하면 액추에이터는 수축한다.
"""

from __future__ import annotations

import math
from functools import lru_cache
import os
import re
import time
from typing import NamedTuple, Optional

__all__ = [
    "actuator_positions", "ActuatorPositions", "neutral_position",
    "articulation_state", "ArticulationState",
    "articulation_deg", "motor_angle_from_articulation",
    "max_effective_steer", "max_axle_steer",
    "axle_steer_deg", "vehicle_steer_deg", "wheelbase_mm",
    "solve_steering", "SteeringSolution", "steering_to_actuators",
    "L12Pair", "L12Error", "resolve_pwm_path", "pwm_block_address",
    "FRONT_PWM_PATH", "REAR_PWM_PATH",
    "STROKE_MAX", "DEFAULT_ARTICULATION", "MAX_MOTOR_ANGLE_DEG",
    "AXLE_STEER_LIMIT_DEG", "MAX_STEER_DEG", "DEFAULT_STEER_LIMIT_DEG",
]


# =========================================================================
# 설계 상수  (CAD 실측값)
# =========================================================================

COUPLER_LENGTH = 8.475     # l  [mm]  커플러 길이. 여기만 바꾸면 중립도 자동 갱신
PERP_OFFSET = 18.0         # e  [mm]  조향축 -> 액추에이터 축 수직거리
AXIAL_OFFSET = 12.575      # u  [mm]  조향축 -> 너클 암 핀 C 의 축방향 거리

LINK_REAR = 209.0          # x1 [mm]  굴절 관절 -> 후방 액슬 링크 길이
LINK_FRONT = 133.4         # x2 [mm]  굴절 관절 -> 전방 액슬 링크 길이

STROKE_MAX = 30.0          # [mm] 액추에이터 최대 인출 길이 (완전 수축 = 0)

# 인출 길이 교정 상수 K : E(delta) = K - x_B(delta)
# 기준점 (l = 8.475 mm 에서 중립 인출 16.0 mm) 으로부터 K = 16.0 + 8.475 - 12.575
STROKE_ORIGIN = 11.9       # K [mm]  커플러 길이와 무관한 상수

DEFAULT_STEER_LIMIT_DEG = 20.0  # Operational limit until bench measurement.
DEFAULT_ARTICULATION = 150.0   # [deg] 기본 굴절각 (굴절 주행 자세)
MAX_MOTOR_ANGLE_DEG = 45.0     # [deg] XM430 굴절 구동 한계 (+-)

# 축 유효조향각의 기계적 한계. 최소 회전반경을 결정한다. [deg]
AXLE_STEER_LIMIT_DEG = 25.0
# 위 한계와 같은 최소 회전반경을 주는 차량 유효조향각 한계 [deg]
#   atan(2 * tan 25 deg) = 43.003 deg
MAX_STEER_DEG = math.degrees(math.atan(2.0 * math.tan(math.radians(AXLE_STEER_LIMIT_DEG))))

_EPS = 1e-12


class ActuatorPositions(NamedTuple):
    """(front, rear) 액추에이터 인출 길이 [mm]. 일반 튜플처럼 언패킹 가능."""
    front: float
    rear: float


class ArticulationState(NamedTuple):
    """XM430 각도 + 전복 플래그로부터 유도한 굴절 상태."""
    alpha_eff_deg: float   # 기구학에 넣을 굴절각 (0 ~ 180). theta 부호와 무관
    alpha_raw_deg: float   # 실제 굴절각 (135 ~ 225). 표시/로그용
    steer_sign: float      # +1 정상, -1 전복
    inverted: bool         # 전복 여부 (외부에서 주어지는 독립 상태)


class SteeringSolution(NamedTuple):
    """조향 지령 하나를 푼 전체 결과. 로그에 그대로 남길 수 있다."""
    steer_cmd_deg: float      # 조종자가 준 차량 유효조향각 (clamp 전)
    steer_eff_deg: float      # clamp 후 실제 적용된 차량 유효조향각 (조종자 기준)
    steer_limit_deg: float    # 현재 굴절각에서 가능한 최대 차량 유효조향각
    axle_front_deg: float     # 전방 축 유효조향각 (지면 투영, 하드웨어 기준 부호)
    axle_rear_deg: float      # 후방 축 유효조향각 (= -axle_front_deg)
    axle_limit_deg: float     # 현재 굴절각에서 가능한 최대 축 유효조향각
    wheelbase_mm: float       # 현재 굴절각에서의 휠베이스 L
    curvature_per_m: float    # 휠베이스 중점 궤적의 곡률 [1/m] (조종자 기준 부호)
    alpha_eff_deg: float      # 기구학에 쓴 굴절각
    alpha_raw_deg: float      # 실제 굴절각
    inverted: bool
    gamma_front_deg: float    # 전방 캐스터각
    gamma_rear_deg: float     # 후방 캐스터각
    delta_front_deg: float    # 전방 휠 조향각 (조향축 둘레 실제 회전각)
    delta_rear_deg: float     # 후방 휠 조향각
    front_mm: float           # 전방 액추에이터 인출 길이 (지령값)
    rear_mm: float            # 후방 액추에이터 인출 길이 (지령값)


# =========================================================================
# [A] 조향 기구학 — 내부 계산
# =========================================================================

@lru_cache(maxsize=128)
def _caster_angles(alpha_rad: float) -> tuple[float, float]:
    """굴절각 -> (전방 캐스터각, 후방 캐스터각) 크기 [rad].

    하류에서 cos 만 사용하므로 부호(음의 캐스터)는 생략한다.
    atan2 형식이라 캐스터각이 90 deg 를 넘는 형상에서도 사분면이 유지된다.
    """
    x1, x2 = LINK_REAR, LINK_FRONT
    sa, ca = math.sin(alpha_rad), math.cos(alpha_rad)
    gamma_f = math.atan2(x2 * sa, x1 - x2 * ca)
    gamma_r = math.atan2(x1 * sa, x2 - x1 * ca)
    return gamma_f, gamma_r


@lru_cache(maxsize=128)
def _wheelbase(alpha_rad: float) -> float:
    """굴절각 -> 전/후방 액슬 사이 지면 거리 L [mm]."""
    x1, x2 = LINK_REAR, LINK_FRONT
    return math.sqrt(x1 * x1 + x2 * x2 - 2.0 * x1 * x2 * math.cos(alpha_rad))


def _axle_from_vehicle(vehicle_rad: float) -> float:
    """차량 유효조향각 -> 축 유효조향각 [rad].  tan Delta_a = tan Delta_v / 2.

    (-pi/2, pi/2) 에서 단조 증가하는 홀함수이므로 부호와 대소 관계가 보존된다.
    """
    return math.atan(0.5 * math.tan(vehicle_rad))


def _vehicle_from_axle(axle_rad: float) -> float:
    """축 유효조향각 -> 차량 유효조향각 [rad].  tan Delta_v = 2 tan Delta_a."""
    return math.atan(2.0 * math.tan(axle_rad))


def _wheel_steer(axle_rad: float, gamma_rad: float) -> float:
    """축 유효조향각 + 캐스터각 -> 조향축 둘레 휠 회전각 [rad]."""
    return math.atan(math.tan(axle_rad) / math.cos(gamma_rad))


def _extension(delta_rad: float) -> float:
    """휠 조향각 -> 액추에이터 인출 길이 [mm]."""
    e, u, l = PERP_OFFSET, AXIAL_OFFSET, COUPLER_LENGTH
    g = e * (1.0 - math.cos(delta_rad)) + u * math.sin(delta_rad)
    disc = l * l - g * g
    if disc < 0.0:
        raise ValueError(
            f"토글 한계 초과: 휠 조향각 {math.degrees(delta_rad):.2f} deg 에서 "
            f"커플러가 슬라이더 축에 수직이 됩니다 "
            f"(|g| = {abs(g):.3f} mm > l = {l:.3f} mm). "
            "커플러 길이를 늘리거나 조향각을 줄이십시오."
        )
    x_b = -u * math.cos(delta_rad) - e * math.sin(delta_rad) + math.sqrt(disc)
    return STROKE_ORIGIN - x_b


def _solve_axle(axle_rad: float, alpha_rad: float
                ) -> tuple[float, float, float, float, float, float]:
    """축 유효조향각(하드웨어 기준 부호) + 굴절각 -> 캐스터각, 휠 조향각, 인출 길이.

    반환: (gamma_f, gamma_r, delta_f, delta_r, front_mm, rear_mm)
    토글 한계를 넘으면 _extension 이 ValueError 를 낸다.
    """
    gamma_f, gamma_r = _caster_angles(alpha_rad)
    # 후방은 역상. 캐스터각이 서로 다르므로 휠 조향각의 크기도 서로 다르다.
    delta_f = +_wheel_steer(axle_rad, gamma_f)
    delta_r = -_wheel_steer(axle_rad, gamma_r)
    return (gamma_f, gamma_r, delta_f, delta_r,
            _extension(delta_f), _extension(delta_r))


# =========================================================================
# [A] 조향 기구학 — 공개 API
# =========================================================================

def neutral_position() -> float:
    """현재 COUPLER_LENGTH 에 대응하는 중립(직진) 인출 길이 [mm]."""
    return STROKE_ORIGIN + AXIAL_OFFSET - COUPLER_LENGTH


def axle_steer_deg(vehicle_steer_deg: float) -> float:
    """차량 유효조향각 [deg] -> 축 유효조향각 [deg].  Delta_a = atan(tan(Delta_v) / 2)."""
    return math.degrees(_axle_from_vehicle(math.radians(vehicle_steer_deg)))


def vehicle_steer_deg(axle_steer_deg: float) -> float:
    """축 유효조향각 [deg] -> 차량 유효조향각 [deg].  Delta_v = atan(2 tan(Delta_a))."""
    return math.degrees(_vehicle_from_axle(math.radians(axle_steer_deg)))


def wheelbase_mm(alpha_eff_deg: float) -> float:
    """굴절각 [deg] -> 휠베이스 L [mm]."""
    return _wheelbase(math.radians(alpha_eff_deg))


def actuator_positions(
    effective_steer_deg: float,
    articulation_deg: float = DEFAULT_ARTICULATION,
    *,
    check_limits: bool = True,
) -> ActuatorPositions:
    """차량 유효조향각과 굴절각 -> 전/후방 액추에이터 인출 길이.

    effective_steer_deg 는 차량 유효조향각 Delta_v 이다 (전/후방 조향을 합친
    결과로 차량이 도는 각도). 내부에서 축 유효조향각 Delta_a 로 환산한다.

    articulation_deg 는 0 ~ 180 범위여야 한다. 전복 상태(alpha > 180)는
    articulation_state() 로 alpha_eff 와 steer_sign 을 먼저 구해서 넣을 것.

    check_limits=True 면 기계 한계(AXLE_STEER_LIMIT_DEG)와 행정(0 ~ STROKE_MAX)을
    모두 검사하고, 벗어나면 ValueError 를 낸다. clamp 는 하지 않는다.
    """
    if not 0.0 < articulation_deg <= 180.0:
        raise ValueError(
            f"굴절각은 0 초과 180 이하여야 합니다 (입력: {articulation_deg}). "
            "전복 상태라면 articulation_state() 를 먼저 쓰십시오."
        )
    if not math.isfinite(effective_steer_deg) or abs(effective_steer_deg) >= 90.0:
        raise ValueError(
            f"차량 유효조향각의 크기는 90 deg 미만이어야 합니다 "
            f"(입력: {effective_steer_deg})."
        )

    alpha = math.radians(articulation_deg)
    axle = _axle_from_vehicle(math.radians(effective_steer_deg))

    if check_limits and abs(math.degrees(axle)) > AXLE_STEER_LIMIT_DEG + 1e-9:
        raise ValueError(
            f"축 유효조향각 {math.degrees(axle):.3f} deg 가 기계 한계 "
            f"+-{AXLE_STEER_LIMIT_DEG:.1f} deg 를 넘습니다 "
            f"(차량 유효조향각 {effective_steer_deg:.2f} deg, "
            f"한계 +-{MAX_STEER_DEG:.3f} deg)."
        )

    _gf, _gr, delta_f, delta_r, front, rear = _solve_axle(axle, alpha)

    if check_limits:
        for name, delta, pos in (("전방", delta_f, front), ("후방", delta_r, rear)):
            if not -_EPS <= pos <= STROKE_MAX + _EPS:
                raise ValueError(
                    f"{name} 액추에이터 인출 {pos:.3f} mm 가 행정 "
                    f"0 ~ {STROKE_MAX:.1f} mm 를 벗어납니다 "
                    f"(차량 유효조향각 {effective_steer_deg:.2f} deg, "
                    f"축 유효조향각 {math.degrees(axle):.2f} deg, "
                    f"휠 조향각 {math.degrees(delta):.2f} deg, "
                    f"중립 {neutral_position():.3f} mm)."
                )

    return ActuatorPositions(front, rear)


def articulation_deg(motor_angle_deg: float) -> float:
    """XM430 각도 theta (굴절 원점 기준) -> 굴절각 alpha = 180 + theta [deg]."""
    return 180.0 + float(motor_angle_deg)


def motor_angle_from_articulation(alpha_deg: float) -> float:
    """굴절각 alpha -> XM430 각도 theta = alpha - 180 [deg]. articulation_deg 의 역."""
    return float(alpha_deg) - 180.0


def articulation_state(motor_angle_deg: float,
                      inverted: bool = False) -> ArticulationState:
    """
    XM430 각도 + 전복 플래그 -> 굴절 상태.

        alpha_raw = 180 + theta          실제 굴절각 (135 ~ 225)
        alpha_eff = 180 - |theta|        기구학에 넣을 굴절각 (135 ~ 180)
        steer_sign = -1 if inverted else +1

    alpha_eff 가 |theta| 로만 정해지는 이유:
        gamma(360 - alpha) = -gamma(alpha) 이고 하류에서 cos 만 쓰므로
        캐스터각의 크기는 굴절 방향과 무관하게 |theta| 로 결정된다.

    전복 여부는 theta 의 부호와 무관한 독립 상태다. 조종자가 알려준다.
    """
    theta = float(motor_angle_deg)
    return ArticulationState(
        alpha_eff_deg=180.0 - abs(theta),
        alpha_raw_deg=articulation_deg(theta),
        steer_sign=-1.0 if inverted else 1.0,
        inverted=bool(inverted),
    )


@lru_cache(maxsize=128)
def max_axle_steer(alpha_eff_deg: float,
                   ceiling_deg: float = AXLE_STEER_LIMIT_DEG,
                   tol_deg: float = 0.001) -> float:
    """
    주어진 굴절각에서 허용되는 최대 축 유효조향각 [deg].

        한계 = min(ceiling_deg, 행정 0 ~ STROKE_MAX 와 토글 한계를 지키는 최대값)

    ceiling_deg 의 기본값은 기계 한계 AXLE_STEER_LIMIT_DEG 이다.
    전방이 신장·후방이 수축하는 방향과 그 반대 방향은 행정 여유가 서로 다를 수
    있으므로 +/- 양쪽을 모두 만족하는 크기를 찾는다 (대칭 clamp 용).
    """
    if not (math.isfinite(alpha_eff_deg) and 0 < alpha_eff_deg <= 180
            and math.isfinite(ceiling_deg) and 0 <= ceiling_deg < 90
            and math.isfinite(tol_deg) and tol_deg > 0):
        raise ValueError('Invalid steering limit search parameters')
    alpha = math.radians(alpha_eff_deg)

    def feasible(d_deg: float) -> bool:
        for s in (+1.0, -1.0):
            try:
                *_, f, r = _solve_axle(math.radians(s * d_deg), alpha)
            except ValueError:
                return False
            if not all(0.0 <= v <= STROKE_MAX for v in (f, r)):
                return False
        return True

    if feasible(ceiling_deg):
        return ceiling_deg
    lo, hi = 0.0, ceiling_deg
    while hi - lo > tol_deg:
        mid = 0.5 * (lo + hi)
        if feasible(mid):
            lo = mid
        else:
            hi = mid
    return lo


def max_effective_steer(alpha_eff_deg: float,
                        axle_ceiling_deg: float = AXLE_STEER_LIMIT_DEG,
                        tol_deg: float = 0.001) -> float:
    """
    주어진 굴절각에서 허용되는 최대 차량 유효조향각 [deg]. 조향 지령 clamp 용.

    축 기준 한계(max_axle_steer)를 구한 뒤 tan Delta_v = 2 tan Delta_a 로 환산한다.
    현 설계 상수에서는 기계 한계가 지배하므로 theta = +-45 deg 전 구간에서
    MAX_STEER_DEG (= 43.003 deg) 를 돌려준다.
    """
    return vehicle_steer_deg(max_axle_steer(alpha_eff_deg, axle_ceiling_deg, tol_deg))


def solve_steering(steer_deg: float,
                   motor_angle_deg: float,
                   inverted: bool = False,
                   *,
                   clamp: bool = True) -> SteeringSolution:
    """Return steering geometry with the existing solution fields.

    Non-finite and out-of-range values raise ValueError. The legacy ``clamp``
    keyword is accepted but no longer saturates unsafe input. Hardware callers
    use L12Pair.set_steering, which requests neutral before raising L12Error.
    motor_angle_deg is retained for geometry studies; the rover uses -30 (150 alpha).
    """
    if not math.isfinite(steer_deg) or not math.isfinite(motor_angle_deg) or abs(motor_angle_deg) > MAX_MOTOR_ANGLE_DEG:
        raise ValueError('Non-finite steering or invalid motor angle')
    st = articulation_state(motor_angle_deg, inverted)
    axle_limit = max_axle_steer(st.alpha_eff_deg)
    limit = vehicle_steer_deg(axle_limit)
    # Retain clamp keyword for callers, but never saturate unsafe commands.
    if abs(steer_deg) > limit + 1e-9:
        raise ValueError(f'Steering {steer_deg} exceeds +/-{limit:.3f} deg')
    cmd_eff = steer_deg

    alpha = math.radians(st.alpha_eff_deg)
    L = _wheelbase(alpha)
    axle = _axle_from_vehicle(math.radians(st.steer_sign * cmd_eff))
    gamma_f, gamma_r, delta_f, delta_r, front, rear = _solve_axle(axle, alpha)

    return SteeringSolution(
        steer_cmd_deg=float(steer_deg),
        steer_eff_deg=float(cmd_eff),
        steer_limit_deg=limit,
        axle_front_deg=math.degrees(axle),
        axle_rear_deg=-math.degrees(axle),
        axle_limit_deg=axle_limit,
        wheelbase_mm=L,
        # kappa = tan(Delta_v) / L. 조종자 기준 부호(전복 보정 전)로 기록한다.
        curvature_per_m=math.tan(math.radians(cmd_eff)) / (L * 1e-3),
        alpha_eff_deg=st.alpha_eff_deg,
        alpha_raw_deg=st.alpha_raw_deg,
        inverted=st.inverted,
        gamma_front_deg=math.degrees(gamma_f),
        gamma_rear_deg=math.degrees(gamma_r),
        delta_front_deg=math.degrees(delta_f),
        delta_rear_deg=math.degrees(delta_r),
        front_mm=front,
        rear_mm=rear,
    )


def steering_to_actuators(steer_deg: float,
                          motor_angle_deg: float,
                          inverted: bool = False,
                          *,
                          clamp: bool = True) -> ActuatorPositions:
    """solve_steering() 의 인출 길이만 필요할 때 쓰는 얇은 래퍼."""
    sol = solve_steering(steer_deg, motor_angle_deg, inverted, clamp=clamp)
    return ActuatorPositions(sol.front_mm, sol.rear_mm)


# =========================================================================
# [B] PWM 구동
# =========================================================================

# L12 PWM 출력. 'pwmN'(RK3566 PWM 번호) / 블록 주소 'fe6f0010' / sysfs 경로 중 하나로 준다.
# pwmchip 번호는 켜진 overlay 순서로 매겨져서 overlay 하나만 더 켜도 밀린다. 그래서 기본값은
# PWM 번호로 두고 resolve_pwm_path() 가 블록 주소로 실제 pwmchipN 을 찾는다.
#   261010 배선 변경: 전방 PWM8 (fe6f0000, 그대로), 후방 PWM14 (fe700020) -> PWM9 (fe6f0010)
FRONT_PWM_PATH = "pwm8"
REAR_PWM_PATH = "pwm9"
PWM_CLASS_DIR = "/sys/class/pwm"
# RK3566/RK3568 PWM 컨트롤러 주소. 4채널씩 한 묶음, 채널 간격 0x10.
# Rockchip 드라이버는 채널마다 pwmchip 을 따로 만들므로(npwm = 1) 채널은 항상 pwm0 이다.
_PWM_BASES = (0xFDD70000, 0xFE6E0000, 0xFE6F0000, 0xFE700000)   # PWM0-3, 4-7, 8-11, 12-15

MIN_PULSE_NS = 1_000_000      # 0 mm  (완전 수축)
MAX_PULSE_NS = 2_000_000      # STROKE_MAX  (완전 신장)
PERIOD_NS = 20_000_000        # 50 Hz


class L12Error(RuntimeError):
    pass


def pwm_block_address(pwm_index: int) -> str:
    """RK3566 PWM 번호 -> 블록 주소 문자열. 9 -> 'fe6f0010'."""
    if not 0 <= int(pwm_index) < 4 * len(_PWM_BASES):
        raise L12Error(f"PWM 번호는 0 ~ {4 * len(_PWM_BASES) - 1} 이어야 합니다: {pwm_index}")
    n = int(pwm_index)
    return f"{_PWM_BASES[n // 4] + 0x10 * (n % 4):08x}"


def resolve_pwm_path(spec: str, class_dir: str = PWM_CLASS_DIR) -> str:
    """
    PWM 지정 -> sysfs 채널 경로 (.../pwmchipN/pwm0).

        '/sys/class/pwm/pwmchip1/pwm0'  그대로 (예전 방식)
        'pwm9'                          RK3566 PWM9 -> 블록 fe6f0010 -> 그 블록의 pwmchipN
        'fe6f0010'                      블록 주소로 직접

    pwmchipN 은 /sys/class/pwm/pwmchipN -> .../platform/<주소>.pwm/pwm/pwmchipN 심볼릭 링크로 찾는다.
    pwm0 이 아직 export 안 됐어도 경로는 돌려준다 (export 는 radxa.ensure_pwm_channel 이 한다).
    해당 블록이 없으면(overlay 꺼짐) L12Error.
    """
    spec = str(spec).strip()
    if spec.startswith("/"):
        return spec
    m = re.fullmatch(r"pwm(\d+)", spec.lower())
    if m:
        addr = pwm_block_address(int(m.group(1)))
    elif re.fullmatch(r"[0-9a-f]{8}", spec.lower()):
        addr = spec.lower()
    else:
        raise L12Error(f"PWM 지정 형식이 아닙니다: {spec!r} (예: 'pwm9', 'fe6f0010', '/sys/class/pwm/pwmchip1/pwm0')")
    found = {}
    try:
        chips = sorted(c for c in os.listdir(class_dir) if c.startswith("pwmchip"))
    except OSError:
        chips = []
    for chip in chips:
        real = os.path.realpath(os.path.join(class_dir, chip))
        hit = re.search(r"/([0-9a-f]{8})\.pwm/", real + "/")
        found[chip] = hit.group(1) if hit else "?"
        if hit and hit.group(1) == addr:
            return os.path.join(class_dir, chip, "pwm0")
    have = ", ".join(f"{c}={a}" for c, a in found.items()) or "없음"
    raise L12Error(
        f"{spec} ({addr}.pwm) 에 해당하는 pwmchip 이 없습니다. rsetup 의 Overlays 에서 "
        f"그 PWM 을 켜고 재부팅했는지 확인하십시오. (현재 {class_dir}: {have})")


def mm_to_pulse(position_mm: float) -> int:
    """인출 길이 [mm] -> 펄스폭 [ns]. 0 ~ STROKE_MAX 선형 (실측 캘리브레이션 기준)."""
    if not math.isfinite(position_mm) or position_mm < -_EPS or position_mm > STROKE_MAX + _EPS:
        raise L12Error(f"인출 길이는 0 ~ {STROKE_MAX} mm 여야 합니다 (입력: {position_mm})")
    p = max(0.0, min(STROKE_MAX, position_mm))
    return int(round(MIN_PULSE_NS + (p / STROKE_MAX) * (MAX_PULSE_NS - MIN_PULSE_NS)))


class L12Pair:
    """
    전/후방 L12 한 쌍. sysfs PWM 으로 구동한다.

    simulate=True 면 sysfs 에 쓰지 않고 값만 계산한다. 하드웨어 없이
    상위 로직을 검증할 때 쓴다. PWM 경로가 없으면 자동으로 이 모드가 된다.

    hold_timeout_s
    --------------
    지령이 바뀌지 않은 채 이 시간이 지나면 PWM 출력을 끈다(enable = 0).
    다음 지령이 오면 duty 를 먼저 쓰고 다시 켠다.

    왜 필요한가: TXS0108E 같은 '자동 방향 감지형' 레벨 시프터는 출력을
    약한 원샷 + 풀업으로 구동하기 때문에, PWM 처럼 규칙적인 전이가 계속되면
    방향 감지 로직이 오동작해 글리치가 난다. 액추에이터가 제자리에서 떨거나
    엉뚱한 위치로 튀는 원인이 된다. 엣지 자체를 없애면 이 경로가 차단된다.
    L12 는 리드스크류 구동이라 신호가 없어도 위치를 유지한다.

    주의 1: hold_timeout_s 는 액추에이터가 최대 이동거리를 주파하는 시간보다
            반드시 길어야 한다. 짧으면 이동 도중에 신호가 끊겨 목표에 못 간다.
            실측 방법은 README 참조. 0 이면 이 기능을 끈다(항상 출력 유지).
    주의 2: enable = 0 이후 핀이 로우로 구동되는지 하이임피던스가 되는지는
            드라이버/핀먹스에 따라 다르다. 플로팅이 되면 시프터 풀업이
            5 V 로 끌어올려 연속 하이가 되므로, 실제 핀 전압을 한 번 확인할 것.
    주의 3: polarity 는 반드시 normal 이어야 한다. inversed 이면 duty 가 LOW
            구간의 길이가 되어 L12 가 18 ms 짜리 펄스를 읽는다. export 직후
            inversed 로 시작하는 채널이 있으므로 initialize() 가 검증한다.

    이건 증상 회피이지 근본 해결이 아니다. 단방향 버퍼(74AHCT125 등)로
    교체하는 것이 정석이다.
    """

    def __init__(self,
                 front_path: str = FRONT_PWM_PATH,
                 rear_path: str = REAR_PWM_PATH,
                 simulate: bool = False,
                 hold_timeout_s: float = 3.0,
                 deadband_mm: float = 0.02):
        # 'pwm9' 같은 지정을 실제 경로로. 못 찾으면 지정 그대로 둔다 -> 아래에서 simulate 로 떨어진다.
        self.front_path = self._resolve(front_path)
        self.rear_path = self._resolve(rear_path)
        # 지정('pwm9')이 아니라 resolve 한 경로로 판정한다. 지정으로 보면 항상 simulate 가 된다.
        self.simulate = simulate or not (os.path.isdir(self.front_path)
                                         and os.path.isdir(self.rear_path))
        self.hold_timeout_s = float(hold_timeout_s)
        self.deadband_mm = float(deadband_mm)
        self.front_mm: Optional[float] = None
        self.rear_mm: Optional[float] = None
        self.output_on = False          # PWM enable 상태
        self._last_change = 0.0         # 마지막으로 duty 를 바꾼 시각 [perf_counter]

    # ---------- 저수준 ----------
    @staticmethod
    def _resolve(spec: str) -> str:
        try:
            return resolve_pwm_path(spec)
        except L12Error:
            return spec

    def _write(self, pwm_path: str, filename: str, value):
        if self.simulate:
            return
        try:
            with open(os.path.join(pwm_path, filename), "w") as f:
                f.write(str(value))
        except FileNotFoundError as e:
            raise L12Error(
                f"PWM 경로를 찾을 수 없습니다: {pwm_path}/{filename}\n"
                f"  채널이 export 되지 않았을 수 있습니다:\n"
                f"    echo 0 | sudo tee {os.path.dirname(pwm_path)}/export") from e
        except PermissionError as e:
            raise L12Error(
                f"PWM 쓰기 권한이 없습니다: {pwm_path}/{filename}. "
                "root 로 실행하거나 컨테이너를 --privileged 로 띄우십시오.") from e
        except OSError as e:
            raise L12Error(
                f"PWM 쓰기 실패: {pwm_path}/{filename} <- {value}  ({e})\n"
                f"  현재 상태: {self._state_str(pwm_path)}\n"
                f"  EINVAL(22) 이면 period 가 0 이거나 duty > period 인 경우가 많습니다.") from e

    def _read(self, pwm_path: str, filename: str) -> Optional[str]:
        if self.simulate:
            return None
        try:
            with open(os.path.join(pwm_path, filename)) as f:
                return f.read().strip()
        except OSError:
            return None

    def _state_str(self, pwm_path: str) -> str:
        vals = {k: self._read(pwm_path, k)
                for k in ("period", "duty_cycle", "enable", "polarity")}
        return " ".join(f"{k}={v}" for k, v in vals.items())

    def _init_one(self, pwm_path: str, start_mm: float):
        """
        sysfs PWM 초기화. 순서가 중요하다.

        방금 export 된 채널은 period = 0, duty_cycle = 0 으로 시작한다.
        이 상태에서 enable 이나 polarity 를 쓰면 커널이 period = 0 으로
        상태를 적용하려다 EINVAL(22) 을 돌려준다. 또 기존 duty 가 새 period
        보다 크면 period 쓰기 자체가 EINVAL 이 된다.

        따라서:
          1) duty_cycle -> 0   (period 를 줄일 때의 충돌 방지)
          2) period     -> 20 ms
          3) polarity            (disabled 상태에서만. 미지원 드라이버는 건너뜀)
          4) duty_cycle -> 목표
          5) enable 은 _set_output 이 따로 처리
        """
        if self.simulate:
            return
        if self._read(pwm_path, "enable") == "1":
            self._write(pwm_path, "enable", 0)

        # 1) duty 를 먼저 0 으로 내려야 period 를 자유롭게 바꿀 수 있다
        try:
            self._write(pwm_path, "duty_cycle", 0)
        except L12Error:
            pass                      # period 가 0 이면 이 쓰기 자체가 무의미

        # 2) period 가 먼저 서야 나머지 쓰기가 유효해진다
        self._write(pwm_path, "period", PERIOD_NS)

        # 3) polarity 는 반드시 normal 이어야 한다. 실패하면 진행하지 않는다.
        #
        #    inversed 이면 duty_cycle 이 LOW 구간의 길이가 된다. 즉
        #    duty=1.533 ms / period=20 ms 에서 실제 HIGH 는 18.47 ms 가 되어
        #    L12 가 기대하는 1~2 ms 범위를 완전히 벗어난다. 이 상태로 구동하면
        #    액추체이터가 예측 불가능하게 움직인다. 경고만 하고 넘어가면 안 된다.
        pol = self._read(pwm_path, "polarity")
        if pol is not None and pol != "normal":
            self._write(pwm_path, "polarity", "normal")
            pol = self._read(pwm_path, "polarity")
            if pol != "normal":
                raise L12Error(
                    f"polarity 를 normal 로 바꾸지 못했습니다: {pwm_path} (현재 {pol})\n"
                    f"  inversed 상태에서는 duty 가 LOW 구간 길이가 되어 "
                    f"L12 가 완전히 잘못된 펄스폭을 읽습니다.\n"
                    f"  수동 확인:  echo normal | sudo tee {pwm_path}/polarity\n"
                    f"  (polarity 는 enable=0 이고 period 가 설정된 뒤에만 쓸 수 있습니다.)")

        # 4) 목표 duty
        self._write(pwm_path, "duty_cycle", mm_to_pulse(start_mm))

        # 5) 최종 검증. 여기서 어긋나면 켜지 않는다.
        got_p = self._read(pwm_path, "period")
        got_d = self._read(pwm_path, "duty_cycle")
        want_d = mm_to_pulse(start_mm)
        if got_p is not None and int(got_p) != PERIOD_NS:
            raise L12Error(f"period 반영 실패: {pwm_path} "
                           f"기대 {PERIOD_NS}, 실제 {got_p}")
        if got_d is not None and abs(int(got_d) - want_d) > 1000:
            raise L12Error(f"duty_cycle 반영 실패: {pwm_path} "
                           f"기대 {want_d}, 실제 {got_d}")

    def _set_output(self, on: bool):
        """양쪽 PWM enable 을 한꺼번에 바꾼다. 상태가 다를 때만 쓴다."""
        if on == self.output_on:
            return
        for p in (self.front_path, self.rear_path):
            if self._read(p, "period") in ("0", None) and not self.simulate:
                raise L12Error(
                    f"period 가 설정되지 않았습니다: {p}\n"
                    f"  현재 상태: {self._state_str(p)}\n"
                    f"  initialize() 를 먼저 호출해야 합니다.")
            self._write(p, "enable", 1 if on else 0)
        self.output_on = on

    # ---------- 공개 API ----------
    def initialize(self):
        """중립 위치로 초기화한다. 중립은 기구학에서 유도된 값(16.0 mm)이다."""
        n = neutral_position()
        self.output_on = False
        self._init_one(self.front_path, n)
        self._init_one(self.rear_path, n)
        self.front_mm = self.rear_mm = n
        self._set_output(True)
        self._last_change = time.perf_counter()
        return n

    def set_positions(self, front_mm: float, rear_mm: float,
                      deadband_mm: Optional[float] = None) -> ActuatorPositions:
        """
        인출 길이를 직접 지령한다. 변화가 deadband 미만이면 sysfs 쓰기를 건너뛴다.
        L12 는 피드백이 없으므로 여기서 반환하는 값은 '지령값'이지 실제 위치가 아니다.

        deadband 는 XM430 엔코더의 미세 진동이 캐스터각을 통해 인출 길이를
        흔드는 것을 걸러내기 위한 것이다. 중립 부근 감도 dE/d(delta) = e = 18 mm/rad
        기준으로 0.02 mm 는 축 유효조향각 약 0.06 deg, 차량 유효조향각 약 0.13 deg
        이므로 조향 정밀도에 영향이 없다.

        지령이 실제로 바뀌면 PWM 출력을 켜고 hold 타이머를 초기화한다.
        """
        db = self.deadband_mm if deadband_mm is None else deadband_mm
        if not all(math.isfinite(x) and 0 <= x <= STROKE_MAX for x in (front_mm, rear_mm)):
            self.neutral()
            raise L12Error('Invalid actuator position; returned to neutral')
        f = front_mm
        r = rear_mm

        changed = (self.front_mm is None or abs(f - self.front_mm) >= db or
                   self.rear_mm is None or abs(r - self.rear_mm) >= db)
        if changed:
            # duty 를 먼저 쓰고 켠다. 꺼진 채널에 쓴 값이 그대로 반영되므로
            # 켜지는 순간 이전 값으로 튀는 글리치가 없다.
            self._write(self.front_path, "duty_cycle", mm_to_pulse(f))
            self._write(self.rear_path, "duty_cycle", mm_to_pulse(r))
            self.front_mm, self.rear_mm = f, r
            self._set_output(True)
            self._last_change = time.perf_counter()
        return ActuatorPositions(f, r)

    def update(self, now: Optional[float] = None) -> bool:
        """
        매 주기 호출한다. hold_timeout_s 가 지나면 PWM 출력을 끈다.
        반환값은 현재 출력 상태(True = 켜짐).
        """
        if self.hold_timeout_s <= 0.0:
            return self.output_on
        t = time.perf_counter() if now is None else now
        if self.output_on and (t - self._last_change) >= self.hold_timeout_s:
            self._set_output(False)
        return self.output_on

    def set_steering(self, steer_deg: float, motor_angle_deg: float,
                     inverted: bool = False) -> SteeringSolution:
        """
        조종 지령(차량 유효조향각 [deg]), 현재 XM430 각도, 전복 플래그로 구동한다.
        전복 보정과 한계 clamp(기계·행정)를 포함한다. 상위 계층은 이것만 부르면 된다.

        반환값은 중간 변수까지 담은 SteeringSolution 이라 그대로 로그에 쓸 수 있다.
        """
        try:
            sol = solve_steering(steer_deg, motor_angle_deg, inverted)
        except (ValueError, TypeError, OverflowError) as exc:
            self.neutral()
            raise L12Error(f'Invalid steering; returned to neutral: {exc}') from exc
        self.set_positions(sol.front_mm, sol.rear_mm)
        return sol

    def neutral(self) -> ActuatorPositions:
        n = neutral_position()
        return self.set_positions(n, n)

    def close(self):
        """PWM 출력을 끈다. 액추에이터는 마지막 위치에서 힘 없이 멈춘다."""
        try:
            self.output_on = True      # _set_output 이 실제로 쓰도록
            self._set_output(False)
        except L12Error:
            pass


# =========================================================================
# 자체 검증
# =========================================================================

if __name__ == "__main__":
    print(f"커플러 길이 l = {COUPLER_LENGTH} mm  ->  중립 인출 "
          f"{neutral_position():.3f} mm  (행정 0 ~ {STROKE_MAX} mm)")
    print(f"조향 한계: 축 +-{AXLE_STEER_LIMIT_DEG:.1f} deg  <->  "
          f"차량 +-{MAX_STEER_DEG:.3f} deg\n")

    # [1] 항등식: |gamma_f| + |gamma_r| == 180 - alpha
    for a in (180.0, 165.0, 150.0, 120.0):
        gf, gr = _caster_angles(math.radians(a))
        assert abs(math.degrees(gf + gr) - (180.0 - a)) < 1e-9
    print("[1] 캐스터각 항등식 |gf|+|gr| = 180-alpha  ... OK")

    # [2] 차량 <-> 축 유효조향각 환산: 왕복 항등, 홀함수, 단조성
    prev = -1e9
    for v in [x * 0.5 for x in range(-170, 171)]:
        a = axle_steer_deg(v)
        assert abs(vehicle_steer_deg(a) - v) < 1e-9
        assert abs(axle_steer_deg(-v) + a) < 1e-12
        assert a > prev
        prev = a
    assert abs(axle_steer_deg(MAX_STEER_DEG) - AXLE_STEER_LIMIT_DEG) < 1e-9
    print("[2] Delta_v <-> Delta_a 왕복/홀함수/단조  ... OK")

    # [3] 곡률 정의 검증: 두 휠 법선의 교점(ICR)을 직접 구해
    #     중점 회전반경이 L / tan(Delta_v) 와 같은지 확인한다 (식과 독립적인 기하 검증).
    def _icr_radius(L, da_f, da_r):
        # 전방 액슬 (L/2, 0), 후방 (-L/2, 0). 각 휠 법선: p + t * (-sin, cos)
        # (L/2 - t1 sf, t1 cf) = (-L/2 - t2 sr, t2 cr)  ->  2x2 선형계
        sf, cf, sr, cr = math.sin(da_f), math.cos(da_f), math.sin(da_r), math.cos(da_r)
        a11, a12, b1 = -sf, sr, -L
        a21, a22, b2 = cf, -cr, 0.0
        det = a11 * a22 - a12 * a21
        t1 = (b1 * a22 - a12 * b2) / det
        xc, yc = L / 2 - t1 * sf, t1 * cf
        return xc, yc
    for th in (0.0, 30.0, 45.0):
        for v in (5.0, 20.0, MAX_STEER_DEG):
            sol = solve_steering(v, th, False)
            xc, yc = _icr_radius(sol.wheelbase_mm, math.radians(sol.axle_front_deg),
                                 math.radians(sol.axle_rear_deg))
            r_expect = sol.wheelbase_mm / math.tan(math.radians(v))
            assert abs(xc) < 1e-9 and abs(yc - r_expect) < 1e-9
            assert abs(sol.curvature_per_m - 1e3 / r_expect) < 1e-9
    print("[3] ICR 기하: 중점 회전반경 = L / tan(Delta_v), 곡률 필드 일치  ... OK")

    # [4] theta 부호는 기구학에만, 전복은 조향 부호에만 영향
    print("\n[4] theta 부호와 전복 플래그의 독립성 (지령 +30 deg)")
    for th in (15.0, 30.0, 45.0):
        a = solve_steering(+30.0, +th, False)
        b = solve_steering(+30.0, -th, False)
        assert abs(a.front_mm - b.front_mm) < 1e-12
        assert abs(a.rear_mm - b.rear_mm) < 1e-12
        c = solve_steering(+30.0, +th, True)
        flipped = (a.front_mm - neutral_position()) * (c.front_mm - neutral_position()) < 0
        assert flipped
        d = solve_steering(-30.0, +th, False)
        assert abs(c.front_mm - d.front_mm) < 1e-12 and abs(c.rear_mm - d.rear_mm) < 1e-12
        print(f"    theta=+-{th:4.1f} 정상: ({a.front_mm:7.3f},{a.rear_mm:7.3f})   "
              f"전복: ({c.front_mm:7.3f},{c.rear_mm:7.3f})   반전={flipped}")

    # [5] 전복 플래그가 theta 부호와 무관한지
    assert articulation_state(+30.0, True).steer_sign == -1.0
    assert articulation_state(-30.0, False).steer_sign == +1.0
    print("\n[5] 전복 플래그는 theta 부호와 독립  ... OK")

    # [6] 구동 한계 전 구간에서 최대 지령이 축 25 deg 가 되고 행정 안에 드는지
    print(f"\n[6] theta -45 ~ +45 에서 최대 지령 +{MAX_STEER_DEG:.3f} deg (정상 자세)")
    print(f"    {'theta':>6}{'L':>7}{'D_a':>7}{'gam_f':>8}{'gam_r':>8}"
          f"{'del_f':>8}{'del_r':>8}{'front':>8}{'rear':>8}{'R_mid':>7}{'ax_lim':>8}")
    for th in (-45.0, -30.0, -15.0, 0.0, 15.0, 30.0, 45.0):
        sol = solve_steering(+MAX_STEER_DEG, th, False)
        assert abs(sol.axle_front_deg - AXLE_STEER_LIMIT_DEG) < 1e-9
        assert 0.0 <= sol.front_mm <= STROKE_MAX
        assert 0.0 <= sol.rear_mm <= STROKE_MAX
        print(f"    {th:6.1f}{sol.wheelbase_mm:7.1f}{sol.axle_front_deg:7.2f}"
              f"{sol.gamma_front_deg:8.3f}{sol.gamma_rear_deg:8.3f}"
              f"{sol.delta_front_deg:8.3f}{sol.delta_rear_deg:8.3f}"
              f"{sol.front_mm:8.3f}{sol.rear_mm:8.3f}"
              f"{1e3 / sol.curvature_per_m:7.1f}"
              f"{max_axle_steer(sol.alpha_eff_deg, ceiling_deg=40.0):8.2f}")
    print("    (ax_lim = 기계 한계를 빼고 행정만으로 본 축 한계. 기계 한계 25 deg 가 지배)")

    # [7] 조향 0 에서 전후 모두 중립
    sol = solve_steering(0.0, 30.0, False)
    assert abs(sol.front_mm - neutral_position()) < 1e-12
    assert abs(sol.rear_mm - neutral_position()) < 1e-12
    assert sol.curvature_per_m == 0.0
    print("\n[7] 조향 0 에서 전후 모두 중립 위치  ... OK")

    # Invalid commands must be rejected, including NaN and infinity.
    for bad in (80.0, -80.0, float('nan'), float('inf')):
        try:
            solve_steering(bad, -30.0)
            raise AssertionError('Invalid steering accepted')
        except ValueError:
            pass
    print('[8] invalid steering rejection OK')

    # [9] PWM 환산
    print("\n[9] mm -> pulse")
    for mm in (0.0, 8.0, 16.0, 24.0, 30.0):
        print(f"    {mm:5.1f} mm -> {mm_to_pulse(mm):>9d} ns")

    # [10] 시뮬레이션 모드
    pair = L12Pair(simulate=True)
    pair.initialize()
    assert abs(pair.front_mm - 16.0) < 1e-9
    s1 = pair.set_steering(+30.0, 30.0, False)
    s2 = pair.set_steering(+30.0, 30.0, True)
    print(f"\n[10] simulate: 중립 {neutral_position():.1f} mm | "
          f"정상 ({s1.front_mm:.3f},{s1.rear_mm:.3f}) | "
          f"전복 ({s2.front_mm:.3f},{s2.rear_mm:.3f})  ... OK")

    # [11] hold timeout: 지령이 멈추면 출력이 꺼지고, 새 지령에 다시 켜진다
    import time as _t
    ev = []
    pair2 = L12Pair(simulate=True, hold_timeout_s=0.3, deadband_mm=0.02)
    pair2._write = lambda path, name, val: ev.append((name, val))
    pair2.initialize()
    assert pair2.output_on
    pair2.set_steering(+MAX_STEER_DEG, 30.0, False)
    n_before = len([e for e in ev if e[0] == 'duty_cycle'])
    # 같은 지령을 반복해도 duty 를 다시 쓰지 않는다 (deadband)
    for _ in range(50):
        pair2.set_steering(+MAX_STEER_DEG, 30.0 + 0.0879, False)
        pair2.update()
    assert len([e for e in ev if e[0] == 'duty_cycle']) == n_before, "deadband 미동작"
    assert pair2.output_on, "타임아웃 전에 꺼짐"
    _t.sleep(0.35)
    pair2.update()
    assert not pair2.output_on, "타임아웃 후에도 켜져 있음"
    pair2.set_steering(0.0, 30.0, False)      # 중립 복귀 = 새 지령
    assert pair2.output_on, "새 지령에 다시 안 켜짐"
    print(f"[11] hold_timeout 0.3 s: 유지 중 duty 재기록 0회, "
          f"타임아웃 후 enable=0, 새 지령에 재기동  ... OK")