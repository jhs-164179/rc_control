#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
mock_dxl.py — 하드웨어 없이 코드 경로를 점검하기 위한 XM430 대역품.

dxl_driver 의 공개 API(레지스터 수준 w1/w2/w4, read(), configure(), ...)를
같은 모양으로 흉내낸다. 상위 코드는 실물과 mock 을 구분하지 않는다.

⚠️ 물리 모델은 '방향 비대칭이 존재하는 1자유도 회전계' 수준의 조잡한
   근사다. 여기서 나온 숫자를 실제 튜닝값으로 쓰면 안 된다. 목적은
   상태기계·로깅·실험 스크립트가 제대로 도는지 확인하는 것뿐이다.

   일부러 넣은 비대칭: 중력 토크(+방향을 거스름)와 쿨롱 마찰.
   그래서 asymmetry.py 의 식별/보정이 실제로 수렴하는지 여기서 볼 수 있다.
"""

from __future__ import annotations

import math
import os
import time
from dataclasses import dataclass, field
from typing import NamedTuple, Optional

__all__ = [
    "DxlError", "MotorConfig", "Feedback", "XM430",
    "find_port", "read_latency_timer", "estimate_torque_mnm",
    "RAD_PER_PULSE", "CURRENT_LSB_TO_A", "KT",
]

# ---- dxl_driver 와 동일해야 하는 상수/주소 ----
ADDR_MODEL_NUMBER = 0
ADDR_OPERATING_MODE = 11
ADDR_CURRENT_LIMIT = 38
ADDR_VELOCITY_LIMIT = 44
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR = 70
ADDR_POSITION_D_GAIN = 80
ADDR_POSITION_I_GAIN = 82
ADDR_POSITION_P_GAIN = 84
ADDR_GOAL_CURRENT = 102
ADDR_PROFILE_ACCELERATION = 108
ADDR_PROFILE_VELOCITY = 112
ADDR_GOAL_POSITION = 116
ADDR_MOVING = 122
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_TEMPERATURE = 146

OP_CURRENT_BASED_POSITION = 5
MODEL_XM430_W210 = 1010

TAU = 2.0 * math.pi
RAD_PER_PULSE = TAU / 4096.0
PULSE_PER_RAD = 1.0 / RAD_PER_PULSE
CURRENT_LSB_TO_A = 2.69e-3
DPS_PER_VEL_LSB = 0.229 * 360.0 / 60.0
DPSS_PER_ACC_LSB = 214.577 * 360.0 / 3600.0
KT = 1.304
VELOCITY_LIMIT_LSB = 330


def estimate_torque_mnm(current_a: float) -> float:
    return KT * current_a * 1000.0


class DxlError(RuntimeError):
    pass


@dataclass
class MotorConfig:
    """dxl_driver.MotorConfig 와 필드가 일치해야 한다."""
    dxl_id: int = 1
    baudrate: int = 57600
    current_limit: int = 250
    current_moving: int = 120
    current_recover: int = 7
    current_holding: int = 3
    profile_velocity_dps: float = 90.0
    profile_accel_dpss: float = 100.0
    p_gain: int = 200
    i_gain: int = 0
    d_gain: int = 300
    home_rad: float = 0.0
    settled_band_rad: float = math.radians(1.0)
    disturb_band_rad: float = math.radians(5.0)
    reissue_interval_s: float = 0.4

    def validate(self):
        levels = (self.current_holding, self.current_recover, self.current_moving)
        if not all(0 < v <= self.current_limit for v in levels):
            raise DxlError("전류 설정 오류: 세 값 모두 0 초과, current_limit 이하여야 합니다.")


class Feedback(NamedTuple):
    moving: bool
    current_a: float
    velocity_rad_s: float
    position_rad: float


def find_port() -> str:
    return "MOCK"


def read_latency_timer(port_path: str) -> Optional[int]:
    return 1


# ============================================================
# 물리 모델
# ============================================================
def _env(name, default):
    try:
        return float(os.environ[name])
    except (KeyError, ValueError):
        return default


@dataclass
class PlantParams:
    """
    환경변수로 덮어쓸 수 있다. 비대칭 보정이 실제로 수렴하는지 보려면
    중력을 키워서 돌려 보면 된다:
        MOCK_GRAVITY_NM=0.30 python3 asymmetry.py cal --mock
    """
    inertia: float = field(default_factory=lambda: _env("MOCK_INERTIA", 4.0e-3))
    gravity_torque: float = field(
        default_factory=lambda: _env("MOCK_GRAVITY_NM", 0.10))
    coulomb: float = field(default_factory=lambda: _env("MOCK_COULOMB_NM", 0.020))
    viscous: float = 2.0e-3        # [N.m/(rad/s)]
    kp_scale: float = 0.020        # 게인 LSB -> 토크 환산 (임의)
    kd_scale: float = 5.0e-4


class _Plant:
    """1 kHz 내부 루프 + 사다리꼴 프로파일 생성기를 흉내낸다."""

    DT = 1.0e-3

    def __init__(self, params: PlantParams):
        self.p = params
        self.theta = 0.0
        self.omega = 0.0
        self.theta_d = 0.0          # 프로파일 생성기 출력 (목표 위치)
        self.omega_d = 0.0
        self.goal = 0.0
        self.torque_on = False
        self.goal_current = 0
        self.current_limit = 250
        self.prof_vel_lsb = 66
        self.prof_acc_lsb = 5
        self.p_gain, self.i_gain, self.d_gain = 200, 0, 300
        self.tau_applied = 0.0
        self.external = 0.0         # 외란 토크 주입구 [N.m]
        self._t = time.monotonic()

    # ---- 프로파일 생성기 ----
    def _profile_step(self):
        v_max = max(self.prof_vel_lsb, 1) * DPS_PER_VEL_LSB * math.pi / 180.0
        a_max = max(self.prof_acc_lsb, 1) * DPSS_PER_ACC_LSB * math.pi / 180.0
        err = self.goal - self.theta_d
        # 남은 거리에서 정지하려면 필요한 감속 구간을 넘지 않게 속도를 제한
        v_stop = math.sqrt(max(2.0 * a_max * abs(err), 0.0))
        v_tgt = math.copysign(min(v_max, v_stop), err)
        dv = v_tgt - self.omega_d
        dv = max(-a_max * self.DT, min(a_max * self.DT, dv))
        self.omega_d += dv
        self.theta_d += self.omega_d * self.DT
        # 실물처럼 목표 근처에서 딱 끊는다. 안 그러면 생성기가 미세
        # 리밋사이클에 빠져 Moving 플래그가 영원히 서 있다.
        if abs(self.goal - self.theta_d) < 1e-3 and abs(self.omega_d) < 2e-2:
            self.theta_d, self.omega_d = self.goal, 0.0

    def _dyn_step(self):
        p = self.p
        if self.torque_on:
            err = self.theta_d - self.theta
            tau = p.kp_scale * self.p_gain * err + \
                  p.kd_scale * self.d_gain * (self.omega_d - self.omega)
            cap = KT * min(self.goal_current, self.current_limit) * CURRENT_LSB_TO_A
            tau = max(-cap, min(cap, tau))
        else:
            tau = 0.0
        self.tau_applied = tau

        net = tau + self.external - p.gravity_torque * math.cos(self.theta)
        net -= p.viscous * self.omega
        # 쿨롱 마찰: 정지 상태에서는 구동 토크를 상쇄하는 정지마찰로 동작
        if abs(self.omega) < 1e-3:
            if abs(net) <= p.coulomb:
                net = 0.0
                self.omega = 0.0
            else:
                net -= math.copysign(p.coulomb, net)
        else:
            net -= math.copysign(p.coulomb, self.omega)

        self.omega += (net / p.inertia) * self.DT
        self.theta += self.omega * self.DT

    def advance(self, now: float):
        n = int((now - self._t) / self.DT)
        if n <= 0:
            return
        n = min(n, 5000)            # 오래 멈춰 있었으면 따라잡기를 포기한다
        self._t += n * self.DT
        for _ in range(n):
            self._profile_step()
            self._dyn_step()

    @property
    def current_a(self) -> float:
        return self.tau_applied / KT


class XM430:
    """dxl_driver.XM430 의 공개 API 를 흉내낸다."""

    ST_CMD, ST_REC, ST_HOLD = 0, 1, 2

    def __init__(self, port_name: str, cfg: MotorConfig = None,
                 plant: PlantParams = None):
        self.cfg = cfg or MotorConfig()
        self.cfg.validate()
        self.port_name = port_name
        self.plant = _Plant(plant or PlantParams())
        self._goal = 0.0

    # ---- 레지스터 ----
    def _write(self, addr, val):
        pl = self.plant
        pl.advance(time.monotonic())
        if addr == ADDR_TORQUE_ENABLE:
            pl.torque_on = bool(val)
        elif addr == ADDR_GOAL_CURRENT:
            pl.goal_current = val if val < 0x8000 else val - 0x10000
        elif addr == ADDR_CURRENT_LIMIT:
            pl.current_limit = val
        elif addr == ADDR_GOAL_POSITION:
            v = val - (1 << 32) if val & (1 << 31) else val
            pl.goal = v * RAD_PER_PULSE
        elif addr == ADDR_PROFILE_VELOCITY:
            pl.prof_vel_lsb = val
        elif addr == ADDR_PROFILE_ACCELERATION:
            pl.prof_acc_lsb = val
        elif addr == ADDR_POSITION_P_GAIN:
            pl.p_gain = val
        elif addr == ADDR_POSITION_I_GAIN:
            pl.i_gain = val
        elif addr == ADDR_POSITION_D_GAIN:
            pl.d_gain = val

    def _read(self, addr):
        pl = self.plant
        pl.advance(time.monotonic())
        return {
            ADDR_MODEL_NUMBER: MODEL_XM430_W210,
            ADDR_OPERATING_MODE: OP_CURRENT_BASED_POSITION,
            ADDR_CURRENT_LIMIT: pl.current_limit,
            ADDR_VELOCITY_LIMIT: VELOCITY_LIMIT_LSB,
            ADDR_POSITION_P_GAIN: pl.p_gain,
            ADDR_POSITION_I_GAIN: pl.i_gain,
            ADDR_POSITION_D_GAIN: pl.d_gain,
            ADDR_PROFILE_VELOCITY: pl.prof_vel_lsb,
            ADDR_PROFILE_ACCELERATION: pl.prof_acc_lsb,
            ADDR_HARDWARE_ERROR: 0,
            ADDR_PRESENT_TEMPERATURE: 32,
        }.get(addr, 0)

    def w1(self, a, v, what=""): self._write(a, v)
    def w2(self, a, v, what=""): self._write(a, v & 0xFFFF)
    def w4(self, a, v, what=""): self._write(a, v & 0xFFFFFFFF)
    def r1(self, a, what=""): return self._read(a)
    def r2(self, a, what=""): return self._read(a)
    def r4(self, a, what=""): return self._read(a)

    def torque(self, on: bool):
        self.w1(ADDR_TORQUE_ENABLE, 1 if on else 0)

    # ---- 상위 API ----
    def configure(self, verbose: bool = True) -> dict:
        c = self.cfg
        self.torque(False)
        self.w2(ADDR_CURRENT_LIMIT, c.current_limit)
        self.torque(True)
        self.w2(ADDR_POSITION_P_GAIN, c.p_gain)
        self.w2(ADDR_POSITION_I_GAIN, c.i_gain)
        self.w2(ADDR_POSITION_D_GAIN, c.d_gain)
        self.w4(ADDR_PROFILE_ACCELERATION,
                max(1, int(round(c.profile_accel_dpss / DPSS_PER_ACC_LSB))))
        self.w4(ADDR_PROFILE_VELOCITY,
                max(1, int(round(c.profile_velocity_dps / DPS_PER_VEL_LSB))))
        self.w2(ADDR_GOAL_CURRENT, c.current_holding)
        self._goal = self.plant.theta
        self.w4(ADDR_GOAL_POSITION, int(round(self._goal * PULSE_PER_RAD)))
        info = {
            "model": MODEL_XM430_W210,
            "operating_mode": OP_CURRENT_BASED_POSITION,
            "current_limit": c.current_limit,
            "p_gain": c.p_gain, "i_gain": c.i_gain, "d_gain": c.d_gain,
            "profile_velocity": self.r4(ADDR_PROFILE_VELOCITY),
            "profile_accel": self.r4(ADDR_PROFILE_ACCELERATION),
            "velocity_limit": VELOCITY_LIMIT_LSB,
        }
        if verbose:
            print("── read-back (MOCK — 실제 하드웨어 아님) ─────")
            for k, v in info.items():
                print(f"  {k:<20s} = {v}")
            print("──────────────────────────────────────────────")
        return info

    def read(self) -> Feedback:
        pl = self.plant
        pl.advance(time.monotonic())
        moving = abs(pl.goal - pl.theta_d) > 1e-4 or abs(pl.omega_d) > 1e-3
        return Feedback(moving, pl.current_a, pl.omega, pl.theta)

    def set_gains(self, p=None, i=None, d=None):
        if p is not None:
            self.cfg.p_gain = int(p); self.w2(ADDR_POSITION_P_GAIN, int(p))
        if i is not None:
            self.cfg.i_gain = int(i); self.w2(ADDR_POSITION_I_GAIN, int(i))
        if d is not None:
            self.cfg.d_gain = int(d); self.w2(ADDR_POSITION_D_GAIN, int(d))

    def set_profile(self, velocity_dps=None, accel_dpss=None):
        if velocity_dps is not None:
            self.cfg.profile_velocity_dps = float(velocity_dps)
            self.w4(ADDR_PROFILE_VELOCITY,
                    min(max(1, int(round(velocity_dps / DPS_PER_VEL_LSB))),
                        VELOCITY_LIMIT_LSB))
        if accel_dpss is not None:
            self.cfg.profile_accel_dpss = float(accel_dpss)
            self.w4(ADDR_PROFILE_ACCELERATION,
                    max(1, int(round(accel_dpss / DPSS_PER_ACC_LSB))))

    def hardware_error(self) -> int: return 0

    def temperature(self) -> int: return 32

    def inject_external(self, torque_nm: float):
        """mock 전용. 외력을 흉내내기 위한 주입구."""
        self.plant.external = float(torque_nm)

    def shutdown(self):
        self.plant.torque_on = False

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.shutdown()
        return False
