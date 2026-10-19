#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
dxl_driver.py — DYNAMIXEL XM430-W210-R 제어 계층 (Linux, Protocol 2.0)

DDS 와 무관하다. 이 모듈만 가져다 로컬 제어에 써도 된다.

핵심 사용법
-----------
    from motor_control.xm430.driver import XM430, MotorConfig, find_port

    m = XM430(find_port())
    m.configure()                 # Current-based Position Control 진입
    m.set_goal(math.radians(90))  # 목표각 설정 (프로파일 궤적으로 이동)
    fb = m.read()                 # Feedback 네임드튜플
    m.shutdown()

설계 요지
---------
  Goal Position 을 매 주기 밀어넣지 않는다. 최종 목표를 한 번만 쓰고,
  Profile Velocity(112)/Acceleration(108) 이 만드는 사다리꼴 궤적을
  모터 내부 1 kHz 루프가 보간하게 한다. 통신 지연·지터가 궤적 품질에
  영향을 주지 않는다.

제어표 출처: ROBOTIS e-Manual, XM430-W210-T/R
  https://emanual.robotis.com/docs/en/dxl/x/xm430-w210/
"""

from __future__ import annotations

import glob
import math
import os
from dataclasses import dataclass
from typing import NamedTuple, Optional

from dynamixel_sdk import (PortHandler, PacketHandler,
                           GroupSyncRead, COMM_SUCCESS)
from serial.tools import list_ports

__all__ = [
    "DxlError", "MotorConfig", "Feedback", "XM430",
    "find_port", "read_latency_timer", "estimate_torque_mnm",
    "RAD_PER_PULSE", "CURRENT_LSB_TO_A", "KT",
]


# ============================================================
# 제어표 (XM430-W210, Protocol 2.0)
# ============================================================
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
ADDR_MOVING = 122            # 122~135 를 한 번에 읽는다
ADDR_PRESENT_CURRENT = 126
ADDR_PRESENT_VELOCITY = 128
ADDR_PRESENT_POSITION = 132
ADDR_PRESENT_TEMPERATURE = 146
LEN_FEEDBACK = 14

OP_CURRENT_BASED_POSITION = 5
MODEL_XM430_W210 = 1010
PROTOCOL_VERSION = 2.0

TAU = 2.0 * math.pi
RAD_PER_PULSE = TAU / 4096.0
PULSE_PER_RAD = 1.0 / RAD_PER_PULSE
VEL_LSB_TO_RAD_S = 0.229 * TAU / 60.0
CURRENT_LSB_TO_A = 2.69e-3

# Profile Velocity(112) 1 LSB = 0.229 [rev/min]
DPS_PER_VEL_LSB = 0.229 * 360.0 / 60.0            # 1.374 [deg/s]
# Profile Acceleration(108) 1 LSB = 214.577 [rev/min^2]
DPSS_PER_ACC_LSB = 214.577 * 360.0 / 3600.0       # 21.4577 [deg/s^2]

# 토크 상수. @12V 규격표의 stall torque / stall current 에서 유도: 3.0/2.3 = 1.304.
# 이 계수는 stall 지점의 손실을 이미 포함하므로 무부하/standby 전류를 추가로
# 감산하면 이중 차감이 되어 저전류 영역이 0 으로 뭉갠다.
# 11.1 V / 14.8 V 조건에서는 각각 1.286 / 1.370 이므로 전압이 다르면 바꿀 것.
KT = 1.304


def estimate_torque_mnm(current_a: float) -> float:
    """
    Present Current(126) 로부터의 토크 추정치 [mN.m].
    XM430 에는 토크 센서가 없다. 선형 근사이며 저전류/백래시 구간에서
    오차가 크다. 정량값은 로드셀 캘리브레이션 필요.
    """
    return KT * current_a * 1000.0


def _to_signed(v, bits):
    return v - (1 << bits) if v & (1 << (bits - 1)) else v


class DxlError(RuntimeError):
    pass


# ============================================================
# 설정
# ============================================================
@dataclass
class MotorConfig:
    """모든 튜닝 파라미터. 각도 단위는 radian."""
    dxl_id: int = 1
    baudrate: int = 57600

    # 전류 (1 LSB = 2.69 mA). 세 값 모두 current_limit 이하여야 한다.
    current_limit: int = 250
    current_moving: int = 120     # 지령 이동 중
    current_recover: int = 7      # 외란 복귀 중 (낮출수록 부드럽게 돌아온다)
    current_holding: int = 3      # 정지 중 (유지 토크가 세면 낮춘다)

    # 궤적
    profile_velocity_dps: float = 90.0
    profile_accel_dpss: float = 100.0

    # 게인 (XM430 기본값 P=800, I=0, D=0)
    p_gain: int = 200
    i_gain: int = 0
    d_gain: int = 300

    # 판정 밴드
    home_rad: float = 0.0
    settled_band_rad: float = math.radians(1.0)
    disturb_band_rad: float = math.radians(5.0)
    reissue_interval_s: float = 0.4

    def validate(self):
        levels = (self.current_holding, self.current_recover, self.current_moving)
        if not all(0 < v <= self.current_limit for v in levels):
            raise DxlError("전류 설정 오류: 세 값 모두 0 초과, current_limit 이하여야 "
                           "합니다. Goal Current 는 Current Limit 으로 clamp 됩니다.")
        if not (self.current_holding <= self.current_recover <= self.current_moving):
            print("[dxl] 경고: holding <= recover <= moving 순서가 아닙니다.")


class Feedback(NamedTuple):
    moving: bool
    current_a: float
    velocity_rad_s: float
    position_rad: float


# ============================================================
# 포트
# ============================================================
def find_port() -> str:
    """
    /dev/serial/by-id/ 의 고정 경로를 우선한다.
    /dev/ttyUSB<n> 은 연결 순서에 따라 번호가 바뀌므로 하드코딩하면 깨진다.
    """
    by_id = sorted(glob.glob('/dev/serial/by-id/*FTDI*'))
    if len(by_id) == 1:
        return by_id[0]
    if len(by_id) > 1:
        msg = "FTDI 장치가 여러 개입니다. 하나를 명시하세요:\n"
        for p in by_id:
            msg += f"    {p}  ->  {os.path.realpath(p)}\n"
        raise DxlError(msg)

    cands = [p for p in list_ports.comports() if p.vid == 0x0403]
    if not cands:
        raise DxlError("FTDI 장치를 찾지 못했습니다. "
                       "ls -l /dev/serial/by-id/ 와 lsusb | grep -i ftdi 로 확인하세요.")
    if len(cands) > 1:
        msg = "FTDI 장치가 여러 개입니다:\n"
        for p in cands:
            msg += f"    {p.device}  {p.description}  SN={p.serial_number}\n"
        raise DxlError(msg)
    return cands[0].device


def read_latency_timer(port_path: str) -> Optional[int]:
    """
    FTDI latency timer [ms] 를 읽는다. 조정하지는 않는다.
    기본값 16 ms 는 왕복 지연의 지배적 요인이므로, 루프 주기가 안 나올 때
    가장 먼저 확인할 값이다. 낮추려면:
        echo 1 | sudo tee /sys/bus/usb-serial/devices/<dev>/latency_timer
    """
    dev = os.path.basename(os.path.realpath(port_path))
    sysfs = f"/sys/bus/usb-serial/devices/{dev}/latency_timer"
    try:
        with open(sysfs) as f:
            return int(f.read().strip())
    except Exception:
        return None


# ============================================================
# 모터
# ============================================================
class XM430:
    """
    XM430-W210-R 단일 축. Current-based Position Control Mode 전용.

    상태 전환(CMD / REC / HLD)과 Goal Current 스위칭까지 이 클래스가 담당한다.
    상위 계층(DDS 에이전트, 로컬 콘솔)은 set_goal() 과 update() 만 호출하면 된다.
    """

    ST_CMD, ST_REC, ST_HOLD = 0, 1, 2

    def __init__(self, port_name: str, cfg: MotorConfig = None):
        self.cfg = cfg or MotorConfig()
        self.cfg.validate()
        self.port_name = port_name

        self.port = PortHandler(port_name)
        self.packet = PacketHandler(PROTOCOL_VERSION)
        if not self.port.openPort():
            raise DxlError(f"포트를 열 수 없습니다: {port_name}\n"
                           f"  권한 문제라면: sudo usermod -aG dialout $USER (재로그인)")
        if not self.port.setBaudRate(self.cfg.baudrate):
            raise DxlError(f"baudrate 설정 실패: {self.cfg.baudrate}")

        self.reader = GroupSyncRead(self.port, self.packet, ADDR_MOVING, LEN_FEEDBACK)
        if not self.reader.addParam(self.cfg.dxl_id):
            raise DxlError("GroupSyncRead addParam 실패")

        self._gc_cache = None
        self._goal = 0.0
        self._commanded = False
        self._last_issue = 0.0
        self.state = self.ST_HOLD

    # ---------- 저수준 ----------
    def _chk(self, comm, err, what):
        if comm != COMM_SUCCESS:
            raise DxlError(f"[{what}] {self.packet.getTxRxResult(comm)}")
        if err != 0:
            raise DxlError(f"[{what}] {self.packet.getRxPacketError(err)}")

    def w1(self, a, v, what=""):
        self._chk(*self.packet.write1ByteTxRx(self.port, self.cfg.dxl_id, a, v), what or a)

    def w2(self, a, v, what=""):
        self._chk(*self.packet.write2ByteTxRx(self.port, self.cfg.dxl_id, a, v & 0xFFFF), what or a)

    def w4(self, a, v, what=""):
        self._chk(*self.packet.write4ByteTxRx(self.port, self.cfg.dxl_id, a, v & 0xFFFFFFFF), what or a)

    def r1(self, a, what=""):
        v, c, e = self.packet.read1ByteTxRx(self.port, self.cfg.dxl_id, a)
        self._chk(c, e, what or a)
        return v

    def r2(self, a, what=""):
        v, c, e = self.packet.read2ByteTxRx(self.port, self.cfg.dxl_id, a)
        self._chk(c, e, what or a)
        return v

    def r4(self, a, what=""):
        v, c, e = self.packet.read4ByteTxRx(self.port, self.cfg.dxl_id, a)
        self._chk(c, e, what or a)
        return v

    def torque(self, on: bool):
        self.w1(ADDR_TORQUE_ENABLE, 1 if on else 0, "Torque Enable(64)")

    def _write_goal_position(self, rad: float):
        self.w4(ADDR_GOAL_POSITION, int(round(rad * PULSE_PER_RAD)), "Goal Position(116)")

    def _set_goal_current(self, raw: int):
        """값이 바뀔 때만 쓴다. 불필요한 왕복은 지연만 늘린다."""
        if raw == self._gc_cache:
            return
        self.w2(ADDR_GOAL_CURRENT, raw, "Goal Current(102)")
        self._gc_cache = raw

    # ---------- 초기화 ----------
    def configure(self, verbose: bool = True) -> dict:
        """
        순서가 중요하다. e-Manual 명시:
          - EEPROM(Operating Mode, Current Limit) 은 Torque OFF 에서만 기록 가능
          - Operating Mode 변경 시 Profile Velocity/Acceleration -> 0,
            Goal Current -> Current Limit, Mode 5 면 Position PID 와 PWM Limit 도 리셋
        따라서 프로파일, PID, Goal Current 는 모드 변경 이후에 쓴다.
        """
        c = self.cfg
        model = self.r2(ADDR_MODEL_NUMBER, "Model Number(0)")
        if model != MODEL_XM430_W210:
            raise DxlError(f"Model Number = {model} (기대값 {MODEL_XM430_W210}). "
                           f"ID 또는 포트를 확인하세요.")

        self.torque(False)
        self.w1(ADDR_OPERATING_MODE, OP_CURRENT_BASED_POSITION, "Operating Mode(11)")
        self.w2(ADDR_CURRENT_LIMIT, c.current_limit, "Current Limit(38)")
        self.torque(True)

        self.w2(ADDR_POSITION_P_GAIN, c.p_gain, "Position P Gain(84)")
        self.w2(ADDR_POSITION_I_GAIN, c.i_gain, "Position I Gain(82)")
        self.w2(ADDR_POSITION_D_GAIN, c.d_gain, "Position D Gain(80)")

        # 프로파일이 0 이면 'step'(프로파일 미사용)이 되어 끊긴 동작이 된다. 최소 1 보장.
        vel_lsb = max(1, int(round(c.profile_velocity_dps / DPS_PER_VEL_LSB)))
        acc_lsb = max(1, int(round(c.profile_accel_dpss / DPSS_PER_ACC_LSB)))
        vel_limit = self.r4(ADDR_VELOCITY_LIMIT, "Velocity Limit(44)")
        if vel_lsb > vel_limit:
            print(f"[dxl] 경고: Profile Velocity {vel_lsb} > Velocity Limit {vel_limit}. "
                  f"{vel_limit} 로 제한합니다.")
            vel_lsb = vel_limit
        self.w4(ADDR_PROFILE_ACCELERATION, acc_lsb, "Profile Acceleration(108)")
        self.w4(ADDR_PROFILE_VELOCITY, vel_lsb, "Profile Velocity(112)")

        self._set_goal_current(c.current_holding)

        info = {
            "model": model,
            "operating_mode": self.r1(ADDR_OPERATING_MODE),
            "current_limit": self.r2(ADDR_CURRENT_LIMIT),
            "p_gain": self.r2(ADDR_POSITION_P_GAIN),
            "i_gain": self.r2(ADDR_POSITION_I_GAIN),
            "d_gain": self.r2(ADDR_POSITION_D_GAIN),
            "profile_velocity": self.r4(ADDR_PROFILE_VELOCITY),
            "profile_accel": self.r4(ADDR_PROFILE_ACCELERATION),
            "velocity_limit": vel_limit,
        }
        if verbose:
            self.print_info(info)

        self._goal = self.read().position_rad
        self._write_goal_position(self._goal)
        return info

    def print_info(self, info: dict):
        def tq(raw):
            return estimate_torque_mnm(raw * CURRENT_LSB_TO_A)
        c = self.cfg
        print("── read-back ─────────────────────────────────")
        print(f"  port                     = {self.port_name}")
        print(f"  Model Number(0)          = {info['model']}")
        print(f"  Operating Mode(11)       = {info['operating_mode']}")
        print(f"  Current Limit(38)        = {info['current_limit']}"
              f"  ({info['current_limit'] * 2.69:.0f} mA)")
        for label, raw in (("moving", c.current_moving),
                           ("recover", c.current_recover),
                           ("holding", c.current_holding)):
            print(f"  Goal Current {label:<8s}    = {raw:>4d}"
                  f"  ({raw * 2.69:7.1f} mA, 상한 토크 ≈ {tq(raw):7.1f} mN.m)")
        print(f"  Position P/I/D Gain      = {info['p_gain']} / {info['i_gain']} / {info['d_gain']}")
        print(f"  Profile Velocity(112)    = {info['profile_velocity']} LSB"
              f"  ({info['profile_velocity'] * DPS_PER_VEL_LSB:.1f} deg/s)")
        print(f"  Profile Accel(108)       = {info['profile_accel']} LSB"
              f"  ({info['profile_accel'] * DPSS_PER_ACC_LSB:.0f} deg/s^2)")
        print(f"  Velocity Limit(44)       = {info['velocity_limit']}")
        print("──────────────────────────────────────────────")

    # ---------- 런타임 ----------
    def read(self) -> Feedback:
        """122~135 를 1 회 왕복으로 읽는다."""
        comm = self.reader.txRxPacket()
        if comm != COMM_SUCCESS:
            raise DxlError(f"SyncRead: {self.packet.getTxRxResult(comm)}")
        i = self.cfg.dxl_id
        if not self.reader.isAvailable(i, ADDR_MOVING, LEN_FEEDBACK):
            raise DxlError("SyncRead 데이터 없음")
        mov = self.reader.getData(i, ADDR_MOVING, 1)
        cur = _to_signed(self.reader.getData(i, ADDR_PRESENT_CURRENT, 2), 16)
        vel = _to_signed(self.reader.getData(i, ADDR_PRESENT_VELOCITY, 4), 32)
        pos = _to_signed(self.reader.getData(i, ADDR_PRESENT_POSITION, 4), 32)
        return Feedback(bool(mov),
                        cur * CURRENT_LSB_TO_A,
                        vel * VEL_LSB_TO_RAD_S,
                        pos * RAD_PER_PULSE)

    @property
    def goal_rad(self) -> float:
        return self._goal

    def set_goal(self, rad: float, now: float = 0.0):
        """절대 목표각을 설정하고 궤적을 시작한다."""
        self._goal = rad
        self._set_goal_current(self.cfg.current_moving)
        self._write_goal_position(rad)
        self._commanded = True
        self._last_issue = now
        self.state = self.ST_CMD

    def set_goal_relative(self, delta_rad: float, position_rad: float, now: float = 0.0):
        self.set_goal(position_rad + delta_rad, now)

    def stop_here(self, position_rad: float, now: float = 0.0):
        """현재 위치를 목표로 삼아 즉시 정지. 워치독 만료 시에도 이걸 쓴다."""
        self._goal = position_rad
        self._write_goal_position(position_rad)
        self._set_goal_current(self.cfg.current_holding)
        self._commanded = False
        self._last_issue = now
        self.state = self.ST_HOLD

    def update(self, fb: Feedback, now: float) -> int:
        """
        매 주기 호출한다. 상태를 갱신하고 Goal Current 를 전환한다.

        외란(지령 없이 벌어진 오차) 시에는 궤적 생성기가 놀고 있어
        순수 PID 외란 응답이 되고, 그것이 오버슈트와 진동으로 나타난다.
        Goal Position 을 다시 써서 궤적 생성기를 재기동하면
        Profile Velocity/Acceleration 의 지배를 받아 완만하게 복귀한다.
        """
        c = self.cfg
        err = self._goal - fb.position_rad

        if (not fb.moving) and abs(err) < c.settled_band_rad:
            self._commanded = False

        if self._commanded:
            self.state = self.ST_CMD
            self._set_goal_current(c.current_moving)
        elif abs(err) >= c.settled_band_rad:
            self.state = self.ST_REC
            self._set_goal_current(c.current_recover)
            if abs(err) >= c.disturb_band_rad and (now - self._last_issue) >= c.reissue_interval_s:
                self._write_goal_position(self._goal)
                self._last_issue = now
        else:
            self.state = self.ST_HOLD
            self._set_goal_current(c.current_holding)
        return self.state

    def hardware_error(self) -> int:
        return self.r1(ADDR_HARDWARE_ERROR, "Hardware Error Status(70)")

    def temperature(self) -> int:
        return self.r1(ADDR_PRESENT_TEMPERATURE, "Present Temperature(146)")

    # ---------- 런타임 재튜닝 ----------
    def set_gains(self, p: int = None, i: int = None, d: int = None):
        """
        Position PID Gain 은 RAM 이므로 Torque ON 상태에서도 쓸 수 있다.
        (Operating Mode 를 바꾸면 리셋되므로, 그때는 configure() 를 다시 부를 것.)
        """
        if p is not None:
            self.cfg.p_gain = int(p)
            self.w2(ADDR_POSITION_P_GAIN, self.cfg.p_gain, "Position P Gain(84)")
        if i is not None:
            self.cfg.i_gain = int(i)
            self.w2(ADDR_POSITION_I_GAIN, self.cfg.i_gain, "Position I Gain(82)")
        if d is not None:
            self.cfg.d_gain = int(d)
            self.w2(ADDR_POSITION_D_GAIN, self.cfg.d_gain, "Position D Gain(80)")

    def set_profile(self, velocity_dps: float = None, accel_dpss: float = None):
        """Profile Velocity/Acceleration 도 RAM. 0 이 되지 않도록 최소 1 을 보장한다."""
        if velocity_dps is not None:
            self.cfg.profile_velocity_dps = float(velocity_dps)
            lsb = max(1, int(round(velocity_dps / DPS_PER_VEL_LSB)))
            limit = self.r4(ADDR_VELOCITY_LIMIT)
            self.w4(ADDR_PROFILE_VELOCITY, min(lsb, limit), "Profile Velocity(112)")
        if accel_dpss is not None:
            self.cfg.profile_accel_dpss = float(accel_dpss)
            lsb = max(1, int(round(accel_dpss / DPSS_PER_ACC_LSB)))
            self.w4(ADDR_PROFILE_ACCELERATION, lsb, "Profile Acceleration(108)")

    def set_current_levels(self, moving: int = None, recover: int = None,
                           holding: int = None):
        """
        Goal Current 3 단계 값 갱신. Goal Current(102) 는 RAM 이라 즉시 반영된다.
        Current Limit(38) 은 EEPROM 이라 Torque OFF 가 필요하므로 여기서는 못 바꾼다.
        """
        c = self.cfg
        if moving is not None:
            c.current_moving = int(moving)
        if recover is not None:
            c.current_recover = int(recover)
        if holding is not None:
            c.current_holding = int(holding)
        c.validate()
        self._gc_cache = None      # 다음 update() 에서 강제로 다시 쓰게 한다

    def shutdown(self):
        try:
            self.torque(False)
        except Exception:
            pass
        self.port.closePort()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.shutdown()
        return False
