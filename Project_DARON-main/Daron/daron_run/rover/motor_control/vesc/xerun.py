#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
vesc_driver.py — VESC 구동모터 제어 (UART, RADXA). front/back 2 채널.

ROS·키보드·조향과 무관하다. 상위 계층은 역할 이름으로 드라이버를 만들고
입력 API 와 snapshot() 만 쓴다.

핵심 API
--------
    from motor_control.vesc.xerun import ROLES, make_driver, load_role_config, pair_errors

    cfgs = {r: load_role_config(r, require_motion_ready=False) for r in ROLES}
    problems = pair_errors(cfgs, motion=True)      # 쌍으로서 성립하는지
    motor = make_driver("front", cfgs["front"]).start()
    motor.command("on")            # 준비 확인 후 출력 활성화 (비동기)
    motor.set_direction(+1)        # +1 전진 / -1 후진 / 0 감속 정지
    motor.heartbeat()              # watchdog_timeout_s 안에 계속 호출
    motor.command("off"); motor.close()

역할과 방향 반전
----------------
  전/후방 모터의 장착 방향 반전은 '설정에서 단 한 번'만
  적용된다. 부호가 사는 곳은 ROLES 딕셔너리와 각 json 의 direction_sign 뿐이다.

      ROLES = {"front": +1, "back": +1}   # [261017] VESC 교체 후 back 도 +1
      config/front.json  direction_sign = +1  ->  +ERPM
      config/back.json   direction_sign = +1  ->  +ERPM

  따라서 호출자는 두 드라이버에 '같은 차체 기준 방향'을 그대로 보낸다.
  RoleVescDriver 가 config 의 direction_sign 이 역할의 기대값과 다르면
  생성 자체를 거부하므로, 부호를 두 번 뒤집는 사고가 구조적으로 막힌다.
  호출자는 역할 '이름'만 넘기고 부호를 직접 만지지 않는다.

  모터를 추가할 때는 ROLES 에 항목 하나와 config/<role>.json 만 더하면 된다.

주의
----
  * VESC Tool 에서 모터 검출, 센서 모드, 하드웨어 제한을 먼저 저장해야 한다.
    이 코드는 펌웨어/모터 설정을 쓰지 않는다. Python 쪽 한계값은 감시용이다.
  * hall_setup_confirmed / motor_poles_confirmed 는 작업자의 기록이지
    VESC 를 읽어 검증한 결과가 아니다.
  * command("on") 은 비동기다. snapshot()['control']['enabled'] 로 확인할 것.
  * watchdog_timeout_s(기본 0.5 s) 안에 입력이 없으면 출력이 해제된다.
  * 두 채널의 UART worker 는 독립이며 하드웨어 동기화가 아니다.

단독 테스트 (ROS 없이)
----------------------
    python3 -m motor_control.vesc.xerun --dual --check-config
    python3 -m motor_control.vesc.xerun --dual --readonly --seconds 5
    python3 -m motor_control.vesc.xerun --dual --on --direction 1 --seconds 3
    python3 -m motor_control.vesc.xerun --role front --on --speed 500 --seconds 3

파일 구성
---------
  [1] Config / load_config      설정
  [2] telemetry 해석·연산
  [3] UART 프로토콜·파서
  [4] DriveController           목표속도 계산
  [5] TelemetryLogger           기록
  [6] VescDriver                worker 와 공개 API
  [7] 역할 계층                 ROLES, RoleVescDriver, make_driver, pair_errors
  [8] 단독 테스트 CLI
"""

from __future__ import annotations

import argparse
import itertools
import binascii
import copy
import csv
import json
import math
import queue
import struct
import threading
import time
import uuid
from collections import deque
from dataclasses import asdict, dataclass, fields, replace
from pathlib import Path


MOTOR_MODEL = "Hobbywing XeRun 2848SD 2800KV"
MOTOR_POLES = 4
MOTOR_KV_RPM_PER_V = 2800
MOTOR_SPEC_URL = "https://www.hobbywing.com/en/products/xerun2848"
DEFAULT_CONFIG_PATH = Path(__file__).resolve().parents[2] / "config" / "front.json"


# ============================================================
# [1] 설정 — UART, 구동 속도·가속도, 텔레메트리 환산 파라미터
# ============================================================
@dataclass(frozen=True)
class Config:
    """구동모터 전용 설정. speed_unit은 속도와 가속도·감속도의 기준 단위다."""

    port: str = ""
    baudrate: int = 115200
    mock: bool = False
    enable_motion: bool = False
    hardware_config_confirmed: bool = False
    motor_model: str = MOTOR_MODEL
    sensor_mode: str = "hall"  # dedicated Hall profile; configure VESC separately
    hall_setup_confirmed: bool = False  # operator confirmation, NOT live readback
    firmware_major: int = 7
    firmware_minor: int = 0
    expected_controller_id: int | None = None
    expected_uuid: str | None = None
    speed_unit: str = "rpm"  # rpm or erpm; applies to speed, acceleration, deceleration
    motor_poles: int = MOTOR_POLES
    motor_poles_confirmed: bool = True  # manufacturer specification, not live motor identification
    direction_sign: int = 1
    max_speed: float = 0.0
    max_erpm: float = 0.0
    acceleration: float = 0.0
    deceleration: float = 0.0
    min_voltage_v: float | None = None
    max_voltage_v: float | None = None
    max_motor_current_a: float | None = None
    max_input_current_a: float | None = None
    max_mos_temp_c: float = 70.0
    motor_temp_sensor_present: bool = False
    max_motor_temp_c: float = 70.0
    vesc_timeout_s: float | None = None  # confirmed in VESC Tool; this code does not set it
    control_hz: float = 50.0
    telemetry_hz: float = 20.0
    setup_hz: float = 1.0
    publish_hz: float = 10.0
    watchdog_timeout_s: float = 0.5  # 연결 입력이 끊겼을 때 출력 해제; 0이면 비활성
    telemetry_timeout_s: float = 0.5
    loop_stall_s: float = 0.15
    write_timeout_s: float = 0.02
    reconnect_s: float = 1.0
    torque_constant_nm_per_iq_a: float | None = None
    flux_linkage_wb: float | None = None
    gear_ratio: float | None = None  # motor revolutions / wheel revolution
    wheel_diameter_m: float | None = None
    drivetrain_efficiency: float | None = None
    derivative_filter_tau_s: float = 0.2
    integration_max_gap_s: float = 0.25
    log_directory: str = ""  # load_config resolves relative paths against the config file

    @property
    def pole_pairs(self) -> float | None:
        return None if self.motor_poles is None else self.motor_poles / 2

    def validate(self, *, require_motion_ready: bool = True):
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, float) and not math.isfinite(value):
                raise ValueError(f"{f.name}: non-finite value")
        defaults = Config()
        for name in (
            "mock",
            "enable_motion",
            "hardware_config_confirmed",
            "hall_setup_confirmed",
            "motor_temp_sensor_present",
            "motor_poles_confirmed",
        ):
            if type(getattr(self, name)) is not bool:
                raise ValueError(f"{name}: boolean required")
        for name in ("baudrate", "firmware_major", "firmware_minor", "direction_sign"):
            if type(getattr(self, name)) is not int:
                raise ValueError(f"{name}: integer required")
        for f in fields(self):
            if isinstance(getattr(defaults, f.name), float):
                value = getattr(self, f.name)
                if type(value) not in (int, float) or not math.isfinite(value):
                    raise ValueError(f"{f.name}: finite number required")
        if self.speed_unit not in ("rpm", "erpm") or self.direction_sign not in (-1, 1):
            raise ValueError("speed_unit must be rpm/erpm; direction_sign must be -1/+1")
        if self.sensor_mode != "hall":
            raise ValueError("XeRun 2848SD driver requires sensor_mode='hall'; use its dedicated config")
        if self.motor_model != MOTOR_MODEL:
            raise ValueError(f"This driver is dedicated to {MOTOR_MODEL}; use the XeRun config")
        if type(self.motor_poles) is not int or self.motor_poles != MOTOR_POLES:
            raise ValueError("XeRun 2848SD 2800KV requires motor_poles=4 (2 pole pairs)")
        if self.baudrate <= 0:
            raise ValueError("baudrate must be positive")
        if self.motor_poles is not None and (
            type(self.motor_poles) is not int or self.motor_poles < 2 or self.motor_poles % 2
        ):
            raise ValueError("motor_poles must be a positive even integer >= 2")
        if self.motor_poles_confirmed and self.motor_poles is None:
            raise ValueError("motor_poles required when motor_poles_confirmed=true")
        for name in (
            "min_voltage_v",
            "max_voltage_v",
            "max_motor_current_a",
            "max_input_current_a",
            "vesc_timeout_s",
            "torque_constant_nm_per_iq_a",
            "flux_linkage_wb",
            "gear_ratio",
            "wheel_diameter_m",
            "drivetrain_efficiency",
        ):
            value = getattr(self, name)
            if value is not None and (
                type(value) not in (int, float) or not math.isfinite(value) or value <= 0
            ):
                raise ValueError(f"{name}: null or positive finite number required")
        if self.drivetrain_efficiency is not None and self.drivetrain_efficiency > 1:
            raise ValueError("drivetrain_efficiency must be <= 1")
        if (
            self.min_voltage_v is not None
            and self.max_voltage_v is not None
            and self.min_voltage_v >= self.max_voltage_v
        ):
            raise ValueError("min_voltage_v must be less than max_voltage_v")
        for name in (
            "control_hz",
            "telemetry_hz",
            "setup_hz",
            "publish_hz",
            "telemetry_timeout_s",
            "loop_stall_s",
            "write_timeout_s",
            "reconnect_s",
            "integration_max_gap_s",
            "derivative_filter_tau_s",
        ):
            if getattr(self, name) <= 0:
                raise ValueError(f"{name} must be positive")
        if not 1 <= self.control_hz <= 100 or not 1 <= self.telemetry_hz <= 50:
            raise ValueError("control_hz must be 1..100; telemetry_hz must be 1..50")
        if self.setup_hz > 5 or self.publish_hz > self.control_hz:
            raise ValueError("setup_hz <= 5 and publish_hz <= control_hz required")
        if self.loop_stall_s <= 2 / self.control_hz:
            raise ValueError("require 2/control_hz < loop_stall_s")
        if self.watchdog_timeout_s < 0 or (
            self.watchdog_timeout_s > 0 and self.watchdog_timeout_s <= self.loop_stall_s
        ):
            raise ValueError("watchdog_timeout_s must be 0 or greater than loop_stall_s")
        if self.telemetry_timeout_s <= 2 / self.telemetry_hz:
            raise ValueError("telemetry_timeout_s must exceed two sample periods")
        if self.write_timeout_s >= self.loop_stall_s:
            raise ValueError("write_timeout_s must be shorter than loop_stall_s")
        if self.expected_controller_id is not None and (
            type(self.expected_controller_id) is not int
            or not 0 <= self.expected_controller_id <= 253
        ):
            raise ValueError("expected_controller_id must be null or 0..253")
        if self.expected_uuid is not None and (
            not isinstance(self.expected_uuid, str)
            or len(self.expected_uuid) != 24
            or any(c not in "0123456789abcdef" for c in self.expected_uuid)
        ):
            raise ValueError("expected_uuid must be null or 24 lowercase hex characters")
        for name in ("port", "log_directory"):
            if not isinstance(getattr(self, name), str):
                raise ValueError(f"{name}: string required")
        for name in ("max_speed", "max_erpm", "acceleration", "deceleration"):
            if getattr(self, name) < 0:
                raise ValueError(f"{name}: must be >= 0")
        if self.enable_motion and require_motion_ready:
            errors = self.motion_errors()
            if errors:
                raise ValueError("motion configuration: " + "; ".join(errors))

    def motion_errors(self) -> list[str]:
        errors = []
        if not self.enable_motion:
            errors.append("enable_motion=false")
        if not self.mock and not self.hardware_config_confirmed:
            errors.append(
                "confirm VESC motor detection, current/voltage limits, UART and hardware timeout"
            )
        if self.sensor_mode == "unconfirmed":
            errors.append("sensor_mode required: confirm sensorless or hall in VESC Tool")
        if not self.mock and self.sensor_mode == "hall" and not self.hall_setup_confirmed:
            errors.append("confirm FOC/Hall sensor detection and saved settings in VESC Tool")
        if self.speed_unit == "rpm" and self.pole_pairs is None:
            errors.append("motor_poles required for RPM control (or explicitly use erpm)")
        for name in (
            "max_speed",
            "max_erpm",
            "acceleration",
            "deceleration",
            "min_voltage_v",
            "max_voltage_v",
            "max_motor_current_a",
            "max_input_current_a",
            "vesc_timeout_s",
        ):
            if getattr(self, name) is None or getattr(self, name) <= 0:
                errors.append(f"{name} required")
        factor = (self.pole_pairs or 1) if self.speed_unit == "rpm" else 1
        if self.max_speed * factor > self.max_erpm or self.max_erpm >= (1 << 31):
            errors.append("max_speed conversion must fit max_erpm and signed int32")
        if self.max_erpm != math.floor(self.max_erpm):
            errors.append("max_erpm must be an integer-valued limit")
        if self.vesc_timeout_s is not None and not 2 / self.control_hz < self.vesc_timeout_s <= 1.0:
            errors.append("confirmed VESC timeout must be > two control periods and <= 1 s")
        return errors

    def to_dict(self):
        return asdict(self)


def load_config(path: str | Path = DEFAULT_CONFIG_PATH, *, require_motion_ready: bool = True) -> Config:
    """Always validate types/ranges; allow incomplete motion setup only for inspection.

    VescDriver still performs full validation before starting any worker.
    """
    data = json.loads(Path(path).read_text(encoding="utf-8-sig"))
    if not isinstance(data, dict):
        raise ValueError("config must be a JSON object")
    unknown = data.keys() - {f.name for f in fields(Config)}
    if unknown:
        raise ValueError("unknown config keys: " + ", ".join(sorted(unknown)))
    config = Config(**data)
    config.validate(require_motion_ready=require_motion_ready)
    if config.log_directory and not Path(config.log_directory).is_absolute():
        config = replace(
            config,
            log_directory=str((Path(path).resolve().parent / config.log_directory).resolve()),
        )
    return config




# ============================================================
# [2] telemetry 해석·연산 계층  (구 sensored_telemetry.py)
#
#     decode(payload)        VESC 응답의 필드·부호·배율 해석
#     Estimator.update(...)  RPM·각속도·전력·토크·회전수·거리 등 계산
#     field_units()          수신 데이터의 단위
#     DERIVED_UNITS          계산 결과의 단위
#
#     각속도/축 위치는 구동모터의 회전량이며 조향각이나 굴절각이 아니다.
#     필요한 파라미터가 없는 계산값은 None 이다.
# ============================================================

# ============================================================
# VESC 명령 ID·telemetry 필드 표 — 구동모터 데이터만 해석
# ============================================================
FW_VERSION, GET_VALUES, SET_DUTY, SET_CURRENT, SET_BRAKE, SET_RPM = 0, 4, 5, 6, 7, 8
GET_SETUP, GET_VALUES_SELECTIVE, GET_SETUP_SELECTIVE = 47, 50, 51
ALL_MASK = (1 << 22) - 1
MAX_PAYLOAD = 4096

# One tuple per selective mask bit: ((name, big-endian format, divisor, unit), ...).
RAW_FIELDS = (
    (("temp_mos_c", "h", 10, "degC"),),
    (("temp_motor_c", "h", 10, "degC"),),
    (("motor_current_a", "i", 100, "A"),),
    (("input_current_a", "i", 100, "A"),),
    (("id_a", "i", 100, "A"),),
    (("iq_a", "i", 100, "A"),),
    (("duty", "h", 1000, "ratio"),),
    (("erpm", "i", 1, "electrical_rpm"),),
    (("input_voltage_v", "h", 10, "V"),),
    (("amp_hours", "i", 10000, "Ah"),),
    (("amp_hours_charged", "i", 10000, "Ah"),),
    (("watt_hours", "i", 10000, "Wh"),),
    (("watt_hours_charged", "i", 10000, "Wh"),),
    (("tachometer", "i", 1, "electrical_steps"),),
    (("tachometer_abs", "i", 1, "electrical_steps"),),
    (("fault_code", "B", 1, "enum"),),
    (("pid_position_deg", "i", 1000000, "deg"),),
    (("controller_id", "B", 1, "id"),),
    tuple((f"temp_mos_{i}_c", "h", 10, "degC") for i in (1, 2, 3)),
    (("vd_v", "i", 1000, "V"),),
    (("vq_v", "i", 1000, "V"),),
    (("timeout_kill_status", "B", 1, "bitfield"),),
)
SETUP_FIELDS = (
    RAW_FIELDS[0],
    RAW_FIELDS[1],
    RAW_FIELDS[2],
    RAW_FIELDS[3],
    RAW_FIELDS[6],
    RAW_FIELDS[7],
    (("speed_mps", "i", 1000, "m/s"),),
    RAW_FIELDS[8],
    (("battery_level", "h", 1000, "ratio"),),
    RAW_FIELDS[9],
    RAW_FIELDS[10],
    RAW_FIELDS[11],
    RAW_FIELDS[12],
    (("distance_m", "i", 1000, "m"),),
    (("distance_abs_m", "i", 1000, "m"),),
    RAW_FIELDS[16],
    RAW_FIELDS[15],
    RAW_FIELDS[17],
    (("controller_count", "B", 1, "count"),),
    (("battery_wh_remaining", "i", 1000, "Wh"),),
    (("odometer_m", "I", 1, "m"),),
    (("uptime_ms", "I", 1, "ms"),),
)
FAULT_NAMES = (
    "NONE OVER_VOLTAGE UNDER_VOLTAGE DRV ABS_OVER_CURRENT OVER_TEMP_FET OVER_TEMP_MOTOR "
    "GATE_DRIVER_OVER_VOLTAGE GATE_DRIVER_UNDER_VOLTAGE MCU_UNDER_VOLTAGE "
    "BOOTING_FROM_WATCHDOG_RESET ENCODER_SPI ENCODER_SINCOS_BELOW_MIN_AMPLITUDE "
    "ENCODER_SINCOS_ABOVE_MAX_AMPLITUDE FLASH_CORRUPTION HIGH_OFFSET_CURRENT_SENSOR_1 "
    "HIGH_OFFSET_CURRENT_SENSOR_2 HIGH_OFFSET_CURRENT_SENSOR_3 UNBALANCED_CURRENTS BRK "
    "RESOLVER_LOT RESOLVER_DOS RESOLVER_LOS FLASH_CORRUPTION_APP_CFG FLASH_CORRUPTION_MC_CFG "
    "ENCODER_NO_MAGNET ENCODER_MAGNET_TOO_STRONG PHASE_FILTER ENCODER_FAULT LV_OUTPUT_FAULT "
    "ENCODER_SLIP OVERSPEED UNDERSPEED ABS_OVERSPEED"
).split()


# ============================================================
# 수신 데이터 해석 — wire 정수와 물리 단위 값을 함께 보존
# ============================================================
def decode(payload: bytes) -> tuple[str, dict]:
    if not payload:
        raise ValueError("empty payload")
    command = payload[0]
    if command == FW_VERSION:
        if len(payload) < 4:
            raise ValueError("truncated firmware response")
        end = payload.find(b"\0", 3)
        if end < 4 or len(payload) < end + 13:
            raise ValueError("firmware hardware name/UUID missing")
        return "firmware", {
            "major": payload[1],
            "minor": payload[2],
            "hardware": payload[3:end].decode("utf-8", errors="replace"),
            "uuid": payload[end + 1 : end + 13].hex(),
            "extra_hex": payload[end + 13 :].hex(),
        }
    if command not in (GET_VALUES, GET_VALUES_SELECTIVE, GET_SETUP, GET_SETUP_SELECTIVE):
        return "unknown", {"command_id": command, "payload_hex": payload.hex()}
    selective = command in (GET_VALUES_SELECTIVE, GET_SETUP_SELECTIVE)
    setup = command in (GET_SETUP, GET_SETUP_SELECTIVE)
    offset = 5 if selective else 1
    if len(payload) < offset:
        raise ValueError("missing mask")
    mask = int.from_bytes(payload[1:5], "big") if selective else ALL_MASK
    if mask & ~ALL_MASK:
        raise ValueError("unknown telemetry mask; firmware layout needs review")
    schema = SETUP_FIELDS if setup else RAW_FIELDS
    values, raw_integers = {}, {}
    try:
        for bit, group in enumerate(schema):
            for name, fmt, divisor, _unit in group:
                if mask & (1 << bit):
                    value = struct.unpack_from(">" + fmt, payload, offset)[0]
                    offset += struct.calcsize(fmt)
                    raw_integers[name] = value
                    values[name] = value if divisor == 1 else value / divisor
                else:
                    values[name] = None
    except struct.error as exc:
        raise ValueError("truncated telemetry response") from exc
    if offset != len(payload):
        raise ValueError("unexpected telemetry bytes; firmware layout needs review")
    fault = values.get("fault_code")
    values["fault_name"] = (
        FAULT_NAMES[fault] if fault is not None and fault < len(FAULT_NAMES) else f"UNKNOWN_{fault}"
    )
    if not setup:
        status = values["timeout_kill_status"]
        values["timeout"] = None if status is None else bool(status & 1)
        values["kill_switch_active"] = None if status is None else bool(status & 2)
    return "setup" if setup else "raw", {
        "mask": mask,
        "complete": mask == ALL_MASK,
        "values": values,
        "wire_integers": raw_integers,
        "payload_hex": payload.hex(),
    }


def field_units(setup: bool = False) -> dict[str, str]:
    return {
        name: unit
        for group in (SETUP_FIELDS if setup else RAW_FIELDS)
        for name, _fmt, _scale, unit in group
    }


def sensor_status(config: Config) -> dict:
    """Configuration provenance only: never fabricate Hall data from RPM/tachometer.

    COMM_GET_VALUES[_SELECTIVE] does not report Hall H1/H2/H3 bits, the Hall table,
    or whether the controller currently uses Hall rather than the high-speed observer.
    Reading RPM successfully must never be presented as proof of Hall operation.
    """
    return {
        "motor_model": config.motor_model,
        "manufacturer_pole_count": MOTOR_POLES,
        "kv_rating_rpm_per_v": MOTOR_KV_RPM_PER_V,
        "manufacturer_spec_url": MOTOR_SPEC_URL,
        "requested_mode": config.sensor_mode,
        "expected_motor_control": "FOC",
        "setup_confirmed_by_operator": (
            config.hall_setup_confirmed
            if config.sensor_mode == "hall"
            else config.hardware_config_confirmed
            if config.sensor_mode == "sensorless"
            else False
        ),
        "setup_confirmation_source": (
            "hall_setup_confirmed"
            if config.sensor_mode == "hall"
            else "hardware_config_confirmed"
            if config.sensor_mode == "sensorless"
            else "unconfirmed"
        ),
        "configuration_read_back": False,
        "active_feedback": None,
        "hall_states": None,
        "hall_states_available": False,
        "temperature_sensor_configured": config.motor_temp_sensor_present,
        "reason": "standard UART telemetry does not include live Hall states or active sensor mode",
        "mock": config.mock,
    }


# ============================================================
# 구동모터 연산 — 단위, 누적량, RPM·토크·전력·휠 환산
# ============================================================
DERIVED_UNITS = {
    "electrical_frequency_hz": "Hz",
    "motor_rpm": "rpm",
    "motor_rad_s": "rad/s",
    "motor_rpm_forward": "rpm",
    "motor_acceleration_rpm_s": "rpm/s",
    "motor_acceleration_filtered_rpm_s": "rpm/s",
    "motor_position_rad": "rad",
    "motor_revolutions": "rev",
    "motor_revolutions_abs": "rev",
    "wheel_rpm": "rpm",
    "wheel_rad_s": "rad/s",
    "wheel_position_rad": "rad",
    "wheel_speed_mps": "m/s",
    "wheel_acceleration_mps2": "m/s^2",
    "wheel_distance_m": "m",
    "wheel_distance_abs_m": "m",
    "motor_torque_est_nm": "N*m",
    "wheel_torque_est_nm": "N*m",
    "wheel_force_est_n": "N",
    "electrical_input_power_w": "W",
    "dq_power_est_w": "W",
    "mechanical_power_est_w": "W",
    "motor_efficiency_est_ratio": "ratio",
    "dq_current_magnitude_a": "A",
    "dq_voltage_magnitude_v": "V",
    "duty_percent": "%",
    "net_charge_ah": "Ah",
    "net_energy_wh": "Wh",
    "host_energy_consumed_wh": "Wh",
    "host_energy_returned_wh": "Wh",
    "host_net_energy_wh": "Wh",
    "host_charge_consumed_ah": "Ah",
    "host_charge_returned_ah": "Ah",
    "host_integrated_duration_s": "s",
    "host_skipped_duration_s": "s",
    "sample_dt_s": "s",
    "rpm_tracking_error": "rpm",
    "erpm_tracking_error": "electrical_rpm",
}


def signed_delta32(current: int, previous: int) -> int:
    return ((current - previous + (1 << 31)) % (1 << 32)) - (1 << 31)


def split_integral(a: float, b: float, dt: float) -> tuple[float, float]:
    """Positive and negative magnitudes of a linearly interpolated signal integral."""
    if a >= 0 and b >= 0:
        return (a + b) * dt / 2, 0.0
    if a <= 0 and b <= 0:
        return 0.0, -(a + b) * dt / 2
    first_dt = dt * abs(a) / (abs(a) + abs(b))
    first, second = abs(a) * first_dt / 2, abs(b) * (dt - first_dt) / 2
    return (first, second) if a > 0 else (second, first)


class Estimator:
    """구동모터 telemetry를 환산·누적한다. 토크·휠 주행량은 추정값이다."""

    def __init__(self, config: Config):
        self.cfg = config
        self.previous: dict | None = None
        self.previous_time: float | None = None
        self.last_uptime: int | None = None
        self.filtered_accel: float | None = None
        self.motor_rev = self.motor_rev_abs = 0.0
        self.energy_in = self.energy_out = self.charge_in = self.charge_out = 0.0
        self.integrated_s = self.skipped_s = 0.0
        self.baseline_resets = 0

    def reset_baseline(self):
        self.previous = None
        self.previous_time = None
        self.filtered_accel = None
        self.baseline_resets += 1

    def observe_uptime(self, uptime_ms: int) -> bool:
        # uint32 clock rollover is normal; a backwards non-wrap is a reboot.
        reboot = self.last_uptime is not None and signed_delta32(uptime_ms, self.last_uptime) < 0
        self.last_uptime = uptime_ms
        if reboot:
            self.reset_baseline()
        return reboot

    def update(self, raw: dict, now: float, target_erpm: int = 0) -> dict:
        c = self.cfg
        v = {name: None for name in DERIVED_UNITS}
        missing, warnings = {}, []

        def calc(name, dependencies, formula, reason="raw telemetry field unavailable"):
            if all(x is not None and math.isfinite(x) for x in dependencies):
                try:
                    result = formula(*dependencies)
                    if math.isfinite(result):
                        v[name] = result
                    else:
                        missing[name] = "calculation not finite"
                except (OverflowError, ZeroDivisionError):
                    missing[name] = "calculation out of range"
            else:
                missing[name] = reason

        erpm = raw.get("erpm")
        pairs = c.pole_pairs
        if pairs is not None and not c.motor_poles_confirmed:
            warnings.append(
                f"motor_poles_unverified: assuming {c.motor_poles} poles; "
                "RPM and pole-dependent values are provisional"
            )
        calc("electrical_frequency_hz", [erpm], lambda x: x / 60)
        calc("motor_rpm", [erpm, pairs], lambda e, p: e / p, "motor_poles or ERPM unavailable")
        calc("motor_rad_s", [v["motor_rpm"]], lambda x: x * math.tau / 60)
        calc("motor_rpm_forward", [v["motor_rpm"]], lambda x: x * c.direction_sign)
        calc("erpm_tracking_error", [erpm], lambda x: target_erpm - x)
        calc(
            "rpm_tracking_error",
            [v["erpm_tracking_error"], pairs],
            lambda x, p: x / p * c.direction_sign,
        )
        calc("duty_percent", [raw.get("duty")], lambda x: x * 100)
        calc("dq_current_magnitude_a", [raw.get("id_a"), raw.get("iq_a")], math.hypot)
        calc("dq_voltage_magnitude_v", [raw.get("vd_v"), raw.get("vq_v")], math.hypot)
        calc(
            "electrical_input_power_w",
            [raw.get("input_voltage_v"), raw.get("input_current_a")],
            lambda u, i: u * i,
        )
        calc(
            "dq_power_est_w",
            [raw.get("vd_v"), raw.get("id_a"), raw.get("vq_v"), raw.get("iq_a")],
            lambda vd, i_d, vq, iq: 1.5 * (vd * i_d + vq * iq),
        )
        kt = c.torque_constant_nm_per_iq_a
        torque_method = "configured_Kt_times_Iq"
        if kt is None and pairs is not None and c.flux_linkage_wb is not None:
            kt = 1.5 * pairs * c.flux_linkage_wb
            torque_method = "1.5_pole_pairs_flux_times_Iq"
        calc(
            "motor_torque_est_nm",
            [kt, raw.get("iq_a")],
            lambda k, i: k * i,
            "Iq-compatible Kt, or motor_poles + flux_linkage_wb required",
        )
        calc(
            "mechanical_power_est_w",
            [v["motor_torque_est_nm"], v["motor_rad_s"]],
            lambda t, w: t * w,
        )
        pin, pm = v["electrical_input_power_w"], v["mechanical_power_est_w"]
        # Diagnostic estimate only: suppress near-zero, opposite-power and >100% cases.
        if pin is not None and pm is not None:
            eta = pm / pin if pin > 1 and pm > 0 else (pin / pm if pin < -1 and pm < -1 else None)
            if eta is not None and 0 <= eta <= 1:
                v["motor_efficiency_est_ratio"] = eta
            else:
                missing["motor_efficiency_est_ratio"] = (
                    "near zero/inconsistent power or estimate outside 0..1"
                )
        calc(
            "net_charge_ah",
            [raw.get("amp_hours"), raw.get("amp_hours_charged")],
            lambda a, b: a - b,
        )
        calc(
            "net_energy_wh",
            [raw.get("watt_hours"), raw.get("watt_hours_charged")],
            lambda a, b: a - b,
        )
        calc(
            "wheel_rpm",
            [v["motor_rpm_forward"], c.gear_ratio],
            lambda n, g: n / g,
            "motor_poles and gear_ratio required",
        )
        calc("wheel_rad_s", [v["wheel_rpm"]], lambda n: n * math.tau / 60)
        calc(
            "wheel_speed_mps",
            [v["wheel_rad_s"], c.wheel_diameter_m],
            lambda w, d: w * d / 2,
            "motor_poles, gear_ratio and wheel_diameter_m required",
        )
        # Backdriving: |wheel power| = |motor power| / efficiency, not multiplied.
        # ERPM and shaft speed have the same sign even without a known pole count.
        # Do not infer motoring from unavailable mechanical power / rotation data.
        calc(
            "wheel_torque_est_nm",
            [v["motor_torque_est_nm"], c.gear_ratio, c.drivetrain_efficiency, erpm],
            lambda t, g, e, n: t * g * (1 / e if t * n < 0 else e) * c.direction_sign,
            "motor torque, ERPM, gear_ratio and drivetrain_efficiency required",
        )
        calc(
            "wheel_force_est_n",
            [v["wheel_torque_est_nm"], c.wheel_diameter_m],
            lambda t, d: 2 * t / d,
            "wheel torque and wheel_diameter_m required",
        )

        dt = None if self.previous_time is None else now - self.previous_time
        v["sample_dt_s"] = dt
        continuous = dt is not None and 0 < dt <= c.integration_max_gap_s
        if dt is not None and dt <= 0:
            raise ValueError("telemetry time must increase")
        if dt is not None and not continuous:
            self.skipped_s += dt
            self.filtered_accel = None
            warnings.append("sample_gap: derivative/integration/tachometer increment skipped")
        previous = self.previous
        if continuous and previous is not None:
            if pairs is not None and erpm is not None and previous.get("erpm") is not None:
                acc = (erpm - previous["erpm"]) / pairs / dt
                self.filtered_accel = (
                    acc
                    if self.filtered_accel is None
                    else self.filtered_accel
                    + (dt / (c.derivative_filter_tau_s + dt)) * (acc - self.filtered_accel)
                )
                v["motor_acceleration_rpm_s"] = acc
                v["motor_acceleration_filtered_rpm_s"] = self.filtered_accel
                calc(
                    "wheel_acceleration_mps2",
                    [acc, c.gear_ratio, c.wheel_diameter_m],
                    lambda a, g, d: a * c.direction_sign / g * math.pi * d / 60,
                )
            p0 = previous.get("_power")
            i0, i1 = previous.get("input_current_a"), raw.get("input_current_a")
            if p0 is not None and pin is not None and i0 is not None and i1 is not None:
                ein, eout = split_integral(p0, pin, dt)
                qin, qout = split_integral(i0, i1, dt)
                self.energy_in += ein / 3600
                self.energy_out += eout / 3600
                self.charge_in += qin / 3600
                self.charge_out += qout / 3600
                self.integrated_s += dt
            else:
                self.skipped_s += dt
            if pairs is not None and erpm is not None and previous.get("erpm") is not None:
                bound = max(abs(erpm), abs(previous["erpm"])) / 10 * dt * 4 + 20
                for counter, attr, absolute in (
                    ("tachometer", "motor_rev", False),
                    ("tachometer_abs", "motor_rev_abs", True),
                ):
                    if raw.get(counter) is None or previous.get(counter) is None:
                        continue
                    delta = signed_delta32(raw[counter], previous[counter])
                    if abs(delta) <= bound and (not absolute or delta >= 0):
                        setattr(self, attr, getattr(self, attr) + delta / (6 * pairs))
                    else:
                        self.baseline_resets += 1
                        warnings.append(f"{counter}: reset/implausible jump; increment skipped")
        if pairs is not None and raw.get("tachometer") is not None:
            v["motor_revolutions"] = self.motor_rev
            v["motor_position_rad"] = self.motor_rev * math.tau
            calc(
                "wheel_position_rad",
                [v["motor_position_rad"], c.gear_ratio],
                lambda a, g: a / g * c.direction_sign,
            )
            calc(
                "wheel_distance_m",
                [v["wheel_position_rad"], c.wheel_diameter_m],
                lambda a, d: a * d / 2,
            )
        if pairs is not None and raw.get("tachometer_abs") is not None:
            v["motor_revolutions_abs"] = self.motor_rev_abs
            calc(
                "wheel_distance_abs_m",
                [self.motor_rev_abs, c.gear_ratio, c.wheel_diameter_m],
                lambda n, g, d: n / g * math.pi * d,
            )
        v.update(
            host_energy_consumed_wh=self.energy_in,
            host_energy_returned_wh=self.energy_out,
            host_net_energy_wh=self.energy_in - self.energy_out,
            host_charge_consumed_ah=self.charge_in,
            host_charge_returned_ah=self.charge_out,
            host_integrated_duration_s=self.integrated_s,
            host_skipped_duration_s=self.skipped_s,
        )
        for name, value in v.items():
            if value is None and name not in missing:
                missing[name] = "required data/parameters or consecutive samples unavailable"
        self.previous = dict(raw, _power=pin)
        self.previous_time = now
        return {
            "values": v,
            "unavailable": missing,
            "warnings": warnings,
            "torque_method": torque_method if v["motor_torque_est_nm"] is not None else None,
            "baseline_resets": self.baseline_resets,
            "distance_kind": "wheel-equivalent estimate; not measured ground travel",
        }



# ============================================================
# [3] VESC UART 프로토콜 — CRC, 프레임, ERPM 명령, 수신 재조립
# ============================================================
def crc16(data: bytes) -> int:
    return binascii.crc_hqx(data, 0)


def next_deadline(previous: float, now: float, period: float) -> float:
    """Keep the requested average rate; skip missed ticks instead of replaying them."""
    due = previous + period
    return due if due > now else now + period


def frame(payload: bytes) -> bytes:
    if not 1 <= len(payload) <= MAX_PAYLOAD:
        raise ValueError("payload length must be 1..4096")
    header = (
        bytes((2, len(payload)))
        if len(payload) <= 255
        else b"\x03" + struct.pack(">H", len(payload))
    )
    return header + payload + struct.pack(">H", crc16(payload)) + b"\x03"


def set_rpm(erpm: int) -> bytes:
    if type(erpm) is not int or not -(1 << 31) <= erpm < (1 << 31):
        raise ValueError("ERPM must be signed int32")
    return struct.pack(">Bi", SET_RPM, erpm)


def set_current(amperes: float) -> bytes:
    if not math.isfinite(amperes):
        raise ValueError("non-finite current")
    value = math.trunc(amperes * 1000)
    if not -(1 << 31) <= value < (1 << 31):
        raise ValueError("current overflow")
    return struct.pack(">Bi", SET_CURRENT, value)


def request_values(setup: bool = False, mask: int = ALL_MASK) -> bytes:
    if type(mask) is not int or not 0 < mask <= ALL_MASK:
        raise ValueError("unsupported selective mask")
    return struct.pack(">BI", GET_SETUP_SELECTIVE if setup else GET_VALUES_SELECTIVE, mask)


@dataclass
class ParserStats:
    frames_ok: int = 0
    crc_errors: int = 0
    framing_errors: int = 0
    discarded_bytes: int = 0
    partial_timeouts: int = 0


class PacketParser:
    """Incremental bounded parser; inter-byte expiry prevents a bad length pinning RX.

    Never scans inside an incomplete valid frame: a payload can itself contain a
    valid framed packet. Recover corrupt lengths on the next inter-byte expiry.
    """

    def __init__(self, inter_byte_timeout: float = 0.1):
        self.buffer = bytearray()
        self.stats = ParserStats()
        self.timeout = inter_byte_timeout
        self.last_byte_time: float | None = None

    def reset(self):
        self.buffer.clear()
        self.last_byte_time = None

    def feed(self, data: bytes, now: float | None = None) -> list[bytes]:
        now = time.monotonic() if now is None else now
        expired = bool(
            self.buffer
            and self.last_byte_time is not None
            and now - self.last_byte_time > self.timeout
        )
        old_bytes = len(self.buffer)
        if expired:
            # Discard the stalled candidate start, then recover any following frame.
            del self.buffer[0]
            old_bytes -= 1
            self.stats.partial_timeouts += 1
            self.stats.discarded_bytes += 1
        self.buffer.extend(data)
        if data:
            self.last_byte_time = now
        result = []
        while self.buffer:
            if self.buffer[0] not in (2, 3):
                del self.buffer[0]
                old_bytes = max(0, old_bytes - 1)
                self.stats.discarded_bytes += 1
                continue
            header = 2 if self.buffer[0] == 2 else 3
            if len(self.buffer) < header:
                break
            length = int.from_bytes(self.buffer[1:header], "big")
            if not 0 < length <= MAX_PAYLOAD or (header == 3 and length < 256):
                self.stats.framing_errors += 1
                del self.buffer[0]
                old_bytes = max(0, old_bytes - 1)
                continue
            total = header + length + 3
            if len(self.buffer) < total:
                if expired and old_bytes > 0:
                    del self.buffer[0]
                    old_bytes -= 1
                    self.stats.discarded_bytes += 1
                    continue
                break
            payload = bytes(self.buffer[header : header + length])
            if self.buffer[total - 1] != 3:
                self.stats.framing_errors += 1
                del self.buffer[0]
                old_bytes = max(0, old_bytes - 1)
                continue
            if crc16(payload) != int.from_bytes(self.buffer[header + length : total - 1], "big"):
                self.stats.crc_errors += 1
                del self.buffer[0]
                old_bytes = max(0, old_bytes - 1)
                continue
            result.append(payload)
            self.stats.frames_ok += 1
            del self.buffer[:total]
            old_bytes = max(0, old_bytes - total)
        return result


# ============================================================
# 구동 제어 — 최신 방향 입력, 일정 가속, 감속 정지, 출력 상태
# ============================================================
# ============================================================
# [4] DriveController — 목표속도 계산
# ============================================================
class DriveController:
    """위치/각도 제어 없이 구동모터의 목표속도만 계산한다."""

    def __init__(self, config: Config):
        self.cfg = config
        self.enabled = False
        self.fault = None
        self.last_error = ""
        self.direction = 0
        self.target_speed = 0.0
        self.acceleration = config.acceleration
        self.stopping = False
        self.last_contact = self.last_tick = self.enabled_at = None
        self.raw = self.raw_time = self.firmware = None

    @property
    def target_erpm(self) -> int:
        factor = (self.cfg.pole_pairs or 1) if self.cfg.speed_unit == "rpm" else 1
        value = self.target_speed * factor * self.cfg.direction_sign
        return int(math.copysign(math.floor(abs(value) + 0.5), value))

    def trip(self, reason):
        self.enabled = False
        self.fault = reason
        self.direction = 0
        self.target_speed = 0.0
        self.stopping = False

    def disconnect(self):
        if self.enabled:
            self.trip("UART disconnected")
        self.enabled = False
        self.raw = self.raw_time = self.firmware = None
        self.direction = 0
        self.target_speed = 0.0
        self.stopping = False

    def raw_problem(self):
        r, c = self.raw, self.cfg
        if r is None:
            return "no raw telemetry"
        for key in (
            "erpm",
            "input_voltage_v",
            "motor_current_a",
            "input_current_a",
            "temp_mos_c",
            "fault_code",
        ):
            if r.get(key) is None or not math.isfinite(r[key]):
                return f"missing/invalid {key}"
        if r["fault_code"]:
            return f"VESC fault: {r.get('fault_name', r['fault_code'])}"
        if r.get("kill_switch_active"):
            return "VESC kill switch"
        if (
            c.expected_controller_id is not None
            and r.get("controller_id") != c.expected_controller_id
        ):
            # [261016] 실제로 읽힌 ID 를 같이 보여 준다 (포트가 바뀐 것인지, VESC 설정이 바뀐 것인지 구분)
            return (f"controller ID mismatch (reported {r.get('controller_id')}, "
                    f"expected {c.expected_controller_id})")
        for key, low, high in (
            ("input_voltage_v", c.min_voltage_v, c.max_voltage_v),
            ("temp_mos_c", None, c.max_mos_temp_c),
            ("temp_motor_c", None, c.max_motor_temp_c if c.motor_temp_sensor_present else None),
        ):
            value = r.get(key)
            if high is not None and (value is None or not math.isfinite(value) or value > high):
                return f"{key} unavailable/over limit"
            if low is not None and (value is None or value < low):
                return f"{key} below limit"
        for key, limit in (
            ("motor_current_a", c.max_motor_current_a),
            ("input_current_a", c.max_input_current_a),
        ):
            if limit is not None and abs(r[key]) > limit:
                return f"{key} exceeds monitoring limit"
        if c.max_erpm and abs(r["erpm"]) > c.max_erpm * 1.1 + 10:
            return "measured ERPM over limit"
        return None

    def observe_raw(self, raw, now):
        self.raw, self.raw_time = raw, now
        problem = self.raw_problem()
        if problem:
            self.trip(problem)

    def readiness(self, now):
        errors = self.cfg.motion_errors()
        fw = self.firmware
        if fw is None or (fw["major"], fw["minor"]) != (
            self.cfg.firmware_major,
            self.cfg.firmware_minor,
        ):
            errors.append("firmware version unverified/mismatch")
        if self.cfg.expected_uuid is not None and (
            fw is None or fw["uuid"] != self.cfg.expected_uuid
        ):
            errors.append("VESC UUID mismatch")
        if self.raw_time is None or now - self.raw_time > self.cfg.telemetry_timeout_s:
            errors.append("telemetry stale/missing")
        problem = self.raw_problem()
        if problem:
            errors.append(problem)
        return errors

    def heartbeat(self, now):
        self.last_contact = now

    def set_direction(self, direction, now):
        """+1/-1 applies the configured constant acceleration; 0 decelerates to rest."""
        if type(direction) is not int or direction not in (-1, 0, 1):
            raise ValueError("direction must be -1, 0 or +1")
        self.heartbeat(now)
        if not self.enabled:
            if direction:
                raise ValueError("send 'on' first")
            return
        self.direction = direction
        self.stopping = direction == 0

    def set_speed(self, speed, now):
        """Direct speed setpoint for local/ROS tests without the keyboard console."""
        if (
            type(speed) not in (int, float)
            or not math.isfinite(speed)
            or abs(speed) > self.cfg.max_speed
        ):
            raise ValueError("goal speed must be finite and within max_speed")
        if not self.enabled:
            raise ValueError("send 'on' first")
        self.heartbeat(now)
        self.direction = 0
        self.target_speed = float(speed)
        self.stopping = False

    def set_acceleration(self, value, now):
        if (
            type(value) not in (int, float)
            or not math.isfinite(value)
            or not 0 < value <= self.cfg.acceleration
        ):
            raise ValueError("acceleration must be positive and <= configured acceleration limit")
        self.acceleration = float(value)
        self.heartbeat(now)

    def command(self, name, now):
        if name not in ("on", "off", "stop"):
            raise ValueError("command must be on, off or stop")
        if name == "on":
            if self.enabled:
                self.heartbeat(now)
                return
            errors = self.readiness(now)
            if errors:
                raise ValueError("; ".join(errors))
            self.enabled = True
            self.enabled_at = self.last_tick = now
            self.fault = None
            self.target_speed = 0.0
            self.direction = 0
            self.stopping = False
        elif name == "off":
            self.enabled = False
            self.direction = 0
            self.target_speed = 0.0
            self.stopping = False
        else:
            self.direction = 0
            self.stopping = True
        self.heartbeat(now)
        self.last_error = ""

    def check_timeouts(self, now):
        if not self.enabled:
            return
        if self.last_tick is not None and now - self.last_tick > self.cfg.loop_stall_s:
            self.trip("control loop stalled")
        elif self.cfg.watchdog_timeout_s > 0 and (
            self.last_contact is None or now - self.last_contact > self.cfg.watchdog_timeout_s
        ):
            self.trip("connection watchdog timeout")
        elif self.raw_time is None or now - self.raw_time > self.cfg.telemetry_timeout_s:
            self.trip("raw telemetry timeout")
        elif (
            self.raw.get("timeout")
            and self.enabled_at is not None
            and now - self.enabled_at > 2 / self.cfg.telemetry_hz
        ):
            self.trip("VESC command timeout flag")

    def step(self, now):
        self.check_timeouts(now)
        dt = 0 if self.last_tick is None else max(0.0, now - self.last_tick)
        self.last_tick = now
        if not self.enabled:
            return None  # release output with zero current
        if self.stopping:
            self.target_speed = math.copysign(
                max(0.0, abs(self.target_speed) - self.cfg.deceleration * dt), self.target_speed
            )
        else:
            self.target_speed += self.direction * self.acceleration * dt
            self.target_speed = max(-self.cfg.max_speed, min(self.cfg.max_speed, self.target_speed))
        return self.target_erpm if self.target_speed else None

    def status(self, now):
        return {
            "state": "FAULT" if self.fault else ("ON" if self.enabled else "OFF"),
            "enabled": self.enabled,
            "fault": self.fault,
            "last_error": self.last_error,
            "direction": self.direction,
            "target_speed": self.target_speed,
            "target_erpm": self.target_erpm,
            "speed_unit": self.cfg.speed_unit,
            "max_speed": self.cfg.max_speed,
            "fixed_acceleration": self.acceleration,
            "max_acceleration": self.cfg.acceleration,
            "deceleration": self.cfg.deceleration,
            "stopping": self.stopping,
            "on_blockers": self.readiness(now),
        }


# ============================================================
# 장치 연결 — RADXA UART (Mock은 experiments/에서만 제공)
# ============================================================
def open_serial(config: Config):
    if config.mock:
        try:
            from experiments.mock_vesc import MockTransport   # 옛 experiments/ 패키지 — 이 저장소에는 없다
        except ImportError as exc:
            raise RuntimeError(
                "mock VESC transport (experiments/mock_vesc.py) 가 이 저장소에 없습니다 — "
                "config mock:true / drive_keys.py --mock 은 쓸 수 없습니다. 실제 UART 로 돌리세요.") from exc

        voltage = (
            (config.min_voltage_v + config.max_voltage_v) / 2
            if config.min_voltage_v is not None and config.max_voltage_v is not None
            else 11.1
        )
        return MockTransport(input_voltage_v=voltage, motor_poles=MOTOR_POLES)
    if not config.port:
        raise ValueError("port is required: specify the RADXA UART device explicitly")
    try:
        import serial
    except ImportError as exc:
        raise RuntimeError(
            "pySerial missing: install python3-serial or python3 -m pip install -r requirements.txt"
        ) from exc
    import os

    return serial.Serial(
        config.port,
        config.baudrate,
        timeout=0,
        write_timeout=config.write_timeout_s,
        exclusive=True if os.name == "posix" else None,
        xonxoff=False,
        rtscts=False,
        dsrdtr=False,
    )


# ============================================================
# 선택적 기록 — UART 제어와 분리된 JSONL·CSV 저장
# ============================================================
# ============================================================
# [5] TelemetryLogger — 기록
# ============================================================
class TelemetryLogger:
    """Bounded asynchronous JSONL + flat CSV. Never blocks the UART worker on disk."""

    def __init__(self, directory: str, config: Config, *, session_id=None):
        self.pending = queue.Queue(maxsize=512)
        self.dropped = 0
        self.written = 0
        self.finished = False
        self.error = ""
        self.path = ""
        self.thread = None
        self.closed = threading.Event()
        if directory:
            folder = Path(directory).resolve()
            folder.mkdir(parents=True, exist_ok=True)
            self.path = str(folder / (time.strftime("vesc_%Y%m%d_%H%M%S_") + uuid.uuid4().hex[:8]))
            self.thread = threading.Thread(target=self._run, name="vesc-log", daemon=True)
            self.thread.start()
            self.put(
                {
                    "kind": "metadata",
                    "session_id": session_id,
                    "started_unix_ns": time.time_ns(),
                    "config": config.to_dict(),
                    "raw_units": field_units(),
                    "setup_units": field_units(True),
                    "derived_units": DERIVED_UNITS,
                    "mock": config.mock,
                    "schema_version": 2,
                    "recording": "all decoded UART samples, not downsampled ROS publications",
                    "power_scope": "VESC DC input only; excludes separately powered devices",
                }
            )

    def put(self, record: dict):
        if self.thread is None:
            return
        if self.error or self.closed.is_set():
            self.dropped += 1
            return
        try:
            self.pending.put_nowait(record)
        except queue.Full:
            self.dropped += 1

    def status(self):
        return {
            "enabled": self.thread is not None,
            "path_prefix": self.path,
            "dropped": self.dropped,
            "written": self.written,
            "queued": self.pending.qsize(),
            "finished": self.finished,
            "error": self.error,
        }

    def _run(self):
        raw_names = [*field_units(), "fault_name", "timeout", "kill_switch_active"]
        setup_names = [*field_units(True), "fault_name"]
        derived_names = list(DERIVED_UNITS)
        metadata_columns = [
            "session_id",
            "connection_seq",
            "sample_seq",
            "received_unix_ns",
            "received_unix_sec",
            "received_unix_nanosec",
            "received_monotonic_s",
            "request_monotonic_s",
            "request_rtt_s",
            "target_erpm",
            "control_enabled",
            "control_target_speed",
            "control_direction",
            "control_fault",
            "torque_method",
            "baseline_resets",
            "warnings_json",
            "unavailable_json",
        ]
        columns = (
            metadata_columns
            + ["raw_" + k for k in raw_names]
            + ["derived_" + k for k in derived_names]
        )
        try:
            with (
                open(self.path + ".jsonl", "x", encoding="utf-8") as js,
                open(self.path + ".csv", "x", newline="", encoding="utf-8-sig") as cs,
                open(self.path + ".setup.csv", "x", newline="", encoding="utf-8-sig") as ss,
            ):
                writer = csv.DictWriter(cs, fieldnames=columns)
                writer.writeheader()
                setup_writer = csv.DictWriter(
                    ss, fieldnames=metadata_columns + ["setup_" + k for k in setup_names]
                )
                setup_writer.writeheader()
                while not self.closed.is_set() or not self.pending.empty():
                    try:
                        record = self.pending.get(timeout=0.1)
                    except queue.Empty:
                        continue
                    js.write(
                        json.dumps(
                            record, ensure_ascii=False, allow_nan=False, separators=(",", ":")
                        )
                        + "\n"
                    )
                    if record.get("kind") in ("raw", "setup"):
                        row = {k: record.get(k) for k in metadata_columns}
                        stamp = record.get("received_unix_stamp") or {}
                        row["received_unix_sec"] = stamp.get("sec")
                        row["received_unix_nanosec"] = stamp.get("nanosec")
                        control = record.get("control") or {}
                        row.update(
                            {
                                "control_" + k: control.get(k)
                                for k in ("enabled", "target_speed", "direction", "fault")
                            }
                        )
                    if record.get("kind") == "raw":
                        derived = record["derived"]
                        row.update({"raw_" + k: record["values"].get(k) for k in raw_names})
                        row.update(
                            {"derived_" + k: derived["values"].get(k) for k in derived_names}
                        )
                        row.update(
                            torque_method=derived.get("torque_method"),
                            baseline_resets=derived.get("baseline_resets"),
                            warnings_json=json.dumps(derived.get("warnings", [])),
                            unavailable_json=json.dumps(derived.get("unavailable", {})),
                        )
                        writer.writerow(row)
                    elif record.get("kind") == "setup":
                        row.update({"setup_" + k: record["values"].get(k) for k in setup_names})
                        setup_writer.writerow(row)
                    self.written += 1
                    js.flush()
                    cs.flush()
                    ss.flush()
                # Missing footer means interrupted/incomplete recording, not a clean run.
                js.write(
                    json.dumps(
                        {
                            "kind": "log_end",
                            "closed_unix_ns": time.time_ns(),
                            "written_records": self.written,
                            "dropped_records": self.dropped,
                            "error": self.error,
                        },
                        allow_nan=False,
                    )
                    + "\n"
                )
        except Exception as exc:
            self.error = f"{type(exc).__name__}: {exc}"
        else:
            self.finished = True

    def close(self):
        self.closed.set()
        if self.thread:
            self.thread.join(timeout=2)
            if self.thread.is_alive():
                self.error = self.error or "logger shutdown timeout; some records may be unflushed"


# ============================================================
# 공개 드라이버 — 통신 수신부가 재사용할 장치 객체
# ============================================================
# ============================================================
# [6] VescDriver — worker 와 공개 API
# ============================================================
class VescDriver:
    """명령 수신부에는 입력 API, 결과 송신부에는 snapshot()을 제공한다.

    UART 송수신과 telemetry 계산은 내부 worker가 수행한다.
    인스턴스 하나가 VESC 한 대의 포트를 소유한다.
    """

    def __init__(self, config: Config, transport_factory=None, *, ros_stamp_provider=None):
        config.validate()
        if ros_stamp_provider is not None and not callable(ros_stamp_provider):
            raise ValueError("ros_stamp_provider must be callable or None")
        self.config = config
        self.session_id = uuid.uuid4().hex
        # Optional non-blocking ROS clock reader; core UART never imports ROS.
        self.ros_stamp_provider = ros_stamp_provider
        self.controller = DriveController(config)
        self.estimator = Estimator(config)
        self.parser = PacketParser()
        self.factory = transport_factory or (lambda: open_serial(config))
        self.transport = None
        self.logger = TelemetryLogger(config.log_directory, config, session_id=self.session_id)
        self._lock = threading.Lock()
        self._inputs = deque()
        self._snapshot = {}
        self._stop = threading.Event()
        self._thread = None
        self._raw = self._setup = None
        self._sample_seq = 0
        self._pending = {}
        self._next_request = {}
        self._last_loop = None
        self._next_connect = 0.0
        self._next_control = 0.0
        self._next_snapshot = 0.0
        self._io_error = ""
        self._stats = {
            "tx_bytes": 0,
            "rx_bytes": 0,
            "decode_errors": 0,
            "request_timeouts": 0,
            "unsolicited_responses": 0,
            "connections": 0,
            "control_ticks": 0,
            "max_control_interval_s": 0.0,
            "last_control_interval_s": None,
            "last_request_rtt_s": None,
            "command_errors": 0,
            "timestamp_errors": 0,
        }

    def start(self):
        with self._lock:
            if self._stop.is_set():
                raise RuntimeError("driver is closed")
            if self._thread is not None:
                raise RuntimeError("driver may only be started once")
            self._thread = threading.Thread(target=self._run, name="vesc-uart", daemon=True)
            self._thread.start()
        return self

    def _enqueue(self, kind, value=None):
        with self._lock:
            if self._stop.is_set():
                raise RuntimeError("driver is closed")
            if kind == "command" and value in ("off", "stop"):
                # STOP may discard pending motion, but must not weaken a pending OFF.
                # A new ON received after this stop/off can explicitly enable again.
                if value == "stop" and ("command", "off") in self._inputs:
                    value = "off"
                self._inputs.clear()
            if kind != "command":
                # Live inputs are latest-value slots, never a driving history.
                # Direction and direct speed are mutually exclusive input modes.
                replaced = ("direction", "speed") if kind in ("direction", "speed") else (kind,)
                self._inputs = deque(item for item in self._inputs if item[0] not in replaced)
            if len(self._inputs) < 64:
                self._inputs.append((kind, value))
            else:
                raise RuntimeError("input queue full")

    def set_direction(self, direction):
        if type(direction) is not int or direction not in (-1, 0, 1):
            raise ValueError("direction must be -1, 0 or +1")
        self._enqueue("direction", direction)

    def set_speed(self, speed):
        if (
            type(speed) not in (int, float)
            or not math.isfinite(speed)
            or abs(speed) > self.config.max_speed
        ):
            raise ValueError("speed must be finite and within max_speed")
        self._enqueue("speed", speed)

    def command(self, name):
        if not isinstance(name, str):
            raise ValueError("command must be on, off or stop")
        name = name.strip().lower()
        if name not in ("on", "off", "stop"):
            raise ValueError("command must be on, off or stop")
        self._enqueue("command", name)

    def set_acceleration(self, value):
        if (
            type(value) not in (int, float)
            or not math.isfinite(value)
            or not 0 < value <= self.config.acceleration
        ):
            raise ValueError("acceleration must be positive and <= configured acceleration limit")
        self._enqueue("acceleration", value)

    def heartbeat(self):
        self._enqueue("heartbeat")

    def snapshot(self) -> dict:
        with self._lock:
            result = copy.deepcopy(self._snapshot)
        if not result:
            return {
                "schema_version": 1,
                "session_id": self.session_id,
                "mock": self.config.mock,
                "connected": False,
                "starting": True,
                "sensor": sensor_status(self.config),
            }
        now = time.monotonic()
        result["log"] = self.logger.status()
        result["worker_age_s"] = max(0, now - result["worker_monotonic_s"])
        for name in ("raw", "setup"):
            sample = result.get(name)
            if sample:
                sample["age_s"] = max(0, now - sample["request_monotonic_s"])
                sample["receive_age_s"] = max(0, now - sample["received_monotonic_s"])
                limit = (
                    self.config.telemetry_timeout_s
                    if name == "raw"
                    else max(3 / self.config.setup_hz, 1)
                )
                sample["fresh"] = result["connected"] and sample["age_s"] <= limit
        return result

    def _write(self, payload: bytes):
        packet = frame(payload)
        count = self.transport.write(packet)
        if count != len(packet):
            raise OSError(f"partial UART write: {count}/{len(packet)}; closing port")
        self._stats["tx_bytes"] += count
        if self.logger.thread is not None:
            self.logger.put(
                {
                    "kind": "tx",
                    "session_id": self.session_id,
                    "connection_seq": self._stats["connections"],
                    "unix_ns": time.time_ns(),
                    "monotonic_s": time.monotonic(),
                    "timestamp_source": "host_after_uart_write_not_motor_execution",
                    "command_id": payload[0],
                    "payload_hex": payload.hex(),
                    "frame_hex": packet.hex(),
                }
            )

    def _connect(self, now):
        self.transport = self.factory()
        self.parser.reset()
        self._pending.clear()
        self._next_request.clear()
        self.controller.disconnect()
        self.estimator.reset_baseline()
        self._raw = self._setup = None
        self._stats["connections"] += 1
        self._io_error = ""
        self._next_control = now

    def _disconnect(self, error):
        self._io_error = f"{type(error).__name__}: {error}"
        # Best effort coast before closing a still-writable port. No ALIVE packets.
        if self.transport is not None:
            if self.config.enable_motion:
                try:
                    self._write(set_current(0))
                except Exception as stop_error:
                    self._io_error += f"; coast send failed: {stop_error}"
            try:
                self.transport.close()
            except Exception as close_error:
                self._io_error += f"; close failed: {close_error}"
        self.transport = None
        self.controller.disconnect()
        self._pending.clear()
        self.parser.reset()
        self.estimator.reset_baseline()
        self._next_connect = time.monotonic() + self.config.reconnect_s
        with self._lock:
            self._inputs.clear()
        self.logger.put(
            {
                "kind": "disconnect",
                "unix_ns": time.time_ns(),
                "monotonic_s": time.monotonic(),
                "session_id": self.session_id,
                "connection_seq": self._stats["connections"],
                "error": self._io_error,
            }
        )

    def _receive(self, now):
        # A strict byte budget keeps flooding/noisy UART from monopolizing control.
        waiting = self.transport.in_waiting
        if waiting > 8192:
            raise OSError("UART RX backlog exceeds 8192 bytes")
        chunks = bytearray()
        for _ in range(16):
            waiting = self.transport.in_waiting
            if not waiting or len(chunks) >= 4096:
                break
            part = self.transport.read(min(waiting, 4096 - len(chunks)))
            if not part:
                break
            chunks.extend(part)
        data = bytes(chunks)
        # Host observation of the completed read, not the loop-start time and
        # not a hardware acquisition timestamp (VESC supplies none here).
        received_monotonic = time.monotonic()
        received_unix = time.time_ns()
        received_ros, timestamp_error = None, None
        if data and self.ros_stamp_provider is not None:
            try:
                stamp = self.ros_stamp_provider()
                if (
                    not isinstance(stamp, dict)
                    or type(stamp.get("sec")) is not int
                    or not -(1 << 31) <= stamp["sec"] < (1 << 31)
                    or type(stamp.get("nanosec")) is not int
                    or not 0 <= stamp["nanosec"] < 1_000_000_000
                    or type(stamp.get("ros_time_active")) is not bool
                    or type(stamp.get("valid")) is not bool
                ):
                    raise ValueError("invalid ROS receive timestamp")
                received_ros = {k: stamp[k] for k in ("sec", "nanosec", "ros_time_active", "valid")}
            except Exception as exc:
                timestamp_error = f"{type(exc).__name__}: {exc}"
                self._stats["timestamp_errors"] += 1
        self._stats["rx_bytes"] += len(data)
        for payload in self.parser.feed(data, received_monotonic):
            request_time = self._pending.pop(payload[0], None)
            if request_time is None:
                self._stats["unsolicited_responses"] += 1
                continue
            try:
                kind, sample = decode(payload)
                if kind in ("raw", "setup") and not sample["complete"]:
                    raise ValueError("full selective mask required")
            except ValueError as exc:
                self._stats["decode_errors"] += 1
                self._io_error = str(exc)
                continue
            rtt = received_monotonic - request_time
            self._stats["last_request_rtt_s"] = rtt
            sample.update(
                kind=kind,
                session_id=self.session_id,
                connection_seq=self._stats["connections"],
                received_unix_ns=received_unix,
                received_unix_stamp=dict(
                    zip(("sec", "nanosec"), divmod(received_unix, 1_000_000_000), strict=True)
                ),
                received_monotonic_s=received_monotonic,
                received_ros_stamp=received_ros,
                timestamp_source="host_uart_receive",
                timestamp_error=timestamp_error,
                request_monotonic_s=request_time,
                request_rtt_s=rtt,
                payload_hex=payload.hex(),
                frame_hex=frame(payload).hex(),
            )
            if kind == "firmware":
                old = self.controller.firmware
                if old is not None and old != {k: sample[k] for k in old}:
                    self.controller.trip("VESC identity/firmware changed")
                self.controller.firmware = {
                    k: sample[k] for k in ("major", "minor", "hardware", "uuid")
                }
            elif kind == "raw":
                self._sample_seq += 1
                sample["sample_seq"] = self._sample_seq
                sample["target_erpm"] = self.controller.target_erpm
                sample["derived"] = self.estimator.update(
                    sample["values"], request_time, sample["target_erpm"]
                )
                self.controller.observe_raw(sample["values"], request_time)
                sample["control"] = self.controller.status(received_monotonic)
                self._raw = sample
            elif kind == "setup":
                if self.estimator.observe_uptime(sample["values"]["uptime_ms"]):
                    self.controller.trip("VESC reboot detected")
                self._setup = sample
            self.logger.put(sample)

    def _commands(self, now):
        with self._lock:
            inputs = list(self._inputs)
            self._inputs.clear()
        for kind, value in inputs:
            try:
                if kind == "direction":
                    self.controller.set_direction(value, now)
                elif kind == "speed":
                    self.controller.set_speed(value, now)
                elif kind == "acceleration":
                    self.controller.set_acceleration(value, now)
                elif kind == "heartbeat":
                    self.controller.heartbeat(now)
                else:
                    self.controller.command(value, now)
                    self._next_control = min(self._next_control, now)
            except ValueError as exc:
                self.controller.last_error = str(exc)
                self._stats["command_errors"] += 1
                self.logger.put(
                    {
                        "kind": "input_rejected",
                        "input_kind": kind,
                        "value": value,
                        "session_id": self.session_id,
                        "unix_ns": time.time_ns(),
                        "monotonic_s": now,
                        "error": str(exc),
                    }
                )
            else:
                self.logger.put(
                    {
                        "kind": "command" if kind == "command" else "input",
                        "input_kind": kind,
                        "value": value,
                        "command": value if kind == "command" else None,
                        "session_id": self.session_id,
                        "unix_ns": time.time_ns(),
                        "monotonic_s": now,
                        "control": self.controller.status(now),
                    }
                )

    def _queries(self, now):
        # Never wait here. A silent optional SETUP response cannot block motor commands.
        intervals = {
            FW_VERSION: 5.0 if self.controller.firmware else 0.5,
            GET_VALUES_SELECTIVE: 1 / self.config.telemetry_hz,
            GET_SETUP_SELECTIVE: 1 / self.config.setup_hz,
        }
        timeout = min(0.2, self.config.telemetry_timeout_s / 2)
        for cmd, interval in intervals.items():
            sent = self._pending.get(cmd)
            if sent is not None and now - sent > timeout:
                self._pending.pop(cmd)
                self._stats["request_timeouts"] += 1
            if cmd not in self._pending and now >= self._next_request.get(cmd, 0):
                # Timestamp immediately before handing this request to the UART.
                self._pending[cmd] = time.monotonic()
                self._next_request[cmd] = next_deadline(
                    self._next_request.get(cmd, now), now, interval
                )
                self._write(
                    bytes((FW_VERSION,))
                    if cmd == FW_VERSION
                    else request_values(cmd == GET_SETUP_SELECTIVE)
                )

    def _publish_snapshot(self, now):
        snapshot_unix = time.time_ns()
        snap = {
            "schema_version": 1,
            "session_id": self.session_id,
            "connection_seq": self._stats["connections"],
            "mock": self.config.mock,
            "connected": self.transport is not None,
            "worker_monotonic_s": now,
            "snapshot_unix_ns": snapshot_unix,
            "published_unix_ns": snapshot_unix,  # legacy alias; NOT ROS publication time
            "firmware": self.controller.firmware,
            "sensor": sensor_status(self.config),
            "control": self.controller.status(now),
            "raw": self._raw,
            "setup": self._setup,
            "io_error": self._io_error,
            "stats": {**self._stats, **asdict(self.parser.stats)},
            "log": self.logger.status(),
        }
        with self._lock:
            self._snapshot = copy.deepcopy(snap)

    def _run(self):
        try:
            while not self._stop.is_set():
                now = time.monotonic()
                try:
                    # Do this before processing new telemetry or commands to expose outages.
                    was_enabled = self.controller.enabled
                    self.controller.check_timeouts(now)
                    if self.transport is None and now >= self._next_connect:
                        self._connect(now)
                    if self.transport is not None:
                        self._receive(now)
                    self._commands(now)
                    if was_enabled and not self.controller.enabled:
                        self._next_control = now
                    if now >= self._next_control:
                        erpm = self.controller.step(now)
                        if self._last_loop is not None:
                            interval = now - self._last_loop
                            self._stats["last_control_interval_s"] = interval
                            self._stats["max_control_interval_s"] = max(
                                self._stats["max_control_interval_s"], interval
                            )
                        self._last_loop = now
                        self._stats["control_ticks"] += 1
                        if self.transport is not None and self.config.enable_motion:
                            self._write(set_current(0) if erpm is None else set_rpm(erpm))
                        self._next_control = next_deadline(
                            self._next_control, now, 1 / self.config.control_hz
                        )
                    if self.transport is not None:
                        self._queries(now)
                except Exception as exc:
                    # Include unexpected decoder/transport errors: do not leave a running motor.
                    if self.controller.enabled:
                        self.controller.trip(f"worker exception: {type(exc).__name__}")
                    self._disconnect(exc)
                cache_now = time.monotonic()
                if cache_now >= self._next_snapshot:
                    self._publish_snapshot(cache_now)
                    self._next_snapshot = next_deadline(
                        self._next_snapshot,
                        cache_now,
                        1
                        / max(
                            self.config.control_hz, self.config.telemetry_hz, self.config.publish_hz
                        ),
                    )
                self._stop.wait(0.002)
        finally:
            self.controller.trip("driver shutdown")
            self._disconnect(RuntimeError("driver shutdown"))
            self.logger.put(
                {
                    "kind": "run_end",
                    "session_id": self.session_id,
                    "unix_ns": time.time_ns(),
                    "monotonic_s": time.monotonic(),
                    "stats": {**self._stats, **asdict(self.parser.stats)},
                    "control": self.controller.status(time.monotonic()),
                }
            )
            self._publish_snapshot(time.monotonic())

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)
            if self._thread.is_alive():
                raise RuntimeError("UART worker did not stop; rely on VESC hardware timeout")
        self.logger.close()


# ============================================================
# RADXA 단독 점검 — 지상국 통신 없이 드라이버를 직접 실행
# ============================================================

# ============================================================
# [7] 역할 계층 — 방향 반전이 사는 유일한 곳
# ============================================================
#   부호는 여기와 각 config json 의 direction_sign 두 곳에만 있고,
#   RoleVescDriver 가 둘이 일치하는지 검사한다. 호출자는 역할 이름만 넘긴다.
# [261017] VESC 교체 후 뒷바퀴가 반대로 돌아 back 을 +1 로 바꿨다 (config/back.json 과 함께).
ROLES = {"front": +1, "back": +1}

CONFIG_DIR = Path(__file__).resolve().parents[2] / "config"

# 이 값들이 역할별로 다르면 같은 지령에 서로 다르게 반응한다. 직진 시험이 성립하지 않는다.
_SHARED_FIELDS = ("speed_unit", "max_speed", "acceleration", "deceleration")


def role_config_path(role: str) -> Path:
    if role not in ROLES:
        raise ValueError(f"unknown role {role!r}; expected one of {tuple(ROLES)}")
    return CONFIG_DIR / f"{role}.json"


def load_role_config(role: str, path=None, *, require_motion_ready: bool = True) -> Config:
    """역할별 설정을 읽고 direction_sign 이 역할과 맞는지 확인한다."""
    cfg = load_config(path or role_config_path(role),
                      require_motion_ready=require_motion_ready)
    if cfg.direction_sign != ROLES[role]:
        raise ValueError(
            f"{role}: direction_sign must be {ROLES[role]:+d} "
            f"(got {cfg.direction_sign:+d}); do not invert the input twice")
    return cfg


class RoleVescDriver(VescDriver):
    """역할에 묶인 드라이버. 장착 부호는 코어가 정확히 한 번 적용한다."""

    def __init__(self, role: str, config: Config, transport_factory=None, *,
                 ros_stamp_provider=None):
        if role not in ROLES:
            raise ValueError(f"unknown role {role!r}; expected one of {tuple(ROLES)}")
        self.motor_role = role
        self.mounting_sign = ROLES[role]
        if config.direction_sign != self.mounting_sign:
            raise ValueError(
                f"{role}: direction_sign must be {self.mounting_sign:+d}; "
                "pass the same rover-frame command to every role")
        super().__init__(config, transport_factory, ros_stamp_provider=ros_stamp_provider)

    def snapshot(self):
        state = super().snapshot()
        state["motor_role"] = self.motor_role
        state["uart_port"] = self.config.port
        state["direction_sign"] = self.mounting_sign
        return state


def make_driver(role: str, config: Config = None, path=None, **kwargs) -> RoleVescDriver:
    """역할 이름으로 드라이버를 만든다. config 를 주지 않으면 json 에서 읽는다."""
    if config is None:
        config = load_role_config(role, path)
    return RoleVescDriver(role, config, **kwargs)


def pair_errors(configs: dict, *, motion: bool = False) -> list[str]:
    """
    여러 역할의 설정이 '한 대의 로버로서' 성립하는지 검사한다.

    한쪽만 물려서 한 축으로 끌리는 상황을 시작 전에 막는 것이 목적이다.
    ROS 노드와 단독 테스트가 같은 규칙을 쓰도록 여기 한 곳에 둔다.
    """
    errors = []
    for role, cfg in configs.items():
        if role not in ROLES:
            errors.append(f"{role}: unknown role; expected one of {tuple(ROLES)}")
            continue
        try:
            cfg.validate(require_motion_ready=False)
        except ValueError as exc:
            errors.append(f"{role}: {exc}")
        if cfg.direction_sign != ROLES[role]:
            errors.append(f"{role}: direction_sign must be {ROLES[role]:+d}")
        if not cfg.mock and not cfg.port.strip():
            errors.append(f"{role}: port is required; verify the RADXA UART device")
        if motion:
            errors.extend(f"{role}: {item}" for item in cfg.motion_errors())

    if len({cfg.mock for cfg in configs.values()}) > 1:
        errors.append("all roles must be real or all mock")
    # [261017] mock transport(experiments/mock_vesc.py)는 이 저장소에 없다. 워커 스레드가 1 s 마다 재연결하다
    #   5 s 뒤 'no fresh VESC response' 로 끝나던 것을 시작 전에 명확히 막는다.
    if any(cfg.mock for cfg in configs.values()):
        errors.append("mock VESC transport (experiments/mock_vesc.py) is not in this repo; run on real UART")

    for (ra, ca), (rb, cb) in itertools.combinations(configs.items(), 2):
        if ca.port and cb.port:
            # /dev/serial/by-path 같은 별칭을 실제 장치로 풀어서 비교한다.
            a, b = Path(ca.port).resolve(), Path(cb.port).resolve()
            same = a == b
            if a.exists() and b.exists():
                same = same or a.samefile(b)
            if same:
                errors.append(f"{ra}/{rb} require DIFFERENT UART devices")
        if ca.log_directory and cb.log_directory:
            if Path(ca.log_directory).resolve() == Path(cb.log_directory).resolve():
                errors.append(f"{ra}/{rb} require separate log directories")
        if ca.expected_uuid and ca.expected_uuid == cb.expected_uuid:
            errors.append(f"{ra}/{rb} cannot use the same expected_uuid")
        for field in _SHARED_FIELDS:
            if getattr(ca, field) != getattr(cb, field):
                errors.append(f"{ra}/{rb} {field} must match for a straight-drive test")
    return errors


# ============================================================
# [8] 단독 테스트 CLI — ROS 없이 1채널 또는 전 채널
# ============================================================
def _ready(state) -> bool:
    return bool(state.get("connected") and state.get("firmware")
                and (state.get("raw") or {}).get("fresh"))


def _release(motors, emit) -> bool:
    """전 채널에 OFF 를 먼저 큐에 넣고, 그다음 순서대로 닫는다.

    한쪽을 닫는 동안 다른 쪽이 마지막 지령을 물고 계속 도는 것을 막는다.
    """
    failed = False
    for role, motor in motors.items():
        try:
            motor.command("off")
        except Exception as exc:
            emit(f"CLEANUP ERROR: {role} off: {exc}")
            failed = True
    for role, motor in motors.items():
        try:
            motor.close()
        except Exception as exc:
            emit(f"CLEANUP ERROR: {role} close: {exc}")
            failed = True
    return failed


def run_trial(configs: dict, *, seconds=5.0, direction=0, speed=None, on=False,
              emit=print, factories=None) -> int:
    """유한 시간 시험. factories 는 오프라인 바이트 수준 시험용 주입점이다."""
    if type(seconds) not in (int, float) or not math.isfinite(seconds) or not 0 < seconds <= 3600:
        raise ValueError("seconds must be finite, positive and <= 3600")
    if type(direction) is not int or direction not in (-1, 0, 1):
        raise ValueError("direction must be -1, 0 or +1")
    if (direction or speed is not None) and not on:
        raise ValueError("motion input requires on=True")
    if speed is not None:
        if direction or type(speed) not in (int, float) or not math.isfinite(speed):
            raise ValueError("finite speed and no direction input required")
        if any(abs(speed) > c.max_speed for c in configs.values()):
            raise ValueError("speed exceeds max_speed of some role")
    problems = pair_errors(configs, motion=on)
    if problems:
        raise ValueError("; ".join(problems))
    if not on:
        configs = {r: replace(c, enable_motion=False) for r, c in configs.items()}

    factories = factories or {}
    motors, result = {}, 0
    try:
        for role, cfg in configs.items():
            motors[role] = (factories.get(role) or (lambda c, r=role: make_driver(r, c)))(cfg)
            motors[role].start()

        deadline = time.monotonic() + 5.0
        while True:
            states = {r: m.snapshot() for r, m in motors.items()}
            if all(_ready(s) for s in states.values()):
                break
            if time.monotonic() >= deadline:
                missing = [r for r, s in states.items() if not _ready(s)]
                raise RuntimeError("no fresh VESC response: " + ", ".join(missing))
            time.sleep(0.02)

        if on:
            # 전 채널이 ON 을 확인하기 전에는 어느 쪽에도 0 이 아닌 지령을 주지 않는다.
            for motor in motors.values():
                motor.command("on")
            deadline = time.monotonic() + 5.0
            while True:
                states = {r: m.snapshot() for r, m in motors.items()}
                for role, state in states.items():
                    ctl = state.get("control", {})
                    if ctl.get("last_error") or ctl.get("fault") or not _ready(state):
                        raise RuntimeError(f"{role} cannot enable: {ctl}")
                if all(s.get("control", {}).get("enabled") for s in states.values()):
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError("not all VESCs acknowledged ON")
                for motor in motors.values():
                    motor.heartbeat()
                time.sleep(0.02)

        end, next_print = time.monotonic() + seconds, 0.0
        period = 1.0 / min(c.publish_hz for c in configs.values())
        while time.monotonic() < end:
            states = {r: m.snapshot() for r, m in motors.items()}
            for role, state in states.items():
                ctl = state.get("control", {})
                if not _ready(state):
                    raise RuntimeError(f"{role}: telemetry unavailable; "
                                       f"{state.get('io_error', '')}")
                if on and (not ctl.get("enabled") or ctl.get("fault") or ctl.get("last_error")):
                    raise RuntimeError(f"{role} stopped: {ctl}")
            if on:
                for motor in motors.values():
                    if speed is None:
                        motor.set_direction(direction)
                    else:
                        motor.set_speed(speed)
            now = time.monotonic()
            if now >= next_print:
                # 독립 샘플이다. 각 수신/요청 타임스탬프를 그대로 남긴다.
                emit(json.dumps({"kind": "trial_snapshot",
                                 "snapshot_unix_ns": time.time_ns(), **states},
                                ensure_ascii=False, allow_nan=False))
                next_print = now + period
            time.sleep(0.02)
    except KeyboardInterrupt:
        result = 130
    except Exception as exc:
        emit(f"ERROR: {type(exc).__name__}: {exc}")
        result = 1
    finally:
        if _release(motors, emit):
            result = 1
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="VESC drive test (no ROS). One role or every role at once.")
    group = parser.add_mutually_exclusive_group()
    group.add_argument("--role", choices=tuple(ROLES), help="single role")
    group.add_argument("--dual", action="store_true", help="every role in ROLES")
    parser.add_argument("--config", help="config path override; only with --role")
    parser.add_argument("--seconds", type=float, default=5.0)
    parser.add_argument("--direction", type=int, choices=(-1, 0, 1), default=0,
                        help="+1/-1: configured acceleration; 0: decelerate to zero")
    parser.add_argument("--speed", type=float, default=None,
                        help="fixed signed speed instead of acceleration (DIRECT, no ramp)")
    parser.add_argument("--on", action="store_true", help="allow motor output")
    parser.add_argument("--readonly", action="store_true", help="receive/log only")
    parser.add_argument("--check-config", action="store_true",
                        help="show setup blockers without opening UART")
    args = parser.parse_args(argv)

    if not args.role and not args.dual:
        args.dual = True
    if args.config and not args.role:
        parser.error("--config requires --role")
    if args.readonly and args.on:
        parser.error("--readonly and --on conflict")
    if (args.direction or args.speed is not None) and not args.on:
        parser.error("motion input requires --on")

    roles = (args.role,) if args.role else tuple(ROLES)
    try:
        configs = {r: load_role_config(r, args.config if args.role else None,
                                       require_motion_ready=False)
                   for r in roles}
    except (ValueError, OSError) as exc:
        parser.error(str(exc))

    if args.check_config:
        print(json.dumps({
            "roles": {r: {"port": c.port, "direction_sign": c.direction_sign,
                          "sensor": sensor_status(c),
                          "log_directory": c.log_directory}
                      for r, c in configs.items()},
            "motion_blockers": pair_errors(configs, motion=True),
            "input_current_monitor_sum_a": sum(c.max_input_current_a or 0
                                               for c in configs.values()),
            "note": "Python monitor limits do NOT write the VESC hardware limits",
        }, ensure_ascii=False, indent=2, allow_nan=False))
        return 0

    try:
        return run_trial(configs, seconds=args.seconds, direction=args.direction,
                         speed=args.speed, on=args.on,
                         emit=lambda line: print(line, flush=True))
    except (ValueError, OSError) as exc:
        parser.error(str(exc))


if __name__ == "__main__":
    raise SystemExit(main())