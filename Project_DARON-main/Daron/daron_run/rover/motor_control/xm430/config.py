#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
susp_config.py — 서스펜션 실험용 파라미터 묶음.

dxl_driver.MotorConfig 를 대체하지 않는다. 감싼다.
드라이버가 아는 필드(전류/게인/프로파일)는 to_motor_config() 로 넘기고,
서스펜션 상태기계만 아는 필드(comply, 릴리스 검출, 방향 보정)는 여기 남는다.

방향 규약
---------
  DIR_POS = +1 : Present Position 이 증가하는 쪽. XM430 은 출력축에서 볼 때 CCW.
  DIR_NEG = -1 : 감소하는 쪽. CW.
  '복원 방향' 은 오차 (goal - position) 의 부호다. 축이 아래로 눌렸고
  위로 되돌아가야 한다면, 그 되돌아가는 방향이 복원 방향이다.

전류 단위는 전부 LSB (1 LSB = 2.69 mA ≈ 3.5 mN.m) 이고,
각도는 파일/CLI 에서 deg, 내부 계산에서 rad 이다.
"""

from __future__ import annotations

import json
import math
import os
from dataclasses import asdict, dataclass, field, fields

DIR_POS = 1
DIR_NEG = -1

CURRENT_LSB_MA = 2.69
KT_NM_PER_A = 1.304
MNM_PER_LSB = KT_NM_PER_A * CURRENT_LSB_MA        # ≈ 3.51 mN.m / LSB

COMP_MODES = ("off", "bias", "scale", "both")


def lsb_to_mnm(lsb: float) -> float:
    return lsb * MNM_PER_LSB


def unwrap_zero_deg(zero_deg: float, position_deg: float) -> float:
    """
    굴절 원점을 360 deg 단위로 옮겨 (position - zero) 가 [-180, 180) 에 들게 한다.

    XM430 은 전원을 켜면 Present Position 을 한 바퀴(0 ~ 360 deg) 안으로 다시
    잡는다. 프로파일의 home_deg 가 0 근처(예: 22.5)면 원점 아래쪽 자세(중립 150 =
    -7.5 deg)가 352.5 deg 로 읽혀, 원점과 그대로 빼면 330 deg 차이가 난다.
    원점을 읽힌 회전 쪽으로 옮겨 두면 굴절각과 모든 목표가 같은 회전에서 계산된다.
    (radxa.py 에서 옮겨 옴. 261016: 캘리브레이션 스크립트도 같이 쓴다)
    """
    turns = math.floor((position_deg - zero_deg + 180.0) / 360.0)
    return zero_deg + 360.0 * turns


def wrap_deg(deg: float) -> float:
    """[-180, 180) 로 접는다. 프로파일에 남길 원점은 회전 수와 무관해야 해서 이걸로 저장한다."""
    return (deg + 180.0) % 360.0 - 180.0


def clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


@dataclass
class SuspensionConfig:
    # ---------- 연결 ----------
    port: str = ""                       # 비우면 find_port()
    dxl_id: int = 1
    baudrate: int = 57600

    # ---------- 전류 상한 [LSB] ----------
    # current_limit 은 EEPROM 이다. 런타임에 못 바꾸므로 실험 범위 전체를
    # 덮도록 넉넉히 잡는다. 나머지 셋은 RAM 이라 매 주기 스위칭할 수 있고,
    # 그 스위칭이 이 도구의 존재 이유다.
    current_limit: int = 350             # ≈ 941 mA, 상한 토크 ≈ 1.23 N.m
    current_moving: int = 200            # 지령 이동 (실험 초기조건 생성용)
    current_recover: int = 150           # 복원 — '빠른 복원'의 1번 손잡이
    current_holding: int = 30            # 정지 유지 — '낮은 유지 토크'의 손잡이
    current_comply: int = 20             # 충격 인가 중. holding 이하로 둔다.

    # ---------- 위치 게인 ----------
    p_gain: int = 120                    # 유효 스프링 강성
    i_gain: int = 0                      # wind-up 위험. 정적 처짐이 남을 때만
    d_gain: int = 600                    # 유효 댐퍼

    # ---------- 궤적 ----------
    # 복원 트리거에서 Goal Position 을 다시 쓸 때 이 프로파일로 사다리꼴이
    # 재생성된다. '빠른 복원'의 2번 손잡이.
    profile_velocity_dps: float = 240.0
    profile_accel_dpss: float = 1200.0

    # ---------- 상태 판정 ----------
    home_deg: float = 0.0
    settled_band_deg: float = 0.7        # 이 안이면 복원 완료로 본다
    comply_band_deg: float = 2.0         # 이 밖이면 외란으로 본다
    release_vel_dps: float = 12.0        # 이보다 느리면 '외력이 멈췄다'
    release_dwell_s: float = 0.03        # 그 상태가 이만큼 지속돼야 복원 시작
    settle_dwell_s: float = 0.10         # 밴드 안에 이만큼 머물러야 HOLD
    recover_timeout_s: float = 2.0       # 복원이 이보다 길면 포기하고 YIELD
    yield_retry_s: float = 0.5           # YIELD 에서 복원을 재시도하기까지
    reissue_interval_s: float = 0.15     # 복원 중 Goal Position 재기입 최소 간격
    reimpact_ratio: float = 1.25         # 오차가 이 배로 커지면 새 충격으로 판정

    # ---------- 방향 보정 ----------
    # I_cap(dir) = current_recover * k_dir + b_dir
    # 중력 부하는 '덧셈' 오프셋이므로 bias 가 물리적으로 옳은 형태다.
    # scale 은 좁은 동작 범위에서의 근사. both 는 둘 다.
    dir_comp_mode: str = "off"           # off | bias | scale | both
    k_pos: float = 1.0                   # + 방향(CCW) 복원 배율
    k_neg: float = 1.0                   # - 방향(CW)  복원 배율
    b_pos: float = 0.0                   # + 방향 전류 오프셋 [LSB]
    b_neg: float = 0.0                   # - 방향 전류 오프셋 [LSB]
    # 방향별 프로파일. None 이면 공통값을 쓴다. 빠른 쪽의 전류를 굶기는 대신
    # 궤적 속도를 낮추고 싶을 때 쓴다 (감속 토크가 유지되어 오버슈트에 유리).
    profile_velocity_pos_dps: float = 0.0   # 0 = 공통값 사용
    profile_velocity_neg_dps: float = 0.0

    # ---------- 실시간 속도 피드백 (L3, 기본 OFF) ----------
    # Present Velocity 는 이미 같은 sync read 에 들어 있어 추가 통신이 없다.
    # 비용은 CPU 가 아니라 Goal Current 재기입(57600 baud 에서 왕복 ≈ 4 ms)이다.
    # 그래서 데드밴드 밖에서만 쓴다.
    vel_fb_enable: bool = False
    vel_fb_target_dps: float = 150.0     # 복원 중 유지하고 싶은 속도
    vel_fb_gain: float = 0.25            # [LSB / (deg/s) / s] — 적분형
    vel_fb_deadband_lsb: int = 5         # 이만큼 바뀔 때만 실제로 쓴다
    vel_fb_span_lsb: int = 80            # 기준 전류 대비 ± 허용 폭

    # ---------- 가동 범위 ----------
    # home 기준 ±. 프로그램이 이 밖을 '의도적으로 지령하지 않는다'는 뜻이다.
    # 외력에 밀려 이 밖으로 나가는 것은 막지 않는다 (막을 수도 없다).
    # 그때는 경계로 되돌리는 목표가 걸릴 뿐 실행이 중단되지는 않는다.
    # 0 = 제한 없음.
    travel_limit_deg: float = 35.0

    # ---------- 루프 / 로깅 ----------
    control_hz: float = 100.0
    log_dir: str = "logs"
    note: str = ""

    # ---------- 파생값 ----------
    @property
    def home_rad(self): return math.radians(self.home_deg)

    @property
    def settled_band_rad(self): return math.radians(self.settled_band_deg)

    @property
    def comply_band_rad(self): return math.radians(self.comply_band_deg)

    @property
    def release_vel_rad_s(self): return math.radians(self.release_vel_dps)

    @property
    def travel_limit_rad(self): return math.radians(self.travel_limit_deg)

    # ---------- 방향 보정 적용 ----------
    def recover_current(self, direction: int) -> int:
        """복원 방향에 맞춰 보정된 Goal Current [LSB]."""
        base = float(self.current_recover)
        k = self.k_pos if direction >= 0 else self.k_neg
        b = self.b_pos if direction >= 0 else self.b_neg
        m = self.dir_comp_mode
        if m == "bias":
            v = base + b
        elif m == "scale":
            v = base * k
        elif m == "both":
            v = base * k + b
        else:
            v = base
        return int(clamp(round(v), 1, self.current_limit))

    def recover_profile_velocity(self, direction: int) -> float:
        v = self.profile_velocity_pos_dps if direction >= 0 else self.profile_velocity_neg_dps
        return v if v > 0 else self.profile_velocity_dps

    def dir_profile_active(self) -> bool:
        return self.profile_velocity_pos_dps > 0 or self.profile_velocity_neg_dps > 0

    # ---------- 검증 ----------
    def validate(self):
        errs = []
        for name in ("current_moving", "current_recover",
                     "current_holding", "current_comply"):
            v = getattr(self, name)
            if not (0 < v <= self.current_limit):
                errs.append(f"{name}={v} 는 0 초과 current_limit({self.current_limit}) "
                            f"이하여야 합니다. Goal Current 는 Current Limit 으로 clamp 됩니다.")
        if self.dir_comp_mode not in COMP_MODES:
            errs.append(f"dir_comp_mode={self.dir_comp_mode!r} — {COMP_MODES} 중 하나여야 합니다.")
        if self.comply_band_deg <= self.settled_band_deg:
            errs.append("comply_band_deg 는 settled_band_deg 보다 커야 합니다 "
                        "(그렇지 않으면 상태가 채터링합니다).")
        if errs:
            raise ValueError("설정 오류:\n  - " + "\n  - ".join(errs))
        if self.current_comply > self.current_holding:
            print("[cfg] 주의: comply > holding 입니다. 충격 시 더 단단해집니다. 의도한 것인가요?")
        if self.current_recover <= self.current_holding:
            print("[cfg] 주의: recover <= holding 입니다. 방향 보정 이전에 이미 "
                  "'낮은 유지 + 빠른 복원' 구조가 아닙니다.")
        return self

    # ---------- 드라이버로 넘기기 ----------
    def to_motor_config(self, dxl):
        """dxl_driver.MotorConfig 생성. 드라이버가 모르는 필드는 넘기지 않는다."""
        self.validate()
        return dxl.MotorConfig(
            dxl_id=self.dxl_id,
            baudrate=self.baudrate,
            current_limit=self.current_limit,
            current_moving=self.current_moving,
            current_recover=self.current_recover,
            current_holding=self.current_holding,
            profile_velocity_dps=self.profile_velocity_dps,
            profile_accel_dpss=self.profile_accel_dpss,
            p_gain=self.p_gain, i_gain=self.i_gain, d_gain=self.d_gain,
            home_rad=self.home_rad,
            settled_band_rad=self.settled_band_rad,
            disturb_band_rad=self.comply_band_rad,
            reissue_interval_s=self.reissue_interval_s,
        )

    # ---------- 직렬화 ----------
    def to_dict(self):
        return asdict(self)

    def save(self, path: str):
        os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(self.to_dict(), f, ensure_ascii=False, indent=2)
        return path

    @classmethod
    def load(cls, path: str) -> "SuspensionConfig":
        with open(path, encoding="utf-8") as f:
            d = json.load(f)
        return cls.from_dict(d)

    @classmethod
    def from_dict(cls, d: dict) -> "SuspensionConfig":
        known = {f.name for f in fields(cls)}
        unknown = set(d) - known
        if unknown:
            print(f"[cfg] 모르는 키는 무시합니다: {sorted(unknown)}")
        return cls(**{k: v for k, v in d.items() if k in known})

    def apply_overrides(self, pairs) -> "SuspensionConfig":
        """CLI 의 --set name=value 들을 타입에 맞춰 반영한다."""
        types = {f.name: f.type for f in fields(self)}
        for item in pairs or []:
            if "=" not in item:
                raise ValueError(f"--set 은 name=value 형식입니다: {item!r}")
            name, _, raw = item.partition("=")
            name = name.strip()
            if name not in types:
                raise ValueError(f"모르는 파라미터: {name}")
            cur = getattr(self, name)
            if isinstance(cur, bool):
                val = raw.strip().lower() in ("1", "true", "yes", "on")
            elif isinstance(cur, int):
                val = int(round(float(raw)))
            elif isinstance(cur, float):
                val = float(raw)
            else:
                val = raw
            setattr(self, name, val)
        return self

    def summary(self) -> str:
        c = self
        lim = ("제한 없음" if c.travel_limit_deg <= 0
               else f"home ±{c.travel_limit_deg:.1f}°")
        lines = [
            f"  전류 [LSB]  limit={c.current_limit}  moving={c.current_moving}  "
            f"recover={c.current_recover}  holding={c.current_holding}  comply={c.current_comply}",
            f"              (holding ≈ {lsb_to_mnm(c.current_holding):.0f} mN.m, "
            f"recover ≈ {lsb_to_mnm(c.current_recover):.0f} mN.m)",
            f"  게인        P={c.p_gain}  I={c.i_gain}  D={c.d_gain}",
            f"  프로파일    v={c.profile_velocity_dps:.0f} deg/s  a={c.profile_accel_dpss:.0f} deg/s^2",
            f"  밴드        settled={c.settled_band_deg:.2f}°  comply={c.comply_band_deg:.2f}°  "
            f"release<{c.release_vel_dps:.0f} deg/s for {c.release_dwell_s * 1000:.0f} ms",
            f"  가동 범위   {lim}  (지령을 이 안으로 자른다. 셧다운 아님)",
            f"  방향보정    mode={c.dir_comp_mode}  k=({c.k_pos:.3f}, {c.k_neg:.3f})  "
            f"b=({c.b_pos:+.1f}, {c.b_neg:+.1f}) LSB"
            f"  ->  I+={c.recover_current(DIR_POS)}  I-={c.recover_current(DIR_NEG)}",
        ]
        if c.dir_profile_active():
            lines.append(f"  방향프로파일 v+={c.recover_profile_velocity(DIR_POS):.0f}  "
                         f"v-={c.recover_profile_velocity(DIR_NEG):.0f} deg/s")
        if c.vel_fb_enable:
            lines.append(f"  속도피드백  target={c.vel_fb_target_dps:.0f} deg/s  "
                         f"gain={c.vel_fb_gain}  deadband={c.vel_fb_deadband_lsb} LSB")
        return "\n".join(lines)


def add_config_args(ap):
    """모든 CLI 가 공유하는 인자."""
    ap.add_argument("--profile", default="", help="프리셋 JSON 경로 (profiles/*.json)")
    ap.add_argument("--set", dest="overrides", action="append", metavar="name=value",
                    help="파라미터 개별 덮어쓰기. 여러 번 쓸 수 있음")
    ap.add_argument("--port", default="", help="시리얼 포트 (기본: 자동 탐색)")
    ap.add_argument("--mock", action="store_true",
                    help="하드웨어 없이 시뮬레이션으로 실행 (코드 점검 전용)")
    ap.add_argument("--log-dir", default="", help="로그 디렉토리")
    return ap


def config_from_args(args) -> SuspensionConfig:
    here = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
    path = args.profile
    if path and not os.path.isabs(path):
        cand = os.path.join(here, path)
        if os.path.exists(cand):
            path = cand
        elif os.path.exists(os.path.join(here, "profiles", path)):
            path = os.path.join(here, "profiles", path)
        elif os.path.exists(os.path.join(here, "profiles", path + ".json")):
            path = os.path.join(here, "profiles", path + ".json")
    cfg = SuspensionConfig.load(path) if path else SuspensionConfig()
    cfg.apply_overrides(getattr(args, "overrides", None))
    if getattr(args, "port", ""):
        cfg.port = args.port
    if getattr(args, "log_dir", ""):
        cfg.log_dir = args.log_dir
    if not os.path.isabs(cfg.log_dir):
        cfg.log_dir = os.path.join(here, cfg.log_dir)
    return cfg.validate()


# ============================================================
# CLI — 단계별로 찾은 값을 프로파일에 쌓는다
# ============================================================
def _main(argv):
    """
    각 실험이 출력하는 `--set name=value` 줄을 프로파일에 반영해 둔다.
    이렇게 해야 다음 단계가 앞 단계의 결과 위에서 돌아간다.

        python3 -m motor_control.xm430.config --profile medium --set home_deg=22.5 \
                               --save profiles/rover.json
        python3 -m motor_control.xm430.config --profile rover          # 현재 값 확인
    """
    import argparse
    ap = argparse.ArgumentParser(description="서스펜션 프로파일 확인/편집")
    add_config_args(ap)
    ap.add_argument("--save", default="", metavar="PATH",
                    help="반영한 결과를 이 경로에 쓴다 (없으면 출력만)")
    args = ap.parse_args(argv[1:])
    args.mock = True                      # 하드웨어를 열지 않는다
    cfg = config_from_args(args)
    print(cfg.summary())
    if args.save:
        path = args.save
        if not os.path.isabs(path):
            path = os.path.join(os.path.abspath(os.path.join(os.path.dirname(__file__), "../..")), path)
        print(f"\n저장: {cfg.save(path)}")
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(_main(sys.argv))
