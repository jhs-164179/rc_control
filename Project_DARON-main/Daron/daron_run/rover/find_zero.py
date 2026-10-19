#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
find_zero.py — 굴절 0 (= 굴절각 180도) 에 해당하는 XM430 엔코더 각도를 찾는다.

무엇을 구하는가
---------------
radxa.py 의 정의는

    theta_eff = theta_meas - articulation_zero_deg      (radxa.py:492)
    alpha     = 180 + theta_eff     (261010 부호 정정. 이전 판은 180 - theta_eff)

따라서  굴절 180도  <=>  theta_eff = 0  <=>  theta_meas = articulation_zero_deg.
구해야 할 값은 이 하나뿐이다. 조그 한계도 art_zero +- jog_limit 으로 걸리므로
(radxa.py:709) 이 값만 맞으면 가동 범위도 자동으로 대칭이 된다.

왜 틀리면 안 되는가
-------------------
l12_driver 의 기구학은  alpha_eff = 180 - |theta_eff|  로 절댓값을 쓴다.
원점이 45도 어긋나면 좌우 가동 범위가 +22.5 / -67.5 로 쪼개지고, 조향 부호
규약(theta 의 부호)이 뒤집히는 구간이 생긴다. 조용히 틀리는 종류의 오차다.

네 가지 방법 — 위에서부터 권장
------------------------------
  limits   양쪽 기계 스토퍼를 저전류로 살짝 눌러 보고 그 중점을 원점으로 삼는다.
           육안 판단이 전혀 들어가지 않고 분해능이 0.1도 수준이라 가장 정확하다.
           단, 굴절부에 실제 물리 스토퍼가 있어야 한다.
  check    후보 각도로 차례로 이동해 세우고, 눈으로 180도인지 확인한다.
           후보가 45도 떨어져 있어 육안으로도 절대 헷갈리지 않는다.
  hand     토크를 끄고 손으로 180도에 맞춘 뒤 엔코더를 읽는다.
           하중 때문에 손으로 안 움직이면 못 쓴다.
  goto     임의 각도로 보내고 잡아 둔다. 눈으로 확인하며 더듬을 때.

예)
  python3 find_zero.py limits
  python3 find_zero.py check --candidates 22.5,-22.5
  python3 find_zero.py hand
  python3 find_zero.py goto 22.5
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from motor_control.xm430.metrics import _median
from motor_control.xm430.link import open_motor
from motor_control.xm430.config import add_config_args, config_from_args, lsb_to_mnm, unwrap_zero_deg, wrap_deg
from motor_control.xm430.suspension import DEG, RateLoop, SuspensionController

MAX_MOTOR_ANGLE_DEG = 45.0        # l12_driver.MAX_MOTOR_ANGLE_DEG 와 같아야 한다


def open_rig(args, verbose=True):
    cfg = config_from_args(args)
    dxl, motor, info = open_motor(cfg, mock=args.mock, port=cfg.port,
                                  verbose=False)
    ctrl = SuspensionController(dxl, motor, cfg)
    fb = ctrl.attach(info)
    if verbose:
        print(f"  포트 {motor.port_name}   현재 엔코더 각도 "
              f"{fb.position_rad * DEG:+.2f}°")
    return cfg, dxl, motor, ctrl


def to_this_turn(motor, angle_deg: float) -> float:
    """[261016] 원점·후보 각도를 지금 엔코더가 읽히는 회전으로 옮긴다 [deg].
    모드 5 는 여러 바퀴를 세므로 352.5 로 읽히는 자리에 22.5 를 그대로 지령하면 330 deg 를 돈다."""
    return unwrap_zero_deg(angle_deg, motor.read().position_rad * DEG)


def report(zero_deg: float, travel_deg: float = None, source: str = ""):
    """찾은 원점을 해석해서 보여준다."""
    print("\n══════════════ 결과 ══════════════")
    print(f"  articulation_zero_deg = {zero_deg:+.2f}°"
          f"{'   (' + source + ')' if source else ''}")
    print(f"\n  뜻: 엔코더가 {zero_deg:+.2f}° 일 때 굴절각이 180° (평면 전개)")
    print(f"      엔코더 0° 에서는 굴절각이 {180.0 + (0.0 - zero_deg):.1f}°")
    lo, hi = zero_deg - MAX_MOTOR_ANGLE_DEG, zero_deg + MAX_MOTOR_ANGLE_DEG
    print(f"      가동 범위 = {lo:+.1f}° ~ {hi:+.1f}°  (굴절각 135° ~ 225°)")
    if travel_deg is not None:
        d = abs(travel_deg - MAX_MOTOR_ANGLE_DEG)
        mark = "OK" if d < 3.0 else "⚠ 불일치"
        print(f"\n  측정된 편측 행정 = {travel_deg:.2f}°  "
              f"(기대값 {MAX_MOTOR_ANGLE_DEG:.0f}°)  [{mark}]")
        if d >= 3.0:
            print("     스토퍼가 아니라 다른 곳에 걸렸거나, 기계 한계가")
            print("     l12_driver.MAX_MOTOR_ANGLE_DEG 와 다릅니다.")
            print("     --probe-current 를 올려 다시 재 보세요. 전류가 모자라면")
            print("     중력을 거스르는 쪽 스토퍼에 못 닿아 중점이 그쪽으로 치우칩니다.")
            print("     두 번 다른 전류로 재서 중점이 같으면 믿을 수 있습니다.")
    print(f"\n  반영: 이 값을 캘리브레이션에 넘기면 프로파일의 home_deg 가 된다.")
    print(f"    python3 calibrate.py --zero {zero_deg:.2f} --out profiles/rover.json")
    print(f"    (radxa.py 는 프로파일의 home_deg 를 굴절 원점으로 그대로 쓴다.")
    print(f"     한 번에 하려면:  python3 calibrate.py --auto-zero)")
    print("══════════════════════════════════")


# ======================================================================
# limits — 양쪽 스토퍼의 중점
# ======================================================================
def push_to_stop(ctrl, motor, direction: int, args) -> float:
    """
    저전류로 스토퍼 쪽으로 밀어붙이고, 멈춘 지점의 각도를 돌려준다.

    전류 상한을 낮게 걸어 두므로 스토퍼를 때리는 힘이 제한된다. 속도가
    죽은 상태가 --stop-dwell 동안 이어지면 닿은 것으로 본다.
    """
    loop = RateLoop(ctrl.cfg.control_hz)
    start = motor.read().position_rad
    target = start + direction * math.radians(args.search_deg)

    ctrl.force_goal_current(args.probe_current)
    ctrl.set_profile(velocity_dps=args.probe_speed)
    ctrl.set_home(target)                     # 상태기계를 거치지 않고 목표만 쓴다

    quiet_since = None
    t_end = time.monotonic() + args.stop_timeout
    last = start
    while time.monotonic() < t_end:
        loop.wait()
        fb = motor.read()
        last = fb.position_rad
        if abs(fb.velocity_rad_s) * DEG < args.stop_vel:
            if quiet_since is None:
                quiet_since = time.monotonic()
            elif time.monotonic() - quiet_since >= args.stop_dwell:
                return last
        else:
            quiet_since = None
    print(f"  [경고] {args.stop_timeout:.0f} 초 안에 멈추지 않았습니다. "
          f"마지막 위치를 씁니다.")
    return last


def measure_zero_limits(args):
    """
    양쪽 기계 스토퍼를 눌러 중점을 재고 (zero_deg, travel_deg) 를 돌려준다.

    포트는 이 함수가 스스로 열고 닫는다. calibrate.py --auto-zero 가 본
    캘리브레이션을 시작하기 '전에' 이걸 부르고, 그다음 자기 포트를 연다.
    측정을 못 했으면 (None, None).
    """
    cfg, dxl, motor, ctrl = open_rig(args)
    print(f"\n양쪽 기계 스토퍼를 {args.probe_current} LSB "
          f"({args.probe_current * 2.69:.0f} mA ≈ "
          f"{lsb_to_mnm(args.probe_current):.0f} mN·m) 로 살짝 눌러 봅니다.")
    print("굴절부에 물리 스토퍼가 있어야 의미가 있습니다. "
          "주변에 손이나 케이블이 끼지 않았는지 확인하세요.")
    if not args.yes:
        if input("진행할까요? [y/N] ").strip().lower() not in ("y", "yes"):
            motor.shutdown()
            return None, None
    zero = travel = None
    try:
        plus = push_to_stop(ctrl, motor, +1, args)
        print(f"  +CCW 스토퍼 = {plus * DEG:+.2f}°")
        minus = push_to_stop(ctrl, motor, -1, args)
        print(f"  -CW  스토퍼 = {minus * DEG:+.2f}°")
        mid = 0.5 * (plus + minus) * DEG                 # 이번 실행의 엔코더 회전 기준
        travel = 0.5 * abs(plus - minus) * DEG
        print(f"  중점 = ({plus * DEG:+.2f} + {minus * DEG:+.2f}) / 2 "
              f"= {mid:+.2f}°,  편측 행정 {travel:.2f}°")
        # 확인 삼아 중점으로 보내 둔다
        ctrl.force_goal_current(cfg.current_moving)
        ctrl.goto_blocking(math.radians(mid), timeout=6.0)
        print(f"\n  중점 {mid:+.2f}° 로 이동했습니다. "
              f"지금 굴절각이 180° 로 보이는지 눈으로 확인하세요.")
        # [261016] 프로파일에는 [-180, 180) 로 접어 남긴다. 다음 전원 투입 때 엔코더가 어느
        # 회전으로 읽히든 radxa.py / Rig 가 unwrap_zero_deg 로 다시 맞춘다
        zero = wrap_deg(mid)
        if abs(zero - mid) > 1e-6:
            print(f"  (엔코더가 0/360° 경계를 넘어 셌습니다. 원점은 {zero:+.2f}° 로 저장합니다)")
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C")
    finally:
        ctrl.force_goal_current(cfg.current_holding)
        motor.shutdown()
    return zero, travel


def cmd_limits(args):
    zero, travel = measure_zero_limits(args)
    if zero is None:
        return 1
    report(zero, travel, "스토퍼 중점")
    if args.candidates:
        cands = [float(c) for c in args.candidates.split(",")]
        best = min(cands, key=lambda c: abs(wrap_deg(c - zero)))
        print(f"\n  후보 {cands} 중 가장 가까운 값 = {best:+.1f}° "
              f"(차이 {abs(wrap_deg(best - zero)):.2f}°)")
        if abs(wrap_deg(best - zero)) > 5.0:
            print("  ⚠ 후보와 5° 넘게 차이납니다. 결착 상태를 다시 보세요.")
    return 0


# ======================================================================
# check — 후보로 가서 눈으로 확인
# ======================================================================
def cmd_check(args):
    cfg, dxl, motor, ctrl = open_rig(args)
    cands = [float(c) for c in args.candidates.split(",") if c.strip()]
    print(f"\n후보 {cands} 를 차례로 방문합니다.")
    print("두 후보는 45° 떨어져 있습니다. 굴절각으로는 "
          f"{180 - (cands[0] - cands[-1]) if len(cands) > 1 else 45:.0f}° 차이라")
    print("육안으로 헷갈릴 수 없습니다.\n")
    found = None
    try:
        for c in cands:
            print(f"  → {c:+.2f}° 로 이동 중...")
            ctrl.force_goal_current(cfg.current_moving)
            ok = ctrl.goto_blocking(math.radians(to_this_turn(motor, c)), timeout=8.0)
            pos = motor.read().position_rad * DEG
            print(f"    도착 {pos:+.2f}° "
                  f"{'' if ok else '(목표에 완전히 도달하지 못했습니다)'}")
            ans = input("    지금 굴절각이 180° (평면 전개) 로 보입니까? [y/N] ")
            if ans.strip().lower() in ("y", "yes"):
                found = c
                break
        if found is None:
            print("\n어느 후보도 180° 가 아니었습니다.")
            print("  - goto 로 그 사이를 더듬어 보거나")
            print("  - limits 로 스토퍼 중점을 재는 쪽이 확실합니다.")
            return 1
        report(found, source="육안 확인")
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C")
    finally:
        ctrl.force_goal_current(cfg.current_holding)
        motor.shutdown()
    return 0


# ======================================================================
# hand — 토크 끄고 손으로 맞춘 뒤 읽기
# ======================================================================
def cmd_hand(args):
    cfg, dxl, motor, ctrl = open_rig(args)
    print("\n토크를 끕니다. 손으로 굴절각을 180° (평면 전개) 에 맞추세요.")
    print("맞췄으면 Ctrl-C. 직전 1 초의 중앙값을 원점으로 씁니다.\n")
    motor.torque(False)
    buf = []
    loop = RateLoop(20.0)
    try:
        while True:
            loop.wait()
            pos = motor.read().position_rad * DEG
            buf.append((time.monotonic(), pos))
            buf[:] = [b for b in buf if time.monotonic() - b[0] <= 1.0]
            sys.stdout.write(f"\r  엔코더 {pos:+8.2f}°   "
                             f"(최근 1초 중앙값 {_median([b[1] for b in buf]):+8.2f}°)  ")
            sys.stdout.flush()
    except KeyboardInterrupt:
        print()
    if len(buf) < 3:
        print("표본이 부족합니다.")
        motor.shutdown()
        return 1
    zero = wrap_deg(_median([b[1] for b in buf]))      # [261016] 회전 수와 무관하게 저장
    spread = max(b[1] for b in buf) - min(b[1] for b in buf)
    if spread > 1.0:
        print(f"  ⚠ 마지막 1초 동안 {spread:.2f}° 흔들렸습니다. "
              f"가만히 둔 상태에서 다시 재세요.")
    report(zero, source="손으로 맞춘 뒤 읽음")
    if args.candidates:
        cands = [float(c) for c in args.candidates.split(",")]
        best = min(cands, key=lambda c: abs(wrap_deg(c - zero)))
        print(f"\n  후보 {cands} 중 가장 가까운 값 = {best:+.1f}° "
              f"(차이 {abs(wrap_deg(best - zero)):.2f}°)")
    motor.shutdown()
    return 0


# ======================================================================
# goto — 임의 각도로 보내고 잡아 두기
# ======================================================================
def cmd_goto(args):
    cfg, dxl, motor, ctrl = open_rig(args)
    try:
        print(f"\n{args.angle:+.2f}° 로 이동합니다.")
        ctrl.force_goal_current(cfg.current_moving)
        ctrl.goto_blocking(math.radians(to_this_turn(motor, args.angle)), timeout=8.0)
        print(f"  도착 {motor.read().position_rad * DEG:+.2f}°. "
              f"{args.hold:.0f} 초 동안 잡아 둡니다 (Ctrl-C 로 종료).")
        loop = RateLoop(20.0)
        end = time.monotonic() + args.hold
        while time.monotonic() < end:
            loop.wait()
            fb = motor.read()
            sys.stdout.write(f"\r  {fb.position_rad * DEG:+8.2f}°   "
                             f"{fb.current_a * 1000:+7.1f} mA  ")
            sys.stdout.flush()
        print()
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C")
    finally:
        ctrl.force_goal_current(cfg.current_holding)
        motor.shutdown()
    return 0


# ======================================================================
def build_parser():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        add_config_args(p)
        p.add_argument("--candidates", default="22.5,-22.5",
                       help="후보 각도들 [deg], 콤마 구분")
        return p

    p = common(sub.add_parser("limits", help="스토퍼 중점 (가장 정확)"))
    p.add_argument("--probe-current", type=int, default=60,
                   help="스토퍼를 누르는 전류 상한 [LSB]")
    p.add_argument("--probe-speed", type=float, default=30.0,
                   help="접근 속도 [deg/s]")
    p.add_argument("--search-deg", type=float, default=70.0,
                   help="스토퍼를 찾아 이만큼까지 보낸다 [deg]")
    p.add_argument("--stop-vel", type=float, default=1.5,
                   help="이보다 느리면 멈춘 것으로 본다 [deg/s]")
    p.add_argument("--stop-dwell", type=float, default=0.5,
                   help="멈춘 상태가 이만큼 지속돼야 스토퍼로 인정 [s]")
    p.add_argument("--stop-timeout", type=float, default=15.0)
    p.add_argument("--yes", action="store_true", help="확인 질문을 건너뛴다")
    p.set_defaults(func=cmd_limits)

    p = common(sub.add_parser("check", help="후보로 이동해 육안 확인"))
    p.set_defaults(func=cmd_check)

    p = common(sub.add_parser("hand", help="토크 끄고 손으로 맞춘 뒤 읽기"))
    p.set_defaults(func=cmd_hand)

    p = common(sub.add_parser("goto", help="임의 각도로 보내고 잡아 두기"))
    p.add_argument("angle", type=float, help="목표 엔코더 각도 [deg]")
    p.add_argument("--hold", type=float, default=30.0, help="잡아 두는 시간 [s]")
    p.set_defaults(func=cmd_goto)
    return ap


def main(argv):
    args = build_parser().parse_args(argv[1:])
    args.base_overrides = list(args.overrides or [])
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
