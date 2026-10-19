#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
calibrate.py — 캘리브레이션 한 번에.

experiments.py / asymmetry.py 의 개별 명령을 순서대로 치는 대신, 꼭 필요한
세 가지만 자동으로 돌리고 완성된 프로파일을 저장한다. 5 분쯤 걸린다.

    python3 calibrate.py --auto-zero --out profiles/rover.json

  0) 굴절 원점      --auto-zero 를 주면 스토퍼 중점을 먼저 재서 home_deg 로 쓴다
  1) 유지 전류      차체가 주저앉기 시작하는 전류를 찾아 --hold-margin 배(기본 1.8) 여유를 준다
  2) 비대칭 측정    양방향 복원 속도비 R 을 잰다
  3) 비대칭 보정    R 이 5 % 밖일 때만. 전류/궤적 중 병목인 쪽을 알아서 고른다

여기서 빠진 것 (필요할 때만 따로)
  - 중력/마찰 분리   asymmetry.py breakaway   비대칭의 '원인'이 궁금할 때
  - 게인 최적화      experiments.py sweep     기본 게인이 마음에 안 들 때

원점을 이미 알고 있으면 재지 말고 그 값을 넣는다:
    python3 calibrate.py --zero 22.5 --out profiles/rover.json

원점을 손으로 확인하며 잡고 싶으면 find_zero.py 를 따로 쓴다 (check / hand / goto).
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import time

from asymmetry import _apply_x, _bottleneck_note, _measure_R
from find_zero import measure_zero_limits, report as report_zero
from experiments import (HOLD_MARGIN, Rig, measure_load, round_up, run_step_series,
                         sag_interpret, sag_sweep, summarize_events)
from motor_control.xm430.config import (DIR_NEG, DIR_POS, add_config_args, clamp,
                         config_from_args, lsb_to_mnm)

AUTO_CURRENT_MAX = 600      # [261016] 자동 전류 조정 상한 = radxa ShapeConfig.hold_current

WATCH = ("p_gain", "current_limit", "current_moving", "current_recover",
         "current_holding", "current_comply", "settled_band_deg",
         "comply_band_deg", "home_deg",
         "dir_comp_mode", "b_pos", "b_neg",
         "profile_velocity_pos_dps", "profile_velocity_neg_dps")


def banner(n, total, title):
    print(f"\n{'═' * 62}")
    print(f"  [{n}/{total}] {title}")
    print("═" * 62)


def stage_holding(rig, args):
    """1단계 — 차체를 받치는 최소 유지 전류.

    [261016] 자동 전류 조정 (--no-auto-current 로 끔). 무거워져서 처음부터 실패하면
    사람이 값을 고쳐 다시 돌리지 않아도 되게, 상한(--current-max) 안에서 알아서 올린다.
      a) home 에서 버티는 전류(중력 부하)를 재고, 못 버티면 current_moving / p_gain 을 올린다
      b) sag 시작 전류를 부하의 1.5 배 이상으로. 첫 단계부터 무너지면 1.5 배씩 올려 다시
      c) 나온 holding 이 recover 이상이면 recover / moving 을 올린다
    """
    banner(1, 3, "유지 전류 — 차체가 주저앉는 지점 찾기")
    print("  차체 하중이 굴절축에 실린 상태여야 합니다. 축을 놓고 기다리세요.\n")
    cfg = rig.cfg
    cmax = cfg.current_limit if args.auto_current else None

    load = measure_load(rig, cmax, p_max=args.p_max if args.auto_current else None)
    if load is None:
        return False
    start, step = args.sag_start, args.sag_step
    if args.auto_current and start < 1.3 * load:
        start = min(cmax, round_up(1.5 * load, 10))
        print(f"  -> sag 시작 전류를 {args.sag_start} -> {start} LSB 로 올립니다 (부하 x 1.5).")
    while True:
        if args.auto_current:
            step = max(args.sag_step, round_up(start / 20.0, 5))   # 시작이 높아도 20 단계 안팎
        results = sag_sweep(rig, start, args.sag_end, step,
                            args.sag_dwell, args.max_droop)
        early = 0 < len(results) < 3                               # 시작하자마자 무너짐
        if not (early and args.auto_current and start < cmax):
            break
        new = min(cmax, round_up(start * 1.5, 10))
        print(f"\n  -> {start} LSB 부터 이미 무너졌습니다. 시작 전류를 {new} LSB 로 올려 다시 잽니다.")
        start = new
        if cfg.current_moving < start:                             # home 으로 되돌릴 힘도 같이
            cfg.current_moving = min(cmax, start + 50)
            rig.ctrl.invalidate_current_cache()

    out = sag_interpret(results, args.droop_ok, margin=args.hold_margin)
    if out is None:
        print("  측정 실패. 위 메시지를 보세요 (하중이 실려 있는지, 전류가 하중을 이기는지)."
              + ("" if args.auto_current else " --no-auto-current 를 빼면 전류를 알아서 올립니다."))
        return False
    cfg.current_holding = min(out["holding"], cfg.current_limit)
    cfg.current_comply = out["comply"]
    cfg.settled_band_deg = out["settled_band_deg"]
    cfg.comply_band_deg = max(cfg.comply_band_deg, out["comply_band_deg"])
    rig.ctrl.invalidate_current_cache()
    print(f"\n  -> current_holding = {cfg.current_holding} LSB "
          f"({lsb_to_mnm(cfg.current_holding):.0f} mN·m), "
          f"comply = {cfg.current_comply}, "
          f"settled_band = {cfg.settled_band_deg}°, "
          f"comply_band = {cfg.comply_band_deg}°")
    if out["holding"] > cfg.current_limit:
        print(f"  주의: 계산한 holding {out['holding']} 이 current_limit {cfg.current_limit} 를 넘어 "
              f"상한으로 잘랐습니다. --current-max 를 올리세요.")
    if not out["margin"]:
        print("  주의: 시험 범위 안에서 무너지지 않았습니다. 시험한 가장 낮은 전류에 "
              "여유를 곱했습니다. --sag-end 를 더 낮춰 다시 재면 값을 줄일 수 있습니다.")
    if cfg.current_holding >= cfg.current_recover:
        # 복원이 유지보다 약하면 2·3단계가 의미가 없다
        lim = cfg.current_limit
        rec = min(lim, round_up(cfg.current_holding * 1.5, 10))     # 상한에 걸리면 상한까지만
        mov = min(lim, max(cfg.current_moving, rec + 50))
        if not args.auto_current or rec < cfg.current_holding * 1.1:
            print(f"  [실패] holding({cfg.current_holding}) >= recover({cfg.current_recover}). "
                  + ("자동 조정이 꺼져 있습니다." if not args.auto_current
                     else f"상한 {lim} 안에서 recover 를 holding 보다 충분히 크게 못 잡습니다. "
                          f"--current-max {round_up(cfg.current_holding * 1.5 + 50, 10)} 로 다시 돌리세요."))
            return False
        print(f"  -> holding 이 recover 이상이라 current_recover {cfg.current_recover} -> {rec}, "
              f"current_moving {cfg.current_moving} -> {mov} LSB 로 올립니다."
              + (" (상한에 걸림)" if rec == lim else ""))
        cfg.current_recover, cfg.current_moving = rec, mov
        rig.ctrl.invalidate_current_cache()
    return True


def stage_measure(rig, args):
    """2단계 — 비대칭 지수 R."""
    banner(2, 3, "비대칭 측정 — 시계/반시계 복원 속도가 같은가")
    print(f"  모터가 스스로 ±{args.amp:.0f}° 변위를 만들고 복원시킵니다. "
          f"손대지 마세요.\n")
    n0 = len(rig.ctrl.events)
    run_step_series(rig, [args.amp, -args.amp], args.reps,
                    args.settle, args.timeout)
    evs = rig.ctrl.events[n0:]
    R = summarize_events(evs, "(양방향)")
    note = _bottleneck_note(rig, evs)
    if note:
        print(note)
    return R, bool(note)


def stage_correct(rig, args, mode):
    """3단계 — R = 1 이 되도록 보정값 수렴."""
    banner(3, 3, f"비대칭 보정 — {mode} 모드")
    cfg = rig.cfg
    x_max = {"bias": 0.5 * cfg.current_recover, "profile": 0.6}[mode]
    x_second = (0.15 * cfg.current_recover) if mode == "bias" else 0.12
    print(f"  x > 0 이면 +CCW 를 세게/빠르게, -CW 를 약하게/느리게 합니다.")
    print(f"\n{'회':>3} {'x':>9} {'+CCW':>6} {'-CW':>6} "
          f"{'t90+[ms]':>10} {'t90-[ms]':>10} {'R':>7}")

    hist = []
    x_next = 0.0
    converged = False
    for it in range(args.iters):
        x = clamp(x_next, -x_max, x_max)
        _apply_x(cfg, mode, x)
        rig.ctrl.invalidate_current_cache()
        R, tp, tn, _ = _measure_R(rig, args, f"CAL:{it}")
        if mode == "profile":
            a = f"{cfg.recover_profile_velocity(DIR_POS):.0f}"
            b = f"{cfg.recover_profile_velocity(DIR_NEG):.0f}"
        else:
            a = f"{cfg.recover_current(DIR_POS)}"
            b = f"{cfg.recover_current(DIR_NEG)}"
        print(f"{it:>3} {x:>9.2f} {a:>6} {b:>6} "
              f"{tp * 1000:>10.0f} {tn * 1000:>10.0f} "
              f"{'    --' if R != R else f'{R:7.3f}'}")
        if R != R:
            print("  t90 을 못 구했습니다. --amp 를 키워 보세요.")
            break
        hist.append((x, R))
        if abs(R - 1.0) <= args.tol:
            converged = True
            break
        if len(hist) < 2:
            x_next = x_second
            continue
        (x0, R0), (x1, R1) = hist[-2], hist[-1]
        f0, f1 = math.log(R0), math.log(R1)
        if abs(x1 - x0) > 1e-9 and abs(f1 - f0) < 0.02:
            print(f"  x 를 바꿔도 R 이 거의 안 변합니다. "
                  f"이 손잡이는 병목이 아닙니다.")
            break
        if abs(f1 - f0) < 1e-6:
            x_next = clamp(x1 + math.copysign(0.1 * x_max, f1), -x_max, x_max)
        else:
            nxt = x1 - f1 * (x1 - x0) / (f1 - f0)
            lim = max(0.35 * x_max, 2.0 * abs(x1 - x0))
            x_next = clamp(clamp(nxt, x1 - lim, x1 + lim), -x_max, x_max)

    if not hist:
        return False, mode
    best = min(hist, key=lambda h: abs(h[1] - 1.0))
    _apply_x(cfg, mode, best[0])
    rig.ctrl.invalidate_current_cache()
    print(f"\n  -> x = {best[0]:+.2f},  R = {best[1]:.3f}"
          f"{'  (수렴)' if converged else '  (미수렴 — 가장 가까운 값)'}")
    return converged, mode


def main(argv):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_config_args(ap)
    ap.add_argument("--out", default="profiles/rover.json",
                    help="완성된 프로파일 저장 경로")
    ap.add_argument("--zero", type=float, default=None,
                    help="이미 아는 굴절 원점 [deg]. home_deg 로 쓴다")
    ap.add_argument("--auto-zero", action="store_true",
                    help="0단계: 양쪽 스토퍼 중점을 먼저 재서 home_deg 로 쓴다 "
                         "(결착을 다시 했다면 켤 것)")
    # --auto-zero 일 때 find_zero.measure_zero_limits 가 쓰는 값들
    ap.add_argument("--probe-current", type=int, default=60,
                    help="스토퍼를 누르는 전류 상한 [LSB]")
    ap.add_argument("--probe-speed", type=float, default=30.0)
    ap.add_argument("--search-deg", type=float, default=70.0)
    ap.add_argument("--stop-vel", type=float, default=1.5)
    ap.add_argument("--stop-dwell", type=float, default=0.5)
    ap.add_argument("--stop-timeout", type=float, default=15.0)
    ap.add_argument("--home-here", action="store_true",
                    help="시작 위치를 기준 자세로 (원점을 모를 때)")
    ap.add_argument("--limit-deg", type=float, default=-1.0)
    ap.add_argument("--no-log", action="store_true")
    ap.add_argument("--yes", action="store_true", help="확인 질문을 건너뛴다")
    # 측정 조건
    ap.add_argument("--amp", type=float, default=12.0, help="시험 변위 [deg]")
    ap.add_argument("--reps", type=int, default=4, help="방향당 반복 횟수")
    ap.add_argument("--settle", type=float, default=0.6)
    ap.add_argument("--timeout", type=float, default=4.0)
    ap.add_argument("--iters", type=int, default=5, help="보정 반복 상한")
    ap.add_argument("--tol", type=float, default=0.05, help="|R-1| 허용치")
    # sag 조건
    ap.add_argument("--sag-start", type=int, default=60)
    ap.add_argument("--sag-end", type=int, default=5)
    ap.add_argument("--sag-step", type=int, default=5)
    ap.add_argument("--sag-dwell", type=float, default=1.5)
    ap.add_argument("--droop-ok", type=float, default=0.5)
    ap.add_argument("--max-droop", type=float, default=8.0)
    ap.add_argument("--hold-margin", type=float, default=HOLD_MARGIN,
                    help=f"유지 전류 = 붕괴점 x 이 배수 (기본 {HOLD_MARGIN:g}, 261015 전 1.5)")
    ap.add_argument("--current-max", type=int, default=AUTO_CURRENT_MAX,
                    help=f"자동 전류 조정 상한 [LSB] (기본 {AUTO_CURRENT_MAX} = radxa 형상 전환 전류). "
                         f"current_limit 이 이보다 낮으면 이 값으로 올려 연다")
    ap.add_argument("--p-max", type=int, default=800,
                    help="자동 조정 때 p_gain 상한 (기본 800 = radxa 형상 전환 P)")
    ap.add_argument("--no-auto-current", dest="auto_current", action="store_false",
                    help="처음부터 실패해도 전류를 자동으로 올리지 않는다 (예전 동작)")
    ap.add_argument("--skip-holding", action="store_true",
                    help="1단계를 건너뛴다 (유지 전류를 이미 알 때)")
    args = ap.parse_args(argv[1:])
    args.base_overrides = list(args.overrides or [])
    if args.zero is not None and args.auto_zero:
        ap.error("--zero 와 --auto-zero 는 같이 쓸 수 없습니다. "
                 "원점을 이미 알면 --zero, 지금 재려면 --auto-zero.")

    print(__doc__.split("여기서 빠진 것")[0].rstrip())
    if not args.yes:
        print("\n  확인:  차체 하중이 굴절축에 실려 있고, 축 주변에 손이나")
        print("         케이블이 없어야 합니다. 측정 중에는 건드리지 마세요.")
        if input("\n  시작할까요? [y/N] ").strip().lower() not in ("y", "yes"):
            return 1

    # 0단계 — 굴절 원점. 포트를 자기가 열고 닫으므로 Rig 보다 먼저 끝내야 한다.
    if args.auto_zero:
        banner(0, 3, "굴절 원점 — 양쪽 스토퍼의 중점")
        zero, travel = measure_zero_limits(args)
        if zero is None:
            print("  원점을 재지 못했습니다. 중단합니다.")
            return 1
        report_zero(zero, travel, "스토퍼 중점")
        args.zero = zero

    if args.zero is not None:
        args.overrides = (args.overrides or []) + [f"home_deg={args.zero}"]

    before = config_from_args(args).to_dict()
    if args.auto_current and before["current_limit"] < args.current_max:
        # current_limit 은 EEPROM 이라 토크를 꺼야 바꿀 수 있다. 하중이 실린 채 실행 중에 끄면
        # 차체가 떨어지므로 처음 열 때 상한까지 올려 둔다 (Goal Current 가 실제 전류를 정한다)
        print(f"  current_limit {before['current_limit']} -> {args.current_max} LSB "
              f"(자동 전류 조정 상한, radxa.py 도 형상 전환 때 600 으로 올린다)")
        args.overrides = (args.overrides or []) + [f"current_limit={args.current_max}"]

    rig = Rig(args, "calibrate", {"mode": "auto"})
    t0 = time.monotonic()
    ok_hold = True
    R = float("nan")
    corrected = None
    try:
        if not args.skip_holding:
            ok_hold = stage_holding(rig, args)
        else:
            banner(1, 3, "유지 전류 — 건너뜀 (--skip-holding)")
        if not ok_hold:
            # [261016] 유지 전류가 틀리면 2·3단계도 틀린다. 기존 프로파일을 덮어쓰지 않고 멈춘다
            print(f"\n  1단계 실패 — 저장하지 않고 멈춥니다 ({args.out} 는 그대로).")
            return 1
        rig.ctrl.goto_blocking(rig.home, timeout=6.0)

        R, profile_bound = stage_measure(rig, args)
        if R != R:
            print("\n  비대칭을 재지 못했습니다. --amp 를 키워 다시 돌리세요.")
        elif abs(R - 1.0) <= args.tol:
            banner(3, 3, "비대칭 보정 — 불필요")
            print(f"  R = {R:.3f} 이 이미 허용치({args.tol}) 안입니다. "
                  f"보정하지 않습니다.")
        else:
            mode = "profile" if profile_bound else "bias"
            done, mode = stage_correct(rig, args, mode)
            corrected = mode
            if not done and mode == "bias":
                # 전류로 안 되면 궤적 쪽으로 한 번 더
                print("\n  전류 보정으로는 부족합니다. 궤적 속도로 다시 시도합니다.")
                _apply_x(rig.cfg, "bias", 0.0)
                done, mode = stage_correct(rig, args, "profile")
                corrected = mode
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C")
    finally:
        rig.close("auto calibrate")

    # ---------- 요약 ----------
    cfg = rig.cfg
    after = cfg.to_dict()
    print(f"\n{'═' * 62}")
    print(f"  결과  ({time.monotonic() - t0:.0f} 초 소요)")
    print("═" * 62)
    changed = [(k, before[k], after[k]) for k in WATCH if before[k] != after[k]]
    if changed:
        w = max(len(k) for k, _, _ in changed)
        for k, a, b in changed:
            print(f"  {k.ljust(w)}  {a}  ->  {b}")
    else:
        print("  바뀐 값이 없습니다.")
    if R == R:
        print(f"\n  비대칭 지수 R = {R:.3f}"
              f"{f' -> {corrected} 보정 적용' if corrected else ''}")
    print(cfg.summary())
    try:
        cfg.validate()
    except ValueError as e:
        print(f"\n  ⚠ 설정이 모순됩니다. 저장은 하지만 그대로 쓰지 마세요.\n{e}")
    path = cfg.save(args.out)
    print(f"\n  저장: {path}")
    print(f"\n  다음:")
    print(f"    python3 run_suspension.py --profile {args.out}"
          f"      # 실제로 눌러 보며 확인 (로컬, ROS 없이)")
    if os.path.basename(args.out) == "rover.json":
        print(f"    python3 radxa.py"
              f"                                      # 메인 기동 (이 프로파일을 읽는다)")
    else:
        print(f"    python3 radxa.py --ros-args -p susp_profile:={args.out}"
              f"   # 메인 기동")
    print(f"    python3 experiments.py sweep --profile {args.out} \\")
    print(f"        --grid p_gain=80,120,160 --grid d_gain=400,600,800"
          f"   # 더 조이고 싶을 때")
    return 0 if ok_hold else 1


if __name__ == "__main__":
    sys.exit(main(sys.argv))
