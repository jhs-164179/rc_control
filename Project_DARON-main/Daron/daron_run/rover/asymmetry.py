#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
asymmetry.py — 복원 속도의 시계/반시계 비대칭을 '측정'하고 보정한다.

왜 생기는가
-----------
Current-based Position Control 에서 Goal Current 는 부호 없는 대칭 상한이다.
양방향에 같은 크기의 토크 한계를 건다. 그런데 실제 순 토크는

    tau_net = KT*I_cap - sign(v)*tau_friction - tau_gravity(theta)

이다. 중력 부하는 한 방향을 돕고 반대 방향을 방해하므로, 같은 I_cap 이라도
순 토크가 2*tau_gravity 만큼 차이난다. 그래서 보정항은 '곱셈(배율)'이 아니라
'덧셈(토크 오프셋)'이 물리적으로 옳은 형태다. 다만 동작 범위가 좁으면
배율 근사도 잘 맞으므로 두 형태를 모두 지원하고, 어느 쪽이 맞는지는
측정으로 고른다.

명령
----
  breakaway  방향별 기동 전류를 재서 tau_gravity 와 tau_friction 을 분리한다.
             한 번만 재면 보정 상수의 물리적 출발점이 나온다.
  bidir      양방향 복원 시험으로 비대칭 지수 R = t90(+)/t90(-) 를 측정한다.
  cal        R 이 1 이 되도록 보정값을 자동으로 수렴시킨다 (모델 불필요).

예)
  python3 asymmetry.py breakaway --angles -15,0,15
  python3 asymmetry.py bidir --amp 12 --reps 6
  python3 asymmetry.py cal --mode bias --iters 5 --save profiles/my_cal.json
"""

from __future__ import annotations

import argparse
import math
import sys
import time

from motor_control.xm430.metrics import _median
from experiments import Rig, run_step_series, summarize_events
from motor_control.xm430.config import (CURRENT_LSB_MA, DIR_NEG, DIR_POS, MNM_PER_LSB,
                         add_config_args, clamp)

NAN = float("nan")


# ======================================================================
# breakaway — 중력항과 마찰항의 분리
# ======================================================================
def _moves(rig, angle_rad: float, direction: int, raw: int, args) -> bool:
    """
    전류 상한을 raw 로 고정했을 때, 축이 direction 쪽으로 움직이는가?

    상태기계는 돌리지 않는다. 상태기계가 개입하면 전류를 제 마음대로
    바꿔 버려 측정이 무의미해진다.
    """
    ctrl, cfg = rig.ctrl, rig.cfg
    ctrl.force_goal_current(cfg.current_moving)
    ctrl.set_home(angle_rad)
    ctrl.goto_blocking(angle_rad, timeout=5.0)
    rig.run_for(0.25, "BRK_PARK", run_state_machine=False)

    _, fb0 = rig.sample(run_state_machine=False)
    start = fb0.position_rad

    # 목표를 그 방향으로 조금 옮기고 전류 상한을 내린다.
    ctrl.force_goal_current(raw)
    ctrl.set_home(angle_rad + direction * math.radians(args.probe))

    moved = False
    end = time.monotonic() + args.brk_dwell
    while time.monotonic() < end:
        _, fb = rig.sample(f"BRK:{direction}:{raw}", run_state_machine=False)
        travel = (fb.position_rad - start) * direction
        if travel > math.radians(args.brk_move):
            moved = True
            break                      # 움직인 걸 확인했으면 바로 멈춘다
        if -travel > math.radians(args.brk_runaway):
            # 전류 상한이 하중을 못 이겨 반대로 끌려간다. 이 시도는 '안 움직임'
            # 이고, 더 지켜보면 하중이 그대로 무너진다. 즉시 끊는다.
            break
    ctrl.force_goal_current(cfg.current_moving)
    ctrl.set_home(angle_rad)
    return moved


def measure_breakaway(rig, angle_rad: float, direction: int, args) -> float:
    """
    그 방향으로 축을 움직이기 시작하는 최소 전류 [LSB]. 이분탐색으로 찾는다.

    전류를 1 LSB 부터 올리는 상승 램프는 쓸 수 없다. 중력이 돕는 방향에서는
    램프를 시작하기도 전에 하중이 무너져, 늘 1 LSB 가 답으로 나온다.
    이분탐색은 매 시도를 '원위치 -> 고정 전류 -> 짧은 관찰 -> 복귀' 로 끊으므로
    어느 방향이든 같은 조건에서 재고, 이동량도 시도마다 제한된다.
    """
    hi = int(args.brk_max)
    if not _moves(rig, angle_rad, direction, hi, args):
        return NAN                     # 상한까지 올려도 안 움직인다
    lo = 1
    if _moves(rig, angle_rad, direction, lo, args):
        return float(lo)               # 1 LSB 로도 움직인다 (중력이 돕는 방향)
    while hi - lo > max(1, int(args.brk_step)):
        mid = (lo + hi) // 2
        if _moves(rig, angle_rad, direction, mid, args):
            hi = mid
        else:
            lo = mid
    return float(hi)


def cmd_breakaway(args):
    angles = [float(a) for a in args.angles.split(",") if a.strip()]
    rig = Rig(args, "breakaway", {"angles_deg": angles})
    print("\n기동 전류 측정 (이분탐색). 축에 실제 하중이 실린 상태여야 합니다.")
    print(f"방향마다 약 {2 + math.ceil(math.log2(max(args.brk_max, 2)))} 회 "
          f"시도하며, 매 시도마다 원위치로 되돌립니다.")
    print(f"{'각도[°]':>8} {'I+[LSB]':>9} {'I-[LSB]':>9} "
          f"{'중력[mN.m]':>12} {'마찰[mN.m]':>12}")
    rows = []
    try:
        for a in angles:
            tgt = rig.home + math.radians(a)
            ip = measure_breakaway(rig, tgt, DIR_POS, args)
            im = measure_breakaway(rig, tgt, DIR_NEG, args)
            if ip != ip or im != im:
                print(f"{a:8.1f}  측정 실패 (--brk-max {args.brk_max} 까지 "
                      f"움직이지 않았습니다). --brk-max 를 올리거나 "
                      f"--probe 를 키우세요.")
                continue
            grav = MNM_PER_LSB * (ip - im) / 2.0
            fric = MNM_PER_LSB * (ip + im) / 2.0
            rows.append((a, ip, im, grav, fric))
            if min(ip, im) <= 1:
                # 한쪽이 바닥에 붙었으면 단순 분리식이 성립하지 않는다.
                print(f"{a:8.1f} {ip:9.0f} {im:9.0f} "
                      f"{'(포화 — 아래 해석 참고)':>26}")
            else:
                print(f"{a:8.1f} {ip:9.0f} {im:9.0f} {grav:12.1f} {fric:12.1f}")
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C")
    finally:
        rig.close("breakaway")

    if not rows:
        return 1

    sat = [r for r in rows if min(r[1], r[2]) <= 1]
    print("\n────────── 해석 ──────────")
    if sat and args.i_fall <= 0:
        # 한쪽 방향이 1 LSB 로도 움직인다 = 그 방향은 중력이 혼자 끌고 간다.
        # 이러면 두 측정값만으로 중력항과 마찰항을 분리할 수 없다.
        ip = _median([r[1] for r in rows])
        im = _median([r[2] for r in rows])
        up = "+CCW" if ip > im else "-CW"
        dn = "-CW" if ip > im else "+CCW"
        big = max(ip, im)
        b = big / 2.0
        print(f"  {dn} 방향은 1 LSB 로도 움직입니다. 중력이 그 방향으로 혼자")
        print(f"  끌고 간다는 뜻이고, 이 측정만으로는 중력항과 마찰항을 "
              f"분리할 수 없습니다.")
        print(f"  알 수 있는 것:  tau_grav + tau_fric = "
              f"{big * MNM_PER_LSB:.1f} mN.m  ({up} 기동 전류 {big:.0f} LSB)")
        print(f"\n  분리하려면 sag 실험의 '붕괴 시작' 전류 I_fall 을 같이 넣으세요:")
        print(f"      python3 experiments.py sag ...        # I_fall 을 얻고")
        print(f"      python3 asymmetry.py breakaway --i-fall <그 값> ...")
        print(f"    (I_fall 은 tau_grav - tau_fric 에 해당하므로 둘이 모이면 "
              f"연립이 풀립니다)")
    elif sat and args.i_fall > 0:
        ip = _median([r[1] for r in rows])
        im = _median([r[2] for r in rows])
        big = max(ip, im)
        sign = 1.0 if ip > im else -1.0
        grav = MNM_PER_LSB * (big + args.i_fall) / 2.0
        fric = MNM_PER_LSB * (big - args.i_fall) / 2.0
        b = sign * (big + args.i_fall) / 2.0
        print(f"  기동 전류 {big:.0f} LSB (= tau_grav + tau_fric)")
        print(f"  붕괴 전류 {args.i_fall:.0f} LSB (= tau_grav - tau_fric)")
        print(f"  -> 중력 tau_grav = {grav:.1f} mN.m,  "
              f"마찰 tau_fric = {fric:.1f} mN.m")
    else:
        b = _median([(r[1] - r[2]) / 2.0 for r in rows])
        fric = _median([r[4] for r in rows])
        print(f"  중력 상당 전류  b = (I+ - I-)/2 = {b:+.1f} LSB "
              f"({b * CURRENT_LSB_MA:+.1f} mA, {b * MNM_PER_LSB:+.1f} mN.m)")
        print(f"  쿨롱 마찰       = {fric:.1f} mN.m  (양방향 공통 손실)")

    if len(rows) >= 3:
        span = max(r[3] for r in rows) - min(r[3] for r in rows)
        base = max(abs(r[3]) for r in rows) or 1.0
        if span / base > 0.3:
            print(f"\n  중력항이 각도에 따라 {span:.0f} mN.m 변합니다 (cos 의존). "
                  f"중력이 원인일 가능성이 높습니다.")
        else:
            print(f"\n  중력항이 각도와 거의 무관합니다 ({span:.0f} mN.m 변화). "
                  f"기구적 편향/케이블 간섭 쪽을 의심하세요.")

    print(f"\n  권장 초기 보정:  --set dir_comp_mode=bias "
          f"--set b_pos={b:+.0f} --set b_neg={-b:+.0f}")
    print("  (이건 출발점입니다. asymmetry.py cal 로 마무리하세요.)")
    print("  링크를 떼고 같은 측정을 해서 값이 사라지면 하중이 원인,")
    print("  남으면 모터/감속기 쪽 원인입니다.")
    if args.save:
        cfg = rig.cfg
        cfg.dir_comp_mode = "bias"
        cfg.b_pos, cfg.b_neg = round(b, 1), round(-b, 1)
        print(f"\n저장: {cfg.save(args.save)}")
    return 0


# ======================================================================
# bidir — 비대칭 지수 측정
# ======================================================================
def cmd_bidir(args):
    rig = Rig(args, "bidir", {"amp_deg": args.amp, "reps": args.reps})
    try:
        rig.ctrl.goto_blocking(rig.home, timeout=5.0)
        evs = run_step_series(rig, [args.amp, -args.amp], args.reps,
                              args.settle, args.timeout)
        R = summarize_events(evs, "(양방향)")
        if R == R:
            if 0.95 <= R <= 1.05:
                print("  -> 목표 범위 안입니다. 보정이 필요 없습니다.")
            else:
                slow = "+CCW" if R > 1 else "-CW"
                print(f"  -> {slow} 쪽이 {abs(1 - R) * 100:.0f} % 느립니다. "
                      f"asymmetry.py cal 로 보정하세요.")
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C")
    finally:
        rig.close("bidir")
    return 0


# ======================================================================
# cal — 보정값 자동 수렴
# ======================================================================
def _apply_x(cfg, mode: str, x: float):
    """
    보정 파라미터 하나(x)를 양방향에 반대 부호로 건다. 평균 세기는 유지된다.
    x > 0 이면 언제나 +CCW 를 세게/빠르게, -CW 를 약하게/느리게 만든다.

    mode 세 가지는 '무엇이 병목인가'에 대응한다.
      bias    전류가 병목  — 중력 오프셋을 덧셈으로 상쇄 (물리적으로 옳은 형태)
      scale   전류가 병목  — 배율 근사. 동작 범위가 좁을 때만
      profile 궤적이 병목  — 전류를 굶기지 않고 빠른 쪽의 목표 속도를 낮춘다.
                             감속 토크가 유지되므로 오버슈트에 유리하다.
    """
    # 모드를 바꿔 가며 부를 수 있으므로 매번 다른 채널은 중립으로 되돌린다.
    cfg.profile_velocity_pos_dps = 0.0
    cfg.profile_velocity_neg_dps = 0.0
    cfg.k_pos = cfg.k_neg = 1.0
    cfg.b_pos = cfg.b_neg = 0.0
    if mode == "bias":
        cfg.dir_comp_mode = "bias"
        cfg.b_pos, cfg.b_neg = x, -x
    elif mode == "scale":
        cfg.dir_comp_mode = "scale"
        cfg.k_pos, cfg.k_neg = 1.0 + x, 1.0 - x
    else:
        cfg.dir_comp_mode = "off"
        v = cfg.profile_velocity_dps
        cfg.profile_velocity_pos_dps = max(5.0, v * (1.0 + x))
        cfg.profile_velocity_neg_dps = max(5.0, v * (1.0 - x))


def _measure_R(rig, args, tag: str):
    """현재 설정으로 양방향 복원을 재고 R = t90(+)/t90(-) 를 돌려준다."""
    evs_before = len(rig.ctrl.events)
    run_step_series(rig, [args.amp, -args.amp], args.reps,
                    args.settle, args.timeout, quiet=True)
    evs = rig.ctrl.events[evs_before:]
    pos = [e.t90_s for e in evs if e.direction > 0]
    neg = [e.t90_s for e in evs if e.direction < 0]
    tp, tn = _median(pos), _median(neg)
    if tp != tp or tn != tn or tn <= 0:
        return NAN, tp, tn, evs
    return tp / tn, tp, tn, evs


def _bottleneck_note(rig, evs) -> str:
    """
    피크 전류가 상한에 닿지 않았다면 복원을 제한한 것은 전류가 아니다.
    그 상태에서 전류 보정을 아무리 돌려도 R 은 움직이지 않는다.
    """
    if not evs:
        return ""
    ipk = _median([e.peak_current_mA for e in evs])
    cap = _median([float(e.goal_current) for e in evs]) * CURRENT_LSB_MA
    if cap <= 0:
        return ""
    ratio = ipk / cap
    if ratio < 0.8:
        return (f"  [진단] 피크 전류가 상한의 {ratio * 100:.0f} % 에 그쳤습니다. "
                f"복원을 제한한 것은 전류가 아니라 궤적입니다.\n"
                f"          --mode profile 로 다시 돌리세요.")
    return ""


def cmd_cal(args):
    rig = Rig(args, "cal", {"mode": args.mode, "amp_deg": args.amp})
    cfg = rig.cfg
    x_max = args.x_max if args.x_max > 0 else {
        "bias": 0.5 * cfg.current_recover, "scale": 0.4, "profile": 0.6,
    }[args.mode]
    print(f"\n보정 파라미터 x 를 {args.mode} 모드로 수렴시킵니다 "
          f"(|x| <= {x_max:g}).")
    print("  x > 0 이면 +CCW 복원을 세게, -CW 복원을 약하게 만듭니다.")
    print(f"\n{'iter':>4} {'x':>9} {'I+':>5} {'I-':>5} {'t90+[ms]':>10} "
          f"{'t90-[ms]':>10} {'R':>7}")

    hist = []
    converged = False
    try:
        rig.ctrl.goto_blocking(rig.home, timeout=5.0)
        # 1) x=0 에서 보정 전 기준선을 먼저 잰다. 2) x0 로 한 번 흔든다.
        # 3) 그 다음부터는 시컨트법으로 R=1 을 향해 좁힌다.
        x_next = 0.0
        x_second = (args.x0 if args.x0 is not None else
                    (0.15 * cfg.current_recover if args.mode == "bias" else 0.12))
        for it in range(args.iters):
            x = clamp(x_next, -x_max, x_max)
            _apply_x(cfg, args.mode, x)
            rig.ctrl.invalidate_current_cache()
            R, tp, tn, evs = _measure_R(rig, args, f"CAL:{it}")
            if args.mode == "profile":
                ip = int(cfg.recover_profile_velocity(DIR_POS))
                im = int(cfg.recover_profile_velocity(DIR_NEG))
            else:
                ip, im = cfg.recover_current(DIR_POS), cfg.recover_current(DIR_NEG)
            rs = "    --" if R != R else f"{R:7.3f}"
            print(f"{it:>4} {x:>9.3f} {ip:>5d} {im:>5d} "
                  f"{tp * 1000:>10.0f} {tn * 1000:>10.0f} {rs}")
            if it == 0 and args.mode in ("bias", "scale"):
                note = _bottleneck_note(rig, evs)
                if note:
                    print(note)
            if R != R:
                print("  t90 을 못 구했습니다. --amp 를 키우거나 "
                      "settled_band_deg 를 확인하세요.")
                break
            hist.append((x, R))
            if abs(R - 1.0) <= args.tol:
                print(f"\n  수렴: |R-1| <= {args.tol}")
                converged = True
                break
            if len(hist) >= 2 and abs(abs(x) - x_max) < 1e-9 \
                    and abs(abs(hist[-2][0]) - x_max) < 1e-9:
                print(f"\n  [중단] 보정값이 한계 |x| = {x_max:g} 에 붙었는데도 "
                      f"R = {R:.3f} 입니다.")
                print("         이 손잡이 하나로 지울 수 있는 비대칭이 아닙니다. "
                      "--x-max 를 키우거나,")
                print("         다른 --mode 와 병행하거나, 기구 쪽(마찰/케이블/"
                      "링크 기하)을 의심하세요.")
                break
            if len(hist) < 2:
                x_next = x_second
                continue
            # 시컨트법. f(x)=ln R 은 x 에 대해 단조 감소한다고 본다.
            (x0, R0), (x1, R1) = hist[-2], hist[-1]
            f0, f1 = math.log(R0), math.log(R1)
            if abs(x1 - x0) > 1e-9 and abs(f1 - f0) < 0.02:
                # x 를 크게 움직였는데 R 이 꿈쩍도 안 한다 = 이 손잡이가
                # 병목이 아니다. 여기서 외삽하면 경계로 튕겨나갈 뿐이다.
                print(f"  [중단] x 를 {x0:g} -> {x1:g} 로 바꿨는데 R 이 "
                      f"{R0:.3f} -> {R1:.3f} 밖에 안 변했습니다.")
                print(f"         이 손잡이는 병목이 아닙니다. "
                      f"{'--mode profile' if args.mode != 'profile' else '--mode bias'}"
                      f" 로 바꿔 보세요.")
                break
            if abs(f1 - f0) < 1e-6 or abs(x1 - x0) < 1e-9:
                x_next = clamp(x1 + math.copysign(0.1 * x_max, f1), -x_max, x_max)
            else:
                nxt = x1 - f1 * (x1 - x0) / (f1 - f0)
                # 시컨트 한 걸음의 크기를 제한한다. 잡음이 섞인 측정에서
                # 기울기가 작게 잡히면 한 번에 경계까지 날아간다.
                lim = max(0.35 * x_max, 2.0 * abs(x1 - x0))
                nxt = clamp(nxt, x1 - lim, x1 + lim)
                x_next = clamp(nxt, -x_max, x_max)
        else:
            print(f"\n  {args.iters} 회 안에 수렴하지 않았습니다. "
                  f"--iters 를 늘리거나 --amp 를 키워 보세요.")
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C")
    finally:
        rig.close(f"cal {args.mode}")

    if not hist:
        return 1
    best = min(hist, key=lambda h: abs(h[1] - 1.0))
    _apply_x(cfg, args.mode, best[0])
    print(f"\n────────── 결과 ──────────")
    print(f"  x = {best[0]:+.3f}   R = {best[1]:.3f}")
    if args.mode == "bias":
        print(f"  --set dir_comp_mode=bias --set b_pos={cfg.b_pos:+.1f} "
              f"--set b_neg={cfg.b_neg:+.1f}")
        print(f"  복원 전류: +CCW {cfg.recover_current(DIR_POS)} LSB / "
              f"-CW {cfg.recover_current(DIR_NEG)} LSB")
    elif args.mode == "scale":
        print(f"  --set dir_comp_mode=scale --set k_pos={cfg.k_pos:.3f} "
              f"--set k_neg={cfg.k_neg:.3f}")
        print(f"  복원 전류: +CCW {cfg.recover_current(DIR_POS)} LSB / "
              f"-CW {cfg.recover_current(DIR_NEG)} LSB")
    else:
        print(f"  --set profile_velocity_pos_dps={cfg.profile_velocity_pos_dps:.0f} "
              f"--set profile_velocity_neg_dps={cfg.profile_velocity_neg_dps:.0f}")
        print(f"  복원 궤적속도: +CCW {cfg.profile_velocity_pos_dps:.0f}°/s / "
              f"-CW {cfg.profile_velocity_neg_dps:.0f}°/s")
    if not converged:
        print("  ⚠ 수렴하지 않은 결과입니다. 그대로 쓰지 마세요.")
    if args.save or converged:
        path = args.save or "profiles/asymmetry_cal.json"
        print(f"  저장: {cfg.save(path)}")
        print("\n  이제 release 실험으로 '실제 외력' 상황에서도 R 이 유지되는지 "
              "확인하세요:")
        print(f"    python3 experiments.py release --profile {path} --duration 60")
    else:
        print("  (수렴하지 않아 저장하지 않았습니다. 강제로 남기려면 --save 를 쓰세요.)")
    return 0


# ======================================================================
def build_parser():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        add_config_args(p)
        p.add_argument("--home-here", action="store_true")
        p.add_argument("--limit-deg", type=float, default=-1.0,
                       help="home 기준 가동 범위 ±[deg]. 0=제한 없음, "
                            "생략하면 프로파일 값")
        p.add_argument("--no-log", action="store_true")
        return p

    p = common(sub.add_parser("breakaway", help="방향별 기동 전류 -> 중력/마찰 분리"))
    p.add_argument("--angles", default="0", help="측정 각도들 [deg], 콤마 구분")
    p.add_argument("--probe", type=float, default=4.0, help="목표 오프셋 [deg]")
    p.add_argument("--brk-step", type=int, default=1, help="이분탐색 해상도 [LSB]")
    p.add_argument("--brk-dwell", type=float, default=0.4,
                   help="한 시도에서 움직이는지 지켜보는 시간 [s]")
    p.add_argument("--brk-max", type=int, default=200, help="탐색 상한 [LSB]")
    p.add_argument("--brk-move", type=float, default=0.5, help="기동 판정 변위 [deg]")
    p.add_argument("--brk-runaway", type=float, default=3.0,
                   help="반대 방향으로 이만큼 끌려가면 그 시도를 즉시 중단 [deg]")
    p.add_argument("--i-fall", type=float, default=0.0,
                   help="sag 실험에서 얻은 '붕괴 시작' 전류 [LSB]. "
                        "한쪽 방향이 포화됐을 때 중력/마찰 분리에 쓴다")
    p.add_argument("--save", default="", help="결과를 프로파일 JSON 으로 저장")
    p.set_defaults(func=cmd_breakaway)

    p = common(sub.add_parser("bidir", help="비대칭 지수 R 측정"))
    p.add_argument("--amp", type=float, default=12.0)
    p.add_argument("--reps", type=int, default=6)
    p.add_argument("--settle", type=float, default=0.6)
    p.add_argument("--timeout", type=float, default=4.0)
    p.set_defaults(func=cmd_bidir)

    p = common(sub.add_parser("cal", help="보정값 자동 수렴"))
    p.add_argument("--mode", choices=("bias", "scale", "profile"), default="bias",
                   help="bias/scale=전류 보정, profile=궤적속도 보정")
    p.add_argument("--amp", type=float, default=12.0)
    p.add_argument("--reps", type=int, default=3)
    p.add_argument("--iters", type=int, default=6)
    p.add_argument("--tol", type=float, default=0.03, help="|R-1| 허용치")
    p.add_argument("--x0", type=float, default=None, help="두 번째 시도값")
    p.add_argument("--x-max", type=float, default=0.0, help="|x| 상한 (0=자동)")
    p.add_argument("--settle", type=float, default=0.6)
    p.add_argument("--timeout", type=float, default=4.0)
    p.add_argument("--save", default="", help="프로파일 저장 경로")
    p.set_defaults(func=cmd_cal)
    return ap


def main(argv):
    args = build_parser().parse_args(argv[1:])
    args.base_overrides = list(args.overrides or [])
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
