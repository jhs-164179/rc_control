#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
run_suspension.py — 찾은 프로파일 그대로 서스펜션을 '실제로' 돌린다.

experiments.py 의 release 와 같은 루프지만, 실험용 장치가 전혀 없다.
지령 이동도, 파라미터 변경도, 스텝 시험도 하지 않는다. 프로파일을 읽어
그대로 물린 뒤 상태기계만 돌린다. 로버에 얹기 전 마지막 확인이자,
서스펜션 단독 운용에 그대로 쓸 수 있는 형태다.

    python3 run_suspension.py --profile rover_final

가동 범위
---------
`travel_limit_deg` 는 '프로그램이 이 밖을 지령하지 않는다'는 뜻이다.
외력에 밀려 그 밖으로 나가는 것은 막지 않는다 — 서스펜션은 눌리는 것이
일이고, 그때 실행을 멈추면 오히려 토크가 풀려 차체가 주저앉는다.
범위를 벗어나면 경계로 되돌리는 목표가 걸리고, 경고만 찍힌다.

종료할 때
---------
기본값은 `--on-exit hold` 다. Ctrl-C 로 빠져나가도 토크를 끄지 않고 유지
전류를 건 채 포트만 닫는다. 서보는 마지막 Goal Position/Current 를 그대로
유지하므로 차체가 내려앉지 않는다. 정말로 힘을 빼려면 `--on-exit off`.
"""

from __future__ import annotations

import argparse
import sys
import time

from motor_control.xm430.link import open_motor
from motor_control.xm430.config import add_config_args, config_from_args, lsb_to_mnm, unwrap_zero_deg
from motor_control.xm430.suspension import DEG, RateLoop, SuspensionController
from motor_control.xm430.log import Recorder

STATE_MARK = {"HOLD": "·", "COMPLY": "↓", "RECOVER": "↑", "YIELD": "!",
              "CMD": "→"}


def main(argv):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_config_args(ap)
    ap.add_argument("--home-here", action="store_true",
                    help="시작 위치를 기준 자세로 삼는다 "
                         "(기본: 프로파일의 home_deg)")
    ap.add_argument("--limit-deg", type=float, default=-1.0,
                    help="가동 범위 ±[deg]. 0=제한 없음, 생략하면 프로파일 값")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="이 시간 뒤 종료 [s]. 0 = 무한 (기본)")
    ap.add_argument("--on-exit", choices=("hold", "off"), default="hold",
                    help="종료 시 토크 유지(hold, 기본) 또는 해제(off)")
    ap.add_argument("--no-log", action="store_true")
    ap.add_argument("--quiet", action="store_true", help="상태 줄을 찍지 않는다")
    args = ap.parse_args(argv[1:])
    args.base_overrides = list(args.overrides or [])

    cfg = config_from_args(args)
    if args.limit_deg >= 0.0:
        cfg.travel_limit_deg = float(args.limit_deg)
    print(cfg.summary())

    dxl, motor, info = open_motor(cfg, mock=args.mock, port=cfg.port,
                                  verbose=False)
    ctrl = SuspensionController(dxl, motor, cfg)
    fb = ctrl.attach(info)

    # [261016] 원점을 지금 읽힌 엔코더 회전으로 옮긴다 (experiments.Rig 와 같음)
    home = (fb.position_rad if args.home_here
            else unwrap_zero_deg(cfg.home_deg, fb.position_rad * DEG) / DEG)
    ctrl.set_travel_limit(home, cfg.travel_limit_rad)
    ctrl.set_home(home)

    rec = Recorder(cfg.log_dir, "run",
                   {"mode": "run", "config": cfg.to_dict(),
                    "home_deg": home * DEG, "mock": bool(args.mock)},
                   enabled=not args.no_log)

    # 터미널이 아니면 상태 줄이 로그 파일을 가득 채운다. 알아서 끈다.
    live = not args.quiet and sys.stdout.isatty()
    print(f"\n  기준 자세 {home * DEG:+.2f}°   포트 {motor.port_name}")
    print(f"  로그 {rec.path or '(없음)'}")
    print(f"  종료 시 {'토크 유지' if args.on_exit == 'hold' else '토크 해제'}"
          f"   {f'({args.duration:.0f} 초 뒤 자동 종료)' if args.duration > 0 else '(Ctrl-C 로 종료)'}")
    print()

    loop = RateLoop(cfg.control_hz)
    t0 = time.monotonic()
    t_end = t0 + args.duration if args.duration > 0 else None
    n_ev = [0]

    def on_event(ev):
        n_ev[0] += 1
        t = ev.t90_s
        sys.stdout.write(
            f"\r\x1b[2K  복원 #{n_ev[0]:<3d} "
            f"{'+CCW' if ev.direction > 0 else '-CW ':>4s} "
            f"변위 {ev.err0_deg:+6.2f}°  "
            f"t90 {'--' if t != t else f'{t * 1000:4.0f}ms'}  "
            f"OS {ev.overshoot_pct:4.1f}%  진동 {ev.zero_crossings}  "
            f"잔류 {ev.residual_deg:+5.2f}°  {ev.outcome}\n")
        sys.stdout.flush()

    ctrl.event_cb = on_event
    warned = 0.0
    next_draw = 0.0
    try:
        while True:
            now = loop.wait()
            fb = motor.read()
            ctrl.update(fb, now)
            rec.write(now - t0, ctrl.state, 0, ctrl.goal_deg,
                      fb.position_rad * DEG, fb.velocity_rad_s * DEG,
                      fb.current_a, ctrl.goal_current, ctrl.take_tag())

            over = ctrl.outside_travel(fb.position_rad)
            if over > 0 and now - warned > 5.0:
                warned = now
                sys.stdout.write(f"\r\x1b[2K  [경고] 가동 범위를 "
                                 f"{over * DEG:.1f}° 벗어났습니다. "
                                 f"지령은 경계로 잘려 있습니다.\n")
            if live and now >= next_draw:
                next_draw = now + 0.1          # 10 Hz. 100 Hz 로 다시 그릴 이유가 없다
                sys.stdout.write(
                    f"\r  {STATE_MARK.get(ctrl.state, '?')} {ctrl.state:<8s}"
                    f"pos {fb.position_rad * DEG:+7.2f}°  "
                    f"err {(ctrl.goal_rad - fb.position_rad) * DEG:+6.2f}°  "
                    f"{fb.current_a * 1000:+7.1f} mA "
                    f"(상한 {ctrl.goal_current:3d} LSB)  "
                    f"{loop.hz_actual:5.1f} Hz  복원 {n_ev[0]}   ")
                sys.stdout.flush()
            if t_end and now >= t_end:
                break
    except KeyboardInterrupt:
        pass
    finally:
        if live:
            print()
        ctrl.force_goal_current(cfg.current_holding)
        rec.update_meta(finished=time.strftime("%Y-%m-%d %H:%M:%S"),
                        loop_hz_actual=round(loop.hz_actual, 1),
                        loop_overruns=loop.overruns,
                        recoveries=n_ev[0])
        path = rec.close()
        if args.on_exit == "off":
            motor.shutdown()
            print("  토크를 해제했습니다. 차체가 내려앉을 수 있습니다.")
        else:
            # shutdown() 은 토크를 끈다. 여기서는 포트만 닫아 유지 전류를 남긴다.
            try:
                motor.port.closePort()
            except AttributeError:
                pass
            print(f"  토크 유지 상태로 종료 "
                  f"(유지 전류 {cfg.current_holding} LSB ≈ "
                  f"{lsb_to_mnm(cfg.current_holding):.0f} mN·m).")
        if loop.hz_actual < cfg.control_hz * 0.8:
            print(f"  [경고] 실제 루프 {loop.hz_actual:.0f} Hz "
                  f"< 설정 {cfg.control_hz:.0f} Hz. "
                  f"sudo sh setup_latency.sh 를 확인하세요.")
        print(f"  복원 {n_ev[0]} 회" + (f",  로그 {path}" if path else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
