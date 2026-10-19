#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
drive_keys.py — 구동모터(VESC)만 키보드로 시험한다.

ROS, 서보 없이 RADXA 터미널(SSH 가능)에서 vesc_driver 만 쓴다.
radxa.py 가 떠 있으면 UART 를 빼앗기므로 먼저 끈다. 바퀴를 띄우고 시험한다.

조작
----
  W / S     전진 / 후진   누르고 있는 동안 설정 가속도로 max_speed 까지
            키를 떼면 (HOLD_S 동안 입력 없음) 설정 감속도로 정지
  SPACE     감속 정지
  Q / ESC   종료 (출력 OFF)

실행
----
  python3 drive_keys.py               # front + back 동시
  python3 drive_keys.py --role back   # 한쪽만
  python3 drive_keys.py --mock        # 하드웨어 없이 동작 확인
"""
import argparse
import os
import select
import sys
import termios
import time
import tty
from dataclasses import replace

from motor_control.vesc.xerun import ROLES, _ready, _release, load_role_config, make_driver, pair_errors

HOLD_S = 0.6  # 터미널 키 반복 시작 지연(약 0.5 s)보다 길어야 누르는 동안 끊기지 않는다


def _wait(motors, cond, what, timeout=5.0):
    deadline = time.monotonic() + timeout
    while True:
        states = {r: m.snapshot() for r, m in motors.items()}
        if all(cond(s) for s in states.values()):
            return
        if time.monotonic() >= deadline:
            bad = {r: {"io_error": s.get("io_error"), "control": s.get("control")}
                   for r, s in states.items() if not cond(s)}
            raise RuntimeError(f"{what}: {bad}")
        for m in motors.values():
            m.heartbeat()
        time.sleep(0.02)


def _fmt(x, spec="+7.1f"):
    return "    n/a" if x is None else format(x, spec)


def main():
    ap = argparse.ArgumentParser(description="keyboard drive-motor test (no ROS)")
    ap.add_argument("--role", choices=tuple(ROLES), help="한쪽만 (기본: 전부)")
    ap.add_argument("--mock", action="store_true", help="하드웨어 없이 mock VESC")
    args = ap.parse_args()

    roles = (args.role,) if args.role else tuple(ROLES)
    configs = {r: load_role_config(r, require_motion_ready=False) for r in roles}
    if args.mock:
        configs = {r: replace(c, mock=True) for r, c in configs.items()}
    problems = pair_errors(configs, motion=True)
    if problems:
        sys.exit("; ".join(problems))

    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    motors, result = {}, 0
    try:
        for r, c in configs.items():
            motors[r] = make_driver(r, c)
            motors[r].start()
        _wait(motors, _ready, "no fresh VESC response")
        for m in motors.values():
            m.command("on")
        _wait(motors, lambda s: s.get("control", {}).get("enabled"), "VESC did not acknowledge ON")

        print(__doc__.split("실행")[0].split("조작")[1].strip("-\n"))
        tty.setcbreak(fd)
        direction, last_key, next_print = 0, 0.0, 0.0
        while True:
            now = time.monotonic()
            quit_ = False
            if select.select([fd], [], [], 0.02)[0]:
                for key in os.read(fd, 32).lower():
                    if key in (ord("q"), 0x1B):
                        quit_ = True
                    elif key == ord("w"):
                        direction, last_key = 1, now
                    elif key == ord("s"):
                        direction, last_key = -1, now
                    elif key == ord(" "):
                        direction = 0
            if quit_:
                break
            if direction and now - last_key > HOLD_S:
                direction = 0

            states = {r: m.snapshot() for r, m in motors.items()}
            for r, s in states.items():
                ctl = s.get("control", {})
                if not _ready(s) or not ctl.get("enabled") or ctl.get("fault") or ctl.get("last_error"):
                    raise RuntimeError(f"{r} stopped: io_error={s.get('io_error')!r} control={ctl}")
            for m in motors.values():
                m.set_direction(direction)

            if now >= next_print:
                parts = []
                for r, s in states.items():
                    raw = s.get("raw") or {}
                    d = (raw.get("derived") or {}).get("values") or {}
                    v = raw.get("values") or {}
                    parts.append(f"{r}: rpm {_fmt(d.get('motor_rpm_forward'))}"
                                 f"  I {_fmt(v.get('motor_current_a'), '+5.2f')} A")
                sys.stdout.write(f"\rdir {direction:+d}   " + "   ".join(parts) + "   ")
                sys.stdout.flush()
                next_print = now + 0.1
    except KeyboardInterrupt:
        result = 130
    except Exception as exc:
        print(f"\nERROR: {type(exc).__name__}: {exc}")
        result = 1
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)
        if _release(motors, print):
            result = 1
        print()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
