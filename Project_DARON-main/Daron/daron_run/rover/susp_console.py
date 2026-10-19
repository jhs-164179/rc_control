#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
susp_console.py — 실시간 튜닝 콘솔.

터미널 한 개면 된다. pygame 도 ROS 도 필요 없으므로 RADXA 에 SSH 로
붙은 채 그대로 돌릴 수 있다. 파라미터를 키 하나로 바꾸고, 그 자리에서
축을 눌러 보고, 복원 지표가 바뀌는 걸 즉시 본다.

바꾸는 값은 전부 RAM 레지스터다 (Position PID, Profile, Goal Current).
Torque ON 상태에서 바로 먹는다. Current Limit(38) 만 EEPROM 이라
런타임에 못 바꾸므로, 실험 범위를 덮도록 처음부터 크게 잡아 둔다.

키
--
  ?        도움말          q      종료 (토크 OFF)
  h        현재 위치를 home 으로      0   goal 을 현재 위치로 리셋
  g / G    home -1° / +1°
  p / P    P gain  -10 / +10        d / D   D gain -50 / +50
  i / I    I gain   -1 / +1
  t / T    holding 전류 -1 / +1      c / C   comply 전류 -1 / +1
  r / R    recover 전류 -5 / +5
  v / V    Profile Velocity -20 / +20 deg/s
  a / A    Profile Accel   -100 / +100 deg/s^2
  b / B    방향 보정 x  -2 / +2      m   보정 모드 순환(off/bias/scale/both)
  f        속도 피드백(L3) on/off
  [ / ]    스텝 시험 변위 -1° / +1°
  s / S    스텝 시험: -방향 / +방향 변위 후 복원 (약 2 초 블로킹)
  e        지금까지의 복원 이벤트 표
  w        현재 설정을 profiles/console_<시각>.json 으로 저장
"""

from __future__ import annotations

import argparse
import math
import os
import select
import sys
import termios
import time
import tty

from experiments import Rig, displace_and_recover, summarize_events
from motor_control.xm430.config import (COMP_MODES, DIR_NEG, DIR_POS, add_config_args,
                         lsb_to_mnm)
from motor_control.xm430.suspension import DEG

STATUS_LINES = 11
REDRAW_HZ = 12.0


class RawKeys:
    """블로킹 없이 한 글자씩 읽는다. 터미널 설정은 반드시 원복한다."""

    def __init__(self):
        self.fd = sys.stdin.fileno()
        self.saved = None

    def __enter__(self):
        if sys.stdin.isatty():
            self.saved = termios.tcgetattr(self.fd)
            tty.setcbreak(self.fd)
        return self

    def __exit__(self, *exc):
        if self.saved is not None:
            termios.tcsetattr(self.fd, termios.TCSADRAIN, self.saved)
        return False

    def get(self):
        if not sys.stdin.isatty():
            return None
        if select.select([sys.stdin], [], [], 0)[0]:
            return sys.stdin.read(1)
        return None


class Screen:
    """같은 자리에 상태를 다시 그린다."""

    def __init__(self, lines: int):
        self.lines = lines
        self.drawn = False

    def draw(self, text: str):
        out = []
        if self.drawn:
            out.append(f"\x1b[{self.lines}A")
        for line in text.split("\n")[:self.lines]:
            out.append("\x1b[2K" + line + "\r\n")
        sys.stdout.write("".join(out))
        sys.stdout.flush()
        self.drawn = True

    def release(self):
        self.drawn = False


def bar(value, lo, hi, width=24):
    v = 0.0 if hi <= lo else (value - lo) / (hi - lo)
    v = min(max(v, 0.0), 1.0)
    n = int(round(v * width))
    return "[" + "#" * n + "·" * (width - n) + "]"


def render(rig, fb, amp, msg, last_ev):
    cfg, ctrl = rig.cfg, rig.ctrl
    err = (ctrl.goal_rad - fb.position_rad) * DEG
    pos = fb.position_rad * DEG
    state = ctrl.state
    mark = {"HOLD": "·", "COMPLY": "↓", "RECOVER": "↑", "YIELD": "!",
            "CMD": "→"}.get(state, "?")
    ev = ""
    if last_ev:
        t90 = last_ev.t90_s
        ev = (f"최근복원 {'+CCW' if last_ev.direction > 0 else '-CW '} "
              f"err0={last_ev.err0_deg:+6.2f}°  "
              f"t90={'--' if t90 != t90 else f'{t90 * 1000:.0f}ms'}  "
              f"OS={last_ev.overshoot_pct:.0f}%  진동={last_ev.zero_crossings}  "
              f"{last_ev.outcome}")
    return "\n".join([
        f"  {mark} {state:<8s} pos={pos:+8.2f}°  goal={ctrl.goal_deg:+8.2f}°  "
        f"err={err:+7.2f}°  vel={fb.velocity_rad_s * DEG:+7.1f}°/s",
        f"    전류 {fb.current_a * 1000:+7.1f} mA "
        f"(상한 {ctrl.goal_current:3d} LSB = {ctrl.goal_current * 2.69:5.0f} mA "
        f"≈ {lsb_to_mnm(ctrl.goal_current):5.0f} mN.m)  "
        f"{bar(abs(fb.current_a) * 1000, 0, max(ctrl.goal_current * 2.69, 1))}",
        f"    루프 {rig.loop.hz_actual:5.1f} Hz (설정 {cfg.control_hz:.0f})  "
        f"overrun={rig.loop.overruns}  이벤트={len(ctrl.events)}",
        "",
        f"    [t/T] holding {cfg.current_holding:3d}   "
        f"[c/C] comply {cfg.current_comply:3d}   "
        f"[r/R] recover {cfg.current_recover:3d}  (limit {cfg.current_limit})",
        f"    [p/P] P {cfg.p_gain:4d}   [i/I] I {cfg.i_gain:3d}   "
        f"[d/D] D {cfg.d_gain:4d}",
        f"    [v/V] vprof {cfg.profile_velocity_dps:5.0f}°/s   "
        f"[a/A] aprof {cfg.profile_accel_dpss:6.0f}°/s²",
        f"    [m] 보정 {cfg.dir_comp_mode:<5s} [b/B] x  "
        f"b=({cfg.b_pos:+.1f},{cfg.b_neg:+.1f}) k=({cfg.k_pos:.2f},{cfg.k_neg:.2f})"
        f"  ->  I+={cfg.recover_current(DIR_POS):3d} I-={cfg.recover_current(DIR_NEG):3d}",
        f"    [f] 속도피드백 {'ON ' if cfg.vel_fb_enable else 'OFF'}   "
        f"[s/S] 스텝시험 [{amp:.0f}°]   [h] home 재설정   [?] 도움말   [q] 종료",
        f"    {ev}",
        f"    {msg}",
    ])


def bump_x(cfg, delta):
    """방향 보정 파라미터를 반대 부호로 움직인다 (평균 세기는 유지)."""
    if cfg.dir_comp_mode in ("scale",):
        cfg.k_pos = round(cfg.k_pos + delta * 0.01, 3)
        cfg.k_neg = round(cfg.k_neg - delta * 0.01, 3)
    else:
        if cfg.dir_comp_mode == "off":
            cfg.dir_comp_mode = "bias"
        cfg.b_pos = round(cfg.b_pos + delta, 1)
        cfg.b_neg = round(cfg.b_neg - delta, 1)


def main(argv):
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_config_args(ap)
    ap.add_argument("--home-here", action="store_true", default=True,
                    help="시작 위치를 home 으로 (기본 켜짐)")
    ap.add_argument("--limit-deg", type=float, default=-1.0,
                    help="home 기준 가동 범위 ±[deg]. 0=제한 없음")
    ap.add_argument("--no-log", action="store_true")
    ap.add_argument("--amp", type=float, default=12.0, help="스텝 시험 변위 [deg]")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="이 시간 뒤 자동 종료 [s]. 0 = 무한 (기본)")
    args = ap.parse_args(argv[1:])
    args.base_overrides = list(args.overrides or [])

    rig = Rig(args, "console", {"mode": "interactive"})
    cfg, ctrl = rig.cfg, rig.ctrl
    amp = args.amp
    msg = "키를 누르세요. ? 는 도움말."
    last_ev = [None]
    ctrl.event_cb = lambda ev: last_ev.__setitem__(0, ev)
    screen = Screen(STATUS_LINES)
    next_draw = 0.0
    t_end = time.monotonic() + args.duration if args.duration > 0 else None
    if not sys.stdin.isatty():
        print("[console] stdin 이 터미널이 아닙니다. 키 입력 없이 "
              "기록만 합니다.")
    print("\n")

    try:
        with RawKeys() as keys:
            while True:
                now, fb = rig.sample()
                k = keys.get()
                if k:
                    if k == "q":
                        break
                    res = handle_key(rig, k, amp)
                    if isinstance(res, tuple):
                        msg, amp = res
                    elif res:
                        msg = res
                if t_end and now >= t_end:
                    break
                if now >= next_draw:
                    next_draw = now + 1.0 / REDRAW_HZ
                    screen.draw(render(rig, fb, amp, msg, last_ev[0]))
    except KeyboardInterrupt:
        pass
    finally:
        screen.release()
        print()
        if ctrl.events:
            summarize_events(ctrl.events, "(콘솔 세션 전체)")
        rig.close("console session")
    return 0


def handle_key(rig, k, amp):
    """키 하나를 처리한다. 문자열(메시지) 또는 (메시지, 새 amp) 를 돌려준다."""
    cfg, ctrl = rig.cfg, rig.ctrl

    if k == "?":
        return "도움말은 파일 상단 docstring 참고 (python3 susp_console.py -h)"
    if k == "h":
        fb = rig.motor.read()
        rig.home = fb.position_rad
        ctrl.set_home(rig.home)
        return f"home = {rig.home * DEG:+.2f}°"
    if k == "0":
        fb = rig.motor.read()
        ctrl.set_home(fb.position_rad)
        return "goal 을 현재 위치로 리셋"
    if k in "gG":
        d = math.radians(1.0 if k == "G" else -1.0)
        ctrl.set_home(ctrl.goal_rad + d)
        return f"goal = {ctrl.goal_deg:+.2f}°"

    if k in "pP":
        ctrl.set_gains(p=max(0, cfg.p_gain + (10 if k == "P" else -10)))
        return f"P = {cfg.p_gain}"
    if k in "iI":
        ctrl.set_gains(i=max(0, cfg.i_gain + (1 if k == "I" else -1)))
        return f"I = {cfg.i_gain}"
    if k in "dD":
        ctrl.set_gains(d=max(0, cfg.d_gain + (50 if k == "D" else -50)))
        return f"D = {cfg.d_gain}"

    if k in "tT":
        cfg.current_holding = int(max(1, min(cfg.current_limit,
                                             cfg.current_holding + (1 if k == "T" else -1))))
        ctrl.invalidate_current_cache()
        return f"holding = {cfg.current_holding} LSB ({lsb_to_mnm(cfg.current_holding):.0f} mN.m)"
    if k in "cC":
        cfg.current_comply = int(max(1, min(cfg.current_limit,
                                            cfg.current_comply + (1 if k == "C" else -1))))
        ctrl.invalidate_current_cache()
        return f"comply = {cfg.current_comply} LSB"
    if k in "rR":
        cfg.current_recover = int(max(1, min(cfg.current_limit,
                                             cfg.current_recover + (5 if k == "R" else -5))))
        ctrl.invalidate_current_cache()
        return f"recover = {cfg.current_recover} LSB ({lsb_to_mnm(cfg.current_recover):.0f} mN.m)"

    if k in "vV":
        ctrl.set_profile(velocity_dps=max(1.0, cfg.profile_velocity_dps
                                          + (20 if k == "V" else -20)))
        return f"Profile Velocity = {cfg.profile_velocity_dps:.0f}°/s"
    if k in "aA":
        ctrl.set_profile(accel_dpss=max(1.0, cfg.profile_accel_dpss
                                        + (100 if k == "A" else -100)))
        return f"Profile Accel = {cfg.profile_accel_dpss:.0f}°/s²"

    if k in "bB":
        bump_x(cfg, 2.0 if k == "B" else -2.0)
        ctrl.invalidate_current_cache()
        return (f"보정 I+={cfg.recover_current(DIR_POS)} "
                f"I-={cfg.recover_current(DIR_NEG)} LSB")
    if k == "m":
        cfg.dir_comp_mode = COMP_MODES[(COMP_MODES.index(cfg.dir_comp_mode) + 1)
                                       % len(COMP_MODES)]
        ctrl.invalidate_current_cache()
        return f"보정 모드 = {cfg.dir_comp_mode}"
    if k == "f":
        cfg.vel_fb_enable = not cfg.vel_fb_enable
        return f"속도 피드백 = {'ON' if cfg.vel_fb_enable else 'OFF'}"

    if k == "[":
        return (f"스텝 변위 = {amp - 1:.0f}°", max(1.0, amp - 1))
    if k == "]":
        return (f"스텝 변위 = {amp + 1:.0f}°", amp + 1)
    if k in "sS":
        d = amp if k == "s" else -amp      # 변위 방향의 반대가 복원 방향
        ev = displace_and_recover(rig, math.radians(d), 0.4, 4.0)
        if ev is None:
            return "스텝 시험: 복원 이벤트를 못 잡았습니다"
        t = ev.t90_s
        return (f"스텝 {'+CCW' if ev.direction > 0 else '-CW'} 복원 "
                f"t90={'--' if t != t else f'{t * 1000:.0f}ms'} "
                f"OS={ev.overshoot_pct:.0f}%")

    if k == "e":
        return f"이벤트 {len(ctrl.events)} 개 — 종료 시 요약이 출력됩니다"
    if k == "w":
        here = os.path.dirname(os.path.abspath(__file__))
        p = os.path.join(here, "profiles",
                         f"console_{time.strftime('%Y%m%d_%H%M%S')}.json")
        cfg.save(p)
        return f"저장: {os.path.basename(p)}"
    return None


if __name__ == "__main__":
    sys.exit(main(sys.argv))
