#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
experiments.py — 서스펜션 파라미터 실험.

  sag      유지 전류를 낮춰가며 정적 처짐을 재고, current_holding 의 하한을 찾는다
  step     모터가 스스로 초기 변위를 만든 뒤 복원시킨다 (재현 가능한 복원 시험)
  release  사람이 손으로 눌렀다 놓는다. 릴리스를 자동 검출해 지표를 낸다
  sweep    파라미터 격자를 돌며 step 을 반복하고 순위표를 만든다

공통 옵션
  --profile soft|medium|firm|<path>   프리셋
  --set name=value                    개별 덮어쓰기 (여러 번)
  --mock                              하드웨어 없이 시뮬레이션
  --limit-deg D                       home 기준 ±D 를 벗어나면 즉시 중단

예)
  python3 experiments.py sag --from 60 --to 5 --step 5
  python3 experiments.py step --amp 12 --reps 5
  python3 experiments.py release --duration 60
  python3 experiments.py sweep --grid p_gain=80,120,160 --grid d_gain=400,600,800
"""

from __future__ import annotations

import argparse
import itertools
import math
import sys
import time

from motor_control.xm430.metrics import MIN_SPAN_DEG, _median
from motor_control.xm430.link import open_motor
from motor_control.xm430.config import CURRENT_LSB_MA, add_config_args, config_from_args, lsb_to_mnm, unwrap_zero_deg
from motor_control.xm430.suspension import DEG, HOLD, RateLoop, SuspensionController
from motor_control.xm430.log import Recorder


# ======================================================================
# 공통 실행기
# ======================================================================
class Rig:
    """모터 + 컨트롤러 + 로거 + 안전 한계를 한 덩어리로 묶는다."""

    def __init__(self, args, prefix: str, meta_extra: dict = None):
        self.cfg = config_from_args(args)
        self.args = args
        # --limit-deg 를 주면 프로파일 값을 덮어쓴다. 음수면 프로파일을 따른다.
        if getattr(args, "limit_deg", -1.0) >= 0.0:
            self.cfg.travel_limit_deg = float(args.limit_deg)
        print(self.cfg.summary())
        self.dxl, self.motor, self.info = open_motor(
            self.cfg, mock=args.mock, port=self.cfg.port)
        self.ctrl = SuspensionController(self.dxl, self.motor, self.cfg)
        fb = self.ctrl.attach(self.info)
        if args.home_here:
            self.home = fb.position_rad
        else:
            # [261016] XM430 은 전원 투입 때 엔코더를 0 ~ 360 안으로 다시 잡고, 모드 5 는 그 뒤로
            # 여러 바퀴를 센다. 원점을 지금 읽힌 회전으로 옮기지 않으면 home 이 360 deg 떨어져
            # 한 바퀴를 돌려 한다 (radxa.py 와 같은 처리)
            home_deg = unwrap_zero_deg(self.cfg.home_deg, fb.position_rad * DEG)
            if abs(home_deg - self.cfg.home_deg) > 1e-6:
                print(f"  원점 {self.cfg.home_deg:+.2f}° -> {home_deg:+.2f}° "
                      f"(엔코더 {fb.position_rad * DEG:+.2f}° 와 같은 회전으로 옮김, 이번 실행만)")
            self.home = math.radians(home_deg)
        # 가동 범위는 '지령을 자르는' 장치다. 벗어났다고 실행을 멈추지 않는다.
        self.ctrl.set_travel_limit(self.home, self.cfg.travel_limit_rad)
        self.ctrl.set_home(self.home)
        self._warned_outside = 0.0
        meta = {"experiment": prefix, "config": self.cfg.to_dict(),
                "home_deg": self.home * DEG, "mock": bool(args.mock)}
        meta.update(meta_extra or {})
        self.rec = Recorder(self.cfg.log_dir, prefix, meta,
                            enabled=not args.no_log)
        self.loop = RateLoop(self.cfg.control_hz)
        self.t0 = time.monotonic()
        print(f"  home = {self.home * DEG:+.2f}°   "
              f"로그 = {self.rec.path or '(없음)'}")

    def sample(self, event: str = "", run_state_machine: bool = True):
        """한 주기: 대기 -> 읽기 -> (상태기계) -> 기록."""
        now = self.loop.wait()
        fb = self.motor.read()
        if run_state_machine:
            self.ctrl.update(fb, now)
        # 외력에 밀려 범위를 넘는 것은 서스펜션의 정상 동작이다. 알리기만 한다.
        over = self.ctrl.outside_travel(fb.position_rad)
        if over > 0 and now - self._warned_outside > 3.0:
            self._warned_outside = now
            print(f"\n[경고] 가동 범위를 {over * DEG:.1f}° 벗어났습니다 "
                  f"(현재 {(fb.position_rad - self.home) * DEG:+.1f}°). "
                  f"지령은 경계로 잘려 있습니다.")
        tag = "|".join(x for x in (self.ctrl.take_tag(), event) if x)
        self.rec.write(now - self.t0, self.ctrl.state, 0, self.ctrl.goal_deg,
                       fb.position_rad * DEG, fb.velocity_rad_s * DEG,
                       fb.current_a, self.ctrl.goal_current, tag)
        return now, fb

    def run_for(self, seconds: float, event: str = "", **kw):
        end = time.monotonic() + seconds
        last = None
        while time.monotonic() < end:
            last = self.sample(event, **kw)
            event = ""
        return last

    def close(self, note: str = ""):
        try:
            self.ctrl.force_goal_current(self.cfg.current_holding)
        except Exception:
            pass
        self.rec.update_meta(finished=time.strftime("%Y-%m-%d %H:%M:%S"),
                             loop_hz_actual=round(self.loop.hz_actual, 1),
                             loop_overruns=self.loop.overruns, note=note)
        p = self.rec.close()
        self.motor.shutdown()
        if self.loop.hz_actual < self.cfg.control_hz * 0.8:
            print(f"\n[경고] 실제 루프 {self.loop.hz_actual:.0f} Hz "
                  f"< 설정 {self.cfg.control_hz:.0f} Hz. "
                  f"FTDI latency_timer 를 확인하세요 (sudo sh setup_latency.sh).")
        if p:
            print(f"\n로그: {p}")
        return p


def apply_live(ctrl, cfg):
    """cfg 의 게인/프로파일/전류를 실제 레지스터에 반영한다."""
    ctrl.set_gains(cfg.p_gain, cfg.i_gain, cfg.d_gain)
    ctrl.set_profile(cfg.profile_velocity_dps, cfg.profile_accel_dpss)
    ctrl.invalidate_current_cache()


# ======================================================================
# sag — 유지 전류 하한 찾기
# ======================================================================
# 유지 전류 = 하한 x HOLD_MARGIN. 261015: 1.5 -> 1.8 (1.5 는 자세 유지가 겨우 되는 정도라 약간 더 단단하게)
HOLD_MARGIN = 1.8


def round_up(v, unit=5):
    return int(math.ceil(float(v) / unit - 1e-9)) * unit


def measure_load(rig, current_max=None, verbose=True, p_max=None):
    """
    [261016] home 에서 버티는 데 실제로 드는 전류(= 중력 부하) [LSB] 를 잰다. 못 재면 None.

    current_max 를 주면 자동 조정: home 에서 처져 있을 때
      - 전류가 포화(moving 전류를 다 씀)  -> current_moving 을 1.5 배씩 current_max 까지
      - 전류가 남는데 처짐 (P 게인 부족) -> p_gain 을 1.5 배씩 p_max 까지
    바뀐 값은 rig.cfg 에 들어가 프로파일로 저장된다.
    """
    cfg, ctrl = rig.cfg, rig.ctrl
    tol_deg = max(2.0, 3.0 * cfg.settled_band_deg)
    while True:
        ctrl.force_goal_current(cfg.current_moving)
        reached = ctrl.goto_blocking(rig.home, timeout=5.0)
        ctrl.force_goal_current(cfg.current_moving)   # HOLD 에 들어가면 holding 으로 내려가므로 다시
        rig.run_for(0.5, run_state_machine=False)
        cur, pos = [], []
        t_end = time.monotonic() + 0.5
        while time.monotonic() < t_end:
            _, fb = rig.sample(run_state_machine=False)
            cur.append(abs(fb.current_a) * 1000.0 / CURRENT_LSB_MA)
            pos.append(fb.position_rad)
        load, err = _median(cur), (_median(pos) - rig.home) * DEG
        saturated = load >= 0.9 * cfg.current_moving
        if verbose:
            print(f"  moving {cfg.current_moving} LSB, P {cfg.p_gain}: home 오차 {err:+.1f}°, "
                  f"버티는 전류 ≈ {load:.0f} LSB ({load * CURRENT_LSB_MA:.0f} mA, "
                  f"{lsb_to_mnm(load):.0f} mN.m){'  [포화]' if saturated else ''}")
        if reached and abs(err) <= tol_deg and not saturated:
            return load
        if current_max is None:
            print("  [실패] 하중을 못 버팁니다 (자동 전류 조정 없음). current_moving / p_gain 을 올려 다시 하세요.")
            return None
        if saturated:
            if cfg.current_moving >= current_max:
                print(f"  [실패] current_moving {cfg.current_moving} LSB(상한)로도 하중을 못 이깁니다. "
                      f"--current-max 를 올리세요 (XM430 최대 1193).")
                return None
            new = min(current_max, round_up(cfg.current_moving * 1.5, 10))
            print(f"  -> 전류 포화: current_moving {cfg.current_moving} -> {new} LSB")
            cfg.current_moving = new
            ctrl.invalidate_current_cache()
        else:
            if p_max is None or cfg.p_gain >= p_max:
                print(f"  주의: 전류가 남는데 {err:+.1f}° 처져 있습니다 (P {cfg.p_gain}). 이 상태로 계속합니다.")
                return load
            new = min(p_max, round_up(cfg.p_gain * 1.5, 10))
            print(f"  -> P 게인 부족(전류가 남는데 처짐): p_gain {cfg.p_gain} -> {new}")
            ctrl.set_gains(p=new)


def sag_sweep(rig, start, end, step, dwell, max_droop, verbose=True):
    """
    유지 전류를 내려가며 정적 처짐을 잰다. [(I_LSB, 처짐_deg, 최대속도_dps), ...]

    상태기계는 돌리지 않는다. 돌리면 처짐을 외란으로 보고 복원해 버려서
    재려는 값 자체가 사라진다.
    """
    ctrl = rig.ctrl
    results = []
    ctrl.force_goal_current(rig.cfg.current_moving)
    # [261016] 무거워지면 moving 전류로 home 까지 못 올라갈 수 있다. 그러면 '처짐'의 기준이
    # 틀려서 결과가 조용히 엉뚱해지므로 여기서 멈춘다
    if not ctrl.goto_blocking(rig.home, timeout=5.0):
        pos = rig.motor.read().position_rad
        print(f"  [중단] current_moving({rig.cfg.current_moving} LSB) 로 home 에 못 갔습니다 "
              f"(오차 {(pos - rig.home) * DEG:+.1f}°). 하중을 못 이기는 것입니다. "
              f"--set current_moving=... 을 올리세요 (current_limit {rig.cfg.current_limit} 이하).")
        return results
    if verbose:
        print(f"{'I[LSB]':>7} {'[mA]':>7} {'[mN.m]':>8} "
              f"{'처짐[deg]':>10} {'속도[°/s]':>10}")
    for raw in range(int(start), int(end) - 1, -int(step)):
        ctrl.force_goal_current(raw)
        rig.run_for(dwell * 0.5, f"SAG_SET:{raw}", run_state_machine=False)
        xs, vs = [], []
        t_end = time.monotonic() + dwell * 0.5
        while time.monotonic() < t_end:
            _, fb = rig.sample(run_state_machine=False)
            xs.append(fb.position_rad)
            vs.append(abs(fb.velocity_rad_s))
        droop = (sum(xs) / len(xs) - rig.home) * DEG
        vmax = max(vs) * DEG
        results.append((raw, droop, vmax))
        if verbose:
            print(f"{raw:7d} {raw * 2.69:7.1f} {lsb_to_mnm(raw):8.1f} "
                  f"{droop:10.3f} {vmax:10.2f}")
        if abs(droop) > max_droop:
            if verbose:
                print(f"  -> 처짐이 {max_droop}° 를 넘었습니다. 중단합니다.")
            break
    ctrl.force_goal_current(rig.cfg.current_moving)
    ctrl.goto_blocking(rig.home, timeout=5.0)
    return results


def sag_interpret(results, droop_ok, verbose=True, margin=HOLD_MARGIN):
    """
    붕괴점을 찾는다.

    낮은 유지 전류에서 늘 조금은 처진다(P 게인이 만드는 정상 처짐). 찾고 싶은
    것은 그 값이 아니라 '갑자기 무너지는 지점'이다. 높은 전류 쪽 절반의 처짐을
    기준선으로 삼고, 거기서 크게 벗어나는 첫 전류를 붕괴점으로 본다.

    유지 전류 = 붕괴점 x margin. 시험 범위 안에서 무너지지 않았으면 시험한 가장
    낮은 전류 x margin (전에는 여유 없이 그 값 그대로).
    """
    if not results:
        return None
    if len(results) < 3:
        # [261016] 시작하자마자 max_droop 를 넘었다 = 시작 전류부터 이미 하중을 못 버팀.
        # 예전에는 이 한두 점을 '정상 처짐'으로 보고 '무너지지 않았다'고 잘못 판정했다
        if verbose:
            print(f"\n  {results[0][0]} LSB 에서 이미 무너졌습니다 (처짐 {results[0][1]:+.1f}°). "
                  f"--sag-start 를 올려 다시 재세요 (위의 '버티는 전류' x 1.5 이상).")
        return None
    stable = _median([abs(r[1]) for r in results[:max(1, len(results) // 2)]])
    thresh = max(2.0 * stable + 0.3, droop_ok)
    collapse = None
    for raw, droop, _ in results:
        if abs(droop) > thresh:
            collapse = raw
            break
    out = {"stable_droop_deg": stable, "threshold_deg": thresh,
           "collapse_lsb": collapse}
    if collapse is None:
        out["holding"] = int(math.ceil(results[-1][0] * margin))
        out["margin"] = False
    else:
        out["holding"] = int(math.ceil(collapse * margin))
        out["margin"] = True
    out["comply"] = max(1, int(out["holding"] * 0.7))
    # 정상 처짐보다 판정 밴드가 좁으면 복원이 영원히 '완료'로 안 잡힌다.
    out["settled_band_deg"] = round(max(0.7, stable * 1.5), 2)
    # comply 밴드는 settled 보다 넓어야 한다. 좁으면 '외란 판정'이 '복원 완료
    # 판정'보다 먼저 걸려 상태가 채터링한다.
    out["comply_band_deg"] = round(max(2.0, out["settled_band_deg"] * 2.0), 2)
    if verbose:
        print(f"\n  정상 처짐(높은 전류 쪽 기준선) = {stable:.2f}°  "
              f"-> 붕괴 판정선 {thresh:.2f}°")
        if collapse is None:
            print(f"  시험한 {results[-1][0]} LSB 까지 무너지지 않았습니다. "
                  f"--end 를 더 낮춰 다시 재세요. (지금 권장 = {out['holding']} LSB, "
                  f"그 값의 {margin:g} 배)")
        else:
            print(f"  붕괴 시작 = {collapse} LSB "
                  f"({collapse * 2.69:.0f} mA, {lsb_to_mnm(collapse):.0f} mN.m)")
            print(f"  권장 유지 전류 = {out['holding']} LSB "
                  f"(붕괴점의 {margin:g} 배 여유)")
        if stable > droop_ok:
            print(f"\n  주의: 정상 처짐 {stable:.2f}° 가 허용치 {droop_ok}° 보다 "
                  f"큽니다.\n"
                  f"        이건 전류가 아니라 P 게인이 만드는 처짐입니다. "
                  f"P 를 올리거나 I 를 조금 넣으세요.\n"
                  f"        settled_band_deg 는 "
                  f"{out['settled_band_deg']}° 이상이어야 복원이 "
                  f"'완료'로 판정됩니다.")
    return out


def cmd_sag(args):
    rig = Rig(args, "sag", {"sweep": "current_holding"})
    print("\n차체 하중을 실은 상태여야 의미가 있습니다. 축을 놓고 시작하세요.")
    results = []
    try:
        load = measure_load(rig)
        if load is not None and args.start < 1.3 * load:
            print(f"  주의: --start {args.start} 가 부하에 비해 낮습니다. "
                  f"--start {round_up(1.5 * load, 10)} 정도로 올리세요.")
        results = sag_sweep(rig, args.start, args.end, args.step,
                            args.dwell, args.max_droop)
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C")
    finally:
        rig.close("sag sweep")
    out = sag_interpret(results, args.droop_ok, margin=args.hold_margin)
    if out is None:
        return 1
    print(f"\n  다음 실험부터:  --set current_holding={out['holding']} "
          f"--set current_comply={out['comply']} "
          f"--set settled_band_deg={out['settled_band_deg']} "
          f"--set comply_band_deg={out['comply_band_deg']}")
    return 0


# ======================================================================
# step — 재현 가능한 복원 시험
# ======================================================================
def displace_and_recover(rig, amp_rad: float, settle_s: float,
                         timeout: float) -> "RecoveryEvent":
    """
    1) moving 전류로 home+amp 까지 확실히 밀어 초기 변위를 만든다
    2) 복원 전류로 바꾼 뒤 Goal Position 을 home 으로 되돌린다
    3) HOLD/YIELD 가 될 때까지 돌린다
    amp 의 부호가 '초기 변위 방향'이고, 복원 방향은 그 반대다.
    """
    ctrl = rig.ctrl
    ctrl.command(rig.home + amp_rad)
    end = time.monotonic() + timeout
    while ctrl.state != HOLD and time.monotonic() < end:
        rig.sample("")
    rig.run_for(settle_s, "PRE_SETTLED")

    n0 = len(ctrl.events)
    ctrl.arm_recovery(rig.home)
    end = time.monotonic() + timeout
    while len(ctrl.events) == n0 and time.monotonic() < end:
        rig.sample("")
    rig.run_for(0.2, "POST")
    return ctrl.events[-1] if len(ctrl.events) > n0 else None


def run_step_series(rig, amps, reps, settle_s, timeout, quiet=False):
    """amps 의 각 변위를 reps 회씩. 완료된 RecoveryEvent 리스트를 돌려준다."""
    evs = []
    if not quiet:
        print("\n" + "  " + _EVHDR)
    for i in range(reps):
        for amp in amps:
            ev = displace_and_recover(rig, math.radians(amp), settle_s, timeout)
            if ev is None:
                print(f"  [skip] amp={amp:+.1f}° 복원 이벤트가 잡히지 않았습니다.")
                continue
            evs.append(ev)
            if not quiet:
                print("  " + ev.line())
    return evs


_EVHDR = ("dir  err0[deg]  I[LSB]  t90[ms]  정착[ms]  "
          "vpk[deg/s]  OS[%]  진동  잔류[°]  Ipk[mA]  결과")


def summarize_events(evs, label=""):
    # 복원이 실질적으로 일어나지 않은 이벤트(정적 처짐을 조금 되돌리려다 만
    # 것 등)는 통계에서 뺀다. 넣으면 t90 중앙값이 오염된다.
    n_all = len(evs)
    evs = [e for e in evs
           if abs(e.metrics.get("span_deg", 0.0) or 0.0) >= MIN_SPAN_DEG]
    pos = [e for e in evs if e.direction > 0]
    neg = [e for e in evs if e.direction < 0]
    def med(g, k):
        return _median([getattr(e, k) for e in g])
    print(f"\n── 중앙값 {label} ──")
    if n_all > len(evs):
        print(f"   (복원량 {MIN_SPAN_DEG}° 미만 {n_all - len(evs)} 개 제외)")
    print(f"{'방향':>6} {'N':>3} {'t90[ms]':>9} {'정착[ms]':>9} "
          f"{'OS[%]':>7} {'진동':>5} {'잔류[°]':>8} {'Ipk[mA]':>9}")
    f = lambda v: "       --" if v != v else f"{v * 1000:9.0f}"
    for g, name in ((pos, "+CCW"), (neg, "-CW")):
        if not g:
            continue
        print(f"{name:>6} {len(g):>3} {f(med(g, 't90_s'))} {f(med(g, 't_settle_s'))} "
              f"{med(g, 'overshoot_pct'):7.1f} "
              f"{med(g, 'zero_crossings'):5.1f} {med(g, 'residual_deg'):8.2f} "
              f"{med(g, 'peak_current_mA'):9.0f}")
    if pos and neg:
        a, b = med(pos, "t90_s"), med(neg, "t90_s")
        if a == a and b == b and b > 0:
            print(f"  비대칭 지수 R = t90(+)/t90(-) = {a / b:.3f}")
            return a / b
    return float("nan")


def cmd_step(args):
    rig = Rig(args, "step", {"amp_deg": args.amp, "reps": args.reps})
    amps = {"both": [args.amp, -args.amp],
            "pos": [-abs(args.amp)],      # +방향 복원을 보려면 -쪽으로 변위
            "neg": [abs(args.amp)]}[args.dirs]
    try:
        rig.ctrl.goto_blocking(rig.home, timeout=5.0)
        evs = run_step_series(rig, amps, args.reps, args.settle, args.timeout)
        summarize_events(evs)
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C")
    finally:
        rig.close(f"step amp={args.amp} reps={args.reps}")
    return 0


# ======================================================================
# release — 사람이 손으로 미는 실제 외란
# ======================================================================
def cmd_release(args):
    rig = Rig(args, "release", {"mode": "manual"})
    print("\n축을 손으로 눌렀다가 놓으세요. 릴리스를 자동으로 검출합니다.")
    print(f"{args.duration:.0f} 초 동안 기록합니다. Ctrl-C 로 조기 종료.\n")
    print("  " + _EVHDR)
    rig.ctrl.event_cb = lambda ev: print("  " + ev.line())
    try:
        rig.run_for(args.duration)
    except KeyboardInterrupt:
        print("\n[중단] Ctrl-C")
    finally:
        evs = rig.ctrl.events
        if evs:
            summarize_events(evs, "(실제 외란)")
        else:
            print("\n복원 이벤트가 없습니다. comply_band_deg 를 낮추거나 "
                  "더 크게 눌러 보세요.")
        rig.close("manual release")
    return 0


# ======================================================================
# sweep — 파라미터 격자
# ======================================================================
def parse_grid(items):
    grid = {}
    for it in items or []:
        name, _, vals = it.partition("=")
        if not vals:
            raise ValueError(f"--grid 는 name=v1,v2,.. 형식입니다: {it!r}")
        grid[name.strip()] = [v.strip() for v in vals.split(",") if v.strip()]
    return grid


def cmd_sweep(args):
    grid = parse_grid(args.grid)
    if not grid:
        print("--grid 가 필요합니다. 예: --grid p_gain=80,120,160")
        return 1
    names = list(grid)
    combos = list(itertools.product(*(grid[n] for n in names)))
    print(f"조합 {len(combos)} 개 × 변위 {2 if args.dirs == 'both' else 1} 방향 "
          f"× {args.reps} 회")

    rows = []
    base_args = args
    for ci, combo in enumerate(combos, 1):
        over = [f"{n}={v}" for n, v in zip(names, combo)]
        base_args.overrides = (args.base_overrides or []) + over
        label = " ".join(over)
        print(f"\n════ [{ci}/{len(combos)}] {label} ════")
        rig = Rig(base_args, "sweep", {"combo": dict(zip(names, combo))})
        try:
            apply_live(rig.ctrl, rig.cfg)
            rig.ctrl.goto_blocking(rig.home, timeout=5.0)
            amps = {"both": [args.amp, -args.amp],
                    "pos": [-abs(args.amp)],
                    "neg": [abs(args.amp)]}[args.dirs]
            evs = run_step_series(rig, amps, args.reps, args.settle,
                                  args.timeout, quiet=True)
            R = summarize_events(evs, label)
            rows.append(dict(
                label=label,
                t90=_median([e.t90_s for e in evs]) * 1000.0,
                ts=_median([e.t_settle_s for e in evs]) * 1000.0,
                os=_median([e.overshoot_pct for e in evs]),
                osc=_median([float(e.zero_crossings) for e in evs]),
                R=R, path=rig.rec.path))
        except KeyboardInterrupt:
            rig.close(label)
            print("\n[중단] Ctrl-C")
            break
        finally:
            try:
                rig.close(label)
            except Exception:
                pass

    if not rows:
        return 1
    print("\n══════ 순위 (t90 오름차순, 진동 " f"{args.max_osc} 이하만) ══════")
    print(f"{'조합':<40} {'t90[ms]':>8} {'정착[ms]':>9} {'OS[%]':>6} "
          f"{'진동':>5} {'R':>6}")
    ok = [r for r in rows if r["osc"] <= args.max_osc
          and r["os"] <= args.max_overshoot]
    for r in sorted(ok, key=lambda r: r["t90"] if r["t90"] == r["t90"] else 1e9):
        Rs = "   --" if r["R"] != r["R"] else f"{r['R']:6.3f}"
        print(f"{r['label']:<40} {r['t90']:>8.0f} {r['ts']:>9.0f} "
              f"{r['os']:>6.1f} {r['osc']:>5.1f} {Rs}")
    dropped = [r for r in rows if r not in ok]
    if dropped:
        print(f"\n(진동/오버슈트 기준 초과로 제외: {len(dropped)} 개)")
        for r in dropped:
            print(f"  {r['label']:<40} OS={r['os']:.1f}%  진동={r['osc']:.1f}")
    return 0


# ======================================================================
def build_parser():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)

    def common(p):
        add_config_args(p)
        p.add_argument("--home-here", action="store_true",
                       help="시작 위치를 home 으로 삼는다 (기본: 설정의 home_deg)")
        p.add_argument("--limit-deg", type=float, default=-1.0,
                       help="home 기준 가동 범위 ±[deg]. 이 밖은 지령하지 않는다 "
                            "(중단이 아니라 지령을 자른다). 0=제한 없음, "
                            "생략하면 프로파일의 travel_limit_deg")
        p.add_argument("--no-log", action="store_true")
        return p

    p = common(sub.add_parser("sag", help="유지 전류 하한 찾기"))
    p.add_argument("--start", type=int, default=60, help="시작 전류 [LSB]")
    p.add_argument("--end", type=int, default=5, help="끝 전류 [LSB]")
    p.add_argument("--step", type=int, default=5, help="감소 폭 [LSB]")
    p.add_argument("--dwell", type=float, default=2.0, help="각 값에서 머무는 시간 [s]")
    p.add_argument("--droop-ok", type=float, default=0.5, help="허용 처짐 [deg]")
    p.add_argument("--max-droop", type=float, default=8.0, help="이 이상이면 중단 [deg]")
    p.add_argument("--hold-margin", type=float, default=HOLD_MARGIN,
                   help=f"유지 전류 = 붕괴점 x 이 배수 (기본 {HOLD_MARGIN:g})")
    p.set_defaults(func=cmd_sag)

    p = common(sub.add_parser("step", help="재현 가능한 복원 시험"))
    p.add_argument("--amp", type=float, default=12.0, help="초기 변위 [deg]")
    p.add_argument("--reps", type=int, default=5)
    p.add_argument("--dirs", choices=("both", "pos", "neg"), default="both")
    p.add_argument("--settle", type=float, default=0.6, help="변위 후 안정 대기 [s]")
    p.add_argument("--timeout", type=float, default=4.0)
    p.set_defaults(func=cmd_step)

    p = common(sub.add_parser("release", help="손으로 누르는 실제 외란"))
    p.add_argument("--duration", type=float, default=60.0)
    p.set_defaults(func=cmd_release)

    p = common(sub.add_parser("sweep", help="파라미터 격자 탐색"))
    p.add_argument("--grid", action="append", metavar="name=v1,v2",
                   help="예: --grid p_gain=80,120,160 (여러 번)")
    p.add_argument("--amp", type=float, default=12.0)
    p.add_argument("--reps", type=int, default=3)
    p.add_argument("--dirs", choices=("both", "pos", "neg"), default="both")
    p.add_argument("--settle", type=float, default=0.6)
    p.add_argument("--timeout", type=float, default=4.0)
    p.add_argument("--max-osc", type=float, default=2.0, help="진동 허용 상한")
    p.add_argument("--max-overshoot", type=float, default=25.0, help="OS 허용 상한 [%%]")
    p.set_defaults(func=cmd_sweep)
    return ap


def main(argv):
    ap = build_parser()
    args = ap.parse_args(argv[1:])
    args.base_overrides = list(args.overrides or [])
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main(sys.argv))
