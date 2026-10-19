#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
sim_offline.py — ROS 없이 follower_core 를 폐루프로 돌린다 (CLAUDE.md §7.2 의 오프라인 판)

    python3 sim_offline.py PATH/path_left.csv PATH/path_right.csv ...
    python3 sim_offline.py PATH/path_*.csv --noise 0.01 --latency 0.1
    python3 sim_offline.py PATH/path_*.csv --swap      # 주행 중 경로 교체 + 빈 경로 시험
    python3 sim_offline.py PATH/path_*.csv --turn-ratio 0.8 --understeer-gain 1.25   # 언더스티어 보정 시험

로버 모델은 path_tracking.SimRover (sim_rover.py 와 같은 것: 조향 1차 지연 + 변화율 한계,
속도 1차 지연). pose 는 pose_hz 로 latency 만큼 늦게, 잡음을 섞어 들어오고 노드처럼
odom 으로 보간(PoseEstimator)한다. 제어는 50 Hz.

출력: 끝점 도달 여부, 첫 0.3 m 이후 max |e_y|, p95 |e_y|, 포화 비율, 최대 |dkappa_cmd|/dt.
"""

from __future__ import annotations

import argparse
import csv
import math
import os
import random
import sys

from follower_core import (STOP_END, STOP_GOAL, TRACKING, Follower, FollowerParams,
                           speed_to_rpm)
from path_tracking import DEFAULT_WHEELBASE_M, PoseEstimator, SimRover

RPM_PER_MPS = 60.0 * 10.0 / (math.pi * 0.1)     # 바퀴 0.1 m, 기어 10 (§7.2 명령과 같은 값)


def read_csv(path):
    pts = []
    with open(path, encoding="utf-8") as f:
        for r in csv.reader(line for line in f if not line.startswith("#")):
            if r and r[0] != "x_m":
                pts.append((float(r[0]), float(r[1])))
    return pts


def transform(pts, x0, y0, yaw0):
    c, s = math.cos(yaw0), math.sin(yaw0)
    return [(x0 + c * x - s * y, y0 + s * x + c * y) for x, y in pts]


def run(pts, *, p=None, noise=0.0, latency=0.0, pose_hz=10.0, swap_pts=None, swap_t=None,
        empty_t=None, start=(0.0, 0.0, 0.0), seconds=40.0, seed=1, turn_ratio=1.0):
    rnd = random.Random(seed)
    p = p or FollowerParams()
    f = Follower(p)
    rover = SimRover(*start, wheelbase_m=DEFAULT_WHEELBASE_M, rpm_per_mps=RPM_PER_MPS,
                     turn_ratio=turn_ratio)
    est = PoseEstimator(use_odom=True)
    dt, t, dist = 1.0 / 50.0, 0.0, 0.0
    queue, hist, t_pose = [], [], -1.0         # 지연 중인 pose (도착 시각, stamp, pose)
    stamp = None
    f.submit_path(pts)
    rpm, steer = 0.0, 0.0
    log = []
    swapped = None
    while t < seconds:
        for _ in range(20):
            rover.step(rpm, steer, dt / 20)
        t += dt
        dist += abs(rover.v) * dt
        hist.append((t, rover.x, rover.y, rover.yaw))
        est.on_odom(*rover.odom, rover.v, t)
        if t - t_pose >= 1.0 / pose_hz - 1e-9:
            t_pose = t
            queue.append((t + latency, t, (rover.x + rnd.gauss(0, noise), rover.y + rnd.gauss(0, noise),
                                           rover.yaw + rnd.gauss(0, math.radians(noise * 100)))))
        while queue and queue[0][0] <= t:
            _, stamp, mp = queue.pop(0)
            # 지연된 pose 를 받는 순간: 노드처럼 그 시점 odom 기준으로 보간
            est.on_map_pose(*mp, t)
        pose_age = math.inf if stamp is None else t - stamp

        if swap_pts is not None and swapped is None and t >= swap_t:
            # 계획 시점(0.3 s 전) 위치에서 시작하는 새 경로가 지금 도착
            _, px, py, pyaw = min(hist, key=lambda h: abs(h[0] - (t - 0.3)))
            f.submit_path(transform(swap_pts, px, py, pyaw))
            swapped = len(log)
        if empty_t is not None and t >= empty_t:
            f.submit_path([])
            empty_t = None

        out = f.step(dt, est.estimate(), pose_age, rover.v, True)
        rpm = speed_to_rpm(out.v_cmd, RPM_PER_MPS, 100.0, 2000.0)
        steer = math.degrees(out.delta_cmd)
        log.append((t, dist, out, f.active.s_proj if f.active else math.nan))
        if out.state in (STOP_END, STOP_GOAL) and rover.v < 1e-3:
            break

    trk = [(d, o) for _, d, o, _ in log if o.state == TRACKING]
    after = sorted(abs(o.e_y) for d, o in trk if d > 0.3)
    dk = max((abs(b[2].kappa_cmd - a[2].kappa_cmd) / dt for a, b in zip(log, log[1:])), default=0)
    last = log[-1][2]
    res = dict(
        state=last.state, t=t, path_id=last.path_id,
        end_err=None if f.active is None else math.hypot(rover.x - f.active.x[-1], rover.y - f.active.y[-1]),
        max_after=after[-1] if after else math.nan,
        p95=after[int(0.95 * (len(after) - 1))] if after else math.nan,
        sat=sum(o.saturated for _, o in trk) / max(1, len(trk)),
        dk_max=dk, swap_s=None if swapped is None else log[min(swapped, len(log) - 1)][3],
        swap_dk=None if swapped is None or swapped == 0 else
        max(abs(log[i][2].kappa_cmd - log[i - 1][2].kappa_cmd) / dt
            for i in range(swapped, min(swapped + 5, len(log)))),
    )
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+")
    ap.add_argument("--noise", type=float, default=0.0, help="pose 잡음 [m] (yaw 는 noise*100 deg)")
    ap.add_argument("--latency", type=float, default=0.0, help="pose 지연 [s]")
    ap.add_argument("--pose-hz", type=float, default=10.0)
    ap.add_argument("--k-e", type=float, default=1.5)
    ap.add_argument("--swap", action="store_true", help="경로 교체 / 빈 경로 시험도 한다")
    ap.add_argument("--turn-ratio", type=float, default=1.0, help="로버 모델 언더스티어 (실제/지령 곡률)")
    ap.add_argument("--understeer-gain", type=float, default=1.0, help="추종기 보정 (1 = 끔)")
    a = ap.parse_args()

    p = FollowerParams(k_e=a.k_e, understeer_gain=a.understeer_gain)
    rate = p.kappa_rate / a.understeer_gain       # kappa_cmd 는 실제 곡률이라 변화율 한계도 그만큼 작다
    print(f"k_e={a.k_e}, noise={a.noise} m, latency={a.latency} s, pose {a.pose_hz} Hz, "
          f"turn_ratio={a.turn_ratio}, understeer_gain={a.understeer_gain}, "
          f"kappa_rate={rate:g} -> 허용 |dk/dt| <= {rate:g}")
    paths = {os.path.basename(c): read_csv(c) for c in a.csv}
    ok = True
    for name, pts in paths.items():
        r = run(pts, noise=a.noise, latency=a.latency, pose_hz=a.pose_hz, turn_ratio=a.turn_ratio,
                p=FollowerParams(k_e=a.k_e, understeer_gain=a.understeer_gain))
        good = r["state"] == STOP_END and r["end_err"] < 0.05 and r["dk_max"] <= rate + 1e-6
        ok &= good
        print(f"  {name:<18} {r['state']:<10} t={r['t']:5.1f}s  끝점오차 {r['end_err'] * 100:4.1f} cm  "
              f"max|e_y|(0.3 m 후) {r['max_after'] * 100:4.1f} cm  p95 {r['p95'] * 100:4.1f} cm  "
              f"포화 {r['sat'] * 100:4.1f} %  max|dk/dt| {r['dk_max']:.2f}  {'OK' if good else 'FAIL'}")

    if a.swap and len(paths) >= 2:
        names = list(paths)
        print("경로 교체 (2.0 s 에 다른 경로를 0.3 s 전 pose 기준으로 도착시킴. 첫 투영점 s > 0 = 앞쪽에서 잡음):")
        for i, n1 in enumerate(names):
            n2 = names[(i + 1) % len(names)]
            r = run(paths[n1], swap_pts=paths[n2], swap_t=2.0, noise=a.noise, latency=a.latency,
                    p=FollowerParams(k_e=a.k_e))
            good = (r["path_id"] == 2 and r["swap_dk"] <= rate + 1e-6 and r["swap_s"] > 0.02
                    and r["state"] == STOP_END)
            ok &= good
            print(f"  {n1} -> {n2:<18} path_id={r['path_id']}  첫 투영점 s={r['swap_s'] * 100:.1f} cm  "
                  f"교체 직후 max|dk/dt| {r['swap_dk']:.2f}  {r['state']}  {'OK' if good else 'FAIL'}")
        r = run(paths[names[0]], empty_t=2.0)
        good = r["state"] == STOP_GOAL
        ok &= good
        print(f"빈 경로 (2.0 s): {r['state']} t={r['t']:.2f}s  {'OK' if good else 'FAIL'}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
