#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze.py — 로그 CSV -> 지표.

지표 정의는 여기 한 곳에만 둔다. susp_controller 의 실시간 표시도
같은 함수를 부른다. 정의가 두 벌이면 콘솔 숫자와 리포트 숫자가 갈라진다.

지표
----
  t90       초기 변위의 10 % 까지 줄어드는 데 걸린 시간. '복원 속도'의 본체.
  t_settle  이후로 다시는 settled 밴드를 벗어나지 않게 된 시각. 진동까지 포함.
  OS        오버슈트 [%]. 목표를 지나친 최대량 / 초기 변위.
  osc       목표선을 가로지른 횟수. 2 이상이면 D 를 올리거나 복원 전류를 낮춘다.
  Ipk       복원 중 최대 전류 [mA]. 설정한 상한에 닿았는지 = 전류가 병목인지.

사용
----
  python3 analyze.py summary logs/release_*.csv
  python3 analyze.py asym    logs/bidir_*.csv
  python3 analyze.py compare logs/step_*.csv
"""

from __future__ import annotations

from motor_control.xm430.metrics import NAN, MIN_SPAN_DEG, recovery_metrics, _median

import glob
import sys

from motor_control.xm430.log import read_csv


# 실제로 복원이 일어나지 않은 이벤트는 통계에서 뺀다. 정적 처짐을 조금
# 되돌리려다 만 것까지 세면 t90 중앙값이 오염된다. span = 초기 오차에서
# 최종 평형까지 실제로 움직인 각도.




def split_events(rows, start_tag="RECOVER_START", end_tags=()):
    """event 열의 마커로 복원 구간을 잘라낸다."""
    segs = []
    cur = None
    for r in rows:
        ev = (r.get("event") or "").strip()
        if start_tag in ev:
            cur = [r]
        elif cur is not None:
            cur.append(r)
            if "RECOVER_END" in ev:
                segs.append(cur)
                cur = None
    if cur and len(cur) > 2:
        segs.append(cur)
    return segs


def seg_metrics(seg, settled_deg):
    t = [r["t"] for r in seg]
    e = [r["err_deg"] for r in seg]
    v = [r["vel_dps"] for r in seg]
    c = [r["cur_mA"] for r in seg]
    m = recovery_metrics(t, e, v, c, settled_deg)
    m["dir"] = 1 if m["err0_deg"] >= 0 else -1
    m["gc"] = seg[0].get("gc_lsb", NAN)
    return m


def _ms(v):
    return "   --" if v != v else f"{v * 1000:5.0f}"


def _fmt(m):
    d = "+CCW" if m["dir"] > 0 else "-CW "
    return (f"  {d}  {m['err0_deg']:+7.2f}  {m['gc']:5.0f}  {_ms(m['t90_s'])}  "
            f"{_ms(m['t_settle_s'])}  {m['peak_vel_dps']:7.1f}  "
            f"{m['overshoot_pct']:6.1f}  {m['osc']:3d}  "
            f"{m['residual_deg']:+7.2f}  {m['peak_mA']:6.0f}")


HDR = ("  방향   err0[°]  I[LSB]  t90[ms] 정착[ms]  vpk[°/s]  "
       "OS[%]  진동  잔류[°]  Ipk[mA]")




def summarize(paths, verbose=True):
    """파일들의 모든 복원 구간 지표. 방향별 중앙값과 비대칭 지수를 낸다."""
    all_m = []
    for p in paths:
        rows, meta = read_csv(p)
        settled = float(meta.get("config", {}).get("settled_band_deg", 0.7))
        segs = split_events(rows)
        if not segs:
            print(f"[analyze] {p}: 복원 구간 마커가 없습니다.")
            continue
        if verbose:
            print(f"\n=== {p}  ({len(segs)} 개 복원) ===")
            print(HDR)
        skipped = 0
        for s in segs:
            m = seg_metrics(s, settled)
            span = m.get("span_deg")
            if span != span or abs(span) < MIN_SPAN_DEG:
                skipped += 1
                continue
            m["file"] = p
            all_m.append(m)
            if verbose:
                print(_fmt(m))
        if verbose and skipped:
            print(f"  (복원량 {MIN_SPAN_DEG}° 미만인 이벤트 {skipped} 개 제외)")
    return all_m


def asymmetry_report(all_m):
    """방향별 중앙값 대조 + 비대칭 지수 R."""
    pos = [m for m in all_m if m["dir"] > 0]
    neg = [m for m in all_m if m["dir"] < 0]
    print("\n────────── 방향 비대칭 ──────────")
    if not pos or not neg:
        print("  양방향 데이터가 모두 필요합니다 (bidir 실험을 쓰세요).")
        return None
    rows = []
    for key, label, scale in (("t90_s", "t90 [ms]", 1000.0),
                              ("t_settle_s", "정착 [ms]", 1000.0),
                              ("peak_vel_dps", "피크속도 [°/s]", 1.0),
                              ("mean_vel_dps", "평균속도 [°/s]", 1.0),
                              ("overshoot_pct", "오버슈트 [%]", 1.0),
                              ("residual_deg", "잔류 [°]", 1.0),
                              ("peak_mA", "피크전류 [mA]", 1.0)):
        a = _median([m[key] for m in pos]) * scale
        b = _median([m[key] for m in neg]) * scale
        rows.append((label, a, b))
    w = max(len(r[0]) for r in rows)
    print(f"  {'지표'.ljust(w)}   +CCW      -CW      비   (N={len(pos)}/{len(neg)})")
    for label, a, b in rows:
        r = (a / b) if (b and b == b and a == a and abs(b) > 1e-9) else NAN
        rs = "  --  " if r != r else f"{r:5.3f}"
        print(f"  {label.ljust(w)}  {a:7.1f}  {b:7.1f}   {rs}")

    t_pos = _median([m["t90_s"] for m in pos])
    t_neg = _median([m["t90_s"] for m in neg])
    if t_pos == t_pos and t_neg == t_neg and t_neg > 0:
        R = t_pos / t_neg
        print(f"\n  비대칭 지수 R = t90(+CCW)/t90(-CW) = {R:.3f}")
        if 0.95 <= R <= 1.05:
            print("  -> 목표 범위(0.95~1.05) 안입니다. 보정 불필요.")
        else:
            slow = "+CCW" if R > 1 else "-CW"
            print(f"  -> {slow} 쪽이 느립니다. asymmetry.py cal 로 보정값을 구하세요.")
        return R
    return None


def compare(paths):
    """여러 런을 한 줄씩 요약해 파라미터 스윕 결과를 비교한다."""
    print(f"{'파일':<34} {'P':>4} {'D':>4} {'Irec':>5} {'vprof':>6} "
          f"{'t90[ms]':>8} {'정착[ms]':>9} {'OS[%]':>6} {'진동':>4}")
    out = []
    for p in sorted(paths):
        rows, meta = read_csv(p)
        c = meta.get("config", {})
        settled = float(c.get("settled_band_deg", 0.7))
        ms = [seg_metrics(s, settled) for s in split_events(rows)]
        ms = [m for m in ms if m["span_deg"] == m["span_deg"]
              and abs(m["span_deg"]) >= MIN_SPAN_DEG]
        if not ms:
            continue
        rec = dict(
            file=p.split("/")[-1],
            p=c.get("p_gain"), d=c.get("d_gain"),
            irec=c.get("current_recover"), v=c.get("profile_velocity_dps"),
            t90=_median([m["t90_s"] for m in ms]) * 1000.0,
            ts=_median([m["t_settle_s"] for m in ms]) * 1000.0,
            os=_median([m["overshoot_pct"] for m in ms]),
            osc=_median([float(m["osc"]) for m in ms]),
        )
        out.append(rec)
    for r in sorted(out, key=lambda r: (r["t90"] if r["t90"] == r["t90"] else 1e9)):
        print(f"{r['file']:<34} {r['p']:>4} {r['d']:>4} {r['irec']:>5} "
              f"{r['v']:>6.0f} {r['t90']:>8.0f} {r['ts']:>9.0f} "
              f"{r['os']:>6.1f} {r['osc']:>4.0f}")
    return out


def main(argv):
    if len(argv) < 2:
        print(__doc__)
        return 1
    cmd = argv[1]
    paths = []
    for a in argv[2:]:
        paths.extend(sorted(glob.glob(a)) or [a])
    if not paths:
        print("파일이 없습니다.")
        return 1
    if cmd == "summary":
        summarize(paths)
    elif cmd == "asym":
        asymmetry_report(summarize(paths))
    elif cmd == "compare":
        compare(paths)
    else:
        print(f"모르는 명령: {cmd}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
