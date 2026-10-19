#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
plot_pitch_validation.py -- Fig. 7 (Active Pitch Compliance) for the ICRA paper.

Three run logs recorded with identical home angle, bands, gains and trajectory
profile, so the goal-current bounds are the only varied factor:

    (a) one disturb-release cycle of the proposed dual-bound scheme
    (b) resisting current while the load is applied, per condition
    (c) release-aligned recovery, per condition

Performance is recomputed here with one set of criteria that does not depend on
any per-run config value; the FSM's own RECOVER_END labels depend on
settled_band_deg and are therefore not comparable across conditions.

    python3 plot_pitch_validation.py \
        --high logs/run_20260917_132345.csv \
        --low  logs/run_20260917_132432.csv \
        --dual logs/run_20260917_132515.csv \
        --out  figures/pitch_validation.pdf
"""

from __future__ import annotations

import argparse
import os

import matplotlib
matplotlib.use("Agg")

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

# ---------------------------------------------------------------- criteria
CURRENT_LSB_MA = 2.69      # XM430 goal-current unit

REL_MIN_DEG = 20.0         # a RECOVER_START below this is not a release
T90_FRAC = 0.10            # t90: first time |e| <= T90_FRAC * |e0|
SETTLE_TOL_DEG = 1.0       # config-independent settling criterion
WIN_S = 2.0                # analysis window after a release
GRID_DT = 0.005
GRID_T_MAX = 0.90

SHADE = {"HOLD": "#ffffff", "COMPLY": "#ececec", "RECOVER": "#d0d0d0"}

# ---------------------------------------------------------------- style
FIG_W_IN, FIG_H_IN = 7.16, 2.05

plt.rcParams.update({
    "font.family": "serif",
    "font.serif": ["Times New Roman", "Nimbus Roman", "Liberation Serif",
                   "DejaVu Serif"],
    "mathtext.fontset": "dejavuserif",
    "font.size": 7, "axes.labelsize": 7, "axes.titlesize": 7,
    "legend.fontsize": 6, "xtick.labelsize": 6.5, "ytick.labelsize": 6.5,
    "axes.linewidth": 0.6, "xtick.major.width": 0.6, "ytick.major.width": 0.6,
    "xtick.direction": "in", "ytick.direction": "in",
    "axes.spines.top": False, "axes.spines.right": False,
    "lines.linewidth": 1.0, "legend.frameon": False,
    "pdf.fonttype": 42, "ps.fonttype": 42,
    "savefig.bbox": "standard", "savefig.pad_inches": 0.02,
    "figure.constrained_layout.use": True,
    "figure.constrained_layout.w_pad": 0.04,
    "figure.constrained_layout.wspace": 0.08,
})


# ---------------------------------------------------------------- data
def load_run(path):
    df = pd.read_csv(path)
    need = {"t", "state", "err_deg", "cur_mA", "gc_lsb", "event"}
    if need - set(df.columns):
        raise ValueError(f"{path}: missing {sorted(need - set(df.columns))}")
    df["event"] = df["event"].fillna("")
    df["abs_err"] = df["err_deg"].abs()
    df["abs_cur"] = df["cur_mA"].abs()
    return df.reset_index(drop=True)


def _first_below(w, thr, t0):
    hit = w[w["abs_err"] <= thr]
    return float(hit["t"].iloc[0] - t0) if len(hit) else np.nan


def release_events(df):
    """One entry per release: a RECOVER_START with |e0| above REL_MIN_DEG.
    Smaller triggers fire while the push is still in progress."""
    out = []
    for i in df.index[(df["event"] == "RECOVER_START") &
                      (df["abs_err"] >= REL_MIN_DEG)]:
        t0, e0 = df.at[i, "t"], df.at[i, "abs_err"]
        w = df[(df.index >= i) & (df["t"] <= t0 + WIN_S)]
        out.append({"idx": i, "t0": t0, "e0": e0,
                    "t90": _first_below(w, T90_FRAC * e0, t0),
                    "t_settle": _first_below(w, SETTLE_TOL_DEG, t0),
                    "t": w["t"].to_numpy() - t0,
                    "e_norm": (w["abs_err"] / e0).to_numpy()})
    return out


def recovery_median(events):
    grid = np.arange(0.0, GRID_T_MAX + 1e-9, GRID_DT)
    if not events:
        return grid, np.full_like(grid, np.nan)
    stack = np.vstack([np.interp(grid, e["t"], e["e_norm"],
                                 left=np.nan, right=e["e_norm"][-1])
                       for e in events])
    return grid, np.nanmedian(stack, axis=0)


def state_blocks(w):
    blocks, s0, t0 = [], None, None
    for _, r in w.iterrows():
        if r["state"] != s0:
            if s0 is not None:
                blocks.append((s0, t0, r["t"]))
            s0, t0 = r["state"], r["t"]
    return blocks + [(s0, t0, w["t"].iloc[-1])] if s0 else blocks


def pick_example(df, events):
    """Window around the median-amplitude release, extended back to the start
    of the preceding COMPLY block so the whole cycle is visible."""
    if not events:
        raise ValueError("no release event for panel (a)")
    e0 = np.array([e["e0"] for e in events])
    ev = events[int(np.argmin(np.abs(e0 - np.median(e0))))]
    j = ev["idx"]
    while j > 0 and df.at[j - 1, "state"] in ("COMPLY", "RECOVER"):
        j -= 1
    w = df[(df["t"] >= df.at[j, "t"] - 0.25) &
           (df["t"] <= ev["t0"] + 0.55)].copy()
    w["t_rel"] = w["t"] - ev["t0"]
    return w


# ---------------------------------------------------------------- panels
def panel_a(ax, df, events):
    w = pick_example(df, events)
    t = w["t_rel"].to_numpy()
    shift = float(w["t"].iloc[0] - t[0])

    ax2 = ax.twinx()                      # drawn underneath: shading goes here
    for state, a, b in state_blocks(w):
        ax2.axvspan(a - shift, b - shift, color=SHADE.get(state, "#fff"), lw=0)
    ax2.step(t, w["gc_lsb"] * CURRENT_LSB_MA, where="post", color="C3", lw=0.9)
    ax2.set_ylabel("Bound [mA]", color="C3")
    ax2.tick_params(axis="y", colors="C3")
    ax2.spines["right"].set_visible(True)
    ax2.spines["right"].set_color("C3")
    ax2.set_ylim(0, 1.5 * 150.0 * CURRENT_LSB_MA)

    ax.plot(t, w["err_deg"], color="k", lw=1.1)
    ax.axhline(0.0, color="0.6", lw=0.5, ls=":")
    ax.set_xlabel("Time from release [s]")
    ax.set_ylabel(r"Displacement [$^\circ$]")
    ax.set_xlim(t[0], t[-1])
    lo, hi = w["err_deg"].min(), w["err_deg"].max()
    ax.set_ylim(lo - 0.1 * (hi - lo), hi + 0.45 * (hi - lo))
    ax.set_zorder(ax2.get_zorder() + 1)
    ax.patch.set_visible(False)

    for state, a, b in state_blocks(w):
        if state != "HOLD" and (b - a) >= 0.06:
            ax.text(0.5 * (a + b) - shift, ax.get_ylim()[1], state,
                    ha="center", va="top", fontsize=5.5, color="0.3")
    ax.set_title("(a) Dual bound, one cycle", loc="left")


def panel_b(ax, conds):
    x = np.arange(len(conds))
    med = [c["df"].loc[c["df"]["state"] == "COMPLY", "abs_cur"].median()
           for c in conds]
    ax.bar(x, med, width=0.55, color=[c["color"] for c in conds])
    for xi, m in zip(x, med):
        ax.text(xi, m * 1.18, f"{m:.0f}", ha="center", va="bottom", fontsize=6)
    ax.set_xticks(x, [c["short"] for c in conds])
    ax.set_yscale("log")
    ax.set_ylim(5, 4.0 * max(med))
    ax.set_ylabel("Resisting current [mA]")
    ax.set_title("(b) Resistance under load", loc="left")


def panel_c(ax, conds):
    for c in conds:
        g, med = recovery_median(c["events"])
        ax.plot(g, med, color=c["color"], ls=c["ls"], lw=1.1,
                label=f"{c['short']} ({c['reached']})")
    ax.axhline(T90_FRAC, color="0.6", lw=0.5, ls=":")
    ax.set_xlabel("Time from release [s]")
    ax.set_ylabel(r"$|e|\,/\,|e_0|$")
    ax.set_xlim(0, GRID_T_MAX)
    ax.set_ylim(0, 1.05)
    ax.set_title("(c) Recovery after release", loc="left")
    ax.legend(loc="center right", handlelength=1.8, labelspacing=0.3)


# ---------------------------------------------------------------- main
def summarize(conds):
    rows = []
    for c in conds:
        ev, df = c["events"], c["df"]
        rows.append({
            "condition": c["label"],
            "bounds hold/comply/recover [LSB]": c["bounds"],
            "n releases": len(ev),
            "median |e0| [deg]": round(float(np.median([e["e0"] for e in ev])), 1),
            "median resisting current [mA]":
                round(df.loc[df["state"] == "COMPLY", "abs_cur"].median(), 1),
            "median t90 [s]":
                round(float(np.nanmedian([e["t90"] for e in ev])), 3),
            f"reached {SETTLE_TOL_DEG:g} deg": c["reached"],
            "median offset in HOLD [deg]":
                round(df.loc[df["state"] == "HOLD", "abs_err"].median(), 2),
        })
    return pd.DataFrame(rows)


def main():
    ap = argparse.ArgumentParser(
        description=__doc__,
        formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--high", required=True, help="single bound, high")
    ap.add_argument("--low", required=True, help="single bound, low")
    ap.add_argument("--dual", required=True, help="proposed dual bound")
    ap.add_argument("--out", default="figures/pitch_validation.pdf")
    ap.add_argument("--also-png", action="store_true")
    args = ap.parse_args()

    conds = [
        {"path": args.high, "label": "Single bound, high", "short": "High",
         "bounds": "150/150/150", "color": "C0", "ls": "-"},
        {"path": args.low, "label": "Single bound, low", "short": "Low",
         "bounds": "15/15/15", "color": "C1", "ls": "--"},
        {"path": args.dual, "label": "Dual bound (proposed)", "short": "Dual",
         "bounds": "15/10/150", "color": "k", "ls": "-"},
    ]
    for c in conds:
        c["df"] = load_run(c["path"])
        c["events"] = release_events(c["df"])
        if not c["events"]:
            raise SystemExit(f"{c['label']}: no release above {REL_MIN_DEG} deg")
        ok = int(np.sum(~np.isnan([e["t_settle"] for e in c["events"]])))
        c["reached"] = f"{ok}/{len(c['events'])}"

    table = summarize(conds)
    print(table.to_string(index=False))

    fig, axes = plt.subplots(1, 3, figsize=(FIG_W_IN, FIG_H_IN))
    panel_a(axes[0], conds[-1]["df"], conds[-1]["events"])
    panel_b(axes[1], conds)
    panel_c(axes[2], conds)

    out = os.path.abspath(args.out)
    os.makedirs(os.path.dirname(out), exist_ok=True)
    fig.savefig(out)
    table.to_csv(os.path.splitext(out)[0] + "_summary.csv", index=False)
    if args.also_png:
        fig.savefig(os.path.splitext(out)[0] + ".png", dpi=300)
    print(f"\nwrote {out}")


if __name__ == "__main__":
    main()