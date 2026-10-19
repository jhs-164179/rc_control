#!/usr/bin/env python3
import re
import argparse
import numpy as np
import matplotlib.pyplot as plt


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("log_file", help="PPM terminal log (.txt)")
    parser.add_argument("--output_ch2", default="ch2_ppm_plot.png")
    parser.add_argument("--output_all", default="ch_all_ppm_plot.png")
    parser.add_argument("--ymin", type=float, default=800.0)
    parser.add_argument("--ymax", type=float, default=2500.0)
    args = parser.parse_args()

    pattern = re.compile(
        r"\[\s*([\d.]+)s\]\s+"
        r"CH1=\s*(\d+)\s+"
        r"CH2=\s*(\d+)\s+"
        r"CH3=\s*(\d+)\s+"
        r"CH4=\s*(\d+)"
    )

    times = []
    ch1 = []
    ch2 = []
    ch3 = []
    ch4 = []

    loss_times = []
    pending_loss = False

    with open(args.log_file, "r", encoding="utf-8", errors="ignore") as f:
        for line in f:
            if "[SIGNAL_LOST]" in line:
                pending_loss = True
                continue

            match = pattern.search(line)
            if match is None:
                continue

            t = float(match.group(1))
            v1 = int(match.group(2))
            v2 = int(match.group(3))
            v3 = int(match.group(4))
            v4 = int(match.group(5))

            if pending_loss:
                if times:
                    loss_times.append((times[-1] + t) / 2.0)
                pending_loss = False

            times.append(t)
            ch1.append(v1)
            ch2.append(v2)
            ch3.append(v3)
            ch4.append(v4)

    if not times:
        raise RuntimeError("No valid PPM samples found")

    times = np.array(times)
    ch1 = np.array(ch1)
    ch2 = np.array(ch2)
    ch3 = np.array(ch3)
    ch4 = np.array(ch4)

    print(f"Samples: {len(times)}")
    print(f"Duration: {times[-1] - times[0]:.2f} s")
    print(f"Signal loss count: {len(loss_times)}")
    print()
    print(f"CH1 Mean: {np.mean(ch1):.2f} us, Min: {np.min(ch1)}, Max: {np.max(ch1)}")
    print(f"CH2 Mean: {np.mean(ch2):.2f} us, Min: {np.min(ch2)}, Max: {np.max(ch2)}")
    print(f"CH3 Mean: {np.mean(ch3):.2f} us, Min: {np.min(ch3)}, Max: {np.max(ch3)}")
    print(f"CH4 Mean: {np.mean(ch4):.2f} us, Min: {np.min(ch4)}, Max: {np.max(ch4)}")

    # -------------------------------
    # Figure 1: CH2 only
    # -------------------------------
    fig1, ax1 = plt.subplots(figsize=(14, 5))
    ax1.plot(times, ch2, linewidth=0.8, label="CH2")
    ax1.axhline(
        np.mean(ch2),
        linestyle="--",
        color="green",
        label=f"Mean = {np.mean(ch2):.1f} us"
    )

    for i, t in enumerate(loss_times):
        ax1.axvline(
            t,
            color="red",
            linestyle="--",
            linewidth=1.5,
            label="Signal Lost" if i == 0 else None
        )

    ax1.set_title("RC PPM CH2 Throttle Signal")
    ax1.set_xlabel("Elapsed Time (s)")
    ax1.set_ylabel("Pulse Width (us)")
    ax1.set_ylim(args.ymin, args.ymax)
    ax1.grid(True, alpha=0.3)
    ax1.legend()
    plt.tight_layout()
    fig1.savefig(args.output_ch2, dpi=200)

    # -------------------------------
    # Figure 2: CH1~CH4 together
    # -------------------------------
    fig2, ax2 = plt.subplots(figsize=(14, 6))
    ax2.plot(times, ch1, linewidth=0.8, label="CH1")
    ax2.plot(times, ch2, linewidth=0.8, label="CH2")
    ax2.plot(times, ch3, linewidth=0.8, label="CH3")
    ax2.plot(times, ch4, linewidth=0.8, label="CH4")

    for i, t in enumerate(loss_times):
        ax2.axvline(
            t,
            color="red",
            linestyle="--",
            linewidth=1.2,
            label="Signal Lost" if i == 0 else None
        )

    ax2.set_title("RC PPM CH1~CH4 Signals")
    ax2.set_xlabel("Elapsed Time (s)")
    ax2.set_ylabel("Pulse Width (us)")
    ax2.set_ylim(args.ymin, args.ymax)
    ax2.grid(True, alpha=0.3)
    ax2.legend()
    plt.tight_layout()
    fig2.savefig(args.output_all, dpi=200)

    plt.show()

    print()
    print(f"Saved CH2 plot : {args.output_ch2}")
    print(f"Saved All plot : {args.output_all}")


if __name__ == "__main__":
    main()