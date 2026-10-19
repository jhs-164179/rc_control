
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import time
from datetime import datetime
from pathlib import Path

from ppm_decoder import PPMDecoder

LOG_HZ = 10.0
CHANGE_THRESHOLD_US = 5
TIMEOUT_S = 0.3

def main():
    log_path = Path(
        f"ppm_log_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
    )

    decoder = PPMDecoder(
        chip="gpiochip3",
        line=20,
        num_channels=8
    )

    previous = None
    last_valid = False

    try:
        decoder.start()
        print("PPM monitoring started (CH1~CH4)")
        print(f"CSV: {log_path.resolve()}")
        print("Press Ctrl+C to stop.\n")

        with open(log_path, "w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp", "elapsed_s", "status",
                "ch1_us", "ch2_us", "ch3_us", "ch4_us"
            ])

            start = time.monotonic()

            while True:
                channels = decoder.get_channels(timeout_s=TIMEOUT_S)
                now = time.monotonic()

                if channels is None:
                    if last_valid:
                        print("[SIGNAL_LOST]", flush=True)
                    writer.writerow([
                        datetime.now().isoformat(timespec="milliseconds"),
                        round(now - start, 3),
                        "SIGNAL_LOST", "", "", "", ""
                    ])
                    last_valid = False
                    previous = None

                else:
                    values = channels[:4]

                    writer.writerow([
                        datetime.now().isoformat(timespec="milliseconds"),
                        round(now - start, 3),
                        "OK", *values
                    ])

                    if previous is None:
                        changed = True
                    else:
                        changed = any(
                            abs(a - b) >= CHANGE_THRESHOLD_US
                            for a, b in zip(values, previous)
                        )

                    if changed or not last_valid:
                        print(
                            f"[{now - start:8.2f}s] "
                            f"CH1={values[0]:4d}  "
                            f"CH2={values[1]:4d}  "
                            f"CH3={values[2]:4d}  "
                            f"CH4={values[3]:4d}",
                            flush=True
                        )
                        previous = values.copy()

                    last_valid = True

                f.flush()
                time.sleep(1.0 / LOG_HZ)

    except KeyboardInterrupt:
        print("\nMonitoring stopped.")
    finally:
        decoder.close()
        print(f"Saved: {log_path.resolve()}")

if __name__ == "__main__":
    main()
