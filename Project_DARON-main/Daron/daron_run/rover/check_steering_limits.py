#!/usr/bin/env python3
"""Inspect a single steering angle; optionally apply it for manual bench measurement.

Stop the rover node before --apply. This tool cannot detect mechanical contact:
increase angles gradually and record the independently measured left/right limits.
Without --apply no GPIO or PWM writes occur.
"""
import argparse
from motor_control.l12.driver import (L12Pair, solve_steering, max_effective_steer,
                        FRONT_PWM_PATH, REAR_PWM_PATH)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--angle', type=float, default=0.0)
    parser.add_argument('--apply', action='store_true')
    parser.add_argument('--front-pwm', default=FRONT_PWM_PATH)
    parser.add_argument('--rear-pwm', default=REAR_PWM_PATH)
    args = parser.parse_args()
    sol = solve_steering(args.angle, -30.0)
    print(f'Geometry-only bound: +/-{max_effective_steer(150):.3f} deg (not a measured limit)')
    print(f'Command {sol.steer_eff_deg:+.3f} deg: front={sol.front_mm:.3f} mm, rear={sol.rear_mm:.3f} mm')
    if not args.apply:
        return
    pair = L12Pair(args.front_pwm, args.rear_pwm)
    if pair.simulate:
        raise RuntimeError('PWM channels unavailable; refusing a simulated hardware measurement')
    try:
        pair.initialize()
        input('Ready on bench; press Enter to apply this angle (Ctrl+C to cancel): ')
        pair.set_steering(args.angle, -30.0)
        input('Measure angle and clearance. Press Enter to return to neutral: ')
    finally:
        import time
        try:
            pair.neutral()
            time.sleep(3.0)
        finally:
            pair.close()


if __name__ == '__main__':
    main()
