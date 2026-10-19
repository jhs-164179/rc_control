#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""XM430 driver selection and SuspensionConfig-to-motor initialization.

Real SDK imports are deferred so --mock runs without hardware dependencies.
"""

from __future__ import annotations

def load_driver(mock: bool = False):
    """Import hardware dependencies only when a real motor is requested."""
    if mock:
        from . import mock as driver
    else:
        from . import driver
    return driver


def open_motor(cfg, mock: bool = False, port: str = "", verbose: bool = True):
    """
    SuspensionConfig -> (driver_module, XM430, configure() 반환 info).

    Operating Mode 를 바꾸면 Profile/PID/Goal Current 가 리셋되므로
    configure() 안의 기록 순서를 그대로 따르는 것이 중요하다. 그래서
    드라이버의 configure() 를 재구현하지 않고 그대로 호출한다.
    """
    dxl = load_driver(mock)
    mcfg = cfg.to_motor_config(dxl)
    port = port or cfg.port or dxl.find_port()
    motor = dxl.XM430(port, mcfg)
    info = motor.configure(verbose=verbose)

    if not mock:
        lat = dxl.read_latency_timer(port)
        if lat is not None and lat > 2:
            print(f"[link] 경고: FTDI latency_timer = {lat} ms. "
                  f"제어 주기가 {1000.0 / max(lat, 1):.0f} Hz 근처에서 막힙니다.\n"
                  f"       낮추려면: sudo sh setup_latency.sh")
    return dxl, motor, info
