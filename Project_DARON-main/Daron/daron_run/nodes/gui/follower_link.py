#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
follower_link — 제어창의 '웨이포인트' 버튼 ↔ 경로 추종기 (ROS 쪽, Qt 없음). 작성 261012.

추종기 = daron_run/rover/server/path_follower.py (노드 /path_follower, Stanley 곡률 제어, rover/PATH_TRACKING.md).
daron_gui.launch.py 가 follower:=true(기본)로 같이 띄우고, 기동 때는 꺼져 있다(start_enabled false).

  보냄                                      받음
    <ns>/enable  std_msgs/Bool  추종 ON/OFF     <ns>/status  DiagnosticArray (10 Hz, status[0].values = state·e_y_m·rpm ...)

  - 버튼 상태의 기준은 추종기가 status 로 알려 주는 enabled 값이다. 누른 직후 status 가 따라올 때까지(PENDING_S)만
    누른 값을 보여 준다 → 메시지가 안 갔으면 버튼이 저절로 원래대로 돌아온다 (다시 보내기는 안 함:
    다른 곳에서 끈 것을 제어창이 다시 켜면 안 되므로).
  - 추종기가 꺼질 때는 0 지령을 한 번 보내고 발행을 멈춘다. 로버(radxa.py)는 auto_timeout(0.5 s) 뒤 조향을 중립으로.
  - 제어창을 닫을 때 추종기가 켜져 있으면 끈다 (조작자 화면 없이 추종 주행이 계속되지 않게).
  - 로버 쪽 자율 허용(/rover/auto_enable, console.py P 키)은 보내지 않는다. 진단으로 보여 주기만 한다 (rover_panel).
"""
from __future__ import annotations

import threading
import time

from diagnostic_msgs.msg import DiagnosticArray
from std_msgs.msg import Bool

STALE_S = 2.0       # status 가 이만큼 끊기면 '추종기 없음' (10 Hz 발행)
PENDING_S = 1.0     # 버튼을 누른 뒤 status 가 따라오기를 기다리는 시간

# path_follower 상태 (follower_core) → 표시 이름
STATE_NAME = {'IDLE': '대기', 'TRACKING': '추종 중', 'STOP_GOAL': '정지 (빈 경로)', 'STOP_END': '도착',
              'STOP_STALE': '정지 (위치 끊김)', 'HOLD_DEVIATION': '정지 (경로 이탈)'}


class FollowerLink:
    """제어창 노드(GroundNode)에 붙인다. 구독 콜백은 spin 스레드, 나머지는 Qt 스레드에서 부른다."""

    def __init__(self, node, ns='/path_follower'):
        self.node = node
        self.ns = ns.rstrip('/') or '/path_follower'
        self.pub_enable = node.create_publisher(Bool, f'{self.ns}/enable', 10)
        node.create_subscription(DiagnosticArray, f'{self.ns}/status', self._on_status, 10)
        self._lock = threading.Lock()
        self._st = {}
        self._msg = ''
        self._last_rx = 0.0
        self._want = None           # (누른 값, monotonic) — status 가 따라오기 전까지
        node.get_logger().info(f'[261012] 웨이포인트 버튼 → {self.ns}/enable, 상태 ← {self.ns}/status')

    # ---------- 수신 (spin 스레드)
    def _on_status(self, msg: DiagnosticArray):
        if not msg.status:
            return
        st = msg.status[0]
        with self._lock:
            self._st = {kv.key: kv.value for kv in st.values}
            self._msg = st.message
            self._last_rx = time.monotonic()

    def snapshot(self):
        """→ (status dict, message, 마지막 수신 이후 초)."""
        with self._lock:
            return dict(self._st), self._msg, (time.monotonic() - self._last_rx if self._last_rx else 1e9)

    def alive(self):
        return self.snapshot()[2] <= STALE_S

    def enabled(self):
        """추종기가 켜져 있는가. 누른 직후 PENDING_S 동안은 누른 값 (status 가 같아지면 바로 status 값)."""
        st, _m, age = self.snapshot()
        actual = age <= STALE_S and st.get('enabled') == 'True'
        want = self._want
        if want is not None:
            if want[0] != actual and time.monotonic() - want[1] < PENDING_S:
                return want[0]
            self._want = None
        return actual

    # ---------- 송신 (Qt 스레드)
    def set_enabled(self, on: bool):
        on = bool(on)
        self.pub_enable.publish(Bool(data=on))
        self._want = (on, time.monotonic())
        self.node.get_logger().info(f'웨이포인트 추종 {"ON — 추종기 켬" if on else "OFF — 추종기 끔"} ({self.ns}/enable)')

    def shutdown(self):
        """창 닫을 때. 추종기가 켜져 있으면 끈다."""
        if self.enabled():
            try:
                self.pub_enable.publish(Bool(data=False))
                time.sleep(0.3)
            except Exception:  # noqa: BLE001 — 종료 중 context 내려감
                pass
            self.node.get_logger().info('제어창 종료 — 추종기 끔')
