#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
auto_link — 제어창의 완전 자율(AUTO)·경기장 놓기·틈 지정 ↔ 경로계획 노드 (ROS 쪽, Qt 없음). 작성 261014 (경로계획 채팅, daron-nav auto-1014).

경로계획(daron-nav planner_node, 브랜치 auto-1014 판)이 받는 것 / 내는 것:
  보냄                                                          받음
    /ui/mode         std_msgs/String  AUTO | WAYPOINT | MANUAL     /planner/status  String JSON (mode, state, reason, auto{...}, field{...}, field_pose)
    /ui/field        std_msgs/String  JSON                          /planner/field   String JSON latched (how, size, start, gap, corners_odom, last_cmd)
                     {"cmd":"here","size":[W,L],"start":[x,y,deg]}  = 지금 로버(base_link) 자세가 경기장 start 자리 → 경기장을 그 자리에 놓음
                     {"cmd":"here",...,"band_exit":true}            = [261016] 위 + 출발 자리가 경계 금지 띠 안이어도 나가는 칸만 열기 (운영자 확인)
                     {"cmd":"band_exit"}                            = [261016] 띠 안에서 막혔을 때 나가는 칸만 열기 (운영자 확인, 띠를 나오면 끝)
                     {"cmd":"gap","xy":[x,y]}                        = 틈(페이즈2 입구, 경기장 좌표)
                     {"cmd":"grid","cols":C,"rows":R}                = [10/15] 방문점 격자 (자율 중 거부, 방문 기록 지움)
                     {"cmd":"clear"}                                 = [10/15] 경기장 풀기 → 연구실 범위 (field.how = lab)
    /ui/gap_point    geometry_msgs/PointStamped  틈 클릭 (waypoint_frame = map)
    /ui/waypoint_cmd std_msgs/String  AUTO_SKIP | AUTO_RESET

- /ui/mode 는 바꿀 때 한 번만 보낸다 (1 Hz 재발행 안 함). 버튼 상태의 기준은 경로계획이 status 로 알려 주는 mode —
  경로계획이 다시 켜져 기본 모드로 돌아가면 버튼도 꺼진 것으로 보인다 (= 멈춤 쪽, 안전).
- 정지 우선 (케일럽 10/14 '수동·정지 입력이 항상 최우선'): 자율 끄기 = MANUAL(경로계획이 빈 경로 → 정지) + 추종기 끔.
  수동 버튼을 켜면 자율을 끈다 (rover_panel). 제어창을 닫을 때 자율이면 MANUAL 을 보낸다.
"""
from __future__ import annotations

import json
import threading
import time

from geometry_msgs.msg import PointStamped
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String

STALE_S = 2.0       # /planner/status 가 이만큼 끊기면 '경로계획 없음' (2 Hz 발행)
PENDING_S = 1.5     # 모드를 보낸 뒤 status 가 따라오기를 기다리는 시간
LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

CORNERS = {'bl': '왼쪽 아래', 'br': '오른쪽 아래', 'tl': '왼쪽 위', 'tr': '오른쪽 위'}
FACING = {'+y': 90.0, '+x': 0.0, '-x': 180.0, '-y': -90.0}
FACING_NAME = {'+y': '+y (긴 변·페이즈2 쪽)', '+x': '+x (짧은 변, 오른쪽)', '-x': '-x (짧은 변, 왼쪽)', '-y': '-y (뒤쪽)'}


def start_pose(size, corner, offset, facing):
    """경기장 크기·구석·구석에서 base_link 까지 거리·보는 방향 → 경기장 좌표 [x, y, deg] (daron-nav tools/field_set.py 와 같은 식)."""
    W, L = float(size[0]), float(size[1])
    dx, dy = float(offset[0]), float(offset[1])
    x = -W / 2 + dx if corner[1] == 'l' else W / 2 - dx
    y = -L / 2 + dy if corner[0] == 'b' else L / 2 - dy
    return [round(x, 3), round(y, 3), FACING[facing]]


class AutoLink:
    """제어창 노드(GroundNode)에 붙인다. 구독 콜백은 spin 스레드, 나머지는 Qt 스레드에서 부른다.
    planner_latest = GroundNode.planner (Latest, /planner/status JSON) — 이미 구독 중인 것을 같이 씀."""

    def __init__(self, node, planner_latest, frame='map', wp_cmd_topic='/ui/waypoint_cmd'):
        self.node = node
        self.planner = planner_latest
        self.frame = frame
        self.pub_mode = node.create_publisher(String, '/ui/mode', 10)
        self.pub_field = node.create_publisher(String, '/ui/field', 10)
        self.pub_gap = node.create_publisher(PointStamped, '/ui/gap_point', 10)
        self.pub_cmd = node.create_publisher(String, wp_cmd_topic, 10)
        node.create_subscription(String, '/planner/field', self._on_field, LATCHED)
        self._lock = threading.Lock()
        self._field = {}
        self._field_t = 0.0
        self._want = None           # (보낸 모드, monotonic)
        node.get_logger().info('[261014] 자율 버튼 → /ui/mode, 경기장 놓기 → /ui/field, 틈 → /ui/gap_point, 상태 ← /planner/status·/planner/field')

    # ---------- 수신
    def _on_field(self, msg: String):
        try:
            f = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._field, self._field_t = f, time.monotonic()

    def field(self):
        with self._lock:
            return dict(self._field)

    def status(self):
        """→ (status dict, 살아 있나)."""
        st = self.planner.v if self.planner.v is not None else {}
        age = self.planner.age()
        return st, age is not None and age <= STALE_S

    def supported(self):
        """경로계획이 경기장 놓기·방문점 AUTO 를 아는 판인가 (auto-1014: status 에 field 키)."""
        st, alive = self.status()
        return alive and 'field' in st

    def mode(self):
        """경로계획 모드. 보낸 직후 PENDING_S 동안은 보낸 값."""
        st, alive = self.status()
        actual = st.get('mode') if alive else None
        w = self._want
        if w is not None:
            if w[0] != actual and time.monotonic() - w[1] < PENDING_S:
                return w[0]
            self._want = None
        return actual

    def auto_on(self):
        return self.mode() == 'AUTO'

    def field_set(self):
        """경기장을 운영자가 놓았나 (/ui/field here 를 받은 뒤 how = here)."""
        st, alive = self.status()
        return alive and (st.get('field') or {}).get('how') == 'here'

    def can_start(self):
        """→ (가능, 이유)."""
        st, alive = self.status()
        if not alive:
            return False, '경로계획 상태(/planner/status)가 안 들어옴'
        if 'field' not in st:
            return False, '경로계획이 경기장 놓기를 모르는 판 (daron-nav auto-1014 판으로 띄울 것)'
        if not self.field_set():
            return False, '경기장 위치 미지정 — 로버를 출발 자리에 두고 [여기가 출발점]'
        return True, ''

    # ---------- 송신 (Qt 스레드)
    def set_mode(self, m: str):
        self.pub_mode.publish(String(data=m))
        self._want = (m, time.monotonic())
        self.node.get_logger().info(f'경로계획 모드 → {m} (/ui/mode)')

    def place_field(self, size, start, gap=None, band_exit=False):
        c = dict(cmd='here', size=[float(size[0]), float(size[1])], start=[float(v) for v in start])
        if gap is not None:
            c['gap'] = [float(gap[0]), float(gap[1])]
        if band_exit:                                               # [261016] 출발 자리가 경계 금지 띠 안인데 운영자가 '예' — 나가는 칸만 열기
            c['band_exit'] = True
        self.pub_field.publish(String(data=json.dumps(c)))
        self.node.get_logger().info(f'경기장 놓기 → /ui/field {c}')
        return c

    def allow_band_exit(self):
        """[261016] 로버가 경계 금지 띠 안에서 막힘(START_BLOCKED) → 운영자 확인 → 띠 밖으로 나가는 칸만 열기 (띠를 나오면 경로계획이 끝냄)."""
        c = dict(cmd='band_exit')
        self.pub_field.publish(String(data=json.dumps(c)))
        self.node.get_logger().info(f'경계 금지 띠 나가기 허용 → /ui/field {c}')
        return c

    def set_grid(self, cols, rows):
        """[10/15] 방문점 격자 (열 = 4 m 변 x, 행 = 5 m 변 y). 경로계획이 AUTO 중이면 거부, 바꾸면 방문 기록 지움."""
        c = dict(cmd='grid', cols=int(cols), rows=int(rows))
        self.pub_field.publish(String(data=json.dumps(c)))
        self.node.get_logger().info(f'방문점 격자 → /ui/field {c}')
        return c

    def clear_field(self):
        """[10/15] 경기장 풀기 → 연구실 범위 (경로계획 lab_until_set 일 때만, AUTO 중 거부)."""
        c = dict(cmd='clear')
        self.pub_field.publish(String(data=json.dumps(c)))
        self.node.get_logger().info(f'경기장 풀기 → /ui/field {c}')
        return c

    def set_gap(self, x, y):
        c = dict(cmd='gap', xy=[float(x), float(y)])
        self.pub_field.publish(String(data=json.dumps(c)))
        self.node.get_logger().info(f'틈 (경기장 좌표) → /ui/field {c}')
        return c

    def gap_point(self, x, y, z=0.0):
        m = PointStamped()
        m.header.frame_id = self.frame
        m.header.stamp = self.node.get_clock().now().to_msg()
        m.point.x, m.point.y, m.point.z = float(x), float(y), float(z)
        self.pub_gap.publish(m)
        self.node.get_logger().info(f'틈 클릭 → /ui/gap_point ({x:.2f}, {y:.2f}) {self.frame}')

    def command(self, c: str):
        self.pub_cmd.publish(String(data=c))
        self.node.get_logger().info(f'자율 명령 → {c}')

    def shutdown(self):
        """창 닫을 때: 자율이면 MANUAL (경로계획이 빈 경로 → 정지)."""
        if self.auto_on():
            try:
                self.pub_mode.publish(String(data='MANUAL'))
                time.sleep(0.2)
            except Exception:  # noqa: BLE001 — 종료 중 context 내려감
                pass
            self.node.get_logger().info('제어창 종료 — 자율 끔 (MANUAL)')
