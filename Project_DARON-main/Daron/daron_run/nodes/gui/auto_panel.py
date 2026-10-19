#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
auto_panel — 제어창 '완전 자율' 칸 (Qt). 작성 261014 (경로계획 채팅, daron-nav auto-1014). ROS 쪽은 gui/auto_link.py.

  1줄  경기장 놓기: 크기 W x L · 출발 구석 · 구석에서 뒷축 가운데까지 x, y · 로버가 보는 방향 → [여기가 출발점]
       = 로버를 바닥 테이프의 출발 자리에 둔 채 누른다 → 경로계획이 지금 로버 자세를 기준으로 경기장을 놓음 (/ui/field here).
       놓은 뒤 탑뷰 RViz 노란 테두리가 바닥 테이프와 겹치는지 볼 것. 자율 중에는 못 누름(경로계획도 거부).
  2줄  틈(페이즈2 입구): x, y 숫자 [틈 적용] 또는 [틈 찍기] 누른 뒤 탑뷰에서 2D Goal Pose(G) 한 번 → 그 클릭은 웨이포인트가 아니라 틈.
       [점 건너뛰기] = 지금 겨누는 방문점 건너뜀 (갇힘 정지도 풂), [자율 기록 지우기] = 지나간 점 기록을 지우고 처음부터.
  3줄  상태: 경기장(놓임/안 놓임·크기·출발·로버의 경기장 좌표), 자율(지금 목표 점·지나감·건너뜀·경과/한도·갇힘 이유).
자율 켜기·끄기 버튼은 로버 제어 칸의 [자율] (gui/rover_panel.py).
"""
from __future__ import annotations

import html
import math
import time

from PyQt5 import QtCore, QtWidgets
from PyQt5.QtCore import Qt

from gui.auto_link import CORNERS, FACING_NAME, start_pose

GREEN, YELLOW, RED, GRAY, BLUE = '#2e9d4a', '#d39b00', '#d0342c', '#8a8a8a', '#2f6fd0'   # ground_station 과 같은 색
GAP_ARM_S = 60.0     # [틈 찍기] 후 이 시간 안에 클릭이 없으면 풂
# [10/15] 방문점 격자 후보 (열 = 4 m 변 가로, 행 = 5 m 변 세로). 케일럽 10/15 결정: 3×3·3×4(기본)·4×4.
#   근거 = daron_sim/auto_compare.py 배치 시뮬 22판 (틈 도착 3×3 13 · 3×4 12 · 4×4 15 · 5×4 11 · 4×5 8, ±2판은 잡음).
#   다른 격자도 경로계획은 받음 (터미널 field_set.py --grid C R) — 그때는 이 칸에 그 값이 추가돼 보임.
GRIDS = [(3, 3, '짧게'), (3, 4, '기본'), (4, 4, '많이 보기')]


def grid_label(c_, r_):
    name = {(c, r): n for c, r, n in GRIDS}.get((c_, r_))
    return f'{c_}열×{r_}행' + (f' ({name})' if name else '')


def badge(text, color):
    return f'<span style="background:{color};color:white;padding:1px 4px;border-radius:3px">&nbsp;{text}&nbsp;</span>'


def _spin(v, lo, hi, step=0.05, dec=2, suffix=' m', width=78):
    s = QtWidgets.QDoubleSpinBox()
    s.setRange(lo, hi)
    s.setSingleStep(step)
    s.setDecimals(dec)
    s.setValue(v)
    s.setSuffix(suffix)
    s.setFixedWidth(width)
    s.setFocusPolicy(Qt.ClickFocus)                                 # 키보드 조종 중 휠·키가 새지 않게
    return s


class AutoPanel(QtWidgets.QGroupBox):
    def __init__(self, link, parent=None):
        super().__init__('완전 자율 — 경기장 놓기 · 틈(페이즈2 입구) · 상태  (켜기/끄기 = 로버 제어의 [자율])', parent)
        self.link = link
        self.gap_armed = False
        self._arm_t = 0.0
        self.msg = ''
        self.confirm = self._confirm_dialog                          # 시험에서 바꿔 끼움
        self._band_asked = False                                     # [261016] 띠 안 막힘 질문을 이번 막힘에 한 번만
        lay = QtWidgets.QVBoxLayout(self)

        r1 = QtWidgets.QHBoxLayout()
        r1.addWidget(QtWidgets.QLabel('경기장'))
        self.sp_w, self.sp_l = _spin(4.0, 1.0, 10.0, 0.1, 1), _spin(5.0, 1.0, 10.0, 0.1, 1)
        r1.addWidget(self.sp_w); r1.addWidget(QtWidgets.QLabel('×')); r1.addWidget(self.sp_l)
        r1.addSpacing(10)
        r1.addWidget(QtWidgets.QLabel('출발 구석'))
        self.cb_corner = QtWidgets.QComboBox()
        for k, v in CORNERS.items():
            self.cb_corner.addItem(v, k)
        self.cb_corner.setFocusPolicy(Qt.ClickFocus)
        r1.addWidget(self.cb_corner)
        r1.addWidget(QtWidgets.QLabel('구석→뒷축 가운데 x'))
        self.sp_dx = _spin(0.60, 0.0, 5.0)
        r1.addWidget(self.sp_dx)
        r1.addWidget(QtWidgets.QLabel('y'))
        self.sp_dy = _spin(0.43, 0.0, 5.0)
        r1.addWidget(self.sp_dy)
        r1.addWidget(QtWidgets.QLabel('보는 방향'))
        self.cb_face = QtWidgets.QComboBox()
        for k, v in FACING_NAME.items():
            self.cb_face.addItem(v, k)
        self.cb_face.setFocusPolicy(Qt.ClickFocus)
        r1.addWidget(self.cb_face)
        self.b_here = QtWidgets.QPushButton('여기가 출발점')
        self.b_here.setFocusPolicy(Qt.NoFocus)
        self.b_here.setToolTip('로버를 바닥 테이프의 출발 자리에 둔 채 누른다 → 경로계획이 지금 로버 자세를 기준으로 경기장을 놓음.\n'
                               '좌표: 경기장 가운데 원점, x = 폭(첫 값) 방향, y = 길이 방향, +y = 페이즈2 쪽 변.\n'
                               '놓은 뒤 탑뷰 RViz 노란 테두리가 바닥 테이프와 겹치는지 확인. 자율 중에는 안 됨.')
        self.b_here.clicked.connect(self._place)
        r1.addWidget(self.b_here)
        self.b_clear = QtWidgets.QPushButton('경기장 풀기')           # [10/15] 다시 연구실 범위 (경로계획 lab_until_set 일 때)
        self.b_clear.setFocusPolicy(Qt.NoFocus)
        self.b_clear.setToolTip('경기장을 풀고 연구실 범위(지금 로버 자리 둘레, 경계 없음)로 — 테이프 없는 곳에서 웨이포인트로 다닐 때.\n'
                                '자율 중엔 못 누름. 풀면 자율은 다시 [여기가 출발점] 뒤에.')
        self.b_clear.clicked.connect(self._clear)
        r1.addWidget(self.b_clear)
        r1.addStretch(1)
        lay.addLayout(r1)

        r2 = QtWidgets.QHBoxLayout()
        r2.addWidget(QtWidgets.QLabel('틈 (경기장 좌표) x'))
        self.sp_gx = _spin(-1.5, -10.0, 10.0)
        r2.addWidget(self.sp_gx)
        r2.addWidget(QtWidgets.QLabel('y'))
        self.sp_gy = _spin(2.3, -10.0, 10.0)
        r2.addWidget(self.sp_gy)
        self.b_gap = QtWidgets.QPushButton('틈 적용')
        self.b_gap.setFocusPolicy(Qt.NoFocus)
        self.b_gap.clicked.connect(self._gap_num)
        r2.addWidget(self.b_gap)
        self.b_pick = QtWidgets.QPushButton('틈 찍기 (탑뷰 G 한 번)')
        self.b_pick.setCheckable(True)
        self.b_pick.setFocusPolicy(Qt.NoFocus)
        self.b_pick.setStyleSheet(f'QPushButton:checked {{ background:{YELLOW}; color:white; font-weight:bold }}')
        self.b_pick.setToolTip('누른 뒤 탑뷰(또는 3D) RViz 에서 2D Goal Pose(G)로 한 번 찍으면 그 점이 틈이 된다 (웨이포인트로 안 들어감).\n'
                               f'{GAP_ARM_S:.0f} s 안에 안 찍으면 풀림. 다시 누르면 취소.')
        self.b_pick.toggled.connect(self._arm)
        r2.addWidget(self.b_pick)
        r2.addSpacing(16)
        r2.addWidget(QtWidgets.QLabel('방문점'))                      # [10/15] 격자 고르기
        self.cb_grid = QtWidgets.QComboBox()
        for c_, r_, _n in GRIDS:
            self.cb_grid.addItem(grid_label(c_, r_), f'{c_}x{r_}')     # 데이터는 문자열 (Qt 가 튜플을 그대로 못 돌려줌)
        self.cb_grid.setCurrentIndex(self.cb_grid.findData('3x4'))      # 기본 (경로계획 기본값과 같음, 경로계획 값이 오면 그걸로 맞춤)
        self.cb_grid.setFocusPolicy(Qt.ClickFocus)
        self.cb_grid.setToolTip('열 = 4 m 변(가로) 방향 점 수, 행 = 5 m 변(세로, 페이즈2 쪽) 방향 점 수. 자율 중엔 못 바꿈, 바꾸면 방문 기록 지움')
        r2.addWidget(self.cb_grid)
        self.b_grid = QtWidgets.QPushButton('격자 적용')
        self.b_grid.setFocusPolicy(Qt.NoFocus)
        self.b_grid.clicked.connect(self._grid)
        r2.addWidget(self.b_grid)
        r2.addSpacing(16)
        self.b_skip = QtWidgets.QPushButton('점 건너뛰기')
        self.b_skip.setFocusPolicy(Qt.NoFocus)
        self.b_skip.setToolTip('지금 겨누는 방문점을 건너뜀 (/ui/waypoint_cmd AUTO_SKIP). 갇힘(AUTO_BOXED) 정지도 풂')
        self.b_skip.clicked.connect(lambda: self._cmd('AUTO_SKIP'))
        r2.addWidget(self.b_skip)
        self.b_reset = QtWidgets.QPushButton('자율 기록 지우기')
        self.b_reset.setFocusPolicy(Qt.NoFocus)
        self.b_reset.setToolTip('지나간 점·시간 한도 기록을 지움 → 다음 자율 때 지금 자리에서 처음부터 (/ui/waypoint_cmd AUTO_RESET)')
        self.b_reset.clicked.connect(self._reset)
        r2.addWidget(self.b_reset)
        r2.addStretch(1)
        lay.addLayout(r2)

        self.lbl_field = QtWidgets.QLabel()
        self.lbl_field.setTextFormat(Qt.RichText)
        self.lbl_field.setWordWrap(True)
        lay.addWidget(self.lbl_field)
        self.lbl_auto = QtWidgets.QLabel()
        self.lbl_auto.setTextFormat(Qt.RichText)
        self.lbl_auto.setWordWrap(True)
        lay.addWidget(self.lbl_auto)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(200)
        self.refresh()

    # ---------- 값
    def start_value(self):
        return start_pose((self.sp_w.value(), self.sp_l.value()), self.cb_corner.currentData(),
                          (self.sp_dx.value(), self.sp_dy.value()), self.cb_face.currentData())

    # ---------- 동작
    def _confirm_dialog(self, text):
        r = QtWidgets.QMessageBox.question(self, '경기장 놓기', text, QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                                           QtWidgets.QMessageBox.No)
        return r == QtWidgets.QMessageBox.Yes

    def _place(self):
        size = (self.sp_w.value(), self.sp_l.value())
        st = self.start_value()
        hx, hy = size[0] / 2, size[1] / 2
        if not (-hx < st[0] < hx and -hy < st[1] < hy):
            self.msg = f'출발 자리 ({st[0]:.2f}, {st[1]:.2f}) 가 경기장 밖 — 구석에서 거리 확인'
            self.refresh()
            return
        warn = ''
        st_now, _alive = self.link.status()
        fp = st_now.get('field_pose')
        if (st_now.get('field') or {}).get('how') == 'here' and fp:  # [261015] 이미 놓은 경기장 기준으로 로버가 출발 자리에서 멀면 경고
            d = math.hypot(fp[0] - st[0], fp[1] - st[1])
            dth = abs((fp[2] - st[2] + 180.0) % 360.0 - 180.0)
            if d > 0.30 or dth > 20.0:
                warn = (f'⚠ 지금 경기장 기준으로 로버는 ({fp[0]:+.2f}, {fp[1]:+.2f}, {fp[2]:.0f}°) — 출발 자리에서 {d:.2f} m·{dth:.0f}° 떨어져 있음.\n'
                        f'   로버를 출발 자리로 옮기지 않고 누르면 경기장 전체가 그만큼 옮겨진다 (10/15 가상 로버 시험에서 이렇게 틀어짐).\n\n')
        band_exit = False
        fs = st_now.get('field') or {}
        band, mfb = fs.get('band_m'), fs.get('mid_from_base_m')
        if band is not None and mfb is not None:                     # [261016] 출발 자리(차체 가운데)가 경계 금지 띠 안이면 경고하고 묻는다
            mx, my = st[0] + mfb * math.cos(math.radians(st[2])), st[1] + mfb * math.sin(math.radians(st[2]))
            d_edge = min(hx - abs(mx), hy - abs(my))
            if d_edge < band:
                band_exit = True
                warn += (f'⚠ 이 출발 자리에서 차체 가운데가 경계에서 {d_edge:.2f} m — 경계 금지 띠({band:.2f} m) 안입니다.\n'
                         f'   그대로 진행하면 출발할 때만 띠 밖으로 나가는 경로를 허용합니다 (경계 쪽으로는 더 들어가지 않고, '
                         f'회전반경으로 나갈 수 있을 때만. 벽을 보고 있으면 못 나감 → 수동으로 빼기). 그래도 진행할까요?\n\n')
        text = warn + (f'로버 뒷축 가운데가 지금 경기장 {size[0]:.1f} x {size[1]:.1f} m 의 '
                f'{self.cb_corner.currentText()} 구석에서 x {self.sp_dx.value():.2f} m, y {self.sp_dy.value():.2f} m 들어간 곳에 있고 '
                f'{self.cb_face.currentText()} 를 보고 있다고 경기장을 놓습니다.\n'
                f'(경기장 좌표 {st[0]:+.2f}, {st[1]:+.2f}, {st[2]:.0f}°)\n\n'
                '지나간 방문점 기록은 지워집니다. 놓은 뒤 탑뷰 노란 테두리가 바닥 테이프와 겹치는지 확인하세요.')
        if not self.confirm(text):
            return
        self.link.place_field(size, st, band_exit=band_exit)        # 틈은 경로계획이 가진 값(경기장 좌표) 그대로
        self.msg = '경기장 놓기 보냄 — 아래 상태가 "놓임"으로 바뀌는지, 노란 테두리를 확인'
        self.refresh()

    def _clear(self):
        if not self.confirm('경기장을 풀고 연구실 범위(지금 로버 자리 둘레, 경계 없음)로 돌아갑니다.\n'
                            '웨이포인트는 경기장 밖에도 찍을 수 있게 되고, 자율은 다시 [여기가 출발점]을 누른 뒤에 켜집니다.'):
            return
        self.link.clear_field()
        self.msg = '경기장 풀기 보냄 — 상태 줄이 "연구실 범위"로, 탑뷰 테두리가 회색으로 바뀌는지'
        self.refresh()

    def _grid(self):
        c_, r_ = (int(v) for v in str(self.cb_grid.currentData()).split('x'))
        if not self.confirm(f'방문점 격자를 {c_}열 × {r_}행으로 바꿉니다. 지나간 점 기록은 지워집니다.'):
            return
        self.link.set_grid(c_, r_)
        self.msg = f'격자 {c_}×{r_} 보냄 — 탑뷰 흰 점이 바뀌는 건 다음 [자율] 때 (그 전엔 상태 줄의 격자 값)'
        self.refresh()

    def _gap_num(self):
        self.link.set_gap(self.sp_gx.value(), self.sp_gy.value())
        self.msg = f'틈 ({self.sp_gx.value():+.2f}, {self.sp_gy.value():+.2f}) 보냄'
        self.refresh()

    def _arm(self, on):
        self.gap_armed = bool(on)
        self._arm_t = time.monotonic()
        self.msg = '탑뷰 RViz 에서 2D Goal Pose(G)로 틈을 한 번 찍으세요' if on else ''
        self.refresh()

    def take_gap_click(self, x, y, z=0.0):
        """ground_station tick 이 RViz 클릭을 받을 때 부름. 틈 찍기 중이면 틈으로 보내고 True (웨이포인트로 안 넣음)."""
        if not self.gap_armed:
            return False
        self.link.gap_point(x, y, z)
        self.b_pick.setChecked(False)                               # → _arm(False)
        self.msg = f'틈 클릭 보냄 (map {x:.2f}, {y:.2f}) — 경기장 밖 0.5 m 넘으면 경로계획이 거부'
        return True

    def _cmd(self, c):
        self.link.command(c)
        self.msg = f'{c} 보냄'
        self.refresh()

    def _reset(self):
        if not self.confirm('지나간 방문점·시간 기록을 지웁니다. 다음 자율 때 지금 자리에서 처음부터 고릅니다.'):
            return
        self._cmd('AUTO_RESET')

    # ---------- 표시
    def refresh(self):
        L = self.link
        if self.gap_armed and time.monotonic() - self._arm_t > GAP_ARM_S:
            self.b_pick.setChecked(False)
            self.msg = '틈 찍기 시간 지남 — 다시 누르세요'
        st, alive = L.status()
        sup = L.supported()
        auto = L.auto_on()
        self.b_here.setEnabled(sup and not auto)
        self.b_clear.setEnabled(sup and not auto and (st.get('field') or {}).get('how') == 'here')
        self.b_grid.setEnabled(sup and not auto)
        for b in (self.b_gap, self.b_pick, self.b_reset):
            b.setEnabled(sup)
        self.b_skip.setEnabled(sup and st.get('auto') is not None)

        if not alive:
            self.lbl_field.setText(badge('경로계획 없음', GRAY) + f' <span style="color:{GRAY}">/planner/status 안 들어옴</span>')
            self.lbl_auto.setText(self._msg_html())
            return
        if not sup:
            self.lbl_field.setText(badge('지원 안 함', GRAY) + f' <span style="color:{GRAY}">경로계획이 경기장 놓기를 모르는 판 — '
                                   'daron-nav auto-1014 판으로 띄울 것</span>')
            self.lbl_auto.setText(self._msg_html())
            return
        bd = st.get('band') or {}                                    # [261016] 로버가 경계 금지 띠 안에서 막힘 → 한 번 묻는다
        if bd.get('in_band') and not bd.get('exit_ok') and st.get('state') == 'START_BLOCKED':
            if not self._band_asked:
                self._band_asked = True
                if self.confirm(f'로버(차체 가운데)가 경계 금지 띠 안에 있어 경로계획이 멈췄습니다 '
                                f'(띠 안쪽 끝까지 {bd.get("depth_m", 0):.2f} m).\n\n'
                                '띠 밖으로 나가는 경로를 허용할까요? (경계 쪽으로는 더 들어가지 않고, 회전반경으로 나갈 수 있을 때만. '
                                '띠를 나오면 허용은 자동으로 끝)\n아니오 = 정지 유지 (수동으로 빼기)'):
                    L.allow_band_exit()
                    self.msg = '띠 나가기 허용 보냄 — 경로계획이 나가는 길을 찾는지 확인'
        elif not bd.get('in_band'):
            self._band_asked = False
        f = st.get('field') or {}
        fp = st.get('field_pose')
        lc = (L.field().get('last_cmd') or {})
        size, start, gap = f.get('size') or ['?', '?'], f.get('start') or ['?', '?', '?'], f.get('gap') or ['?', '?']
        here = f.get('how') == 'here'
        lab = f.get('how') == 'lab'                                  # [10/15] 경로계획 lab_until_set: 놓기 전 = 연구실 범위
        txt = (badge('경기장 놓임', GREEN) if here else
               badge('경기장 안 놓임 (연구실 범위) — 자율 못 켬', YELLOW) if lab else badge('경기장 안 놓임 — 자율 못 켬', RED))
        grid = f.get('grid')
        if grid and tuple(grid) != getattr(self, '_grid_seen', None) and not self.cb_grid.hasFocus():   # 경로계획 값으로 맞춤 (바뀔 때만)
            self._grid_seen = tuple(grid)
            k = self.cb_grid.findData(f'{grid[0]}x{grid[1]}')
            if k < 0:
                self.cb_grid.addItem(grid_label(*grid), f'{grid[0]}x{grid[1]}'); k = self.cb_grid.count() - 1   # 터미널로 후보 밖 격자를 넣은 경우
            self.cb_grid.setCurrentIndex(k)
        if lab:
            txt += (f' 웨이포인트 범위 {size[0]} x {size[1]} m (SLAM 시작·풀었던 자리 둘레, 경계 없음) · 틈 ({gap[0]}, {gap[1]}) 경기장 좌표'
                    + (f' · 격자 {grid[0]}열×{grid[1]}행' if grid else ''))
        else:
            p2 = f.get('p2')                                         # [10/15] 붙인 페이즈2 (경로계획 p2_enable)
            op = (p2 or {}).get('opening')
            pw = (p2 or {}).get('pass_w')
            al = {'left': '왼쪽', 'center': '가운데', 'right': '오른쪽'}.get((p2 or {}).get('align'), 'x %s' % (p2 or {}).get('x'))
            p2txt = '' if not p2 else (f' · 페이즈2 {p2["size"][0]:g}×{p2["size"][1]:g} m 붙임({al}), 입구 x '
                                       + (f'{op[0]:+.2f}~{op[1]:+.2f}' if op else badge('없음 — 틈이 페이즈2 폭 밖', RED))
                                       + ('' if pw is None else (f' (지나갈 폭 {pw:.2f} m)' if pw >= 0.1 else ' ' + badge(f'입구 좁음 {pw:.2f} m — 틈 위치 확인', RED))))
            txt += (f' {size[0]} x {size[1]} m · 출발(뒷축) ({start[0]}, {start[1]}, {start[2]}°) · 틈 ({gap[0]}, {gap[1]})'
                    + (f' · 격자 {grid[0]}열×{grid[1]}행' if grid else '') + p2txt)
        if fp and not lab:
            txt += f' · <b>로버 지금 경기장 좌표 ({fp[0]:+.2f}, {fp[1]:+.2f}, {fp[2]:.0f}°)</b>'
        if lc and not lc.get('ok', True):
            txt += ' ' + badge({'grid': '격자 거부', 'clear': '풀기 거부', 'gap_click': '틈 거부'}.get(lc.get('cmd'), '놓기 거부'), RED) + f' {html.escape(str(lc.get("why", "")))}'
        self.lbl_field.setText(txt)
        if isinstance(gap[0], (int, float)):                         # 틈 칸 = 경로계획 값 (클릭으로 바뀐 것도 보이게, 고치는 중이면 그대로)
            for sp, v in ((self.sp_gx, gap[0]), (self.sp_gy, gap[1])):
                if not sp.hasFocus() and abs(sp.value() - v) > 1e-3:
                    sp.setValue(v)

        a = st.get('auto')
        state, reason = st.get('state', '?'), st.get('reason', '')
        if st.get('mode') == 'AUTO':
            col = RED if state == 'AUTO_BOXED' else YELLOW if state in ('NO_FIELD', 'INFEASIBLE', 'NO_ROUTE', 'MODE_CHANGED') \
                else BLUE if state == 'ARRIVED' else GREEN if state == 'OK' else YELLOW
            head = badge('자율 ' + state, col)
        else:
            head = badge(f'자율 꺼짐 (모드 {st.get("mode", "?")})', GRAY)
        if a:
            tg = a.get('target')
            tgt = '틈' if tg == 'gap' else (f'점 {tg}' if tg else '—')
            el, lim = a.get('elapsed_s'), a.get('time_limit_s')
            head += (f' 목표 {tgt} · 지나감 {a.get("visited")}/{a.get("n")} · 건너뜀 {a.get("skipped")}'
                     + (f' · {el:.0f}/{lim:.0f} s' if el is not None and lim else '')
                     + (' · 순서 ' + '-'.join(str(i) for i in a.get('order', [])) if a.get('order') else ''))
            sk = a.get('skipped_why') or {}
            if sk:
                head += '<br><small>건너뜀: ' + html.escape(', '.join(f'{k}: {v}' for k, v in sk.items())) + '</small>'
        if st.get('mode') == 'AUTO' and reason:
            head += f'<br><small>{html.escape(reason)}</small>'
        self.lbl_auto.setText(head + self._msg_html())

    def _msg_html(self):
        return f'<br><span style="color:{BLUE}">{html.escape(self.msg)}</span>' if self.msg else ''
