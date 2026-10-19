#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
rover_panel — 제어창 아래쪽 '로버 제어' 칸 (Qt). 작성 261010.  ROS 쪽은 gui/rover_teleop.py · gui/follower_link.py.

  [수동] 버튼 = 키보드 조종 ON/OFF
  [웨이포인트] 버튼 = 경로 추종기 ON/OFF (/path_follower/enable, 261012) — 찍은 웨이포인트로 planner 가 낸 경로를 따라감.
    planner 는 WAYPOINT 모드(live_lab.yaml 기본)라고 본다 (/ui/mode 는 안 보냄). 버튼 상태는 추종기 status 를 따르고,
    추종기 status 가 없으면 잠긴다. 추종 중에도 키보드 입력이 우선 (로버 radxa.py 가 키 입력 후 1 s 동안 수동).
  [자율] 버튼 = [261014 daron-nav auto-1014] 완전 자율 ON/OFF (gui/auto_link.py · gui/auto_panel.py).
    켜기 = 경로계획 /ui/mode AUTO + 추종기 ON + 지상국 웨이포인트 목록 일시정지. 조건: 경로계획이 auto-1014 판, 경기장 놓임, 추종기 있음, 수동 꺼짐.
    끄기 = /ui/mode MANUAL(경로계획 빈 경로 → 정지) + 추종기 OFF. **정지 우선**: [수동]을 켜면 자율은 바로 꺼짐 (자동으로 다시 안 켜짐).
    [웨이포인트]를 누르면 경로계획을 WAYPOINT 로 바꾼다 (자율에서 넘어올 때·MANUAL 이었을 때). 버튼 상태 = 경로계획 status 의 mode.
    '추종기' 칸 = 추종 상태·횡오차·남은 거리·rpm, '지령 출처' 칸 = 로버가 실제로 따르는 지령 (진단 drive_mode).
  ON 이고 제어창(또는 [상세] 창)이 활성 창일 때만 키를 가로챈다:
    W/S 구동  A/D 조향  Q/E 굴절  SPACE 전복 토글  1 중립(150°) 2 수평(180°) 0 수납  ESC = 조종 OFF (창은 안 닫음)
  가로챈 키는 다른 위젯에 안 간다 (SPACE 가 포커스 버튼 — 예: 웨이포인트 '전체 삭제' — 을 누르지 않게).
  다른 창(RViz, 파일 열기 창, 크레이터 정보 창 …)이 활성이 되면 그 순간 모든 키를 뗀 것으로 본다 → 조향 0·구동 0 바로 송신.
  키 자동반복은 무시. 키 이름은 Qt 키 코드로, 못 알아보면(한글 입력 상태 등) X11 스캔코드로 본다.

  표시: 링크(NO LINK / STALE / 서스펜션 상태), 전복, 배터리(261010), 형상 모드, 굴절각, 조향, 구동, F/B ON·RPM, 휠베이스, 구동 에러.
  배터리 배지: 링크가 살아 있을 때 잔량 % (VESC battery_level) — battery_warn_pct 미만이면 빨강, 모르면 회색 'BATT ?'.
  [상세] = console.py 창의 텔레메트리 줄 전부 (rover_teleop.detail_rows).
"""
from __future__ import annotations

import html

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import QEvent, Qt

from gui.follower_link import STATE_NAME
from gui.rover_teleop import STALE_S, STATE_MARK, battery_state, detail_rows, drive_errors, low_hz

GREEN, YELLOW, RED, GRAY, BLUE = '#2e9d4a', '#d39b00', '#d0342c', '#8a8a8a', '#2f6fd0'   # ground_station 과 같은 색
PURPLE = '#7d4cc2'   # [261014] 자율 버튼

KEY_NAMES = {Qt.Key_W: 'w', Qt.Key_A: 'a', Qt.Key_S: 's', Qt.Key_D: 'd', Qt.Key_Q: 'q', Qt.Key_E: 'e',
             Qt.Key_Space: 'space', Qt.Key_1: '1', Qt.Key_2: '2', Qt.Key_0: '0', Qt.Key_Escape: 'esc'}
# X11(xcb) 키코드 = evdev + 8. 한글 입력 상태 등으로 key() 가 영문으로 안 올 때만 쓴다 (KP1/KP2/KP0 포함)
X11_SCAN = {25: 'w', 38: 'a', 39: 's', 40: 'd', 24: 'q', 26: 'e', 65: 'space',
            10: '1', 11: '2', 19: '0', 87: '1', 88: '2', 90: '0', 9: 'esc'}
SHAPE_OF = {'1': 'neutral', '2': 'flat', '0': 'stow'}
CAPS = [('W', 'w'), ('A', 'a'), ('S', 's'), ('D', 'd'), ('Q', 'q'), ('E', 'e')]
KEY_HELP = 'W/S 구동 · A/D 조향 · Q/E 굴절각 −/+ · SPACE 전복 · 1 중립 2 수평 0 수납 · ESC 조종 끄기 · 입력 칸에 커서가 있으면 조종 키 안 먹음'


def badge(text, color):
    return f'<span style="background:{color};color:white;padding:1px 4px;border-radius:3px">&nbsp;{text}&nbsp;</span>'


def is_text_entry(w):
    """[261016] 키를 직접 받아야 하는 입력 위젯 (숫자·글자 칸). 편집 안 되는 콤보는 아님 — 거기선 조종 키가 먹는다."""
    if isinstance(w, (QtWidgets.QAbstractSpinBox, QtWidgets.QLineEdit, QtWidgets.QTextEdit, QtWidgets.QPlainTextEdit)):
        return True
    return isinstance(w, QtWidgets.QComboBox) and w.isEditable()


def key_name(ev):
    name = KEY_NAMES.get(ev.key())
    if name is None and QtGui.QGuiApplication.platformName() == 'xcb':
        name = X11_SCAN.get(ev.nativeScanCode())
    return name


class RoverKeyFilter(QtCore.QObject):
    """QApplication 전체 이벤트 필터. 조종 ON + 제어 창 활성일 때만 조종 키를 먹는다."""

    def __init__(self, panel):
        super().__init__(panel)
        self.panel = panel

    def eventFilter(self, obj, ev):  # noqa: N802 — Qt 이름
        t = ev.type()
        if t in (QEvent.WindowDeactivate, QEvent.ApplicationDeactivate):
            QtCore.QTimer.singleShot(0, self.panel.check_focus)    # 활성 창이 바뀐 뒤에 판단
            return False
        if t not in (QEvent.KeyPress, QEvent.KeyRelease, QEvent.ShortcutOverride):
            return False
        p = self.panel
        if not p.link.enabled or not p.control_window_active():
            return False
        if is_text_entry(QtWidgets.QApplication.focusWidget()):
            # [261016] 입력 칸에 포커스면 키를 칸으로 — 예전엔 숫자 0/1/2 가 형상 명령(0 = 수납)이 되고 칸은 값을 못 받았다.
            # 눌려 있던 조종 키는 뗀다 (뗌 이벤트가 칸으로 가서 키가 눌린 채 남지 않게).
            if any(p.link.keys.values()):
                p.link.release_keys()
                p.refresh()
            return False
        name = key_name(ev)
        if name is None:
            return False
        if t == QEvent.ShortcutOverride:                          # 단축키·버튼 니모닉으로 안 새게
            ev.accept()
            return True
        if ev.isAutoRepeat():
            return True
        down = t == QEvent.KeyPress
        if name in p.link.keys:
            p.link.set_key(name, down)
        elif down and name == 'space':
            p.link.toggle_inverted()
        elif down and name in SHAPE_OF:
            p.link.shape(SHAPE_OF[name])
        elif down and name == 'esc':
            p.b_manual.setChecked(False)
        p.refresh()
        return True


class RoverDetailDialog(QtWidgets.QDialog):
    def __init__(self, parent):
        super().__init__(parent)
        self.setWindowTitle('로버 상세 (console.py 와 같은 항목)')
        lay = QtWidgets.QVBoxLayout(self)
        self.lbl = QtWidgets.QLabel()
        self.lbl.setTextFormat(Qt.RichText)
        self.lbl.setTextInteractionFlags(Qt.TextSelectableByMouse)
        self.lbl.setAlignment(Qt.AlignTop | Qt.AlignLeft)
        lay.addWidget(self.lbl)
        self.resize(560, 640)

    def refresh(self, d, rows_logged, ok):
        if not ok:
            self.lbl.setText(f'<span style="color:{GRAY}">링크 없음 — /joint_states·diagnostics 를 못 받는 중</span>')
            return
        trs = ''.join(
            f'<tr><td style="color:{GRAY};padding-right:14px">{html.escape(k)}</td>'
            f'<td style="color:{YELLOW if w else "black"};font-family:monospace">{html.escape(str(v))}</td></tr>'
            for k, v, w in detail_rows(d, rows_logged))
        self.lbl.setText(f'<table cellspacing="0" cellpadding="1">{trs}</table>')


class RoverPanel(QtWidgets.QGroupBox):
    def __init__(self, link, follower=None, auto=None, on_auto=None, parent=None):
        super().__init__('로버 제어 — 수동 = 키보드 조종 (켜져 있을 때만 /rover 로 송신) · 웨이포인트 = 경로 추종기 · 자율 = 방문점 → 틈', parent)
        self.link = link
        self.auto = auto                                           # [261014] gui/auto_link.AutoLink (None 이면 자율 버튼 비활성)
        self.on_auto = on_auto                                     # [261014] f(on) — 제어창이 웨이포인트 목록을 멈추고 되돌림
        self._auto_msg = ''
        self.follower = follower                                   # [261012] gui/follower_link.FollowerLink
        lay = QtWidgets.QVBoxLayout(self)

        top = QtWidgets.QHBoxLayout()
        self.b_stop = QtWidgets.QPushButton('정지')                # [261016] 수동·웨이포인트·자율 전부 끔 (항상 누를 수 있음)
        self.b_stop.setFocusPolicy(Qt.NoFocus)
        self.b_stop.setStyleSheet(f'QPushButton {{ background:{RED}; color:white; font-weight:bold; padding:2px 14px }}')
        self.b_stop.setToolTip('정지: 수동(키보드) 끔 + 웨이포인트 추종기 끔 + 자율이면 경로계획 MANUAL.\n'
                               '다시 가려면 웨이포인트 또는 자율을 다시 누른다. 제어창이 안 들으면 tools/estop.sh')
        self.b_stop.clicked.connect(self._on_stop)
        top.addWidget(self.b_stop)
        self.b_manual = QtWidgets.QPushButton('수동')
        self.b_manual.setCheckable(True)
        self.b_manual.setFocusPolicy(Qt.NoFocus)                  # 꺼진 동안 SPACE 로 켜지지 않게
        self.b_manual.setStyleSheet(f'QPushButton:checked {{ background:{GREEN}; color:white; font-weight:bold }}')
        self.b_manual.setToolTip('켜면 키보드로 로버 조종 (제어창이 활성일 때만 키를 받음). ESC 또는 다시 누르면 끔.\n'
                                 '켜면 웨이포인트 추종·자율은 꺼진다 (수동 = 추종 정지) [261016].\n'
                                 'console.py 와 동시에 쓰지 말 것 — 끈 동안은 아무것도 안 보내므로 console.py 사용 가능')
        self.b_manual.toggled.connect(self._on_manual)
        top.addWidget(self.b_manual)
        self.b_wp = QtWidgets.QPushButton('웨이포인트')            # [261012] 경로 추종기 ON/OFF
        self.b_wp.setCheckable(True)
        self.b_wp.setFocusPolicy(Qt.NoFocus)                      # 키보드 조종 중 SPACE 로 눌리지 않게
        self.b_wp.setStyleSheet(f'QPushButton:checked {{ background:{BLUE}; color:white; font-weight:bold }}')
        self.b_wp.setToolTip('켜면 경로 추종기가 찍은 웨이포인트의 경로(/planner/path)를 따라 로버를 몬다 '
                             '(/path_follower/enable). 다시 누르면 끔.\n'
                             '수동이 켜져 있으면 못 켬 (수동을 켜면 추종도 꺼짐) [261016].\n'
                             '로버 쪽 자율 허용(console.py P 키)이 꺼져 있으면 로버가 따르지 않는다 (지령 출처 칸).')
        self.b_wp.toggled.connect(self._on_wp)
        self.b_wp.setEnabled(follower is not None)
        top.addWidget(self.b_wp)
        self.b_auto = QtWidgets.QPushButton('자율')                # [261014] 완전 자율 ON/OFF
        self.b_auto.setCheckable(True)
        self.b_auto.setFocusPolicy(Qt.NoFocus)
        self.b_auto.setStyleSheet(f'QPushButton:checked {{ background:{PURPLE}; color:white; font-weight:bold }}')
        self.b_auto.toggled.connect(self._on_auto)
        self.b_auto.setEnabled(auto is not None)
        top.addWidget(self.b_auto)
        self.b_resume = QtWidgets.QPushButton('재개')              # [261016] 링크 끊김으로 추종기가 스스로 꺼졌을 때만
        self.b_resume.setFocusPolicy(Qt.NoFocus)
        self.b_resume.setStyleSheet(f'QPushButton:enabled {{ background:{YELLOW}; color:black; font-weight:bold }}')
        self.b_resume.setToolTip('로버 링크가 길게(link_resume_max_s, 기본 3 s) 끊겨 추종기가 스스로 멈췄을 때, '
                                 '로버 주변을 확인하고 누르면 추종을 다시 켠다 (웨이포인트·자율 그대로)')
        self.b_resume.clicked.connect(self._on_resume)
        self.b_resume.setEnabled(False)
        top.addWidget(self.b_resume)
        top.addSpacing(16)
        self.caps = {}
        for label, name in CAPS:
            c = QtWidgets.QLabel(label)
            c.setAlignment(Qt.AlignCenter)
            c.setFixedSize(28, 24)
            self.caps[name] = c
            top.addWidget(c)
        top.addSpacing(10)
        self.lbl_inv = QtWidgets.QLabel()
        self.lbl_inv.setTextFormat(Qt.RichText)
        top.addWidget(self.lbl_inv)
        self.lbl_link = QtWidgets.QLabel()
        self.lbl_link.setTextFormat(Qt.RichText)
        self.lbl_link.setWordWrap(True)                            # [261012] 긴 문구가 창 폭을 밀지 않게
        top.addWidget(self.lbl_link)
        self.lbl_batt = QtWidgets.QLabel()                         # [261010] 배터리 잔량
        self.lbl_batt.setTextFormat(Qt.RichText)
        top.addWidget(self.lbl_batt)
        top.addStretch(1)
        self.lbl_help = QtWidgets.QLabel()
        self.lbl_help.setTextFormat(Qt.RichText)
        self.lbl_help.setWordWrap(True)
        top.addWidget(self.lbl_help)
        b_det = QtWidgets.QPushButton('상세')
        b_det.setFocusPolicy(Qt.NoFocus)
        b_det.clicked.connect(self._show_detail)
        top.addWidget(b_det)
        lay.addLayout(top)

        grid = QtWidgets.QGridLayout()
        self.val = {}
        names = ('형상', '굴절각', '조향 (차량)', '구동', 'F/B ON', 'F/B RPM',
                 '서스펜션 루프', '휠베이스', '추종기', '지령 출처', '로그', '경고')
        for i, k in enumerate(names):
            lab = QtWidgets.QLabel()
            lab.setTextFormat(Qt.RichText)
            lab.setWordWrap(True)                                  # [261012] 긴 값(추종기·지령 출처)이 칸 폭을 밀지 않게
            self.val[k] = lab
            grid.addWidget(lab, i // 6, i % 6)
        lay.addLayout(grid)

        self.detail = RoverDetailDialog(self)
        self.filter = RoverKeyFilter(self)
        QtWidgets.QApplication.instance().installEventFilter(self.filter)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(100)
        self.refresh()

    # ---------- 동작
    def _on_manual(self, on):
        if on and self.auto is not None and self.auto.auto_on():   # [261014] 정지 우선: 수동을 켜면 자율은 끈다
            self._stop_auto('수동 켬')
        elif on and self.follower is not None and self.follower.enabled():
            self.follower.set_enabled(False)                       # [261016] 웨이포인트 추종도 끈다 (예전엔 키를 안 누르면 계속 추종)
            self._auto_msg = '수동 켬 → 웨이포인트 추종 끔'
        self.link.set_enabled(on)
        self.refresh()

    def _on_stop(self):
        """[261016] 정지 버튼: 수동 키보드 OFF(0 지령 한 번) + 추종기 OFF + 자율이면 경로계획 MANUAL.
        웨이포인트 모드의 경로계획·웨이포인트 목록은 그대로 둔다 — 웨이포인트를 다시 누르면 이어 간다."""
        if self.link.enabled:
            self.link.set_enabled(False)                           # 키를 떼고 0 지령 한 번
        if self.auto is not None and self.auto.auto_on():
            self._stop_auto('정지 버튼')
        elif self.follower is not None:
            self.follower.set_enabled(False)
            self._auto_msg = '정지 버튼 → 추종 끔'
        self.refresh()

    def _on_resume(self):
        """[261016] 링크 끊김으로 스스로 꺼진 추종기를 다시 켠다 (경로계획 모드는 그대로)."""
        if self.follower is not None:
            self.follower.set_enabled(True)
            self._auto_msg = '재개'
        self.refresh()

    def _on_wp(self, on):
        if on and self.link.enabled:                               # [261016] 수동 = 추종 정지. 자율과 같은 규칙
            self.b_wp.blockSignals(True)
            self.b_wp.setChecked(False)
            self.b_wp.blockSignals(False)
            self._auto_msg = '수동(키보드)이 켜져 있음 — 수동을 끄고 누를 것'
            self.refresh()
            return
        if on:
            self._auto_msg = ''
        if on and self.auto is not None and self.auto.status()[1] and self.auto.mode() != 'WAYPOINT':
            was_auto = self.auto.auto_on()
            self.auto.set_mode('WAYPOINT')                         # [261014] 자율(또는 MANUAL)에서 넘어오면 경로계획도 웨이포인트로
            if was_auto and self.on_auto is not None:
                self.on_auto(False)
        if self.follower is not None:
            self.follower.set_enabled(on)
        self.refresh()

    # ---------- [261014] 자율
    def _set_auto_btn(self, on):
        self.b_auto.blockSignals(True)
        self.b_auto.setChecked(on)
        self.b_auto.blockSignals(False)

    def _on_auto(self, on):
        A, F = self.auto, self.follower
        if A is None:
            return
        if on:
            ok, why = A.can_start()
            if ok and (F is None or not F.alive()):
                ok, why = False, '추종기 노드 없음 (daron_gui follower:=true 확인)'
            if ok and self.link.enabled:
                ok, why = False, '수동(키보드)이 켜져 있음 — 수동을 끄고 누를 것 (수동 우선)'
            if not ok:
                self._set_auto_btn(False)
                self._auto_msg = why
                self.refresh()
                return
            self._auto_msg = ''
            if self.on_auto is not None:
                self.on_auto(True)                                 # 지상국 웨이포인트 목록 일시정지 (자율 중 목표를 밀어 넣지 않게)
            A.set_mode('AUTO')
            F.set_enabled(True)
        else:
            self._stop_auto('자율 끔')
        self.refresh()

    def _stop_auto(self, why):
        """자율 끄기 = 정지: 경로계획 MANUAL(빈 경로) + 추종기 OFF. 웨이포인트 목록은 자율 전 상태로."""
        if self.auto is not None:
            self.auto.set_mode('MANUAL')
        if self.follower is not None:
            self.follower.set_enabled(False)
        self._set_auto_btn(False)
        self._auto_msg = f'{why} → 정지 (MANUAL)'
        if self.on_auto is not None:
            self.on_auto(False)

    def _show_detail(self):
        self.detail.show()
        self.detail.raise_()

    def control_window_active(self):
        w = QtWidgets.QApplication.activeWindow()
        return w is not None and w in (self.window(), self.detail)

    def check_focus(self):
        """제어 창이 아닌 창이 활성이면 키를 전부 뗀 것으로 → 0 지령 바로."""
        if self.link.enabled and not self.control_window_active() and any(self.link.keys.values()):
            self.link.release_keys()
            self.refresh()

    def _tick(self):
        self.check_focus()                                         # 이벤트를 놓쳤을 때의 보험
        self.link.tick()
        self.refresh()

    def shutdown(self):
        QtWidgets.QApplication.instance().removeEventFilter(self.filter)
        if self.auto is not None:
            self.auto.shutdown()                                   # [261014] 자율이면 MANUAL
        if self.follower is not None:
            self.follower.shutdown()                               # 웨이포인트 추종 ON 이었으면 끔
        self.link.shutdown()

    # ---------- 표시
    def _set(self, k, v, color=None):
        name = f'<span style="color:{GRAY}">{k}:</span> '
        self.val[k].setText(name + (f'<span style="color:{color}">{v}</span>' if color else v))

    def refresh(self):
        L = self.link
        on = L.enabled
        if self.b_manual.isChecked() != on:
            self.b_manual.blockSignals(True)
            self.b_manual.setChecked(on)
            self.b_manual.blockSignals(False)
        for name, c in self.caps.items():
            down = on and L.keys[name]
            c.setStyleSheet(f'background:{GREEN if down else ("#3a3a40" if on else "#c8c8cc")};'
                            f'color:{"white" if down or on else "#777"};border-radius:4px;font-weight:bold')
        self.lbl_inv.setText(badge('INVERTED', YELLOW) if L.inverted else badge('NORMAL', GREEN if on else GRAY))
        active = self.control_window_active()
        self.lbl_help.setText(
            f'<span style="color:{GRAY}">{KEY_HELP}</span>' if on and active else
            badge('키 안 받음 — 제어창을 클릭', YELLOW) if on else
            f'<span style="color:{GRAY}">수동을 누르면 키보드 조종</span>')

        js, d, dmsg, age = L.snapshot()
        ok = js is not None and age <= STALE_S
        if js is None:
            self.lbl_link.setText(badge('NO LINK', RED if on else GRAY) + f' <span style="color:{GRAY}">{L.ns} 노드·ROS_DOMAIN_ID 확인</span>')
        elif age > STALE_S:
            self.lbl_link.setText(badge(f'STALE {age:.1f}s', YELLOW))
        else:
            st = d.get('state', '?')
            col = YELLOW if st == 'YIELD' else (RED if dmsg and dmsg != st else GREEN)
            self.lbl_link.setText(badge(f'{STATE_MARK.get(st, "")} {st}'.strip(), col)
                                  + (f' <span style="color:{YELLOW}">{html.escape(dmsg)}</span>' if dmsg and dmsg != st else ''))

        pct, volt, blevel = battery_state(d) if ok else (None, '', 'unknown')
        if pct is None:
            self.lbl_batt.setText(badge('BATT ?', GRAY) + (f' <span style="color:{GRAY}">{html.escape(volt)} V</span>'
                                                          if volt else ''))
        else:
            self.lbl_batt.setText(badge(f'BATT {pct:.0f}%', RED if blevel == 'low' else GREEN)
                                  + (f' <span style="color:{GRAY}">{html.escape(volt)} V</span>' if volt else ''))
        g = lambda k: html.escape(str(d.get(k, '?'))) if ok else '—'  # noqa: E731
        drivable = d.get('drivable', 'True') == 'True'
        tgt = d.get('shape_target', '')
        self._set('형상', g('shape_mode') + (f' → {html.escape(tgt)}' if ok and tgt else '')
                  + ('' if not ok or drivable else ' [잠김]'), YELLOW if ok and not drivable else None)
        self._set('굴절각', f"{g('articulation_deg')}°")
        self._set('조향 (차량)', f"{g('steer_eff_deg')} / {g('steer_limit_deg')}°")
        armed = d.get('drive_armed') == 'True'
        self._set('구동', f"{g('drive_cmd')} → {g('drive_applied')}" + ('' if not ok else ' [armed]' if armed else ' [미준비]'),
                  YELLOW if ok and not armed else None)
        ons = f"{g('drive_front_enabled')} / {g('drive_back_enabled')}"
        self._set('F/B ON', ons, YELLOW if ok and 'False' in ons else None)
        self._set('F/B RPM', f"{g('drive_front_rpm')} / {g('drive_back_rpm')}")
        self._set('서스펜션 루프', f"{g('susp_hz')} / {g('susp_hz_target')} Hz",
                  YELLOW if ok and low_hz(d.get('susp_hz'), d.get('susp_hz_target')) else None)
        self._set('휠베이스', f"{g('wheelbase_mm')} mm")
        self._refresh_follower(ok, d)
        self._set('로그', f'{L.csv.rows} 줄' if L.csv.paths else '수동 처음 켤 때 생성', RED if L.csv.error else None)
        warns = [f'{r}: {html.escape(e)}' for r, e in drive_errors(d)] if ok else []
        if L.csv.error:
            warns.append(html.escape(L.csv.error))
        self._set('경고', ' · '.join(warns) if warns else '없음', RED if warns else GRAY)
        if self.detail.isVisible():
            self.detail.refresh(d, L.csv.rows, ok)

    def _refresh_follower(self, ok, d):
        """[261012] 웨이포인트 버튼 · '추종기' 칸 · '지령 출처' 칸. ok/d = 로버 링크 상태·진단 (refresh 와 같은 값)."""
        F = self.follower
        alive = F is not None and F.alive()
        f_on = F is not None and F.enabled()
        A = self.auto
        a_on = A is not None and A.auto_on()                       # [261014] 경로계획 mode = AUTO
        wp_on = f_on and not a_on                                  # 자율 중에는 추종기가 켜져 있어도 웨이포인트 버튼은 꺼짐으로
        if self.b_wp.isChecked() != wp_on:                       # 버튼 = 추종기가 알려 준 상태
            self.b_wp.blockSignals(True)
            self.b_wp.setChecked(wp_on)
            self.b_wp.blockSignals(False)
        self.b_wp.setEnabled(alive)
        st0 = F.snapshot()[0] if alive else {}
        self.b_resume.setEnabled(alive and st0.get('stop_reason') == 'link_lost' and not f_on)
        if A is not None:
            if self.b_auto.isChecked() != a_on:
                self._set_auto_btn(a_on)
            can, why = A.can_start()
            self.b_auto.setEnabled(a_on or (can and alive))
            self.b_auto.setToolTip('자율 켜기: 경로계획 AUTO + 추종기 ON (방문점을 골라 지나간 뒤 틈으로). 끄기 = 정지(MANUAL).\n'
                                   '수동을 켜면 자율은 바로 꺼진다.' + ('' if can or a_on else f'\n지금 못 켬: {why}')
                                   + ('' if alive else '\n추종기 노드 없음'))

        mode = d.get('drive_mode', '') if ok else ''             # radxa.py 중재 결과 (manual / auto / idle)
        if F is None:
            self._set('추종기', '없음', GRAY)
        elif not alive:
            self._set('추종기', '노드 없음 (daron_gui follower:=true 확인)' + (f' · {html.escape(self._auto_msg)}' if self._auto_msg else ''), GRAY)
        else:
            st, msg, _age = F.snapshot()
            state = st.get('state', '?')
            text = STATE_NAME.get(state, state)
            col = {'TRACKING': BLUE, 'STOP_END': GREEN, 'STOP_STALE': YELLOW, 'HOLD_DEVIATION': RED}.get(state, GRAY)
            link_lost = st.get('stop_reason') == 'link_lost' and not f_on
            if link_lost:                                          # [261016] 긴 링크 끊김 → 추종기가 스스로 끔
                text += f" · 로버 링크 끊김으로 추종 멈춤 (진단 {st.get('rover_diag_age_s') or '?'} s 전) — 확인 뒤 [재개]"
                col = YELLOW
            elif state == 'IDLE':
                text += ' (경로 없음)' if st.get('enabled') == 'True' else ' (꺼짐)'
            if self._auto_msg:
                text += f' · {self._auto_msg}'
            if a_on and not f_on:
                text += ' · 자율인데 추종기 꺼짐 (로버 안 움직임)'
                col = YELLOW
            elif state == 'TRACKING':
                text += (f" · 횡오차 {st.get('e_y_m') or '?'} m · 남은 {st.get('remain_m') or '?'} m"
                         f" · {st.get('rpm') or '?'} rpm · 조향 {st.get('steer_deg') or '?'}°")
                if ok and mode and mode != 'auto':
                    text += f' · 로버 {mode} (키보드 우선 또는 자율 금지)'
                    col = YELLOW
            elif state in ('STOP_STALE', 'HOLD_DEVIATION'):
                reason = msg[len(state):].strip() if msg.startswith(state) else msg
                text += f' · {reason}' if reason else ''
            self._set('추종기', html.escape(text), col)

        on = self.link.enabled
        if mode == 'auto':
            src, col = ('추종기 (자율)', PURPLE) if a_on else ('추종기 (웨이포인트)', BLUE)
        elif mode == 'manual':
            src, col = ('제어창 키보드' if on else '키보드 (console.py 등 외부)'), GREEN
        elif mode == 'idle':
            src, col = '없음', GRAY
        else:                                                      # 링크 없음, 또는 drive_mode 없는 옛 radxa.py
            src, col = ('제어창 키보드' if on else '없음 (console.py 등 외부 가능)'), (GREEN if on else GRAY)
        if ok and d.get('auto_enable') == 'False':
            src += ' · 로버 자율 금지 (console.py P 키)'
            col = YELLOW if f_on else col
        self._set('지령 출처', src, col)
