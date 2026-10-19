"""[261011] 영상 클릭 웨이포인트 창 — 제어창 '영상에서 찍기' 버튼으로 띄우는 별도 창 (PyQt5).

  infra1 실시간 (~10 Hz 갱신). 'depth 있는 곳 표시' = 찍을 수 있는 화소(0.07~1.8 m)를 초록으로 칠함
  마우스 왼쪽 누름  → 화면 프레임 고정 (같은 stamp 의 depth·TF 로 계산하려고)
  드래그           → 누른 점에서 화살표 (도착 방향)
  뗌               → 계산 (gui/image_click.py) → 성공이면 제어창 목록에 추가 (source=image), 결과를 1.5 s 보여 주고 실시간으로
  오른쪽 클릭 / Esc → 취소

창이 열려 있는 동안만 노드가 영상·depth 를 역직렬화한다 (GroundNode.set_image_active) — 닫아 두면 부하 없음.
계산·ROS 는 GroundNode.image_click(), 목록 추가는 MainWindow 콜백 (on_result).
"""
from __future__ import annotations

import math
import time

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt

from gui.image_click import DEPTH_MAX_M, DEPTH_MIN_M, decode_depth, valid_mask

GREEN, RED, GRAY, YELLOW = '#2e9d4a', '#d0342c', '#8a8a8a', '#d39b00'
HOLD_S = 1.5            # 결과를 보여 주는 시간 (그동안 고정 프레임)
OVERLAY_PERIOD_S = 0.3  # depth 표시 갱신 주기


def image_to_qimage(m):
    """sensor_msgs/Image → QImage (사본). mono8 / mono16 / rgb8 / bgr8. 못 하면 None."""
    w, h, step = m.width, m.height, m.step
    buf = bytes(m.data)
    if m.encoding == 'mono8':
        return QtGui.QImage(buf, w, h, step, QtGui.QImage.Format_Grayscale8).copy()
    if m.encoding in ('mono16', '16UC1'):
        a = np.frombuffer(buf, np.uint16).reshape(h, step // 2)[:, :w]
        a8 = np.ascontiguousarray((a >> 8).astype(np.uint8))
        return QtGui.QImage(a8.data, w, h, w, QtGui.QImage.Format_Grayscale8).copy()
    if m.encoding in ('rgb8', 'bgr8'):
        a = np.frombuffer(buf, np.uint8).reshape(h, step)[:, :3 * w].reshape(h, w, 3)
        if m.encoding == 'bgr8':
            a = a[..., ::-1]
        a = np.ascontiguousarray(a)
        return QtGui.QImage(a.data, w, h, 3 * w, QtGui.QImage.Format_RGB888).copy()
    return None


def stamp_ns(m):
    return m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec


class Canvas(QtWidgets.QWidget):
    """영상 + 겹쳐 그리기. 화면 좌표 ↔ 영상 화소 변환 (비율 유지 맞춤)."""

    def __init__(self, owner):
        super().__init__()
        self.owner = owner
        self.qimg = None
        self.overlay = None
        self.setMinimumSize(424, 240)
        self.setMouseTracking(False)
        self.setFocusPolicy(Qt.StrongFocus)
        self.setCursor(Qt.CrossCursor)

    def _rect(self):
        if self.qimg is None:
            return None
        iw, ih = self.qimg.width(), self.qimg.height()
        s = min(self.width() / iw, self.height() / ih)
        w, h = iw * s, ih * s
        return QtCore.QRectF((self.width() - w) / 2, (self.height() - h) / 2, w, h), s

    def to_img(self, pos):
        r = self._rect()
        if r is None:
            return None
        rect, s = r
        u, v = (pos.x() - rect.x()) / s, (pos.y() - rect.y()) / s
        if not (0 <= u < self.qimg.width() and 0 <= v < self.qimg.height()):
            return None
        return u, v

    def to_widget(self, uv):
        rect, s = self._rect()
        return QtCore.QPointF(rect.x() + uv[0] * s, rect.y() + uv[1] * s)

    def paintEvent(self, _e):
        p = QtGui.QPainter(self)
        p.fillRect(self.rect(), QtGui.QColor('#111'))
        r = self._rect()
        if r is None:
            p.setPen(QtGui.QColor('#ddd'))
            p.drawText(self.rect(), Qt.AlignCenter, '영상 없음 — infra1 토픽 확인')
            return
        rect, _ = r
        p.drawImage(rect, self.qimg)
        if self.overlay is not None and self.owner.chk_overlay.isChecked():
            p.drawImage(rect, self.overlay)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        self.owner.paint_marks(p, self)

    # 마우스 → 창
    def mousePressEvent(self, e):
        if e.button() == Qt.RightButton:
            self.owner.cancel('취소 (오른쪽 클릭)')
            return
        if e.button() == Qt.LeftButton:
            uv = self.to_img(e.pos())
            if uv is not None:
                self.owner.press(uv)

    def mouseMoveEvent(self, e):
        uv = self.to_img(e.pos())
        if uv is not None:
            self.owner.drag(uv)

    def mouseReleaseEvent(self, e):
        if e.button() == Qt.LeftButton:
            self.owner.release(self.to_img(e.pos()))

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_Escape:
            self.owner.cancel('취소 (Esc)')
        else:
            super().keyPressEvent(e)


class ImageClickWindow(QtWidgets.QWidget):
    def __init__(self, parent, node, on_result, min_drag_px=15.0):
        super().__init__(parent, Qt.Window)
        self.node, self.on_result, self.min_drag_px = node, on_result, min_drag_px
        self.setWindowTitle('영상에서 웨이포인트 찍기 — D405 infra1')
        self.resize(900, 600)
        lay = QtWidgets.QVBoxLayout(self)
        hint = QtWidgets.QLabel(
            '<b>누르고 드래그</b> = 위치 + 도착 방향 (화살표 방향) &nbsp;·&nbsp; <b>클릭만</b> = 방향은 로버→목표 '
            f'&nbsp;·&nbsp; 오른쪽 클릭/Esc = 취소<br><span style="color:{GRAY}">바닥만, 거리 {DEPTH_MIN_M:.2f}~{DEPTH_MAX_M:.1f} m '
            '안에서만 찍힘 (먼 곳은 탑뷰/3D 지도에서). 질감 없는 바닥·그늘은 depth 구멍으로 거부될 수 있음</span>')
        hint.setTextFormat(Qt.RichText)
        hint.setWordWrap(True)
        lay.addWidget(hint)
        self.canvas = Canvas(self)
        lay.addWidget(self.canvas, 1)
        row = QtWidgets.QHBoxLayout()
        self.chk_overlay = QtWidgets.QCheckBox('depth 있는 곳 표시 (찍을 수 있는 곳)')
        self.chk_overlay.setChecked(True)
        self.chk_overlay.toggled.connect(lambda _on: self.canvas.update())
        row.addWidget(self.chk_overlay)
        row.addStretch(1)
        self.lbl_src = QtWidgets.QLabel()
        self.lbl_src.setStyleSheet(f'color:{GRAY};')
        row.addWidget(self.lbl_src)
        lay.addLayout(row)
        self.lbl = QtWidgets.QLabel()
        self.lbl.setTextFormat(Qt.RichText)
        self.lbl.setWordWrap(True)
        lay.addWidget(self.lbl)

        self.live = None            # 마지막으로 그린 실시간 프레임 (msg)
        self.frozen = None          # 누른 동안·결과 보여 주는 동안 고정 프레임 (msg)
        self.press_uv = self.cur_uv = None
        self.result = None          # 마지막 결과 dict (+ press_uv, release_uv)
        self.hold_until = 0.0
        self._ov_t = 0.0
        self._ov_stamp = None
        self._shown_stamp = None
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)

    # ---------------- 열기·닫기 ----------------
    def showEvent(self, e):
        self.node.set_image_active(True)
        self.timer.start(100)
        super().showEvent(e)

    def closeEvent(self, e):
        self.timer.stop()
        self.node.set_image_active(False)
        self.frozen, self.press_uv, self.result = None, None, None
        super().closeEvent(e)

    # ---------------- 주기 ----------------
    def refresh(self):
        n = self.node
        now = time.monotonic()
        if self.frozen is not None and self.press_uv is None and now >= self.hold_until:
            self.frozen, self.result = None, None              # 결과 보여 주기 끝 → 실시간
        m = self.frozen if self.frozen is not None else n.img_latest.v
        age = n.img_latest.age()
        self.lbl_src.setText(f'{n.image_topic}  ' + ('없음' if age is None else f'{age:.1f} s 전')
                             + f' · depth 버퍼 {len(n.depth_buf.q)}')
        if m is None:
            self.canvas.qimg = None
            self.canvas.update()
            return
        st = stamp_ns(m)
        if st != self._shown_stamp:
            self._shown_stamp = st
            self.canvas.qimg = image_to_qimage(m)
            if self.canvas.qimg is None:
                self.lbl.setText(f'<span style="color:{RED}">영상 encoding {m.encoding!r} 표시 못 함</span>')
            self.live = m
        if self.chk_overlay.isChecked() and (now - self._ov_t >= OVERLAY_PERIOD_S or self.frozen is not None) \
                and self._ov_stamp != st:
            self._ov_t = now
            self._ov_stamp = st
            self.canvas.overlay = self._make_overlay(m)
        self.canvas.update()

    def _make_overlay(self, img):
        dm, _ = self.node.depth_buf.nearest(stamp_ns(img), tol_s=0.1)
        if dm is None:
            return None
        try:
            d = decode_depth(dm.encoding, dm.height, dm.width, dm.step, dm.data, self.node.depth_scale)
        except ValueError:
            return None
        ok = valid_mask(d)
        a = np.zeros(ok.shape + (4,), np.uint8)
        a[ok] = (40, 220, 60, 70)
        a = np.ascontiguousarray(a)
        q = QtGui.QImage(a.data, a.shape[1], a.shape[0], 4 * a.shape[1], QtGui.QImage.Format_RGBA8888).copy()
        return q

    # ---------------- 마우스 ----------------
    def press(self, uv):
        if self.live is None:
            return
        self.frozen = self.live                                  # 이 프레임의 stamp 로 depth·TF 를 찾는다
        self.press_uv = self.cur_uv = uv
        self.result = None
        self.lbl.setText('드래그해서 방향 지정 → 놓으면 추가')

    def drag(self, uv):
        if self.press_uv is not None:
            self.cur_uv = uv
            self.canvas.update()

    def release(self, uv):
        if self.press_uv is None:
            return
        if uv is None:
            uv = self.cur_uv
        press = self.press_uv
        drag_px = math.hypot(uv[0] - press[0], uv[1] - press[1]) if uv else 0.0
        res = self.node.image_click(self.frozen, press, uv if drag_px >= self.min_drag_px else None)
        res.update(press_uv=press, release_uv=uv, drag_px=drag_px)
        self.result = res
        self.press_uv = None
        self.hold_until = time.monotonic() + HOLD_S
        if res['ok']:
            it = self.on_result(res)
            yaw = '방향 없음' if res['yaw'] is None else \
                f'방위 {math.degrees(res["yaw"]):+.0f}° ({"드래그" if res["yaw_source"] == "drag" else "로버→목표"})'
            x, y, _ = res['point']
            txt = (f'<span style="color:{GREEN}"><b>추가</b></span> #{len(self.node_items())} '
                   f'(id {it["id"]}) ({x:+.2f}, {y:+.2f}) {yaw} · depth {res["depth_m"]:.2f} m')
        else:
            txt = f'<span style="color:{RED}"><b>거부</b></span> {res["reason"]}'
        if res.get('notes'):
            txt += ''.join(f'<br><span style="color:{YELLOW}">· {w}</span>' for w in res['notes'])
        self.lbl.setText(txt)
        self.canvas.update()

    def node_items(self):
        return self.parent().seq.items if self.parent() is not None else []

    def cancel(self, why):
        self.press_uv, self.result, self.frozen = None, None, None
        self.lbl.setText(why)
        self.canvas.update()

    # ---------------- 겹쳐 그리기 ----------------
    def paint_marks(self, p, cv):
        if self.press_uv is not None:
            self._arrow(p, cv, self.press_uv, self.cur_uv, QtGui.QColor('#ffd400'))
        elif self.result is not None:
            r = self.result
            col = QtGui.QColor(GREEN if r['ok'] else RED)
            end = r['release_uv'] if r.get('yaw_source') == 'drag' else None
            self._arrow(p, cv, r['press_uv'], end, col)
            if r.get('uv_used') and r['ok']:
                u, v = r['uv_used']
                c = cv.to_widget((u, v))
                p.setPen(QtGui.QPen(col, 1))
                p.drawLine(c + QtCore.QPointF(-6, 0), c + QtCore.QPointF(6, 0))
                p.drawLine(c + QtCore.QPointF(0, -6), c + QtCore.QPointF(0, 6))

    @staticmethod
    def _arrow(p, cv, a_uv, b_uv, col):
        a = cv.to_widget(a_uv)
        p.setPen(QtGui.QPen(col, 2))
        p.setBrush(Qt.NoBrush)
        p.drawEllipse(a, 7, 7)
        if b_uv is None:
            return
        b = cv.to_widget(b_uv)
        dx, dy = b.x() - a.x(), b.y() - a.y()
        L = math.hypot(dx, dy)
        if L < 3:
            return
        p.drawLine(a, b)
        ux, uy = dx / L, dy / L
        for s in (+1, -1):                                     # 화살촉
            hx = b.x() - 12 * ux + s * 6 * -uy
            hy = b.y() - 12 * uy + s * 6 * ux
            p.drawLine(b, QtCore.QPointF(hx, hy))
