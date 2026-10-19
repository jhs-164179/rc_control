"""[261011] D405 영상 클릭 → 웨이포인트 (위치 + 도착 방위). 계산만 — ROS·Qt 의존 없음 (tests/test_image_click.py).

흐름 (gui/image_window.py 가 화면, ground_station.py 가 ROS 를 맡는다)
  마우스 누름  → 그 순간 화면의 infra1 프레임 고정. 같은 stamp 의 depth 를 DepthBuffer 에서 찾는다
  마우스 뗌    → compute_click(): 누른 점 = 위치, 뗀 점 = 방향
    1. depth: 누른 화소 둘레 win×win 의 유효 depth 중앙값. 유효가 1/4 미만이면 search_px 안의 가장 가까운
       유효 화소 둘레로 (D405 는 패턴 프로젝터가 없어 질감 없는 바닥에 구멍이 난다)
    2. 역투영(depth camera_info K) → 영상 stamp 시각의 TF(waypoint_frame ← depth 광학 프레임)로 지도 좌표.
       영상 시각의 TF 를 쓰므로 WiFi 지연(190~300 ms, 261008) 동안 로버가 움직여도 맞다
    3. 바닥 검사: 로버(base_link, 같은 시각)보다 max_dz 넘게 높거나 낮으면 거부 (벽·배경 판)
    4. 방위: 뗀 점도 1~2 단계로 바닥점 → atan2(끝 − 시작). 3D 로 옮긴 뒤 재므로 원근 왜곡 없음.
       드래그가 min_drag_px 미만이거나 끝점 depth 가 없으면 '로버 → 목표' 방향 (yaw_source 로 구분)

infra1 과 depth 는 같은 화소다: D400 계열 depth 는 왼쪽 IR(infra1) 시점이고 두 프로파일이 같은 848x480.
크기가 다르면 비율로 맞춘다 (scale_uv).

함수 출처: decode_depth·backproject·quat_to_R·transform_point 는 daron-nav daron_planner/ros/waypoint_tools.py
(10/5 판) 를 옮김 — 레포가 달라 import 하지 않는다. depth_at 은 search_px(가장 가까운 유효 화소) 를 더함.
"""
from __future__ import annotations

import math
from collections import deque

import numpy as np

DEPTH_MIN_M = 0.07      # D405 Min-Z 7 cm [데이터시트]
DEPTH_MAX_M = 1.8       # 판정에 쓰는 유효 깊이 [9/28 초안] — 이보다 먼 곳은 지도에서 찍을 것


def decode_depth(encoding, height, width, step, data, depth_scale=0.001):
    """sensor_msgs/Image → (H, W) float32 [m]. 0 / NaN = 값 없음.
    16UC1(mono16): 정수 × depth_scale [m] (pin_node·live_tsdf·pin_gallery 와 같은 0.001). 32FC1: 이미 m."""
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if encoding in ('16UC1', 'mono16'):
        a = buf.view(np.uint16).reshape(height, step // 2)[:, :width].astype(np.float32) * depth_scale
    elif encoding == '32FC1':
        a = buf.view(np.float32).reshape(height, step // 4)[:, :width].astype(np.float32)
    else:
        raise ValueError('지원 안 하는 depth encoding: %s' % encoding)
    a[~np.isfinite(a)] = 0.0
    return a


def valid_mask(depth_m, rmin=DEPTH_MIN_M, rmax=DEPTH_MAX_M):
    return (depth_m > rmin) & (depth_m < rmax)


def depth_at(depth_m, u, v, win=5, rmin=DEPTH_MIN_M, rmax=DEPTH_MAX_M, search_px=12):
    """(u, v) 의 depth [m] → (z, (u_used, v_used)) 또는 (None, None).
    둘레 win×win 유효값 중앙값 (유효가 1/4 미만이면 실패). 실패하면 search_px 반경 안의 가장 가까운 유효 화소 둘레로 한 번 더."""
    h, w = depth_m.shape
    ui, vi = int(round(u)), int(round(v))
    if not (0 <= ui < w and 0 <= vi < h):
        return None, None
    k = win // 2

    def med(uc, vc):
        patch = depth_m[max(0, vc - k):vc + k + 1, max(0, uc - k):uc + k + 1]
        good = patch[(patch > rmin) & (patch < rmax)]
        if good.size < max(3, patch.size // 4):
            return None
        return float(np.median(good))

    z = med(ui, vi)
    if z is not None or search_px <= 0:
        return (z, (ui, vi)) if z is not None else (None, None)
    r = int(search_px)
    v0, v1, u0, u1 = max(0, vi - r), min(h, vi + r + 1), max(0, ui - r), min(w, ui + r + 1)
    ok = valid_mask(depth_m[v0:v1, u0:u1], rmin, rmax)
    if not ok.any():
        return None, None
    vv, uu = np.nonzero(ok)
    d2 = (vv + v0 - vi) ** 2 + (uu + u0 - ui) ** 2
    order = np.argsort(d2)
    for j in order[:20]:                                    # 가장 가까운 몇 개 중 둘레가 충분한 첫 화소
        if d2[j] > r * r:
            break
        uc, vc = int(uu[j] + u0), int(vv[j] + v0)
        z = med(uc, vc)
        if z is not None:
            return z, (uc, vc)
    return None, None


def backproject(u, v, z, K):
    """핀홀: K = [fx 0 cx; 0 fy cy; 0 0 1] (CameraInfo.k 9개). 반환 광학 좌표 (x, y, z)."""
    fx, cx, fy, cy = K[0], K[2], K[4], K[5]
    return np.array([(u - cx) * z / fx, (v - cy) * z / fy, z])


def quat_to_R(q):
    x, y, z, w = q
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def transform_point(t_xyz, q_xyzw, p):
    """TF (부모 ← 자식) 를 점에 적용: 부모 좌표 = R p + t."""
    return quat_to_R(q_xyzw) @ np.asarray(p, float) + np.asarray(t_xyz, float)


def yaw_of_quat(x, y, z, w):
    """→ yaw [rad]. 쿼터니언이 전부 0(미설정)이면 None."""
    if x == y == z == w == 0:
        return None
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


def scale_uv(u, v, img_wh, depth_wh):
    """infra1 화소 → depth 화소 (크기가 같으면 그대로)."""
    if tuple(img_wh) == tuple(depth_wh):
        return u, v
    return u * depth_wh[0] / float(img_wh[0]), v * depth_wh[1] / float(img_wh[1])


class DepthBuffer:
    """최근 depth 메시지 (stamp_ns, msg). 콜백 스레드가 넣고 Qt 스레드가 찾는다 (deque append·list 복사는 원자적)."""

    def __init__(self, keep=45):
        self.q = deque(maxlen=keep)

    def push(self, stamp_ns, msg):
        self.q.append((int(stamp_ns), msg))

    def clear(self):
        self.q.clear()

    def nearest(self, stamp_ns, tol_s=0.034):
        """|Δt| ≤ tol_s 인 것 중 가장 가까운 → (msg, Δt[s]) 또는 (None, 가장 가까운 Δt 또는 None)."""
        items = list(self.q)
        if not items:
            return None, None
        ts, m = min(items, key=lambda e: abs(e[0] - stamp_ns))
        dt = (ts - stamp_ns) * 1e-9
        return (m, dt) if abs(dt) <= tol_s else (None, dt)


def pixel_to_point(u, v, depth_m, K, tf_cam, **kw):
    """→ (지도 좌표 점 (3,), depth z, 쓴 화소) 또는 (None, None, None)."""
    z, uv = depth_at(depth_m, u, v, **kw)
    if z is None:
        return None, None, None
    pc = backproject(uv[0], uv[1], z, K)
    return transform_point(tf_cam[0], tf_cam[1], pc), z, uv


def compute_click(press_uv, release_uv, depth_m, K, tf_cam, base_xyz=None, max_dz=0.5, min_drag_px=15.0,
                  win=5, rmin=DEPTH_MIN_M, rmax=DEPTH_MAX_M, search_px=12, near_warn_m=0.15):
    """누른 점·뗀 점 (depth 화소) → dict.
      ok, reason(실패 사유 또는 ''), notes(경고 목록), point (x, y, z), yaw [rad] 또는 None,
      yaw_source 'drag' | 'rover_bearing' | None, depth_m, uv_used, end_point, dz
    tf_cam = (t, q): waypoint_frame ← depth 광학 프레임 (영상 stamp 시각). base_xyz: 같은 시각 base_link 위치 (없으면 None)."""
    kw = dict(win=win, rmin=rmin, rmax=rmax, search_px=search_px)
    out = dict(ok=False, reason='', notes=[], point=None, yaw=None, yaw_source=None, depth_m=None, uv_used=None,
               end_point=None, dz=None)
    p, z, uv = pixel_to_point(press_uv[0], press_uv[1], depth_m, K, tf_cam, **kw)
    if p is None:
        out['reason'] = f'누른 곳 depth 없음 ({rmin:.2f}~{rmax:.1f} m 밖이거나 질감 없는 곳) — 먼 곳은 지도에서 찍을 것'
        return out
    out.update(point=tuple(float(c) for c in p), depth_m=z, uv_used=uv)
    if z < near_warn_m:                                     # 261011 step bag: 아래 가장자리 구멍 옆에서 0.105 m 를 집음
        out['notes'].append(f'depth {z:.2f} m — 카메라 바로 앞 (차체·구멍 가장자리 잡음일 수 있음, 지도에서 위치 확인)')
    if uv != (int(round(press_uv[0])), int(round(press_uv[1]))):
        out['notes'].append(f'depth 구멍 → {math.hypot(uv[0] - press_uv[0], uv[1] - press_uv[1]):.0f} px 옆 화소 사용')
    if base_xyz is not None:
        dz = float(p[2] - base_xyz[2])
        out['dz'] = dz
        if abs(dz) > max_dz:
            out['reason'] = f'찍은 점이 로버보다 {dz:+.2f} m {"높음" if dz > 0 else "낮음"} → 바닥이 아님 (벽·배경?)'
            return out
    else:
        out['notes'].append('로버 위치(TF) 없음 → 바닥 높이 검사 생략')
    drag = math.hypot(release_uv[0] - press_uv[0], release_uv[1] - press_uv[1]) if release_uv is not None else 0.0
    if drag >= min_drag_px:
        q, _, _ = pixel_to_point(release_uv[0], release_uv[1], depth_m, K, tf_cam, **kw)
        if q is not None and math.hypot(q[0] - p[0], q[1] - p[1]) > 1e-3:
            out.update(yaw=math.atan2(q[1] - p[1], q[0] - p[0]), yaw_source='drag', end_point=tuple(float(c) for c in q))
        else:
            out['notes'].append('드래그 끝점 depth 없음 → 방향 = 로버→목표')
    if out['yaw'] is None and base_xyz is not None and math.hypot(p[0] - base_xyz[0], p[1] - base_xyz[1]) > 1e-3:
        out.update(yaw=math.atan2(p[1] - base_xyz[1], p[0] - base_xyz[0]), yaw_source='rover_bearing')
    if out['yaw'] is None:
        out['notes'].append('방향 못 정함 (드래그 없음 + 로버 위치 없음) → 방향 지정 없이 추가')
    out['ok'] = True
    return out
