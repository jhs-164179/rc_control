#!/usr/bin/env python3
"""루프 클로저 피쳐맵 재동기화 코어 (261011) — 순수 numpy + zlib. ROS·open3d 를 쓰지 않는다.

왜: 그리드 피쳐맵은 칸 통계만 누적해 원래 관측을 버리므로, cuVSLAM 루프 클로저(PGO)로 과거 포즈가 바뀌어도
    이미 쌓인 칸을 옮길 수 없었다. 이 모듈은 키프레임마다 솎은 depth 원재료를 압축 보관하고, 보정된
    map<-odom 이력이 오면 피쳐맵을 처음부터(원래 적분 순서 그대로) 다시 쌓는다.

포즈 보정식 (키프레임마다):
    T_wc_new = M_new(t) · M_used⁻¹ · T_wc_used
      T_wc_used  적분 때 쓴 map<-depth (= M_used · T_odom_base · T_base_depth, HOLD 해제분은 M_used = I)
      M_used     적분 때 쓴 map<-odom
      M_new(t)   cuVSLAM get_all_slam_poses() 로 다시 계산한 그 시각 map<-odom (/slam/map_odom_history)
    = '적분 당시 cuVSLAM 이 지금의 보정을 알았다면 썼을 포즈'. EKF-C(odom) 쪽 포즈는 건드리지 않는다.

보정값이 항등이면 재적분 결과는 실시간 피쳐맵과 칸 단위로 같다 (tests/test_lc_resync.py).
"""
from __future__ import annotations

import copy
import zlib

import numpy as np


def points_from_sub(sub, K, stride, T_wc):
    """솎은 depth [m] (H/stride, W/stride) → 맵 좌표 점 (N,3). live_tsdf._integrate_grid 와 같은 계산."""
    v, u = np.nonzero(sub > 0)
    if len(u) == 0:
        return None
    Z = sub[v, u].astype(np.float64)
    uu, vv = u * stride, v * stride
    X = (uu - K[0, 2]) / K[0, 0] * Z
    Y = (vv - K[1, 2]) / K[1, 1] * Z
    return np.column_stack([X, Y, Z]) @ T_wc[:3, :3].T + T_wc[:3, 3]


class KeyframeStore:
    """적분한 키프레임의 원재료 (append 전용, 적분 순서 그대로).

    record = [t_ns, zlib(uint16 원본 depth 단위), shape, T_wc_used(4x4), M_used(4x4)]
    max_bytes 를 넘으면 보관을 멈추고 full=True — 그 뒤로는 재동기화를 할 수 없다(노드가 경고).
    """

    def __init__(self, max_bytes):
        self.recs = []
        self.nbytes = 0
        self.max_bytes = int(max_bytes)
        self.full = False

    def add(self, t_ns, q_u16, T_wc, M_used):
        if self.full:
            return False
        blob = zlib.compress(np.ascontiguousarray(q_u16, np.uint16).tobytes(), 1)
        if self.nbytes + len(blob) > self.max_bytes:
            self.full = True
            return False
        self.recs.append([int(t_ns), blob, tuple(q_u16.shape), np.array(T_wc, np.float64), np.array(M_used, np.float64)])
        self.nbytes += len(blob)
        return True

    def __len__(self):
        return len(self.recs)

    @staticmethod
    def depth_m(rec, depth_scale):
        """보관한 원본 단위 → m. 실시간과 같은 식(uint16 → float32 × depth_scale)이라 값이 비트 단위로 같다."""
        q = np.frombuffer(zlib.decompress(rec[1]), np.uint16).reshape(rec[2])
        return q.astype(np.float32) * depth_scale


class MapOdomHistory:
    """/slam/map_odom_history 를 stamp 순 배열로. lookup = 가장 가까운 stamp (tol_ns 안)."""

    def __init__(self, stamps_ns, mats):
        order = np.argsort(np.asarray(stamps_ns, np.int64), kind="stable")
        self.t = np.asarray(stamps_ns, np.int64)[order]
        self.M = np.asarray(mats, np.float64).reshape(-1, 4, 4)[order]

    def __len__(self):
        return len(self.t)

    def lookup(self, t_ns, tol_ns):
        if len(self.t) == 0:
            return None
        i = int(np.searchsorted(self.t, t_ns))
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(self.t):
                dt = abs(int(self.t[j]) - int(t_ns))
                if dt <= tol_ns and (best is None or dt < best[0]):
                    best = (dt, j)
        return None if best is None else self.M[best[1]]


def quats_to_mats(q):
    """(N,4) x,y,z,w → (N,3,3). scipy 없이 (pin_node·지상국은 시스템 python)."""
    q = np.asarray(q, np.float64).reshape(-1, 4)
    q = q / np.maximum(np.linalg.norm(q, axis=1, keepdims=True), 1e-12)
    x, y, z, w = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)], -1),
        np.stack([2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)], -1),
        np.stack([2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)], -1)], 1)


def history_from_path_msg(m):
    """nav_msgs/Path (/slam/map_odom_history) → MapOdomHistory. 빈 경로면 None."""
    n = len(m.poses)
    if n == 0:
        return None
    st = np.empty(n, np.int64)
    p = np.empty((n, 3))
    q = np.empty((n, 4))
    for i, ps in enumerate(m.poses):
        st[i] = ps.header.stamp.sec * 1_000_000_000 + ps.header.stamp.nanosec
        p[i] = (ps.pose.position.x, ps.pose.position.y, ps.pose.position.z)
        o = ps.pose.orientation
        q[i] = (o.x, o.y, o.z, o.w)
    M = np.tile(np.eye(4), (n, 1, 1))
    M[:, :3, :3] = quats_to_mats(q)
    M[:, :3, 3] = p
    return MapOdomHistory(st, M)


def correct_point(hist, t_ns, M_ref, p, tol_ns):
    """map 좌표 점 p 가 map<-odom = M_ref 기준으로 계산된 것일 때, 보정 이력의 그 시각 값으로 옮긴 점과 새 기준.
    (새 점, 새 M, 옮긴 변환 D = M_new·M_ref⁻¹) 또는 이력에 그 시각이 없으면 None. 핀·웨이포인트용."""
    M_new = hist.lookup(t_ns, tol_ns)
    if M_new is None:
        return None
    D = M_new @ np.linalg.inv(M_ref)
    return D[:3, :3] @ np.asarray(p, np.float64) + D[:3, 3], M_new, D


def corrected_pose(rec, hist, tol_ns):
    """(새 T_wc, 새 M) 또는 이력에 그 시각이 없으면 (None, None)."""
    M_new = hist.lookup(rec[0], tol_ns)
    if M_new is None:
        return None, None
    return M_new @ np.linalg.inv(rec[4]) @ rec[3], M_new


def pose_delta(A, B):
    """(이동 m, 회전 deg)."""
    D = np.linalg.inv(A) @ B
    c = (float(np.trace(D[:3, :3])) - 1.0) / 2.0
    return float(np.linalg.norm(A[:3, 3] - B[:3, 3])), float(np.degrees(np.arccos(max(-1.0, min(1.0, c)))))


def rebuild(base_grid, new_grid_fn, recs, hist, K, stride, depth_scale, tol_ns):
    """recs 를 순서대로 보정 포즈로 다시 적분한 새 피쳐맵.

    base_grid   이전 세션에서 불러온 피쳐맵(원재료 없음 → 그대로 바탕) 또는 None
    new_grid_fn 빈 FeatureGrid 를 만드는 함수 (실시간 노드와 같은 파라미터)
    반환: (grid, updates, stats)  updates[i] = (새 T_wc, 새 M) 또는 (None, None) = 이력에 없어 그대로 둠
    """
    g = copy.deepcopy(base_grid) if base_grid is not None else new_grid_fn()
    updates, stats = integrate_recs(g, recs, hist, K, stride, depth_scale, tol_ns)
    return g, updates, stats


def integrate_recs(g, recs, hist, K, stride, depth_scale, tol_ns):
    """recs 를 순서대로 보정 포즈로 g 에 이어 쌓는다. 반환 (updates, stats) — rebuild 와 같은 형식."""
    updates = []
    moved_m = moved_deg = 0.0
    n_miss = 0
    for rec in recs:
        T_new, M_new = corrected_pose(rec, hist, tol_ns)
        T_use = rec[3] if T_new is None else T_new
        if T_new is None:
            n_miss += 1
        else:
            dm, da = pose_delta(rec[3], T_new)
            moved_m, moved_deg = max(moved_m, dm), max(moved_deg, da)
        updates.append((T_new, M_new))
        pw = points_from_sub(KeyframeStore.depth_m(rec, depth_scale), K, stride, T_use)
        if pw is not None:
            g.integrate(pw, T_use[:3, 3], rec[0] * 1e-9)
    return updates, dict(n=len(recs), miss=n_miss, moved_max_m=moved_m, moved_max_deg=moved_deg)


def max_move(recs, hist, tol_ns):
    """재적분 전에 보정량만 미리 본다 (작으면 건너뛰기용). (최대 이동 m, 최대 회전 deg, 이력 있는 키프레임 수)"""
    mm = md = 0.0
    n = 0
    for rec in recs:
        T_new, _ = corrected_pose(rec, hist, tol_ns)
        if T_new is None:
            continue
        n += 1
        dm, da = pose_delta(rec[3], T_new)
        mm, md = max(mm, dm), max(md, da)
    return mm, md, n
