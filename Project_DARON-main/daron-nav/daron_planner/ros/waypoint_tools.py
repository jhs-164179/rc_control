"""웨이포인트 클릭 변환 도구 (261004) — ROS 없이 시험 가능한 부분.

화면 클릭 픽셀 (u, v) -> 그 픽셀 depth -> 카메라 광학 좌표 3D 점 -> (TF) -> odom 점.
광학 좌표(REP-103 optical): x 오른쪽, y 아래, z 앞.
"""
import math

import numpy as np


def decode_depth(encoding, height, width, step, data, depth_scale=0.001):
    """sensor_msgs/Image -> (H, W) float32 [m]. 0 / NaN = 값 없음.
    16UC1(mono16): 정수 x depth_scale [m] (realsense-ros 기본 mm 단위로 알려져 있음 [추정, 우분투에서 확인]).
    32FC1: 이미 m."""
    buf = np.frombuffer(bytes(data), dtype=np.uint8)
    if encoding in ('16UC1', 'mono16'):
        a = buf.view(np.uint16).reshape(height, step // 2)[:, :width].astype(np.float32) * depth_scale
    elif encoding == '32FC1':
        a = buf.view(np.float32).reshape(height, step // 4)[:, :width].astype(np.float32)
    else:
        raise ValueError('지원 안 하는 depth encoding: %s' % encoding)
    a[~np.isfinite(a)] = 0.0
    return a


def depth_at(depth_m, u, v, win=5, rmin=0.07, rmax=1.8):
    """(u, v) 둘레 win x win 안 유효 depth 의 중앙값 [m]. 없으면 None.
    rmin·rmax: D405 Min-Z 7 cm [데이터시트], 판정에 쓰는 유효 깊이 1.8 m [9/28 초안]."""
    h, w = depth_m.shape
    ui, vi = int(round(u)), int(round(v))
    if not (0 <= ui < w and 0 <= vi < h):
        return None
    k = win // 2
    patch = depth_m[max(0, vi - k):vi + k + 1, max(0, ui - k):ui + k + 1]
    good = patch[(patch > rmin) & (patch < rmax)]
    if good.size < max(3, patch.size // 4):
        return None
    return float(np.median(good))


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
    """TF (부모 <- 자식) 변환을 점에 적용: 부모 좌표 = R p + t."""
    return quat_to_R(q_xyzw) @ np.asarray(p, float) + np.asarray(t_xyz, float)


def click_to_odom(u, v, depth_m, K, tf_t, tf_q, win=5, rmin=0.07, rmax=1.8):
    """반환 (odom 점 (3,), None) 또는 (None, 이유 문자열)."""
    z = depth_at(depth_m, u, v, win, rmin, rmax)
    if z is None:
        return None, '클릭 위치 depth 없음(너무 멀거나 가깝거나 하늘)'
    pc = backproject(u, v, z, K)
    return transform_point(tf_t, tf_q, pc), None


# ------------------------------------------------------------------ 지도 점군 -> 바닥 높이 (261007, RViz 3D 지도 위에 웨이포인트·경로 그리기)
_PF_FMT = {1: 'i1', 2: 'u1', 3: 'i2', 4: 'u2', 5: 'i4', 6: 'u4', 7: 'f4', 8: 'f8'}   # sensor_msgs/PointField datatype


def cloud_xyz(fields, point_step, data, width, height, is_bigendian=False):
    """sensor_msgs/PointCloud2 의 x, y, z 만 (N, 3) float64. 유한하지 않은 점은 뺌.
    fields: name·offset·datatype 를 가진 것들. data: bytes / bytearray / array('B') / list."""
    off = {f.name: (int(f.offset), int(f.datatype)) for f in fields}
    if not all(k in off for k in ('x', 'y', 'z')):
        return np.zeros((0, 3))
    n = int(width) * int(height)
    try:
        buf = np.frombuffer(memoryview(data), dtype=np.uint8)
    except TypeError:
        buf = np.asarray(data, dtype=np.uint8)
    n = min(n, buf.size // max(1, int(point_step)))
    if n <= 0:
        return np.zeros((0, 3))
    e = '>' if is_bigendian else '<'
    dt = np.dtype({'names': ['x', 'y', 'z'], 'formats': [e + _PF_FMT[off[k][1]] for k in 'xyz'],
                   'offsets': [off[k][0] for k in 'xyz'], 'itemsize': int(point_step)})
    a = buf[:n * int(point_step)].view(dt)
    P = np.stack([a['x'], a['y'], a['z']], axis=1).astype(float)
    return P[np.isfinite(P).all(axis=1)]


class HeightGrid:
    """점군(odom) -> res 칸마다 높이 중앙값. 웨이포인트·경로를 3D 지도 위(바닥 높이)에 그리는 용도.
    중앙값: 칸 안 점 대부분이 바닥이면 바닥, 벽 옆 칸은 벽 점이 섞여 조금 높게 나올 수 있음."""

    def __init__(self, pts, res=0.05, max_extent_m=15.0, max_pts=150000):
        P = np.asarray(pts, float).reshape(-1, 3)
        P = P[np.isfinite(P).all(axis=1)]
        if len(P) > max_pts:                                       # 일정 간격으로 솎음 (20만 점 약 80 ms → 15만 점 이하)
            P = P[::int(math.ceil(len(P) / max_pts))]
        self.res = float(res)
        self.n_pts = len(P)
        if len(P) == 0:
            self.z, self.o = np.full((1, 1), np.nan), (0.0, 0.0)
            return
        c = np.median(P[:, :2], axis=0)                            # 멀리 튄 점 때문에 칸 배열이 커지지 않게
        P = P[(np.abs(P[:, 0] - c[0]) <= max_extent_m) & (np.abs(P[:, 1] - c[1]) <= max_extent_m)]
        x0, y0 = float(P[:, 0].min()), float(P[:, 1].min())
        ix = ((P[:, 0] - x0) / res).astype(np.int64)
        iy = ((P[:, 1] - y0) / res).astype(np.int64)
        nx, ny = int(ix.max()) + 1, int(iy.max()) + 1
        key = iy * nx + ix
        order = np.lexsort((P[:, 2], key))
        k, z = key[order], P[order, 2]
        starts = np.flatnonzero(np.r_[True, k[1:] != k[:-1]])
        counts = np.diff(np.r_[starts, len(k)])
        Z = np.full(nx * ny, np.nan)
        Z[k[starts]] = z[starts + (counts - 1) // 2]
        self.z, self.o = Z.reshape(ny, nx), (x0, y0)

    def lookup(self, x, y, r_max=0.15):
        """(x, y) 칸 높이. 비었으면 r_max 안 가장 가까운 칸. 없으면 nan."""
        ny, nx = self.z.shape
        cx = int(math.floor((x - self.o[0]) / self.res)); cy = int(math.floor((y - self.o[1]) / self.res))
        if 0 <= cy < ny and 0 <= cx < nx and np.isfinite(self.z[cy, cx]):
            return float(self.z[cy, cx])
        k = int(math.ceil(r_max / self.res))
        y0, y1, x0, x1 = max(0, cy - k), min(ny, cy + k + 1), max(0, cx - k), min(nx, cx + k + 1)
        if y0 >= y1 or x0 >= x1:
            return float('nan')
        W = self.z[y0:y1, x0:x1]
        ok = np.isfinite(W)
        if not ok.any():
            return float('nan')
        jj, ii = np.nonzero(ok)
        d = np.hypot(jj + y0 - cy, ii + x0 - cx)
        j = int(np.argmin(d))
        if d[j] * self.res > r_max + 1e-9:
            return float('nan')
        return float(W[jj[j], ii[j]])
