"""경로 점에 높이(z)와 지면 자세(pitch·roll)를 채운다 (261007) — numpy 만, ROS 불필요.

경로 탐색은 그대로 평면(x, y, θ). 계획이 끝난 경로의 점마다 지면 높이 층에서
둘레 r_fit 안 칸으로 평면 z = a + b·x + c·y 를 맞춰
  z      = 그 점의 지면 높이 + 기준점 높이(z_offset, 지면 위로 떠 있는 높이)
  자세   = 진행 방향(yaw)을 지면에 올린 방향이 x 축, 지면 법선이 z 축 (경사 오르막 = pitch 음수, REP-103)
을 낸다.

높이 층(HeightLayer): 경기장 지도(ArenaMap)와 같은 칸. 판정 노드의 /traversability/cloud
(판정 격자 칸 중심의 지면 3D 점, odom)를 받을 때마다 칸 평균으로 덮어 씀(최근 값 우선, 비용 누적과 같은 규칙).
불가 칸(상자 윗면 등)은 맞춤에서 뺀다 — 경로 옆 장애물 높이가 경로 z 에 섞이지 않게.
"""
import math

import numpy as np


class HeightLayer:
    def __init__(self, amap):
        self.amap = amap
        self.z = np.full((amap.ny, amap.nx), np.nan)
        self.stamp = np.full((amap.ny, amap.nx), -np.inf)
        self.n_updates = 0

    def carry_from(self, old):
        """[auto-1014] 경기장을 다시 놓을 때 옛 높이 칸을 옮겨 옴 (odom 기준 가장 가까운 칸)."""
        a = self.amap
        r, c, ok = old.cells(np.stack([a.XO.ravel(), a.YO.ravel()], -1))
        z, st = self.z.reshape(-1), self.stamp.reshape(-1)
        z[ok] = old.z[r[ok], c[ok]]; st[ok] = old.stamp[r[ok], c[ok]]
        self.n_updates = old.n_updates

    def cells(self, xy_odom):
        """odom (N,2) -> (행, 열, 경기장 안 여부)."""
        f = self.amap.odom_to_field(np.asarray(xy_odom, float).reshape(-1, 2))
        c = np.floor((f[:, 0] - self.amap.o_field[0]) / self.amap.res).astype(int)
        r = np.floor((f[:, 1] - self.amap.o_field[1]) / self.amap.res).astype(int)
        ok = (r >= 0) & (r < self.amap.ny) & (c >= 0) & (c < self.amap.nx)
        return r, c, ok

    def integrate(self, pts_odom, stamp=0.0):
        """지면 점 (N,3) odom. 칸마다 평균해서 덮어 씀. 반환: 갱신한 칸 수."""
        p = np.asarray(pts_odom, float).reshape(-1, 3)
        p = p[np.isfinite(p).all(1)]
        if not len(p):
            return 0
        r, c, ok = self.cells(p[:, :2])
        if not ok.any():
            return 0
        k = r[ok] * self.amap.nx + c[ok]
        u, inv = np.unique(k, return_inverse=True)
        zs = np.bincount(inv, weights=p[ok, 2]) / np.bincount(inv)
        ur, uc = u // self.amap.nx, u % self.amap.nx
        self.z[ur, uc] = zs
        self.stamp[ur, uc] = stamp
        self.n_updates += 1
        return int(len(u))

    def ground_at(self, xy, r_fit=0.15, min_n=10, exclude=None, max_shift=0.35):
        """(x, y) 둘레 r_fit 안 칸으로 평면 맞춤. 반환 (z, b, c) 또는 None (칸 부족).
        exclude: (ny, nx) bool, 맞춤에서 뺄 칸 (불가 칸).
        max_shift: 쓴 칸들의 무게중심이 (x, y) 에서 r_fit x 이 값보다 멀면 None (반원이면 0.42)."""
        a = self.amap
        f = a.odom_to_field(np.asarray(xy, float).reshape(1, 2))[0]
        ci = (f[0] - a.o_field[0]) / a.res - 0.5
        ri = (f[1] - a.o_field[1]) / a.res - 0.5
        k = int(math.ceil(r_fit / a.res))
        r0, r1 = max(0, int(round(ri)) - k), min(a.ny, int(round(ri)) + k + 1)
        c0, c1 = max(0, int(round(ci)) - k), min(a.nx, int(round(ci)) + k + 1)
        if r0 >= r1 or c0 >= c1:
            return None
        Z = self.z[r0:r1, c0:c1]
        X, Y = a.XO[r0:r1, c0:c1], a.YO[r0:r1, c0:c1]
        m = np.isfinite(Z) & (np.hypot(X - xy[0], Y - xy[1]) <= r_fit)
        if exclude is not None:
            m &= ~exclude[r0:r1, c0:c1]
        if m.sum() < min_n:
            return None
        dx, dy, z = X[m] - xy[0], Y[m] - xy[1], Z[m]
        if math.hypot(dx.mean(), dy.mean()) > max_shift * r_fit:     # 칸이 한쪽에만 있음 = 관측 가장자리: 표면 끝이 말려 기울기가 틀림 -> 모름
            return None                                               # (10/7 재생: 가장자리 반원 맞춤 16°, 평지인데)
        A = np.stack([np.ones_like(dx), dx, dy], 1)
        if np.linalg.matrix_rank(A) < 3:                          # 한 줄로만 늘어선 칸: 기울기 못 정함 -> 평평하다고 봄
            return float(np.median(z)), 0.0, 0.0
        sol, *_ = np.linalg.lstsq(A, z, rcond=None)
        res = z - A @ sol
        keep = np.abs(res) <= max(0.03, 2.5 * np.median(np.abs(res)))   # 튀는 칸(턱 모서리 등) 한 번 걸러 다시 맞춤
        if keep.sum() >= min_n and not keep.all() and np.linalg.matrix_rank(A[keep]) == 3:
            sol, *_ = np.linalg.lstsq(A[keep], z[keep], rcond=None)
        return float(sol[0]), float(sol[1]), float(sol[2])


def surface_quat(yaw, b, c):
    """지면 기울기 dz/dx = b, dz/dy = c 위에서 yaw 방향을 바라보는 자세 (x, y, z, w).
    x 축 = 진행 방향을 지면에 올린 것, z 축 = 지면 법선."""
    t = np.array([math.cos(yaw), math.sin(yaw), b * math.cos(yaw) + c * math.sin(yaw)])
    t /= np.linalg.norm(t)
    n = np.array([-b, -c, 1.0])
    n /= np.linalg.norm(n)
    y = np.cross(n, t)
    y /= np.linalg.norm(y)
    x = np.cross(y, n)
    R = np.stack([x, y, n], 1)
    return R_to_quat(R)


def R_to_quat(R):
    tr = R[0, 0] + R[1, 1] + R[2, 2]
    if tr > 0:
        s = math.sqrt(tr + 1.0) * 2
        return ((R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s, 0.25 * s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    if i == 0:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        return (0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s, (R[2, 1] - R[1, 2]) / s)
    if i == 1:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        return ((R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s, (R[0, 2] - R[2, 0]) / s)
    s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
    return ((R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s, (R[1, 0] - R[0, 1]) / s)


def quat_pitch_roll(q):
    """(pitch, roll) [rad], REP-103 (pitch 양수 = 코가 아래)."""
    x, y, z, w = q
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return pitch, roll


def fill_path(layer, pts, z_offset, fallback_z, r_fit=0.15, min_n=10, exclude=None):
    """평면 경로 점 [(x, y, _, yaw, d), ...] -> [(x, y, z, yaw, d, q), ...] 와 지면을 찾은 점 비율.
    z_offset: 기준점이 지면 위로 떠 있는 높이. None = 모름 -> 경로 첫 점(= 지금 로버 기준점)이 fallback_z 가 되게 맞춤.
    fallback_z: 지금 로버 기준점 z.
    지면을 못 찾은 점(미관측): z 는 경로를 따라 앞뒤 찾은 점 사이 선형 보간(끝은 가장 가까운 찾은 점 값),
      자세는 그 z 의 경로 방향 기울기만(roll 0) — 관측 가장자리 기울기를 끌고 가지 않음
      (10/7 재생: 가장자리 15° 맞춤이 미관측 구간으로 이어져 방향이 돌면서 roll 25° 가 됨).
    하나도 못 찾으면 fallback_z·수평."""
    n = len(pts)
    if n == 0:
        return [], 0.0
    g = [layer.ground_at((p[0], p[1]), r_fit, min_n, exclude) if layer is not None else None for p in pts]
    known = np.array([v is not None for v in g])
    zg = np.array([v[0] if v is not None else np.nan for v in g])
    bc = np.array([(v[1], v[2]) if v is not None else (0.0, 0.0) for v in g])
    if known.any():
        s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(np.array([(p[0], p[1]) for p in pts]), axis=0).T))])
        zg = np.interp(s, s[known], zg[known])                    # 사이는 보간, 양 끝은 가장 가까운 값
        if not known.all() and n > 1:
            dzds = np.gradient(zg, s) if np.all(np.diff(s) > 0) else np.zeros(n)
            yaw = np.array([p[3] for p in pts])
            u = ~known
            bc[u, 0], bc[u, 1] = dzds[u] * np.cos(yaw[u]), dzds[u] * np.sin(yaw[u])   # 경로 방향 기울기만
        off = (float(fallback_z) - zg[0]) if z_offset is None else z_offset
        z = zg + off
    else:
        z = np.full(n, float(fallback_z))
    out = []
    for i, p in enumerate(pts):
        q = surface_quat(p[3], bc[i, 0], bc[i, 1])
        out.append((float(p[0]), float(p[1]), float(z[i]), float(p[3]), int(p[4]), q))
    return out, float(known.mean())
