#!/usr/bin/env python3
"""주행가능성 격자 계산 코어 — 순수 numpy/scipy. ROS도 open3d도 쓰지 않는다.

실시간 노드(traversability_node.py)와 오프라인 CLI(cloud_to_trav.py)가
같은 함수를 부르게 해서, 오프라인에서 튜닝한 임계값이 실시간에서 그대로 재현되게 한다.

좌표계: 입력 점군은 odom 프레임. 내부 계산은 전부 '중력 정렬 프레임'(+Z가 위)에서 한다.
"""
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

UNKNOWN = -1
FREE_MAX = 65      # warn 시작 비용
BLOCKED = 100


@dataclass
class TravParams:
    cell_size: float = 0.05
    min_points_per_cell: int = 5
    max_range_m: float = 8.0
    slope_warn_deg: float = 15.0
    slope_block_deg: float = 25.0
    step_warn_m: float = 0.025
    step_block_m: float = 0.0449
    rough_warn_m: float = 0.01
    rough_block_m: float = 0.02
    inflate_radius_m: float = 0.146


def gravity_align_R(g_odom):
    """중력 벡터(아래 방향, odom 프레임) -> R_go. R_go @ p_odom = p_grav (+Z가 위).

    odom의 X축을 가능한 한 유지해서 회전이 최소가 되게 한다(맵이 빙빙 돌지 않도록).
    """
    g = np.asarray(g_odom, dtype=float)
    n = np.linalg.norm(g)
    if n < 1e-6:
        return np.eye(3)
    z = -g / n                                   # up
    ref = np.array([1.0, 0.0, 0.0])
    if abs(np.dot(ref, z)) > 0.95:               # X가 up과 거의 나란하면 Y로 바꾼다
        ref = np.array([0.0, 1.0, 0.0])
    x = ref - z * np.dot(ref, z)
    x /= np.linalg.norm(x)
    y = np.cross(z, x)
    return np.stack([x, y, z])                   # 행벡터 basis


def _cell_stats(pts, cell, origin, shape):
    """셀별 n / z평균 / z표준편차 / z최소 / z최대. reduceat 기반(=정렬 1회)."""
    nx, ny = shape
    ix = np.floor((pts[:, 0] - origin[0]) / cell).astype(np.int64)
    iy = np.floor((pts[:, 1] - origin[1]) / cell).astype(np.int64)
    ok = (ix >= 0) & (ix < nx) & (iy >= 0) & (iy < ny)
    ix, iy, z = ix[ok], iy[ok], pts[ok, 2]
    flat = iy * nx + ix

    order = np.argsort(flat, kind="stable")
    flat, z = flat[order], z[order]
    uniq, start = np.unique(flat, return_index=True)
    n = np.diff(np.append(start, flat.size))

    s = np.add.reduceat(z, start)
    s2 = np.add.reduceat(z * z, start)
    zmin = np.minimum.reduceat(z, start)
    zmax = np.maximum.reduceat(z, start)

    mean = s / n
    var = np.maximum(s2 / n - mean * mean, 0.0)

    cnt = np.zeros(nx * ny, np.int32)
    zm = np.full(nx * ny, np.nan)
    zs = np.full(nx * ny, np.nan)
    lo = np.full(nx * ny, np.nan)
    hi = np.full(nx * ny, np.nan)
    cnt[uniq] = n
    zm[uniq], zs[uniq] = mean, np.sqrt(var)
    lo[uniq], hi[uniq] = zmin, zmax
    r = (ny, nx)
    return cnt.reshape(r), zm.reshape(r), zs.reshape(r), lo.reshape(r), hi.reshape(r)


def _neighbor_max_absdiff(h, valid):
    """4-이웃과의 높이차 최대값. 한쪽이 unknown이면 그 방향은 건너뛴다."""
    out = np.zeros_like(h)
    hv = np.where(valid, h, 0.0)
    for ax, sh in ((0, 1), (0, -1), (1, 1), (1, -1)):
        nh = np.roll(hv, sh, axis=ax)
        nv = np.roll(valid, sh, axis=ax)
        # 경계는 감싸지 않게 잘라낸다
        sl = [slice(None), slice(None)]
        sl[ax] = slice(0, 1) if sh > 0 else slice(-1, None)
        nv = nv.copy()
        nv[tuple(sl)] = False
        out = np.maximum(out, np.where(nv & valid, np.abs(h - nh), 0.0))
    return out


def _slope_deg(h, valid, cell):
    """이웃 높이차의 중심차분으로 경사각. 양쪽 이웃이 다 있어야 계산한다."""
    grad = np.zeros_like(h)
    hv = np.where(valid, h, 0.0)
    for ax in (0, 1):
        p, n = np.roll(hv, -1, axis=ax), np.roll(hv, 1, axis=ax)
        pv, nv = np.roll(valid, -1, axis=ax), np.roll(valid, 1, axis=ax)
        for arr, sh in ((pv, -1), (nv, 1)):
            sl = [slice(None), slice(None)]
            sl[ax] = slice(-1, None) if sh < 0 else slice(0, 1)
            arr[tuple(sl)] = False
        both = pv & nv & valid
        g = np.where(both, (p - n) / (2 * cell), 0.0)
        grad = grad + g * g
    return np.degrees(np.arctan(np.sqrt(grad)))


def _ramp(x, warn, block):
    """warn 이하 0, block 이상 1로 가는 선형 램프."""
    if block <= warn:
        return (x >= block).astype(float)
    return np.clip((x - warn) / (block - warn), 0.0, 1.0)


# ══════════════════════════════════════════════════════════════════════
# 1단계: 높이 격자 — 판정과 무관. 어떤 판정기를 쓰든 이 결과를 입력으로 받는다.
# ══════════════════════════════════════════════════════════════════════
def build_heightmap(pts_grav, p: TravParams, center_xy=None):
    """중력 정렬 프레임의 점군 -> 높이 격자. **판정은 하지 않는다.**

    반환 dict (= 판정기 입력 계약):
      origin (2,)      격자 [0,0] 셀의 **모서리** 좌표 (x, y) [m]
      cell   float     셀 한 변 [m]
      shape  (nx, ny)
      height (ny, nx)  셀 높이(z 평균). **미관측은 NaN**
      span   (ny, nx)  셀 안 z_max - z_min. 수직 구조물 판정용. 미관측은 NaN
      count  (ny, nx)  셀 안 점 개수
      valid  (ny, nx)  count >= min_points_per_cell

    ⚠ 배열은 전부 (ny, nx) = [행=y, 열=x] 이다.
      judge_core 처럼 [x, y] 순서를 쓰는 판정기는 어댑터에서 전치할 것.
    """
    pts = np.asarray(pts_grav, dtype=np.float64)
    if center_xy is not None and p.max_range_m > 0:
        d = np.linalg.norm(pts[:, :2] - np.asarray(center_xy), axis=1)
        pts = pts[d <= p.max_range_m]
    if pts.shape[0] < p.min_points_per_cell:
        return None

    cell = p.cell_size
    lo_xy = np.floor(pts[:, :2].min(axis=0) / cell) * cell - cell
    hi_xy = np.ceil(pts[:, :2].max(axis=0) / cell) * cell + cell
    nx, ny = (np.round((hi_xy - lo_xy) / cell).astype(int) + 1)
    nx, ny = int(max(nx, 1)), int(max(ny, 1))

    cnt, zmean, zstd, zmin, zmax = _cell_stats(pts, cell, lo_xy, (nx, ny))
    valid = cnt >= p.min_points_per_cell
    if not valid.any():
        return None
    return dict(origin=lo_xy, cell=cell, shape=(nx, ny), count=cnt, valid=valid,
                center_xy=None if center_xy is None else np.asarray(center_xy, float),
                height=np.where(valid, zmean, np.nan),
                span=np.where(valid, zmax - zmin, np.nan),
                zstd=np.where(valid, zstd, np.nan))


# ══════════════════════════════════════════════════════════════════════
# 2단계: 판정기 — ★ 여기가 교체 지점 ★
# ══════════════════════════════════════════════════════════════════════
#
#   판정기 계약 (JudgeFn):
#       judge(hmap: dict, p: TravParams) -> dict
#
#   입력  hmap : build_heightmap() 반환값 그대로
#   출력  반드시 포함:
#           cost   (ny, nx) int   -1 미관측 / 0~99 통행가능 / 100 통행불가
#         있으면 PNG·로그에 쓰임(없어도 동작):
#           slope  (ny, nx) [deg]
#           step   (ny, nx) [m]
#           rough  (ny, nx) [m]
#           why    (ny, nx) int   사유 코드 (판정기마다 의미가 다름)
#
#   새 판정기를 붙이는 법:
#     1. 위 계약대로 함수를 하나 쓴다 (다른 파일이어도 된다)
#     2. traversability_node.py / cloud_to_trav.py 의 JUDGES 사전에 이름을 추가한다
#     3. 실행 시 judge:=<이름> 또는 --judge <이름>
#   나머지(중력 정렬, 격자화, 팽창, 발행)는 건드릴 필요가 없다.
#
def judge_builtin(hmap, p: TravParams):
    """기본 판정기 — 경사/단차/거칠기 3지표의 램프 최댓값. 로버 제원에서 유도한 임계값."""
    cell, valid = hmap["cell"], hmap["valid"]
    h = np.where(valid, hmap["height"], 0.0)
    slope = _slope_deg(h, valid, cell)
    step = np.maximum(_neighbor_max_absdiff(h, valid),
                      np.where(valid, np.nan_to_num(hmap["span"]), 0.0))
    rough = np.where(valid, np.nan_to_num(hmap["zstd"]), 0.0)

    r = np.maximum.reduce([
        _ramp(slope, p.slope_warn_deg, p.slope_block_deg),
        _ramp(step, p.step_warn_m, p.step_block_m),
        _ramp(rough, p.rough_warn_m, p.rough_block_m),
    ])
    cost = np.rint(r * FREE_MAX).astype(np.int16)
    hard = ((slope >= p.slope_block_deg) | (step >= p.step_block_m)
            | (rough >= p.rough_block_m)) & valid
    cost[hard] = BLOCKED
    cost[~valid] = UNKNOWN
    return dict(cost=cost, slope=slope, step=step, rough=rough)


# ══════════════════════════════════════════════════════════════════════
# 3단계: 후처리 — 판정기와 무관하게 공통 적용
# ══════════════════════════════════════════════════════════════════════
def inflate_blocked(cost, valid, p: TravParams, cell):
    """통행불가 셀을 반차폭만큼 부풀린다. 로버를 점으로 계획하기 위한 것.
    판정기가 무엇이든 동일하게 적용된다. p.inflate_radius_m = 0 이면 생략."""
    rad = int(round(p.inflate_radius_m / cell))
    hard = cost >= BLOCKED
    if rad <= 0 or not hard.any():
        return cost
    yy, xx = np.mgrid[-rad:rad + 1, -rad:rad + 1]
    disk = (xx * xx + yy * yy) <= rad * rad
    infl = ndimage.binary_dilation(hard, structure=disk)
    out = cost.copy()
    out[infl & valid & ~hard] = BLOCKED
    return out


def compute_grid(pts_grav, p: TravParams, center_xy=None, judge=None, hmap=None):
    """점군 -> 주행가능성 격자. judge=None 이면 judge_builtin 을 쓴다.
    [261008] hmap 을 주면(그리드 피쳐맵에서 만든 높이 격자) 점군·center_xy 는 안 쓴다.

    반환 dict: build_heightmap 의 키 + 판정기가 낸 키(cost/slope/step/rough/why)
    """
    if hmap is None:
        hmap = build_heightmap(pts_grav, p, center_xy=center_xy)
    if hmap is None:
        return None
    res = (judge or judge_builtin)(hmap, p)
    cost = inflate_blocked(np.asarray(res["cost"]), hmap["valid"], p, hmap["cell"])
    out = dict(hmap)
    out.update(res)
    out["cost"] = cost.astype(np.int8)
    return out


def summarize(g):
    """판정기가 slope/step/rough 를 안 내놔도 동작한다(그 부분만 생략)."""
    c, v = g["cost"], g["valid"]
    n = int(v.sum())
    if n == 0:
        return "유효 셀 없음"
    blocked = int((c >= BLOCKED).sum())
    warn = int(((c > 0) & (c < BLOCKED)).sum())
    out = (f"셀 {n}개 (unknown {int((~v).sum())})  "
           f"free {n - warn - blocked} / warn {warn} / blocked {blocked}")
    for key, sc, unit, fmt in (("slope", 1, "deg", ".1f"), ("step", 1000, "mm", ".0f"),
                               ("rough", 1000, "mm", ".0f")):
        if key in g and g[key] is not None:
            m = np.nanmax(np.where(v, g[key], np.nan))
            if np.isfinite(m):
                out += f"  | {key} max {m * sc:{fmt}}{unit}"
    return out
