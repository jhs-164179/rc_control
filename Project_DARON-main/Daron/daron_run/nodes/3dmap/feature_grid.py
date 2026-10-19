#!/usr/bin/env python3
"""그리드 피쳐맵 코어 (261008) — 순수 numpy. ROS·open3d 를 쓰지 않는다.

교수님 지시(10/8): dense global map(TSDF 전체 누적) 대신, 경로계획 cost 계산에 필요한 축만
칸마다 **채널(벡터)**로 누적한다. dense 맵은 주행 뒤 오프라인 refine 용으로만.
누적 칸은 2.5 cm(세부 칸, live_tsdf grid_cell 기본), 판정 칸은 5 cm — 판정 칸 하나 = 세부 칸 2×2 로
칸 안 거칠기·높이 폭을 프레임 평균이 끝난 세부 칸 높이로 잰다(heightmap_from_cells).

채널 (칸마다, 맵 좌표계 z = 위)
  W    가중치 합                 — 융합 가중치·신뢰도 (관측이 많고 가까울수록 큼)
  WZ   Σ w·z̄_f                   → 높이 평균 z = WZ / W          (판정: 경사·턱·낙차)
  WZ2  Σ w·z̄_f²                  → 높이 분산 var = WZ2/W − z²     (프레임 사이 흔들림 = 포즈·depth 오차)
  WLO  Σ w·lo_f, WHI Σ w·hi_f    → zlo, zhi (프레임별 칸 하/상단의 가중평균) → span = zhi − zlo (판정: 벽 > 15 cm)
  NP   점 수 합                   — 판정 '관측 칸' 기준(min_points_per_cell)
  NF   본 프레임 수
  T    마지막 관측 시각 [s]
  (z̄_f, lo_f, hi_f = 그 키프레임에서 그 칸에 떨어진 점들의 평균·하단·상단)

프레임 가중치 w = min(n, n_cap) · (d_ref / max(d, d_ref))^4
  D405 스테레오 거리 오차 σ ∝ d² → 분산 ∝ d⁴. 가까이서 많이 본 칸일수록 무겁다(d_ref 보다 가까우면 더 안 무거워짐 — 바퀴 자국 방지).
잊기: W 가 w_max 를 넘으면 그 칸의 합들을 같은 비율로 줄인다 → 오래된 관측이 무한히 버티지 않고
  새 관측(포즈 보정 뒤, 지형 변화)이 반영된다. TSDF max weight 와 같은 역할.
하/상단 lo_f, hi_f 는 그 프레임 칸 점들의 min/max 를 mean ± k·std 로 자른 값(튀는 점 하나에 휘둘리지 않게).

배열 배치는 [행 = y, 열 = x]. 칸 (iy, ix) 의 중심 = ((ix + ix0 + 0.5)·cell, (iy + iy0 + 0.5)·cell).
"""
from __future__ import annotations

import warnings

import numpy as np

CHANNELS = ("W", "WZ", "WZ2", "WLO", "WHI", "NP", "NF", "T")


class FeatureGrid:
    def __init__(self, cell=0.05, d_ref=0.5, n_cap=50, w_max=2000.0, clip_k=2.5, grow_m=2.0):
        self.cell = float(cell)
        self.d_ref = float(d_ref)
        self.n_cap = int(n_cap)
        self.w_max = float(w_max)
        self.clip_k = float(clip_k)
        self.grow = max(1, int(round(grow_m / self.cell)))
        self.ix0 = self.iy0 = 0          # 배열 [0,0] 칸의 전역 정수 인덱스
        self.a = {k: np.zeros((0, 0), np.float64) for k in CHANNELS}
        self.n_frames = 0

    # ── 저장소 ───────────────────────────────────────────────────────
    @property
    def shape(self):
        return self.a["W"].shape                     # (ny, nx)

    def _ensure(self, gx_min, gx_max, gy_min, gy_max):
        """전역 칸 인덱스 범위가 배열 안에 들도록 키운다(grow 단위로 여유를 둔다)."""
        ny, nx = self.shape
        if nx and ny and gx_min >= self.ix0 and gy_min >= self.iy0 \
                and gx_max < self.ix0 + nx and gy_max < self.iy0 + ny:
            return
        if nx == 0 or ny == 0:
            x0, y0 = gx_min - self.grow, gy_min - self.grow
            x1, y1 = gx_max + self.grow + 1, gy_max + self.grow + 1
        else:
            x0 = min(self.ix0, gx_min - self.grow)
            y0 = min(self.iy0, gy_min - self.grow)
            x1 = max(self.ix0 + nx, gx_max + self.grow + 1)
            y1 = max(self.iy0 + ny, gy_max + self.grow + 1)
        new = {k: np.zeros((y1 - y0, x1 - x0), np.float64) for k in CHANNELS}
        if nx and ny:
            sy, sx = self.iy0 - y0, self.ix0 - x0
            for k in CHANNELS:
                new[k][sy:sy + ny, sx:sx + nx] = self.a[k]
        self.a, self.ix0, self.iy0 = new, x0, y0

    # ── 적분 ─────────────────────────────────────────────────────────
    def integrate(self, pts, cam_pos, stamp_s):
        """pts (N,3) 맵 좌표 점(한 키프레임), cam_pos (3,) 그 프레임 카메라 위치. 반환: 갱신한 칸 수."""
        pts = np.asarray(pts, np.float64)
        if pts.ndim != 2 or len(pts) == 0:
            return 0
        c = self.cell
        gx = np.floor(pts[:, 0] / c).astype(np.int64)
        gy = np.floor(pts[:, 1] / c).astype(np.int64)
        self._ensure(int(gx.min()), int(gx.max()), int(gy.min()), int(gy.max()))
        ny, nx = self.shape
        flat = (gy - self.iy0) * nx + (gx - self.ix0)
        z = pts[:, 2]
        d = np.linalg.norm(pts - np.asarray(cam_pos, np.float64)[None, :], axis=1)

        order = np.argsort(flat, kind="stable")
        flat, z, d = flat[order], z[order], d[order]
        uniq, start = np.unique(flat, return_index=True)
        n = np.diff(np.append(start, flat.size)).astype(np.float64)
        s = np.add.reduceat(z, start)
        s2 = np.add.reduceat(z * z, start)
        zmin = np.minimum.reduceat(z, start)
        zmax = np.maximum.reduceat(z, start)
        dm = np.add.reduceat(d, start) / n
        mean = s / n
        std = np.sqrt(np.maximum(s2 / n - mean * mean, 0.0))
        lo = np.maximum(zmin, mean - self.clip_k * std)
        hi = np.minimum(zmax, mean + self.clip_k * std)
        # [261008 밤] 거리 가중 상한: d_ref(0.5 m)보다 가까워도 d_ref 와 같은 무게. 상한이 없으면 0.18 m 에 찍힌
        # 로버 앞바퀴(영상 아래쪽)가 바닥 관측 수십 장보다 50~300 배 무거워져 주행 궤적 옆에 +4~8 cm '장애물 줄'이 남았다
        # (step·trav_ramp19·trav_block4_6). 같은 포즈 오프라인 재계산: step 평지 주의/불가 7.8/9.2 % → 0.3/0.0 %, 물체 검출 그대로.
        w = np.minimum(n, self.n_cap) * (self.d_ref / np.maximum(dm, self.d_ref)) ** 4

        A = {k: v.reshape(-1) for k, v in self.a.items()}   # 뷰 (같은 메모리)
        A["W"][uniq] += w
        A["WZ"][uniq] += w * mean
        A["WZ2"][uniq] += w * mean * mean
        A["WLO"][uniq] += w * lo
        A["WHI"][uniq] += w * hi
        A["NP"][uniq] += n
        A["NF"][uniq] += 1
        A["T"][uniq] = float(stamp_s)
        over = A["W"][uniq] > self.w_max
        if over.any():                                   # 잊기: 합들을 같은 비율로 줄임 (평균은 그대로)
            idx = uniq[over]
            f = self.w_max / A["W"][idx]
            for k in ("W", "WZ", "WZ2", "WLO", "WHI"):
                A[k][idx] *= f
        self.n_frames += 1
        return int(len(uniq))

    # ── 조회 ─────────────────────────────────────────────────────────
    def cells(self, center_xy=None, radius=0.0, min_np=1):
        """관측된 칸 목록 (dict of 1D arrays): x, y(칸 중심), z, zlo, zhi, var, w, np, nf, t.
        center_xy·radius>0 이면 그 원 안 칸만."""
        W = self.a["W"]
        ok = (W > 0) & (self.a["NP"] >= min_np)
        iy, ix = np.nonzero(ok)
        c = self.cell
        x = (ix + self.ix0 + 0.5) * c
        y = (iy + self.iy0 + 0.5) * c
        if center_xy is not None and radius > 0:
            keep = np.hypot(x - center_xy[0], y - center_xy[1]) <= radius
            iy, ix, x, y = iy[keep], ix[keep], x[keep], y[keep]
        w = W[iy, ix]
        z = self.a["WZ"][iy, ix] / w
        var = np.maximum(self.a["WZ2"][iy, ix] / w - z * z, 0.0)
        return dict(x=x, y=y, z=z,
                    zlo=self.a["WLO"][iy, ix] / w, zhi=self.a["WHI"][iy, ix] / w,
                    var=var, w=w, np=self.a["NP"][iy, ix], nf=self.a["NF"][iy, ix],
                    t=self.a["T"][iy, ix])

    def n_cells(self):
        return int((self.a["W"] > 0).sum())

    def nbytes(self):
        return int(sum(v.nbytes for v in self.a.values()))

    # ── 저장 ─────────────────────────────────────────────────────────
    def save(self, path, meta=None):
        """meta: dict (frame, map_id 등) — JSON 문자열로 같이 저장. load 가 g.meta 로 돌려준다."""
        import json
        self.meta = dict(getattr(self, "meta", {}) or {}, **(meta or {}))
        np.savez_compressed(path, cell=self.cell, ix0=self.ix0, iy0=self.iy0, n_frames=self.n_frames,
                            d_ref=self.d_ref, n_cap=self.n_cap, w_max=self.w_max, clip_k=self.clip_k,
                            meta=np.array(json.dumps(self.meta, ensure_ascii=False)),
                            **{k: self.a[k] for k in CHANNELS})

    @classmethod
    def load(cls, path):
        import json
        f = np.load(path)
        g = cls(cell=float(f["cell"]), d_ref=float(f["d_ref"]), n_cap=int(f["n_cap"]),
                w_max=float(f["w_max"]), clip_k=float(f["clip_k"]))
        g.ix0, g.iy0, g.n_frames = int(f["ix0"]), int(f["iy0"]), int(f["n_frames"])
        g.a = {k: np.array(f[k], np.float64) for k in CHANNELS}
        g.meta = json.loads(str(f["meta"])) if "meta" in f.files else {}
        return g


# ══════════════════════════════════════════════════════════════════════
# 판정 쪽: 칸 목록 → build_heightmap 과 같은 높이 격자 (trav_core 계약)
# ══════════════════════════════════════════════════════════════════════
VERT_MIN = 0.08   # [m] 세부 칸 프레임 하/상단 폭이 이 이상이면 '수직 구조'로 보고 span 에 넣는다 (바닥 depth 잡음 폭 ≲ 4 cm)


def infer_cell(x, y):
    """칸 중심 좌표에서 칸 크기 추정 (메시지에 칸 크기가 없으므로). 가장 작은 양의 간격, 0.1 mm 반올림."""
    best = np.inf
    for v in (x, y):
        u = np.unique(np.round(np.asarray(v) / 1e-4).astype(np.int64))
        if len(u) > 1:
            best = min(best, int(np.diff(u).min()))
    return None if not np.isfinite(best) else best * 1e-4


def heightmap_from_cells(cells, R_go, origin, shape, cell, min_points=5, cell_src=None):
    """칸 목록(맵 좌표) → 중력 정렬 프레임의 높이 격자. 반환: cnt, zmean, zstd, zmin, zmax (각 (ny, nx)).

    trav_core._cell_stats 와 같은 꼴로 돌려준다. 점을 다시 칸에 넣는 대신 **목표 칸마다 원래 칸을 찾아
    읽는다**(역사상) — 회전한 칸 중심을 다시 칸에 넣으면 기울기만큼 칸 간격이 줄어 빈칸·겹칸이 생긴다.
      목표 칸 안 k×k 표본점 (gx, gy) [중력] → p_map ≈ R_goᵀ·(gx, gy, z_ref) → 원래(세부) 칸 → 높이는 R_go 3행으로.
    z_ref 는 칸들 높이 중앙값(xy 근사 오차 = 기울기 × 높이차, 수 mm). k = round(cell / cell_src).

    판정 칸(5 cm) 하나 = 세부 칸(2.5 cm) k×k 개:
      zmean  세부 칸 융합 높이의 가중평균(가중 = 융합 가중치 w)
      zstd   세부 칸 융합 높이들의 표준편차 — 칸 안 표면 거칠기. 프레임 평균이 끝난 값이라 depth 잡음이 빠진다
             (TSDF 표면에서 잰 '칸 안 점 퍼짐'과 같은 뜻). 세부 칸이 하나뿐이면 0.
      zmin/zmax  세부 칸 융합 높이의 최소·최대. 단, 어떤 세부 칸의 프레임 하/상단 폭이 VERT_MIN 이상이면
             (벽·상자 옆면처럼 한 칸 안에 수직으로 펼쳐진 면) 그 하/상단도 포함 → span = zmax − zmin 이 벽 높이를 담는다.
    origin: 목표 격자 [0,0] 칸 모서리 (중력 프레임 x, y), shape (nx, ny)."""
    nx, ny = shape
    out_n = np.zeros((ny, nx), np.int32)
    nan = np.full((ny, nx), np.nan)
    if cells is None or len(cells["x"]) == 0:
        return out_n, nan.copy(), nan.copy(), nan.copy(), nan.copy()
    x, y = np.asarray(cells["x"], np.float64), np.asarray(cells["y"], np.float64)
    if cell_src is None:
        cell_src = infer_cell(x, y) or cell
    k = max(1, int(round(cell / cell_src)))
    gx = np.floor(x / cell_src).astype(np.int64)
    gy = np.floor(y / cell_src).astype(np.int64)
    gx0, gy0 = int(gx.min()), int(gy.min())
    W_, H_ = int(gx.max()) - gx0 + 1, int(gy.max()) - gy0 + 1
    lut = np.full((H_, W_), -1, np.int64)
    lut[gy - gy0, gx - gx0] = np.arange(len(x))

    R = np.asarray(R_go, np.float64)
    zc, lo_c, hi_c = (np.asarray(cells[n_], np.float64) for n_ in ("z", "zlo", "zhi"))
    zg_all = R[2, 0] * x + R[2, 1] * y + R[2, 2] * zc                  # 세부 칸 융합 높이 (중력)
    lo_all = R[2, 0] * x + R[2, 1] * y + R[2, 2] * lo_c
    hi_all = R[2, 0] * x + R[2, 1] * y + R[2, 2] * hi_c
    vert = (hi_c - lo_c) >= VERT_MIN
    w_all = np.asarray(cells.get("w", np.ones(len(x))), np.float64)
    np_all = np.asarray(cells["np"], np.float64)
    z_ref = float(np.median(zc))

    offs = (np.arange(k) + 0.5) / k * cell                             # 칸 모서리 기준 표본 위치
    cx = origin[0] + np.arange(nx) * cell
    cy = origin[1] + np.arange(ny) * cell
    SX = (cx[None, :, None, None] + offs[None, None, None, :])         # (1, nx, 1, k)
    SY = (cy[:, None, None, None] + offs[None, None, :, None])         # (ny, 1, k, 1)
    SX, SY = np.broadcast_arrays(SX, SY)                               # (ny, nx, k, k)
    P = np.stack([SX, SY, np.full(SX.shape, z_ref)], -1) @ R          # 중력 → 맵 (= R.T @ p)
    sx = np.floor(P[..., 0] / cell_src).astype(np.int64) - gx0
    sy = np.floor(P[..., 1] / cell_src).astype(np.int64) - gy0
    inside = (sx >= 0) & (sx < W_) & (sy >= 0) & (sy < H_)
    src = np.full(sx.shape, -1, np.int64)
    src[inside] = lut[sy[inside], sx[inside]]
    src = src.reshape(ny, nx, k * k)
    # 같은 세부 칸이 표본 둘에 걸리면(기울기) 한 번만 센다
    if k > 1:
        srt = np.sort(src, axis=-1)
        dup = np.zeros_like(srt, bool)
        dup[..., 1:] = srt[..., 1:] == srt[..., :-1]
        src = np.where(dup, -1, srt)
    hit = src >= 0
    any_hit = hit.any(-1)
    if not any_hit.any():
        return out_n, nan.copy(), nan.copy(), nan.copy(), nan.copy()
    sidx = np.where(hit, src, 0)

    def g(a):
        v = a[sidx]
        return np.where(hit, v, np.nan)

    Z, Wt = g(zg_all), np.where(hit, w_all[sidx], 0.0)
    with np.errstate(invalid="ignore", divide="ignore"), warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)              # 빈 칸의 nan 집계 경고
        zm = np.nansum(Z * Wt, -1) / Wt.sum(-1)
        n_sub = hit.sum(-1)
        zs = np.where(n_sub > 1, np.nanstd(Z, -1), 0.0)
        zmin, zmax = np.nanmin(Z, -1), np.nanmax(Z, -1)
        V = hit & vert[sidx]
        if V.any():
            zmin = np.fmin(zmin, np.nanmin(np.where(V, lo_all[sidx], np.nan), -1))
            zmax = np.fmax(zmax, np.nanmax(np.where(V, hi_all[sidx], np.nan), -1))
    out_n = np.where(any_hit, np.minimum((np_all[sidx] * hit).sum(-1), np.iinfo(np.int32).max), 0).astype(np.int32)
    zm[~any_hit] = np.nan
    zs[~any_hit] = np.nan
    zmin[~any_hit] = np.nan
    zmax[~any_hit] = np.nan
    return out_n, zm, zs, zmin, zmax
