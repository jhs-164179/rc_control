#!/usr/bin/env python3
"""지면 연결 필터 — 3D 맵에서 지면과 이어지지 않은(공중에 뜬) 덩어리를 제거한다.

방법:
  1. 점군을 voxel 크기 3D 격자로 점유 표시
  2. 26-이웃 연결 성분으로 라벨링
  3. 아래 '시드'를 하나라도 포함한 성분만 남긴다
     a. **가장 큰 성분** (항상) — 실제 바닥+구조물 본체
     b. **궤적 시드** (traj 를 주면) — 로버가 지나간 자리(카메라 궤적 xy 반경 traj_radius)
        의 카메라 아래 traj_below 이내 점유 복셀. 로버가 실제로 밟은 땅이므로 확실한 지면이다.
     c. traj 가 없을 때만: XY 기둥별 최저 복셀 중 local_win 창 최저면에서 seed_band 이내
        (옛 방식. 아래 ⚠ 참고)

지면에 닿아 있는 벽·물체는 '이어진 부분'이므로 **남는다.** 제거되는 것은
바닥과 떨어져 떠 있는 덩어리와 가장자리 flying pixel 파편이다.

⚠ 260928 사고: 옛 방식(c)만 쓰던 때, 맵이 기울고(odom Z ≠ 중력, 포즈 z 1.7 m 상승)
  바닥 아래 잡음 조각이 국소 최저를 차지해 시드가 잡음 5칸(점 18개)에만 잡혔다.
  본체 157k 점이 통째로 '공중 덩어리'로 판정돼 RViz 에서 맵이 사라졌다.
  그래서 (a)(b)를 넣었다. (c)는 기울기 1 m 당 seed_band 를 넘으면(10 cm → 5.7°) 무너진다.
"""
import numpy as np
from scipy import ndimage

_STRUCT26 = np.ones((3, 3, 3), bool)


def ground_connected_mask(P, voxel=0.03, seed_band=0.05, local_win=1.0, max_cells=60_000_000,
                          traj=None, traj_radius=0.3, traj_below=0.5, min_seed_voxels=100):
    """반환: (keep_mask (N,) bool, info dict). 격자가 max_cells 를 넘으면 필터 생략(전부 유지).
    traj: (M,3) 카메라 위치 (P 와 같은 프레임). 주면 궤적 시드를 쓰고 기둥-최저 규칙은 안 쓴다.
    min_seed_voxels: 최대 성분이 아닌 시드 성분은 복셀 이만큼 이상일 때만 남긴다
      (3 cm 복셀 100개 ≈ 0.09 m²). 궤적 근처 작은 잡음 조각이 시드에 걸려 살아나는 것 방지 —
      포즈가 튀어 바닥이 두 겹으로 갈라진 경우의 두 번째 바닥은 이보다 훨씬 크다."""
    P = np.asarray(P, dtype=np.float64)
    info = dict(components=0, kept_components=0, removed=0, skipped=None, seeds="")
    if len(P) == 0:
        return np.ones(0, bool), info
    o = np.floor(P / voxel).astype(np.int64).min(axis=0)
    idx = np.floor(P / voxel).astype(np.int64) - o
    shp = tuple(int(v) for v in idx.max(axis=0) + 1)
    cells = shp[0] * shp[1] * shp[2]
    if cells > max_cells:
        info["skipped"] = f"격자 {shp} = {cells / 1e6:.0f}M 셀 > {max_cells / 1e6:.0f}M (포즈 폭주 의심)"
        return np.ones(len(P), bool), info

    occ = np.zeros(shp, bool)
    occ[idx[:, 0], idx[:, 1], idx[:, 2]] = True
    lab, n = ndimage.label(occ, structure=_STRUCT26)
    info["components"] = int(n)
    plab = lab[idx[:, 0], idx[:, 1], idx[:, 2]]

    # (a) 가장 큰 성분 (크기 = 점유 복셀 수)
    sizes = np.bincount(lab[occ], minlength=n + 1)
    sizes[0] = 0
    keep_lab_max = int(np.argmax(sizes))
    keep_lab = [keep_lab_max]
    n_traj = n_col = 0

    if traj is not None and len(traj):
        # (b) 궤적 시드: 기둥별 '그 위를 지난 카메라의 최고 z 인덱스', 반경만큼 팽창
        T = np.floor(np.asarray(traj, np.float64) / voxel).astype(np.int64) - o
        ok = (T[:, 0] >= 0) & (T[:, 0] < shp[0]) & (T[:, 1] >= 0) & (T[:, 1] < shp[1])
        camz = np.full(shp[:2], -(1 << 30), np.int64)
        np.maximum.at(camz, (T[ok, 0], T[ok, 1]), T[ok, 2])
        r = max(1, int(round(traj_radius / voxel)))
        yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
        camz = ndimage.maximum_filter(camz, footprint=(xx ** 2 + yy ** 2) <= r * r, mode="constant",
                                      cval=-(1 << 30))
        cz = camz[idx[:, 0], idx[:, 1]]
        s = (idx[:, 2] <= cz) & (idx[:, 2] >= cz - traj_below / voxel)
        tl = np.unique(plab[s])
        n_traj = len(tl)
        keep_lab.extend(tl.tolist())
    else:
        # (c) 옛 방식: 기둥 최저 복셀이 국소 최저면 근처
        has = occ.any(axis=2)
        colmin = np.full(has.shape, np.inf)
        colmin[has] = np.argmax(occ, axis=2)[has]
        w = max(1, int(round(local_win / voxel)))
        local = ndimage.minimum_filter(colmin, size=2 * w + 1, mode="nearest")
        seed = has & (colmin <= local + seed_band / voxel)
        sx, sy = np.nonzero(seed)
        cl = np.unique(lab[sx, sy, colmin[sx, sy].astype(np.int64)])
        n_col = len(cl)
        keep_lab.extend(cl.tolist())

    keep_lab = np.unique(np.array(keep_lab))
    keep_lab = keep_lab[(keep_lab > 0) & ((sizes[keep_lab] >= min_seed_voxels) | (keep_lab == keep_lab_max))]
    info["kept_components"] = int(len(keep_lab))
    info["seeds"] = (f"최대성분 + 궤적시드 {n_traj}" if traj is not None and len(traj)
                     else f"최대성분 + 기둥최저시드 {n_col}") + f", 복셀<{min_seed_voxels} 제외"
    good = np.zeros(n + 1, bool)
    good[keep_lab] = True
    mask = good[plab]
    info["removed"] = int((~mask).sum())
    return mask, info


def height_clip_mask(P, max_height, cell=0.05, win=0.3):
    """지면 위 max_height [m] 를 넘는 점을 버리는 mask. 반환 (keep (N,) bool, info).

    지면 높이 = cell 격자 칸별 최저 z 를 win 반경 창에서 다시 최저로 편 값.
    경사로처럼 지면이 올라가면 지면도 같이 올라간다(칸별 최저가 경사면 자체).
    +Z 가 대략 위여야 한다(odom 프레임 그대로 가능 — ground_connected_mask 와 같은 가정).
    max_height <= 0 이면 끔(전부 유지).

    용도: 로버 높이(115.5 mm) 훨씬 위의 구조물 윗부분·번짐을 맵과 격자에서 뺀다.
    ⚠ 실제 구조물 윗부분도 같이 잘린다(예: 문틀 기둥은 지면+max_height 까지만 남음).
    """
    P = np.asarray(P, dtype=np.float64)
    info = dict(removed=0, max_height=max_height)
    if max_height <= 0 or len(P) == 0:
        return np.ones(len(P), bool), info
    ij = np.floor(P[:, :2] / cell).astype(np.int64)
    ij -= ij.min(axis=0)
    shp = tuple(int(v) for v in ij.max(axis=0) + 1)
    zmin = np.full(shp, np.inf)
    np.minimum.at(zmin, (ij[:, 0], ij[:, 1]), P[:, 2])
    r = max(1, int(round(win / cell)))
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    ground = ndimage.minimum_filter(zmin, footprint=(xx ** 2 + yy ** 2) <= r * r, mode="nearest")
    keep = P[:, 2] - ground[ij[:, 0], ij[:, 1]] <= max_height
    info["removed"] = int((~keep).sum())
    return keep, info


def main():
    """오프라인: 저장된 *_cloud.ply / *_mesh.ply 에 필터 적용 -> *_ground.ply"""
    import argparse
    import open3d as o3d
    o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
    ap = argparse.ArgumentParser(description="지면 연결 필터 (공중에 뜬 덩어리 제거)")
    ap.add_argument("ply", nargs="+", help="*_cloud.ply 또는 *_mesh.ply")
    ap.add_argument("--voxel", type=float, default=0.03)
    ap.add_argument("--seed-band", type=float, default=0.10)
    ap.add_argument("--local-win", type=float, default=1.0)
    ap.add_argument("--max-height", type=float, default=0.0,
                    help="지면 위 이 높이[m] 초과 점 제거 (height_clip_mask). 0 = 끔")
    ap.add_argument("--traj", default="auto",
                    help="TUM 궤적 (궤적 시드). auto = 같은 이름의 *_traj_tum.txt 가 있으면 사용, none = 안 씀")
    a = ap.parse_args()
    import os
    import re
    for f in a.ply:
        out = f[:-4] + "_ground.ply"
        traj = None
        tp = re.sub(r"_(cloud|mesh)(_ground)?\.ply$", "_traj_tum.txt", f) if a.traj == "auto" else a.traj
        if tp != "none" and os.path.exists(tp):
            traj = np.loadtxt(tp, ndmin=2)[:, 1:4]
            print(f"[traj] {tp} ({len(traj)} poses)")
        mesh = o3d.io.read_triangle_mesh(f)
        if len(mesh.triangles):
            V = np.asarray(mesh.vertices)
            keep, info = ground_connected_mask(V, a.voxel, a.seed_band, a.local_win, traj=traj)
            keep &= height_clip_mask(V, a.max_height)[0]
            info["removed"] = int((~keep).sum())
            mesh.remove_vertices_by_mask(~keep)
            mesh.compute_vertex_normals()
            o3d.io.write_triangle_mesh(out, mesh)
            n = len(V)
        else:
            pc = o3d.io.read_point_cloud(f)
            V = np.asarray(pc.points)
            keep, info = ground_connected_mask(V, a.voxel, a.seed_band, a.local_win, traj=traj)
            keep &= height_clip_mask(V, a.max_height)[0]
            info["removed"] = int((~keep).sum())
            o3d.io.write_point_cloud(out, pc.select_by_index(np.nonzero(keep)[0]))
            n = len(V)
        if info["skipped"]:
            print(f"[skip] {f}: {info['skipped']}")
        print(f"{f} -> {out}  성분 {info['components']} 중 {info['kept_components']} 유지 ({info['seeds']}), "
              f"제거 {info['removed']}/{n} ({info['removed'] / max(n, 1) * 100:.1f}%)")


if __name__ == "__main__":
    main()
