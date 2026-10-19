#!/usr/bin/env python3
"""거리 -> turbo 색. depth 영상(depth_colorize.py)과 3D 맵(live_tsdf_node.py)이 **같은 함수**를 쓴다.

두 곳이 이 모듈 하나를 부르므로, 같은 거리는 항상 같은 RGB 가 된다.
거리의 정의는 depth 영상 값과 같다 — 카메라 **광축 방향 거리 Z** (유클리드 거리 아님).
"""
import numpy as np

_LUT = None


def turbo_lut():
    """256단계 turbo LUT (uint8, (256,3)). matplotlib 이 없으면 단순 그라디언트."""
    global _LUT
    if _LUT is None:
        try:
            import matplotlib
            # matplotlib 3.9 에서 cm.get_cmap 이 제거됐다. colormaps 쪽을 먼저 쓴다.
            cmap = matplotlib.colormaps["turbo"] if hasattr(matplotlib, "colormaps") \
                else matplotlib.cm.get_cmap("turbo")
            _LUT = (np.asarray(cmap(np.linspace(0, 1, 256)))[:, :3] * 255).astype(np.uint8)
        except Exception:
            g = np.linspace(0, 255, 256).astype(np.uint8)
            _LUT = np.stack([g, g, 255 - g], axis=1)
    return _LUT


def range_to_rgb(z, dmin, dmax):
    """z: 광축 거리 [m]. 반환 (rgb uint8 (...,3), valid bool (...)).

    valid = dmin <= z <= dmax. valid 가 아닌 곳의 색은 호출하는 쪽이 정한다
    (depth 영상은 검정, 3D 맵은 회색).
    """
    z = np.asarray(z, dtype=np.float32)
    valid = (z >= dmin) & (z <= dmax)
    idx = np.clip((z - np.float32(dmin)) / np.float32(max(dmax - dmin, 1e-6)), 0, 1)
    return turbo_lut()[(np.nan_to_num(idx) * 255).astype(np.uint8)], valid
