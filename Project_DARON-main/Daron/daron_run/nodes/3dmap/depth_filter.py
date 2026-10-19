#!/usr/bin/env python3
"""depth 영상 고립 조각(speckle) 제거 — 공중에 뿌려지는 스테레오 오매칭 점 방지.

D405 는 패시브 스테레오라 텍스처가 약한 곳(어두운 벽, 단색 면)에서 매칭이 틀려
실제 면과 동떨어진 거리의 **작은 덩어리**가 생긴다. 영상에서는 무효(0) 픽셀에 둘러싸인
섬으로 보이고, 3D 로 올리면 구조물 앞 공중에 뜬 점 뭉치가 된다.

방법: [dmin, dmax] 로 자른 뒤 남은 유효 픽셀을 8-연결로 묶고, 물리 면적
(픽셀 수 × (평균거리/fx)²)이 min_area_m2 미만인 섬을 버린다.
  - 바닥에 닿은 물체(돌, 기둥 밑동)는 영상에서 바닥 영역과 붙어 있으므로 남는다.
  - 비스듬한 바닥은 이웃 픽셀 depth 차가 커서(1 m 에서 3x3 안 10 cm, 260928 실측)
    '경계 튐' 기준 필터를 쓰면 바닥이 깎인다. 그래서 차이가 아니라 연결성만 본다.

260928 실측 (정지, dmax 1.0, 30프레임): 섬 프레임당 약 24개, 약 430 px(유효 픽셀의 0.3%),
전부 25 cm² 미만. 제거 후 바닥 위 5 cm 이상 점 1777 -> 0.
"""
import numpy as np
from scipy import ndimage

_S8 = np.ones((3, 3), bool)


def speckle_keep(dm, fx, min_area_m2=0.0025):
    """dm: (H,W) float [m], 0 = 무효 (범위 컷 이후). 반환: 남길 픽셀 mask (H,W) bool."""
    v = dm > 0
    if min_area_m2 <= 0 or not v.any():
        return v
    lab, n = ndimage.label(v, structure=_S8)
    cnt = np.bincount(lab.ravel(), minlength=n + 1)
    dsum = np.bincount(lab.ravel(), weights=dm.ravel(), minlength=n + 1)
    area = cnt * (dsum / np.maximum(cnt, 1) / fx) ** 2
    good = area >= min_area_m2
    good[0] = False
    return good[lab]
