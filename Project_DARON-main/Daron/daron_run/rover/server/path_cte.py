#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
path_cte.py — 경로 전처리 + CTE 함수 (ROS 없음). CLAUDE.md §4.1 전처리, §4.2 계약, §4.2.1 곡률.

    path = prepare_path([(x, y), ...], init_search_m=0.5)    # None = 점 2개 미만 (빈 경로로 처리)
    e_y, e_th, kappa_ref, remaining = cte(path, x, y, psi, preview_m)

부호
----
    e_y       [m]   로버가 경로 왼쪽이면 +
    e_th      [rad] wrap(psi - 경로접선각), [-pi, pi). 로버가 반시계 쪽이면 +
    kappa_ref [1/m] 최근접점에서 preview_m 앞의 경로 곡률. 좌회전 +
    remaining [m]   최근접점부터 경로 끝까지 호 길이 (>= 0)

최근접점
--------
    - 경로를 받은 뒤 첫 호출: 앞쪽 init_search_m 안의 세그먼트에서 전역 최근접 (index 0 고정 아님).
    - 이후: 직전 세그먼트 기준 뒤 1개, 앞 WINDOW_AHEAD 개만 본다.
    - 세그먼트에 투영한다 (꼭짓점 선택 아님). 첫 세그먼트는 시작점 뒤로, 마지막 세그먼트는
      끝점 앞으로 연장해서 투영한다. 그래야 끝점을 지나도 e_y 가 순수 횡오차로 남는다.

접선각
------
    꼭짓점 접선 = 양옆 세그먼트 각의 평균, 세그먼트 안에서는 선형 보간한다. 세그먼트 각을
    그대로 쓰면 5 cm 마다 접선이 kappa * 0.05 (최대 7.8 deg) 씩 계단으로 바뀌어 heading
    피드백이 톱니가 된다. (CLAUDE.md 에 명시된 것은 아니고 이 구현의 선택이다.)
"""

from __future__ import annotations

import bisect
import math
from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

WINDOW_BACK = 1
WINDOW_AHEAD = 8
DUP_DIST_M = 1e-3          # 이보다 가까운 연속점은 중복점


def wrap(a: float) -> float:
    return (a + math.pi) % (2 * math.pi) - math.pi


@dataclass
class PreparedPath:
    x: List[float]
    y: List[float]
    s: List[float]                 # 꼭짓점 누적 호 길이 [m]
    seg_len: List[float]           # 세그먼트 길이 L[i] (점 i -> i+1)
    th: List[float]                # 세그먼트 접선각 th[i] (unwrap)
    th_v: List[float]              # 꼭짓점 접선각 (unwrap)
    k: List[float]                 # 꼭짓점 곡률 (3점 이동평균 후) [1/m]
    init_search_m: float = 0.5
    # ---- 탐색 상태 (cte 가 갱신) ----
    idx: int = 0                   # 직전 최근접 세그먼트
    need_init: bool = True
    init_idx: int = -1             # 첫 탐색이 고른 세그먼트 (진단용)
    s_proj: float = 0.0            # 직전 투영점의 호 길이

    @property
    def length(self) -> float:
        return self.s[-1]

    def __len__(self):
        return len(self.x)


def prepare_path(points: Sequence[Tuple[float, float]],
                 init_search_m: float = 0.5) -> Optional[PreparedPath]:
    """중복점 제거 후 s, th, k 를 미리 계산한다. 점이 2개 미만이면 None."""
    xs, ys = [], []
    for px, py in points:
        if not xs or math.hypot(px - xs[-1], py - ys[-1]) >= DUP_DIST_M:
            xs.append(float(px))
            ys.append(float(py))
    n = len(xs)
    if n < 2:
        return None

    seg_len, th, s = [], [], [0.0]
    for i in range(n - 1):
        dx, dy = xs[i + 1] - xs[i], ys[i + 1] - ys[i]
        L = math.hypot(dx, dy)
        a = math.atan2(dy, dx)
        if th:                                   # unwrap
            a = th[-1] + wrap(a - th[-1])
        seg_len.append(L)
        th.append(a)
        s.append(s[-1] + L)

    # 꼭짓점 접선: 양 끝은 붙은 세그먼트, 가운데는 평균
    th_v = [th[0]] + [0.5 * (th[i - 1] + th[i]) for i in range(1, n - 1)] + [th[-1]]

    # §4.2.1 곡률: k[i] = (th[i] - th[i-1]) / ((L[i-1] + L[i]) / 2), 양 끝은 이웃값 복사
    k = [0.0] * n
    for i in range(1, n - 1):
        k[i] = (th[i] - th[i - 1]) / (0.5 * (seg_len[i - 1] + seg_len[i]))
    if n >= 3:
        k[0], k[-1] = k[1], k[-2]
    # 3점 이동평균 (끝에서는 창을 줄인다)
    k = [sum(k[max(0, i - 1):i + 2]) / len(k[max(0, i - 1):i + 2]) for i in range(n)]

    return PreparedPath(xs, ys, s, seg_len, th, th_v, k, init_search_m=init_search_m)


def _project(path: PreparedPath, j: int, x: float, y: float):
    """세그먼트 j 에 투영. 반환 (d2, t). 첫/마지막 세그먼트는 바깥으로 연장한다."""
    x0, y0 = path.x[j], path.y[j]
    dx, dy = path.x[j + 1] - x0, path.y[j + 1] - y0
    L2 = dx * dx + dy * dy
    t = ((x - x0) * dx + (y - y0) * dy) / L2
    last = len(path.seg_len) - 1
    if t < 0.0 and j != 0:
        t = 0.0
    elif t > 1.0 and j != last:
        t = 1.0
    px, py = x0 + t * dx, y0 + t * dy
    return (x - px) ** 2 + (y - py) ** 2, t


def _best(path: PreparedPath, js, x: float, y: float):
    best = None
    for j in js:
        d2, t = _project(path, j, x, y)
        if best is None or d2 <= best[0]:        # 같으면 뒤쪽(진행 방향) 세그먼트
            best = (d2, j, t)
    return best


def _init_index(path: PreparedPath, x: float, y: float) -> int:
    """새 경로: 앞쪽 init_search_m 안의 세그먼트에서 전역 최근접."""
    js = [j for j in range(len(path.seg_len)) if path.s[j] <= path.init_search_m] or [0]
    return _best(path, js, x, y)[1]


def _interp(s_arr: List[float], v: List[float], sq: float) -> float:
    if sq <= s_arr[0]:
        return v[0]
    if sq >= s_arr[-1]:
        return v[-1]
    i = bisect.bisect_right(s_arr, sq) - 1
    t = (sq - s_arr[i]) / (s_arr[i + 1] - s_arr[i])
    return v[i] + t * (v[i + 1] - v[i])


def cte(path: PreparedPath, x: float, y: float, psi: float,
        preview_m: float) -> Tuple[float, float, float, float]:
    """CLAUDE.md §4.2 계약. path 의 탐색 상태(idx)를 갱신한다."""
    nseg = len(path.seg_len)
    if path.need_init:
        path.idx = path.init_idx = _init_index(path, x, y)
        path.need_init = False

    lo = max(0, path.idx - WINDOW_BACK)
    hi = min(nseg - 1, path.idx + WINDOW_AHEAD)
    _, j, t = _best(path, range(lo, hi + 1), x, y)
    path.idx = j

    x0, y0 = path.x[j], path.y[j]
    L = path.seg_len[j]
    ux, uy = (path.x[j + 1] - x0) / L, (path.y[j + 1] - y0) / L
    px, py = x0 + t * L * ux, y0 + t * L * uy
    e_y = ux * (y - py) - uy * (x - px)                      # 왼쪽 +

    tc = min(1.0, max(0.0, t))
    heading = path.th_v[j] + tc * (path.th_v[j + 1] - path.th_v[j])
    e_th = wrap(psi - heading)

    s_proj = path.s[j] + t * L
    path.s_proj = s_proj
    kappa_ref = _interp(path.s, path.k, s_proj + preview_m)
    remaining = max(0.0, path.s[-1] - s_proj)
    return e_y, e_th, kappa_ref, remaining
