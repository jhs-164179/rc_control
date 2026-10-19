"""Dubins 곡선 (261011) — 전진만·최소 회전반경 rho 인 차가 (x0, y0, θ0) 에서 (x1, y1, θ1) 로 가는 가장 짧은 길.

장애물이 없으면 가장 짧은 길은 항상 '원호-직선-원호'(LSL, RSR, LSR, RSL) 또는 '원호-원호-원호'(RLR, LRL) 6가지 중 하나
(Dubins 1957). 방위각 웨이포인트에서 두 곳에 씀:
  1) Hybrid A* 휴리스틱: 장애물 무시 Dubins 거리 (방향까지 맞추려면 최소 이만큼은 가야 함)
  2) 해석적 확장(analytic expansion): 탐색 중 노드에서 목표까지 Dubins 곡선을 그어 보고, 안 부딪히면 거기서 끝
  (Dolgov et al. 2008, Stanford Junior 의 Hybrid A* 와 같은 구성)
공식: Shkel & Lumelsky 2001 / A. Walker dubins.c 와 같은 꼴. 끝점이 맞는지는 test_planner 가 적분으로 확인.
numpy 만 씀.
"""
import math

import numpy as np

TWO_PI = 2 * math.pi
WORDS = ('LSL', 'RSR', 'LSR', 'RSL', 'RLR', 'LRL')


def _mod2pi(a):
    return a - TWO_PI * math.floor(a / TWO_PI)


def _words(alpha, beta, d):
    """정규화(rho = 1) 좌표에서 6가지 (t, p, q) 또는 None."""
    sa, sb, ca, cb = math.sin(alpha), math.sin(beta), math.cos(alpha), math.cos(beta)
    c_ab = math.cos(alpha - beta)
    out = {}
    # LSL
    p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sa - sb)
    if p_sq >= 0:
        tmp1 = math.atan2(cb - ca, d + sa - sb)
        out['LSL'] = (_mod2pi(tmp1 - alpha), math.sqrt(p_sq), _mod2pi(beta - tmp1))
    # RSR
    p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sb - sa)
    if p_sq >= 0:
        tmp1 = math.atan2(ca - cb, d - sa + sb)
        out['RSR'] = (_mod2pi(alpha - tmp1), math.sqrt(p_sq), _mod2pi(tmp1 - beta))
    # LSR
    p_sq = -2 + d * d + 2 * c_ab + 2 * d * (sa + sb)
    if p_sq >= 0:
        p = math.sqrt(p_sq)
        tmp0 = math.atan2(-ca - cb, d + sa + sb) - math.atan2(-2.0, p)
        out['LSR'] = (_mod2pi(tmp0 - alpha), p, _mod2pi(tmp0 - _mod2pi(beta)))
    # RSL
    p_sq = -2 + d * d + 2 * c_ab - 2 * d * (sa + sb)
    if p_sq >= 0:
        p = math.sqrt(p_sq)
        tmp0 = math.atan2(ca + cb, d - sa - sb) - math.atan2(2.0, p)
        out['RSL'] = (_mod2pi(alpha - tmp0), p, _mod2pi(beta - tmp0))
    # RLR
    tmp0 = (6.0 - d * d + 2 * c_ab + 2 * d * (sa - sb)) / 8.0
    if abs(tmp0) <= 1:
        phi = math.atan2(ca - cb, d - sa + sb)
        p = _mod2pi(TWO_PI - math.acos(tmp0))
        t = _mod2pi(alpha - phi + _mod2pi(p / 2.0))
        out['RLR'] = (t, p, _mod2pi(alpha - beta - t + _mod2pi(p)))
    # LRL
    tmp0 = (6.0 - d * d + 2 * c_ab + 2 * d * (sb - sa)) / 8.0
    if abs(tmp0) <= 1:
        phi = math.atan2(ca - cb, d + sa - sb)
        p = _mod2pi(TWO_PI - math.acos(tmp0))
        t = _mod2pi(-alpha - phi + p / 2.0)
        out['LRL'] = (t, p, _mod2pi(_mod2pi(beta) - alpha - t + _mod2pi(p)))
    return out


def _norm(q0, q1, rho):
    dx, dy = q1[0] - q0[0], q1[1] - q0[1]
    d = math.hypot(dx, dy) / rho
    th = _mod2pi(math.atan2(dy, dx)) if d > 1e-12 else 0.0
    return _mod2pi(q0[2] - th), _mod2pi(q1[2] - th), d


def shortest(q0, q1, rho):
    """반환 (길이 [m], 단어, (t, p, q) 정규화 길이). 못 찾으면 (inf, None, None) (실제로는 항상 있음)."""
    alpha, beta, d = _norm(q0, q1, rho)
    best = (math.inf, None, None)
    for w, tpq in _words(alpha, beta, d).items():
        L = (tpq[0] + tpq[1] + tpq[2]) * rho
        if L < best[0]:
            best = (L, w, tpq)
    return best


def all_words(q0, q1, rho):
    """[261011] 가능한 꼴 전부 [(길이 [m], 단어, (t, p, q))], 짧은 순 (같은 길이면 WORDS 순서 = 왼쪽 꼴 먼저)."""
    alpha, beta, d = _norm(q0, q1, rho)
    W = _words(alpha, beta, d)
    out = [((W[w][0] + W[w][1] + W[w][2]) * rho, w, W[w]) for w in WORDS if w in W]
    out.sort(key=lambda r: r[0])
    return out


def length(q0, q1, rho):
    return shortest(q0, q1, rho)[0]


def sample(q0, word, tpq, rho, step):
    """Dubins 곡선을 호 길이 step 간격으로 [(x, y, θ)] (첫 점 q0 제외, 끝점 포함). 원호는 정확히 적분."""
    x, y, th = q0
    out = []
    for ch, seg in zip(word, tpq):
        L = seg * rho
        if L <= 1e-12:
            continue
        k = {'L': 1.0 / rho, 'R': -1.0 / rho, 'S': 0.0}[ch]
        n = max(1, int(math.ceil(L / step)))
        ds = L / n
        for _ in range(n):
            if k == 0.0:
                x += ds * math.cos(th)
                y += ds * math.sin(th)
            else:
                th1 = th + k * ds
                x += (math.sin(th1) - math.sin(th)) / k
                y += (math.cos(th) - math.cos(th1)) / k
                th = th1
            out.append((x, y, (th + math.pi) % TWO_PI - math.pi))
    return out


# ------------------------------------------------------------------ 휴리스틱 표 (목표 기준 상대 자세 -> Dubins 거리)
_TABLE = {}


def _lengths_np(alpha, beta, d):
    """_words 의 numpy 판: 6가지 중 가장 짧은 정규화 길이 (배열)."""
    m2 = lambda a: np.mod(a, TWO_PI)                                    # noqa: E731
    sa, sb, ca, cb = np.sin(alpha), np.sin(beta), np.cos(alpha), np.cos(beta)
    c_ab = np.cos(alpha - beta)
    best = np.full(np.shape(d), np.inf)
    with np.errstate(invalid='ignore'):
        p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sa - sb)
        tmp1 = np.arctan2(cb - ca, d + sa - sb)
        L = m2(tmp1 - alpha) + np.sqrt(np.maximum(p_sq, 0)) + m2(beta - tmp1)
        best = np.where(p_sq >= 0, np.minimum(best, L), best)
        p_sq = 2 + d * d - 2 * c_ab + 2 * d * (sb - sa)
        tmp1 = np.arctan2(ca - cb, d - sa + sb)
        L = m2(alpha - tmp1) + np.sqrt(np.maximum(p_sq, 0)) + m2(tmp1 - beta)
        best = np.where(p_sq >= 0, np.minimum(best, L), best)
        p_sq = -2 + d * d + 2 * c_ab + 2 * d * (sa + sb)
        p = np.sqrt(np.maximum(p_sq, 0))
        tmp0 = np.arctan2(-ca - cb, d + sa + sb) - np.arctan2(-2.0, p)
        L = m2(tmp0 - alpha) + p + m2(tmp0 - m2(beta))
        best = np.where(p_sq >= 0, np.minimum(best, L), best)
        p_sq = -2 + d * d + 2 * c_ab - 2 * d * (sa + sb)
        p = np.sqrt(np.maximum(p_sq, 0))
        tmp0 = np.arctan2(ca + cb, d - sa - sb) - np.arctan2(2.0, p)
        L = m2(alpha - tmp0) + p + m2(beta - tmp0)
        best = np.where(p_sq >= 0, np.minimum(best, L), best)
        tmp0 = (6.0 - d * d + 2 * c_ab + 2 * d * (sa - sb)) / 8.0
        ok = np.abs(tmp0) <= 1
        phi = np.arctan2(ca - cb, d - sa + sb)
        p = m2(TWO_PI - np.arccos(np.clip(tmp0, -1, 1)))
        t = m2(alpha - phi + m2(p / 2.0))
        L = t + p + m2(alpha - beta - t + m2(p))
        best = np.where(ok, np.minimum(best, L), best)
        tmp0 = (6.0 - d * d + 2 * c_ab + 2 * d * (sb - sa)) / 8.0
        ok = np.abs(tmp0) <= 1
        phi = np.arctan2(ca - cb, d + sa - sb)
        p = m2(TWO_PI - np.arccos(np.clip(tmp0, -1, 1)))
        t = m2(-alpha - phi + p / 2.0)
        L = t + p + m2(m2(beta) - alpha - t + m2(p))
        best = np.where(ok, np.minimum(best, L), best)
    return best


class HeurTable:
    """목표 자세 기준 상대 자세 (x, y, θ) 칸 -> Dubins 거리 [m]. 표 밖은 직접 계산.
    칸 크기만큼 낮춰 둠(slack): 칸 안 어느 자세여도 실제 거리보다 크지 않게 (휴리스틱이 과대평가하지 않게) [계산, 근사]."""

    def __init__(self, rho, res=0.05, half=3.2, n_th=72):
        self.rho, self.res, self.half, self.n_th = rho, res, half, n_th
        n = int(round(2 * half / res)) + 1
        self.n = n
        c = -half + np.arange(n) * res
        th = np.arange(n_th) * TWO_PI / n_th
        X, Y, TH = np.meshgrid(c, c, th, indexing='ij')
        # 상대 자세 q0 = (X, Y, TH) 에서 q1 = (0, 0, 0)
        dx, dy = -X, -Y
        d = np.hypot(dx, dy) / rho
        ang = np.mod(np.arctan2(dy, dx), TWO_PI)
        alpha = np.mod(TH - ang, TWO_PI)
        beta = np.mod(-ang, TWO_PI)
        L = _lengths_np(alpha, beta, d) * rho
        slack = res * 0.75 + rho * (TWO_PI / n_th) * 0.5
        self.L = np.maximum(L - slack, 0.0).astype(np.float32)

    def __call__(self, x, y, th, gx, gy, gth):
        c, s = math.cos(gth), math.sin(gth)
        dx, dy = x - gx, y - gy
        rx, ry = c * dx + s * dy, -s * dx + c * dy
        i = int(round((rx + self.half) / self.res))
        j = int(round((ry + self.half) / self.res))
        if 0 <= i < self.n and 0 <= j < self.n:
            k = int(round(((th - gth) % TWO_PI) / (TWO_PI / self.n_th))) % self.n_th
            return float(self.L[i, j, k])
        return length((x, y, th), (gx, gy, gth), self.rho)


def heur_table(rho):
    key = round(rho, 4)
    if key not in _TABLE:
        _TABLE[key] = HeurTable(rho)
    return _TABLE[key]
