"""DARON 경로계획 초안 (260930) — numpy·scipy 만, ROS 불필요.

입력: 판정 격자 한 장 (nav_msgs/OccupancyGrid 와 같은 배치)
      cost[행=y, 열=x] int, -1 = 미관측, 0~99 = 통과 비용, 100 = 불가
      resolution [m], origin 위치·쿼터니언 (격자 -> odom. 판정 노드는 중력 롤·피치를 여기에 넣는다)
      로버 위치·자세 (odom, base_link), 목표 (odom x, y)
출력: 경로 = 5 cm 간격 점 목록 (odom x, y, z, yaw, 진행 방향 +1 전진 / -1 후진)

두 가지 플래너
  astar_grid   : 8방향 격자 A*. 로버가 제자리 회전이 된다고 보면 이걸로 충분.
                 Hybrid A* 의 거리 추정(휴리스틱)과 격자 밖 목표 처리에도 쓴다.
  hybrid_astar : 최소 회전반경을 지키는 A* (자동차형, 후진 포함).
                 현성님 README 기준 조향 입력이 '차량 유효조향각 Δv (최대 43°)' 하나라
                 제자리 회전이 안 될 가능성이 커서 기본으로 둔다 [추정].

기준점 (중요, 현성님과 맞출 것)
  4륜 대칭 조향(앞 +Δa, 뒤 -Δa, tan Δa = tan Δv / 2)에서 옆미끄럼 없이 진행 방향으로 움직이는 점은
  앞·뒤 축의 가운데다. 곡률 κ = tan Δv / L 도 이 점 기준이다 (추종기의 Δv = atan(L·κ) 와 같음).
  그래서 계획은 '가운데 점(mid)'으로 하고, 필요하면 base_link(뒷축) 경로로 바꿔 내보낸다.
"""
import heapq
import math
import time
from dataclasses import dataclass, field

import numpy as np
from scipy import ndimage


# ------------------------------------------------------------------ 파라미터
@dataclass
class RoverParams:
    delta_max_deg: float = 43.003      # 차량 유효조향각 한계 [README 사실]
    wheelbase_m: float = 0.34          # [추정] CAD 그림에서 잰 값. 굴절각에 따라 변함 (/rover/diagnostics wheelbase_mm)
    length_m: float = 0.45             # 차체 길이 [9/28 초안]
    half_width_m: float = 0.146        # 차체 반폭 [9/28 초안]
    allow_reverse: bool = True         # 추종기가 후진·커스프를 지원함 [README 사실]

    @property
    def r_min(self):
        return self.wheelbase_m / math.tan(math.radians(self.delta_max_deg))

    @property
    def mid_from_base(self):           # base_link(뒷축) -> 두 축 가운데
        return self.wheelbase_m / 2.0

    def footprint_offsets(self):
        """가운데 점 기준, 진행 방향으로 몇 m 앞뒤 점을 검사할지.
        판정 격자는 벽·턱 둘레를 이미 약 15 cm(로버 반폭) 번지게 표시하므로(9/28 초안 3.5),
        반폭만큼의 원 여러 개로 차체 길이를 덮는다: 가운데 ± (반길이 - 반폭)."""
        a = max(0.0, self.length_m / 2 - self.half_width_m)
        return (-a, 0.0, a) if a > 1e-6 else (0.0,)


@dataclass
class PlanParams:
    inflate_m: float = 0.0     # 불가 칸 추가 부풀림. 판정이 이미 벽·턱을 약 15 cm 번지게 하므로 기본 0 (README 참고)
    soft_m: float = 0.15       # 불가 칸에서 이 거리 안이면 추가 비용 (딱 붙어 지나가지 않게)
    w_cost: float = 2.0        # 판정 비용 가중치: 비용 99 칸은 거리의 1 + 2 = 3배
    p_unknown: float = 1.5     # 미관측(-1) 칸 추가 비용 (비용 약 74 칸과 같음)
    w_near: float = 1.0        # soft_m 안 추가 비용 최대값
    goal_tol_m: float = 0.10   # 도착 판정 (인계 문서: 목표 10 cm 이내)
    escape_m: float = 0.15     # 출발 자리가 불가 칸이면 이 반경 안은 비싸게라도 통과 허용
    # Hybrid A*
    step_m: float = 0.10       # 한 번 확장할 때 움직이는 거리
    sub_m: float = 0.025       # 충돌 검사 간격
    n_steer: int = 5           # 조향 후보 수 (-Δmax ~ +Δmax)
    n_theta: int = 72          # 방향 칸 수 (5°)
    reverse_mult: float = 2.0  # 후진 거리 비용 배수
    switch_cost: float = 0.5   # 전·후진 전환 1회 비용 [m 환산]
    w_steer: float = 0.05      # 조향 크기 비용 (직진 선호)
    w_dsteer: float = 0.05     # 조향 변화 비용 (핸들 덜 흔들기)
    max_expand: int = 60000
    pad_m: float = 0.30        # 로버가 격자 밖일 때 로버 둘레로 덧붙일 미관측 여유
    goal_known: bool = True    # 목표 칸이 미관측이면 갈 수 있는 관측 칸 중 가장 가까운 칸을 임시 목표로 (OT: 경로의 33~57% 가 미관측)
    drop_deg: float = 10.0     # 높이가 있으면: 이 각도보다 가파르게 내려가다 미관측이 되는 가장자리 너머를
    drop_shadow_m: float = 0.30  #   이 거리까지 금지 (크레이터 안쪽 가려진 사면 대책, judge_core 낙차 의심과 같은 생각)
    out_step_m: float = 0.05   # 출력 경로 점 간격 (인계 문서: 5 cm)
    unk_cap_m: float = 0.30    # 출발 주변(blind_m) 밖에서 미관측 칸을 지나는 총 거리 상한. inf = 제한 없음
    blind_m: float = 0.40      #   출발 가운데 점에서 이 반경 안 미관측은 셈에서 뺌 (로버 자신·카메라 바로 앞 사각, OT: 미관측 경로점 대부분이 0.35 m 안)
    time_limit_s: float = math.inf   # Hybrid A* 시간 상한 (261004 추가, ROS 노드 1 s 상한용). inf = 기존과 같음


# ------------------------------------------------------------------ 좌표
def quat_to_R(q):
    x, y, z, w = (float(v) for v in q)
    n = math.sqrt(x * x + y * y + z * z + w * w) or 1.0
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def yaw_to_quat(yaw):
    return (0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2))


class Grid:
    """판정 격자 + 격자 평면 좌표 (x = 열 방향, y = 행 방향, 단위 m, 원점 = 칸 [0,0] 모서리)."""

    def __init__(self, cost, resolution, origin_p=(0, 0, 0), origin_q=(0, 0, 0, 1)):
        self.cost = np.asarray(cost).astype(np.int16)
        assert self.cost.ndim == 2, 'cost 는 (ny, nx) 2차원이어야 함'
        self.res = float(resolution)
        self.o = np.asarray(origin_p, float)
        self.R = quat_to_R(origin_q)          # 격자 -> odom 회전 (중력 기울기 포함)
        self.ny, self.nx = self.cost.shape

    # odom <-> 격자 평면
    def odom_to_g(self, p):
        return ((np.asarray(p, float) - self.o) @ self.R)[..., :2]

    def g_to_odom(self, xy):
        xy = np.asarray(xy, float)
        g = np.concatenate([xy, np.zeros(xy.shape[:-1] + (1,))], axis=-1)
        return self.o + g @ self.R.T

    def heading_g(self, R_body):
        """로버 몸체 x 축을 격자 평면에 투영한 방향 [rad]."""
        fx = self.R.T @ np.asarray(R_body)[:, 0]
        return math.atan2(fx[1], fx[0])

    def yaw_odom(self, th_g):
        d = self.R @ np.array([math.cos(th_g), math.sin(th_g), 0.0])
        return math.atan2(d[1], d[0])

    # 칸
    def cell(self, xy):
        return int(math.floor(xy[1] / self.res)), int(math.floor(xy[0] / self.res))   # (행, 열)

    def center(self, rc):
        return np.array([(rc[1] + 0.5) * self.res, (rc[0] + 0.5) * self.res])

    def inside(self, rc):
        return 0 <= rc[0] < self.ny and 0 <= rc[1] < self.nx

    def padded_to_include(self, pts_xy, margin_m):
        """격자 평면 점들(± margin)이 모두 들어가도록 미관측(-1) 칸을 덧붙인 새 격자.
        판정 격자는 관측된 곳만 담아서 로버 자신(카메라 뒤·바로 앞 사각)이 격자 밖인 경우가 많다 [사실, 9/30 bag]."""
        pts = np.atleast_2d(np.asarray(pts_xy, float))
        lo = np.floor((pts.min(axis=0) - margin_m) / self.res).astype(int)     # (x칸, y칸)
        hi = np.ceil((pts.max(axis=0) + margin_m) / self.res).astype(int)
        px0, py0 = max(0, -lo[0]), max(0, -lo[1])
        px1, py1 = max(0, hi[0] - self.nx), max(0, hi[1] - self.ny)
        if px0 == py0 == px1 == py1 == 0:
            return self
        cost = np.pad(self.cost, ((py0, py1), (px0, px1)), constant_values=-1)
        o = self.o + self.R @ np.array([-px0 * self.res, -py0 * self.res, 0.0])
        g = Grid.__new__(Grid)
        g.cost, g.res, g.o, g.R = cost, self.res, o, self.R
        g.ny, g.nx = cost.shape
        g.padded = (py0, py1, px0, px1)
        return g


# ------------------------------------------------------------------ 비용 지도
def _disk(k):
    yy, xx = np.mgrid[-k:k + 1, -k:k + 1]
    return (xx * xx + yy * yy) <= k * k


def local_grad(height, ok, rad=3, min_n=8):
    """각 칸 둘레 반경 rad 칸 안 ok 칸으로 평면 z = a + b*열 + c*행 최소제곱 (judge_core 의 R_FIT=3 과 같은 크기).
    반환 (b, c) [m/칸] (점이 min_n 보다 적거나 ok 가 아니면 NaN)."""
    K = _disk(rad).astype(float)
    yy, xx = np.mgrid[-rad:rad + 1, -rad:rad + 1]
    m = ok.astype(float)
    z = np.where(ok, height, 0.0)
    C = lambda A, W: ndimage.correlate(A, W, mode='constant', cval=0.0)     # out[i] = sum W[k] A[i + k]
    n, su, sv = C(m, K), C(m, K * xx), C(m, K * yy)
    suu, svv, suv = C(m, K * xx * xx), C(m, K * yy * yy), C(m, K * xx * yy)
    sz, suz, svz = C(z, K), C(z, K * xx), C(z, K * yy)
    A = np.stack([np.stack([n, su, sv], -1), np.stack([su, suu, suv], -1), np.stack([sv, suv, svv], -1)], -2)
    B = np.stack([sz, suz, svz], -1)
    good = ok & (n >= min_n)
    sol = np.full(B.shape, np.nan)
    if good.any():
        Ag, Bg = A[good], B[good]
        fine = np.abs(np.linalg.det(Ag)) > 1e-6
        x = np.full(Bg.shape, np.nan)
        x[fine] = np.linalg.solve(Ag[fine], Bg[fine][..., None])[..., 0]
        sol[good] = x
    return sol[..., 1], sol[..., 2]


def drop_shadow(height, unknown, res, drop_deg=10.0, shadow_m=0.30, lethal=None, fit_r=3, min_n=8):
    """관측 칸 -> 미관측 칸으로 넘어가는 가장자리 중, 그 칸 둘레 평면이 미관측 쪽으로 drop_deg 보다 가파르게 내려가는 곳을 찾아
    그 가장자리에서 shadow_m 안의 미관측 칸을 반환 (bool). height 는 cost 와 같은 배치, 미관측 = NaN.
    - lethal (불가 칸) 은 평면 맞춤·가장자리에서 뺌: 벽·상자 뒷면에서 높이가 뚝 떨어지는 것을 크레이터 가장자리로 오인하지 않게
    - 한 칸끼리 높이 차 대신 반경 15 cm 평면 기울기: 1 m 근처 depth 잡음에 덜 흔들림
    [사실, 260930 검사] 9/30 평지 bag 관측 가장자리 30 cm 띠 중 금지: 한 칸 차이 방식 50% -> 불가 제외 25% -> 평면 기울기 9%.
    합성 크레이터(12° 로 보이다 35° 로 가려짐) 9 경우 모두 가려진 사면을 피함. 12° 기준이면 가까운 경우 뚫림."""
    known = np.isfinite(height) & ~unknown
    ok = known & ~lethal if lethal is not None else known
    b, c = local_grad(height, ok, fit_r, min_n)
    thr = math.tan(math.radians(drop_deg)) * res
    ny, nx = height.shape
    edge = np.zeros_like(known)
    for dr, dc in ((0, 1), (0, -1), (1, 0), (-1, 0), (1, 1), (1, -1), (-1, 1), (-1, -1)):
        nxt = np.zeros_like(known)                      # 이 방향 바로 다음 칸이 미관측
        r0, r1 = max(0, -dr), ny - max(0, dr)
        c0, c1 = max(0, -dc), nx - max(0, dc)
        nxt[r0:r1, c0:c1] = unknown[r0 + dr:r1 + dr, c0 + dc:c1 + dc]
        slope = (b * dc + c * dr) / math.hypot(dr, dc)  # 그 방향 칸당 높이 변화
        with np.errstate(invalid='ignore'):
            edge |= ok & nxt & (slope < -thr)
    if not edge.any():
        return np.zeros_like(known)
    k = int(math.ceil(shadow_m / res))
    return ndimage.binary_dilation(edge, _disk(k)) & unknown


class CostMap:
    def __init__(self, grid: Grid, pp: PlanParams, height=None):
        c = grid.cost
        self.g, self.pp = grid, pp
        self.lethal = c >= 100
        self.unknown = c < 0
        self.drop = (drop_shadow(height, self.unknown, grid.res, pp.drop_deg, pp.drop_shadow_m, lethal=self.lethal)
                     if height is not None and pp.drop_shadow_m > 0 else np.zeros(c.shape, bool))
        k = int(round(pp.inflate_m / grid.res))
        self.forbid = ndimage.binary_dilation(self.lethal, _disk(k)) if (k > 0 and self.lethal.any()) else self.lethal.copy()
        self.forbid |= self.drop
        if self.forbid.any():
            self.clear = ndimage.distance_transform_edt(~self.forbid) * grid.res   # 금지 칸 중심까지 거리
        else:
            self.clear = np.full(c.shape, np.inf)
        pen = np.where(self.unknown, pp.p_unknown, pp.w_cost * np.clip(c, 0, 99) / 99.0)
        if pp.soft_m > 0:
            pen = pen + pp.w_near * np.clip(1.0 - (self.clear - grid.res) / pp.soft_m, 0.0, 1.0) * (self.clear < np.inf)
        self.pen = np.where(self.forbid, np.inf, pen)
        self.escaped = 0

    def allow_escape(self, xy, radius_m):
        """출발 자리가 금지 칸이면(벽 옆 띠 안, 잡음 등) 그 둘레만 비싸게 통과 허용."""
        g = self.g
        r0, c0 = g.cell(xy)
        k = int(math.ceil(radius_m / g.res))
        n = 0
        for dr in range(-k, k + 1):
            for dc in range(-k, k + 1):
                r, c = r0 + dr, c0 + dc
                if g.inside((r, c)) and dr * dr + dc * dc <= k * k and not np.isfinite(self.pen[r, c]):
                    self.pen[r, c] = 5.0
                    n += 1
        self.escaped = n
        return n

    def free(self, rc):
        return self.g.inside(rc) and np.isfinite(self.pen[rc])


# ------------------------------------------------------------------ 8방향 격자 탐색
_NB = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
       (-1, -1, math.sqrt(2)), (-1, 1, math.sqrt(2)), (1, -1, math.sqrt(2)), (1, 1, math.sqrt(2))]


def _neighbors(cm, rc):
    r, c = rc
    for dr, dc, L in _NB:
        n = (r + dr, c + dc)
        if not cm.free(n):
            continue
        if dr and dc and not (cm.free((r + dr, c)) and cm.free((r, c + dc))):
            continue                                            # 대각선으로 모서리 끼고 돌지 않기
        yield n, L * cm.g.res * (1.0 + 0.5 * (cm.pen[rc] + cm.pen[n]))


def dijkstra(cm, sources):
    """sources 에서 모든 칸까지 최소 비용 [m 환산]. 도달 못 하면 inf."""
    D = np.full(cm.pen.shape, np.inf)
    h = []
    for s in sources:
        if cm.free(s):
            D[s] = 0.0
            heapq.heappush(h, (0.0, s))
    while h:
        d, u = heapq.heappop(h)
        if d > D[u]:
            continue
        for v, w in _neighbors(cm, u):
            nd = d + w
            if nd < D[v]:
                D[v] = nd
                heapq.heappush(h, (nd, v))
    return D


def astar_grid(cm, start_rc, goal_rc):
    """8방향 A*. 반환: 칸 목록 (행, 열) 또는 None."""
    g = cm.g
    if not (cm.free(start_rc) and cm.free(goal_rc)):
        return None
    gc = g.center(goal_rc)
    hfun = lambda rc: float(np.hypot(*(g.center(rc) - gc)))
    G = {start_rc: 0.0}
    par = {start_rc: None}
    h = [(hfun(start_rc), 0.0, start_rc)]
    while h:
        f, d, u = heapq.heappop(h)
        if u == goal_rc:
            path = []
            while u is not None:
                path.append(u)
                u = par[u]
            return path[::-1]
        if d > G[u]:
            continue
        for v, w in _neighbors(cm, u):
            nd = d + w
            if nd < G.get(v, np.inf):
                G[v] = nd
                par[v] = u
                heapq.heappush(h, (nd + hfun(v), nd, v))
    return None


def unknown_reach(cm, start_rc, count):
    """출발 칸에서 각 칸까지 가는 동안 count 칸(셈에 넣는 미관측)을 지나는 최소 거리 [m]. 금지 칸은 못 지남."""
    U = np.full(cm.pen.shape, np.inf)
    if not cm.free(start_rc):
        return U
    U[start_rc] = 0.0
    h = [(0.0, start_rc)]
    res = cm.g.res
    while h:
        d, u = heapq.heappop(h)
        if d > U[u]:
            continue
        for v, _ in _neighbors(cm, u):
            nd = d + (res * math.hypot(v[0] - u[0], v[1] - u[1]) if count[v] else 0.0)
            if nd < U[v]:
                U[v] = nd
                heapq.heappush(h, (nd, v))
    return U


def choose_goal(cm, start_rc, goal_xy, prefer_known=True, U=None, cap=math.inf):
    """목표가 격자 밖이거나 갈 수 없는 칸이면, 출발점에서 갈 수 있는 칸 중 목표에 가장 가까운 칸을 임시 목표로.
    반환: (목표 칸, 임시 목표인지, 출발점 기준 거리 지도)"""
    g = cm.g
    D = dijkstra(cm, [start_rc])
    if U is not None and math.isfinite(cap):
        D = np.where(U <= cap + 1e-9, D, np.inf)       # 미관측을 cap 넘게 지나야 닿는 칸은 못 가는 칸으로
    grc = g.cell(goal_xy)
    if g.inside(grc) and np.isfinite(D[grc]) and not (prefer_known and cm.unknown[grc] and (np.isfinite(D) & ~cm.unknown).any()):
        return grc, False, D
    reach = np.isfinite(D)
    if prefer_known and (reach & ~cm.unknown).any():
        reach = reach & ~cm.unknown                     # 가능하면 관측된 칸에 임시 목표
    if not reach.any():
        return None, True, D
    rr, cc = np.nonzero(reach)
    cx, cy = (cc + 0.5) * g.res, (rr + 0.5) * g.res
    score = np.hypot(cx - goal_xy[0], cy - goal_xy[1]) + 0.05 * D[rr, cc]
    i = int(np.argmin(score))
    return (int(rr[i]), int(cc[i])), True, D


# ------------------------------------------------------------------ Hybrid A*
@dataclass
class _Node:
    x: float
    y: float
    th: float
    g: float
    parent: int
    d: int          # +1 전진, -1 후진, 0 출발
    steer: float
    seg: list = field(default_factory=list)   # 부모에서 여기까지 세부 점 (x, y, th)
    u: float = 0.0  # 지나온 미관측(셈에 넣는 칸) 거리 [m]


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def _pose_ok(cm, x, y, th, offs):
    """차체 검사점(가운데 점 ± 오프셋)이 모두 금지 칸이 아닌지. 가운데 점은 격자 안이어야 함."""
    g = cm.g
    for k, a in enumerate(offs):
        px, py = x + a * math.cos(th), y + a * math.sin(th)
        rc = g.cell((px, py))
        if not g.inside(rc):
            if a == 0.0:
                return False
            continue                                   # 앞·뒤 검사점이 격자 밖이면 미관측으로 봄
        if not np.isfinite(cm.pen[rc]):
            return False
    return True


def hybrid_astar(cm, start_xyth, goal_xy, rp: RoverParams, pp: PlanParams, h2d=None,
                 goal_th=None, th_tol=math.radians(10.0), shot_dist=0.0, shot_every=0, h_weight=1.0, shot_free_pen=0.05):
    """최소 회전반경을 지키는 A*. 반환: dict(poses=[(x, y, th, d)], ...) 또는 실패 시 poses=None.
    [261011 방위각] goal_th(격자 평면 방향 [rad])를 주면 도착 = 위치 goal_tol 안 그리고 방향 오차 th_tol 안.
      휴리스틱 = max(직선 거리, 장애물 고려 2D 거리 h2d, 장애물 무시 Dubins 거리) (Dolgov 2008).
      shot_every > 0 이면 해석적 확장: 목표 shot_dist 안 노드에서(shot_every 번 확장마다, 회전반경 2배 안이면 매번)
      목표까지 Dubins 곡선(전진만)을 그어 본다. [10/11 교수님 '좌우 경로, 최소 비용인가?'] 차체 검사를 통과한 꼴마다
      비용(길이 x (1 + 평균 지형 비용) + 조향 비용, 펼칠 때와 같은 식)을 매겨 '목표 후보'로 줄에 넣고, 줄에서 그 후보가
      가장 앞에 올 때 끝낸다 (out['shot'] = True). 예전엔 처음 통과한 가장 짧은 곡선에서 바로 끝내 지형 비용을 안 봤음.
      단 이 노드에서 가장 싼 꼴이 거의 공짜 칸만 지나면(평균 지형 비용 <= shot_free_pen) 예전처럼 바로 끝냄 (계산 시간: 리뷰 10/11,
      다 미루면 0.5 s 한도에 걸리는 판이 40판 중 5 → 9). 한도(시간·횟수)에 걸리면 그때까지 찾은 가장 싼 목표 후보를 씀.
      h_weight > 1 이면 가중 A*(휴리스틱 x h_weight): 최적보다 조금 길 수 있지만 훨씬 덜 펼침 (Python 계산 시간 대책)."""
    t0 = time.time()
    g = cm.g
    L, dmax = rp.wheelbase_m, math.radians(rp.delta_max_deg)
    steers = np.linspace(-dmax, dmax, pp.n_steer)
    dirs = (1, -1) if rp.allow_reverse else (1,)
    offs = rp.footprint_offsets()
    nsub = max(1, int(round(pp.step_m / pp.sub_m)))
    dth = 2 * math.pi / pp.n_theta
    gx, gy = goal_xy
    if h2d is None:
        h2d = dijkstra(cm, [g.cell(goal_xy)])

    def key(x, y, th):
        return (int(math.floor(y / g.res)), int(math.floor(x / g.res)), int(((th % (2 * math.pi)) / dth)) % pp.n_theta)

    use_th = goal_th is not None
    if use_th:
        import dubins as DB                                 # 지연 import: 방향 없는 계획은 표를 안 만듦
        rho = rp.r_min
        htab = DB.heur_table(rho)

    def heur(x, y, th=0.0):
        rc = g.cell((x, y))
        e = math.hypot(x - gx, y - gy)
        hv = h2d[rc] if g.inside(rc) else np.inf
        if not np.isfinite(hv):
            return np.inf
        if use_th:
            return max(e, hv, htab(x, y, th, gx, gy, goal_th))
        return max(e, hv)

    def at_goal(x, y, th):
        if math.hypot(x - gx, y - gy) > pp.goal_tol_m:
            return False
        return (not use_th) or abs(_wrap(th - goal_th)) <= th_tol

    sgn = {'L': 1.0, 'R': -1.0, 'S': 0.0}

    def shot(n, g_cap):
        """노드 n 에서 목표까지 Dubins 곡선 꼴들. 차체 검사를 통과한 꼴마다 (부모 기준 추가 비용, 세부 점 목록, 단어).
        g_cap: 이미 찾은 가장 싼 목표 후보의 총비용 — n.g + 길이가 이걸 넘는 꼴은 볼 필요 없음 (비용 >= 길이)."""
        out_ = []
        cands = DB.all_words((n.x, n.y, n.th), (gx, gy, goal_th), rho)
        if not cands:
            return out_
        L0 = cands[0][0]
        for L, w, tpq in cands:
            if L > shot_dist + 3 * rho or L > 2.0 * L0 + 1e-9 or n.g + L >= g_cap:
                break
            seg = DB.sample((n.x, n.y, n.th), w, tpq, rho, pp.sub_m)
            pen, ok = 0.0, True
            for (x, y, th) in seg:
                if not _pose_ok(cm, x, y, th, offs):
                    ok = False
                    break
                pen += cm.pen[g.cell((x, y))]
            if not ok or not seg:
                continue
            arc = sum(tpq[k] for k, ch in enumerate(w) if ch != 'S') * rho
            st_seq = [n.steer / dmax if dmax > 0 else 0.0] + [sgn[ch] for ch, v in zip(w, tpq) if v * rho > 1e-6]
            dsteer = sum(abs(b - a) for a, b in zip(st_seq[:-1], st_seq[1:]))
            c = L * (1.0 + pen / len(seg)) + pp.w_steer * arc + pp.w_dsteer * dsteer
            out_.append((c, seg, w, pen / len(seg)))
        return out_

    cnt = getattr(cm, 'unk_count', None)                  # 미관측 상한 (plan() 이 정함). 상한은 탐색 가지치기라
    cap = getattr(cm, 'unk_cap', math.inf)                #   같은 칸·방향의 더 싼 노드가 먼저 있으면 미관측이 적은 대안을 놓칠 수 있음
    sx, sy, sth = start_xyth
    nodes = [_Node(sx, sy, sth, 0.0, -1, 0, 0.0, [(sx, sy, sth)])]
    best = {key(sx, sy, sth): 0.0}
    h0 = heur(sx, sy, sth)
    heap = [(h_weight * h0 if np.isfinite(h0) else 0.0, 0)]
    nexp = 0
    goal_i = None
    out_timeout = False
    shot_seg, n_shot = None, 0
    shot_ids, best_shot = set(), None                     # 목표 후보(곡선으로 목표까지) 노드 번호들, 가장 싼 것 (총비용, 번호)
    shot_word = None
    while heap:
        _, i = heapq.heappop(heap)
        n = nodes[i]
        if i in shot_ids:                                 # 목표 후보가 줄 맨 앞 = 남은 어떤 후보보다 싸다 (가중 A* 기준) → 끝
            goal_i, shot_seg, shot_word = i, n.seg, getattr(n, 'word', None)
            break
        if n.g > best.get(key(n.x, n.y, n.th), np.inf) + 1e-9:
            continue
        if at_goal(n.x, n.y, n.th):
            goal_i = i
            break
        if use_th and shot_every > 0:
            dg = math.hypot(n.x - gx, n.y - gy)
            if dg <= shot_dist and (nexp % shot_every == 0 or dg <= 2 * rho):
                n_shot += 1
                cands_ = sorted(shot(n, best_shot[0] if best_shot else np.inf), key=lambda c: c[0])
                for c_, sg, w_, pm_ in cands_:
                    gs = n.g + c_
                    if best_shot is None or gs < best_shot[0] - 1e-9:
                        nd = _Node(gx, gy, goal_th, gs, i, 1, 0.0, list(sg), n.u)
                        nd.word = w_
                        nodes.append(nd)
                        shot_ids.add(len(nodes) - 1)
                        best_shot = (gs, len(nodes) - 1)
                        heapq.heappush(heap, (gs, len(nodes) - 1))
                if cands_ and cands_[0][3] <= shot_free_pen and best_shot is not None and nodes[best_shot[1]].parent == i:
                    goal_i = best_shot[1]                         # 이 노드의 가장 싼 꼴이 거의 공짜 칸만 지남 → 바로 끝 (예전과 같은 속도)
                    shot_seg, shot_word = nodes[goal_i].seg, getattr(nodes[goal_i], 'word', None)
                    break
        nexp += 1
        if nexp > pp.max_expand:
            break
        if (nexp & 31) == 0 and time.time() - t0 > pp.time_limit_s:   # 목표 근처는 뻗을 때마다 곡선을 여러 개 그어 봐서 자주 확인
            out_timeout = True
            break
        for d in dirs:
            for st in steers:
                x, y, th = n.x, n.y, n.th
                seg, ok, pen_sum, uu = [], True, 0.0, n.u
                ds = d * pp.step_m / nsub
                for _ in range(nsub):
                    th = _wrap(th + ds * math.tan(st) / L)
                    x += ds * math.cos(th)
                    y += ds * math.sin(th)
                    if not _pose_ok(cm, x, y, th, offs):
                        ok = False
                        break
                    rc = g.cell((x, y))
                    pen_sum += cm.pen[rc]
                    if cnt is not None and cnt[rc]:
                        uu += pp.step_m / nsub
                        if uu > cap + 1e-9:
                            ok = False
                            break
                    seg.append((x, y, th))
                    if at_goal(x, y, th):
                        break                               # 한 걸음 도중에 목표 도달 -> 거기서 끊음
                if not ok:
                    continue
                cost = len(seg) * (pp.step_m / nsub) * (1.0 + pen_sum / len(seg))
                if d < 0:
                    cost *= pp.reverse_mult
                cost += pp.w_steer * abs(st) / dmax * pp.step_m + pp.w_dsteer * abs(st - n.steer) / dmax
                if n.d != 0 and d != n.d:
                    cost += pp.switch_cost
                ng = n.g + cost
                k = key(x, y, th)
                if ng >= best.get(k, np.inf) - 1e-9:
                    continue
                hv = heur(x, y, th)
                if not np.isfinite(hv):
                    continue
                best[k] = ng
                nodes.append(_Node(x, y, th, ng, i, d, st, seg, uu))
                heapq.heappush(heap, (ng + h_weight * hv, len(nodes) - 1))
    if goal_i is None and best_shot is not None:          # 한도에 걸렸거나 줄이 비었는데 목표 후보는 있음 → 그중 가장 싼 것
        goal_i = best_shot[1]
        shot_seg, shot_word = nodes[goal_i].seg, getattr(nodes[goal_i], 'word', None)
    out = dict(expanded=nexp, ms=(time.time() - t0) * 1000, poses=None, cost=None, timeout=out_timeout,
               shot=shot_seg is not None, n_shot=n_shot, shot_word=shot_word, n_shot_cand=len(shot_ids))
    if goal_i is None:
        return out
    chain = []
    i = goal_i
    while i >= 0:
        chain.append(i)
        i = nodes[i].parent
    chain = chain[::-1]
    poses = [(sx, sy, sth, nodes[chain[1]].d if len(chain) > 1 else 1)]
    steer_seq = []
    for j in chain[1:]:
        n = nodes[j]
        for (x, y, th) in n.seg:
            poses.append((x, y, th, n.d))
        steer_seq.append(n.steer)                         # (목표 후보 노드의 seg = 곡선 세부 점이 이 고리에서 같이 붙음)
    out.update(poses=poses, cost=nodes[goal_i].g, steers=steer_seq)
    return out


# ------------------------------------------------------------------ 경로 후처리·지표
def _resample_run(run, step):
    """방향이 같은 한 구간을 호 길이 step 간격으로. 첫 점·끝 점 포함."""
    xy = np.array([(p[0], p[1]) for p in run])
    th = np.unwrap(np.array([p[2] for p in run]))
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))]) if len(run) > 1 else np.array([0.0])
    if s[-1] < 1e-9:
        return [run[0]]
    ss = np.arange(0.0, s[-1], step)
    if s[-1] - ss[-1] > 1e-6:
        ss = np.append(ss, s[-1])
    d = run[-1][3]
    return [(float(np.interp(v, s, xy[:, 0])), float(np.interp(v, s, xy[:, 1])),
             float(_wrap(np.interp(v, s, th))), d) for v in ss]


def resample(poses, step):
    """(x, y, th, d) 목록을 진행 방향이 같은 구간(run)별로 step 간격으로 다시 찍기.
    전·후진 전환점(커스프)은 앞 구간의 끝 점이자 다음 구간의 첫 점으로 한 번만 남긴다."""
    if not poses:
        return []
    runs, cur = [], [poses[0]]
    for p in poses[1:]:
        if p[3] != cur[-1][3] and len(cur) > 1:
            runs.append(cur)
            cur = [cur[-1][:3] + (p[3],), p]          # 전환점에서 새 구간 시작
        else:
            cur.append(p)
    runs.append(cur)
    out = []
    for run in runs:
        pts = _resample_run(run, step)
        if out and math.hypot(pts[0][0] - out[-1][0], pts[0][1] - out[-1][1]) < 1e-6:
            pts = pts[1:]
        out.extend(pts)
    return out


def smooth_ma(xy, w=5):
    """이동평균 (추종기의 '계단 다듬기' 흉내, 창 크기는 [추정])."""
    xy = np.asarray(xy, float)
    if len(xy) < w or w < 2:
        return xy
    k = w // 2
    pad = np.vstack([np.repeat(xy[:1], k, 0), xy, np.repeat(xy[-1:], k, 0)])
    ker = np.ones(w) / w
    return np.stack([np.convolve(pad[:, i], ker, mode='valid') for i in range(2)], axis=1)


def max_curvature(xy, d=None):
    xy = np.asarray(xy, float)
    kmax = 0.0
    for i in range(1, len(xy) - 1):
        if d is not None and not (d[i - 1] == d[i] == d[i + 1]):
            continue
        a, b = xy[i] - xy[i - 1], xy[i + 1] - xy[i]
        la, lb = np.hypot(*a), np.hypot(*b)
        if la < 1e-6 or lb < 1e-6:
            continue
        ang = abs(_wrap(math.atan2(b[1], b[0]) - math.atan2(a[1], a[0])))
        kmax = max(kmax, ang / (0.5 * (la + lb)))
    return kmax


def cells_to_poses(g, cells):
    pts = [g.center(rc) for rc in cells]
    poses = []
    for i, p in enumerate(pts):
        q = pts[min(i + 1, len(pts) - 1)] if i + 1 < len(pts) else pts[i]
        pr = pts[i - 1] if i > 0 else pts[i]
        v = (q - pr) if np.any(q != pr) else np.array([1.0, 0.0])
        poses.append((p[0], p[1], math.atan2(v[1], v[0]), 1))
    return poses


def path_metrics(cm, poses, rp: RoverParams, smooth_w=0):
    """경로 지표. smooth_w>0 이면 이동평균 뒤 곡률(격자 경로용)."""
    if not poses:
        return dict(found=False)
    xy = np.array([(p[0], p[1]) for p in poses])
    d = np.array([p[3] for p in poses])
    length = float(np.sum(np.hypot(*np.diff(xy, axis=0).T))) if len(xy) > 1 else 0.0
    cusps = int(np.sum((d[1:] != d[:-1]) & (d[1:] != 0) & (d[:-1] != 0)))
    kmax = max_curvature(smooth_ma(xy, smooth_w) if smooth_w else xy, d)
    g = cm.g
    dl = ndimage.distance_transform_edt(~cm.lethal) * g.res if cm.lethal.any() else None
    inside = [g.cell(p) for p in xy if g.inside(g.cell(p))]
    clear = [float(dl[rc]) for rc in inside] if dl is not None else []
    hits = sum(1 for rc in inside if cm.lethal[rc])
    unk = len(xy) - len(inside) + sum(1 for rc in inside if cm.unknown[rc])
    return dict(found=True, length_m=length, cusps=cusps, kmax=kmax, r_min_ok=bool(kmax <= 1.03 / rp.r_min),
                min_clear_m=min(clear) if clear else np.inf, lethal_hits=hits, unknown_pts=unk, n=len(poses))


def to_odom_path(g: Grid, poses, rp: RoverParams, ref='mid'):
    """격자 평면 경로 -> odom (x, y, z, yaw, d). ref='base_link' 면 뒷축 점으로 바꿔서."""
    out = []
    for x, y, th, d in poses:
        if ref == 'base_link':
            x, y = x - rp.mid_from_base * math.cos(th), y - rp.mid_from_base * math.sin(th)
        p = g.g_to_odom(np.array([x, y]))
        out.append((float(p[0]), float(p[1]), float(p[2]), g.yaw_odom(th), int(d)))
    return out


# ------------------------------------------------------------------ 한 번에
def plan(grid: Grid, rover_p, rover_R, goal_odom_xy, rp=None, pp=None, planners=('hybrid', 'grid'), height=None):
    """판정 격자 한 장 + 로버 자세 + 목표 -> 경로.
    rover_p, rover_R: base_link 위치(odom)와 회전행렬. goal_odom_xy: odom (x, y).
    반환 dict: start/goal 정보, 'hybrid'/'grid' 결과 (격자 평면 poses, odom 경로, 지표)."""
    rp, pp = rp or RoverParams(), pp or PlanParams()
    th = grid.heading_g(rover_R)
    base_xy = grid.odom_to_g(rover_p)
    mid_xy = base_xy + rp.mid_from_base * np.array([math.cos(th), math.sin(th)])
    # 로버가 격자 밖이거나 가장자리에 걸치면 미관측 칸을 덧붙여 격자를 넓힘
    grid = grid.padded_to_include(mid_xy, rp.length_m / 2 + pp.pad_m)
    if height is not None and getattr(grid, 'padded', None):
        py0, py1, px0, px1 = grid.padded
        height = np.pad(np.asarray(height, float), ((py0, py1), (px0, px1)), constant_values=np.nan)
    base_xy = grid.odom_to_g(rover_p)
    mid_xy = base_xy + rp.mid_from_base * np.array([math.cos(th), math.sin(th)])
    cm = CostMap(grid, pp, height=height)
    # 목표: odom (x, y) 를 격자 평면으로. z 는 로버 높이로 둠 (격자 평면 투영이라 영향 작음)
    gz = goal_odom_xy[2] if len(goal_odom_xy) > 2 else rover_p[2]    # 목표 z 를 알면 씀 (격자가 odom 과 크게 기울었을 때 중요)
    goal_p = np.array([goal_odom_xy[0], goal_odom_xy[1], gz])
    goal_xy = grid.odom_to_g(goal_p)
    res = dict(start_xy=mid_xy, start_th=th, goal_xy=goal_xy, cm=cm, g=grid,
               padded=getattr(grid, 'padded', (0, 0, 0, 0)))
    if not grid.inside(grid.cell(mid_xy)):
        res['error'] = '로버가 격자 밖'
        return res
    srow = grid.cell(mid_xy)
    if not cm.free(srow) or not _pose_ok(cm, mid_xy[0], mid_xy[1], th, rp.footprint_offsets()):
        # 로버가 지금 서 있는 자리는 실제로 갈 수 있는 곳이므로(잡음·벽 옆 띠 등) 차체 크기만큼만 풀어 줌
        cm.allow_escape(mid_xy, max(pp.escape_m, rp.length_m / 2))
    U = None
    if math.isfinite(pp.unk_cap_m):
        rr, cc = np.mgrid[0:grid.ny, 0:grid.nx]
        blind = np.hypot((cc + 0.5) * grid.res - mid_xy[0], (rr + 0.5) * grid.res - mid_xy[1]) <= pp.blind_m
        cm.unk_count, cm.unk_cap = cm.unknown & ~blind, pp.unk_cap_m
        U = unknown_reach(cm, srow, cm.unk_count)
    grc, temp, _ = choose_goal(cm, srow, goal_xy, prefer_known=pp.goal_known, U=U, cap=pp.unk_cap_m)
    res['goal_rc'], res['temp_goal'] = grc, temp
    if grc is None:
        res['error'] = '갈 수 있는 칸 없음'
        return res
    gxy = grid.center(grc) if temp else goal_xy
    res['plan_goal_xy'] = gxy
    h2d = dijkstra(cm, [grc])
    if 'grid' in planners:
        t0 = time.time()
        cells = astar_grid(cm, srow, grc)
        poses = cells_to_poses(grid, cells) if cells else None
        m = path_metrics(cm, poses, rp, smooth_w=5)
        m['ms'] = (time.time() - t0) * 1000
        res['grid'] = dict(poses=poses, metrics=m, odom=to_odom_path(grid, poses, rp) if poses else None)
    if 'hybrid' in planners:
        hr = hybrid_astar(cm, (mid_xy[0], mid_xy[1], th), gxy, rp, pp, h2d=h2d)
        poses = resample(hr['poses'], pp.out_step_m) if hr['poses'] else None
        m = path_metrics(cm, poses, rp)
        m.update(ms=hr['ms'], expanded=hr['expanded'])
        res['hybrid'] = dict(poses=poses, metrics=m, raw=hr, odom=to_odom_path(grid, poses, rp) if poses else None)
    return res
