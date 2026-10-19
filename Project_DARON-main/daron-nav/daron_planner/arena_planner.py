"""DARON 경기장 전체 격자 경로계획 (261004) — numpy·scipy 만, ROS 불필요. ROS 노드와 시뮬이 같이 쓴다.

부분적 given map(경기장 크기·랜더·출발점은 앎, 페이즈1 지형은 모름)에서 흔히 쓰는 방법:
  1) 경기장 전체 5 cm 격자(ArenaMap). 판정 격자(로버 반경 1.25 m)가 올 때마다 최근 값으로 덮어 누적.
  2) 사전 금지 칸: 경기장 경계(로버 반폭 + odom 오차 여유)와 랜더. 출발 위치·방향으로 odom 에 놓는다.
  3) 모르는 칸(-1)은 '갈 수 있다'고 가정(비용만 높게) -> 8방향 최단 경로(Dijkstra, A*와 같은 답)로 진짜 목표까지.
     임시 목표("판정 범위 안에서 목표에 가장 가까운 관측 칸")가 없다. 새 판정이 오면 처음부터 다시 계획.
  4) 회전반경 처리 (method)
       'raw'    : 칸 경로 그대로 (비교용. 모서리에서 꺾여 최소 회전반경을 안 지킴)
       'smooth' : 칸 경로를 추종기 흉내(pure pursuit, 곡률 상한 1/r_min)로 앞으로 굴려 본 궤적을 경로로 낸다
                  -> 곡률은 구조상 지켜지고, 굴린 궤적을 차체 검사점으로 충돌 검사
       'hybrid' : 칸 경로 위 1.2 m 앞 점까지만 전진 전용 Hybrid A* (planner_core.hybrid_astar 재사용)
  5) 전진만(후진·K턴 없음). 못 찾으면 빈 경로 + 이유(Status).

좌표
  경기장 좌표(field): 가운데 원점, +y = 페이즈2 쪽, 단위 m. 사전 정보(경계·랜더·자동 모드 목표)는 이 좌표로 준다.
  odom: EKF-C odom. field -> odom 변환 T_odom_field = (x, y, yaw) 는 출발 자세에서 계산(from_start_pose).
  격자 평면: ArenaMap.grid() 가 planner_core.Grid(경기장 방향으로 돌린 격자) 를 만든다. 경로 출력은 odom.
"""
import math
import time
from dataclasses import dataclass, replace

import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra as cs_dijkstra

import planner_core as P


# ------------------------------------------------------------------ 상태 코드 (/planner/status 와 같음)
class Status:
    OK = 'OK'                         # 경로 냄
    ARRIVED = 'ARRIVED'               # 목표 도착 (빈 경로 = 정지)
    NO_GOAL = 'NO_GOAL'               # 목표 없음
    NO_ROUTE = 'NO_ROUTE'             # 알려진 불가 칸·금지 칸으로 막혀 목표까지 길이 없음
    INFEASIBLE = 'INFEASIBLE'         # 길은 있으나 전진만·최소 회전반경으로 따라갈 수 없음
    START_BLOCKED = 'START_BLOCKED'   # 출발 자리가 사전 금지 칸(경계·랜더)
    GOAL_BLOCKED = 'GOAL_BLOCKED'     # 목표가 금지 칸이고 근처에 갈 수 있는 칸도 없음
    OUTSIDE = 'OUTSIDE'               # 로버가 경기장 격자 밖
    TIMEOUT = 'TIMEOUT'               # 시간 상한 안에 못 찾음
    NO_HEADING = 'NO_HEADING'         # [261011] 목표 위치에는 갈 수 있으나 목표 방향으로 들어갈 공간이 없음 (heading_fail='stop' 일 때만)

    KOR = {OK: '정상', ARRIVED: '도착', NO_GOAL: '목표 없음', NO_ROUTE: '막힘(길 없음)',
           INFEASIBLE: '회전반경으로 따라갈 수 없음', START_BLOCKED: '출발 자리가 금지 칸',
           GOAL_BLOCKED: '목표가 금지 칸', OUTSIDE: '로버가 경기장 밖', TIMEOUT: '계산 시간 초과',
           NO_HEADING: '목표 방향으로 들어갈 공간 없음'}


# ------------------------------------------------------------------ 도착 판정 (계획기·ROS 노드·시뮬이 같이 씀) [261011 방위각]
def arrival(mid_xy, th, goal_xy, goal_th=None, tol_m=0.10, pass_m=0.30, th_tol=math.radians(10.0)):
    """기준점(두 축 가운데) mid_xy·방향 th 가 목표에 도착했나. 같은 좌표계(격자 평면 또는 odom)로 줄 것.
    방향 없는 목표(goal_th None): 예전과 같음 — 거리 ≤ tol_m, 또는 거리 ≤ pass_m 이고 목표가 로버 뒤쪽('지나침 도착').
    방향 있는 목표: (거리 ≤ tol_m 그리고 |방향 오차| ≤ th_tol), 또는 '지나침' = 방향 오차 ≤ th_tol 이면서
      목표 방향선을 따라 앞으로 지나쳤고(0 < 진행 방향 오차 ≤ pass_m) 옆 오차 ≤ tol_m.
      pos_ok = 거리 ≤ tol_m 만 (방향을 못 맞출 때 '위치만 도착' 판단에 씀. 지나침 범위는 방향이 맞을 때만 의미가 있음)
    반환 (도착?, dict(dist, e_along, e_cross, e_th, pos_ok, passed))."""
    dx, dy = mid_xy[0] - goal_xy[0], mid_xy[1] - goal_xy[1]
    dist = math.hypot(dx, dy)
    if goal_th is None:
        behind = -dx * math.cos(th) - dy * math.sin(th) < 0
        pos_ok = dist <= tol_m or (dist <= pass_m and behind)
        return pos_ok, dict(dist=dist, e_along=None, e_cross=None, e_th=None, pos_ok=pos_ok, passed=pos_ok and dist > tol_m)
    c, s_ = math.cos(goal_th), math.sin(goal_th)
    e_s, e_n = dx * c + dy * s_, -dx * s_ + dy * c
    e_th = P._wrap(th - goal_th)
    th_ok = abs(e_th) <= th_tol
    pos_ok = dist <= tol_m
    passed = th_ok and not pos_ok and 0.0 < e_s and dist <= pass_m and abs(e_n) <= tol_m
    ok = (pos_ok and th_ok) or passed
    return ok, dict(dist=dist, e_along=e_s, e_cross=e_n, e_th=e_th, pos_ok=pos_ok, passed=passed)


# ------------------------------------------------------------------ 2D 변환
def se2(x, y, yaw):
    return (float(x), float(y), float(yaw))


def se2_apply(T, xy):
    x, y, a = T
    xy = np.asarray(xy, float)
    c, s = math.cos(a), math.sin(a)
    return np.stack([x + c * xy[..., 0] - s * xy[..., 1], y + s * xy[..., 0] + c * xy[..., 1]], -1)


def se2_inv(T):
    x, y, a = T
    c, s = math.cos(a), math.sin(a)
    return (-c * x - s * y, s * x - c * y, -a)


def se2_compose(A_, B_):
    """A ∘ B : B 좌표계를 A 로 옮김 (B 의 원점 = A 에서 본 B 위치)."""
    p = se2_apply(A_, np.array([B_[0], B_[1]], float))
    return (float(p[0]), float(p[1]), float(A_[2] + B_[2]))


def field_from_rover_pose(rover_pose_odom, start_field):
    """[auto-1014] '지금 로버(base_link) odom 자세 = 경기장 좌표 start_field' 가 되게 하는 T_odom_field.
    (from_start_pose 는 로버가 odom 원점에 있다고 가정. 이건 지금 자세를 직접 씀 — SLAM 을 어디서 켰든, 재추정했든 상관없음)"""
    return se2_compose(se2(*rover_pose_odom), se2_inv(se2(*start_field)))


def from_start_pose(start_field):
    """출발 base_link 의 경기장 좌표 (x, y, yaw) -> T_odom_field.
    가정 [추정, 확인 필요]: EKF-C odom 원점 = 출발 base_link, odom x 축 = 출발 진행 방향 (출발 = 수평, 운용으로 맞춤)."""
    return se2_inv(se2(*start_field))


# ------------------------------------------------------------------ 경기장 지도
@dataclass
class ArenaParams:
    field_w: float = 4.0            # 경기장 x 폭 [m] [사실, 인계: 4 x 5 m]
    field_l: float = 5.0            # 경기장 y 길이 [m]
    res: float = 0.05               # 판정 격자와 같은 칸
    pad_m: float = 0.30             # 격자를 경기장 밖으로 넓히는 폭 (전부 금지)
    edge_margin_m: float = 0.15     # odom 오차 여유 [가정]. 경계 금지 폭 = 로버 반폭 + 이 값 (10/5: vSLAM 축척 오차 약 3% x 4 m = 12 cm [추정] -> 0.10 에서 올림)
    edge_halo_mode: str = 'band'    # 'band' = 띠 안 미관측·의심 씨앗만 번짐(기본, 시뮬 도착 그대로) / 'all' = 근처 띠 씨앗도 번짐(도착 OT 29→24, 메시 8→5) (10/6)
    edge_halo_margin: float = 1.0   # 경계 미관측·의심 씨앗 번짐 = 반폭 + 이 값 x edge_margin_m (시험용 손잡이, 10/6)
    edge_unknown_m: float = 0.30    # 금지 띠 안쪽으로 이 폭 안의 '미관측' 칸은 금지 (경기장 끝 낭떠러지는 판정이 못 잡음, 판정 인계 8-2·8-8). 관측해서 통과면 풀림
    open_edges: tuple = ()          # 금지 띠를 두지 않을 변 ('+x', '-x', '+y', '-y'). p2_enable 이면 안 씀
    # [261015 케일럽 '(가)'] 페이즈2 구역을 +y 변 위에 붙임. 페이즈1 과 사이 변은 벽(금지 띠), 틈(입구)만 열림.
    #   크기·위치·입구 폭은 10/5 주최측 배치안 그림을 해석한 값 [가정] — 연습 주행 때 확인해서 설정 파일로 고침
    p2_enable: bool = False
    p2_w: float = 3.0               # 페이즈2 x 폭 [가정]
    p2_l: float = 4.0               # 페이즈2 y 깊이 [가정]
    p2_x: float = -0.5              # 페이즈2 x 가운데 (경기장 좌표) [가정: 왼쪽 끝에 붙음 = x -2 ~ 1]
    gap_x: float = -1.5             # 입구 x 가운데 = 틈 점의 x (틈을 옮기면 같이 옮김)
    gap_w: float = 1.2              # 입구 폭 [가정]. 차가 지나갈 폭 = 입구 폭 - 2 x (반폭 + edge_margin) = 0.61 m
    lander_xy: tuple = None         # 경기장 좌표 랜더 중심. None = 랜더 없음 (기본). [10/8 케일럽] 랜더 위치는 모름 - 예전 '가운데'는 근거 없음
    lander_body_r: float = 0.28     # [추정, 사진 비율]
    lander_leg_r: float = 0.42
    lander_pad_r: float = 0.045
    lander_heading: float = 0.0
    judge_r_m: float = 1.25         # 판정 노드가 내는 반경 [사실, 인계 6-1]
    rim_m: float = 0.15             # 판정 원 가장자리 이 폭은 누적하지 않음 (이웃 칸이 잘려 턱을 놓칠 수 있음) [가정]
    half_width_m: float = 0.146     # 로버 반폭 [9/28 초안]
    # 불확실성 반영 (261005 시험): 'latest' = 최근 값 우선(기존), 'conf' = 거리 기반 신뢰도
    # [261008] 'replace' = 받은 격자를 그대로 씀(쌓지 않음). 판정 노드가 그리드 피쳐맵 전체를 판정해 보낼 때
    #          (daron_run traversability judge_extent:=map) — 지형 기억은 피쳐맵 한 곳, 비용은 매번 다시 계산된 값.
    merge: str = 'latest'
    lethal_keep_m: float = 0.05     # 'conf': 불가 칸은 그때보다 이만큼 이내로 가깝거나 더 가까이서 본 '통과'만 뒤집음
    conf_dist_m: float = 0.0        # >0 이면: 이 거리 안에서 한 번도 '통과'로 안 본 칸은 비용을 unc_cost 이상으로 (모르는 칸처럼)
    unc_cost: int = 74              #   planner_core 미관측 비용(p_unknown 1.5)과 같은 수준


class ArenaMap:
    """경기장 전체 판정 지도. cost[행=y, 열=x] (-1 미관측, 0~99, 100 불가), 사전 금지 칸 prior."""

    def __init__(self, ap: ArenaParams = None, T_odom_field=(0.0, 0.0, 0.0)):
        self.ap = ap = ap or ArenaParams()
        self.T = se2(*T_odom_field)
        self.res = ap.res
        bx0, by0, bx1, by1 = self.bbox()                               # [261015] 페이즈2 를 붙이면 그만큼 넓힘 (원점은 그대로 페이즈1 가운데)
        self.nx = int(round((bx1 - bx0 + 2 * ap.pad_m) / ap.res))
        self.ny = int(round((by1 - by0 + 2 * ap.pad_m) / ap.res))
        self.o_field = np.array([bx0 - ap.pad_m, by0 - ap.pad_m])     # 칸 [0,0] 모서리 (경기장 좌표)
        xs = self.o_field[0] + (np.arange(self.nx) + 0.5) * ap.res
        ys = self.o_field[1] + (np.arange(self.ny) + 0.5) * ap.res
        self.XF, self.YF = np.meshgrid(xs, ys)                         # 칸 중심 (경기장 좌표)
        C = se2_apply(self.T, np.stack([self.XF, self.YF], -1))
        self.XO, self.YO = C[..., 0], C[..., 1]                         # 칸 중심 (odom)
        self.cost = np.full((self.ny, self.nx), -1, np.int16)
        self.stamp = np.full((self.ny, self.nx), -np.inf)
        self.lethal_d = np.full((self.ny, self.nx), np.inf)    # 불가로 본 거리 (불가일 때만 의미)
        self.pass_d = np.full((self.ny, self.nx), np.inf)      # 통과로 본 가장 가까운 거리
        self.n_obs = np.zeros((self.ny, self.nx), np.int16)
        self.sus = np.zeros((self.ny, self.nx), bool)          # 판정 '낙차 의심'(why 10)으로 본 칸 (최근 값 우선). 경계 근처면 금지
        self.prior = self._prior()
        self.edge_zone = self._edge_zone()
        self.edge_watch = self._edge_watch()
        self.n_updates = 0

    def carry_from(self, old):
        """[auto-1014] 경기장을 다시 놓을 때(크기·위치 바뀜) 옛 지도의 칸 값을 odom 기준 가장 가까운 칸으로 옮겨 옴. 반환 옮긴 칸 수."""
        f = old.odom_to_field(np.stack([self.XO, self.YO], -1))
        c = np.floor((f[..., 0] - old.o_field[0]) / old.res).astype(int)
        r = np.floor((f[..., 1] - old.o_field[1]) / old.res).astype(int)
        ok = (r >= 0) & (r < old.ny) & (c >= 0) & (c < old.nx)
        for name in ('cost', 'stamp', 'lethal_d', 'pass_d', 'n_obs', 'sus'):
            a, b = getattr(self, name), getattr(old, name)
            a[ok] = b[r[ok], c[ok]]
        self.n_updates = old.n_updates
        return int(ok.sum())

    # -- [261015] 페이즈2 (p2_enable)
    def bbox(self):
        """경기장(페이즈1 + 붙인 페이즈2) 바깥 사각형 (x0, y0, x1, y1), 경기장 좌표."""
        ap = self.ap
        hx, hy = ap.field_w / 2, ap.field_l / 2
        if not ap.p2_enable:
            return -hx, -hy, hx, hy
        a0, a1 = ap.p2_x - ap.p2_w / 2, ap.p2_x + ap.p2_w / 2
        return min(-hx, a0), -hy, max(hx, a1), hy + ap.p2_l

    def opening(self):
        """페이즈1 → 페이즈2 입구 x 범위 (g0, g1). 없으면 None (p2 꺼짐, 또는 틈 x 가 페이즈2 폭 밖)."""
        ap = self.ap
        if not ap.p2_enable:
            return None
        hx = ap.field_w / 2
        g0 = max(ap.gap_x - ap.gap_w / 2, ap.p2_x - ap.p2_w / 2, -hx)
        g1 = min(ap.gap_x + ap.gap_w / 2, ap.p2_x + ap.p2_w / 2, hx)
        return (g0, g1) if g1 > g0 else None

    def _region(self, inset, X=None, Y=None):
        """페이즈1 ∪ 페이즈2 ∪ 입구 를 inset 만큼 안쪽으로 줄인 영역 (칸 중심 또는 주어진 점). 사이 변은 입구 밖이면 벽."""
        ap = self.ap
        X = self.XF if X is None else X
        Y = self.YF if Y is None else Y
        hx, hy = ap.field_w / 2, ap.field_l / 2
        m = (X >= -hx + inset) & (X <= hx - inset) & (Y >= -hy + inset) & (Y <= hy - inset)
        if ap.p2_enable:
            a0, a1 = ap.p2_x - ap.p2_w / 2, ap.p2_x + ap.p2_w / 2
            m |= (X >= a0 + inset) & (X <= a1 - inset) & (Y >= hy + inset) & (Y <= hy + ap.p2_l - inset)
            op = self.opening()
            if op is not None:
                m |= (X >= op[0] + inset) & (X <= op[1] - inset) & (Y >= hy - inset) & (Y <= hy + inset)
        return m

    def inside_field(self, xy_field):
        """경기장(페이즈2 포함) 안인가 — 웨이포인트 받을지. xy_field (..., 2)."""
        f = np.asarray(xy_field, float)
        return self._region(0.0, f[..., 0], f[..., 1])

    def set_gap_x(self, gx):
        """틈을 옮기면 입구도 옮김 (p2_enable 일 때). 사전 금지·경계 근처 칸을 다시 계산."""
        if not self.ap.p2_enable:
            return
        self.ap = replace(self.ap, gap_x=float(gx))
        self.prior = self._prior()
        self.edge_zone = self._edge_zone()
        self.edge_watch = self._edge_watch()

    def outline(self):
        """RViz 테두리 꼭짓점 (경기장 좌표, 닫힘). 페이즈2 가 페이즈1 x 범위 안이면 한 줄, 아니면 두 사각형을 이어서."""
        ap = self.ap
        hx, hy = ap.field_w / 2, ap.field_l / 2
        p1 = [(-hx, -hy), (hx, -hy), (hx, hy), (-hx, hy), (-hx, -hy)]
        if not ap.p2_enable:
            return p1
        a0, a1, y2 = ap.p2_x - ap.p2_w / 2, ap.p2_x + ap.p2_w / 2, hy + ap.p2_l
        if a0 >= -hx - 1e-9 and a1 <= hx + 1e-9:
            return [(-hx, -hy), (hx, -hy), (hx, hy), (a1, hy), (a1, y2), (a0, y2), (a0, hy), (-hx, hy), (-hx, -hy)]
        return p1 + [(a0, hy), (a1, hy), (a1, y2), (a0, y2), (a0, hy)]

    # -- 사전 금지 칸
    def _prior(self):
        ap = self.ap
        X, Y = self.XF, self.YF
        b = ap.half_width_m + ap.edge_margin_m
        if ap.p2_enable:                                               # [261015] 페이즈1 + 페이즈2 + 입구: 영역을 b 만큼 줄인 밖 = 금지 (경기장 밖 포함)
            m = ~self._region(b)
            self.edge_band = m.copy()
            return self._lander(m)
        m = np.zeros(X.shape, bool)
        hx, hy = ap.field_w / 2, ap.field_l / 2
        m |= (X < -hx) | (X > hx) | (Y < -hy) | (Y > hy)               # 경기장 밖
        if '-x' not in ap.open_edges: m |= X < -hx + b
        if '+x' not in ap.open_edges: m |= X > hx - b
        if '-y' not in ap.open_edges: m |= Y < -hy + b
        if '+y' not in ap.open_edges: m |= Y > hy - b
        self.edge_band = m.copy()                                      # 경계 쪽 금지만 (랜더 제외)
        return self._lander(m)

    def _lander(self, m):
        ap = self.ap
        X, Y = self.XF, self.YF
        if ap.lander_xy is not None:
            lx, ly = ap.lander_xy
            m |= np.hypot(X - lx, Y - ly) <= ap.lander_body_r + ap.half_width_m
            for k in range(4):
                a = ap.lander_heading + math.pi / 4 + k * math.pi / 2
                m |= np.hypot(X - lx - ap.lander_leg_r * math.cos(a), Y - ly - ap.lander_leg_r * math.sin(a)) \
                    <= ap.lander_pad_r + ap.half_width_m
        return m

    def _edge_zone(self):
        """경계 금지 띠 안쪽 edge_unknown_m 폭 (미관측이면 금지할 칸)."""
        ap = self.ap
        if ap.edge_unknown_m <= 0:
            return np.zeros(self.XF.shape, bool)
        X, Y = self.XF, self.YF
        b = ap.half_width_m + ap.edge_margin_m + ap.edge_unknown_m
        if ap.p2_enable:                                               # [261015] 줄인 영역 밖인데 사전 금지는 아닌 칸 (입구 양끝 벽 근처 포함)
            return ~self._region(b) & ~self.prior
        hx, hy = ap.field_w / 2, ap.field_l / 2
        m = np.zeros(X.shape, bool)
        if '-x' not in ap.open_edges: m |= X < -hx + b
        if '+x' not in ap.open_edges: m |= X > hx - b
        if '-y' not in ap.open_edges: m |= Y < -hy + b
        if '+y' not in ap.open_edges: m |= Y > hy - b
        return m & ~self.prior

    def _edge_watch(self):
        """미관측·의심이면 금지 씨앗이 되는 칸: 경계 쪽 사전 금지(띠·경기장 밖, 열린 변 너머 제외) + 경계 근처(edge_zone)."""
        ap = self.ap
        X, Y = self.XF, self.YF
        hx, hy = ap.field_w / 2, ap.field_l / 2
        w = self.edge_band | self.edge_zone
        if ap.p2_enable:                                               # [261015] 열린 변 없음 (입구는 영역 안)
            return w
        for side, beyond in (('-x', X < -hx), ('+x', X > hx), ('-y', Y < -hy), ('+y', Y > hy)):
            if side in ap.open_edges:
                w &= ~beyond
        return w

    # -- 좌표
    def field_to_odom(self, xy):
        return se2_apply(self.T, xy)

    def odom_to_field(self, xy):
        return se2_apply(se2_inv(self.T), xy)

    def origin_odom(self):
        p = self.field_to_odom(self.o_field)
        return (float(p[0]), float(p[1]), 0.0), P.yaw_to_quat(self.T[2])

    # -- 누적
    def _sample(self, rows, res, origin_p, origin_q, rover_xy_odom):
        """판정 배치 격자(행 = y)를 경기장 칸 중심에서 읽음. 반환 (iy, ix, 값(-1 = 격자 밖), 로버까지 거리, 판정 원 rim 안쪽 여부)"""
        L = P.Grid(rows, res, origin_p, origin_q)
        corners = np.array([[0, 0], [L.nx, 0], [0, L.ny], [L.nx, L.ny]], float) * res
        co = L.g_to_odom(corners)[:, :2]
        lo, hi = co.min(0) - res, co.max(0) + res
        sel = (self.XO >= lo[0]) & (self.XO <= hi[0]) & (self.YO >= lo[1]) & (self.YO <= hi[1])
        iy, ix = np.nonzero(sel)
        p3 = np.stack([self.XO[sel], self.YO[sel], np.full(len(iy), L.o[2])], -1)
        g = L.odom_to_g(p3) if len(iy) else np.zeros((0, 3))
        r = np.floor(g[:, 1] / res).astype(int)
        c = np.floor(g[:, 0] / res).astype(int)
        ok = (r >= 0) & (r < L.ny) & (c >= 0) & (c < L.nx)
        v = np.full(len(iy), -1, np.int16)
        v[ok] = L.cost[r[ok], c[ok]]
        inner = np.ones(len(iy), bool)
        if rover_xy_odom is not None:
            d = np.hypot(p3[:, 0] - rover_xy_odom[0], p3[:, 1] - rover_xy_odom[1])
            if self.ap.rim_m > 0:
                inner = d <= self.ap.judge_r_m - self.ap.rim_m
        else:
            d = np.zeros(len(iy))
        return iy, ix, v, d, inner

    def integrate(self, cost_rows, res, origin_p, origin_q, rover_xy_odom=None, stamp=0.0, why_rows=None):
        """판정 격자 한 장(OccupancyGrid 배치: 행 = y, -1/0~99/100)을 누적. 최근 값 우선, -1 은 덮어쓰지 않음.
        rover_xy_odom: 그 격자를 낸 순간 로버(base_link) 위치. 주면 판정 원 가장자리 rim_m 띠는 쓰지 않음.
        why_rows: 같은 시각의 판정 사유 격자(같은 배치). 주면 '낙차 의심'(10) 칸을 따로 기억(mark_why).
        반환: 갱신한 칸 수."""
        if self.ap.merge == 'replace':                                 # [261008] 쌓지 않음: 이전 비용·의심을 비우고 받은 그대로
            self.cost[:] = -1
            self.pass_d[:] = np.inf
            self.lethal_d[:] = np.inf
            self.sus[:] = False
            rover_xy_odom_rim = None                                   # 맵 전체 판정이라 판정 원 rim 자르기 없음
        else:
            rover_xy_odom_rim = rover_xy_odom
        iy, ix, v, d, inner = self._sample(cost_rows, res, origin_p, origin_q, rover_xy_odom_rim)
        if rover_xy_odom_rim is None and rover_xy_odom is not None and len(iy):
            d = np.hypot(self.XO[iy, ix] - rover_xy_odom[0], self.YO[iy, ix] - rover_xy_odom[1])
        upd = (v >= 0) & inner
        if not upd.any():
            return 0
        a, b, v, d = iy[upd], ix[upd], v[upd], d[upd]
        if self.ap.merge == 'conf':
            leth = v >= 100
            was = self.cost[a, b] >= 100
            # 불가: 바로 받음 (안전 쪽). 불가로 본 거리는 가장 가까운 값
            self.lethal_d[a[leth], b[leth]] = np.where(was[leth], np.minimum(self.lethal_d[a[leth], b[leth]], d[leth]), d[leth])
            # 통과: 지금 불가인 칸은 그때만큼 가까이서 봤을 때만 뒤집음
            ok = ~leth & (~was | (d <= self.lethal_d[a, b] + self.ap.lethal_keep_m))
            keep = ~leth & ~ok
            self.cost[a[leth], b[leth]] = 100
            self.cost[a[ok], b[ok]] = v[ok]
            self.lethal_d[a[ok], b[ok]] = np.inf
            self.pass_d[a[ok], b[ok]] = np.minimum(self.pass_d[a[ok], b[ok]], d[ok])
            self.n_kept = getattr(self, 'n_kept', 0) + int(keep.sum())
        else:
            self.cost[a, b] = v
            pm = v < 100
            self.pass_d[a[pm], b[pm]] = np.minimum(self.pass_d[a[pm], b[pm]], d[pm])
        self.n_obs[a, b] += 1
        self.stamp[a, b] = stamp
        self.n_updates += 1
        self.sus[a, b] &= self.cost[a, b] == self.SUS_COST            # 비용이 바뀐 칸은 의심 풀림 (사유 격자를 놓쳐도 낡은 의심이 안 남게)
        if why_rows is not None:
            self.mark_why(why_rows, res, origin_p, origin_q, rover_xy_odom_rim)
        return int(len(a))

    SUS_WHY = 10                                                   # judge_core NEG_SUS
    SUS_COST = 90                                                  # judge_core SUS_COST (의심 칸 비용)
    OBS_WHY = (1, 2, 3, 4, 5, 7, 8, 9)                             # 관측해서 판정한 사유 (미관측 0·보류 6 제외)

    def mark_why(self, why_rows, res, origin_p, origin_q, rover_xy_odom=None):
        """사유 격자로 '낙차 의심' 층 갱신: 10 이면 의심, 관측 사유면 의심 풀림, 미관측·보류면 그대로 (비용 누적과 같은 규칙)."""
        if self.ap.merge == 'replace':                                 # [261008] 맵 전체 판정: rim 자르기 없음
            rover_xy_odom = None
        iy, ix, v, _, inner = self._sample(why_rows, res, origin_p, origin_q, rover_xy_odom)
        s = inner & (v == self.SUS_WHY)
        o = inner & np.isin(v, self.OBS_WHY)
        self.sus[iy[s], ix[s]] = True
        self.sus[iy[o], ix[o]] = False
        return int(s.sum())

    def edge_block(self):
        """경계 쪽에서 금지할 칸 (사전 금지 띠에 더하는 것). 경기장 끝 낭떠러지는 판정이 정면으로는 못 잡으므로(판정 인계 8-2·8-8·8-12):
        ① 경계 근처(edge_zone, 띠 안쪽 edge_unknown_m)의 미관측·낙차 의심 칸 = 금지 (10/5)
        ② 사전 금지 띠·경기장 밖의 미관측·의심 칸 둘레 반폭 + edge_margin 까지 금지 (10/6, edge_halo_mode='band').
           odom 이 안쪽으로 틀려 진짜 끝이 믿는 띠 안에 오면 띠만으로는 바퀴가 진짜 끝에 걸칠 수 있음(차체 검사는 가운데선 점만 봄).
           좁은 틈 시험: ② 없이 odom 0.3 m 오차면 바퀴가 진짜 끝 밖 2 cm, ② 있으면 0.5 m 오차까지 바퀴-끝 3 cm 이상.
           ①의 칸까지 번지면('all') 출발 때 못 본 경계 근처가 넓게 막혀 시뮬 도착이 줄어서(OT 29→24, 메시 8→5) 기본은 띠만."""
        if self.ap.edge_unknown_m <= 0:
            return np.zeros(self.cost.shape, bool)
        bad = (self.cost < 0) | self.sus
        seed = self.edge_watch & bad
        if self.ap.edge_halo_mode == 'band':                              # 띠 안 씨앗만 번짐, 근처 띠(edge_zone) 씨앗은 그 칸만
            seed = self.edge_watch & ~self.edge_zone & bad
        k = int(math.ceil((self.ap.half_width_m + self.ap.edge_margin_m * self.ap.edge_halo_margin) / self.res - 1e-9))
        m = ndimage.binary_dilation(seed, P._disk(k)) if (k > 0 and seed.any()) else seed
        if self.ap.edge_halo_mode == 'band':
            m |= self.edge_zone & bad
        return m & ~self.prior

    def merged_cost(self):
        c = self.cost.copy()
        if self.ap.conf_dist_m > 0:                                   # 멀리서만 '통과'로 본 칸 = 아직 불확실
            unc = (c >= 0) & (c < 100) & (self.pass_d > self.ap.conf_dist_m)
            c[unc] = np.maximum(c[unc], self.ap.unc_cost)
        c[self.edge_block()] = 100                                    # 경계 근처 미관측·낙차 의심 = 금지
        c[self.prior] = 100
        return c

    def grid(self):
        """planner_core.Grid (사전 금지 칸 = 100). .hard = 출발 자리 풀어주기로도 풀지 않을 칸."""
        p, q = self.origin_odom()
        g = P.Grid(self.merged_cost(), self.res, p, q)
        g.hard = self.prior
        g.edge_unknown = self.edge_block()                             # 경계 근처 미관측·의심 (목표 옮길 때 더 멀리 찾음)
        g.edge_unknown_m = self.ap.edge_unknown_m + self.ap.half_width_m + self.ap.edge_margin_m   # 목표 옮길 때 더할 폭 (근처 띠 + 번짐)
        g.edge_band = self.edge_band
        return g

    def stats(self):
        inside = ~self.prior
        c = self.cost[inside]
        return dict(known=float((c >= 0).mean()), lethal=float((c >= 100).mean()))


# ------------------------------------------------------------------ 빠른 8방향 최단 경로 (P.dijkstra 와 같은 규칙, C 구현)
_OFFS = [(0, 1, 1.0), (1, 0, 1.0), (1, 1, math.sqrt(2)), (1, -1, math.sqrt(2))]   # 한 방향씩만 (무방향 그래프)


def _graph(cm):
    pen = cm.pen
    ny, nx = pen.shape
    free = np.isfinite(pen)
    idx = np.arange(ny * nx).reshape(ny, nx)
    rows, cols, w = [], [], []
    res = cm.g.res
    for dr, dc, L in _OFFS:
        r0, r1 = 0, ny - dr
        c0, c1 = max(0, -dc), nx - max(0, dc)
        a = (slice(r0, r1), slice(c0, c1))
        b = (slice(r0 + dr, r1 + dr), slice(c0 + dc, c1 + dc))
        ok = free[a] & free[b]
        if dr and dc:                                              # 대각선: 모서리 끼고 돌지 않기
            ok &= free[r0 + dr:r1 + dr, c0:c1] & free[r0:r1, c0 + dc:c1 + dc]
        ww = L * res * (1.0 + 0.5 * (np.where(free[a], pen[a], 0) + np.where(free[b], pen[b], 0)))
        rows.append(idx[a][ok]); cols.append(idx[b][ok]); w.append(ww[ok])
    rows, cols, w = np.concatenate(rows), np.concatenate(cols), np.concatenate(w)
    return coo_matrix((w, (rows, cols)), shape=(ny * nx, ny * nx)).tocsr()


def fast_dijkstra(cm, sources, graph=None, pred=False):
    """sources 칸들에서 모든 칸까지 최소 비용 [m 환산] (ny, nx). pred=True 면 (D, 다음 칸 지도)도."""
    ny, nx = cm.pen.shape
    G = graph if graph is not None else _graph(cm)
    src = [s[0] * nx + s[1] for s in sources if cm.free(s)]
    if not src:
        D = np.full((ny, nx), np.inf)
        return (D, None) if pred else D
    if pred:
        D, pr, _ = cs_dijkstra(G, directed=False, indices=src, min_only=True, return_predecessors=True)
        return D.reshape(ny, nx), pr
    D = cs_dijkstra(G, directed=False, indices=src, min_only=True)
    return D.reshape(ny, nx)


def route_from(pred, start_rc, nx, max_len=100000):
    """목표에서 퍼진 Dijkstra 의 predecessor 로 출발 -> 목표 칸 목록."""
    u = start_rc[0] * nx + start_rc[1]
    out = [start_rc]
    for _ in range(max_len):
        v = pred[u]
        if v < 0:
            break
        out.append((int(v // nx), int(v % nx)))
        u = v
    return out


def route_cost(cm, cells):
    c = 0.0
    res = cm.g.res
    for a, b in zip(cells[:-1], cells[1:]):
        if not (cm.free(a) and cm.free(b)):
            return math.inf
        L = math.hypot(a[0] - b[0], a[1] - b[1])
        if L > 1.5:
            return math.inf
        c += L * res * (1.0 + 0.5 * (cm.pen[a] + cm.pen[b]))
    return c


# ------------------------------------------------------------------ 추종기 흉내로 굴리기 (pure pursuit, 전진만)
def pursuit_rollout(cm, route_xy, start, kmax, Ld, horizon, offs, ds=0.025, end_tol=0.05, krate=0.0, kap0=0.0):
    """route_xy (N,2) 를 따라 start(x, y, th)에서 곡률 |κ| <= kmax 로 전진.
    krate > 0 [261014 시험]: 곡률이 주행 1 m 마다 krate 이상 못 바뀜(조향이 천천히 바뀌는 로버), 시작 곡률 kap0.
    반환 (poses [(x, y, th)], 굴린 거리, 끝난 이유 'end' | 'horizon' | 'collide')."""
    R = np.asarray(route_xy, float)
    if len(R) < 2:
        R = np.vstack([R, R])
    s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(R, axis=0).T))])
    x, y, th = start
    poses = [(x, y, th)]
    j, Lr = 0, 0.0
    n = len(R)
    while Lr < horizon - 1e-9:
        lo, hi = j, min(n, j + 25)
        d = np.hypot(R[lo:hi, 0] - x, R[lo:hi, 1] - y)
        j = lo + int(np.argmin(d))
        if math.hypot(R[-1, 0] - x, R[-1, 1] - y) <= end_tol:
            return poses, Lr, 'end'
        st = s[j] + Ld
        if st >= s[-1]:
            tx, ty = R[-1]
        else:
            k = int(np.searchsorted(s, st)) - 1
            f = (st - s[k]) / max(s[k + 1] - s[k], 1e-9)
            tx, ty = R[k] + f * (R[k + 1] - R[k])
        a = P._wrap(math.atan2(ty - y, tx - x) - th)
        Lt = math.hypot(tx - x, ty - y)
        if abs(a) > math.pi / 2:
            kap = math.copysign(kmax, a)                         # 목표점이 뒤: 최대 곡률로 돈다
        else:
            kap = float(np.clip(2.0 * math.sin(a) / max(Lt, 1e-3), -kmax, kmax))
        if krate > 0:
            kap = float(np.clip(kap, kap0 - krate * ds, kap0 + krate * ds))
            kap0 = kap
        th = P._wrap(th + kap * ds)
        x += ds * math.cos(th)
        y += ds * math.sin(th)
        Lr += ds
        if not P._pose_ok(cm, x, y, th, offs):
            return poses, Lr - ds, 'collide'
        poses.append((x, y, th))
    return poses, Lr, 'horizon'


# ------------------------------------------------------------------ 계획기
@dataclass
class ArenaPlanParams:
    method: str = 'smooth'          # 'raw' | 'smooth' | 'hybrid'
    allow_reverse: bool = False     # 'hybrid' 만: 후진·K턴 허용 (비교용. 운용은 당분간 전진만 [결정, 10/4])
    goal_snap_m: float = 0.50       # 목표 칸이 금지 칸이면(또는 차체 여유가 없으면) 이 반경 안 가장 가까운 갈 수 있는 칸으로
    goal_pass_m: float = 0.30       # 목표가 이 거리 안이고 로버 뒤쪽이면 '지나침'으로 도착 처리 (전진만이라 되돌아오는 고리 경로 방지, 10/6 실시간 시험)
    hysteresis: float = 0.15        # 직전 길의 비용이 새 길보다 15% 안에서 비싸면 직전 길 유지 (좌우 번갈림 방지)
    reuse_dist_m: float = 0.30      # 직전 길이 로버에서 이만큼 멀어졌으면 재사용 안 함
    time_limit_s: float = 1.0       # 계산 시간 상한 [인계 5-2]
    window_m: float = 0.0           # > 0 이면 내는 경로를 계획 때 base_link 둘레 이 반경(= 판정 창 1.25 m) 안으로 자름 [10/7 케일럽: 판정 범위와 맞춤]
    moved_goal_arrive_m: float = 0.5  # 목표가 불가 칸이라 옮겼고, 옮긴 목표가 이 거리 안인데 회전반경으로 더 못 다가가면 '도착' [10/7 케일럽 허락]
    # smooth
    lookaheads: tuple = (0.30, 0.20, 0.12)
    horizon_m: float = 2.0          # 굴려 볼 거리 (판정 반경 1.25 m 너머는 어차피 다시 계획)
    min_ok_m: float = 0.50          # 충돌 전까지 이만큼도 못 가면 실패
    smooth_fallback: bool = False   # [261014 시험, 기본 끔] 다듬기가 바로 막히면(INFEASIBLE 직전) 짧은 구간 Hybrid A*(전진만, 곡률 한도 탐색)로 한 번 더
    fallback_time_s: float = 0.5    #   그 대신 찾기의 시간 상한 (두 부분 목표 합)
    steer_rate: float = 0.0         # [261014 시험, 0 = 끔] 다듬기 굴려보기에서 곡률 변화 한계 [1/m, 주행 1 m 마다] — 조향이 느린 로버.
                                    #   시작 곡률 = planner.kap_now (노드라면 조향 지령에서)
    stop_before_m: float = 0.10     # 충돌 지점 이만큼 앞에서 경로를 끊음
    # hybrid
    sub_dists: tuple = (1.2, 0.6)   # 부분 목표까지 칸 경로 거리 (실패하면 다음 값으로)
    sub_tol_m: float = 0.15
    hybrid_expand: int = 15000
    # [261011] 도착 방위각 (목표에 방향이 있을 때만. 방향 없는 목표는 위 그대로)
    heading_mode: str = 'hybrid'    # 'entry'(A: 목표 뒤 진입점 → 직선) | 'hybrid'(B: 전 구간 (x, y, θ) Hybrid A*) | 'mixed'(C: 멀리는 method, 목표 switch_m 안은 B)
                                    # | 'dubins'(10/11 석윤님 안: 멀리는 method(다익스트라+다듬기), switch_m 안에서 지금 자세 → 목표 자세 Dubins 곡선 하나, 탐색 없음)
    th_tol_deg: float = 10.0        # 도착 방향 허용 오차 (교수님 10/11: top-view 에서 위치·방향이 각각 경계 안)
    heading_fail: str = 'position'  # 방향을 맞출 길이 없을 때: 'position' = 위치만 맞추고 도착(정지 이유에 방향 오차) | 'stop' = NO_HEADING 정지
    switch_m: float = 1.5           # 'mixed': 목표까지 칸 경로 거리가 이 안이면 방향 맞춤 탐색
    entry_dists: tuple = (0.5, 0.35, 0.2)   # 'entry': 진입점 거리 (막혔으면 다음 값)
    lead_in_m: float = 0.15         # 'hybrid'·'mixed': 목표 바로 앞 이 길이는 직선으로 들어감 (추종기가 방향을 안정시키게)
    head_tol_frac: float = 0.5      # 탐색은 도착 허용 범위의 이 비율 안까지 (추종 오차 여유)
    head_expand: int = 30000        # 방향 맞춤 탐색 확장 수 상한
    head_time_s: float = 0.5        # 방향 맞춤 탐색 시간 상한. 넘으면 '이 예산 안에 길 없음'으로 봄 (불가능을 증명하려면 전체 상태를 다 봐야 해서 끝이 없음)
    head_weight: float = 1.5        # 가중 A* (휴리스틱 배수). 1 = 최적, 클수록 빠르고 조금 돌아감
    head_use_h2d: bool = True       # 휴리스틱에 다익스트라 거리 지도(장애물 고려) 사용. False 는 비교 시험용 (장애물 무시 거리만)
    shot_every: int = 10            # 해석적 확장(Dubins 곡선 그어 보기) 간격 [확장 수]
    head_r_margin: float = 1.2      # 방향 맞춤 탐색은 최소 회전반경 x 이 배수로 (추종기가 늦게 따라와도 고칠 곡률 여유. 1.0 이면 원 위에서 계속 놓쳐 맴돎 - 10/11 시뮬)
    head_keep: bool = True          # 직전 방향 맞춤 경로 위에 있고(10 cm·15° 안) 안 막혔으면 그대로 씀 (매번 새로 찾으면 좌·우 고리가 바뀌어 맴돎)
    head_r_max_margin: float = 2.0  # [10/11 백업] 회전 부족(set_turn_ratio)으로 키울 때 반경 배수 상한


class ArenaPlanner:
    """상태: 직전 길(히스테리시스용). 목표가 바뀌면 reset()."""

    def __init__(self, app: ArenaPlanParams = None, rp: P.RoverParams = None, pp: P.PlanParams = None):
        self.app = app or ArenaPlanParams()
        self.rp = replace(rp or P.RoverParams(), allow_reverse=bool(self.app.allow_reverse and self.app.method == 'hybrid'))
        self.pp = pp or P.PlanParams()
        self.prev = None
        self.band_exit = False              # [261016] 출발 자리가 경계 금지 띠 안이면 띠 밖으로 나가는 칸만 열기 (운영자 확인 뒤 노드가 켬)
        self.heading_dropped = None         # [261011] 방향을 못 맞춰 위치만 보기로 한 목표 (목표 키). reset() 이면 다시 시도
        self.prev_head = None               # [261011] 직전 방향 맞춤 경로 (목표 칸·방향, 격자 평면 poses)
        self.head_on_key = None             # [261011] 'mixed': 방향 탐색으로 들어간 목표 (전환 떨림 방지)
        self.head_far_fail = None           # [261011] 'hybrid': 멀리서 방향 탐색이 실패한 목표
        self.r_margin_now = max(1.0, self.app.head_r_margin)
        self.k_scale = 1.0                  # [10/11 백업] 회전 부족 비율(실제/지령 곡률) — 다듬기 곡률 상한에 곱함
        r_plan = self.rp.r_min * self.r_margin_now
        self.rp_head = replace(self.rp, delta_max_deg=math.degrees(math.atan(self.rp.wheelbase_m / r_plan)))
        if self.app.heading_mode in ('hybrid', 'mixed'):
            import dubins as DB
            DB.heur_table(self.rp_head.r_min)   # Dubins 휴리스틱 표를 미리 (약 1 s, 프로세스당 한 번). 첫 방향 목표 계획이 시간 상한에 걸리지 않게

    def set_turn_ratio(self, ratio):
        """[10/11 교수님: 슬립으로 최소 회전반경을 못 지킴] 잰 회전 비율(실제/지령 곡률, 1 = 정상)만큼 계획 반경을 키움.
        방향 맞춤 반경 = 최소 회전반경 x head_r_margin / 비율 (0.1 단위로 올림, head_r_max_margin 상한), 다듬기 곡률 상한 x 비율.
        줄일 때는 0.2 넘게 좋아졌을 때만 (값이 왔다 갔다 하지 않게). None = 원래대로. 반환 지금 반경 배수."""
        base = max(1.0, self.app.head_r_margin)
        r = 1.0 if ratio is None else float(min(1.0, max(0.3, ratio)))
        if r >= 0.95:
            m = base                                                  # 정상으로 돌아오면 원래 값 그대로 (0.1 단위로 안 올림)
        else:
            m = max(base, min(max(base, self.app.head_r_max_margin), math.ceil(base / r * 10 - 1e-6) / 10))
        grow, back = m > self.r_margin_now + 1e-9, m < self.r_margin_now - 1e-9 and (m == base or m <= self.r_margin_now - 0.2 + 1e-9)
        if grow or back:
            r_plan = self.rp.r_min * m
            if self.app.heading_mode in ('hybrid', 'mixed') and not self._table_ready(r_plan):
                return self.r_margin_now                              # 표가 아직 (prebuild_tables 가 만드는 중) → 다음에
            self.r_margin_now = m
            self.rp_head = replace(self.rp, delta_max_deg=math.degrees(math.atan(self.rp.wheelbase_m / r_plan)))
            self.prev_head = None
            self.k_scale = base / m if m > base else 1.0
        return self.r_margin_now

    def _margins(self):
        base = max(1.0, self.app.head_r_margin)
        top = max(base, self.app.head_r_max_margin)
        ms, m = [base], math.ceil(base * 10 + 1e-6) / 10
        while m <= top + 1e-9:
            ms.append(round(m, 1)); m += 0.1
        return ms

    def _table_ready(self, r_plan):
        import dubins as DB
        rp_ = replace(self.rp, delta_max_deg=math.degrees(math.atan(self.rp.wheelbase_m / r_plan)))
        return round(rp_.r_min, 4) in DB._TABLE

    def prebuild_tables(self, sync=False):
        """[10/11] turn_adapt 용: 반경 배수마다 Dubins 휴리스틱 표를 미리 (하나 약 1 s). 노드는 스레드로, 시뮬은 그냥 부름."""
        import dubins as DB
        for m in self._margins():
            rp_ = replace(self.rp, delta_max_deg=math.degrees(math.atan(self.rp.wheelbase_m / (self.rp.r_min * m))))
            DB.heur_table(rp_.r_min)

    def reset(self):
        self.prev = None
        self.heading_dropped = None
        self.prev_head = None
        self.head_on_key = None
        self.head_far_fail = None

    @staticmethod
    def goal_key(goal_xy, goal_yaw):
        return (round(float(goal_xy[0]), 3), round(float(goal_xy[1]), 3), None if goal_yaw is None else round(float(goal_yaw), 3))

    def heading_active(self, goal_xy, goal_yaw):
        """이 목표에서 방향 조건을 볼지 (방향이 있고, 못 맞춰 위치만 보기로 하지 않았으면)."""
        return goal_xy is not None and goal_yaw is not None and self.heading_dropped != self.goal_key(goal_xy, goal_yaw)

    def plan(self, amap: ArenaMap, base_p, R_body, goal_odom, goal_yaw=None):
        """base_p: base_link 위치(odom, 3), R_body: 몸체 회전행렬(odom), goal_odom: (x, y[, z]) 또는 None (앞 두 값만 씀).
        goal_yaw: 목표 방향 [rad, odom yaw]. 주면 도착 방위각 목표 [261011]: 도착 = 위치 goal_tol_m 안 그리고 방향 th_tol_deg 안,
        국소 계획은 heading_mode. 방향을 맞출 길이 없으면 heading_fail 대로 (위치만 / NO_HEADING 정지).
        반환 dict: status, reason, path [(x, y, z, yaw, d)] (odom, 두 축 가운데 점, 5 cm), route (칸 목록),
                  kmax, ms(단계별), plan_goal_odom, (방향 목표면) e_th, heading_dropped, heading_used."""
        t0 = time.time()
        app, rp, pp = self.app, self.rp, self.pp
        out = dict(status=Status.OK, reason='', path=[], route=None, kmax=0.0, ms={}, kept=False,
                   method=app.method, plan_goal_odom=None, heading_used=None)

        def done(status, reason=''):
            out['status'], out['reason'] = status, reason or Status.KOR.get(status, status)
            out['ms']['total'] = (time.time() - t0) * 1000
            return out

        if goal_odom is None:
            return done(Status.NO_GOAL)
        g = amap.grid()
        cm = P.CostMap(g, pp)
        hard = g.hard
        cm.pen[hard] = np.inf
        th = g.heading_g(R_body)
        base_xy = g.odom_to_g(np.asarray(base_p, float))
        mid = base_xy + rp.mid_from_base * np.array([math.cos(th), math.sin(th)])
        out.update(g=g, cm=cm, start_xy=mid, start_th=th)
        srow = g.cell(mid)
        if not g.inside(srow):
            return done(Status.OUTSIDE)
        offs = rp.footprint_offsets()
        if not cm.free(srow) or not P._pose_ok(cm, mid[0], mid[1], th, offs):
            cm.allow_escape(mid, max(pp.escape_m, rp.length_m / 2))   # 서 있는 자리는 실제로 갈 수 있음(잡음·벽 옆 띠)
            cm.pen[hard] = np.inf                                     # 단, 사전 금지 칸은 풀지 않음
        if not cm.free(srow) and self.band_exit:                     # [261016] 띠 안 출발 — 운영자가 허용함
            out['band_exit_cells'] = open_band_exit(amap, g, cm, srow, mid, rp, max(pp.escape_m, rp.length_m / 2))
        if not cm.free(srow):
            out['in_band'] = bool(g.inside(srow) and g.edge_band[srow])
            return done(Status.START_BLOCKED)
        # 목표
        gz = float(base_p[2])
        gxy = g.odom_to_g(np.array([goal_odom[0], goal_odom[1], gz]))
        grc = g.cell(gxy)
        need = max(abs(a) for a in offs) + 0.5 * g.res          # 목표에 선 차체 검사점이 금지 칸에 안 걸릴 여유

        def goal_ok(rc):
            return cm.free(rc) and cm.clear[rc] >= need

        if not goal_ok(grc):
            snap = app.goal_snap_m
            eu = getattr(g, 'edge_unknown', None)
            if eu is not None and g.inside(grc) and (eu[grc] or g.edge_band[grc]):
                snap += g.edge_unknown_m                             # 경계 띠·근처 미관측 목표: 아직 못 본 띠 + 번짐 폭만큼 더 안쪽까지 (랜더 쪽은 늘리지 않음)
            k = int(math.ceil(snap / g.res))
            best, bd = None, math.inf
            fb, fd = None, math.inf                                  # 로버 쪽 방향에 없으면 쓸 가장 가까운 칸
            tv = mid - gxy[:2]
            tn = math.hypot(*tv)
            for dr in range(-k, k + 1):
                for dc in range(-k, k + 1):
                    rc = (grc[0] + dr, grc[1] + dc)
                    d = math.hypot(dr, dc) * g.res
                    if d <= snap and goal_ok(rc):
                        cv = g.center(rc) - gxy[:2]
                        # [10/7] 목표에서 로버 쪽(±60°)에 있는 칸을 먼저: 상자 위를 찍으면 상자 '앞'으로 (전엔 상자 윗면 가운데로 옮겨져 못 감)
                        toward = tn < 1e-6 or (cv[0] * tv[0] + cv[1] * tv[1]) >= 0.5 * tn * max(math.hypot(*cv), 1e-9)
                        if toward and d < bd:
                            best, bd = rc, d
                        if d < fd:
                            fb, fd = rc, d
            if best is None:
                best, bd = fb, fd
            if best is None:
                return done(Status.GOAL_BLOCKED)
            out['goal_moved_m'] = bd                                  # 목표를 옮긴 거리 (정지 이유 표시용)
            grc, gxy = best, g.center(best)
        out['plan_goal_odom'] = tuple(g.g_to_odom(gxy)[:2])
        dv = gxy - mid
        dist = math.hypot(*dv)
        # [261011] 방향 목표: 격자 평면 방향으로
        gth = None
        if self.heading_active(goal_odom, goal_yaw):
            gy_ = float(goal_yaw)
            gth = g.heading_g(np.array([[math.cos(gy_), -math.sin(gy_), 0.0], [math.sin(gy_), math.cos(gy_), 0.0], [0.0, 0.0, 1.0]]))
            out['heading_used'] = app.heading_mode
        elif goal_yaw is not None:
            out['heading_dropped'] = True
        th_tol = math.radians(app.th_tol_deg)
        arrived, ai = arrival(mid, th, gxy, gth, pp.goal_tol_m, app.goal_pass_m, th_tol)
        out['arrive_info'] = ai
        if ai.get('e_th') is not None:
            out['e_th'] = ai['e_th']

        def near_moved_arrived():
            # 목표가 불가·금지 칸이라 옮겼는데, 옮긴 목표 가까이에서 회전반경 때문에 더 못 다가감 → 더 갈 데가 없으니 도착으로 (10/7 시뮬 '목표가 상자 위')
            self.prev = None
            out['goal_dist_m'] = dist
            out['near_moved'] = True
            return done(Status.ARRIVED, '옮긴 목표 %.2f m 앞, 더 다가가면 부딪힘' % dist)

        if arrived:
            self.prev = None
            out['goal_dist_m'] = dist
            out['passed'] = ai['passed']
            return done(Status.ARRIVED)
        # 칸 경로 (목표에서 퍼진 Dijkstra = A* 와 같은 최단 경로)
        t1 = time.time()
        G = _graph(cm)
        tail = None                                                   # [261011] 'entry': 진입점 → 목표 직선 (격자 평면 점들)
        route_goal = grc
        if gth is not None and app.heading_mode == 'entry':
            u = np.array([math.cos(gth), math.sin(gth)])
            e_s, e_n = -dv @ u, -dv[0] * u[1] + dv[1] * u[0]
            on_line = (-max(app.entry_dists) - 0.05 <= e_s <= 0.0 and abs(e_n) <= 0.15
                       and abs(P._wrap(th - gth)) <= math.radians(45.0))
            if on_line:                                                 # 이미 진입선 위: 목표까지 직선
                n_ = max(2, int(math.ceil(-e_s / 0.025)) + 1)
                tail = np.array([gxy - u * (-e_s) * (1 - k / (n_ - 1)) for k in range(n_)])
            else:
                for de in app.entry_dists:
                    exy = gxy - u * de
                    erc = g.cell(exy)
                    if not goal_ok(erc):
                        continue
                    seg = [gxy - u * de * (1 - k / max(1, int(de / 0.025))) for k in range(int(de / 0.025) + 1)]
                    if all(P._pose_ok(cm, q[0], q[1], gth, offs) for q in seg):
                        route_goal, tail = erc, np.array(seg)
                        out['entry_m'] = de
                        break
                if tail is None:
                    out['entry_blocked'] = True
        D, pred = fast_dijkstra(cm, [route_goal], graph=G, pred=True)
        out['ms']['route'] = (time.time() - t1) * 1000
        if not np.isfinite(D[srow]):
            self.prev = None
            return done(Status.NO_ROUTE)
        route = route_from(pred, srow, g.nx)
        new_cost = float(D[srow])
        # 히스테리시스: 직전 길이 아직 쓸 만하고 비용 차이가 작으면 유지
        if self.prev is not None and self.prev.get('goal') == route_goal and app.hysteresis > 0:
            old = self.prev['route']
            C = np.array([g.center(rc) for rc in old])
            dd = np.hypot(C[:, 0] - mid[0], C[:, 1] - mid[1])
            k = int(np.argmin(dd))
            if dd[k] <= app.reuse_dist_m:
                Dk, pk = fast_dijkstra(cm, [old[k]], graph=G, pred=True)
                if np.isfinite(Dk[srow]):
                    head = route_from(pk, srow, g.nx)
                    cand = head + list(old[k + 1:])
                    oc = route_cost(cm, cand)
                    if oc <= new_cost * (1 + app.hysteresis):
                        route, out['kept'] = cand, True
        out['route'], out['route_cost'] = route, route_cost(cm, route)
        self.prev = dict(route=route, goal=route_goal)
        route_xy = np.array([g.center(rc) for rc in route])
        route_xy[-1] = gxy if tail is None else tail[0]
        route_xy = np.vstack([mid, route_xy[1:]]) if len(route_xy) > 1 else np.vstack([mid, route_xy[-1]])
        if tail is not None:
            route_xy = np.vstack([route_xy, tail[1:]]) if not (gth is not None and out.get('entry_m') is None) else np.vstack([mid, tail])
        route_len = float(np.sum(np.hypot(*np.diff(route_xy, axis=0).T))) if len(route_xy) > 1 else 0.0
        # 회전반경 처리
        t2 = time.time()
        kmax = self.k_scale / rp.r_min                                 # [10/11] 회전 부족이면 다듬기도 덜 꺾게
        local = app.method
        if gth is not None:
            gkey = (round(float(gxy[0]), 3), round(float(gxy[1]), 3), round(float(gth), 3))
            sticky = (self.head_on_key == gkey and route_len <= app.switch_m + 1.0)   # 방향 탐색을 시작했으면 고리 도는 동안 칸 경로가 잠깐 길어져도 그대로 (전환 떨림 방지)
            far_failed = (app.heading_mode == 'hybrid' and self.head_far_fail == gkey and route_len > app.switch_m)
            if (app.heading_mode == 'hybrid' and not far_failed) or (app.heading_mode in ('mixed', 'dubins') and (route_len <= app.switch_m or sticky)):
                local = 'heading'
                self.head_on_key = gkey
            elif far_failed:
                out['heading_deferred'] = True                         # 'hybrid': 멀리서 한 번 못 찾았으면 switch_m 안에 들 때까지 다시 안 찾음 (매 계획 0.5 s 낭비 방지)
            elif app.heading_mode == 'entry' and tail is None:
                local = 'drop'                                        # 진입점이 다 막힘 → 방향 못 맞춤
        out['local'] = local
        if local == 'heading':
            poses, why_h = self._heading_search(g, cm, G, mid, th, gxy, gth, offs, t0, route_len,
                                                shot_only=app.heading_mode == 'dubins')
            out['head_search'] = why_h
            if poses is not None:
                out['ms']['local'] = (time.time() - t2) * 1000
                return self._finish(out, g, poses, base_xy, done, t0)
            if why_h == 'no_time':                                    # 앞 단계에서 시간을 다 씀 (방향 탐색은 시작도 못 함)
                out['ms']['local'] = (time.time() - t2) * 1000
                return done(Status.TIMEOUT, '계산 시간 초과 (방향 맞춤 탐색 전)')
            if route_len > app.switch_m:                              # 'hybrid' 에서 아직 멀 때: 위치로 다가가고 방향은 switch_m 안에서 다시
                out['heading_deferred'] = True
                self.head_far_fail = (round(float(gxy[0]), 3), round(float(gxy[1]), 3), round(float(gth), 3))
                local = app.method
            else:
                local = 'drop'                                        # 목표 근처에서 예산 안에 방향 맞출 길 없음
        if local == 'drop':
            if app.heading_fail == 'stop':
                self.prev = None
                out['ms']['local'] = (time.time() - t2) * 1000
                return done(Status.NO_HEADING, '목표 방향(%.0f°)으로 들어갈 공간 없음' % math.degrees(goal_yaw))
            # 'position': 위치는 이미 맞았으면 방향 없이 도착. 아직이면 이번엔 위치로만 다가가고 다음 계획에서 방향을 다시 시도
            # (멀리선 못 보던 곳이 보이면 길이 생길 수 있음 — 10/11 시뮬: 경계 근처 미관측 금지 때문에 처음엔 막힘)
            self.prev = None
            if ai['pos_ok']:
                self.heading_dropped = self.goal_key(goal_odom, goal_yaw)
                out['heading_dropped'] = True
                out['goal_dist_m'] = dist
                out['ms']['local'] = (time.time() - t2) * 1000
                return done(Status.ARRIVED, '도착 — 목표 방향으로 들어갈 공간 없음: 위치만 맞춤 (방향 오차 %.0f°)'
                            % math.degrees(ai['e_th']))
            res = self.plan(amap, base_p, R_body, goal_odom, None)
            res['heading_deferred'] = True
            res['heading_used'] = app.heading_mode
            res['e_th'] = ai['e_th']
            if res['status'] == Status.ARRIVED:                           # 위치 규칙('지나침' 포함)으로 도착
                self.heading_dropped = self.goal_key(goal_odom, goal_yaw)
                res['heading_dropped'] = True
                res['reason'] = '도착 — 목표 방향으로 들어갈 공간 없음: 위치만 맞춤 (방향 오차 %.0f°)' % math.degrees(ai['e_th'])
            elif res['status'] == Status.OK:
                res['reason'] = '목표 방향으로 들어갈 길 아직 없음 → 위치로 다가감 (다음 계획에서 다시)'
            return res
        if app.method == 'raw':
            poses = [(float(x), float(y), 0.0, 1) for x, y in route_xy]
            for i in range(len(poses)):
                a = route_xy[min(i + 1, len(poses) - 1)] - route_xy[max(i - 1, 0)]
                poses[i] = (poses[i][0], poses[i][1], math.atan2(a[1], a[0]) if np.any(a) else th, 1)
            poses[0] = (mid[0], mid[1], th, 1)
        elif app.method == 'smooth':
            best = None
            for Ld in app.lookaheads:
                ps, L, why = pursuit_rollout(cm, route_xy, (mid[0], mid[1], th), kmax, Ld, app.horizon_m, offs,
                                             krate=app.steer_rate, kap0=getattr(self, 'kap_now', 0.0))
                if why != 'collide':
                    best = (ps, L, why)
                    break
                if best is None or L > best[1]:
                    best = (ps, L, why)
            ps, L, why = best
            if why == 'collide':
                if L < app.min_ok_m:
                    if gth is not None and app.heading_mode == 'mixed':    # [261011] 방향 목표: 다듬기가 막히면 방향 탐색으로 한 번 더 (멀어도)
                        hp, why_h = self._heading_search(g, cm, G, mid, th, gxy, gth, offs, t0, route_len)
                        out['head_search'] = why_h
                        if hp is not None:
                            self.head_on_key = (round(float(gxy[0]), 3), round(float(gxy[1]), 3), round(float(gth), 3))
                            out['local'] = 'heading'
                            out['ms']['local'] = (time.time() - t2) * 1000
                            return self._finish(out, g, hp, base_xy, done, t0)
                    if app.smooth_fallback:                               # [261014] 다듬기(경로 따라가기)는 못 하지만 꺾어 돌아 나갈 길이 있을 수 있음
                        hp = self._hybrid_sub(cm, G, g, route_xy, mid, th, t0)
                        if hp is not None:
                            out['local'] = 'smooth_fallback'
                            out['ms']['local'] = (time.time() - t2) * 1000
                            return self._finish(out, g, hp, base_xy, done, t0)
                    out['ms']['local'] = (time.time() - t2) * 1000
                    if out.get('goal_moved_m') and dist <= app.moved_goal_arrive_m:
                        return near_moved_arrived()
                    return done(Status.INFEASIBLE, '회전반경 안에서 따라가면 %.2f m 앞에서 막힘' % L)
                keep = max(2, int((L - app.stop_before_m) / 0.025))
                ps = ps[:keep]
            poses = [(x, y, a, 1) for x, y, a in ps]
        elif app.method == 'hybrid':
            s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(route_xy, axis=0).T))])
            poses = None
            for sd in app.sub_dists:
                final = s[-1] <= sd
                k = len(route_xy) - 1 if final else int(np.searchsorted(s, sd))
                sub = route_xy[k]
                left = app.time_limit_s - (time.time() - t0)
                if left <= 0.05:
                    break
                ppl = replace(pp, goal_tol_m=0.5 * pp.goal_tol_m if final else app.sub_tol_m,   # 마지막은 도착 판정(10 cm)보다 안쪽까지
                              max_expand=app.hybrid_expand, time_limit_s=left)
                h2d = fast_dijkstra(cm, [g.cell(sub)], graph=G)
                hr = P.hybrid_astar(cm, (mid[0], mid[1], th), tuple(sub), rp, ppl, h2d=h2d)
                if hr['poses']:
                    poses = hr['poses']
                    break
            if poses is None:
                out['ms']['local'] = (time.time() - t2) * 1000
                if time.time() - t0 > app.time_limit_s:
                    return done(Status.TIMEOUT)
                if out.get('goal_moved_m') and dist <= app.moved_goal_arrive_m:
                    return near_moved_arrived()
                return done(Status.INFEASIBLE, '부분 목표까지 전진 경로 없음')
        else:
            raise ValueError(app.method)
        out['ms']['local'] = (time.time() - t2) * 1000
        return self._finish(out, g, poses, base_xy, done, t0)

    def _finish(self, out, g, poses, base_xy, done, t0):
        app, rp, pp = self.app, self.rp, self.pp
        poses = P.resample(poses, pp.out_step_m)
        if app.window_m > 0 and len(poses) > 2:                     # 판정 창 밖(아직 판정 안 된 곳)으로 나가는 부분은 안 냄
            inside = [math.hypot(p[0] - base_xy[0], p[1] - base_xy[1]) <= app.window_m for p in poses]
            if not all(inside):
                k = inside.index(False)
                poses = poses[:max(2, k)]
                out['window_cut'] = True
        out['kmax'] = P.max_curvature([(p[0], p[1]) for p in poses])
        out['path'] = P.to_odom_path(g, poses, rp, ref='mid')
        out['poses_g'] = poses
        if time.time() - t0 > app.time_limit_s:
            out['over_time'] = True
        return done(Status.OK)

    def _hybrid_sub(self, cm, G, g, route_xy, mid, th, t0):
        """[261014] 칸 경로 위 부분 목표(sub_dists)까지 짧은 구간 Hybrid A* (method='hybrid' 와 같은 규칙). 못 찾으면 None."""
        app, rp, pp = self.app, self.rp, self.pp
        s = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(route_xy, axis=0).T))])
        t_end = time.time() + app.fallback_time_s                         # 대신 찾기 전체 시간 상한 (시뮬: 묶지 않으면 최대 1.09 s)
        for sd in app.sub_dists:
            final = s[-1] <= sd
            k = len(route_xy) - 1 if final else int(np.searchsorted(s, sd))
            sub = route_xy[k]
            left = min(app.time_limit_s - (time.time() - t0), t_end - time.time())
            if left <= 0.05:
                return None
            ppl = replace(pp, goal_tol_m=0.5 * pp.goal_tol_m if final else app.sub_tol_m,
                          max_expand=app.hybrid_expand, time_limit_s=left)
            h2d = fast_dijkstra(cm, [g.cell(sub)], graph=G)
            hr = P.hybrid_astar(cm, (mid[0], mid[1], th), tuple(sub), rp, ppl, h2d=h2d)
            if hr['poses']:
                return hr['poses']
        return None

    def _heading_search(self, g, cm, G, mid, th, gxy, gth, offs, t0, route_len, shot_only=False):
        """[261011] (x, y, θ) 목표까지 전진만 Hybrid A* (+ Dubins 해석적 확장). 목표 앞 lead_in_m 은 직선.
        반환 (poses [(x, y, th, d)] 또는 None, 'ok' | 'timeout' | 'none')."""
        app, pp = self.app, self.pp
        rp = self.rp_head
        u = np.array([math.cos(gth), math.sin(gth)])
        key = (round(float(gxy[0]), 3), round(float(gxy[1]), 3), round(float(gth), 3))
        if app.head_keep and self.prev_head is not None and self.prev_head['key'] == key:
            old = self.prev_head['poses']
            Q = np.array([(q[0], q[1]) for q in old])
            dd = np.hypot(Q[:, 0] - mid[0], Q[:, 1] - mid[1])
            k = int(np.argmin(dd))
            if (dd[k] <= 0.10 and abs(P._wrap(old[k][2] - th)) <= math.radians(15.0) and len(old) - k >= 3
                    and all(P._pose_ok(cm, q[0], q[1], q[2], offs) for q in old[k:])):
                k2 = k + 1                                            # 지금 자리에서 0.15 m 앞 점으로 이음 (바로 다음 점에 이으면 옆 어긋남만큼 첫 선분이 꺾임 — 10/11 우분투)
                while k2 < len(old) - 2 and math.hypot(old[k2][0] - mid[0], old[k2][1] - mid[1]) < 0.15:
                    k2 += 1
                return [(mid[0], mid[1], th, 1)] + list(old[k2:]), 'ok'
        self.prev_head = None
        lead = 0.0
        if app.lead_in_m > 0:
            n_ = max(1, int(round(app.lead_in_m / 0.025)))
            seg = [gxy - u * app.lead_in_m * (1 - k / n_) for k in range(n_ + 1)]
            if all(P._pose_ok(cm, q[0], q[1], gth, offs) for q in seg):
                lead = app.lead_in_m
        sub = gxy - u * lead
        left = min(app.head_time_s, app.time_limit_s - (time.time() - t0))
        if left <= 0.05:
            return None, 'no_time'
        if not cm.free(g.cell(sub)):
            return None, 'none'
        if shot_only:                                                 # 'dubins': 탐색 없이 지금 자세 → 목표 자세 Dubins 곡선 하나 (장애물 무시로 그은 뒤 차체 검사)
            import dubins as DB
            L_, w_, tpq = DB.shortest((mid[0], mid[1], th), (sub[0], sub[1], gth), rp.r_min)
            seg = DB.sample((mid[0], mid[1], th), w_, tpq, rp.r_min, pp.sub_m) if w_ else []
            if not seg or not all(P._pose_ok(cm, q[0], q[1], q[2], offs) for q in seg):
                return None, 'none'
            hr = dict(poses=[(mid[0], mid[1], th, 1)] + [(q[0], q[1], q[2], 1) for q in seg])
        else:
            hr = None
        ppl = replace(pp, goal_tol_m=app.head_tol_frac * pp.goal_tol_m, max_expand=app.head_expand, time_limit_s=left)
        if hr is None:
            h2d = fast_dijkstra(cm, [g.cell(sub)], graph=G) if app.head_use_h2d else np.zeros((g.ny, g.nx))
            hr = P.hybrid_astar(cm, (mid[0], mid[1], th), tuple(sub), rp, ppl, h2d=h2d, goal_th=gth,
                                th_tol=app.head_tol_frac * math.radians(app.th_tol_deg),
                                shot_dist=max(route_len, 1.0) + 0.5, shot_every=app.shot_every, h_weight=app.head_weight)
            out_stats = self.last_head_stats = dict(expanded=hr['expanded'], ms=hr['ms'], shot=hr.get('shot'), word=hr.get('shot_word'))   # noqa: F841
        if not hr['poses']:
            return None, 'budget' if (hr.get('timeout') or hr['expanded'] > app.head_expand) else 'none'
        poses = list(hr['poses'])
        if lead > 0:
            n_ = max(1, int(round(lead / 0.025)))
            poses += [(float(q[0]), float(q[1]), gth, 1) for q in
                      [sub + u * lead * k / n_ for k in range(1, n_ + 1)]]
        self.prev_head = dict(key=key, poses=poses)
        return poses, 'ok'


# ------------------------------------------------------------------ 갱신 품질·경로 검사 (ROS 노드용)
def band_depth(g, xy_odom) -> float:
    """[261016] 그 점(odom x, y)이 경계 금지 띠 안이면 띠 안쪽 끝까지 거리 [m] (> 0), 아니면 0. 랜더는 띠가 아님."""
    rc = g.cell(g.odom_to_g(np.array([xy_odom[0], xy_odom[1], 0.0])))
    if not (g.inside(rc) and g.edge_band[rc]):
        return 0.0
    return float(ndimage.distance_transform_edt(g.edge_band)[rc] * g.res)


def open_band_exit(amap: ArenaMap, g, cm, srow, mid_xy, rp: P.RoverParams, escape_r_m: float) -> int:
    """[261016] 출발 자리가 경계 금지 띠(반폭 + edge_margin) 안일 때, 운영자가 허용하면 띠 밖으로 나가는 칸만 비싸게 연다.
    연 칸 = 띠 안 '깊이'(띠 안쪽 끝까지 거리)가 지금 자리 + 차체 앞뒤 검사점 + 한 칸 이하인 칸 → 경계 쪽으로 더 들어가지는 않는다.
    그중 관측돼 통과 판정인 칸(0 <= 비용 < 100), 그리고 서 있는 자리 둘레(escape 반경, 로버가 실제로 서 있음)만.
    랜더는 띠가 아니라 열지 않는다. 실제로 나갈 수 있는지(회전반경)는 계획기가 그대로 본다. 반환 = 연 칸 수."""
    if not (g.inside(srow) and g.edge_band[srow]):
        return 0
    depth = ndimage.distance_transform_edt(g.edge_band) * g.res
    tol = max(abs(a) for a in rp.footprint_offsets()) + g.res
    shallow = g.edge_band & (depth <= depth[srow] + tol)
    rr, cc = np.ogrid[:g.ny, :g.nx]
    k = int(math.ceil(escape_r_m / g.res))
    disk = (rr - srow[0]) ** 2 + (cc - srow[1]) ** 2 <= k * k
    seen_ok = (amap.cost >= 0) & (amap.cost < 100)
    ok = shallow & (seen_ok | disk) & ~np.isfinite(cm.pen)
    cm.pen[ok] = 5.0                                                # allow_escape 와 같은 값 — 빨리 나가는 길이 싸다
    return int(ok.sum())


def path_blocked(amap: ArenaMap, path_odom, rp: P.RoverParams, ahead_m=None, start_idx=0, rover_mid_odom=None, rover_yaw=None,
                 band_exit=False):
    """현재 경로(odom, 두 축 가운데 점)가 최신 지도에서 불가·금지 칸을 차체 검사점으로 밟는지. ahead_m 까지만.
    rover_mid_odom·rover_yaw 를 주면 계획기(ArenaPlanner.plan)와 같게, 로버가 서 있는 자리가 금지 칸이면 그 둘레를 풀어 줌
    (안 그러면 불가 칸 옆에 선 채로 방금 낸 경로가 첫 점에서 막힘으로 잡혀 틱마다 다시 계획됨 - 10/6 우분투 시험 ot1_side)."""
    if not path_odom:
        return False
    g = amap.grid()
    pp = P.PlanParams(soft_m=0.0)
    cm = P.CostMap(g, pp)
    cm.pen[g.hard] = np.inf
    offs = rp.footprint_offsets()
    if rover_mid_odom is not None and rover_yaw is not None:
        m = g.odom_to_g(np.array([rover_mid_odom[0], rover_mid_odom[1], 0.0]))
        th0 = P._wrap(rover_yaw - amap.T[2])
        if g.inside(g.cell(m)) and (not cm.free(g.cell(m)) or not P._pose_ok(cm, m[0], m[1], th0, offs)):
            cm.allow_escape(m, max(pp.escape_m, rp.length_m / 2))
            cm.pen[g.hard] = np.inf
            if band_exit:                                               # [261016] 계획기와 같게 — 안 그러면 띠를 나가는 경로가 틱마다 '막힘'
                open_band_exit(amap, g, cm, g.cell(m), m, rp, max(pp.escape_m, rp.length_m / 2))
    L = 0.0
    prev = None
    for p in path_odom[start_idx:]:
        xy = g.odom_to_g(np.array([p[0], p[1], 0.0]))
        if prev is not None:
            L += math.hypot(*(xy - prev))
            if ahead_m is not None and L > ahead_m:
                break
        prev = xy
        th = P._wrap(p[3] - amap.T[2])
        if g.inside(g.cell(xy)) and not P._pose_ok(cm, xy[0], xy[1], th, offs):
            return True
    return False


def seen_fraction_ahead(amap: ArenaMap, path_odom, ahead_m=1.0, start_idx=0):
    """경로 앞 ahead_m 구간 중 판정된(관측) 칸 비율."""
    if not path_odom:
        return 1.0
    g = amap.grid()
    n = k = 0
    L, prev = 0.0, None
    for p in path_odom[start_idx:]:
        xy = g.odom_to_g(np.array([p[0], p[1], 0.0]))
        if prev is not None:
            L += math.hypot(*(xy - prev))
            if L > ahead_m:
                break
        prev = xy
        rc = g.cell(xy)
        if g.inside(rc):
            n += 1
            k += amap.cost[rc] >= 0
    return k / n if n else 1.0
