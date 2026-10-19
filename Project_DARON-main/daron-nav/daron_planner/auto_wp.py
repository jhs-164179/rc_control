"""완전 자율(AUTO) 다음 웨이포인트 고르기 — 사전 방문점 + 현장 규칙 [261014 시제품, 브랜치 auto-1014 에서 planner_node AUTO 에 붙임].

케일럽 10/14: 경기장이 작고 정해져 있으니 프론티어 같은 탐색 대신 '미리 정한 점 + 로버가 규칙으로 판단'.
              순서만 지키지 말고 막히거나 각이 안 나오면 더 좋은 다른 점으로. 출발 방향이 +y 일지 90° 돌려 넣을지 모름.
              크레이터 관측점은 지금 빼고 주행 먼저.

대회 전 (make_points): 경기장 안, 경계에서 margin_m 안쪽에 방문점 격자. 간격 = U턴 지름 정도.
                      선호 순서 = 출발 방향과 나란한 줄로 ㄹ 자(부스트로페돈) — '동점 깨기'용일 뿐 강제 아님.
대회 중 (AutoSelector.update): 남은 점마다
    비용 = 지금 지도에서 가는 비용(다익스트라, 안 본 칸은 통과 가정·벌점 — 경로계획과 같은 규칙)
         + 방향 바꾸는 비용(w_turn x |지금 방향과 점 방향 차이|, 대략 회전반경 x 각도 = 돌아가는 거리)
         + 선호 순서에서 앞질러 가는 비용(w_order x 순서 차이)
  가장 싼 점이 다음 목표 (지금 목표보다 switch_gain_m 넘게 싸야 바꿈 — 흔들림 방지).
  건너뜀: 점 0.5 m 안에 설 자리 없음(불가·금지), 길 없음, 경로계획이 같은 점에서 연속 실패, 진전 없음(stall).
  지나감: 기준점(두 축 가운데)이 arrive_m 안 → 서지 않고 다음 점 (방향 조건 없음 = 목표 앞 맴돎 없음).
  끝: 남은 점이 없거나 시간 한도 → 페이즈2 틈.
"""
import math
from dataclasses import dataclass

import numpy as np
from scipy import ndimage

import planner_core as P
from arena_planner import fast_dijkstra, open_band_exit


@dataclass
class AutoParams:
    field_w: float = 4.0
    field_l: float = 5.0
    margin_m: float = 0.8          # 경계에서 점까지 최소. 10/14 0.6 (금지 띠 0.30 + 밀림 여유 0.30, 케일럽 OK) → 시뮬에서 바깥 줄이
                                   #   금지 띠에 붙어 왼쪽으로 꺾을 공간이 없고 앞이 막히면 전진만으로 못 빠져나옴(OT 유사 2/9 에서 멈춤) → 0.8
    lane_gap_m: float = 1.2        # 줄 사이 최소 간격 = U턴 지름 (반경 0.6 = 계획 0.44 x 1.36 → 실제 회전이 지령의 73 % 까지 버팀)
    along_m: float = 1.3           # 줄을 따라 점 간격 상한
    n_lanes: int = 0               # (옛 비교용) >0 이면 '출발 방향 기준' 줄 수 — 넣는 방향에 따라 점 자리가 달라짐
    n_along: int = 0
    n_cols: int = 3                # [10/14 밤] 경기장에 고정된 격자: x 방향(4 m 변) 열 수 — 넣는 방향이 바뀌어도 점 자리는 같음
    n_rows: int = 4                #   y 방향(5 m 변, 페이즈2 쪽) 행 수. 3x4(케일럽 안) vs 4x4: 22판 틈 12 vs 14·관측 68 vs 70 % (잡음 수준),
                                   #   최악 조건에서는 3x4 가 셋 중 가장 나음 → 3x4 (daron_sim/auto_compare.py)
    inset_y_m: float = 0.6         #   행은 y 로 이만큼 더 안쪽 (긴 변 끝 U턴 여유). 열은 margin 그대로
    end_inset_m: float = 0.6       # 줄 끝 점을 이만큼 더 안쪽에 (U턴 반경: 줄 끝에서 꺾는 고리가 점 한계 안에 들게)
    clear_m: float = 0.5           # 점 둘레 이 반경 안에 불가·금지 칸이 없어야 함 (회전반경 0.44 + 여유: 거기서 꺾을 수 있게).
                                   #   모자라면 snap_m 안에서 그런 자리로 옮기고, 없으면 건너뜀
    lanes: str = 'start'           # 선호 순서의 줄 방향: start(출발 방향과 나란히) | x | y
    arrive_m: float = 0.30         # 지나감 판정
    snap_m: float = 0.5            # 점이 불가·금지면 이 안에서 설 자리 찾기
    w_turn: float = 0.55           # [m/rad] 방향 바꾸는 비용 (회전반경 0.44~0.65 정도)
    w_order: float = 0.3           # [m/순서] 선호 순서보다 앞질러 갈 때 비용
    switch_gain_m: float = 0.5     # 지금 목표보다 이만큼 싸야 바꿈
    hold_s: float = 2.0            # [10/15] 한 번 고른 점은 이 시간 동안 다른 점으로 안 바꿈 (막힘·지나감·건너뜀은 예외).
                                   #   우분투 10/15: 점 5 설 자리가 지도 갱신으로 조금씩 옮겨지며 1.2 s 동안 5→6→5 로 바뀜
    fail_max: int = 3              # 같은 점에서 경로계획 실패(INFEASIBLE 등) 연속 이만큼이면 건너뜀
    stall_s: float = 20.0          # 이 시간 동안
    stall_m: float = 0.10          #   목표까지 거리가 이만큼도 안 줄면 건너뜀
    time_limit_s: float = 480.0    # AUTO 시작 뒤 이 시간이 지나면 남은 점을 버리고 틈으로
    gap_xy: tuple = (1.5, 2.3)     # 페이즈2 틈 (경기장 좌표) — 연습 주행에서 확인 [가정]


def make_points(ap: AutoParams, start):
    """방문점 [(x, y)] 와 선호 순서(0 부터). start = (기준점 x, y, yaw) 경기장 좌표."""
    x0, x1 = -ap.field_w / 2 + ap.margin_m, ap.field_w / 2 - ap.margin_m
    y0, y1 = -ap.field_l / 2 + ap.margin_m, ap.field_l / 2 - ap.margin_m
    sx, sy, syaw = start
    lanes = ap.lanes
    if lanes == 'start':
        lanes = 'y' if abs(math.sin(syaw)) >= abs(math.cos(syaw)) else 'x'

    def ticks(a0, a1, step, across):
        span = a1 - a0
        n = (int(math.floor(span / step + 1e-9)) if across else int(math.ceil(span / step - 1e-9))) + 1
        if across and ap.n_lanes > 0:
            n = ap.n_lanes
        if not across and ap.n_along > 0:
            n = ap.n_along
        return np.linspace(a0, a1, n) if n > 1 else np.array([(a0 + a1) / 2])
    # 줄 = lanes 방향으로 뻗은 점들. 줄 사이 >= lane_gap_m, 줄 따라 <= along_m.
    # 첫 줄 = 출발에 가까운 줄, 첫 줄 진행 방향 = 출발 방향 (ㄹ 자)
    if ap.n_cols > 0 and ap.n_rows > 0 and ap.n_lanes <= 0:          # 경기장 고정 격자
        cols = np.linspace(x0, x1, ap.n_cols) if ap.n_cols > 1 else np.array([(x0 + x1) / 2])
        rows = np.linspace(y0 + ap.inset_y_m, y1 - ap.inset_y_m, ap.n_rows) if ap.n_rows > 1 else np.array([(y0 + y1) / 2])
        if lanes == 'y':
            lane_c, along, near, first_dir, mk = list(cols), list(rows), sx, (1 if math.sin(syaw) >= 0 else -1), (lambda c, a: (c, a))
        else:
            lane_c, along, near, first_dir, mk = list(rows), list(cols), sy, (1 if math.cos(syaw) >= 0 else -1), (lambda c, a: (a, c))
    elif lanes == 'y':
        lane_c, along = list(ticks(x0, x1, ap.lane_gap_m, True)), list(ticks(y0 + ap.end_inset_m, y1 - ap.end_inset_m, ap.along_m, False))
        first_dir = 1 if math.sin(syaw) >= 0 else -1
        near = sx
        mk = lambda c, a: (c, a)
    else:
        lane_c, along = list(ticks(y0, y1, ap.lane_gap_m, True)), list(ticks(x0 + ap.end_inset_m, x1 - ap.end_inset_m, ap.along_m, False))
        first_dir = 1 if math.cos(syaw) >= 0 else -1
        near = sy
        mk = lambda c, a: (a, c)
    far_side = 1 if lane_c[int(np.argmin([abs(c - near) for c in lane_c]))] <= np.mean(lane_c) else -1
    lane_order = sorted(range(len(lane_c)), key=lambda i: far_side * lane_c[i])
    pts, rank, axis = [], [], []
    d = first_dir
    lane_axis = math.pi / 2 if lanes == 'y' else 0.0                 # 줄 방향 (부호는 쓸 때 정함)
    for k, li in enumerate(lane_order):
        seq = along if d > 0 else along[::-1]
        for a in seq:
            pts.append(mk(lane_c[li], a)); rank.append(len(rank)); axis.append(lane_axis)
        d = -d
    make_points.axis = np.array(axis, float)
    return np.array(pts, float), np.array(rank, int)


class AutoSelector:
    """경기장 좌표 = odom 으로 쓰는 경우(시뮬) 그대로. 노드에서는 to_odom 으로 점을 옮겨 넣으면 됨."""

    VISITED, SKIPPED, LEFT = 'visited', 'skipped', 'left'

    def __init__(self, ap: AutoParams, start, rp: P.RoverParams = None, pp: P.PlanParams = None, to_odom=None):
        self.ap, self.rp, self.pp = ap, rp or P.RoverParams(), pp or P.PlanParams()
        pts, rank = make_points(ap, start)
        self.axis = make_points.axis.copy()                           # 점마다 줄 방향 [rad] (방향 목표로 시험할 때)
        self.pts = np.array([to_odom(p) for p in pts]) if to_odom else pts
        self.rank = rank
        self.gap = np.asarray(to_odom(ap.gap_xy) if to_odom else ap.gap_xy, float)
        self.state = [self.LEFT] * len(self.pts)
        self.why = [''] * len(self.pts)
        self.order = []                    # 실제로 지나간 순서 (점 번호)
        self.visit_xy = {}                 # 점 번호 -> 지나갈 때 기준점 위치
        self.mid = None
        self.target = None                 # 점 번호 또는 'gap'
        self.tgt_xy = None                 # 실제로 겨눈 자리 (옮겼으면 옮긴 곳)
        self.fails = 0
        self.band_exit = False          # [261016] 노드가 켬 — 경계 띠 안에서 나가는 칸 열기 (open_band_exit)
        self.inf_targets = set()            # 움직이지 못한 채 INFEASIBLE 로 건너뛴 점들 (갇힘 판정)
        self.best_d, self.best_t = math.inf, 0.0
        self.t_sel = -math.inf             # 지금 목표를 고른 시각 (hold_s)
        self.t0 = None
        self.log = []
        sx, sy, _ = start
        if to_odom:                        # 점은 odom 으로 옮겼으니 출발 자리도 odom 으로 비교 (시뮬은 경기장 = odom 이라 같음)
            sx, sy = (float(v) for v in np.asarray(to_odom((sx, sy)), float)[:2])
        for i, p in enumerate(self.pts):   # 출발 자리 근처 점은 지나간 것으로
            if math.hypot(p[0] - sx, p[1] - sy) <= max(ap.arrive_m, 0.45):
                self.state[i], self.why[i] = self.VISITED, '출발 자리'

    # -------------------------------------------------------------- 내부
    def _skip(self, i, why, t):
        self.state[i], self.why[i] = self.SKIPPED, why
        self.log.append((round(t, 1), 'skip', int(i), why))
        if self.target == i:
            self.target, self.tgt_xy = None, None

    def _stand_xy(self, cm, g, i, clear):
        """점 i 에 설 자리 (둘레 clear_m 안에 아는 불가 칸·사전 금지가 없는 칸. 아니면 snap_m 안 가장 가까운 그런 칸). 없으면 None.
        clear = 아는 불가 칸 + 사전 금지 띠까지 거리 [m] — 안 본 칸은 막는 것으로 치지 않음(가까이 가면 보임, 경로계획이 다시 판단)."""
        need = max(max(abs(a) for a in self.rp.footprint_offsets()) + 0.5 * g.res, self.ap.clear_m)
        p = self.pts[i]
        gxy = g.odom_to_g(np.array([p[0], p[1], 0.0]))[:2]
        rc = g.cell(gxy)
        ok = lambda rc_: clear[rc_] >= need and (cm.free(rc_) or cm.unknown[rc_])
        if g.inside(rc) and ok(rc):
            return p
        k = int(math.ceil(self.ap.snap_m / g.res))
        best, bd = None, math.inf
        for dr in range(-k, k + 1):
            for dc in range(-k, k + 1):
                d = math.hypot(dr, dc) * g.res
                rc2 = (rc[0] + dr, rc[1] + dc)
                if d <= self.ap.snap_m and d < bd and g.inside(rc2) and ok(rc2):
                    best, bd = rc2, d
        if best is None:
            return None
        o = g.g_to_odom(g.center(best))
        return np.array([o[0], o[1]])

    # -------------------------------------------------------------- 밖에서 부르는 것
    def update(self, amap, mid_xy, yaw, t):
        """지금 겨눌 목표 (x, y) 와 이유. 틈으로 갈 때는 ('gap', gap_xy)."""
        ap = self.ap
        if self.t0 is None:
            self.t0 = t
        mid_xy = np.asarray(mid_xy, float)
        self.mid = mid_xy
        # 지나감
        for i, p in enumerate(self.pts):
            if self.state[i] == self.LEFT and math.hypot(*(p - mid_xy)) <= ap.arrive_m:
                self.state[i], self.why[i] = self.VISITED, '지나감'
                self.order.append(i); self.visit_xy[i] = mid_xy.copy(); self.log.append((round(t, 1), 'visit', int(i), ''))
                if self.target == i:
                    self.target, self.tgt_xy = None, None
        if self.target not in (None, 'gap') and self.tgt_xy is not None and math.hypot(*(self.tgt_xy - mid_xy)) <= ap.arrive_m:
            i = self.target                                              # 옮긴 자리에 닿음 = 지나감
            self.state[i], self.why[i] = self.VISITED, '옮긴 자리 지나감'
            self.order.append(i); self.visit_xy[i] = mid_xy.copy(); self.log.append((round(t, 1), 'visit', int(i), 'moved'))
            self.target, self.tgt_xy = None, None
        # 진전 없음
        if self.target not in (None, 'gap'):
            d = math.hypot(*(self.tgt_xy - mid_xy))
            if d < self.best_d - ap.stall_m:
                self.best_d, self.best_t = d, t
            elif t - self.best_t > ap.stall_s:
                self._skip(self.target, '진전 없음 %.0f s' % ap.stall_s, t)
        if self.target == 'gap':
            return 'gap', self.gap
        left = [i for i in range(len(self.pts)) if self.state[i] == self.LEFT]
        if not left or t - self.t0 > ap.time_limit_s:
            self.target, self.tgt_xy = 'gap', self.gap
            self.log.append((round(t, 1), 'gap', -1, '남은 점 없음' if not left else '시간 한도'))
            return 'gap', self.gap
        # 비용
        g = amap.grid()
        cm = P.CostMap(g, self.pp)
        cm.pen[g.hard] = np.inf
        mid_g = g.odom_to_g(np.array([mid_xy[0], mid_xy[1], 0.0]))[:2]
        src = g.cell(mid_g)
        if not cm.free(src):
            cm.allow_escape(mid_g, max(self.pp.escape_m, self.rp.length_m / 2)); cm.pen[g.hard] = np.inf
            if self.band_exit:                                       # [261016] 경계 띠 안 — 운영자가 나가기 허용 (계획기와 같은 칸)
                open_band_exit(amap, g, cm, src, mid_g, self.rp, max(self.pp.escape_m, self.rp.length_m / 2))
        if not cm.free(src):
            # [261016] 출발 자리가 막힘(경계 띠 등) = 점 탓이 아님. 예전엔 모든 점이 '길 없음'으로 한꺼번에 건너뛰어졌다(12/12).
            return None, None
        D = fast_dijkstra(cm, [src])
        hard = (amap.cost >= 100) | amap.prior
        clear = ndimage.distance_transform_edt(~hard) * g.res
        next_rank = min(self.rank[i] for i in left)
        best, cur_cost, rows = None, math.inf, []
        for i in left:
            xy = self._stand_xy(cm, g, i, clear)
            if xy is None:
                self._skip(i, '0.5 m 안에 꺾을 공간 있는 자리 없음(불가·금지 가까움)', t); continue
            rc = g.cell(g.odom_to_g(np.array([xy[0], xy[1], 0.0]))[:2])
            dd = D[rc] if g.inside(rc) else math.inf
            if not np.isfinite(dd):
                self._skip(i, '길 없음', t); continue
            dth = abs(math.atan2(math.sin(math.atan2(xy[1] - mid_xy[1], xy[0] - mid_xy[0]) - yaw),
                                 math.cos(math.atan2(xy[1] - mid_xy[1], xy[0] - mid_xy[0]) - yaw)))
            c = dd + ap.w_turn * dth + ap.w_order * (self.rank[i] - next_rank)
            rows.append((i, c))
            if best is None or c < best[1]:
                best = (i, c, xy)
            if i == self.target:
                cur_cost = c
        if best is None:
            self.target, self.tgt_xy = 'gap', self.gap
            self.log.append((round(t, 1), 'gap', -1, '갈 수 있는 점 없음'))
            return 'gap', self.gap
        cur_ok = self.target is not None and self.target != 'gap' and self.state[self.target] == self.LEFT and math.isfinite(cur_cost)
        if not cur_ok or (best[1] < cur_cost - ap.switch_gain_m and t - self.t_sel >= ap.hold_s):
            if self.target != best[0]:
                self.log.append((round(t, 1), 'target', int(best[0]), '%.2f' % best[1]))
                self.best_d, self.best_t, self.fails = math.inf, t, 0
                self.t_sel = t
            self.target, self.tgt_xy = best[0], best[2]
        elif self.target == best[0]:
            self.tgt_xy = best[2]
        else:                                                         # 지금 목표 유지 — 설 자리만 갱신
            self.tgt_xy = self._stand_xy(cm, g, self.target, clear)
            if self.tgt_xy is None:
                self._skip(self.target, '0.5 m 안에 꺾을 공간 있는 자리 없음(불가·금지 가까움)', t)
                return self.update(amap, mid_xy, yaw, t)
        return int(self.target), self.tgt_xy

    def target_yaw(self, mid_xy):
        """[시험용] 지금 목표에 줄 방향으로 들어가는 방향 (부호 = 로버에서 점으로 가는 방향과 가까운 쪽). 틈이면 +y."""
        if self.target == 'gap':
            return math.pi / 2
        if self.target is None:
            return None
        b = math.atan2(self.tgt_xy[1] - mid_xy[1], self.tgt_xy[0] - mid_xy[0])
        a = self.axis[self.target]
        return a if math.cos(b - a) >= 0 else a + math.pi

    def mark_visited(self, i, t, why='지나감'):
        if i in (None, 'gap') or self.state[i] != self.LEFT:
            return
        self.state[i], self.why[i] = self.VISITED, why
        self.order.append(i); self.log.append((round(t, 1), 'visit', int(i), why))
        if self.mid is not None:
            self.visit_xy[i] = self.mid.copy()
        if self.target == i:
            self.target, self.tgt_xy = None, None

    def plan_failed(self, status, t):
        """경로계획이 지금 목표로 실패. 반환 'boxed' = 로버가 갇힘(점 탓이 아님 → 점을 지우지 않음, 후진·운영자 몫).
        GOAL_BLOCKED·NO_ROUTE 는 그 점을 바로 건너뜀. INFEASIBLE(회전반경으로 못 따라감)은 같은 점 연속 fail_max 번이면 건너뛰되,
        움직이지 못한 채 서로 다른 점 2개에서 연이어 INFEASIBLE 이면 '갇힘'으로 봄 (점을 줄줄이 지우는 것 방지)."""
        if self.target in (None, 'gap'):
            return 'gap'
        if status == 'START_BLOCKED':                                    # [261016] 로버 자리 탓 (경계 띠 등) — 점을 지우지도 세지도 않음
            return 'blocked'
        if status in ('GOAL_BLOCKED', 'NO_ROUTE'):
            self._skip(self.target, '경로계획 %s' % status, t)
            return 'skip'
        self.fails += 1
        if self.fails >= self.ap.fail_max:
            if len(self.inf_targets) >= 1 and self.target not in self.inf_targets:
                self.fails = 0
                return 'boxed'
            self.inf_targets.add(self.target)
            self._skip(self.target, '경로계획 %s' % status, t)
            return 'skip'
        return 'retry'

    def plan_ok(self):
        self.fails = 0
        self.inf_targets = set()

    def resume(self, t):
        """[노드] 수동 등으로 멈췄다가 AUTO 로 돌아올 때: 진전 없음 시계·실패 횟수만 새로 (지나간 점·시간 한도는 그대로)."""
        self.best_d, self.best_t, self.fails = math.inf, t, 0
        self.inf_targets = set()

    def skip_target(self, t, why='운영자 건너뜀'):
        """[노드] 지금 목표 점을 건너뜀 (/ui/waypoint_cmd AUTO_SKIP). 틈이면 아무것도 안 함."""
        if self.target in (None, 'gap'):
            return False
        self._skip(self.target, why, t)
        return True

    def summary(self):
        return dict(n=len(self.pts), visited=sum(s == self.VISITED for s in self.state),
                    skipped=sum(s == self.SKIPPED for s in self.state), left=sum(s == self.LEFT for s in self.state),
                    order=list(self.order), why=list(self.why), visit_xy={int(k): tuple(map(float, v)) for k, v in self.visit_xy.items()})
