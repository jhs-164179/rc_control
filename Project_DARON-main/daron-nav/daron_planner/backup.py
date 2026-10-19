"""경로를 못 따라갈 때의 백업 (261011, 교수님: '최소 회전반경은 이상적인 값 — 슬립·경사로 못 지킴, 백업 프로세스 필요').

ROS 없이 쓰는 작은 감시 도구들. 경로계획 노드(ros/planner_node.py)와 시뮬(daron_sim/heading_compare.py)이 같이 씀.
  deviation()     : 실제 자세가 지금 경로에서 얼마나 벗어났나 (옆 거리, 방향 차) → 기준을 넘으면 2 s 를 기다리지 않고 바로 다시 계획
  HeadingRetry    : 방향 목표를 몇 번 놓쳤나 / 목표 근처에서 얼마나 돌았나 → 한도를 넘으면 방향을 포기하고 위치만 (또는 정지)
  StuckWatch      : 지령은 나가는데(또는 경로가 있는데) 위치가 안 변함 = 헛바퀴 → 표시 (설정하면 정지)
  TurnWatch       : 지령(또는 경로) 곡률 대비 실제로 돈 비율 → 기록 (반경 자동 조정은 실물 자료를 보고 나중에)
모두 '두 축 가운데 점'(mid) 기준, 같은 좌표계(odom 또는 경기장)로 줄 것.
"""
import math
from collections import deque

import numpy as np


def _wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def deviation(path_xyyaw, mid_xy, yaw, i_hint=None, window=60):
    """경로 [(x, y, ?, yaw, ...)] 에 대한 (옆 거리 [m], 방향 차 [rad], 가장 가까운 점 번호).
    가장 가까운 '선분'까지 거리. i_hint 를 주면 그 둘레 window 점만 봄 (경로가 겹쳐 지나갈 때 엉뚱한 쪽을 잡지 않게)."""
    if not path_xyyaw or len(path_xyyaw) < 2:
        return 0.0, 0.0, 0
    P = np.asarray([(p[0], p[1]) for p in path_xyyaw], float)
    n = len(P)
    lo, hi = (0, n - 1) if i_hint is None else (max(0, i_hint - window), min(n - 1, i_hint + window))
    a, b = P[lo:hi], P[lo + 1:hi + 1]
    if len(a) == 0:
        return float(np.hypot(P[-1, 0] - mid_xy[0], P[-1, 1] - mid_xy[1])), 0.0, n - 1
    v = b - a
    w = np.asarray(mid_xy, float)[None, :] - a
    t = np.clip(np.sum(w * v, 1) / np.maximum(np.sum(v * v, 1), 1e-12), 0.0, 1.0)
    d = np.hypot(w[:, 0] - t * v[:, 0], w[:, 1] - t * v[:, 1])
    k = int(np.argmin(d))
    j = lo + k
    # 방향 기준 = 경로에 적힌 계획 방향(yaw)을 그 선분 위 위치로 보간. 선분 자체 방향은 쓰지 않음:
    # 직전 경로를 이어 쓸 때(head_keep) 첫 선분이 옆으로 몇 cm 꺾여 있어 짧은 선분 방향이 60° 넘게 튐 (10/11 우분투 2-1 '새 경로 직후 80°')
    y0, y1 = float(path_xyyaw[j][3]), float(path_xyyaw[min(j + 1, n - 1)][3])
    ref = y0 + float(t[k]) * _wrap(y1 - y0)
    return float(d[k]), abs(_wrap(yaw - ref)), j


class HeadingRetry:
    """방향 목표 재시도 한도.
    '놓침' 1번 = 마지막 접근(남은 경로가 final_m 안)에서 기준점이 목표 zone_m 안에 들어왔다가, 방향을 못 맞춘 채 zone_m + leave_m 밖으로 나감.
    또는 목표 near_m 안에서 쌓인 주행 거리가 max_near_m 를 넘음 (계속 맴돎).
    max_miss 번 놓치거나 거리 한도를 넘으면 drop = True (방향 포기 → 위치만 / 정지는 부르는 쪽이 정함)."""

    def __init__(self, max_miss=2, max_near_m=8.0, zone_m=0.30, leave_m=0.10, near_m=1.5, final_m=0.6):
        self.max_miss, self.max_near_m = int(max_miss), float(max_near_m)
        self.zone_m, self.leave_m, self.near_m, self.final_m = float(zone_m), float(leave_m), float(near_m), float(final_m)
        self.reset(None)

    def reset(self, key):
        self.key, self.miss, self.near_dist, self.armed, self.last_xy, self.dropped = key, 0, 0.0, False, None, False

    def enabled(self):
        return self.max_miss > 0 or self.max_near_m > 0

    def update(self, key, mid_xy, goal_xy, rem_path_m, arrived=False, scale=1.0):
        """틱마다. key 가 바뀌면 처음부터. 반환 (drop?, 이유 문자열).
        scale: 거리 한도 배수 (계획 반경을 키웠으면 고리도 커지므로 같은 비율로 — set_turn_ratio)."""
        if key != self.key:
            self.reset(key)
        if key is None or self.dropped or not self.enabled():
            return False, ''
        d = math.hypot(mid_xy[0] - goal_xy[0], mid_xy[1] - goal_xy[1])
        if self.last_xy is not None and d <= self.near_m:
            self.near_dist += math.hypot(mid_xy[0] - self.last_xy[0], mid_xy[1] - self.last_xy[1])
        self.last_xy = (float(mid_xy[0]), float(mid_xy[1]))
        if arrived:
            return False, ''
        if d <= self.zone_m and rem_path_m <= self.final_m:
            self.armed = True
        elif self.armed and d > self.zone_m + self.leave_m:
            self.armed = False
            self.miss += 1
        if self.max_miss > 0 and self.miss >= self.max_miss:
            self.dropped = True
            return True, '방향 맞춤 %d번 놓침' % self.miss
        if self.max_near_m > 0 and self.near_dist > self.max_near_m * max(1.0, scale):
            self.dropped = True
            return True, '목표 %.1f m 안에서 %.1f m 돎' % (self.near_m, self.near_dist)
        return False, ''

    def info(self):
        return dict(miss=self.miss, near_m=round(self.near_dist, 2), dropped=self.dropped)


class StuckWatch:
    """헛바퀴(진행 없음) 감시. active = 지령이 나가는 중(rpm 토픽) 또는 경로를 따라가는 중(대용).
    window_s 동안 기준점 이동이 min_move_m 미만이면 stuck. 위치가 움직이면 바로 풀림."""

    def __init__(self, window_s=4.0, min_move_m=0.03):
        self.window_s, self.min_move_m = float(window_s), float(min_move_m)
        self.hist = deque()
        self.since = None

    def reset(self):
        self.hist.clear()
        self.since = None

    def update(self, t, mid_xy, active):
        """반환 (stuck?, 진행 없음이 이어진 시간 [s])."""
        if not active or self.window_s <= 0:
            self.reset()
            return False, 0.0
        self.hist.append((t, float(mid_xy[0]), float(mid_xy[1])))
        while self.hist and t - self.hist[0][0] > self.window_s:
            self.hist.popleft()
        if not self.hist or t - self.hist[0][0] < self.window_s * 0.9:
            return False, 0.0
        xs = np.array([h[1] for h in self.hist]); ys = np.array([h[2] for h in self.hist])
        span = math.hypot(xs.max() - xs.min(), ys.max() - ys.min())
        if span >= self.min_move_m:
            self.since = None
            return False, 0.0
        if self.since is None:
            self.since = self.hist[0][0]
        return True, t - self.since


class TurnWatch:
    """회전 부족 감시 (기록용). 기준점이 seg_m 갈 때마다 실제 곡률 = 방향 변화 / 이동 거리 를 재고,
    그 구간 지령 곡률의 평균과 비교. 지령 곡률이 min_frac x kmax 이상인 구간만 비율(실제/지령)을 지수 평균."""

    def __init__(self, kmax, seg_m=0.15, min_frac=0.4, alpha=0.3):     # min_frac 0.4: 반경을 2배로 키운 뒤에도(지령 ≈ 0.5 kmax) 계속 잼
        self.kmax, self.seg_m, self.min_frac, self.alpha = float(kmax), float(seg_m), float(min_frac), float(alpha)
        self.reset()

    def reset(self):
        self.start = None
        self.cmd = []
        self.ratio = None
        self.n = 0
        self.last = None

    def update(self, mid_xy, yaw, k_cmd):
        """k_cmd: 지금 지령 곡률 [1/m] (조향 지령 토픽 또는 경로 곡률). None 이면 건너뜀."""
        if k_cmd is None:
            return self.ratio
        if self.start is None:
            self.start, self.yaw_acc, self.prev_yaw, self.cmd = (float(mid_xy[0]), float(mid_xy[1])), 0.0, yaw, []
            return self.ratio
        self.yaw_acc += _wrap(yaw - self.prev_yaw)
        self.prev_yaw = yaw
        self.cmd.append(float(k_cmd))
        s = math.hypot(mid_xy[0] - self.start[0], mid_xy[1] - self.start[1])
        if s >= self.seg_m:
            kc = float(np.mean(self.cmd)) if self.cmd else 0.0
            ka = self.yaw_acc / s
            if abs(kc) >= self.min_frac * self.kmax and kc * ka > 0:
                r = ka / kc
                self.ratio = r if self.ratio is None else (1 - self.alpha) * self.ratio + self.alpha * r
                self.n += 1
                self.last = dict(k_cmd=round(kc, 3), k_act=round(ka, 3), ratio=round(r, 3))
            elif abs(kc) >= self.min_frac * self.kmax:                 # 지령과 반대로 돎 = 비율 0 으로 셈
                self.ratio = 0.0 if self.ratio is None else (1 - self.alpha) * self.ratio
                self.n += 1
                self.last = dict(k_cmd=round(kc, 3), k_act=round(ka, 3), ratio=0.0)
            self.start, self.yaw_acc, self.cmd = (float(mid_xy[0]), float(mid_xy[1])), 0.0, []
        return self.ratio


def path_curvature_at(path_xyyaw, i, span=3):
    """경로 i 번째 점 둘레 곡률 [1/m] (방향 변화 / 호 길이). 점이 모자라면 0."""
    n = len(path_xyyaw)
    if n < 3:
        return 0.0
    a, b = max(0, i - span), min(n - 1, i + span)
    if b - a < 2:
        return 0.0
    s = sum(math.hypot(path_xyyaw[k + 1][0] - path_xyyaw[k][0], path_xyyaw[k + 1][1] - path_xyyaw[k][1]) for k in range(a, b))
    if s < 1e-6:
        return 0.0
    return _wrap(path_xyyaw[b][3] - path_xyyaw[a][3]) / s
