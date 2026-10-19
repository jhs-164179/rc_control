"""DARON 주행가능성 판정 핵심 v2.5 (numpy만 사용, ROS·OpenCV 불필요).

[v2.2 변경 (261004) - 새 입력을 주지 않으면 결과가 v2.1과 칸 단위로 같다]
    - 턱 격자 위치 보완 (인계 4-1): hm_fine(2.5 cm 칸 평균 높이)을 주면 턱 1단계를 2.5 cm 칸에서도 비교하고(반경 7.5 cm),
      2단계를 '후보마다 둘레 후보 수로 확정 -> 15 cm 번짐'(support, N_CRIT_FINE=6)으로 바꾼다.
      모서리가 5 cm 칸 안쪽에 있어 칸 평균이 중간값이 되던 문제(5 cm 턱 통과)를 줄임. 근거: step_eval/step_synth_results.md
    - 누적 지도용 낙차 규칙 (인계 4-2): inside(판정 창 안 칸)를 주면
        ① 내리막 너머 가림: 통과 칸의 평면이 미관측 쪽으로 10°보다 가파르게 내려가는 가장자리에서 30 cm 안 미관측 칸 = 낙차(NEG)
           (경로계획 planner_core.drop_shadow v2 와 같은 규칙을 판정으로 옮김)
        ② 가림 사이 높이 차: 미관측 칸 양쪽(40 cm 안)의 관측 칸 높이 차가 4 cm 와 20° 경사 몫을 둘 다 넘으면 사이 미관측 = 낙차(NEG)
      창 밖(inside=False)은 '안 봤음'이 아니라 '안 줌'이라 규칙에서 빼고, 창 경계 15 cm 안 칸(창에 잘려 판정 보류)도 뺀다.
    - crater(크레이터 영역 칸, 선우님 YOLO 표시를 노드가 칸으로 바꾼 것)를 주면 그 안의 통과 칸·낙차 칸 사유를 CRATER(9)로 (비용은 그대로)
    - span(칸 안 높이 폭)을 tall 대신 줄 수 있음 (tall = span > WALL_SPAN)

[v2.3 = 10/5 실측 반영 (10/5 녹화 5개, judge_eval/eval_1005.py) - 기본값은 여전히 v2.1 과 같은 결과]
    - hm_fine(2.5 cm 칸 비교)은 실제 지도에서 쓰지 말 것: 지도 점이 2 cm 로 솎여(칸당 1~2점) 바닥에 가짜 턱이 많음
      (step 녹화 바닥 판정 칸의 26%, v2.1 0.4%). 합성(1 cm 점)에서만 좋았음. 코드는 남겨 둠.
    - TREND_MODE='median' (선택): 턱에서 뺄 기울기를 '반경 R_TREND 안 칸별 기울기의 중앙값'으로. 작은 물체가 평면을 끌어당기지 않음.
      실측 권장 R_TREND=4 (20 cm), MIN_TREND=14: 메모 4·5·6 cm 물체 검출 50/48/68% (v2.1 5/4/45%), 바닥 가짜 턱 0.39% (v2.1 0.41%).
      (지도에서 잰 물체 높이가 메모와 1~2 cm 어긋나 4·5 cm 구분은 못 함. judge_eval/eval_1005_results.md)
    - cam_pose(지금 카메라 x, y, yaw, 격자 좌표계)를 주면 낙차 '가장자리' 규칙을 카메라 시야 부채꼴 안, 카메라에서 봐서 가장자리
      너머로만 제한하고, '높이 차' 규칙은 카메라 쪽이 높을 때만 (실측: 제한 없이는 관측 영역 옆·뒤 가장자리에 가짜 낙차가 많음).
    - up_correct(): 판정 노드가 IMU 로 구한 '위'(base_link 프레임)에 고정 보정각을 더하는 도우미. 10/5 평지 장면 14개 추정값
      UP_CORR_PITCH = +0.71°, UP_CORR_ROLL = -3.27° (노드에서 켤지 결정 전, 함수만 있음)

[v2.4 = 낙차 의심 (261005, 케일럽 결정 B) - DROP_SUSPECT=True 이고 inside·cam_pose 를 줄 때만, 기본은 꺼짐]
    - 카메라 시야 안에서 바닥 관측이 기대 거리(SUS_S_EXP)보다 일찍 끊기고 그 뒤로 관측이 없으면, 끊긴 곳 너머 SUS_BAND 안 미관측 칸을
      '낙차 의심'(사유 NEG_SUS = 10, 비용 SUS_COST = 90)으로. 불가(100)가 아니라 높은 비용: 다른 길이 있으면 피하고, 없으면
      천천히 다가가 다음 격자로 다시 본다(경로계획 몫). 낙떠러지는 끝까지 안 보여 의심이 안 풀리고, 능선은 넘어가며 풀림.
      근거: judge_eval/frontier_1005.py (10/5 녹화: 단 끝을 카메라 0.74 m 부터 잡음, 대신 출발 직후·검은 물체 앞·능선 꼭대기도 칠함)

[v2.5 = 낙차 의심을 원본 depth 로 확인 (261006, 케일럽 제안 C안) - DEPTH_CONFIRM=True 이고 depth_pts 를 줄 때만, 기본은 꺼짐]
    - 의심 광선마다 끊긴 곳 너머의 지금 depth 점(격자 좌표)을 봄: 같은 높이 바닥이 이어지면 의심 해제(지도에 아직 안 쌓였을 뿐),
      끊긴 곳보다 DC_DEEP(10 cm) 넘게 낮은 면이 이웃 광선 DC_RUN 개 이상에서 보이면 낙차(NEG, 100), 그 밖(안 보임·더 높은 면)은 의심 유지.
    - 근거: judge_eval/depth_confirm_1005.py (10/5 녹화: 단 끝을 카메라 0.74 m 부터 낙차로, 다른 녹화 깊은 낙차 0격자, 가짜 의심 18~71% 해제)
    - 한계 [계산]: 카메라가 바닥 위 약 11 cm 라, 끝에서 0.3 m 넘게 떨어져 보면 숨은 부분의 평균 기울기가 시선 각도(20° 미만)와 같아져
      '낭떠러지'와 '숨은 완만한 내리막'을 기하로는 못 가림. 그래서 깊이(DC_DEEP)로 정하는 정책이고, 너머 바닥이 보이는 깊은 능선·크레이터도 막힘.

[사용법]
    import judge_core as JC
    r = JC.judge(h, span=zmax - zmin, hm_fine=h_fine, inside=inside, origin=(ox, oy))
    grid.data = JC.to_occupancy_data(r['cost'])        # nav_msgs/OccupancyGrid.data (-1~100)
    why.data  = JC.to_occupancy_data(r['why'])         # 같은 배치로 사유 코드 (0~9)

[입력 약속]
    h      : 2D 배열 [x칸, y칸] (첫 번째 축 = x, 두 번째 축 = y). 칸 크기 5 cm (RES). 값 = 중력 기준 높이 [m]. 미관측 칸은 NaN.
             배열이 [y, x] 순서면 h.T 를 넣을 것.
    tall   : (선택) 같은 모양 bool. 칸 안 점 높이 폭 > 15 cm 인 칸 = 벽·구조물. 없고 span 이 있으면 span 으로 만듦.
    span   : (선택) 칸 안 점 높이 폭 zmax - zmin [m].
    hm_fine: (선택) 같은 점을 2.5 cm 칸으로 묶은 평균 높이 [2NX, 2NY], 미관측 NaN. 칸 [2i:2i+2, 2j:2j+2] = 5 cm 칸 [i, j].
    inside : (선택) 같은 모양 bool. 판정 창(로버 반경) 안 칸. 주면 낙차 규칙을 켬. 격자는 창 밖으로 30 cm 넘게 여유가 있어야 함.
    crater : (선택) 같은 모양 bool. 크레이터 영역 칸.
    cam_pose: (선택, 261005) (x, y, yaw) 지금 카메라 위치·방향, 격자 좌표계 [m, rad]. 주면 낙차 규칙을 '카메라 시야 부채꼴 안,
             카메라에서 봐서 가장자리 너머' 미관측 칸으로만 제한 (실측: 이 제한 없이는 관측 영역 옆·뒤 가장자리에 가짜 낙차가 많음)
    depth_pts: (선택, 261006) (N, 3) 지금 depth 한 장을 격자 좌표로 옮긴 점 (depth_to_points 로 만듦). DEPTH_CONFIRM 이면 낙차 의심 확인에 씀.
    cam_xy, occ_fn : 한 장면용 옛 낙차 의심 규칙(실험). 누적 지도에서는 None.

[출력] dict
    cost  : [x칸, y칸] int. 0~99 = 통과 가능(클수록 어려움), 100 = 불가, -1 = 미관측/판정 보류
    why   : 사유 코드 UNK 0, PASS 1, SLOPE 2(경사), STEP 3(턱), BOTH 4, NEG 5(낙차), SPARSE 6(보류), WALL 7(벽),
            NEARWALL 8(벽 옆), CRATER 9(크레이터 안: 비용은 판정대로), NEG_SUS 10(낙차 의심: 비용 SUS_COST, v2.4)
            (v2.5 depth 확인으로 깊은 낙차가 보이면 NEG 5)
    sl, rough, step : 칸별 경사[deg], 거칠기[m], 턱[m] (디버그·시각화용)

[규칙]
    경사   : 칸 주변 원(반경 15 cm, 관측 칸 60% 이상)에 평면 적합 -> 기울기. 20° 넘으면 불가
    거칠기 : 그 평면 대비 높이 표준편차. 불가 판정 없이 비용에만 반영
    턱     : 주변 평면(반경 25 cm)의 기울기 몫을 뺀 뒤, 10 cm 안 이웃과 4 cm 넘게 차이 나면 임시 턱 (hm_fine 이 있으면 2.5 cm 칸 7.5 cm 안도)
             -> v2.1: 반경 15 cm 안 임시 턱 최대값 x (임시 턱 칸 수 / 8)
             -> hm_fine 있을 때: 임시 턱 칸마다 (자기 + 둘레 15 cm 임시 턱 칸 수 / 6) 로 확정한 뒤 15 cm 안 최대. 4 cm 넘으면 불가
    벽     : tall 인 칸은 불가. 벽 칸은 이웃 계산에서 빼고, 벽에서 15 cm 이내 칸을 불가(NEARWALL)로
    판정 보류: 경사 평면 원(29칸) 중 관측 칸이 60% 미만이면 보류
    낙차   : 위 [v2.2 변경] 참고 (inside 를 줄 때만)
    위험도 : d = 0.5*경사/20° + 0.25*min(거칠기/3cm, 1) + 0.25*턱/4cm,  비용 = round(99*d)
    기준값(20°, 4 cm 등)은 작업 가정. 창 반경이 칸 수로 정해져 있어 칸 크기는 5 cm 전제.

[성능] 6 x 6 m (120 x 120칸) 약 50 ms (v2.1). hm_fine·inside 를 주면 약 2배 [추정, 합성 격자]
"""
import numpy as np

VERSION = '2.5'
# ---- 격자 (trav_ins와 같음) ----
RES = 0.05; XMAX = 2.0; YH = 1.0; NX = int(XMAX / RES); NY = int(2 * YH / RES); MINPTS = 3
# ---- 임계: 작업 가정. 실제 등판/돌 넘기 실험으로 다시 정할 값 ----
S_CRIT = 20.0            # 경사 [deg] - 치명
H_CRIT = 0.04            # 턱 [m]     - 치명 (지상고 44.9 mm 바로 아래). v2.2 정확도에서는 4.25 cm 후보 (4-1, 실측 확인 전)
R_CRIT = 0.03            # 거칠기 [m] - 비용 정규화용 (CH09 예시값)
W_S, W_R, W_H = 0.5, 0.25, 0.25   # CH09 그림 예시 가중
# ---- 창 크기 [셀] ----
R_FIT = 3; MIN_FIT = 10  # 경사/거칠기: 반경 15 cm 원, 관측 셀 10개 이상일 때만 판정
R_ST1 = 2                # 턱 1단계: 반경 10 cm
R_ST2 = 3                # 턱 2단계: 반경 15 cm
N_CRIT = 8               # CH09 예시 n_crit 50셀 @2 cm(=0.02 m^2)를 면적 기준으로 5 cm 셀에 환산
STEP_MODE = 'plane'      # 'ch09' = 원문대로 생 높이차 / 'plane' = 주변 평면 기울기를 빼고 남은 높이차
R_TREND = 5; MIN_TREND = 20   # 'plane'에서 뺄 기울기를 구하는 원 반경 25 cm
TREND_MODE = 'plane'     # 턱에서 뺄 기울기: 'plane' = v2.1 (반경 R_TREND 최소제곱 평면) / 'median' = 칸별 기울기의 중앙값 (10/5 실측 후보,
                         #   그때 R_TREND=4, MIN_TREND=14 권장)
UP_CORR_PITCH = 0.71     # up_correct() 기본 보정각 [deg] (10/5 녹화 평지 14장면 중앙값, 바닥 수평 가정. 전장부가 바뀌면 다시 잼)
UP_CORR_ROLL = -3.27
# ---- v2.2 턱 보완 (hm_fine 을 줄 때만) ----
R_ST1F = 3               # 2.5 cm 칸 기준 1단계 이웃 반경 (3칸 = 7.5 cm). 2칸(5 cm)이면 면 점이 찍힌 모서리에서 높이차가 두 칸에 나뉨
N_CRIT_FINE = 6          # 2단계 확정에 필요한 둘레 15 cm 임시 턱 칸 수 (곧은 모서리 = 한 줄 약 5~7칸)
# ---- v2.2 낙차 (inside 를 줄 때만) ----
DROP_DEG = 10.0          # 가장자리 평면이 미관측 쪽으로 이보다 가파르게 내려가면 (planner_core drop_deg 와 같음)
DROP_SHADOW = 0.30       # 그 가장자리에서 이 거리 안 미관측 칸을 낙차로 (planner_core drop_shadow_m)
DROP_MIN_N = 8           # 가장자리 평면 맞춤 최소 칸 수 (planner_core local_grad min_n)
GAP_LOOK = 0.40          # 가림 사이 높이 차: 미관측 칸에서 양쪽으로 관측 칸을 찾는 거리
VIEW_HALF = 45.0         # cam_pose 를 줄 때: 낙차를 볼 시야 부채꼴 반각 [deg] (D405 수평 시야 87° 의 반)
VIEW_MIN = 0.20          #   카메라에서 이보다 가까운 칸은 뺌 [m]
BEHIND_DEG = 20.0        #   가장자리 -> 미관측 칸 방향이 카메라 -> 가장자리 방향과 이 각도 안이어야 '너머'
GAP_WEDGE = False        #   가림 사이 높이 차 규칙에도 부채꼴 제한을 걸지 (False = 창 안 전부, 단 카메라 쪽이 높을 때만)
# ---- 낙차 의심(옛 한 장면용 실험 기능) ----
NEG_BACK = 0.15; NEG_LOOK = (0.10, 0.20); NEG_ANG = 10.0
# ---- v2.4 낙차 의심 (DROP_SUSPECT 이고 inside·cam_pose 를 줄 때만) ----
DROP_SUSPECT = False     # 켜면 '바닥 관측이 기대보다 일찍 끊긴 곳 너머'를 낙차 의심으로 (실시간 노드 파라미터로 켬)
SUS_COST = 90            # 의심 칸 비용. 경로계획의 미관측 비용(74)보다 높아야 '모르는 칸'보다 피함, 100 미만이라 다른 길이 없으면 감
SUS_S_EXP = 0.85         # 평지에서 지도가 보통 이어지는 거리 [m, 카메라에서 수평]. 이보다 가까이서 끊기면 의심 (지도 적분 1.0 m 한계)
SUS_GAP = 0.15           # 이보다 짧은 미관측 구멍은 무시 [m]
SUS_BAND = 0.30          # 끊긴 곳 너머 이 거리 안 미관측 칸을 의심으로 [m]
SUS_POS_H = 0.04         # 끊긴 칸이 광선 앞쪽 최저보다 이만큼 넘게 높으면 '물체 위' -> 제외 (양의 장애물 뒤, B13) [m]
SUS_MIN_RAYS = 3         # 이웃 광선 이 개수 이상(2° 간격 -> 6°) 같이 끊길 때만 (B13 의 5° 각도 필터)
SUS_FOV = 38.0           # 광선 반각 [deg] (D405 수평 87° 에서 가장자리 여유)
# ---- v2.5 낙차 의심 depth 확인 (DEPTH_CONFIRM 이고 depth_pts 를 줄 때만) ----
DEPTH_CONFIRM = False    # 켜면 의심 광선을 지금 depth 점으로 확인 (해제 / 낙차 / 유지)
DC_DMAX = 2.2            # 너머를 찾는 최대 거리 [m, 카메라에서 수평]. 단 끝 0.74 m 에서 아래 바닥이 1.3~2.0 m 에 찍힘 (10/5)
DC_BEAR = 1.5            # 광선 방향 ± [deg]
DC_TOL = 0.04            # |끊긴 곳 - 너머 첫 면| 이 이보다 작으면 같은 바닥이 이어짐 -> 의심 해제 [m]
DC_DEEP = 0.10           # 너머 첫 면이 이보다 더 낮으면 깊은 낙차 [m] (10/5: 단 끝 12~16 cm, 다른 녹화 4~9 cm)
DC_RUN = 3               # 깊은 낙차 광선이 이웃으로 이 개수 이상일 때만 낙차
# 사유 코드
UNK, PASS, SLOPE, STEP, BOTH, NEG, SPARSE, WALL, NEARWALL, CRATER, NEG_SUS = 0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10
WALL_INFLATE = 3          # 15 cm - 턱 2단계(R_ST2)와 같은 로버 크기 여유
MIN_FIT_FRAC = 0.6        # 경사 평면 원(29칸) 중 관측 비율이 이보다 낮으면 판정 보류
WALL_SPAN = 0.15          # 한 셀(5 cm) 안 점들의 높이 폭이 15 cm 넘으면 수직 구조물(벽·착륙선 등)로 표시


def offs(r, center=True):
    return [(dx, dy) for dx in range(-r, r + 1) for dy in range(-r, r + 1)
            if dx * dx + dy * dy <= r * r and (center or dx or dy)]


def shift(a, dx, dy, fill):
    """out[i,j] = a[i+dx, j+dy] (범위 밖은 fill)"""
    o = np.full(a.shape, fill, dtype=a.dtype); n0, n1 = a.shape
    if abs(dx) >= n0 or abs(dy) >= n1:     # [261015] 옮기는 양 >= 격자 폭이면 전부 범위 밖 (전에는 음수 슬라이스 끝 → ValueError, 우분투 가상 로버 10/15)
        return o
    o[max(0, -dx):min(n0, n0 - dx), max(0, -dy):min(n1, n1 - dy)] = a[max(0, dx):min(n0, n0 + dx), max(0, dy):min(n1, n1 + dy)]
    return o


def fit_plane(hm, r, res=RES):
    """각 셀 중심 기준 원(반경 r 셀) 안 관측 셀로 h = a*dx + b*dy + c 최소제곱. 반환: coef(NX,NY,3) [a,b: 기울기(m/m)], 셀 수"""
    S = {k: np.zeros(hm.shape) for k in ('n', 'x', 'y', 'xx', 'xy', 'yy', 'h', 'xh', 'yh')}
    for dx, dy in offs(r):
        hk = shift(hm, dx, dy, np.nan); m = ~np.isnan(hk); hk = np.nan_to_num(hk); X, Y = dx * res, dy * res
        S['n'] += m; S['x'] += m * X; S['y'] += m * Y; S['xx'] += m * X * X; S['xy'] += m * X * Y; S['yy'] += m * Y * Y
        S['h'] += hk; S['xh'] += hk * X; S['yh'] += hk * Y
    ok = S['n'] >= 4
    A = np.stack([np.stack([S['xx'], S['xy'], S['x']], -1), np.stack([S['xy'], S['yy'], S['y']], -1),
                  np.stack([S['x'], S['y'], S['n']], -1)], -2)[ok] + np.eye(3) * 1e-9
    coef = np.full(hm.shape + (3,), np.nan)
    coef[ok] = np.linalg.solve(A, np.stack([S['xh'], S['yh'], S['h']], -1)[ok][..., None])[..., 0]
    return coef, S['n']


def median_grad(hm, r, min_n):
    """칸마다 중앙차분 기울기 (dh/dx, dh/dy) [m/m] 를 구하고 반경 r 칸 원 안의 중앙값 (TREND_MODE='median').
    턱 모서리에서는 바로 옆 칸 몇 개의 기울기만 튀므로, 중앙값은 주변 지면의 실제 기울기를 따른다. 유효 기울기가 min_n 개 미만이면 NaN."""
    import warnings
    gx = (shift(hm, 1, 0, np.nan) - shift(hm, -1, 0, np.nan)) / (2 * RES)
    gy = (shift(hm, 0, 1, np.nan) - shift(hm, 0, -1, np.nan)) / (2 * RES)
    o = offs(r)
    out = []
    for g in (gx, gy):
        st = np.stack([shift(g, dx, dy, np.nan) for dx, dy in o], -1)
        cnt = np.sum(~np.isnan(st), -1)
        with warnings.catch_warnings():
            warnings.simplefilter('ignore', RuntimeWarning)
            med = np.nanmedian(st, -1)
        out.append(np.where(cnt >= min_n, med, np.nan))
    return out[0], out[1]


def up_correct(up_b, dpitch_deg=None, droll_deg=None):
    """base_link 프레임 '위' 벡터에 고정 보정각을 더함 (10/5). 피치 = asin(u_x) (+ = 앞이 높음), 롤 = atan2(u_y, u_z) (+ = 왼쪽이 높음)
    으로 바꾼 뒤 각각 더하고 다시 벡터로. 보정각은 평지에 정지한 녹화에서 '카메라로 본 바닥 법선 - IMU 위' 의 중앙값 (eval_1005.py tilt)."""
    dp = UP_CORR_PITCH if dpitch_deg is None else dpitch_deg
    dr = UP_CORR_ROLL if droll_deg is None else droll_deg
    u = np.asarray(up_b, float); u = u / np.linalg.norm(u)
    p = np.arcsin(np.clip(u[0], -1, 1)) + np.radians(dp)
    r = np.arctan2(u[1], u[2]) + np.radians(dr)
    return np.array([np.sin(p), np.cos(p) * np.sin(r), np.cos(p) * np.cos(r)])


def _step_candidates(h, ga, gb, r, res):
    """턱 1단계: 반경 r 칸 안 이웃과 '기울기 몫을 뺀 높이차'가 H_CRIT·S_CRIT 를 넘으면 그 높이차(최대)를 후보로"""
    tmp = np.zeros(h.shape)
    for dx, dy in offs(r, center=False):
        dh = np.abs(shift(h, dx, dy, np.nan) - h - (ga * dx + gb * dy) * res); dist = res * np.hypot(dx, dy)
        with np.errstate(invalid='ignore'):
            cand = (dh > H_CRIT) & (np.degrees(np.arctan(dh / dist)) > S_CRIT)
        tmp = np.where(cand, np.maximum(tmp, np.nan_to_num(dh)), tmp)
    return tmp


def _nearest_along(h, inside, dx, dy, K):
    """칸마다 (dx, dy) 방향으로 1~K 칸 안 첫 관측 칸의 높이와 칸 수. 창 밖(inside=False)을 만나면 멈춤."""
    fh = np.full(h.shape, np.nan); fk = np.full(h.shape, np.nan); stop = np.zeros(h.shape, bool)
    for k in range(1, K + 1):
        hv = shift(h, k * dx, k * dy, np.nan); iv = shift(inside, k * dx, k * dy, False)
        stop |= ~iv
        take = np.isnan(fh) & ~stop & ~np.isnan(hv)
        fh[take] = hv[take]; fk[take] = k
    return fh, fk


def drop_rules(hm, cost, why, inside, cam_pose=None, origin=(0.0, 0.0)):
    """누적 지도용 낙차 규칙 (v2.2). 반환 (내리막 너머 가림 bool, 가림 사이 높이 차 bool) - 둘 다 창 안 미관측 칸만."""
    core = inside.copy()                              # 창 경계에서 15 cm(R_FIT) 안쪽: 창에 잘려 판정 보류가 된 칸을 '안 봄'으로 세지 않게
    for dx, dy in offs(R_FIT):
        core &= shift(inside, dx, dy, False)
    unknown = (cost < 0) & core                       # 미관측·판정 보류 (창 안쪽)
    unknown_gap = unknown
    if cam_pose is not None:                          # 261005: 지금 카메라 시야 부채꼴 안 미관측만
        cx, cy, cyaw = cam_pose; ox, oy = origin
        I, J = np.meshgrid(np.arange(hm.shape[0]), np.arange(hm.shape[1]), indexing='ij')
        PX = ox + (I + .5) * RES; PY = oy + (J + .5) * RES
        VX, VY = PX - cx, PY - cy; DIST = np.hypot(VX, VY)
        bear = np.degrees(np.abs(np.angle(np.exp(1j * (np.arctan2(VY, VX) - cyaw)))))
        unknown = unknown & (bear <= VIEW_HALF) & (DIST >= VIEW_MIN)
        if GAP_WEDGE:
            unknown_gap = unknown
    ok = (cost >= 0) & (cost < 100)                   # 통과 칸만 평면·가장자리에 씀 (벽·상자 뒷면을 크레이터 가장자리로 오인하지 않게)
    coef, n = fit_plane(np.where(ok, hm, np.nan), R_FIT)
    a, b = coef[..., 0], coef[..., 1]; good = ok & (n >= DROP_MIN_N)
    thr = np.tan(np.radians(DROP_DEG))
    edge = np.zeros(hm.shape, bool)
    for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (1, -1), (-1, 1), (-1, -1)):
        nxt = shift(unknown, dx, dy, False)
        with np.errstate(invalid='ignore'):
            edge |= good & nxt & ((a * dx + b * dy) / np.hypot(dx, dy) < -thr)
    shadow = np.zeros(hm.shape, bool)
    if edge.any():
        k = int(np.ceil(DROP_SHADOW / RES - 1e-9))
        cb = np.cos(np.radians(BEHIND_DEG))
        for dx, dy in offs(k):
            src = shift(edge, dx, dy, False)          # 이 칸에서 (dx, dy) 떨어진 곳이 가장자리
            if cam_pose is not None and (dx or dy):
                # 가장자리 e = 칸 + (dx, dy). 미관측 칸이 카메라에서 봐서 e 너머: (칸 - e) 가 (e - 카메라) 와 같은 방향
                EX, EY = PX + dx * RES - cx, PY + dy * RES - cy
                dot = (-dx * EX - dy * EY) / (np.hypot(dx, dy) * np.maximum(np.hypot(EX, EY), 1e-9))
                src &= dot >= cb
            elif cam_pose is not None:
                src &= False
            shadow |= src
        shadow &= unknown
    # 가림 사이 높이 차: 양쪽 관측 칸(벽 제외, 판정된 칸)의 높이 차가 턱 기준과 경사 기준을 둘 다 넘음
    hk = np.where((cost >= 0) & (why != WALL), hm, np.nan)
    K = int(np.ceil(GAP_LOOK / RES - 1e-9)); tS = np.tan(np.radians(S_CRIT))
    gap = np.zeros(hm.shape, bool)
    for dx, dy in ((1, 0), (0, 1), (1, 1), (1, -1)):
        f, fk = _nearest_along(hk, inside, dx, dy, K); g, gk = _nearest_along(hk, inside, -dx, -dy, K)
        with np.errstate(invalid='ignore'):
            L = (fk + gk) * RES * np.hypot(dx, dy)
            if cam_pose is None:
                dh = np.abs(f - g)
            else:                                     # 261005: 카메라 쪽이 높고 먼 쪽이 낮을 때만 (내려다본 낭떠러지)
                df = np.hypot(PX + fk * dx * RES - cx, PY + fk * dy * RES - cy)
                dg = np.hypot(PX - gk * dx * RES - cx, PY - gk * dy * RES - cy)
                dh = np.where(df < dg, f - g, g - f)
            gap |= unknown_gap & (dh > H_CRIT) & (dh > L * tS)
    return shadow, gap


def depth_to_points(depth_m, K, T_grid_cam, step=2, dmin=0.1, dmax=2.5):
    """v2.5 도우미: depth 영상 [m] (0 = 값 없음) -> 격자 좌표 3D 점 (N, 3).
    K: 3x3, T_grid_cam: 4x4 (p_grid = T · p_cam, 광학 프레임 x 오른쪽·y 아래·z 앞). 노드에서 odom->격자 회전과 odom<-카메라를 곱해 넘김."""
    d = np.asarray(depth_m, float)[::step, ::step]
    v, u = np.mgrid[0:depth_m.shape[0]:step, 0:depth_m.shape[1]:step]
    m = (d > dmin) & (d < dmax)
    pc = np.stack([(u[m] - K[0][2]) / K[0][0] * d[m], (v[m] - K[1][2]) / K[1][1] * d[m], d[m]], -1)
    T = np.asarray(T_grid_cam, float)
    return pc @ T[:3, :3].T + T[:3, 3]


def _depth_check(hit_b, sf, hf_map, pts, cam_pose):
    """한 광선의 끊긴 곳 너머를 depth 점으로: ('cont'|'deep'|'keep', D)"""
    cx, cy, cyaw = cam_pose
    dx, dy = pts[:, 0] - cx, pts[:, 1] - cy
    s = np.hypot(dx, dy); br = np.angle(np.exp(1j * (np.arctan2(dy, dx) - cyaw - hit_b)))
    on = np.abs(br) < np.radians(DC_BEAR)
    near = on & (s > sf - 0.10) & (s <= sf)
    hf = float(np.median(pts[near, 2])) if near.sum() >= 5 else float(hf_map)   # 같은 depth 로 재서 지도-depth 어긋남을 피함
    far = on & (s > sf + 0.05) & (s <= DC_DMAX)
    if far.sum() < 5:
        return 'keep', None
    first = far & (s <= s[far].min() + 0.10)
    D = hf - float(np.median(pts[first, 2]))
    if abs(D) < DC_TOL:
        return 'cont', D
    return ('deep' if D >= DC_DEEP else 'keep'), D


def drop_suspect(hm, inside, cam_pose, origin=(0.0, 0.0), depth_pts=None):
    """v2.4 낙차 의심: 카메라에서 수평 광선(2° 간격, ±SUS_FOV)을 따라 관측 칸이 SUS_S_EXP 전에 끊기고 그 뒤 SUS_S_EXP 까지 관측이 없으면
    끊긴 곳 너머 SUS_BAND 안 미관측·창 안 칸 (bool, hm 과 같은 [x, y]). 끊긴 칸이 물체 위면 제외, 이웃 광선 SUS_MIN_RAYS 개 이상만.
    v2.5: depth_pts(격자 좌표 점)를 주면 광선마다 확인 -> (의심 칸, 낙차 칸) 둘 다 반환. 안 주면 (의심 칸, 빈 칸)."""
    cx, cy, cyaw = cam_pose; ox, oy = origin
    nx, ny = hm.shape
    obs_all = np.isfinite(hm)
    sus = np.zeros(hm.shape, bool)
    ss = np.arange(0.15, 1.25, 0.025)
    hits = []
    for b in np.radians(np.arange(-SUS_FOV, SUS_FOV + 0.1, 2.0)):
        a = cyaw + b
        i = np.floor((cx + ss * np.cos(a) - ox) / RES).astype(int)
        j = np.floor((cy + ss * np.sin(a) - oy) / RES).astype(int)
        ok = (i >= 0) & (j >= 0) & (i < nx) & (j < ny)
        h = np.full(len(ss), np.nan); h[ok] = hm[i[ok], j[ok]]
        inn = np.zeros(len(ss), bool); inn[ok] = inside[i[ok], j[ok]]
        obs = np.isfinite(h)
        hit = None
        if obs[ss < 0.5].any():
            k0 = int(np.argmax(obs))
            for k in range(k0, len(ss)):
                if ss[k] >= SUS_S_EXP:
                    break
                if obs[k]:
                    continue
                rest = obs[k:] & (ss[k:] < SUS_S_EXP)
                if rest.any():
                    if ss[k + int(np.argmax(rest))] - ss[k] < SUS_GAP:
                        continue                              # 짧은 구멍
                    break                                     # 다시 관측됨 -> 높이 차 규칙(drop_rules) 몫
                if h[k - 1] - np.nanmin(h[k0:k]) > SUS_POS_H:
                    break                                     # 물체 위에서 끊김 (양의 장애물 뒤)
                sf = ss[k - 1]
                band = (ss > sf) & (ss <= sf + SUS_BAND) & ok & inn & ~obs
                hit = (i[band], j[band], b, sf, h[k - 1])
                break
        hits.append(hit)
    flag = [x is not None for x in hits]
    n = len(hits)
    valid = [False] * n
    for m in range(n - SUS_MIN_RAYS + 1):
        if all(flag[m:m + SUS_MIN_RAYS]):
            for q in range(m, m + SUS_MIN_RAYS):
                valid[q] = True
    negm = np.zeros(hm.shape, bool)
    if depth_pts is None or len(depth_pts) == 0:
        for x, v in zip(hits, valid):
            if v:
                sus[x[0], x[1]] = True
        return sus & ~obs_all & inside, negm
    res = [(_depth_check(x[2], x[3], x[4], depth_pts, cam_pose)[0] if v else None) for x, v in zip(hits, valid)]
    deep = [False] * n
    for m in range(n - DC_RUN + 1):
        if all(r == 'deep' for r in res[m:m + DC_RUN]):
            for q in range(m, m + DC_RUN):
                deep[q] = True
    for x, v, r, dq in zip(hits, valid, res, deep):
        if not v or r == 'cont':
            continue
        if dq:
            negm[x[0], x[1]] = True
        else:
            sus[x[0], x[1]] = True
    return sus & ~obs_all & inside, negm & ~obs_all & inside


def judge(hm, cam_xy=None, occ_fn=None, tall=None, origin=(0.0, -YH), span=None, hm_fine=None, inside=None, crater=None,
          cam_pose=None, depth_pts=None):
    """hm: (NX,NY) 중력 기준 셀 높이[m], nan=미관측. origin: 칸 [0,0]의 모서리 좌표 (x, y) [m].
    나머지 입력은 모듈 설명 [입력 약속] 참고. 격자 크기는 hm.shape를 따름. 칸 크기는 RES(5 cm) 가정"""
    hm_in = hm; NXg, NYg = hm.shape; ox, oy = origin
    if tall is None:
        tall = np.zeros(hm.shape, bool) if span is None else np.nan_to_num(span) > WALL_SPAN
    wall = tall & ~np.isnan(hm)
    hm = np.where(wall, np.nan, hm)          # 벽 칸은 평면·거칠기·턱 계산에서 제외 (v2.1)
    kn = ~np.isnan(hm)
    # --- 경사/거칠기: 원(반경 R_FIT) 안 평면 적합 ---
    coef, n = fit_plane(hm, R_FIT)
    ev = kn & (n >= max(MIN_FIT, MIN_FIT_FRAC * len(offs(R_FIT)))); coef[~ev] = np.nan
    sl = np.degrees(np.arctan(np.hypot(coef[..., 0], coef[..., 1])))
    ss = np.zeros(hm.shape)
    for dx, dy in offs(R_FIT):
        hk = shift(hm, dx, dy, np.nan); m = ~np.isnan(hk)
        res = hk - (coef[..., 0] * dx * RES + coef[..., 1] * dy * RES + coef[..., 2]); ss += np.where(m & ev, res * res, 0)
    rough = np.where(ev, np.sqrt(ss / np.maximum(n, 1)), np.nan)
    # 턱에서 뺄 기울기(STEP_MODE='plane'일 때): 더 넓은 원(반경 R_TREND)의 평면. 없으면 R_FIT 평면
    if STEP_MODE == 'plane' and TREND_MODE == 'median':
        ga, gb = median_grad(hm, R_TREND, MIN_TREND)
        ga = np.where(np.isnan(ga), np.nan_to_num(coef[..., 0]), ga); gb = np.where(np.isnan(gb), np.nan_to_num(coef[..., 1]), gb)
    elif STEP_MODE == 'plane':
        ct, nt = fit_plane(hm, R_TREND); okt = nt >= MIN_TREND
        ga = np.where(okt, ct[..., 0], np.nan_to_num(coef[..., 0])); gb = np.where(okt, ct[..., 1], np.nan_to_num(coef[..., 1]))
    else:
        ga = gb = np.zeros(hm.shape)
    # --- 턱 1단계: 가까운 이웃과의 급한 높이차 ---
    tmp = _step_candidates(hm, ga, gb, R_ST1, RES)
    fine = hm_fine is not None
    if fine:                                 # v2.2: 2.5 cm 칸에서 7.5 cm 안 비교 -> 5 cm 칸 4개 중 최대
        rep = lambda a: np.repeat(np.repeat(a, 2, 0), 2, 1)
        hf = np.where(rep(wall), np.nan, np.asarray(hm_fine, float))
        tf = _step_candidates(hf, rep(ga), rep(gb), R_ST1F, RES / 2)
        tmp = np.where(kn, np.maximum(tmp, tf.reshape(NXg, 2, NYg, 2).max(axis=(1, 3))), 0.0)
    # --- 턱 2단계 ---
    hmax = np.zeros(hm.shape); nst = np.zeros(hm.shape)
    for dx, dy in offs(R_ST2):
        tk = shift(tmp, dx, dy, 0.0); hmax = np.maximum(hmax, tk); nst += tk > 0
    if fine:                                 # v2.2 support: 임시 턱 칸마다 둘레 후보 수로 확정한 뒤 15 cm 로 번짐 (곧은 턱 띠가 얇아지지 않게)
        conf = np.where(tmp > 0, tmp * np.minimum(1.0, nst / N_CRIT_FINE), 0.0)
        smax = np.zeros(hm.shape)
        for dx, dy in offs(R_ST2):
            smax = np.maximum(smax, shift(conf, dx, dy, 0.0))
        step = np.where(ev, smax, np.nan)
    else:                                    # v2.1: 받는 칸 둘레 최대값 x 후보 수 / N_CRIT
        step = np.where(ev, np.minimum(hmax, hmax * nst / N_CRIT), np.nan)
    # --- 위험도/비용/사유 ---
    with np.errstate(invalid='ignore'):
        ls, lh = ev & (sl > S_CRIT), ev & (step > H_CRIT)
        d = W_S * sl / S_CRIT + W_R * np.minimum(rough / R_CRIT, 1) + W_H * step / H_CRIT
    cost = np.full(hm.shape, -1, int); cost[ev] = np.clip(np.round(99 * d[ev]), 0, 99).astype(int); cost[ls | lh] = 100
    why = np.full(hm.shape, UNK, int); why[kn & ~ev] = SPARSE; why[ev] = PASS
    why[ls & ~lh] = SLOPE; why[lh & ~ls] = STEP; why[ls & lh] = BOTH
    why[wall] = WALL; cost[wall] = 100
    near = np.zeros(hm.shape, bool)
    for dx, dy in offs(WALL_INFLATE):
        near |= shift(wall, dx, dy, False)
    near &= kn & ~np.isin(why, [SLOPE, STEP, BOTH])
    why[near] = NEARWALL; cost[near] = 100
    # --- v2.2 낙차 (누적 지도, inside 를 줄 때만) ---
    neg = np.zeros(hm.shape, bool)
    if inside is not None:
        sh, gp = drop_rules(hm_in, cost, why, np.asarray(inside, bool), cam_pose=cam_pose, origin=origin)
        neg = sh | gp
        why[neg] = NEG; cost[neg] = 100
        # --- v2.4 낙차 의심 ---
        if DROP_SUSPECT and cam_pose is not None:
            sus, dneg = drop_suspect(hm_in, np.asarray(inside, bool), cam_pose, origin,
                                     depth_pts if DEPTH_CONFIRM else None)
            sus &= cost < 0; dneg &= cost < 0
            why[sus] = NEG_SUS; cost[sus] = SUS_COST
            why[dneg] = NEG; cost[dneg] = 100; neg |= dneg
    # --- 낙차 의심 (옛 한 장면용 실험) ---
    if cam_xy is not None:
        cx, cy = cam_xy
        for i, j in zip(*np.nonzero(np.isin(why, [UNK, SPARSE]))):   # 미관측·보류 칸만 (벽·벽 옆 칸은 덮어쓰지 않음)
            px, py = ox + (i + .5) * RES, oy + (j + .5) * RES; vx, vy = px - cx, py - cy; L = np.hypot(vx, vy)
            if L < 0.3: continue
            ux, uy = vx / L, vy / L
            cell = lambda s: (int(np.floor((px - s * ux - ox) / RES)), int(np.floor((py - s * uy - oy) / RES)))
            e = None
            for s in np.arange(RES, NEG_BACK + 1e-6, RES):
                a, b = cell(s)
                if 0 <= a < NXg and 0 <= b < NYg and ev[a, b]:
                    if not tall[a, b]: e, se = (a, b), s
                    break
            if e is None: continue
            for s in np.arange(se + NEG_LOOK[0], se + NEG_LOOK[1] + 1e-6, RES):
                a, b = cell(s)
                if 0 <= a < NXg and 0 <= b < NYg and ev[a, b] and not tall[a, b]:
                    dd = np.hypot((a - e[0]) * RES, (b - e[1]) * RES)
                    if hm[a, b] - hm[e] > np.tan(np.radians(NEG_ANG)) * dd and (occ_fn is None or occ_fn(px, py, hm[e])):
                        why[i, j] = NEG; cost[i, j] = 100
                    break
    # --- v2.2 크레이터 영역: 통과 칸·낙차 칸의 사유만 CRATER 로 (비용 그대로) ---
    if crater is not None:
        cm = np.asarray(crater, bool) & np.isin(why, [PASS, NEG])
        why[cm] = CRATER
    return dict(hm=hm_in, ev=ev, sl=sl, rough=rough, step=step, d=d, cost=cost, why=why, neg=neg)


def stats(J):
    """비율의 분모 = 지면 칸(판정된 칸 + 벽 옆 칸). 벽(WALL) 칸 자체는 뺌"""
    w = J['why']; c = J['cost']; g = np.isin(w, [PASS, SLOPE, STEP, BOTH, NEARWALL]) | ((w == CRATER) & (c >= 0)); ne = max(g.sum(), 1)
    p = c[np.isin(w, [PASS, CRATER]) & (c >= 0) & (c < 100)]
    return dict(eval_cells=int(g.sum()), wall_cells=int((w == WALL).sum()),
                lethal=np.isin(w, [SLOPE, STEP, BOTH, NEARWALL]).sum() / ne,
                slope_only=(w == SLOPE).sum() / ne, step_only=(w == STEP).sum() / ne, both=(w == BOTH).sum() / ne,
                near_wall=(w == NEARWALL).sum() / ne,
                neg_cells=int((w == NEG).sum()), sparse_cells=int((w == SPARSE).sum()), crater_cells=int((w == CRATER).sum()),
                pass_cost_med=float(np.median(p)) if len(p) else float('nan'))


def to_occupancy_data(a):
    """판정 결과 cost[ix, iy] (또는 why) -> OccupancyGrid.data (행 = y, 열 = x, x가 빨리 변함).
    배열을 그대로 flatten 하면 지도가 90도 돌아간/뒤집힌 채로 조용히 나가므로 반드시 전치해서 넣는다.
    OccupancyGrid.info: width = a.shape[0](x칸 수), height = a.shape[1](y칸 수),
    resolution = RES, origin = 칸 [0,0]의 모서리 좌표."""
    return np.asarray(a, dtype=np.int8).T.reshape(-1).tolist()
