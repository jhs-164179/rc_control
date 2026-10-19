#!/usr/bin/env python3
"""판정기 어댑터 — 외부 판정 시스템을 trav_core 의 판정기 계약에 맞춰 끼운다.

계약(입출력)은 trav_core.py 의 "2단계: 판정기" 주석에 있다. 요약하면:
    judge(hmap, p) -> dict(cost=(ny,nx) int, [slope, step, rough, why])
    cost: -1 미관측 / 0~99 통행가능 / 100 통행불가

── 배열 방향 주의 ─────────────────────────────────────────────────────
trav_core 는 (ny, nx) = [행=y, 열=x], judge_core 는 (NX, NY) = [x, y] 다.
그래서 들어갈 때 .T, 나올 때 .T 를 한다. 이걸 빼먹으면 지도가 90도 돌아간 채로
**에러 없이** 나간다 (judge_core.to_occupancy_data 주석도 같은 경고를 한다).
"""
import sys
from pathlib import Path

import numpy as np

# judge_core.py 는 ~/daron_ros/ 에 있다 (3dmap/ 아님)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

# 낙차(negative obstacle) 의심 검사. judge_core 안에서 미관측 셀마다 파이썬
# for 루프를 돌기 때문에 격자가 커지면 느려진다. 실시간에서 주기를 못 맞추면 False.
NEG_CHECK = True


def judge_judgecore(hmap, p):
    """judge_core.judge() 를 trav_core 계약에 맞춘 어댑터."""
    import judge_core as JC

    if abs(hmap["cell"] - JC.RES) > 1e-9:
        raise ValueError(
            f"judge_core 는 셀 크기가 {JC.RES} m 인 것을 전제로 창 반경을 셀 수로 정한다. "
            f"지금 cell_size={hmap['cell']} — config 의 cell_size 를 {JC.RES} 로 맞추거나 "
            f"judge_core.RES 를 바꿀 것.")

    hm = np.ascontiguousarray(hmap["height"].T, dtype=float)      # (ny,nx) -> (nx,ny)
    span = hmap["span"].T
    tall = np.nan_to_num(span, nan=0.0) > JC.WALL_SPAN

    cam = hmap.get("center_xy")
    cam_xy = None
    if NEG_CHECK and cam is not None:
        cam_xy = (float(cam[0]), float(cam[1]))

    ox, oy = float(hmap["origin"][0]), float(hmap["origin"][1])
    r = JC.judge(hm, cam_xy=cam_xy, tall=tall, origin=(ox, oy))

    # (nx,ny) -> (ny,nx) 로 되돌린다
    return dict(
        cost=np.asarray(r["cost"]).T,
        slope=np.asarray(r["sl"]).T,
        step=np.asarray(r["step"]).T,
        rough=np.asarray(r["rough"]).T,
        why=np.asarray(r["why"]).T,
        judge_stats=JC.stats(r),
    )


# ── 판정기 등록표 ────────────────────────────────────────────────────
# 새 판정 시스템을 붙이려면 여기에 한 줄만 추가하면 된다.
#   1) 계약대로 함수를 쓴다 (이 파일이든 다른 파일이든)
#   2) 아래 사전에 '이름': 함수 를 추가
#   3) 실행 시  judge:=<이름>  (노드) 또는  --judge <이름>  (CLI)
def _builtin(hmap, p):
    from trav_core import judge_builtin
    return judge_builtin(hmap, p)


JUDGES = {
    "builtin": _builtin,              # trav_core.judge_builtin — 로버 제원 기반 3지표 램프
    "judge_core": judge_judgecore,    # ~/daron_ros/judge_core.py — CH09/K17 기반 판정
}


def get_judge(name):
    if name not in JUDGES:
        raise SystemExit(f"모르는 판정기: {name!r}. 가능한 값: {sorted(JUDGES)}")
    return JUDGES[name]
