#!/usr/bin/python3
"""kf_resync 단위 시험 (261011). 실행: /usr/bin/python3 tests/test_lc_resync.py

가상 카메라가 바닥(z=0)과 상자 하나를 보며 키프레임을 쌓는다. 실시간 경로(live_tsdf._integrate_grid 와 같은 계산)로
만든 피쳐맵과, 보관한 원재료를 보정 이력으로 다시 쌓은 피쳐맵을 비교한다.
"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', '3dmap'))
from feature_grid import FeatureGrid  # noqa: E402
import kf_resync as KR  # noqa: E402

H, W, STRIDE, SCALE = 480, 848, 2, 0.001
K = np.array([[431.94, 0, 424.0], [0, 431.94, 240.0], [0, 0, 1.0]])


def rot_z(a):
    c, s = np.cos(a), np.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1.0]])


def T_of(R, t):
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, t
    return T


# 광학(z 앞, x 오른쪽, y 아래) ← 로버(x 앞, y 왼쪽, z 위), 20° 아래로
R_BASE_OPT = np.array([[0, 0, 1.0], [-1.0, 0, 0], [0, -1.0, 0]])
TILT = np.radians(20.0)
R_TILT = np.array([[np.cos(TILT), 0, np.sin(TILT)], [0, 1, 0], [-np.sin(TILT), 0, np.cos(TILT)]])


def cam_pose(x, y, yaw):
    return T_of(rot_z(yaw) @ R_TILT @ R_BASE_OPT, (x, y, 0.12))


def render(T_wc):
    """바닥 z=0 + 상자(0.5~0.7, -0.1~0.1, 높이 0.08) 를 광선 추적한 depth (uint16 mm)."""
    v, u = np.mgrid[0:H, 0:W]
    d_cam = np.stack([(u - K[0, 2]) / K[0, 0], (v - K[1, 2]) / K[1, 1], np.ones_like(u, float)], -1)
    d_w = d_cam @ T_wc[:3, :3].T
    o = T_wc[:3, 3]
    z_hit = np.full((H, W), np.inf)
    with np.errstate(divide='ignore', invalid='ignore'):
        s = -o[2] / d_w[..., 2]                       # 바닥
        ok = (s > 0)
        z_hit[ok] = s[ok]                             # 광학 z = s (d_cam z 성분 1)
        s2 = (0.08 - o[2]) / d_w[..., 2]              # 상자 윗면
        p = o + d_w * s2[..., None]
        top = (s2 > 0) & (p[..., 0] > 0.5) & (p[..., 0] < 0.7) & (np.abs(p[..., 1]) < 0.1)
        z_hit[top] = np.minimum(z_hit[top], s2[top])
    dm = np.where(np.isfinite(z_hit) & (z_hit < 1.0) & (z_hit > 0.07), np.round(z_hit / SCALE), 0)
    return dm.astype(np.uint16)


def live_run(poses, M_used, place=None):
    """실시간 노드와 같은 적분 + 원재료 보관. place 를 주면 depth 는 poses 에서 그리고 적분은 place 포즈로."""
    g = FeatureGrid(cell=0.025)
    store = KR.KeyframeStore(1 << 30)
    for i, T_r in enumerate(poses):
        d = render(T_r)
        T_wc = T_r if place is None else place[i]
        dm = d.astype(np.float32) * SCALE
        sub = dm[::STRIDE, ::STRIDE]
        pw = KR.points_from_sub(sub, K, STRIDE, T_wc)
        g.integrate(pw, T_wc[:3, 3], i * 0.5)
        q = np.where(sub > 0, d[::STRIDE, ::STRIDE], 0).astype(np.uint16)
        store.add(int(i * 5e8), q, T_wc, M_used)
    return g, store


POSES = [cam_pose(0.05 * i, 0.01 * i, np.radians(2.0 * i)) for i in range(8)]


def same_grid(a, b):
    assert a.shape == b.shape and a.ix0 == b.ix0 and a.iy0 == b.iy0, (a.shape, b.shape)
    for k in a.a:
        assert np.array_equal(a.a[k], b.a[k]), k


def test_identity_reproduces_live_exactly():
    g_live, store = live_run(POSES, np.eye(4))
    hist = KR.MapOdomHistory([r[0] for r in store.recs], [np.eye(4)] * len(store))
    g2, upd, st = KR.rebuild(None, lambda: FeatureGrid(cell=0.025), store.recs, hist, K, STRIDE, SCALE, 10_000_000)
    same_grid(g_live, g2)
    assert st['miss'] == 0 and st['moved_max_m'] == 0.0


def test_shift_moves_cells():
    """보정 = 맵 +x 3 cm·yaw 1°. 다시 쌓은 맵 = 처음부터 보정 포즈로 쌓은 맵."""
    D = T_of(rot_z(np.radians(1.0)), (0.03, 0.0, 0.0))
    _, store = live_run(POSES, np.eye(4))
    hist = KR.MapOdomHistory([r[0] for r in store.recs], [D] * len(store))
    g2, upd, st = KR.rebuild(None, lambda: FeatureGrid(cell=0.025), store.recs, hist, K, STRIDE, SCALE, 10_000_000)
    g_ref, _ = live_run(POSES, np.eye(4), place=[D @ T for T in POSES])
    same_grid(g_ref, g2)
    assert abs(st['moved_max_m'] - 0.0) > 0.02
    # 상자 칸(합성 상자에 옆면이 없어 칸 평균 ≈ 2.6 cm)의 중심이 +x 로 약 3 cm 옮겨짐
    g_live, _ = live_run(POSES, np.eye(4))
    a, b = g_live.cells(min_np=5), g2.cells(min_np=5)
    ca, cb = a['x'][a['z'] > 0.015].mean(), b['x'][b['z'] > 0.015].mean()
    assert 0.02 < cb - ca < 0.045, (ca, cb)


def test_partial_history_keeps_missing():
    _, store = live_run(POSES, np.eye(4))
    D = T_of(np.eye(3), (0.05, 0, 0))
    stamps = [r[0] for r in store.recs][4:]                  # 앞 4개는 이력 없음
    hist = KR.MapOdomHistory(stamps, [D] * len(stamps))
    _, upd, st = KR.rebuild(None, lambda: FeatureGrid(cell=0.025), store.recs, hist, K, STRIDE, SCALE, 10_000_000)
    assert st['miss'] == 4 and all(u[0] is None for u in upd[:4]) and all(u[0] is not None for u in upd[4:])


def test_composes_with_previous_correction():
    """두 번째 보정은 첫 보정 위에 덮어쓰기(누적 아님): M_used 를 갱신했으면 결과 = 마지막 이력 기준."""
    _, store = live_run(POSES, np.eye(4))
    D1 = T_of(np.eye(3), (0.05, 0, 0))
    D2 = T_of(np.eye(3), (0.02, 0.01, 0))
    h1 = KR.MapOdomHistory([r[0] for r in store.recs], [D1] * len(store))
    _, upd, _ = KR.rebuild(None, lambda: FeatureGrid(cell=0.025), store.recs, h1, K, STRIDE, SCALE, 10_000_000)
    for rec, (T_new, M_new) in zip(store.recs, upd):
        rec[3], rec[4] = T_new, M_new
    h2 = KR.MapOdomHistory([r[0] for r in store.recs], [D2] * len(store))
    g2, upd2, _ = KR.rebuild(None, lambda: FeatureGrid(cell=0.025), store.recs, h2, K, STRIDE, SCALE, 10_000_000)
    for (T_new, _), T in zip(upd2, POSES):                 # D2·D1⁻¹·D1·T = D2·T (부동소수 오차만)
        assert np.allclose(T_new, D2 @ T, atol=1e-12)
    g_ref, _ = live_run(POSES, np.eye(4), place=[D2 @ T for T in POSES])
    a, b = g_ref.cells(min_np=5), g2.cells(min_np=5)
    assert abs(len(a['x']) - len(b['x'])) <= max(2, 0.002 * len(a['x'])), (len(a['x']), len(b['x']))


def test_base_grid_is_kept():
    base = FeatureGrid(cell=0.025)
    base.integrate(np.array([[-2.0, -2.0, 0.3]] * 10), np.zeros(3), 0.0)
    _, store = live_run(POSES, np.eye(4))
    hist = KR.MapOdomHistory([r[0] for r in store.recs], [np.eye(4)] * len(store))
    g2, _, _ = KR.rebuild(base, lambda: FeatureGrid(cell=0.025), store.recs, hist, K, STRIDE, SCALE, 10_000_000)
    c = g2.cells(min_np=5)
    assert np.any((np.abs(c['x'] + 2.0) < 0.03) & (np.abs(c['z'] - 0.3) < 1e-9))
    assert base.n_frames == 1                                 # 바탕은 복사본에만 쌓임


def test_lookup_tolerance_and_store_cap():
    h = KR.MapOdomHistory([100, 300], [np.eye(4), 2 * np.eye(4)])
    assert h.lookup(290, 20)[0, 0] == 2.0 and h.lookup(200, 20) is None and h.lookup(110, 20)[0, 0] == 1.0
    s = KR.KeyframeStore(100)
    assert not s.add(0, np.random.default_rng(1).integers(0, 60000, (50, 50)).astype(np.uint16), np.eye(4), np.eye(4))
    assert s.full and len(s) == 0


# ======================================================== [261011 밤] 핀·웨이포인트·이력 메시지
sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
from types import SimpleNamespace as NS  # noqa: E402
from lib.pins import PinSet  # noqa: E402
from gui.waypoint_seq import ACTIVE, CLEARING, WaypointSeq, reconstruct  # noqa: E402


def fake_path(stamps, mats):
    poses = []
    for t, M in zip(stamps, mats):
        from scipy.spatial.transform import Rotation as Rot
        q = Rot.from_matrix(M[:3, :3]).as_quat()
        poses.append(NS(header=NS(stamp=NS(sec=t // 1_000_000_000, nanosec=t % 1_000_000_000)),
                        pose=NS(position=NS(x=M[0, 3], y=M[1, 3], z=M[2, 3]),
                                orientation=NS(x=q[0], y=q[1], z=q[2], w=q[3]))))
    return NS(poses=poses)


def test_history_msg_roundtrip_and_correct_point():
    D = T_of(rot_z(np.radians(5.0)), (0.1, -0.02, 0.0))
    h = KR.history_from_path_msg(fake_path([1_000_000_000, 2_000_000_000], [np.eye(4), D]))
    assert np.allclose(h.lookup(2_000_000_000, 1), D, atol=1e-12)
    p, M, Dd = KR.correct_point(h, 2_000_000_000, np.eye(4), (1.0, 0.0, 0.0), 10)
    assert np.allclose(p, D[:3, :3] @ [1, 0, 0] + D[:3, 3]) and np.allclose(M, D)
    assert KR.correct_point(h, 5_000_000_000, np.eye(4), (1, 0, 0), 10) is None


def test_pins_reposition_keeps_ids_and_moves_center():
    ps = PinSet(join_m=0.10, min_count=1)
    srcs = []
    for k, (x, t) in enumerate([(1.0, 1), (1.02, 2), (3.0, 3)]):
        src = [t, np.eye(4)]
        srcs.append(src)
        ps.add((x, 0.0, 0.0), 1.0, 0.05, t, 'crater', src=src)
    ids = sorted(p['id'] for p in ps.pins())
    D = T_of(np.eye(3), (0.05, 0.0, 0.0))
    h = KR.MapOdomHistory([1, 2, 3], [D, D, np.eye(4)])        # 앞 핀 두 검출만 +5 cm

    def fn(src, p):
        r = KR.correct_point(h, src[0], src[1], p, 0)
        if r is None:
            return None
        src[1] = r[1]
        return r[0]
    n, dmax = ps.reposition(fn)
    assert n == 3 and abs(dmax - 0.05) < 1e-9, (n, dmax)
    c = {p['id']: p['xyz'] for p in ps.pins()}
    assert sorted(c) == ids
    assert abs(c[ids[0]][0] - 1.06) < 1e-6 and abs(c[ids[1]][0] - 3.0) < 1e-9
    n2, d2 = ps.reposition(fn)                               # 같은 이력 다시 = 기준이 갱신돼 안 움직임
    assert d2 < 1e-12


def test_waypoint_relocate_resends_active_goal():
    s = WaypointSeq()
    it = s.add(1.0, 0.0, 0.0, 0.0)
    acts = s.tick(0.0, [])
    assert acts == [('goal', 1.0, 0.0, 0.0, 0.0)]
    s.tick(0.1, [(1.0, 0.0)])
    assert s.phase == ACTIVE
    assert s.relocate(it['id'], 1.05, 0.01, 0.0, 0.02)
    assert s.tick(0.2, [(1.0, 0.0)]) == [('clear',)] and s.phase == CLEARING
    acts = s.tick(0.3, [])
    assert len(acts) == 1 and acts[0][0] == 'goal' and np.allclose(acts[0][1:], (1.05, 0.01, 0.0, 0.02)), acts
    s.tick(0.4, [(1.05, 0.01)])
    assert s.phase == ACTIVE and not s.paused


def test_reconstruct_applies_moved(tmp=None):
    import json
    import tempfile
    evs = [dict(event='session_start', session='s', frame='map'),
           dict(event='added', id=1, goal=dict(x=1.0, y=0.0, z=0.0, yaw_deg=10.0), source='rviz'),
           dict(event='moved', reason='lc_resync', moves=[dict(id=1, to=dict(x=1.05, y=0.01, z=0.0, yaw_deg=11.0))])]
    with tempfile.NamedTemporaryFile('w', suffix='.jsonl', delete=False) as f:
        f.write('\n'.join(json.dumps(e) for e in evs) + '\n')
    rec = reconstruct(f.name)
    w = rec['waypoints'][0]
    assert (w['x'], w['y']) == (1.05, 0.01) and abs(np.degrees(w['yaw']) - 11.0) < 1e-9 and not rec['bad']



def test_seeded_pins_absorb_reobservation():
    """이전 세션 핀을 seed → 같은 자리 재관측은 같은 핀(개수 그대로, n 증가), 먼 곳은 새 핀."""
    old = PinSet(join_m=0.10, min_count=3)
    for k in range(4):
        old.add((1.0 + 0.005 * k, 0.0, 0.0), 0.9, 0.06, k, 'crater', box=[0, 0, 1, 1])
        old.add((2.0, 1.0 + 0.004 * k, 0.0), 0.8, 0.05, 10 + k, 'crater', box=[0, 0, 1, 1])
    saved = old.pins('map')
    new = PinSet(join_m=0.10, min_count=3)
    for p in saved:
        new.seed(p, loaded=dict(file='x.json', id=p['id']))
    assert len(new.pins()) == 2 and all(p['loaded'] for p in new.pins()) and all(p['confirmed'] for p in new.pins())
    pid, _ = new.add((1.03, 0.01, 0.0), 0.9, 0.06, 100, 'crater', box=[0, 0, 1, 1])
    pins = {p['id']: p for p in new.pins()}
    assert len(pins) == 2 and pins[pid]['n'] == 5 and pins[pid]['top']['stamp_ns'] == 100
    assert abs(pins[pid]['xyz'][0] - 1.0075) < 0.01            # 이전 4개 무게 + 새 1개
    new.add((3.0, -1.0, 0.0), 0.9, 0.05, 101, 'crater', box=[0, 0, 1, 1])
    assert len(new.pins()) == 3
    n, d = new.reposition(lambda src, p: None)                   # seed 는 reposition 대상 아님
    assert d < 1e-12

if __name__ == '__main__':
    fails = 0
    for name, fn in list(globals().items()):
        if name.startswith('test_') and callable(fn):
            try:
                fn()
                print('PASS', name)
            except Exception as e:  # noqa: BLE001
                fails += 1
                print('FAIL', name, type(e).__name__, e)
    sys.exit(1 if fails else 0)
