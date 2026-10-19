#!/usr/bin/python3
"""열화도 순번 연결부 시험 (261013). 실행: /usr/bin/python3 tests/test_degradation.py

gui/degradation_bridge.py (전달본 gui/degradation/crater_rank 위) 와 lib/pins.PinSet 의 5 s 구간 샘플링.
측정값은 합성 — 실제 영상 실험 결과가 아님. 저장된 실제 스냅샷이 있으면 measure_frame 도 한 장 돌려 본다.
"""
import glob
import json
import os
import sys


HERE = os.path.dirname(os.path.abspath(__file__))
sys.path[:0] = [os.path.join(HERE, '..'), os.path.join(HERE, '..', 'gui')]
import degradation_bridge as DB  # noqa: E402
from lib.pins import PinSet  # noqa: E402

PROTO = dict(protocol_id='p', comparison_group='g', view_cell='v', support_id='s')
KEY = ('p', 'g', 'v', 's')
POL = DB.load_policy()
QUAL = dict(bbox_min_side_px=80.0, depth_valid_fraction=0.9, sync_ms=0.0, background_delta_deg=1.0,
            saturation_fraction=0.0, bbox_truncated=False)


def geo(p95, sectors=6, windows=8):
    w = [dict(sector=i % sectors, rise_m=0.01, angle_deg=p95) for i in range(windows)]
    return dict(slopes=dict(angle_p95_deg=p95, observed_max_angle_deg=p95 + 3, valid_sectors=list(range(sectors)), windows=w),
                quality_flags=[])


def obs(cid, t_s, p95, rim, *, verified=False, confirmed=True, slope_ok=True, box=(100, 100, 200, 180)):
    g = geo(p95) if slope_ok else {}
    return DB.observation(mission_id='m', crater_id=cid, stamp_ns=int(t_s * 1e9), box=list(box), geometry=g,
                          rim=dict(status='valid', rim_sharpness_raw=rim), quality=dict(QUAL),
                          errors={} if slope_ok else {'slope': 'poor_plane_fit'}, protocol=PROTO,
                          identity_confirmed=confirmed, acquisition_verified=verified)


def test_order_excludes_uncomputed():
    o = [obs('1', 100, 35.0, 1.5), obs('2', 100, 20.0, 0.5), obs('3', 100, 28.0, 1.0),
         obs('4', 100, 30.0, 1.0, slope_ok=False)]                      # 4 = 경사 실패 → 순서에 없음
    r = DB.rank_order(o, KEY, POL)
    assert [x['crater_id'] for x in r['order']] == ['1', '3', '2'], r['order']   # 경사·림 큰 쪽 = 열화 덜 됨 = 1번
    assert [x['order'] for x in r['order']] == [1, 2, 3]
    assert r['provisional_reference'] and r['reference']['reference_id'] == 'provisional-cohort-3'
    assert r['order'][0]['D'] == 0.0 and r['order'][-1]['D'] == 100.0
    assert all(x['status'] == 'preview' for x in r['order'])               # 촬영 조건 미확인 → 보류
    assert r['per_crater']['4']['status'] == 'no_index'
    assert 'missing_or_nonfinite_slope_deg' in r['per_crater']['4']['reasons']
    assert '4' not in [x['crater_id'] for x in r['order']]
    json.dumps(r, allow_nan=False)


def test_single_crater_no_order():
    r = DB.rank_order([obs('1', 100, 35.0, 1.5)], KEY, POL)
    assert r['order'] == [] and r['reference'] is None and '1개' in r['note']
    assert r['per_crater']['1']['status'] == 'no_index'


def test_ranked_needs_repetition_and_verification():
    o = []
    for cid, p95, rim in (('1', 35.0, 1.5), ('2', 20.0, 0.5)):
        o += [obs(cid, 100 + 5 * k, p95 + 0.1 * k, rim, verified=True) for k in range(6)]   # 5 s 구간 6개, 25 s
    o += [obs('3', 100, 30.0, 1.0, verified=True)]                       # 한 번만 봄 → 보류
    r = DB.rank_order(o, KEY, POL)
    st = {x['crater_id']: x['status'] for x in r['order']}
    assert st == {'1': 'ranked', '2': 'ranked', '3': 'preview'}, st
    one = next(x for x in r['order'] if x['crater_id'] == '1')
    assert one['order'] == 1 and one['confirmed_rank'] == 1 and one['blocks'] == 6


def test_fixed_reference_and_aliases():
    ref = DB.Reference('field-v1', KEY, 15.0, 40.0, 0.3, 2.0, 'test')
    o = [obs('1', 100, 35.0, 1.5), obs('5', 106, 34.0, 1.4), obs('2', 100, 20.0, 0.5)]
    r = DB.rank_order(o, KEY, POL, ref, aliases={'5': '1'})            # 핀 5 가 1 에 합쳐짐
    assert [x['crater_id'] for x in r['order']] == ['1', '2']
    assert next(x for x in r['order'] if x['crater_id'] == '1')['blocks'] == 2
    assert not r['provisional_reference'] and r['reference']['reference_id'] == 'field-v1'
    # D = 50(1-uθ)+50(1-uS), 고정 기준이면 크레이터 수와 무관
    d2 = next(x for x in r['order'] if x['crater_id'] == '2')['D']
    assert abs(d2 - (50 * (1 - 5 / 25) + 50 * (1 - 0.2 / 1.7))) < 1e-9, d2


def test_pinset_blocks():
    ps = PinSet(join_m=0.10, min_count=1, block_s=5.0, block_n=3)
    for k in range(10):                     # 2 s 마다 검출, 박스는 점점 커짐 → 구간마다 큰 박스 하나
        side = 60 + k
        ps.add([0, 0, 0], 0.5, 0.05, int((100 + 2 * k) * 1e9), 'crater', box=[10, 10, 10 + side, 10 + side], z=0.5)
    ps.add([0, 0, 0], 0.9, 0.05, int(121 * 1e9), 'crater', pos_weight=0.0, box=[0, 0, 99, 99], z=0.5)  # 가장자리 → 제외
    ps.add([0, 0, 0], 0.9, 0.05, int(122 * 1e9), 'crater', box=[10, 10, 99, 99], z=2.0)                # Z 범위 밖 → 제외
    b = ps.pins()[0]['blocks']
    assert len(b) == 3                                             # 최근 block_n 구간
    assert [e['stamp_ns'] // int(5e9) for e in b] == [21, 22, 23]
    assert all(e['box'][2] - e['box'][0] == max(60 + k for k in range(10) if (100 + 2 * k) // 5 == e['stamp_ns'] // int(5e9))
               for e in b)
    # 합칠 때 구간 합침
    ps2 = PinSet(join_m=0.10, min_count=1, block_s=5.0, block_n=12)
    ps2.add([0, 0, 0], 0.5, 0.05, int(100e9), 'crater', box=[0, 0, 50, 50], z=0.5)
    ps2.add([1, 0, 0], 0.5, 0.05, int(110e9), 'crater', box=[0, 0, 50, 50], z=0.5)
    ps2.add([0.5, 0, 0], 0.5, 2.0, int(120e9), 'crater', box=[0, 0, 50, 50], z=0.5)   # 큰 반지름 → 둘 다 합침
    p = ps2.pins()
    assert len(p) == 1 and len(p[0]['blocks']) == 3, p


def test_measure_real_snapshot():
    """저장된 실제 스냅샷 한 장 (없으면 건너뜀) — 측정 → 관측 기록이 JSON 으로 저장 가능한지만."""
    import cv2
    for d in sorted(glob.glob(os.path.expanduser('~/daron_gui_runs/*/pins/crater_*'))):
        g = json.load(open(d + '/geometry.json'))
        if g.get('geometry_status') != 'COMPLETE':
            continue
        cam = json.load(open(d + '/camera.json'))
        geo_, rim, q, err = DB.measure_frame(cv2.imread(d + '/rep_infra1.png'), cv2.imread(d + '/rep_depth.png', cv2.IMREAD_UNCHANGED),
                                             cam['K'], g['representative_bbox'], json.load(open(d + '/boxes.json')))
        o = DB.observation(mission_id='m', crater_id='1', stamp_ns=g['representative_stamp'], box=g['representative_bbox'],
                           geometry=geo_, rim=rim, quality=q, errors=err, protocol=PROTO,
                           identity_confirmed=True, acquisition_verified=False)
        json.dumps(o, allow_nan=False)
        assert o['slope_source'] == 'observed_depth' and set(QUAL) <= set(o['quality'])
        assert (o['slope_deg'] is None) == ('slope' in err)
        assert q['sync_ms'] == 0.0 and 0 <= q['depth_valid_fraction'] <= 1
        return 'measured ' + os.path.basename(d) + f' p95={o["slope_deg"]} S={o["rim_score"]}'
    return 'skip (스냅샷 없음)'


if __name__ == '__main__':
    import warnings
    warnings.simplefilter('ignore', RuntimeWarning)            # numba 없음 경고
    for name, fn in list(globals().items()):
        if name.startswith('test_'):
            msg = fn()
            print('PASS', name, msg or '')
