"""
degradation_bridge — 크레이터 열화도(상대비교 지수 D) 를 pin_gallery 에 붙이는 다리. ROS 의존 없음. 작성 261013.

전달본: gui/degradation/ (crater_degradation_equalweight_20261013.zip 그대로, 수정 없음 — manifest.json sha256 일치)
  D = 50·(1 − uθ) + 50·(1 − uS)   uθ, uS = 기준 범위로 0~1 정규화한 관측 내벽 경사 P95 [°] · 림 선명도 S (rim_sharpness_raw)
  D 가 작을수록 그 비교군 안에서 보존이 잘 됨(열화 덜 됨) → 순번 1번. 두 값 중 하나라도 없으면 D 를 만들지 않는다.
  가중치 1:1 고정 (crater_rank.equalweight). 최적 가중치를 검증한 것이 아님 (전달본 README §1).

한 프레임 측정 (measure_frame) — pin_gallery 가 스냅샷해 둔 같은 stamp 의 IR·raw depth·K·같은 프레임 박스로
  경사  전달본 integration/observed_features.measure_observed (기존 crater/crater_analysis 사용, 메시 완성과 분리)
  림    기존 rim/analyze_rim.analyze_representative_crop 의 rim_sharpness_raw (1 보다 클 수 있음 — 자르지 않음)
  품질  bbox_min_side_px · depth_valid_fraction · sync_ms · background_delta_deg · saturation_fraction · bbox_truncated
        background_delta_deg = 같은 센서 샘플을 plane / quadratic 지형으로 각각 보정한 P95 차이 (전달본 지침 5)
        sync_ms = 0 — D405 infra1 과 depth 는 같은 센서·같은 stamp, 스냅샷은 stamp 가 정확히 같을 때만 만든다
  → 전달본 integration/record_adapter.make_observation 으로 관측 기록 한 줄. 측정이 실패해도 기록은 남긴다
    (값 없음 → crater_rank 가 audit 에 사유로 집계, 조용히 버리지 않음 — 전달본 README §3).

순서 (rank_order) — crater_rank.equalweight.evaluate 그대로. ranked(확정) / preview(보류) 를 하나의 순서로 보이되 상태를 같이 준다.
  기준 범위(reference): 파일을 주면 그것(전달본 calibrate 로 만든 고정 기준). 없으면 이번 미션에서 두 값이 나온
  크레이터들의 최소·최대로 **임시 기준**을 만든다(provisional) — 크레이터가 늘면 값·순서가 바뀔 수 있다고 표시.
  D 를 못 만든 크레이터는 순서에 넣지 않는다 (요청 261013). 핀별 사유만 따로 돌려준다.
"""
from __future__ import annotations

import hashlib
import json
import math
import sys
from pathlib import Path

import numpy as np

_HERE = Path(__file__).resolve().parent
for _p in (_HERE / 'degradation', _HERE / 'degradation' / 'integration', _HERE / 'crater', _HERE / 'rim',
           _HERE.parent):                       # nodes/ (lib 패키지)
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

from crater_rank.core import Policy, Reference, aggregate  # noqa: E402
from crater_rank.equalweight import VERSION, WEIGHTS, evaluate  # noqa: E402
from record_adapter import make_observation  # noqa: E402

POLICY_PATH = _HERE / 'degradation' / 'policy.json'
INVALID_RAW = 60000                     # pin_gallery·CDA 와 같은 센서 무효값 규칙


def load_policy(path=None):
    return Policy(**json.loads(Path(path or POLICY_PATH).read_text(encoding='utf-8-sig')))


def load_reference(path):
    d = json.loads(Path(path).read_text(encoding='utf-8-sig'))
    d['protocol_key'] = tuple(d['protocol_key'])
    d['calibration_ids'] = tuple(d.get('calibration_ids', ()))
    return Reference(**d)


def _py(v):
    """JSON 저장용 (numpy 스칼라 → float/int, NaN → None)"""
    if isinstance(v, dict):
        return {k: _py(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_py(x) for x in v]
    if isinstance(v, (np.integer,)):
        return int(v)
    if isinstance(v, (float, np.floating)):
        v = float(v)
        return v if math.isfinite(v) else None
    return v


def _background_delta(arrays, radius, cfg):
    """plane / quadratic 지형 보정 P95 차이 [°]. 한쪽이라도 못 구하면 None (0 으로 바꾸지 않음)."""
    from crater_analysis.core import fit_terrain, profile_grid, surface_height
    from observed_features import observed_windows
    pts = arrays['points_local_m']
    xy, z = pts[:, :2] / radius, pts[:, 2]
    ground, crater = arrays['ground_mask'], arrays['crater_mask']
    p95 = {}
    for mode in ('plane', 'quadratic'):
        try:
            terrain = fit_terrain(xy[ground], z[ground], mode)
            bg = surface_height(xy, terrain)
            rr, hh, _, rp, bp = profile_grid(xy[crater], (z - bg)[crater], cfg, return_positions=True,
                                             background_m=bg[crater])
            p95[mode] = observed_windows(rr, hh, radius, cfg, rp, bp)['angle_p95_deg']
        except (ValueError, np.linalg.LinAlgError) as e:
            return None, f'{mode}: {e}'
    return abs(p95['plane'] - p95['quadratic']), None


def measure_frame(image_bgr, depth_u16, K, box, boxes, *, depth_scale=1e-3, edge_margin=2.0, rim=None,
                  analyze_crop=None):
    """같은 stamp 의 IR(BGR)·raw depth(u16)·K·대상 박스·같은 프레임 박스들 → (geometry, rim, quality, errors).
    geometry·rim 은 make_observation 입력 그대로. 실패한 쪽은 {} / status!='valid' 이고 사유는 errors 에.
    rim: 같은 프레임·같은 박스로 이미 계산한 림 결과가 있으면 재사용 (analyze_representative_crop 의 data)."""
    from crater_analysis import Box as CBox, Camera as CCam
    from crater_analysis.core import Config
    from lib.pins import box_on_edge
    from observed_features import measure_observed

    H, W = depth_u16.shape
    errors = {}
    x1, y1, x2, y2 = (float(v) for v in box)
    c0, r0, c1, r1 = max(int(round(x1)), 0), max(int(round(y1)), 0), min(int(round(x2)), W), min(int(round(y2)), H)
    roi = depth_u16[r0:r1, c0:c1]
    crop = image_bgr[r0:r1, c0:c1]
    quality = dict(bbox_min_side_px=float(min(c1 - c0, r1 - r0)),
                   depth_valid_fraction=float(((roi > 0) & (roi < INVALID_RAW)).mean()) if roi.size else 0.0,
                   sync_ms=0.0,
                   background_delta_deg=None,
                   saturation_fraction=float(((crop.max(axis=2) if crop.ndim == 3 else crop) >= 255).mean())
                   if crop.size else None,
                   bbox_truncated=bool(box_on_edge(box, W, H, edge_margin)))

    geometry = {}
    tb = CBox(x1, y1, x2, y2)
    objs = [CBox(*(float(v) for v in b)) for b in boxes]
    idx = max(range(len(objs)), key=lambda i: objs[i].iou(tb)) if objs else -1
    if idx < 0 or objs[idx].iou(tb) < 0.99:
        objs.append(tb)
        idx = len(objs) - 1
    depth_m = depth_u16.astype(np.float64) * depth_scale
    depth_m[(depth_u16 == 0) | (depth_u16 >= INVALID_RAW)] = np.nan
    try:
        cfg = Config(wall_max_r=.7)                          # measure_observed 기본값과 같게
        geometry, arrays = measure_observed(depth_m, CCam(W, H, np.asarray(K, np.float64)), objs, idx, cfg=cfg)
        quality['background_delta_deg'], why = _background_delta(arrays, geometry['diameter_used_m'] / 2, cfg)
        if why:
            errors['background_delta'] = why
    except Exception as e:  # noqa: BLE001  전달본 거부(ValueError) 포함 — 값을 만들지 않는다
        errors['slope'] = f'{type(e).__name__}: {e}' if not isinstance(e, ValueError) else str(e)
        geometry = {}

    if rim is None:
        if crop.size == 0:
            rim = dict(status='pending', reason='empty_crop')
        else:
            try:
                if analyze_crop is None:
                    from analyze_rim import analyze_representative_crop as analyze_crop
                rim, _ = analyze_crop(np.ascontiguousarray(crop), color_order='bgr')
            except Exception as e:  # noqa: BLE001
                rim = dict(status='error', reason=f'{type(e).__name__}: {e}')
    if rim.get('status') != 'valid':
        errors['rim'] = rim.get('reason') or rim.get('status')
    return _py(geometry), rim, quality, errors


def observation(*, mission_id, crater_id, stamp_ns, box, geometry, rim, quality, errors, protocol,
                identity_confirmed, acquisition_verified):
    """측정 한 프레임 → crater_rank 관측 기록 (전달본 make_observation + 기록용 필드).
    frame_id = stamp + 박스 좌상단 → 같은 프레임의 다른 검출과 구분 (전달본 지침 6)."""
    frame_id = f'{int(stamp_ns)}_b{int(box[0])}_{int(box[1])}'
    o = make_observation(mission_id=mission_id, frame_id=frame_id, crater_id=str(crater_id),
                         timestamp_s=int(stamp_ns) / 1e9, protocol_id=protocol['protocol_id'],
                         comparison_group=protocol['comparison_group'], view_cell_id=protocol['view_cell'],
                         support_id=protocol['support_id'], geometry=geometry,
                         rim={'status': rim.get('status'), 'rim_sharpness_raw': rim.get('rim_sharpness_raw')},
                         quality=quality, identity_confirmed=bool(identity_confirmed),
                         acquisition_verified=bool(acquisition_verified))
    o.update(bbox=[round(float(v), 1) for v in box], frame=frame_id, stamp_ns=int(stamp_ns),
             roi='slope=bbox_geometry, rim=bbox_crop (same frame, same box)',
             measurement_errors=errors or None)
    return _py(o)


def provisional_reference(observations, policy, protocol_key, aliases=None):
    """이번 미션 크레이터들의 대표값(5 s 구간 중앙값) 최소·최대 → 임시 기준. 두 범위 모두 0 이 아니어야(크레이터 2개 이상)."""
    diag = aggregate(observations, policy, aliases, diagnostic=True)
    rows = next((g['craters'] for g in diag['groups'] if tuple(g['protocol_key']) == tuple(protocol_key)), [])
    if len(rows) < 2:
        return None, f'두 값이 나온 크레이터 {len(rows)}개 (2개 이상 필요)'
    s = [r['slope_deg'] for r in rows]
    m = [r['rim_score'] for r in rows]
    if max(s) <= min(s) or max(m) <= min(m):
        return None, '크레이터들의 값 범위가 0 (비교 불가)'
    return Reference(f'provisional-cohort-{len(rows)}', tuple(protocol_key), min(s), max(s), min(m), max(m),
                     'mission cohort min/max (provisional, not a fixed field reference)',
                     tuple(r['mission_id'] + '/' + r['crater_id'] for r in rows)), None


def rank_order(observations, protocol_key, policy, reference=None, aliases=None):
    """→ dict(order=[...], per_crater={crater_id: ...}, reference=..., note=...).
    order: D 오름차순(열화 덜 된 것 = 1번). ranked·preview 를 같은 D 로 한 줄에 — status 로 구분.
    D 를 못 만든 크레이터는 order 에 없다. per_crater 에만 사유(audit gate_rejections)."""
    observations = list(observations)
    provisional = reference is None
    note = None
    if provisional:
        reference, note = provisional_reference(observations, policy, protocol_key, aliases)
    out = dict(version=VERSION, weights=dict(WEIGHTS), order=[], per_crater={}, provisional_reference=provisional,
               reference=None, note=note, unique_observations=0, rank_direction='lower_D_less_degraded')
    if reference is None:
        audit = aggregate(observations, policy, aliases)['audit'] if observations else []
        out['unique_observations'] = len({(o.get('mission_id'), o.get('observation_id')) for o in observations})
        for a in audit:
            out['per_crater'][a['crater_id']] = dict(status='no_index', raw_count=a['raw_count'],
                                                     reasons=a['gate_rejections'], note=note)
        return out
    res = evaluate(observations, reference, policy, aliases)
    blob = json.dumps(_py(dict(reference.__dict__)), sort_keys=True, default=list).encode()
    out['reference'] = dict(reference_id=reference.reference_id, protocol_key=list(reference.protocol_key),
                            slope_low=reference.slope_low, slope_high=reference.slope_high,
                            rim_low=reference.rim_low, rim_high=reference.rim_high, source=reference.source,
                            sha256=res.get('reference_sha256') or hashlib.sha256(blob).hexdigest())
    out['unique_observations'] = res['unique_observations']
    rows = []
    for r in res['ranked']:
        rows.append(dict(r, status='ranked', D=r['index'], confirmed_rank=r['rank']))
    for r in res['preview']:
        rows.append(dict(r, status='preview', D=r['provisional_index'], confirmed_rank=None))
    rows.sort(key=lambda r: (r['D'], r['crater_id']))
    for i, r in enumerate(rows, 1):
        n = r['normalized_features']
        item = dict(order=i, crater_id=r['crater_id'], status=r['status'], confirmed_rank=r['confirmed_rank'],
                    D=r['D'], slope_deg=r['slope_deg'], rim_score=r['rim_score'],
                    slope_contribution=50 * (1 - n[0]), rim_contribution=50 * (1 - n[1]),
                    blocks=r['blocks'], span_seconds=r['span_seconds'], slope_iqr_deg=r['slope_iqr_deg'],
                    reasons=r.get('reasons') or [], reference_extrapolation=r['reference_extrapolation'],
                    representative_observation=r.get('observation_id'),
                    repeatability_interval95=r.get('repeatability_interval95'))
        out['order'].append(_py(item))
    total = len(out['order'])
    for item in out['order']:
        out['per_crater'][item['crater_id']] = dict(item, total=total)
    for a in res['audit']:
        if a['crater_id'] not in out['per_crater']:
            out['per_crater'][a['crater_id']] = dict(status='no_index', raw_count=a['raw_count'],
                                                     reasons=a['gate_rejections'])
    out['unscored_protocols'] = res.get('unscored_protocols', [])
    return out
