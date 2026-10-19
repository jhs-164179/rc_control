"""Relative morphology index, not a published/calibrated crater-age equation.

Larger observed wall slope and image-rim score mean better preservation under
the declared acquisition protocol. Equal weights are an explicit initial
choice. Never pool different view cells or replace a missing feature by zero.
All thresholds in Policy are engineering defaults, not literature constants.
"""
from dataclasses import dataclass, asdict
from collections import defaultdict, Counter
from statistics import median
import hashlib
import json
import math
import random


def finite(x):
    return isinstance(x, (int, float)) and not isinstance(x, bool) and math.isfinite(x)


def quantile(xs, p):
    xs = sorted(xs)
    if not xs or not 0 <= p <= 1:
        raise ValueError('invalid quantile input')
    j = (len(xs) - 1) * p
    k = int(j)
    return xs[k] + (xs[min(k + 1, len(xs) - 1)] - xs[k]) * (j - k)


@dataclass(frozen=True)
class Policy:
    block_seconds: float = 5.0
    min_blocks: int = 5
    min_span_seconds: float = 20.0
    min_bbox_side_px: float = 32.0
    min_depth_valid_fraction: float = 0.5
    min_wall_sectors: int = 3
    min_wall_windows: int = 6
    max_sync_ms: float = 33.0
    max_background_delta_deg: float = 5.0
    max_saturation_fraction: float = 0.1
    max_temporal_slope_iqr_deg: float = 8.0
    bootstrap_replicates: int = 500
    bootstrap_chunk_blocks: int = 2
    pair_confidence: float = 0.95
    min_reference_craters: int = 4

    def __post_init__(self):
        positive = ('block_seconds', 'min_blocks', 'min_bbox_side_px',
                    'min_wall_sectors', 'min_wall_windows', 'bootstrap_replicates',
                    'bootstrap_chunk_blocks', 'min_reference_craters')
        for key in positive:
            if not finite(getattr(self, key)) or getattr(self, key) <= 0:
                raise ValueError('invalid policy: ' + key)
        for key in ('min_blocks', 'min_wall_sectors', 'min_wall_windows',
                    'bootstrap_replicates', 'bootstrap_chunk_blocks', 'min_reference_craters'):
            if type(getattr(self, key)) is not int:
                raise ValueError('policy must be integer: ' + key)
        for key in ('min_span_seconds', 'max_sync_ms', 'max_background_delta_deg',
                    'max_temporal_slope_iqr_deg'):
            if not finite(getattr(self, key)) or getattr(self, key) < 0:
                raise ValueError('invalid policy: ' + key)
        for key in ('min_depth_valid_fraction', 'max_saturation_fraction'):
            if not finite(getattr(self, key)) or not 0 <= getattr(self, key) <= 1:
                raise ValueError('invalid fraction: ' + key)
        if not .5 < self.pair_confidence < 1:
            raise ValueError('pair_confidence must be > .5 and < 1')


def rejection_reasons(o, policy):
    """Acquisition/measurement gates. No high-slope or high-rim cutoff."""
    reasons = []
    for key in ('observation_id', 'mission_id', 'crater_id', 'protocol_id',
                'comparison_group', 'view_cell', 'support_id'):
        if not isinstance(o.get(key), str) or not o[key].strip():
            reasons.append('missing_' + key)
    if o.get('identity_confirmed') is not True:
        reasons.append('identity_unconfirmed')
    if o.get('acquisition_verified') is not True:
        reasons.append('acquisition_unverified')
    for key in ('timestamp_s', 'slope_deg', 'rim_score'):
        if not finite(o.get(key)):
            reasons.append('missing_or_nonfinite_' + key)
    if finite(o.get('timestamp_s')) and o['timestamp_s'] < 0:
        reasons.append('negative_timestamp')
    if finite(o.get('slope_deg')) and not 0 <= o['slope_deg'] < 90:
        reasons.append('slope_out_of_range')
    if finite(o.get('rim_score')) and o['rim_score'] < 0:
        reasons.append('negative_rim_score')
    q = o.get('quality') or {}
    checks = [('bbox_min_side_px', policy.min_bbox_side_px, None),
              ('depth_valid_fraction', policy.min_depth_valid_fraction, 1.0),
              ('wall_sectors', policy.min_wall_sectors, None),
              ('wall_windows', policy.min_wall_windows, None),
              ('sync_ms', 0, policy.max_sync_ms),
              ('background_delta_deg', 0, policy.max_background_delta_deg),
              ('saturation_fraction', 0, policy.max_saturation_fraction)]
    for key, lower, upper in checks:
        value = q.get(key)
        if not finite(value):
            reasons.append('missing_quality_' + key)
        elif value < lower or (upper is not None and value > upper):
            reasons.append('quality_' + key)
    if q.get('bbox_truncated') is not False:
        reasons.append('truncated_or_unknown_bbox')
    if o.get('slope_source') != 'observed_depth':
        reasons.append('slope_must_be_observed_depth')
    if o.get('rim_status') != 'valid':
        reasons.append('rim_measurement_not_valid')
    if q.get('wall_windows_definition') != 'positive_rising_windows':
        reasons.append('unverified_positive_window_count')
    flags = o.get('geometry_flags')
    if not isinstance(flags, list) or not all(isinstance(f, str) for f in flags):
        reasons.append('missing_geometry_flags')
    else:
        for flag in ('weak_height_signal_relative_to_ground_residual',
                     'weak_observed_depression_evidence',
                     'wall_signal_not_resolved_above_background'):
            if flag in flags:
                reasons.append('geometry_' + flag)
    return reasons


def acquisition_priority(o):
    """Never select the greatest slope, rim score, or composite score."""
    q = o['quality']
    return (-q['depth_valid_fraction'], -q['bbox_min_side_px'],
            q['saturation_fraction'], q['sync_ms'], o['timestamp_s'], o['observation_id'])


def protocol_key(o):
    return (o['protocol_id'], o['comparison_group'], o['view_cell'], o['support_id'])


def aggregate(observations, policy=Policy(), aliases=None, diagnostic=False):
    """One jointly measured pair per time bucket; equal bucket weights.

    Crater IDs are scoped to a mission. Aliases must explicitly map old IDs to
    confirmed physical IDs; no appearance-only cross-frame reidentification.
    Revisit gaps > 1.5 buckets start a new episode for block resampling.
    """
    aliases = aliases or {}
    def canonical(cid):
        seen = set()
        while cid in aliases:
            if cid in seen:
                raise ValueError('alias cycle')
            seen.add(cid)
            cid = aliases[cid]
        return cid
    seen = {}
    buckets = defaultdict(dict)
    audit = defaultdict(Counter)
    total = Counter()
    for original in observations:
        o = dict(original)
        key = (o.get('mission_id'), o.get('observation_id'))
        if not all(isinstance(x, str) and x for x in key):
            raise ValueError('mission_id and observation_id are required')
        blob = json.dumps(o, sort_keys=True, allow_nan=False)
        if key in seen:
            if seen[key] != blob:
                raise ValueError('conflicting duplicate observation: ' + str(key))
            continue
        seen[key] = blob
        cid = canonical(o.get('crater_id', 'unknown'))
        o['crater_id'] = cid
        target = (o['mission_id'], cid)
        total[target] += 1
        reasons = rejection_reasons(o, policy)
        metadata_reasons = [r for r in reasons if r in
                            ('identity_unconfirmed', 'acquisition_unverified')]
        if diagnostic:
            # Retain unresolved metadata on every preview, never manufacture it.
            reasons = [r for r in reasons if r not in metadata_reasons]
        o['_metadata_reasons'] = metadata_reasons
        if reasons:
            audit[target].update(reasons)
            continue
        cell = (o['mission_id'], cid) + protocol_key(o)
        bucket = math.floor(o['timestamp_s'] / policy.block_seconds)
        existing = buckets[cell].get(bucket)
        if existing is None or acquisition_priority(o) < acquisition_priority(existing):
            buckets[cell][bucket] = o
    groups = defaultdict(list)
    represented = set()
    for key, samples in sorted(buckets.items()):
        mission, cid, *cell = key
        represented.add((mission, cid))
        ordered = sorted(samples.items())
        pairs = [[o['slope_deg'], o['rim_score']] for _, o in ordered]
        theta, rim = [median([p[j] for p in pairs]) for j in (0, 1)]
        timestamps = [o['timestamp_s'] for _, o in ordered]
        slope_iqr = quantile([p[0] for p in pairs], .75) - quantile([p[0] for p in pairs], .25)
        reasons = []
        if diagnostic:
            reasons += sorted({r for _, o in ordered for r in o['_metadata_reasons']})
        if len(pairs) < policy.min_blocks:
            reasons.append('insufficient_time_blocks')
        if max(timestamps) - min(timestamps) < policy.min_span_seconds:
            reasons.append('insufficient_time_span')
        if slope_iqr > policy.max_temporal_slope_iqr_deg:
            reasons.append('unstable_observed_slope')
        exemplar = min((o for _, o in ordered), key=lambda o:
                       (abs(o['slope_deg'] - theta) / max(theta, 1.) +
                        abs(o['rim_score'] - rim) / max(rim, 1e-9), o['observation_id']))
        record = dict(mission_id=mission, crater_id=cid, protocol_key=cell,
                      slope_deg=theta, rim_score=rim, blocks=len(pairs),
                      span_seconds=max(timestamps)-min(timestamps), slope_iqr_deg=slope_iqr,
                      status='ready' if not reasons else 'pending', reasons=reasons,
                      observation_id=exemplar['observation_id'], image=exemplar.get('image'),
                      representative_frame_slope_deg=exemplar['slope_deg'],
                      representative_frame_rim_score=exemplar['rim_score'],
                      bbox=exemplar.get('bbox'), frame=exemplar.get('frame'),
                      geometry_flags=sorted({f for _, o in ordered for f in o['geometry_flags']}),
                      pairs=pairs, bucket_ids=[x[0] for x in ordered],
                      selected_observation_ids=[o['observation_id'] for _, o in ordered])
        groups[tuple(cell)].append(record)
    return dict(groups=[dict(protocol_key=list(k), craters=v) for k,v in sorted(groups.items())],
                audit=[dict(mission_id=m, crater_id=c, raw_count=total[(m,c)],
                            gate_rejections=dict(audit[(m,c)]),
                            has_accepted_pair=(m,c) in represented) for m,c in sorted(total)],
                policy=asdict(policy), unique_observations=len(seen))


@dataclass(frozen=True)
class Reference:
    reference_id: str
    protocol_key: tuple
    slope_low: float
    slope_high: float
    rim_low: float
    rim_high: float
    source: str
    calibration_ids: tuple = ()

    def __post_init__(self):
        if not isinstance(self.reference_id, str) or not self.reference_id:
            raise ValueError('reference_id required')
        if len(self.protocol_key) != 4 or not all(isinstance(s,str) and s for s in self.protocol_key):
            raise ValueError('invalid reference protocol')
        values = (self.slope_low, self.slope_high, self.rim_low, self.rim_high)
        if not all(finite(v) for v in values) or self.slope_high <= self.slope_low or self.rim_high <= self.rim_low:
            raise ValueError('reference needs finite, nonconstant feature ranges')

    def normalize(self, slope, rim):
        if not finite(slope) or not finite(rim):
            raise ValueError('both finite measured features required')
        if not 0 <= slope < 90 or rim < 0:
            raise ValueError('invalid feature units/range')
        raw = [(slope-self.slope_low)/(self.slope_high-self.slope_low),
               (rim-self.rim_low)/(self.rim_high-self.rim_low)]
        return [min(1., max(0., x)) for x in raw], any(x < 0 or x > 1 for x in raw)


def calibrate(group, reference_id, source, policy=Policy()):
    """Explicit cohort min/max calibration; persist once, never silently refit.

    These endpoints anchor this comparison, not fresh/old physical extremes.
    All craters have equal influence, regardless of their frame counts.
    """
    rows = [r for r in group['craters'] if r['status'] == 'ready']
    if len(rows) < policy.min_reference_craters:
        raise ValueError('insufficient_ready_craters_for_reference')
    return Reference(reference_id, tuple(group['protocol_key']),
                     min(r['slope_deg'] for r in rows), max(r['slope_deg'] for r in rows),
                     min(r['rim_score'] for r in rows), max(r['rim_score'] for r in rows), source,
                     tuple(r['mission_id'] + '/' + r['crater_id'] for r in rows))


def score(pair, weight):
    return 100. * (1. - weight * pair[0] - (1. - weight) * pair[1])


def _resample_pairs(row, policy, rng):
    # Moving chunks of adjacent time buckets, never crossing a revisit gap.
    chunks = []
    n = len(row['pairs'])
    length = min(policy.bootstrap_chunk_blocks, n)
    for i in range(n-length+1):
        ids = row['bucket_ids'][i:i+length]
        if all(b-a == 1 for a,b in zip(ids, ids[1:])):
            chunks.append(row['pairs'][i:i+length])
    # Do not discard isolated/revisit blocks. Sample short runs separately.
    covered = set()
    for i in range(n-length+1):
        if all(b-a == 1 for a,b in zip(row['bucket_ids'][i:i+length],row['bucket_ids'][i+1:i+length])):
            covered.update(range(i,i+length))
    chunks.extend([[row['pairs'][i]] for i in range(n) if i not in covered])
    result = []
    while len(result) < n:
        result.extend(rng.choice(chunks))
    return [median(p[j] for p in result[:n]) for j in (0,1)]


def rank(group, reference, policy=Policy(), slope_weight=.5, seed=20261012):
    if not finite(slope_weight) or not 0 <= slope_weight <= 1:
        raise ValueError('weight must be in [0,1]')
    if tuple(group['protocol_key']) != tuple(reference.protocol_key):
        raise ValueError('reference_protocol_mismatch')
    ready = [r for r in group['craters'] if r['status'] == 'ready']
    pending = [r for r in group['craters'] if r['status'] != 'ready']
    norm, bootstrap, out = {}, {}, []
    for row in ready:
        uid = row['mission_id'] + '/' + row['crater_id']
        norm[uid], clipped = reference.normalize(row['slope_deg'], row['rim_score'])
        # Stable per-crater RNG: adding another target does not change its draws.
        key_seed = int(hashlib.sha256((str(seed)+uid).encode()).hexdigest()[:16],16)
        rng = random.Random(key_seed)
        bootstrap[uid] = [reference.normalize(*_resample_pairs(row,policy,rng))[0]
                          for _ in range(policy.bootstrap_replicates)]
        scores = [score(p,slope_weight) for p in bootstrap[uid]]
        item = {k:v for k,v in row.items() if k not in ('pairs','bucket_ids')}
        item.update(uid=uid, index=score(norm[uid],slope_weight),
                    normalized_features=norm[uid], reference_extrapolation=clipped,
                    repeatability_interval95=[quantile(scores,.025),quantile(scores,.975)])
        out.append(item)
    out.sort(key=lambda r:(r['index'],r['uid']))
    for row in out:
        row['rank'] = 1 + sum(other['index'] < row['index']-1e-10 for other in out)
    pairs=[]
    weight_cases=[0.,.25,.5,.75,1.]
    for i,a in enumerate(out):
        for b in out[i+1:]:
            ua,ub=a['uid'],b['uid']
            delta=[score(y,slope_weight)-score(x,slope_weight) for x,y in zip(bootstrap[ua],bootstrap[ub])]
            prob=(sum(d>1e-10 for d in delta)+.5*sum(abs(d)<=1e-10 for d in delta))/len(delta)
            x=[norm[ua][j]-norm[ub][j] for j in (0,1)]
            crossover=None if abs(x[0]-x[1])<1e-12 else -x[1]/(x[0]-x[1])
            if crossover is not None and not 0<crossover<1: crossover=None
            # Both endpoint features must be significantly larger/smaller for
            # evidence independent of every nonnegative weighting in [0,1].
            joint=sum(all(p[j]>q[j]+1e-12 for j in (0,1)) for p,q in zip(bootstrap[ua],bootstrap[ub]))/len(delta)
            reverse=sum(all(p[j]<q[j]-1e-12 for j in (0,1)) for p,q in zip(bootstrap[ua],bootstrap[ub]))/len(delta)
            robust = (not a['reference_extrapolation'] and not b['reference_extrapolation'] and
                      max(joint,reverse)>=policy.pair_confidence)
            pairs.append(dict(a=ua,b=ub,p_a_better_at_selected_weight=prob,
                              delta_index_b_minus_a_interval95=[quantile(delta,.025),quantile(delta,.975)],
                              weight_crossover=crossover,
                              robust_better=(ua if joint>=reverse else ub) if robust else None,
                              decision='stable_across_weights_and_resampling' if robust else 'provisional'))
    return dict(reference=asdict(reference),slope_weight=slope_weight,rank_direction='lower_index_better_preserved',
                ranked=out,pending=pending,pairwise=pairs,
                weight_sensitivity=[dict(slope_weight=w,order=[r['uid'] for r in sorted(out,key=lambda r:(score(norm[r['uid']],w),r['uid']))]) for w in weight_cases],
                uncertainty_scope='temporal repeatability conditional on fixed reference, identity and acquisition protocol; not full sensor/model accuracy',
                policy=asdict(policy))
