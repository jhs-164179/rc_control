"""Fixed 0.5 slope + 0.5 image-rim implementation. Standard library only.

The previous generic rank() stays available for sensitivity research. Mission
entry points below always use 1:1; they never fit a weight to the observed order.
"""
import hashlib
import json
from dataclasses import asdict
from .core import Policy, Reference, aggregate, rank, score

VERSION = 'equalweight-20261013-v1'
WEIGHTS = {'observed_wall_p95_deg': 0.5, 'image_rim_S': 0.5}


def index(slope_deg, rim_score, reference):
    """Use frozen reference limits; missing/nonfinite features raise ValueError."""
    normalized, outside = reference.normalize(slope_deg, rim_score)
    return dict(index=score(normalized, 0.5), normalized_features=normalized,
                slope_contribution=50 * (1 - normalized[0]),
                rim_contribution=50 * (1 - normalized[1]),
                reference_extrapolation=outside)


def evaluate(observations, reference, policy=Policy(), aliases=None):
    """Return qualified ranking and clearly separate pending numeric previews.

    No caller flag promotes pending observations. Strict checks are identical
    for streaming and batch. Input order and duplicate retransmission do not
    alter results; identity changes require explicit aliases.
    """
    observations = list(observations)
    strict = aggregate(observations, policy, aliases)
    diag = aggregate(observations, policy, aliases, diagnostic=True)
    key = tuple(reference.protocol_key)
    group = next((g for g in strict['groups'] if tuple(g['protocol_key']) == key),
                 dict(protocol_key=list(key), craters=[]))
    result = rank(group, reference, policy, slope_weight=0.5)
    qualified = {r['uid'] for r in result['ranked']}
    previews = []
    for g in diag['groups']:
        if tuple(g['protocol_key']) != key:
            continue
        for row in g['craters']:
            uid = row['mission_id'] + '/' + row['crater_id']
            if uid in qualified:
                continue
            value = index(row['slope_deg'], row['rim_score'], reference)
            r = {k:v for k,v in row.items() if k not in ('pairs','bucket_ids')}
            r.update(uid=uid, status='pending', provisional_index=value.pop('index'), **value)
            previews.append(r)
    previews.sort(key=lambda r: (r['provisional_index'], r['uid']))
    blob = json.dumps(asdict(reference), sort_keys=True, separators=(',', ':')).encode()
    result.update(version=VERSION, weights=dict(WEIGHTS),
                  reference_sha256=hashlib.sha256(blob).hexdigest(),
                  preview=previews, audit=strict['audit'],
                  unique_observations=strict['unique_observations'],
                  unscored_protocols=[g['protocol_key'] for g in diag['groups']
                                      if tuple(g['protocol_key']) != key])
    return result
