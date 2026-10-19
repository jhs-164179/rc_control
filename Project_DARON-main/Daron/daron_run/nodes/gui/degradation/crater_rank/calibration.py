"""Optional weight fitting to independently supplied blind pair labels.

Input pair labels: {a: uid, b: uid, better: uid}. Never generate these labels
from the existing index. Validation removes every pair involving the held-out
crater from training. It measures agreement, not true crater age accuracy.
Reference bounds must be fixed separately, not fitted on held-out labels.
"""
from .core import score


def fit_weight(features,labels,min_craters=8):
    ids={p[k] for p in labels for k in ('a','b')}
    if len(ids)<min_craters:raise ValueError('at least %d independently labelled craters required'%min_craters)
    if not ids.issubset(features):raise ValueError('missing normalized feature')
    for pair in features.values():
        if len(pair)!=2 or any(not isinstance(x,(int,float)) or not 0<=x<=1 for x in pair):
            raise ValueError('features must be finite normalized pairs')
    seen=set()
    for p in labels:
        if p['a']==p['b'] or p['better'] not in (p['a'],p['b']):raise ValueError('invalid pair label')
        key=tuple(sorted((p['a'],p['b'])))
        if key in seen:raise ValueError('duplicate pair label; resolve annotator disagreement first')
        seen.add(key)
    def agreement(rows,w):
        values=[]
        for p in rows:
            diff=score(features[p['b']],w)-score(features[p['a']],w)
            if abs(diff)<1e-10:values.append(.5)
            else:values.append(float((diff>0)==(p['better']==p['a'])))
        return sum(values)/len(values)
    grid=[i/100 for i in range(101)]
    def best(rows):return min(grid,key=lambda w:(-agreement(rows,w),abs(w-.5),w))
    folds=[]
    for cid in sorted(ids):
        train=[p for p in labels if cid not in (p['a'],p['b'])]
        test=[p for p in labels if cid in (p['a'],p['b'])]
        if not train or not test:continue
        w=best(train)
        folds.append(dict(held_out_crater=cid,slope_weight=w,agreement=agreement(test,w),equal_weight_agreement=agreement(test,.5),test_pairs=len(test)))
    if len(folds)!=len(ids):raise ValueError('insufficient pair connectivity for crater-held-out validation')
    return dict(candidate_slope_weight=best(labels),folds=folds,
                mean_crater_held_out_agreement=sum(f['agreement'] for f in folds)/len(folds),
                equal_weight_mean_agreement=sum(f['equal_weight_agreement'] for f in folds)/len(folds),
                automatically_approved=False,requires='new independent mission validation before replacing deployed weight')
