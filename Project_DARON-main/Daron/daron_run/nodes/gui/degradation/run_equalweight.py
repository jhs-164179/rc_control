"""python run_equalweight.py --input observations.jsonl --reference reference.json --out result"""
import argparse
import csv
import json
from pathlib import Path
from crater_rank.core import Policy, Reference
from crater_rank.equalweight import evaluate


def main():
    ap=argparse.ArgumentParser(description='Fixed 1:1 crater comparison')
    ap.add_argument('--input',required=True)
    ap.add_argument('--reference',required=True)
    ap.add_argument('--out',required=True)
    ap.add_argument('--policy',default=str(Path(__file__).with_name('policy.json')))
    ap.add_argument('--aliases')
    a=ap.parse_args()
    load=lambda p:json.loads(Path(p).read_text(encoding='utf-8-sig'))
    observations=[json.loads(s) for s in Path(a.input).read_text(encoding='utf-8-sig').splitlines() if s.strip()]
    result=evaluate(observations,Reference(**load(a.reference)),Policy(**load(a.policy)),
                    load(a.aliases) if a.aliases else None)
    out=Path(a.out);out.mkdir(parents=True,exist_ok=True)
    (out/'ranking.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    for name,rows in (('qualified',result['ranked']),('pending',result['preview'])):
        fields=['crater_id','slope_deg','rim_score',
                'index' if name=='qualified' else 'provisional_index','blocks','span_seconds','reasons']
        with (out/(name+'.csv')).open('w',encoding='utf-8-sig',newline='') as f:
            w=csv.DictWriter(f,fields,extrasaction='ignore');w.writeheader();w.writerows(rows)
    print(json.dumps(dict(qualified=len(result['ranked']),pending=len(result['preview']),
                         unique_observations=result['unique_observations'],weights=result['weights']),ensure_ascii=False))


if __name__=='__main__': main()
