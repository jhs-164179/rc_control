import argparse
import csv
import html
import json
from dataclasses import asdict
from pathlib import Path
from .core import Policy, Reference, aggregate, calibrate, rank


def load_observations(path):
    with open(path,encoding='utf-8-sig') as f:
        return [json.loads(line) for line in f if line.strip()]


def save(path,value):
    Path(path).write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')


def write_gallery(path,result):
    # External destinations are not fetched; image links are local artifacts.
    cards=[]
    for r in result['ranked']:
        img=r.get('image')
        if img:
            p=Path(img)
            url=p.resolve().as_uri() if p.exists() else ''
        else:url=''
        content=('<img src="'+html.escape(url,quote=True)+'">') if url else '<div class="empty">이미지 미등록</div>'
        cards.append('<article>'+content+'<p>림 선명도 '+format(r['rim_score'],'.3f')+
                     ' · 관측 내벽 경사(P95) '+format(r['slope_deg'],'.2f')+'°<br>상대비교 지수 '+format(r['index'],'.2f')+'</p></article>')
    Path(path).write_text('<!doctype html><html lang="ko"><meta charset="utf-8"><title>크레이터 비교</title><style>body{font-family:Arial,Malgun Gothic;background:#fff;margin:28px}.grid{display:grid;grid-template-columns:repeat(2,1fr);gap:28px}article{margin:0}img{display:block;width:100%;height:290px;object-fit:contain;background:#fafafa}p{text-align:center;line-height:1.8;font-size:18px}.empty{height:290px;display:grid;place-items:center}</style><div class="grid">'+''.join(cards)+'</div></html>',encoding='utf-8')


def main():
    ap=argparse.ArgumentParser(description='Dependency-free multi-observation crater comparison')
    ap.add_argument('command',choices=['audit','calibrate','rank'])
    ap.add_argument('--input',required=True,help='measurement observations JSONL')
    ap.add_argument('--out',required=True,help='output directory')
    ap.add_argument('--policy',help='Policy JSON override')
    ap.add_argument('--aliases',help='confirmed old crater ID to canonical ID mapping JSON')
    ap.add_argument('--group-index',type=int,default=0)
    ap.add_argument('--reference',help='immutable reference JSON')
    ap.add_argument('--reference-id',default='mission-calibration-v1')
    ap.add_argument('--weight',type=float,default=.5)
    args=ap.parse_args();out=Path(args.out);out.mkdir(parents=True,exist_ok=True)
    policy=Policy(**(json.loads(Path(args.policy).read_text(encoding='utf-8')) if args.policy else {}))
    aliases=json.loads(Path(args.aliases).read_text(encoding='utf-8')) if args.aliases else {}
    audit=aggregate(load_observations(args.input),policy,aliases);save(out/'audit.json',audit)
    if args.command=='audit': print('audit.json saved');return
    if not 0<=args.group_index<len(audit['groups']):ap.error('group-index not available; inspect audit.json')
    group=audit['groups'][args.group_index]
    if args.command=='calibrate':
        dest=out/'reference.json'
        if dest.exists():ap.error('reference.json exists; use a new output directory to version a reference')
        ref=calibrate(group,args.reference_id,str(Path(args.input).resolve()),policy)
        save(dest,asdict(ref));print('reference.json saved; preserve it for later ranking');return
    if not args.reference:ap.error('rank requires --reference')
    ref=Reference(**json.loads(Path(args.reference).read_text(encoding='utf-8')))
    result=rank(group,ref,policy,args.weight);save(out/'ranking.json',result)
    with (out/'ranking.csv').open('w',encoding='utf-8-sig',newline='') as f:
        fields=['rank','crater_id','slope_deg','rim_score','index','blocks','reference_extrapolation']
        w=csv.DictWriter(f,fieldnames=fields,extrasaction='ignore');w.writeheader();w.writerows(result['ranked'])
    write_gallery(out/'ranking.html',result);print('ranking.json, ranking.csv, ranking.html saved')


if __name__=='__main__':main()
