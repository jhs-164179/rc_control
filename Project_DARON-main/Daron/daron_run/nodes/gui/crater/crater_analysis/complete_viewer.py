"""Offline viewer whose primary view is the complete, inferred crater surface."""
import json
from pathlib import Path
import numpy as np


def write_complete_viewer(report, arrays, out):
    out = Path(out)
    data = np.load(out / 'completed_mesh.npz')
    metadata=json.loads((out/'completion.json').read_text(encoding='utf-8'))
    terrain_mode=metadata.get('method')=='observed_terrain_constrained_completion'
    v = data['top_local_m']
    p = arrays['points_local_m'][arrays['crater_mask']]
    p = p[::max(1, len(p)//1500)]
    payload = dict(vertices=np.round(v, 9).tolist(), faces=data['top_faces'].tolist(),
        support=data['near_observed_support'].astype(int).tolist(),
        points=np.round(p, 9).tolist(), base=float(data['base_z_m']),
        basis=report['local_to_camera_basis_columns'], center=report['circle_center_camera_m'],
        diameter=report['diameter_used_m'],prior=np.round(data['prior_local_m'] if 'prior_local_m' in data else v,9).tolist())
    title = f"Frame {report['frame']} · 완전한 크레이터 형상 추정"
    m = report['slopes']
    subtitle = (f"bbox 기반 직경 {report['diameter_used_m']*100:.1f} cm · 추정 깊이 {m['inferred_depth_m']*100:.1f} cm"
                f" · 관측 최대 경사 {m['observed_max_angle_deg']:.1f}° · 모델 림 경사 {m['inferred_rim_max_angle_deg']:.1f}°")
    html = '''<!doctype html><html lang="ko"><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>완전한 크레이터 mesh</title><style>
*{box-sizing:border-box}body{margin:0;background:#242424;color:#eee;font:14px system-ui,sans-serif}
header{padding:22px 26px 12px;background:#1c1c1c}h1{font-size:23px;margin:0 0 8px}p{margin:7px 0;line-height:1.6;color:#ccc}
nav{padding:12px 26px;display:flex;align-items:center;gap:17px;flex-wrap:wrap;background:#2c2c2c}
button,select,a{color:#eee;background:#3b3b3b;border:1px solid #666;border-radius:5px;padding:7px 10px;text-decoration:none}
canvas{width:100%;height:67vh;display:block;touch-action:none}footer{padding:12px 26px;line-height:1.7;color:#bcbcbc}
input[type=range]{width:100px;vertical-align:middle}output{font-weight:bold}.hint{color:#bcbcbc;font-size:12px}
</style><header><h1>__TITLE__</h1><p>__SUBTITLE__</p>
<p>관측한 직경·내벽 경사로 회전 포물면을 추정하여, 보이지 않는 바닥과 반대편 내벽까지 연결한 표면입니다.</p></header>
<nav><select id="frame"><option value="local">지면 기준 좌표</option><option value="camera">카메라 좌표 · 지면 기울기 유지</option></select>
<label><input type="checkbox" id="wire"> 삼각형 표시</label><label><input type="checkbox" id="support"> 관측 근처 영역 표시</label>
<label><input type="checkbox" id="points"> 관측 점 겹쳐 보기</label>
<label>높이 표시 <input id="factor" type="range" min="1" max="5" step="0.5" value="1"> <output id="factorValue">×1</output></label>
<button id="reset">시점 초기화</button><span class="hint">드래그: 회전 / 휠: 확대</span></nav>
<canvas id="view"></canvas><footer>
<a href="completed_surface_camera.obj">전체 표면 OBJ · 카메라 좌표</a>
<a href="completed_solid_local.obj">닫힌 입체 OBJ · 지면 좌표</a>
<a href="completed_solid_local.stl">닫힌 입체 STL · 단위 m</a>
<p>시험장 자체의 측정 높이 유지: Z축을 5로 나누거나 다시 곱하지 않음. 높이 표시 배율은 화면에만 적용됩니다.<br>
Zhou et al. (2023), Eq. 6의 회전 포물면에 측정한 주변 지면 추세를 더했습니다. 경사에서 깊이를 역산하는 부분은 본 구현의 확장입니다.<br>
전체 표면은 모델 추정입니다. 영역 표시의 청록색은 관측 점의 투영 위치 근처, 회색은 그 밖의 영역입니다. 점 겹치기는 가려진 점도 표시합니다.<br>
갈색 받침과 닫힌 파일의 측벽·밑면은 내보내기용 구조이며 지하 측정값이 아닙니다.</p></footer>
<script>
const DATA=__DATA__;
const canvas=document.getElementById('view'),ctx=canvas.getContext('2d');
let rx=.95,rz=-.55,zoom=1,drag=null,pending=false;
function requestDraw(){if(!pending){pending=true;requestAnimationFrame(()=>{pending=false;draw()})}}
function draw(){
 const w=canvas.clientWidth,h=canvas.clientHeight,dpr=window.devicePixelRatio||1;
 canvas.width=Math.round(w*dpr);canvas.height=Math.round(h*dpr);ctx.setTransform(dpr,0,0,dpr,0,0);ctx.fillStyle='#242424';ctx.fillRect(0,0,w,h);
 const factor=Number(document.getElementById('factor').value),cam=document.getElementById('frame').value==='camera';
 document.getElementById('factorValue').textContent=`×${factor}`;
 function world(p){const q=[p[0],p[1],p[2]*factor];return cam?DATA.center.map((x,k)=>x+DATA.basis[k].reduce((s,b,j)=>s+b*q[j],0)):q}
 const V=DATA.vertices.map(world),c=[0,1,2].map(k=>V.reduce((s,p)=>s+p[k],0)/V.length);
 const scale=Math.min(w*.8,h*1.05)/DATA.diameter/2*zoom;
 function rotate(p){const [x,y,z]=p.map((q,k)=>q-c[k]);let a=x*Math.cos(rz)-y*Math.sin(rz),b=x*Math.sin(rz)+y*Math.cos(rz);return [a,b*Math.cos(rx)-z*Math.sin(rx),b*Math.sin(rx)+z*Math.cos(rx)]}
 function project(v){return [w/2+v[0]*scale,h*.5+v[1]*scale,v[2]]}
 const R=V.map(rotate),P=R.map(project),n=DATA.vertices.length;
 const xs=DATA.vertices.map(p=>p[0]),ys=DATA.vertices.map(p=>p[1]),loX=Math.min(...xs),hiX=Math.max(...xs),loY=Math.min(...ys),hiY=Math.max(...ys);
 const base=[[loX,loY,DATA.base],[hiX,loY,DATA.base],[hiX,hiY,DATA.base],[loX,hiY,DATA.base]].map(world).map(rotate);
 R.push(...base);P.push(...base.map(project));
 const F=DATA.faces.map(f=>({f,base:false}));F.push({f:[n,n+1,n+2],base:true},{f:[n,n+2,n+3],base:true});
 F.forEach(o=>o.z=o.f.reduce((s,i)=>s+R[i][2],0)/3);F.sort((a,b)=>a.z-b.z);
 const showSupport=document.getElementById('support').checked,wire=document.getElementById('wire').checked;
 for(const o of F){const f=o.f,a=R[f[0]],b=R[f[1]],c=R[f[2]],u=b.map((q,k)=>q-a[k]),v=c.map((q,k)=>q-a[k]);
   const normal=[u[1]*v[2]-u[2]*v[1],u[2]*v[0]-u[0]*v[2],u[0]*v[1]-u[1]*v[0]],length=Math.hypot(...normal)||1;
   const shade=.40+.55*Math.max(0,(normal[0]*-.3+normal[1]*-.5+normal[2]*.81)/length);
   let rgb=o.base?[126,53,13]:showSupport&&f.some(i=>DATA.support[i])?[80,198,184]:[245,245,245];
   ctx.fillStyle=`rgb(${rgb.map(x=>Math.round(x*(o.base?1:shade))).join(',')})`;
   ctx.beginPath();f.forEach((i,k)=>k?ctx.lineTo(P[i][0],P[i][1]):ctx.moveTo(P[i][0],P[i][1]));ctx.closePath();ctx.fill();
   // A subpixel stroke in the same opaque colour prevents anti-alias seams.
   ctx.strokeStyle=wire&&!o.base?'#555':ctx.fillStyle;ctx.lineWidth=wire?.5:.55;ctx.stroke();
 }
 if(document.getElementById('points').checked){ctx.fillStyle='#28dbd2';for(const p of DATA.points){const q=project(rotate(world(p)));ctx.beginPath();ctx.arc(q[0],q[1],1.5,0,Math.PI*2);ctx.fill()}}
 ctx.fillStyle='#ccc';ctx.font='13px system-ui';ctx.fillText(`바닥·반대편 내벽 포함 · 전체 추정 표면 | 표시 ×${factor}`,22,h-20);
}
canvas.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId)});
canvas.addEventListener('pointermove',e=>{if(drag){rz+=(e.clientX-drag[0])*.008;rx+=(e.clientY-drag[1])*.008;drag=[e.clientX,e.clientY];requestDraw()}});
canvas.addEventListener('pointerup',()=>drag=null);canvas.addEventListener('pointercancel',()=>drag=null);
canvas.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.25,Math.min(5,zoom*Math.exp(-e.deltaY*.001)));requestDraw()},{passive:false});
document.querySelectorAll('input,select').forEach(el=>el.addEventListener('input',requestDraw));
document.getElementById('reset').onclick=()=>{rx=.95;rz=-.55;zoom=1;requestDraw()};window.addEventListener('resize',requestDraw);requestDraw();
</script></html>'''
    html = html.replace('__TITLE__',title).replace('__SUBTITLE__',subtitle).replace('__DATA__',json.dumps(payload,separators=(',',':')))
    if terrain_mode:
        new_subtitle=(f"bbox 기반 기준 직경 {report['diameter_used_m']*100:.1f} cm · 최대 함몰 추정 {metadata['inferred_depth_m']*100:.1f} cm"
            f" · 관측 최대 경사 {m['observed_max_angle_deg']:.1f}° · 방향별 높이 제약 {metadata['observed_cells']:,}구역")
        html=html.replace(subtitle,new_subtitle)
        html=html.replace('관측한 직경·내벽 경사로 회전 포물면을 추정하여, 보이지 않는 바닥과 반대편 내벽까지 연결한 표면입니다.',
            '실제 Depth의 방향별 높이와 주변 지형을 따라 표면을 맞추고, 가려진 영역을 연속적으로 채웠습니다. 원형 림과 회전대칭을 강제하지 않습니다.')
        html=html.replace('<nav><select id="frame"><option value="local">지면 기준 좌표</option><option value="camera">카메라 좌표 · 지면 기울기 유지</option></select>',
            '<nav><select id="shape"><option value="terrain">관측 지형 반영 · 현재</option><option value="prior">회전대칭 포물면 · 이전</option></select><select id="frame"><option value="camera">카메라 기준 · 원래 기울기 유지</option><option value="local">지면을 수평으로 정렬 · 분석용</option></select>')
        html=html.replace("function world(p){const q=[p[0],p[1],p[2]*factor];return cam?DATA.center.map((x,k)=>x+DATA.basis[k].reduce((s,b,j)=>s+b*q[j],0)):q}",
            "function world(p){const q=[p[0],p[1],p[2]*factor];if(!cam)return q;const a=DATA.center.map((x,k)=>x+DATA.basis[k].reduce((s,b,j)=>s+b*q[j],0));return [a[0],a[2],-a[1]]}")
        html=html.replace('const V=DATA.vertices.map(world)',"const V=(document.getElementById('shape').value==='prior'?DATA.prior:DATA.vertices).map(world)")
        html=html.replace('Zhou et al. (2023), Eq. 6의 회전 포물면에 측정한 주변 지면 추세를 더했습니다. 경사에서 깊이를 역산하는 부분은 본 구현의 확장입니다.',
            '방향별 관측 높이는 공간 구역별 중앙값으로 맞추며, 빈 구역은 부드러운 2차원 보정장으로 연결합니다. 논문 포물면은 가려진 부분의 초기 형상으로만 사용합니다. 이 보완 알고리즘은 본 구현의 확장입니다.<br>카메라 X/전방 Z/화면 위쪽 −Y로 표시하여 측정된 기울기를 유지합니다. SLAM·중력 기준 좌표로 보정한 것은 아닙니다.')
    (out/'complete_mesh_viewer.html').write_text(html,encoding='utf-8')
