#!/bin/bash
# 지상국 GUI 세 창(3D RViz, 탑뷰 RViz, 제어창)을 동시에 각각 mp4 로 녹화 (261007).
# 사용:  ~/Daron/daron_run/tools/record_gui.sh <초> <출력 앞부분>
#   예)  ~/Daron/daron_run/tools/record_gui.sh 360 ~/videos/site_01
#        → ~/videos/site_01_3d.mp4, site_01_top.mp4, site_01_ctl.mp4  (세 개가 같은 순간에 시작)
#   - 세 창이 서로 겹치지 않게, 화면 안에 다 보이게 놓고 시작 (x11grab 은 화면을 찍으므로 가린 창이 찍힘).
#     시작 때 창 위치로 영역이 고정 — 녹화 중 창을 옮기지 말 것. 겹치면 시작 전에 경고만 한다.
#   - 일찍 끝내려면 Ctrl+C (ffmpeg 가 파일을 마무리하고 끝남).
#   - 인코더 기본 nvenc (GPU 전용 인코더, CPU 를 SLAM·판정에 남김). CPU 로: ENC=x264 record_gui.sh ...
set -u
DUR=${1:?"사용: record_gui.sh <초> <출력 앞부분>   예) record_gui.sh 360 ~/videos/site_01"}
PRE=${2:?"사용: record_gui.sh <초> <출력 앞부분>"}
PRE=${PRE%.mp4}
export ENC=${ENC:-nvenc}
HERE=$(cd "$(dirname "$0")" && pwd)
# 이름  창 제목에 든 글자
WINS=("3d daron_viz3d" "top daron_gui_topview" "ctl 지상국")

# 겹침 검사 (경고만)
geo() { xwininfo -root -tree | grep -E '"[^"]+"' | grep -v -i tooltip | grep -F "$1" |
        awk '{ if (match($0, /[0-9]+x[0-9]+\+-?[0-9]+\+-?[0-9]+ +\+-?[0-9]+\+-?[0-9]+/)) { s = substr($0, RSTART, RLENGTH);
               split(s, a, /[x+ ]+/); if (a[1] * a[2] > best) { best = a[1] * a[2]; r = a[1] " " a[2] " " a[5] " " a[6] } } }
             END { if (best > 100) print r }'; }
declare -A R
for w in "${WINS[@]}"; do
    set -- $w
    g=$(geo "$2")
    if [ -z "$g" ]; then echo "[record_gui] '$2' 창이 없음 — 이 창은 건너뜀"; continue; fi
    R[$1]="$g"
done
names=("${!R[@]}")
for ((i = 0; i < ${#names[@]}; i++)); do for ((j = i + 1; j < ${#names[@]}; j++)); do
    read -r w1 h1 x1 y1 <<< "${R[${names[i]}]}"; read -r w2 h2 x2 y2 <<< "${R[${names[j]}]}"
    if (( x1 < x2 + w2 && x2 < x1 + w1 && y1 < y2 + h2 && y2 < y1 + h1 )); then
        echo "[record_gui] ⚠ ${names[i]} 창과 ${names[j]} 창이 겹침 — 겹친 부분은 위에 있는 창이 찍힘. 창을 옮기고 다시 시작 권장"
    fi
done; done
[ ${#names[@]} -gt 0 ] || { echo "[record_gui] 찍을 창이 없음 — daron_gui 가 떠 있나?"; exit 1; }

pids=()
for w in "${WINS[@]}"; do
    set -- $w
    [ -n "${R[$1]:-}" ] || continue
    "$HERE/record_rviz.sh" "$DUR" "${PRE}_$1.mp4" "$2" &
    pids+=($!)
done
trap 'echo "[record_gui] 중단 — 파일 마무리 중"; kill -INT "${pids[@]}" 2>/dev/null' INT TERM
wait "${pids[@]}"
echo "[record_gui] 끝: $(ls -1 "${PRE}"_*.mp4 2>/dev/null | tr '\n' ' ')"
