#!/bin/bash
# RViz·지상국 창을 mp4 로 화면 녹화 (X11, ffmpeg x11grab). 2026-09-20 스모크 테스트에서 검증.
# 사용:  ~/Daron/daron_run/tools/record_rviz.sh <초> [출력.mp4] [창]   (확장자 생략 시 .mp4, 폴더는 자동 생성)
#   [창] = 창 제목에 들어 있는 글자. 기본 daron_viz3d (3D 창). 탑뷰는 daron_gui_topview, 제어창은 '지상국'.
#   (261006: daron_gui.launch.py 가 RViz 를 두 개 띄우므로 '처음 찾은 RViz 창'이면 탑뷰가 잡히기도 했다)
#   (261007: RViz 가 아닌 창도 찾음 — 제어창 'DARON 지상국'. 같은 제목이 여럿이면 가장 큰 창)
#   세 창을 한 번에: tools/record_gui.sh
#   - rviz2 가 떠 있어야 하고, 녹화 중 창이 다른 창에 가려지면 가린 창이 찍힌다(x11grab 은 화면을 찍음).
#     녹화 영역은 시작할 때 창 위치로 고정 — 녹화 중 창을 옮기면 안 됨.
#   - Wayland 세션이면 안 됨 (echo $XDG_SESSION_TYPE 이 x11 이어야 함). 이 PC 는 x11 (260920 확인).
#   - 인코더: ENC=x264(기본, CPU) / ENC=nvenc (GPU 전용 인코더 — CPU 를 SLAM 에 남김. RTX 3060 261007 확인)
set -e
DUR=${1:?"사용: record_rviz.sh <초> [출력.mp4] [창]"}
OUT=${2:-$HOME/rviz_$(date +%y%m%d_%H%M%S).mp4}
# ffmpeg 는 확장자로 컨테이너를 고르므로 확장자가 없으면 .mp4 를 붙인다
# (260920: `record_rviz.sh 60 ~/videos/test` → "Unable to find a suitable output format").
[[ "$OUT" == *.* ]] || OUT="$OUT.mp4"
mkdir -p "$(dirname "$OUT")"
DISP=${DISPLAY:-:1}
PAT=${3:-daron_viz3d}
ENC=${ENC:-x264}
# 제목 있는 창 중 PAT 가 든 것, 가장 큰 것 (1x1 같은 숨은 창 제외)
pick() { xwininfo -root -tree | grep -E '"[^"]+"' | grep -v -i tooltip | grep -F "$1" |
         awk '{ if (match($0, /[0-9]+x[0-9]+\+-?[0-9]+\+-?[0-9]+/)) { g = substr($0, RSTART, RLENGTH); split(g, a, /[x+]/);
                if (a[1] * a[2] > best) { best = a[1] * a[2]; id = $1 } } } END { if (best > 100) print id }'; }
WID=$(pick "$PAT")
if [ -z "$WID" ]; then             # 그 창이 없으면 예전처럼 처음 찾은 RViz 창
    WID=$(pick "RViz")
    [ -n "$WID" ] && echo "[record_rviz] '$PAT' 창이 없어 다른 RViz 창을 찍습니다: $(xwininfo -id "$WID" | sed -n 's/.*"\(.*\)".*/\1/p' | head -1)"
fi
[ -n "$WID" ] || { echo "[record_rviz] '$PAT' 창도 RViz 창도 못 찾음 — rviz2·지상국이 떠 있나?"; exit 1; }
G=$(xwininfo -id "$WID")
X=$(echo "$G" | awk '/Absolute upper-left X/{print $NF}')
Y=$(echo "$G" | awk '/Absolute upper-left Y/{print $NF}')
W=$(echo "$G" | awk '/Width/{print $NF}'); H=$(echo "$G" | awk '/Height/{print $NF}')
W=$((W/2*2)); H=$((H/2*2))     # libx264 yuv420p 는 짝수 크기 필요
case "$ENC" in
    nvenc) VC=(-c:v h264_nvenc -preset p4 -cq 23) ;;
    *)     VC=(-c:v libx264 -preset veryfast -crf 20) ;;
esac
echo "[record_rviz] '$PAT' $WID ${W}x${H}+${X}+${Y}  ${DUR}s ($ENC) -> $OUT"
ffmpeg -y -loglevel error -f x11grab -framerate 30 -video_size ${W}x${H} -i ${DISP}.0+${X},${Y} \
       -t "$DUR" "${VC[@]}" -pix_fmt yuv420p "$OUT"
echo "[record_rviz] 저장: $OUT ($(du -h "$OUT" | cut -f1))"
