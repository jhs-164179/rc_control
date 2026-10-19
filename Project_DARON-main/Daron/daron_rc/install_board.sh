#!/usr/bin/env bash
# daron_rc/rover/* 를 보드의 daron_run/rover 에 설치한다 (기존 radxa.py 는 백업).
#
#   bash install_board.sh                              # 기본 대상 /workspace/daron_run/rover
#   bash install_board.sh /다른/경로/daron_run/rover
#   bash install_board.sh --force                      # 보드 radxa.py 가 기준판과 달라도 덮어씀
#   bash install_board.sh --check                      # 비교만 하고 아무것도 안 바꿈
#
# 되돌리기: cp <대상>/backup_rc_4ch_<시각>/radxa.py <대상>/radxa.py  (rc_* 파일은 남아 있어도 무해)
set -euo pipefail

SRC="$(cd "$(dirname "$0")" && pwd)/rover"
DST=${DARON_ROVER_DIR:-/workspace/daron_run/rover}
FORCE=0
CHECK=0
for a in "$@"; do
  case "$a" in
    --force) FORCE=1 ;;
    --check) CHECK=1 ;;
    -*) echo "모르는 옵션: $a"; exit 2 ;;
    *) DST=$a ;;
  esac
done

FILES=(radxa.py rc_core.py rc_ppm_node.py rc_channel_monitor.py rc_env.sh
       run_board_ppm.sh run_board_dry.sh run_board_hardware.sh)

# 이 묶음의 radxa.py 가 바탕으로 삼은 판 (Project_DARON daron_run/rover/radxa.py).
#   3b4c… = 261015 작업본 (SoC 온도 진단 포함)    e5e8… = 커밋 62b824f
#   2f40… = 커밋 fc5c425 (unwrap_zero_deg 가 susp_config 로 이동)
#   1ef8… = 261017 리뷰 수정본 (_stop→_stop_evt, 조향 NaN 검사, 자율 rpm·조향 신선도 분리 — RC 판에도 같은 수정 반영)
# 보드 radxa.py 가 이들이 아니면 보드에서 따로 고친 내용이 덮일 수 있다.
KNOWN_BASE=(
  3b4cefd06c60aad7c5f187225bdf3ec0fcb55232beb04403b59c62fc6c488e88
  e5e8a92e7cbe8406f034e87842219fc0aec4423220ba0a6a136ce19c4255873d
  2f40d17a0ab8dd8dccd71a71c9611bb4f06da11241115bd551420c0402772c13
  1ef87f65e24a28a1da2f16c3253f73b631fe5c981d26fa28998786eb1b7a499f
)

[[ -f "$DST/radxa.py" && -f "$DST/dxl_link.py" ]] || {
  echo "daron_run/rover 가 아닙니다: $DST (radxa.py, dxl_link.py 없음)"; exit 1; }
for f in "${FILES[@]}"; do [[ -f "$SRC/$f" ]] || { echo "묶음에 $f 가 없습니다 ($SRC)"; exit 1; }; done

cur=$(sha256sum "$DST/radxa.py" | cut -d' ' -f1)
new=$(sha256sum "$SRC/radxa.py" | cut -d' ' -f1)
state=unknown
if [[ "$cur" == "$new" ]]; then
  state=installed
else
  for h in "${KNOWN_BASE[@]}"; do [[ "$cur" == "$h" ]] && state=base; done
  # 예전에 설치한 daron_rc 판 (갱신). 보드에서 그 위에 고친 것까지는 알 수 없으니 diff 를 권한다.
  [[ $state == unknown ]] && grep -q '^from rc_core import' "$DST/radxa.py" && state=rc_update
fi
echo "대상: $DST"
case $state in
  installed) echo "radxa.py: 이미 이 묶음의 RC 판입니다 (나머지 파일만 다시 복사)" ;;
  base)      echo "radxa.py: 기준판 확인 — RC 판으로 바꿉니다" ;;
  rc_update) echo "radxa.py: 예전 daron_rc 판 -> 새 판으로 갱신 (보드에서 직접 고친 게 있으면 diff 로 확인:"
             echo "    diff -u $DST/radxa.py $SRC/radxa.py | less )" ;;
  unknown)   echo "radxa.py: ⚠ 기준판과 다릅니다. 보드에서 따로 고친 내용이 있을 수 있습니다."
             echo "  지워질 줄(-) $(diff "$DST/radxa.py" "$SRC/radxa.py" | grep -c '^<' || true) 개. 전부 보기:"
             echo "    diff -u $DST/radxa.py $SRC/radxa.py | less"
             echo "  RC 판이 기준판에서 바꾼 줄은 8 개뿐입니다 (std_msgs import, _FIXED_PARAMS 끝, 파라미터 목록 닫는 ],"
             echo "  _select_mode() 호출, '준비 완료' 로그 끝, SuspensionAxis 의 self._stop 3 곳 -> _stop_evt)."
             echo "  그 밖의 '-' 줄은 보드에서 고친 것이니 옮겨 붙인 뒤 설치하세요." ;;
esac
[[ $CHECK == 1 ]] && exit 0
if [[ $state == unknown && $FORCE != 1 ]]; then
  echo "중단. 확인 후 덮어쓰려면 --force"; exit 3
fi

if pgrep -f '[p]ython3? .*(radxa|rc_ppm_node)\.py' >/dev/null; then
  echo "radxa.py / rc_ppm_node.py 가 실행 중입니다. 먼저 종료하세요."; exit 2
fi

stamp=$(date +%Y%m%d_%H%M%S)
bk="$DST/backup_rc_4ch_$stamp"
mkdir -p "$bk"
for f in "${FILES[@]}"; do [[ -f "$DST/$f" ]] && cp -p "$DST/$f" "$bk/$f"; done
for f in "${FILES[@]}"; do cp "$SRC/$f" "$DST/$f"; done
chmod +x "$DST"/run_board_*.sh "$DST"/rc_ppm_node.py "$DST"/rc_channel_monitor.py
echo "설치 완료. 백업: $bk"
echo "다음: README.md 의 '보드 시험 순서' 1단계부터 (./run_board_ppm.sh)"
