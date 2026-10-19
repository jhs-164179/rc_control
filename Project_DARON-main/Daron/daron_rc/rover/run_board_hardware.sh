#!/usr/bin/env bash
# RC 실구동. 벤치 확인(README.md 1~3단계)과 비상정지 준비가 끝나기 전에는 쓰지 않는다.
#
#   ./run_board_hardware.sh --hardware-ready l12     XM430 실기(자세 유지) + L12 조향.  VESC 꺼짐, CH4 꺼짐
#   ./run_board_hardware.sh --hardware-ready xm430   위 + CH4 굴절 조그.               VESC 꺼짐
#   ./run_board_hardware.sh --hardware-ready drive   위 + VESC 구동 (바퀴 공중부터).   CH4 는 RC_CH4=1 일 때만
#
#   뒤에 -p 이름:=값 을 더 붙일 수 있다.  예) ... l12 -p rc_ch3_reverse:=true
set -euo pipefail

usage() { sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }
[[ "${1:-}" == "--hardware-ready" ]] || { echo "실구동은 막혀 있습니다. 벤치 확인 후 --hardware-ready <단계> 로."; usage; }
stage=${2:-}
shift 2 || true

case "$stage" in
  l12)   l12=true; vesc=false; ch4=false ;;
  xm430) l12=true; vesc=false; ch4=true ;;
  drive) l12=true; vesc=true;  ch4=$([[ "${RC_CH4:-0}" == 1 ]] && echo true || echo false) ;;
  *) echo "단계를 고르세요: l12 | xm430 | drive"; usage ;;
esac

source "$(dirname "$0")/rc_env.sh"
rc_need_rover_dir
rc_refuse_if_running radxa.py
rc_refuse_safety_override "$@"
[[ -f "$PROFILE" ]] || { echo "[rc] 서스펜션 프로파일이 없습니다: $PROFILE (SUSP_PROFILE=<경로> 로 지정)"; exit 1; }
pgrep -f '[r]c_ppm_node.py' >/dev/null || echo "[rc] ⚠ rc_ppm_node 가 안 떠 있습니다 — RC 링크 없음 = 계속 STOP"

echo "[rc] 단계 $stage : L12=$l12 VESC=$vesc CH4=$ch4  (RC 실구동, 자율 허용 기본 OFF)"
echo "[rc] 비상정지: CH1 을 가운데/반대로, 콘솔 X, 또는 이 터미널 Ctrl+C. 물리 전원 차단 수단을 손에 둘 것."
exec "$PY" "$RC_DIR/radxa.py" --ros-args \
  -p "susp_profile:=$PROFILE" -p susp_mock:=false \
  -p l12_enable:=$l12 -p vesc_enable:=$vesc \
  -p rc_enable:=true -p rc_actuation_enable:=true -p rc_ch4_enable:=$ch4 \
  -p auto_enable_default:=false "$@"
