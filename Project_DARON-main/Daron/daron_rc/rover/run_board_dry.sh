#!/usr/bin/env bash
# RC 중재 DRY — 모터 출력 없음. XM430 도 mock, L12·VESC 꺼짐, rc_actuation_enable=false.
# 판단 결과(would_mode / would_steer / would_drive / would_jog)는 /rover/rc/status 와 콘솔 RC 패널에서 본다.
# CH4 는 dry 에서만 기본으로 켠다 (출력이 없으니 극성 확인용).
#   ./run_board_dry.sh
#   ./run_board_dry.sh -p rc_ch3_reverse:=true      # 극성 바꿔 보기
set -euo pipefail
source "$(dirname "$0")/rc_env.sh"
rc_need_rover_dir
rc_refuse_if_running radxa.py
rc_refuse_safety_override "$@"

params=(
  -p "susp_profile:=$PROFILE"
  -p susp_mock:=true -p l12_enable:=false -p vesc_enable:=false
  -p rc_enable:=true -p rc_actuation_enable:=false -p rc_ch4_enable:=true
  -p auto_enable_default:=false
)
[[ -f "$PROFILE" ]] || { echo "[rc] 프로파일 없음 ($PROFILE) — mock 이라 기본값으로 진행"; params+=(-p susp_require_profile:=false); }
exec "$PY" "$RC_DIR/radxa.py" --ros-args "${params[@]}" "$@"
