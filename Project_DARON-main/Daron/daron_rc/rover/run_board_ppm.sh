#!/usr/bin/env bash
# R8FM PPM -> /rover/rc/channels. 모터는 건드리지 않는다.
#   ./run_board_ppm.sh                     # gpiochip3:20, 8채널
#   PPM_CHANNELS=6 ./run_board_ppm.sh      # 수신기 PPM 채널 수가 다를 때
#   ./run_board_ppm.sh -p offset:=21       # 그 밖의 rc_ppm_node 파라미터
set -euo pipefail
source "$(dirname "$0")/rc_env.sh"
rc_refuse_if_running rc_ppm_node.py
command -v gpiomon >/dev/null || echo "[rc] ⚠ gpiomon 이 PATH 에 없습니다 (apt install gpiod). 노드는 재시도하며 기다립니다." >&2
exec "$PY" "$RC_DIR/rc_ppm_node.py" --ros-args \
  -p channels:=${PPM_CHANNELS:-8} "$@"
