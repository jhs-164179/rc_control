# rc_env.sh — run_board_*.sh 가 source 한다 (직접 실행하지 않음).
# ROS 도메인·RMW 기본값, rclpy 확인, 설치 위치 확인, 중복 실행 확인.

RC_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
PY=${PYTHON:-python3}

if ! "$PY" -c 'import rclpy' 2>/dev/null; then
  if [[ -f /opt/ros/humble/setup.bash ]]; then
    set +u; source /opt/ros/humble/setup.bash; set -u
  fi
  "$PY" -c 'import rclpy' 2>/dev/null || {
    echo "[rc] rclpy 를 import 하지 못했습니다. ROS 2 setup.bash 를 source 한 셸에서 실행하세요." >&2
    exit 1
  }
fi

# radxa.py 는 dxl_link·susp_*·l12_driver·vesc_driver 를 같은 폴더에서 찾는다.
rc_need_rover_dir() {
  if [[ ! -f "$RC_DIR/dxl_link.py" || ! -f "$RC_DIR/l12_driver.py" ]]; then
    echo "[rc] $RC_DIR 에 dxl_link.py / l12_driver.py 가 없습니다." >&2
    echo "     daron_rc/install_board.sh 로 daron_run/rover 에 설치한 뒤 그 폴더에서 실행하세요." >&2
    exit 1
  fi
}

# 같은 노드가 둘이면 지령·PPM 이 섞인다.
rc_refuse_if_running() {   # $1 = 스크립트 이름 패턴
  local pids
  pids=$(pgrep -f "[p]ython3? .*$1" || true)
  if [[ -n "$pids" ]]; then
    echo "[rc] $1 이 이미 실행 중입니다 (pid $(echo $pids)). 먼저 종료하세요." >&2
    exit 2
  fi
}

# ROS 는 같은 -p 가 두 번이면 뒤의 값을 쓴다. 스크립트가 정한 안전 관련 값을 뒤 인자로 덮지 못하게 한다
# (예: run_board_dry.sh -p rc_actuation_enable:=true 로 dry 가 실구동이 되는 것).
rc_refuse_safety_override() {
  local a
  for a in "$@"; do
    if [[ "$a" =~ ^(susp_mock|l12_enable|vesc_enable|rc_enable|rc_actuation_enable|rc_ch4_enable|susp_profile): ]]; then
      echo "[rc] '$a' 는 이 스크립트가 정하는 값이라 덮을 수 없습니다 (단계/스크립트를 바꾸세요. SUSP_PROFILE, RC_CH4 환경변수 참고)." >&2
      exit 2
    fi
  done
}

PROFILE=${SUSP_PROFILE:-$RC_DIR/profiles/rover.json}
echo "[rc] ROS_DOMAIN_ID=$ROS_DOMAIN_ID RMW=$RMW_IMPLEMENTATION dir=$RC_DIR"
