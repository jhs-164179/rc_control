#!/bin/bash
# RADXA 원격 기동 (261015). 지상국에서 한 번에 ssh → docker exec ros2 → 아래 4개를 띄운다.
#   R1 카메라 / R1.5 depth zstd sender (R1 의 depth 토픽이 보이면 시작) / R2 CV7 IMU / R3 로버 노드
# 새 터미널 창이 열리고, tmux 세션 'daron' 한 화면에 4분할로 보인다.
# 사용:  Daron/daron_run/tools/radxa_up.sh          기동. 이미 떠 있으면 그 세션에 다시 붙기만 한다
#        Daron/daron_run/tools/radxa_up.sh stop     전부 Ctrl+C → 로버 정상 종료 순서를 기다린 뒤 세션을 닫는다
#        Daron/daron_run/tools/radxa_up.sh --here   새 창을 열지 않고 지금 터미널에서
#   - 접속 정보는 <repo>/secrets/radxa.env (git 에 안 올라감). 없으면 radxa.env.example 을 복사해 채운다.
#   - 터미널 창을 닫아도 tmux 세션과 원격 노드는 계속 돈다. 다시 실행하면 붙는다. 창에서 빠지기만: Ctrl+b d
#   - 끌 때는 창을 닫지 말고 stop. 그래야 radxa.py 가 구동 0 → VESC OFF → L12 중립 → 토크 유지 순서로 내려간다.
#   - 첫 ssh 연결을 4개 창이 같이 쓴다(ControlMaster). 비밀번호는 한 번만 묻고, 무선이 끊기면 4개 창이 같이 끊긴다.
#     docker exec 는 클라이언트가 끊겨도 컨테이너 안 프로세스가 남을 수 있어서, 기동 전에 남은 노드를 검사한다.
#   - 기동 전 RADXA 시계 오프셋을 잰다. CLOCK_MAX_MS(기본 50) 를 넘으면 CLOCK_WAIT_S(기본 60) 동안 동기화를 기다리고,
#     그래도 어긋나 있으면 기동하지 않는다. 지상국 EKF·health_monitor 가 RADXA stamp 를 지상국 시계로 판정해서
#     (sensor_timeout 0.2 s) 어긋난 채로 띄우면 EKF 가 에러 없이 출력을 멈춘다. 건너뛰기: RADXA_UP_SKIP_CLOCK=1
set -u
HERE=$(cd "$(dirname "$0")" && pwd)
SELF="$HERE/$(basename "$0")"
REPO=$(cd "$HERE/../../.." && pwd)
ENV_FILE="$REPO/secrets/radxa.env"
SESSION=daron
# 컨테이너 안에서 기동 검사·종료에 쓰는 프로세스 패턴 (첫 글자 [] 는 pkill 이 자기 셸을 잡지 않게)
# radxa_rc*.py (daron_rc, RADXA 에만 있음) 는 radxa.py 의 RC 판이라 같은 VESC·XM430·PWM 을 잡는다 — 같이 뜨면 안 된다
PROCS='[r]s_launch\.py|[r]ealsense2_camera_node|[d]epth_zstd_sender|[m]icrostrain_launch\.py|[m]icrostrain_inertial_driver|[r]adxa(_rc[a-z0-9_]*)?\.py|[r]c_ppm_node\.py'

fail() {
    echo "[radxa_up] $*"
    [ "${RADXA_UP_PAUSE:-0}" = 1 ] && read -r -p "Enter 로 닫기 "
    exit 1
}

[ -f "$ENV_FILE" ] || fail "$ENV_FILE 없음 — cp secrets/radxa.env.example secrets/radxa.env 후 채우세요"
# shellcheck source=/dev/null
. "$ENV_FILE"
: "${RADXA_HOST:?radxa.env 에 RADXA_HOST 필요}" "${RADXA_USER:?radxa.env 에 RADXA_USER 필요}"
CONTAINER=${DOCKER_CONTAINER:-ros2}
ROVER_DIR=${ROVER_DIR:-/workspace/daron_run/rover}
SUSP_PROFILE=${SUSP_PROFILE:-profiles/rover.json}
TARGET="$RADXA_USER@$RADXA_HOST"
SSH_OPTS=(-o ConnectTimeout=5 -o ServerAliveInterval=5 -o ServerAliveCountMax=3
          -o ControlMaster=auto -o "ControlPath=$HOME/.ssh/cm-daron-%C" -o ControlPersist=600)

rssh() {   # 비밀번호가 있으면 sshpass, 없으면 키 인증 / 직접 입력
    if [ -n "${RADXA_SSH_PASS:-}" ]; then
        command -v sshpass >/dev/null || fail "RADXA_SSH_PASS 를 쓰려면 sshpass 필요: sudo apt install sshpass (아니면 비워 두고 직접 입력)"
        SSHPASS="$RADXA_SSH_PASS" sshpass -e ssh "${SSH_OPTS[@]}" "$@"
    else
        ssh "${SSH_OPTS[@]}" "$@"
    fi
}

# 컨테이너 안에서 돌릴 명령. 손으로 칠 때와 같게 대화형 bash(-i, .bashrc 읽음)로 돌린다
pane_cmd() {
    case $1 in
    R1) cat <<'EOF'
ros2 launch realsense2_camera rs_launch.py camera_namespace:=radxa \
  enable_infra1:=true enable_infra2:=true enable_depth:=true enable_color:=false \
  depth_module.infra_profile:=848x480x30 depth_module.depth_profile:=848x480x30 \
  depth_module.enable_auto_exposure:=false depth_module.exposure:=8500 depth_module.gain:=16
EOF
    ;;
    R1.5) cat <<'EOF'
echo '[R1.5] /radxa/camera/depth/image_rect_raw 가 보일 때까지 대기 (R1 이 떠야 시작)'
until ros2 topic list 2>/dev/null | grep -qx /radxa/camera/depth/image_rect_raw; do sleep 2; done
ros2 run depth_transport depth_zstd_sender --ros-args \
  -r __node:=depth_zstd_sender \
  -p input_topic:=/radxa/camera/depth/image_rect_raw \
  -p output_topic:=/camera/transport/depth_zstd \
  -p zstd_level:=1 \
  -p max_fps:=30.0 \
  -p preview_enabled:=false
EOF
    ;;
    R2) cat <<'EOF'
source /opt/ros/humble/setup.bash
ros2 launch microstrain_inertial_driver microstrain_launch.py \
  params_file:=/workspace/daron_cv7.yml namespace:=/ configure:=true activate:=true
EOF
    ;;
    R3) cat <<EOF
cd $ROVER_DIR || exit 1
if [ "\$(id -u)" = 0 ]; then sh setup_latency.sh; else sudo sh setup_latency.sh; fi \
  || echo '[R3] ⚠ latency 설정 실패 — 서스펜션 루프가 60 Hz 근처로 떨어질 수 있음'
python radxa.py --ros-args -p susp_profile:=$SUSP_PROFILE
EOF
    ;;
    esac
}

# tmux 창 하나: ssh → docker exec → 명령. 명령이 끝나도 컨테이너 셸로 남아 로그를 볼 수 있다
run_pane() {
    local name=$1 delay=$2 body
    body="$(pane_cmd "$name")
echo; echo '[$name] 끝남 — 이 창은 컨테이너 셸로 남음'
exec bash"
    printf '\033]2;%s\033\\' "$name"
    echo "[$name] $TARGET → docker $CONTAINER"
    sleep "$delay"   # Zero3 에 docker exec 4개가 한꺼번에 몰리지 않게
    # base64 로 넘겨 원격 셸(sh/bash) 따옴표 처리를 피한다
    rssh -t "$TARGET" "docker exec -it $CONTAINER bash -ic \"\$(echo $(printf '%s' "$body" | base64 -w0) | base64 -d)\""
    local rc=$?
    echo; echo "[$name] ssh 끊김 (코드 $rc) — 원격 노드는 남아 있을 수 있음. 정리: $SELF stop"
    exec bash
}

remote_procs() { rssh "$TARGET" "docker exec $CONTAINER pgrep -af '$PROCS'" 2>/dev/null; }

# RADXA − 지상국 시계 오프셋 [ms] 과 최소 왕복 [ms] 를 한 줄로. 연결 하나로 20 번 왕복(NTP 방식),
# 왕복이 짧은 표본 8 개의 중앙값. ssh 연결 수립 시간이 섞이면 수백 ms 틀린다(261015 실측) — 그래서 한 연결 안에서 잰다.
# 열린 ControlMaster 연결을 재사용하므로 rssh true 뒤에 부른다. 컨테이너는 호스트와 같은 시계라 호스트에서 잰다.
clock_offset() {
    /usr/bin/python3 - "${SSH_OPTS[@]}" "$TARGET" <<'EOF'
import statistics, subprocess, sys, time
remote = 'python3 -u -c "import sys,time\nfor l in sys.stdin: print(repr(time.time()), flush=True)"'
p = subprocess.Popen(['ssh', *sys.argv[1:], remote], stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True)
res = []
try:
    for _ in range(20):
        t0 = time.time(); p.stdin.write('x\n'); p.stdin.flush(); r = float(p.stdout.readline()); t1 = time.time()
        res.append(((t1 - t0) * 1e3, (r - (t0 + t1) / 2) * 1e3)); time.sleep(0.02)
except (ValueError, BrokenPipeError):
    sys.exit(1)
finally:
    p.stdin.close(); p.wait()
res.sort()
print(f"{statistics.median(o for _, o in res[:8]):.1f} {res[0][0]:.1f}")
EOF
}

check_clock() {
    [ "${RADXA_UP_SKIP_CLOCK:-0}" = 1 ] && { echo "[radxa_up] ⚠ 시계 점검 건너뜀 (RADXA_UP_SKIP_CLOCK=1)"; return; }
    local max=${CLOCK_MAX_MS:-50} wait_s=${CLOCK_WAIT_S:-60} m off rtt t_end=$((SECONDS + ${CLOCK_WAIT_S:-60}))
    while :; do
        m=$(clock_offset) || fail "시계 오프셋 측정 실패 (RADXA 에 python3 필요). 건너뛰려면 RADXA_UP_SKIP_CLOCK=1"
        read -r off rtt <<< "$m"
        if awk -v o="$off" -v m="$max" 'BEGIN { exit !(o <= m && o >= -m) }'; then
            echo "[radxa_up] 시계 OK — RADXA − 지상국 ${off} ms (왕복 ${rtt} ms, 허용 ±${max} ms)"
            return
        fi
        if [ $SECONDS -ge $t_end ]; then
            fail "RADXA 시계가 ${off} ms 어긋남 (허용 ±${max} ms, ${wait_s} s 기다림). 이대로 띄우면 EKF 가 RADXA 데이터를
  옛것으로 보고 에러 없이 출력을 멈춘다. RADXA 의 NTP 가 지상국을 보는지 확인:
    ssh $TARGET timedatectl timesync-status      (Server: 지상국 IP 여야 함)
    chronyc clients                              (지상국에서, RADXA 가 보여야 함)
  알고도 띄우려면: RADXA_UP_SKIP_CLOCK=1 $SELF"
        fi
        echo "[radxa_up] RADXA 시계 ${off} ms 어긋남 — 동기화 기다리는 중 ($((t_end - SECONDS)) s 남음)"
        sleep 3
    done
}

do_stop() {
    if tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "[radxa_up] 모든 창에 Ctrl+C"
        for p in $(tmux list-panes -s -t "$SESSION" -F '#{pane_id}'); do tmux send-keys -t "$p" C-c; done
    fi
    rssh "$TARGET" true || fail "$TARGET 접속 실패 — 원격 노드 상태 확인 못 함"
    local left
    for _ in $(seq 15); do   # radxa.py 정상 종료 순서 기다림 (최대 15 s)
        left=$(remote_procs)
        [ -z "$left" ] && break
        sleep 1
    done
    if [ -n "$left" ]; then   # ssh 가 끊겨 고아가 된 노드: 직접 SIGINT
        echo "[radxa_up] 남은 노드에 SIGINT:"; echo "$left"
        rssh "$TARGET" "docker exec $CONTAINER pkill -INT -f '$PROCS'"
        for _ in $(seq 15); do left=$(remote_procs); [ -z "$left" ] && break; sleep 1; done
    fi
    tmux kill-session -t "$SESSION" 2>/dev/null
    ssh "${SSH_OPTS[@]}" -O exit "$TARGET" 2>/dev/null
    if [ -n "$left" ]; then
        echo "[radxa_up] ⚠ 아직 살아 있음 (강제 종료는 하지 않음 — 확인 후 직접):"; echo "$left"
        exit 1
    fi
    echo "[radxa_up] 원격 노드 전부 종료"
}

do_up() {
    if ! tmux has-session -t "$SESSION" 2>/dev/null; then
        echo "[radxa_up] $TARGET 접속 확인"
        rssh "$TARGET" true || fail "$TARGET 접속 실패 — RADXA 전원·네트워크·ssh 확인"
        rssh "$TARGET" "docker ps --format '{{.Names}}'" | grep -qx "$CONTAINER" \
            || fail "컨테이너 '$CONTAINER' 가 안 떠 있음 — RADXA 에서 docker start $CONTAINER"
        check_clock
        local left; left=$(remote_procs)
        [ -z "$left" ] || fail "컨테이너 안에 이전 노드가 아직 돌고 있음 (중복 기동 방지):
$left
→ $SELF stop 후 다시"
        tmux new-session -d -s "$SESSION" -n radxa -x 240 -y 60 "$SELF _pane R1 0"
        tmux split-window -t "$SESSION" "$SELF _pane R1.5 1"
        tmux split-window -t "$SESSION" "$SELF _pane R2 2"
        tmux split-window -t "$SESSION" "$SELF _pane R3 3"
        tmux select-layout -t "$SESSION" tiled
        tmux set -t "$SESSION" mouse on >/dev/null
        tmux set -w -t "$SESSION" pane-border-status top >/dev/null
        tmux set -w -t "$SESSION" pane-border-format ' #{pane_title} ' >/dev/null
    fi
    if [ -n "${TMUX:-}" ]; then exec tmux switch-client -t "$SESSION"; fi
    exec tmux attach -t "$SESSION"
}

case ${1:-} in
_pane) run_pane "$2" "$3" ;;
stop)  do_stop ;;
--here) do_up ;;
"")
    if [ -n "${DISPLAY:-}" ] && command -v gnome-terminal >/dev/null; then
        exec gnome-terminal --title "DARON RADXA" --maximize -- env RADXA_UP_PAUSE=1 "$SELF" --here
    fi
    do_up ;;
*) echo "사용: $(basename "$0") [stop | --here]"; exit 1 ;;
esac
