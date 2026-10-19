#!/bin/bash
# 비상정지 — 제어창(GUI)이 죽었거나 굳었을 때 다른 터미널에서 (261016, logs/DARON_문제분석_261015.md §10·S1)
#   1) 로버 자율 금지   /rover/auto_enable=false  — radxa 가 값을 유지한다(자율 rpm·조향 무시 → 감속 후 정지)
#   2) 추종기 끄기      /path_follower/enable=false
#   3) 경로계획 수동    /ui/mode=MANUAL           — 플래너가 빈 경로(정지)를 낸다
#   셋을 동시에 보낸 뒤, auto_enable 을 다시 켜는 발행자(console)가 있는지와 로버 drive_mode 를 보여 준다.
#   제어창은 enable·mode 를 바꿀 때만 보내므로(1 Hz 재발행 없음) 살아 있어도 이 값을 덮지 않는다.
# ⚠ 정지 = 자유회전(coast). 경사면이면 손으로 잡을 것.
# ⚠ console.py 는 매 주기 auto_enable 을 보낸다 — 떠 있으면 console 에서 P(AUTO PAUSED) 또는 console 종료.
# ⚠ 수동 조종(키보드) 지령은 이걸로 안 멈춘다 — 그 송신 창을 끄면 로버 텔레옵 워치독(0.5 s)이 세운다.
# 사용: Daron/daron_run/tools/estop.sh   (ROS 환경이 없으면 ~/slam_env.sh 를 읽는다 — DOMAIN 42)
# 해제(확인 뒤): console 의 P, 또는  ros2 topic pub -1 /rover/auto_enable std_msgs/msg/Bool "{data: true}"
set -u
if [ -z "${ROS_DISTRO:-}" ]; then
    set +u; source "$HOME/slam_env.sh" >/dev/null 2>&1; set -u
fi
echo "[estop] DOMAIN=${ROS_DOMAIN_ID:-0} RMW=${RMW_IMPLEMENTATION:-default}"

pub() {   # 토픽 타입 값 — 구독자를 기다렸다가(-w 1) 1 s 동안 10 회. 구독자가 6 s 안에 없으면 포기
    local out rc
    out=$(timeout 6 ros2 topic pub -w 1 -t 10 -r 10 "$1" "$2" "$3" 2>&1); rc=$?
    if [ $rc -eq 0 ]; then echo "  OK   $1 <- $3"
    elif [ $rc -eq 124 ]; then echo "  --   $1: 구독자 없음 (그 노드가 안 떠 있거나 링크 끊김)"
    else echo "  FAIL $1 (rc=$rc): $(echo "$out" | tail -1)"; fi
}
pub /rover/auto_enable std_msgs/msg/Bool "{data: false}" &
pub /path_follower/enable std_msgs/msg/Bool "{data: false}" &
pub /ui/mode std_msgs/msg/String "{data: MANUAL}" &
wait

echo "[estop] 확인"
# echo 는 --no-daemon: ros2 데몬이 죽어 있으면(!rclpy.ok()) echo 가 바로 실패한다 (261016 시험에서 겪음)
again=$(timeout 4 ros2 topic echo --no-daemon --spin-time 1 --once /rover/auto_enable std_msgs/msg/Bool 2>/dev/null | awk '/data:/ {print $2}')
if [ "$again" = "true" ]; then
    echo "  ⚠ 누군가 /rover/auto_enable=true 를 계속 보내는 중 (console?) — console 에서 P 또는 종료하고 다시 실행"
elif [ -n "$again" ]; then
    echo "  /rover/auto_enable 을 계속 보내는 발행자 있음 (값 $again)"
fi
diag=$(timeout 5 ros2 topic echo --no-daemon --spin-time 1 --once /rover/diagnostics diagnostic_msgs/msg/DiagnosticArray 2>/dev/null)
if [ -z "$diag" ]; then
    echo "  /rover/diagnostics 가 5 s 안에 안 옴 — 로버 링크 확인 (WiFi 끊김이면 로버 워치독이 세운다)"
else
    for k in drive_mode auto_enable drive_applied; do
        v=$(echo "$diag" | grep -A1 -E "key: '?$k'?\$" | awk '/value:/ {$1=""; print}' | head -1)
        echo "  $k =${v:- ?}"
    done
fi
