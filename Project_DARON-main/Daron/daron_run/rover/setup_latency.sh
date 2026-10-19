#!/bin/sh
# FTDI latency timer 를 1 ms 로 낮춘다.
#
# 기본값 16 ms 는 USB-시리얼 왕복 지연의 지배적 요인이다. 이 값을 그대로
# 두면 제어 루프가 60 Hz 근처에서 막히고, 외력 '해제' 검출이 한 박자 늦는다.
# 재부팅하면 되돌아가므로 udev 규칙으로 고정하려면 아래 주석을 참고.
#
#   sudo sh setup_latency.sh
set -e
found=0
for d in /sys/bus/usb-serial/devices/*; do
    [ -e "$d/latency_timer" ] || continue
    echo 1 > "$d/latency_timer"
    echo "$(basename "$d"): latency_timer = $(cat "$d/latency_timer") ms"
    found=1
done
[ "$found" = 1 ] || echo "USB 시리얼 장치를 찾지 못했습니다."

# 영구 적용:
#   /etc/udev/rules.d/99-ftdi-latency.rules 에 아래 한 줄
#   ACTION=="add", SUBSYSTEM=="usb-serial", DRIVER=="ftdi_sio", ATTR{latency_timer}="1"
#   sudo udevadm control --reload-rules && sudo udevadm trigger
