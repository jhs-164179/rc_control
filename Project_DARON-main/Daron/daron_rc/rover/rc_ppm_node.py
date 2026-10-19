#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rc_ppm_node.py — R8FM 수신기 PPM-Sum (Radxa GPIO) -> /rover/rc/channels. 작성 261015.

모니터만 한다. 모터는 건드리지 않는다 (판단·구동은 radxa.py 의 RC 중재).

  gpiomon(libgpiod 1.6.x) 로 상승 에지 타임스탬프를 받아 rc_core.PpmDecoder 로 프레임을 만든다.
      gpiomon -r -b -F '%s %n' <gpiochip> <offset>        (사용자 보드에서 1.6.3 형식 확인됨)
  - 정확히 channels 개 간격이 동기(sync_gap_us 초과) 사이에 들어온 프레임만 받는다.
    시작 직후의 반쪽 프레임, 노이즈 에지가 낀 프레임, 개수가 다른 프레임은 버린다
    (부분 프레임을 받으면 채널이 밀려 CH2 자리에 다른 값이 들어간다).
  - 20 Hz 로 마지막 프레임을 낸다. timeout_s 동안 새 프레임이 없으면 빈 배열 (radxa.py = 링크 없음).
  - gpiomon 이 죽으면 restart_s 뒤 다시 띄운다. 그동안은 빈 배열.
  - stats_s 마다 프레임/s · 버린 프레임 · 마지막 간격 개수를 로그로 찍는다.
    '간격 개수 6 (기대 8)' 처럼 나오면 수신기 PPM 채널 수에 맞춰 -p channels:=6.

발행
  /rover/rc/channels   std_msgs/Int16MultiArray   [CH1 .. CHn] µs. 빈 배열 = 신호 없음

실행 (보드 도커)
  ./run_board_ppm.sh
  python3 rc_ppm_node.py --ros-args -p gpiochip:=gpiochip3 -p offset:=20 -p channels:=8

⚠ R8FM 이 RF 를 잃어도 Failsafe 값으로 PPM 을 계속 내면 여기서는 '정상 프레임'으로 보인다.
  송신기를 꺼 보고 rc_channel_monitor.py 로 실제 출력을 확인할 것 (README.md 2단계).
"""
from __future__ import annotations

import os
import subprocess
import sys
import threading
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from rc_core import PpmDecoder  # noqa: E402


class PpmNode(Node):
    def __init__(self):
        super().__init__('rc_ppm')
        self.declare_parameters('', [
            ('gpiochip', 'gpiochip3'),
            ('offset', 20),
            ('channels', 8),              # 수신기 PPM 채널 수. 프레임 길이가 정확히 이 값이어야 받는다
            ('sync_gap_us', 3000.0),
            ('max_gap_us', 50000.0),      # 이보다 긴 무신호 뒤에는 끊기기 전 프레임을 버린다
            ('min_us', 800.0),
            ('max_us', 2200.0),
            ('timeout_s', 0.25),
            ('publish_hz', 20.0),
            ('gpiomon', 'gpiomon'),       # 실행 파일 (시험용 가짜 gpiomon 으로 바꿀 수 있다)
            ('restart_s', 1.0),
            ('stats_s', 5.0),
            ('topic', '/rover/rc/channels'),
        ])
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        self.decoder = PpmDecoder(int(g('channels')), float(g('sync_gap_us')),
                                  float(g('min_us')), float(g('max_us')), float(g('max_gap_us')))
        self.timeout_s = float(g('timeout_s'))
        self.pub = self.create_publisher(Int16MultiArray, str(g('topic')), 10)
        self.lock = threading.Lock()
        self.latest = []
        self.latest_t = 0.0
        self.stop_event = threading.Event()
        self.proc = None
        self._was_fresh = False
        self._stats = (time.monotonic(), 0, 0)
        self.reader = threading.Thread(target=self._reader_loop, daemon=True)
        self.reader.start()
        self.create_timer(1.0 / float(g('publish_hz')), self._publish)
        self.create_timer(float(g('stats_s')), self._log_stats)
        self.get_logger().info(
            f"PPM {g('gpiochip')}:{g('offset')}  채널 {g('channels')}  "
            f"동기 > {g('sync_gap_us'):.0f} µs  채널 {g('min_us'):.0f}~{g('max_us'):.0f} µs  "
            f"-> {g('topic')}")

    # ---------- gpiomon ----------
    def _reader_loop(self):
        g = lambda n: self.get_parameter(n).value  # noqa: E731
        cmd = [str(g('gpiomon')), '-r', '-b', '-F', '%s %n', str(g('gpiochip')), str(g('offset'))]
        restart = float(g('restart_s'))
        while not self.stop_event.is_set():
            try:
                self.proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                                             text=True, bufsize=1)
            except OSError as e:
                self.get_logger().error(f"gpiomon 실행 실패 ({' '.join(cmd)}): {e}. "
                                        f"libgpiod-tools(gpiod 패키지) 설치·PATH 확인",
                                        throttle_duration_sec=10.0)
                self.stop_event.wait(restart)
                continue
            self.decoder.reset()
            for line in self.proc.stdout:
                if self.stop_event.is_set():
                    break
                try:
                    sec, nsec = line.split()
                    t_ns = int(sec) * 1_000_000_000 + int(nsec)
                except ValueError:
                    continue
                frame = self.decoder.feed(t_ns)
                if frame is not None:
                    with self.lock:
                        self.latest = frame
                        self.latest_t = time.monotonic()
            if self.stop_event.is_set():
                break
            err = ''
            try:
                err = (self.proc.stderr.read() or '').strip()[-300:]
            except Exception:
                pass
            self.get_logger().error(
                f"gpiomon 종료 (code {self.proc.poll()}): {err or '(stderr 없음)'} — "
                f"GPIO 번호·권한(/dev/gpiochip*)·다른 프로세스 점유 확인. {restart:.1f} s 뒤 재시작")
            self.stop_event.wait(restart)

    # ---------- 발행 ----------
    def _publish(self):
        with self.lock:
            fresh = self.latest_t > 0 and time.monotonic() - self.latest_t <= self.timeout_s
            vals = list(self.latest) if fresh else []
        if fresh != self._was_fresh:
            # 호출 위치마다 severity 가 고정이어야 한다 (rclpy: 'Logger severity cannot be changed
            # between calls' — 한 줄에서 info/warn 을 골라 부르면 노드가 죽는다)
            if fresh:
                self.get_logger().info('PPM 수신 시작')
            else:
                self.get_logger().warn(f'PPM 끊김 ({self.timeout_s:.2f} s 동안 정상 프레임 없음)')
            self._was_fresh = fresh
        self.pub.publish(Int16MultiArray(data=vals))

    def _log_stats(self):
        t0, f0, r0 = self._stats
        now, d = time.monotonic(), self.decoder
        dt = max(1e-6, now - t0)
        self._stats = (now, d.frames, d.rejected)
        with self.lock:
            ch = ' '.join(str(v) for v in self.latest[:4]) if self._was_fresh else '-'
        msg = (f"프레임 {(d.frames - f0) / dt:.1f}/s  버림 {d.rejected - r0}  "
               f"간격 개수 {d.last_len} (기대 {d.channels})  CH1~4 {ch}")
        if d.last_len is not None and d.last_len != d.channels and d.frames == f0:
            self.get_logger().warn(msg + f"  ← 수신기 채널 수가 다르면 -p channels:={d.last_len}")
        else:
            self.get_logger().info(msg)

    def destroy_node(self):
        self.stop_event.set()
        p = self.proc
        if p is not None and p.poll() is None:
            p.terminate()
            try:
                p.wait(timeout=1.0)
            except subprocess.TimeoutExpired:
                p.kill()
        super().destroy_node()


def main(argv=None):
    rclpy.init(args=argv)
    node = PpmNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():           # Ctrl+C 로 이미 내려갔으면 다시 부르지 않는다
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
