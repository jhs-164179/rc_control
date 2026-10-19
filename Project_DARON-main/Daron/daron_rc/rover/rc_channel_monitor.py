#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rc_channel_monitor.py — RC 4채널 터미널 진단 (보드·지상국 어디서나). 모터 지령은 내지 않는다.

  윗줄(덮어쓰기)  /rover/rc/channels  CH1~CH4 [µs] — rc_ppm_node 만 떠 있어도 나온다
  바뀔 때 한 줄  /rover/rc/status    source / switch / active·would 모드 / link / armed / reason
                                      — radxa.py 가 rc_enable:=true 로 떠 있어야 나온다

  python3 rc_channel_monitor.py
"""
import json
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String

NAMES = ('MODE', 'DRIVE', 'STEER', 'XM430')


class Monitor(Node):
    def __init__(self):
        super().__init__('rc_4ch_monitor')
        self.create_subscription(Int16MultiArray, '/rover/rc/channels', self._on_ch, 10)
        self.create_subscription(String, '/rover/rc/status', self._on_status, 10)
        self.last = ''

    def _on_ch(self, msg):
        a = list(msg.data)
        if len(a) >= 4:
            s = ' | '.join(f'CH{i + 1} {n}={a[i]:4d}' for i, n in enumerate(NAMES))
            extra = ' '.join(str(v) for v in a[4:])
            print(f'\r{s} us' + (f'  (CH5~ {extra})' if extra else '') + ' ' * 4, end='', flush=True)
        else:
            print('\rPPM 없음 / timeout' + ' ' * 70, end='', flush=True)

    def _on_status(self, msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        val = (f"source {d.get('source')}  switch {d.get('switch_request')}  "
               f"active {d.get('active_mode')}  would {d.get('would_mode')}  "
               f"link {d.get('rc_link')}  armed {d.get('rc_armed')}  auto_ready {d.get('auto_ready')}  "
               f"latched {d.get('latched')}  dry {not d.get('actuation')}  |  {d.get('reason')}")
        if val != self.last:
            print(f'\n[STATUS] {val}', flush=True)
            self.last = val


def main():
    rclpy.init()
    node = Monitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(main())
