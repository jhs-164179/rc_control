#!/usr/bin/env python3
"""Ground PC: observe onboard RC channels and rover arbitration status via ROS 2."""
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16MultiArray, String

class Monitor(Node):
    def __init__(self):
        super().__init__('ground_rc_monitor')
        self.channels = []
        self.status = {}
        self.create_subscription(Int16MultiArray, '/rover/rc/channels', self.on_ch, 10)
        self.create_subscription(String, '/rover/rc/status', self.on_status, 10)
        self.create_timer(0.5, self.report)

    def on_ch(self, msg):
        self.channels = list(msg.data)

    def on_status(self, msg):
        try:
            self.status = json.loads(msg.data)
        except (ValueError, TypeError):
            self.status = {'raw': msg.data}

    def report(self):
        ch = ' '.join(f'CH{i+1}={v}' for i, v in enumerate(self.channels[:8])) or '(신호 없음)'
        print(f'\r{ch:<85} | {str(self.status):<125}', end='', flush=True)

def main():
    rclpy.init()
    n = Monitor()
    try:
        rclpy.spin(n)
    finally:
        print()
        n.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
