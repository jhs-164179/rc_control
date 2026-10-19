#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
fake_detector — YOLO 대역. 검출 CSV(수동 어노테이션)를 영상 수신에 맞춰 검출 토픽으로 낸다. 작성 261001.

infra1 영상을 받아서, 그 stamp 가 CSV 에 있으면 latency_s 뒤에 /cda/detections 로 발행한다.
= "영상 받음 → 추론 → 발행" 흐름과 지연을 흉내. YOLO 노드는 같은 토픽·같은 JSON 을 내면 이 노드를 대체한다.
CSV 에 없는 프레임은 발행 안 함(publish_empty=true 면 빈 dets 로 발행 — 실제 YOLO 처럼 매 프레임).

CSV: stamp_ns,camera,x1,y1,x2,y2,score,class   (~/Daron/yolo/data/<bag>/annotations.csv)
출력 JSON: {"stamp_ns": .., "camera": "infra1", "source": "fake",
            "dets": [{"box": [x1, y1, x2, y2], "score": .., "class": ".."}]}
"""

import collections
import csv
import json
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String


class FakeDetector(Node):
    def __init__(self):
        super().__init__('fake_detector')
        P = self.declare_parameter
        path = P('csv', '').value
        self.latency = float(P('latency_s', 0.15).value)
        self.publish_empty = bool(P('publish_empty', False).value)
        image_topic = P('image_topic', '/camera/camera/infra1/image_rect_raw').value
        det_topic = P('det_topic', '/cda/detections').value
        if not path:
            raise ValueError('csv 파라미터가 비었습니다')
        self.table = collections.defaultdict(list)
        with open(path) as f:
            for r in csv.DictReader(f):
                if r.get('camera', 'infra1') != 'infra1':
                    continue
                self.table[int(r['stamp_ns'])].append(dict(
                    box=[float(r[k]) for k in ('x1', 'y1', 'x2', 'y2')],
                    score=float(r.get('score', 1.0)), **{'class': r.get('class', '')}))
        self.queue = collections.deque()          # (due_monotonic, json)
        self.n_img = self.n_pub = 0
        self.pub = self.create_publisher(String, det_topic, 50)
        self.create_subscription(Image, image_topic, self.on_image, qos_profile_sensor_data)
        self.create_timer(0.005, self.flush)
        self.get_logger().info(f'csv={path} ({len(self.table)} 프레임, '
                               f'{sum(map(len, self.table.values()))} 검출) latency={self.latency}s → {det_topic}')

    def on_image(self, m):
        self.n_img += 1
        t = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec
        dets = self.table.get(t)
        if dets is None and not self.publish_empty:
            return
        msg = json.dumps(dict(stamp_ns=t, camera='infra1', source='fake', dets=dets or []))
        self.queue.append((time.monotonic() + self.latency, msg))

    def flush(self):
        now = time.monotonic()
        while self.queue and self.queue[0][0] <= now:
            self.pub.publish(String(data=self.queue.popleft()[1]))
            self.n_pub += 1
            self.get_logger().info(f'검출 발행 {self.n_pub} (영상 {self.n_img}장째)')


def main():
    rclpy.init()
    node, rc = None, 0
    try:
        node = FakeDetector()
        rclpy.spin(node)
    except ValueError as exc:
        rclpy.logging.get_logger('fake_detector').error(str(exc))
        rc = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001
        # Ctrl+C 가 launch 와 터미널 양쪽에서 와서 context 가 먼저 내려가면 spin 이 RCLError
        # (failed to initialize wait set) 로 끝난다 — 260911 로그 §2 의 이중 shutdown. 종료 중이면 무시
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
