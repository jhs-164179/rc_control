#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
cda_overlay — /cda/detections 의 bbox 를 같은 stamp 의 infra1 영상 위에 그려 Image 로 낸다. 작성 261005.

YOLO(SCNet) 노드는 JSON 만 내서 RViz 로는 검출이 안 보인다 → 보기 전용. 핀 계산에는 관여하지 않는다.
검출 메시지 1개 = 출력 영상 1장 (빈 dets 도 발행 → 추론이 도는 동안 영상이 끊기지 않음).

입력  image_topic (infra1 mono8), det_topic (std_msgs/String JSON)
출력  out_topic (sensor_msgs/Image bgr8, header = 원본 infra1 header)
그림  박스 색 = depth_decision_state
        confirmed_crater 초록 / indeterminate_pass 노랑 / advisory_pass 하늘 / rejected_flat 빨강 / 그 외 흰색
      라벨 = score, depth_m.  좌상단 = 검출 수, 추론 지연(영상 수신 → 검출 수신, 벽시계), 검출 Hz
"""

import array
import collections
import json
import time

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import String

COLORS = {  # BGR
    'confirmed_crater': (0, 220, 0),
    'indeterminate_pass': (0, 220, 255),
    'advisory_pass': (255, 200, 0),
    'rejected_flat': (0, 0, 255),
}


def stamp_ns(h):
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


class CdaOverlay(Node):
    def __init__(self):
        super().__init__('cda_overlay')
        P = self.declare_parameter
        image_topic = P('image_topic', '/camera/camera/infra1/image_rect_raw').value
        det_topic = P('det_topic', '/cda/detections').value
        out_topic = P('out_topic', '/cda/overlay').value
        self.buffer_s = float(P('buffer_s', 3.0).value)    # 영상 보관 (추론 지연 상한)

        self.images = collections.OrderedDict()            # stamp_ns -> (Image, recv_monotonic)
        self.det_times = collections.deque(maxlen=30)
        self.stats = collections.Counter()
        # 검출은 영상보다 늦게 오므로 영상 큐가 짧으면 짝이 될 영상을 놓친다 (sensor_data 기본 depth 5)
        img_qos = QoSProfile(depth=30, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, image_topic, self.on_image, img_qos)
        self.create_subscription(String, det_topic, self.on_det, 50)
        self.pub = self.create_publisher(Image, out_topic, qos_profile_sensor_data)
        self.create_timer(5.0, self.report)
        self.get_logger().info(f'{image_topic} + {det_topic} -> {out_topic}')

    def on_image(self, m):
        t = stamp_ns(m.header)
        self.images[t] = (m, time.monotonic())
        while self.images and next(iter(self.images)) < t - int(self.buffer_s * 1e9):
            self.images.popitem(last=False)

    def on_det(self, m):
        now = time.monotonic()
        try:
            d = json.loads(m.data)
            t = int(d['stamp_ns'])
        except (ValueError, KeyError, TypeError):
            self.stats['det_bad_json'] += 1
            return
        if t not in self.images:
            self.stats['no_image'] += 1                    # buffer_s 밖이거나 영상을 못 받음
            return
        img, t_img = self.images[t]
        if img.encoding != 'mono8':
            self.stats['bad_encoding'] += 1
            return
        self.det_times.append(now)
        mono = np.frombuffer(img.data, np.uint8).reshape(img.height, img.step)[:, :img.width]
        canvas = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)

        dets = d.get('dets') or []
        for det in dets:
            x1, y1, x2, y2 = (int(round(v)) for v in det['box'])
            state = det.get('depth_decision_state', '')
            color = COLORS.get(state, (255, 255, 255))
            cv2.rectangle(canvas, (x1, y1), (x2, y2), color, 2)
            dm = det.get('depth_m')
            label = f"{det.get('score', 0):.2f}" + (f' {dm:.2f}m' if dm is not None else '')
            cv2.putText(canvas, label, (x1, max(y1 - 6, 12)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv2.LINE_AA)

        hz = (len(self.det_times) - 1) / (self.det_times[-1] - self.det_times[0]) if len(self.det_times) > 1 else 0.0
        head = f'dets {len(dets)}  lat {(now - t_img) * 1000:.0f} ms  {hz:.1f} Hz'
        cv2.putText(canvas, head, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(canvas, head, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)

        out = Image()
        out.header = img.header
        out.height, out.width = canvas.shape[:2]
        out.encoding, out.step = 'bgr8', canvas.shape[1] * 3
        out.data = array.array('B', canvas.tobytes())   # bytes 를 그대로 넣으면 rclpy 가 원소마다 검사해서 영상 1장에 수십 ms
        self.pub.publish(out)
        self.stats['published'] += 1
        self.stats['with_dets'] += int(bool(dets))

    def report(self):
        self.get_logger().info(f'stats {dict(self.stats)} image_buf={len(self.images)}')


def main():
    rclpy.init()
    node = None
    try:
        node = CdaOverlay()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001
        # launch·터미널 이중 Ctrl+C 로 context 가 먼저 내려간 경우 (pin_node 와 같은 처리)
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
