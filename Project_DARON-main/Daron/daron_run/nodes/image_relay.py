#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
image_relay — RADXA 카메라 토픽을 WiFi 로 "한 번만" 받아 지상국 안에서 원래 이름으로 재발행.
배치 경로: ~/Daron/daron_run/nodes/image_relay.py
작성: 2026-10-08
환경: ~/slam_env.sh (시스템 python3 /usr/bin/python3, ROS Humble, CycloneDDS, DOMAIN 42)

[왜 필요한가] (261008 원격 실측)
  지상국에서 원본 영상을 구독하는 프로세스가 늘수록 WiFi 로 같은 영상이 여러 벌 오갔다:
      infra1 구독 1개(cuVSLAM)   → 30 Hz,  stamp 지연 0.19~0.29 s
      infra1 구독 4개(+GUI 3개)  → 5.4 Hz, 0.74~0.86 s
      + depth(구독 5개)          → infra1 6 Hz / depth 3 Hz, /slam/odometry_h 1.55 s
  지연 > 0.5 s 면 health_monitor 가 SLAM 을 dead(FAIL) 로 판정 → EKF-C 는 IMU 만으로 흐르고
  live_tsdf 는 health 게이트로 depth 를 전부 버린다(맵 안 쌓임).
  (CycloneDDS 는 WiFi 인터페이스에서 데이터 멀티캐스트를 안 쓰는 것으로 추정 — 원격 구독
   프로세스마다 유니캐스트 한 벌. 위 수치가 그 패턴과 맞는다.)

[구조]
  RADXA:   rs_launch.py ... camera_namespace:=radxa   → /radxa/camera/{infra1,infra2,depth}/...
  지상국:  이 노드 하나만 /radxa/camera/... 구독 (WiFi 1벌)
           → /camera/camera/... 로 재발행 (지상국 안 루프백)
  기존 노드(cuVSLAM, live_tsdf, GUI, ...)는 토픽 이름 그대로라 수정 없음.
  RADXA 가 camera_namespace 없이(옛 명령) 뜨면 이 노드는 받을 게 없어 아무것도 안 낸다
  — 기존처럼 직접 수신으로 동작하고, 기동 점검이 경고만 찍는다(발행자 이중화 없음).

[처리]
  raw=True 로 직렬화 바이트를 받아 그대로 publish (역직렬화·복사 없음). header.stamp,
  frame_id 등 메시지 내용은 한 바이트도 안 바뀐다.
  지연 통계용으로 스트림당 1초에 한 번만 역직렬화해 stamp 를 본다.

[QoS]
  입력: BEST_EFFORT / KEEP_LAST in_depth(2) — WiFi 에서 재전송 대기열이 쌓이면 지연이 커지므로
        늦은 프레임은 버리고 최신 것을 받는다. (cuVSLAM 이 원래 BEST_EFFORT 로 직접 받던 것과 동일)
        in_reliability:=reliable 로 바꿀 수 있다.
  출력: RELIABLE / VOLATILE / KEEP_LAST 5 — RADXA realsense 발행자와 같은 QoS
        (260911 remote_check 실측 REL/VOL). BEST_EFFORT·RELIABLE 구독자 모두 붙는다.
"""

import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CameraInfo, Image
# 261008_jhs 추가
# import fire
# import zstandard as zstd
# import numpy as np

# DEPTH_ZSTD_TOPIC = "/camera/transport/depth_zstd"
################

DEFAULT_STREAMS = [
    'infra1/image_rect_raw', 'infra1/camera_info',
    'infra2/image_rect_raw', 'infra2/camera_info',
    'depth/image_rect_raw', 'depth/camera_info',
]


def _msg_type(stream):
    return CameraInfo if stream.endswith('camera_info') else Image


class ImageRelay(Node):
    def __init__(self):
        super().__init__('image_relay')
        P = self.declare_parameter
        self.in_prefix = P('in_prefix', '/radxa/camera').value.rstrip('/')
        self.out_prefix = P('out_prefix', '/camera/camera').value.rstrip('/')
        streams = P('streams', DEFAULT_STREAMS).value
        in_rel = P('in_reliability', 'best_effort').value.strip().lower()
        in_depth = int(P('in_depth', 2).value)
        self.report_s = float(P('report_s', 5.0).value)
        check_s = float(P('check_after_s', 5.0).value)

        if in_rel not in ('best_effort', 'reliable'):
            raise ValueError(f"in_reliability 는 best_effort/reliable (받은 값: {in_rel!r})")
        qos_in = QoSProfile(
            reliability=(ReliabilityPolicy.BEST_EFFORT if in_rel == 'best_effort'
                         else ReliabilityPolicy.RELIABLE),
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=max(1, in_depth))
        qos_out = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=5)

        self.s = {}
        for st in streams:
            st = st.strip('/')
            mt = _msg_type(st)
            src, dst = f'{self.in_prefix}/{st}', f'{self.out_prefix}/{st}'
            pub = self.create_publisher(mt, dst, qos_out)
            self.s[st] = dict(src=src, dst=dst, type=mt, pub=pub, n=0, n_last=0,
                              age_t=0.0, ages=[], first=True)
            self.create_subscription(mt, src, lambda raw, st=st: self.on_msg(st, raw),
                                     qos_in, raw=True)

        self.get_logger().info(
            f'image_relay: {self.in_prefix}/* -> {self.out_prefix}/* ({len(self.s)}개 스트림, '
            f'입력 {in_rel.upper()} depth {in_depth} / 출력 RELIABLE VOLATILE 5)')
        for st, d in self.s.items():
            self.get_logger().info(f'  {d["src"]}  ->  {d["dst"]}')

        self._t_last = time.monotonic()
        self.create_timer(self.report_s, self.report)
        self._check_timer = self.create_timer(check_s, self.check_once)

    def on_msg(self, st, raw):
        d = self.s[st]
        d['pub'].publish(raw)
        d['n'] += 1
        now = time.monotonic()
        if d['first'] or now - d['age_t'] >= 1.0:
            d['age_t'] = now
            m = deserialize_message(raw, d['type'])
            stamp = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
            age = self.get_clock().now().nanoseconds * 1e-9 - stamp
            d['ages'].append(age)
            if d['first']:
                d['first'] = False
                self.get_logger().info(
                    f'첫 메시지 중계: {st} frame_id={m.header.frame_id!r} 지연 {age * 1e3:+.0f} ms')

    def report(self):
        now = time.monotonic()
        dt = max(1e-6, now - self._t_last)
        self._t_last = now
        parts = []
        for st, d in self.s.items():
            if st.endswith('camera_info'):
                d['n_last'] = d['n']
                d['ages'].clear()
                continue
            hz = (d['n'] - d['n_last']) / dt
            d['n_last'] = d['n']
            age = (f'{sum(d["ages"]) / len(d["ages"]) * 1e3:.0f}ms' if d['ages'] else '-')
            d['ages'].clear()
            parts.append(f'{st.split("/")[0]} {hz:4.1f}Hz {age}')
        self.get_logger().info('[relay] ' + ' | '.join(parts) + '  (지연 = 지금 - header.stamp)')

    def check_once(self):
        self._check_timer.cancel()
        for st, d in self.s.items():
            if not st.endswith('image_rect_raw'):
                continue
            n_in = self.count_publishers(d['src'])
            others = [i for i in self.get_publishers_info_by_topic(d['dst'])
                      if i.node_name != self.get_name()]
            if n_in == 0 and others:
                self.get_logger().warning(
                    f'{d["src"]} 발행자 없음, {d["dst"]} 는 다른 노드({others[0].node_namespace}/'
                    f'{others[0].node_name})가 직접 발행 중 — RADXA 카메라를 camera_namespace:=radxa '
                    f'없이 띄운 것 같습니다. 중계 없이 직접 수신으로 동작합니다(WiFi 로 구독자 수만큼 오감).')
            elif n_in == 0:
                self.get_logger().warning(f'{d["src"]} 발행자 없음 (RADXA 카메라 미기동 또는 해당 스트림 꺼짐)')
            elif others:
                self.get_logger().warning(
                    f'{d["dst"]} 에 이 노드 말고 발행자가 또 있음({others[0].node_namespace}/'
                    f'{others[0].node_name}) — 같은 영상이 두 번 들어갑니다. 하나를 끄세요.')


    # 261008_jhs 추가
    # def decode_zstd_depth(self, msg):
    #     raw, metadata = self.decompress_zstd(msg)

    #     if metadata["encoding"] not in ["16UC1", "mono16"]:
    #         raise ValueError(f"Unexpected Zstd depth encoding: {metadata['encoding']}")

    #     dtype = np.dtype(">u2" if metadata["bigendian"] else "<u2")
    #     row_elements = metadata["step"] // dtype.itemsize
    #     expected = metadata["height"] * row_elements

    #     depth = np.frombuffer(
    #         raw, dtype=dtype, count=expected
    #     ).reshape(metadata["height"], row_elements)

    #     return depth[:, :metadata["width"]].astype(np.uint16, copy=True)

    
    ################


def main():
    rclpy.init()
    node = ImageRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
