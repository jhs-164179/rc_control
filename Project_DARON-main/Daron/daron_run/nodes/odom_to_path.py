#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
odom_to_path — nav_msgs/Odometry 여러 토픽을 각각 nav_msgs/Path 로 누적해 재발행 (RViz 선 그리기용).
배치 경로: ~/daron_ros/odom_to_path.py
작성: 2026-09-21

d405_cuvslam_viz.py 가 /slam/odometry, /odometry/filtered 를 vo_path / slam_path 로 내는 것과 같은
방식(최대 max_poses, 넘으면 앞에서 삭제). 여기서는 INS 단독(/ins/odometry)과 recalib EKF
(/odometry/filtered_recalib)를 다룬다. 추정에는 관여하지 않는다.
[261005] 포즈는 메시지마다 쌓고 발행은 publish_rate_hz(기본 5, 벽시계)로 묶는다. odom 마다(30~100 Hz) 누적 Path
전체를 보내면 RViz Humble 의 tf2_ros 교착(Path 수신마다 waitForTransform 잠금) 확률과 직렬화 부하가 커진다.
0 이면 예전처럼 메시지마다 발행.

사용 예:
  /usr/bin/python3 ~/daron_ros/odom_to_path.py --ros-args \
      -p odom_topics:="[/ins/odometry, /odometry/filtered_recalib]" \
      -p path_topics:="[/ins/path, /visual_slam/tracking/recalib_path]"
"""

import sys

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.clock import Clock
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path


class OdomToPath(Node):
    def __init__(self):
        super().__init__('odom_to_path')
        self.declare_parameter('odom_topics', ['/ins/odometry', '/odometry/filtered_recalib'])
        self.declare_parameter('path_topics', ['/ins/path', '/visual_slam/tracking/recalib_path'])
        self.declare_parameter('max_poses', 20000)
        self.declare_parameter('publish_rate_hz', 5.0)
        odom_topics = list(self.get_parameter('odom_topics').value)
        path_topics = list(self.get_parameter('path_topics').value)
        if len(odom_topics) != len(path_topics) or not odom_topics:
            raise ValueError(f'odom_topics/path_topics 길이가 다르거나 비었습니다: {odom_topics} / {path_topics}')
        self._max = int(self.get_parameter('max_poses').value)
        self._paths = {}
        self._dirty = set()
        rate = float(self.get_parameter('publish_rate_hz').value)
        self._throttle = rate > 0.0
        if self._throttle:   # 벽시계 — use_sim_time 이어도 재생 끝(/clock 정지) 뒤 마지막 경로까지 나간다
            self.create_timer(1.0 / rate, self._flush, clock=Clock())
        for ot, pt in zip(odom_topics, path_topics):
            path = Path()
            pub = self.create_publisher(Path, pt, 1)
            self._paths[ot] = (path, pub)
            self.create_subscription(Odometry, ot, lambda m, ot=ot: self._on_odom(ot, m), 10)
            self.get_logger().info(f'odom_to_path: {ot} -> {pt}')

    def _on_odom(self, topic, msg: Odometry):
        path, pub = self._paths[topic]
        path.header = msg.header
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose = msg.pose.pose
        path.poses.append(ps)
        if len(path.poses) > self._max:
            del path.poses[: len(path.poses) - self._max]
        if self._throttle:
            self._dirty.add(topic)
        else:
            pub.publish(path)

    def _flush(self):
        dirty, self._dirty = self._dirty, set()
        for topic in dirty:
            path, pub = self._paths[topic]
            pub.publish(path)


def main():
    rclpy.init()
    node = None
    rc = 0
    try:
        node = OdomToPath()
        rclpy.spin(node)
    except ValueError as exc:
        rclpy.logging.get_logger('odom_to_path').error(str(exc))
        rc = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
