#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
send_path.py — 시험용 합성 경로를 map [m] 로 한 번 발행한다

실제 플래너 대신 쓴다. 연속 곡선을 경로계획 출력처럼 약 5 cm 간격 점으로 찍어
nav_msgs/Path (frame 'map', 미터) 로 보낸다. 구독자가 생길 때까지 기다렸다가
딱 한 번만 보낸다. 경로계획 실제 출력 CSV 는 follow_test_paths_261011/send_path_csv.py.

  python3 send_path.py --shape straight
  python3 send_path.py --shape s_curve --x0 0 --y0 0 --yaw0 0
  (cusp / reverse 는 후진 경로라 전진만 하는 현재 추종기로는 따라가지 않는다)

좌표 인자(--x0 --y0 --yaw0)는 map [m, deg] 이고 경로가 이 자세에서 시작한다.
"""

from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path as PathMsg
from geometry_msgs.msg import PoseStamped


SHAPES = ("straight", "s_curve", "cusp", "reverse")


def shape_points(shape: str):
    """로컬 좌표 (시작 (0,0), 헤딩 +x) 의 1 cm 간격 곡선 [m]."""
    if shape == "straight":
        return [(i * 0.01, 0.0) for i in range(301)]
    if shape == "s_curve":
        return [(i * 0.01, 0.5 * math.sin(2 * math.pi * i * 0.01 / 3.0)) for i in range(451)]
    if shape == "cusp":
        pts = [(i * 0.01, 0.0) for i in range(151)]
        return pts + [(1.5 - math.sin(k * 0.01), 1.0 - math.cos(k * 0.01)) for k in range(1, 158)]
    if shape == "reverse":
        return [(-i * 0.01, 0.0) for i in range(201)]
    raise ValueError(shape)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--shape", choices=SHAPES, default="s_curve")
    ap.add_argument("--x0", type=float, default=0.0)
    ap.add_argument("--y0", type=float, default=0.0)
    ap.add_argument("--yaw0", type=float, default=0.0, help="[deg]")
    ap.add_argument("--topic", default="/planner/path")
    ap.add_argument("--frame", default="map")
    ap.add_argument("--ds", type=float, default=0.05, help="점 간격 [m]")
    ap.add_argument("--wait", type=float, default=10.0, help="구독자 대기 [s]")
    args, ros_args = ap.parse_known_args()

    c, s = math.cos(math.radians(args.yaw0)), math.sin(math.radians(args.yaw0))
    src = shape_points(args.shape)
    pts = [src[0]]
    for p in src[1:]:                      # 약 ds 간격으로 솎는다 (끝점은 항상 포함)
        if math.hypot(p[0] - pts[-1][0], p[1] - pts[-1][1]) >= args.ds - 1e-9 or p is src[-1]:
            pts.append(p)
    pts = [(args.x0 + c * lx - s * ly, args.y0 + s * lx + c * ly) for lx, ly in pts]

    rclpy.init(args=ros_args)
    node = Node("send_path")
    pub = node.create_publisher(PathMsg, args.topic, 10)
    t0 = time.monotonic()
    while pub.get_subscription_count() == 0 and time.monotonic() - t0 < args.wait:
        rclpy.spin_once(node, timeout_sec=0.1)
    if pub.get_subscription_count() == 0:
        node.get_logger().error(f"{args.topic} 구독자가 없습니다 ({args.wait:.0f} s).")
        rclpy.shutdown()
        return 1

    msg = PathMsg()
    msg.header.stamp = node.get_clock().now().to_msg()
    msg.header.frame_id = args.frame
    for x, y in pts:
        ps = PoseStamped()
        ps.header = msg.header
        ps.pose.position.x, ps.pose.position.y = float(x), float(y)
        ps.pose.orientation.w = 1.0
        msg.poses.append(ps)
    pub.publish(msg)
    time.sleep(0.5)          # 전송이 끝나기 전에 노드가 사라지지 않게
    node.get_logger().info(f"{args.shape}: {len(pts)} 점 (m) 발행 -> {args.topic}")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
