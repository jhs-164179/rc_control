#!/usr/bin/env python3
"""로버 위치 중계 (261007): TF odom -> base_link (EKF-C) 를 현성님 추종기가 받는 PoseStamped 로 내보낸다.

  출력  /localization/pose   geometry_msgs/PoseStamped
        frame = odom_frame 파라미터 (261008 부터 기본 map), 단위 m (추종기 pose_in_pixels:=false)
        위치 = 경로계획 기준점 = 두 축 가운데 (base_link = 뒷축에서 앞으로 wheelbase/2 = 0.17 m)
        자세 = base_link 자세 그대로 (yaw 는 ROS 표준: x 축 기준 반시계 +)
        stamp = 그 TF 의 시각 (추종기가 지연 보정에 씀)
  TF 가 max_age_s(0.5 s) 넘게 오래됐거나 없으면 내지 않는다 → 추종기 pose_timeout 으로 정지.

왜 따로 노드인가: /planner/path 의 점은 두 축 가운데 점의 경로라서, 추종기가 같은 점의 위치를 받아야 횡오차가 맞다.
추종기가 base_link 를 그대로 쓰면 곡선에서 경로 안쪽/바깥쪽으로 0.17 m x 헤딩오차 만큼 어긋난다.

실행 (우분투 지상국, 파이프라인 또는 가상 로버가 TF 를 내는 상태):
  source ~/Daron/slam_env.sh
  ~/venv/daron3d/bin/python ~/daron-nav/daron_planner/ros/pose_relay.py
  (기준점을 base_link 로: -p ref:=base)
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
import planner_core as P          # noqa: E402

import rclpy                                          # noqa: E402
from rclpy.node import Node                           # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy   # noqa: E402
from rclpy.duration import Duration                   # noqa: E402
from rclpy.time import Time                           # noqa: E402
from geometry_msgs.msg import PoseStamped             # noqa: E402
from tf2_ros import Buffer, TransformListener         # noqa: E402


def stamp_s(st):
    return st.sec + st.nanosec * 1e-9


class PoseRelay(Node):
    def __init__(self):
        super().__init__('pose_relay')
        d = self.declare_parameter
        # [261008] 기본 map: 플래너가 map 좌표(odom_frame:=map, daron_3dmap 기본)로 경로를 내므로 위치도 같은 좌표로.
        #          옛 동작(odom)은 -p odom_frame:=odom
        d('odom_frame', 'map'); d('base_frame', 'base_link'); d('topic', '/localization/pose')
        d('rate_hz', 20.0); d('ref', 'mid'); d('mid_from_base_m', P.RoverParams().mid_from_base); d('max_age_s', 0.5)
        g = lambda n: self.get_parameter(n).value
        self.odom, self.base = g('odom_frame'), g('base_frame')
        self.off = float(g('mid_from_base_m')) if g('ref') == 'mid' else 0.0
        self.max_age = float(g('max_age_s'))
        self.tfbuf = Buffer(cache_time=Duration(seconds=5.0))
        self.tfl = TransformListener(self.tfbuf, self)
        q = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=5, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.VOLATILE)
        self.pub = self.create_publisher(PoseStamped, g('topic'), q)
        self.n_pub, self.n_skip, self.last_warn = 0, 0, -1e9
        self.create_timer(1.0 / float(g('rate_hz')), self.tick)
        self.get_logger().info('pose_relay: TF %s->%s -> %s, 기준점 %s (앞으로 %.3f m), %.0f Hz' % (
            self.odom, self.base, g('topic'), g('ref'), self.off, float(g('rate_hz'))))

    def tick(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        try:
            tr = self.tfbuf.lookup_transform(self.odom, self.base, Time(), timeout=Duration(seconds=0.0))
        except Exception as e:                                        # noqa: BLE001
            self.n_skip += 1
            if now - self.last_warn > 5.0:
                self.get_logger().warn('TF %s->%s 없음: %s' % (self.odom, self.base, str(e)[:80])); self.last_warn = now
            return
        age = now - stamp_s(tr.header.stamp)
        if age > self.max_age:
            self.n_skip += 1
            if now - self.last_warn > 5.0:
                self.get_logger().warn('TF 가 %.2f s 오래됨 → 위치 안 냄 (추종기 정지)' % age); self.last_warn = now
            return
        t, r = tr.transform.translation, tr.transform.rotation
        yaw = math.atan2(2 * (r.w * r.z + r.x * r.y), 1 - 2 * (r.y * r.y + r.z * r.z))
        m = PoseStamped()
        m.header.frame_id = self.odom
        m.header.stamp = tr.header.stamp
        m.pose.position.x = float(t.x + self.off * math.cos(yaw))
        m.pose.position.y = float(t.y + self.off * math.sin(yaw))
        m.pose.position.z = float(t.z + self.off * 2 * (r.x * r.z - r.w * r.y))   # 몸체 x 축의 z 성분 (경사에서 기준점 높이, 경로 z 와 같은 점) [10/7]
        m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = r.x, r.y, r.z, r.w
        self.pub.publish(m)
        self.n_pub += 1


def main():
    rclpy.init()
    n = PoseRelay()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
