#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
imu_cov_relay — CV7 /ekf/imu/data 의 공분산만 덮어써서 재발행하는 릴레이.
배치 경로: ~/daron_ros/imu_cov_relay.py
작성: 2026-09-11
환경: ~/ros_env.sh (클린 ROS Humble, 시스템 python3). cuVSLAM venv 아님.

[왜 필요한가]
  CV7 드라이버(~/cv7_config/daron_cv7.yml, filter_imu_data_rate=100)가 내는
  /ekf/imu/data 는 /imu/data 보다 데이터가 낫지만(정지 시 자이로 bias 31% 감소)
  공분산이 사실상 0 이다 (bags/imu_straight_01 첫 메시지 실측):
      orientation          roll/pitch ~2.7e-07, yaw 0.0
      angular_velocity     전부 0.0
      linear_acceleration  전부 0.0
  robot_localization 은 공분산 0 을 "무한 신뢰"로 해석해 발산한다.
  드라이버 yml 에는 공분산 파라미터가 없어(grep 확인) 소스에서 못 고친다.
  → 메시지를 그대로 넘기되 세 공분산 행렬의 대각만 파라미터 값으로 바꾼다.

[건드리지 않는 것]
  header.stamp / header.frame_id / orientation / angular_velocity /
  linear_acceleration 값 자체. 공분산의 비대각 성분도 그대로 둔다.

[QoS]
  입력 토픽과 같은 RELIABLE / VOLATILE 로 구독·발행한다.
  근거: ~/bags/{imu_*,scale_ref_*}/metadata.yaml 의 offered_qos_profiles
  (8 세션 전부 reliability=1(RELIABLE), durability=2(VOLATILE)).
  기동 시 실제 발행자의 QoS 를 조회해 다르면 경고를 찍는다 — 안 맞으면
  에러 없이 조용히 아무것도 안 들어오는 것이 DDS 의 기본 동작이라서다.

[use_sim_time]
  rclpy Node 가 자동으로 선언하는 표준 파라미터를 그대로 쓴다(여기서 다시
  declare 하지 않음). bag 재생 검증 때 use_sim_time:=true 로 넘기면 된다.
  이 노드는 stamp 를 손대지 않으므로 시계는 로그 타임스탬프에만 쓰인다.

사용 예:
  source ~/ros_env.sh
  python3 ~/daron_ros/imu_cov_relay.py --ros-args \
      -p input_topic:=/ekf/imu/data -p output_topic:=/imu/data_fixed \
      -p orientation_cov:="[0.01, 0.01, 0.01]" \
      -p angular_velocity_cov:="[0.01, 0.01, 0.01]" \
      -p linear_acceleration_cov:="[0.01, 0.01, 0.01]"
"""

import sys

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import Imu

# 기본 공분산: CV7 가 /imu/data 에 박아 내는 상수(0.01, 실측)와 같게 둔다.
# ekf.yaml 이 지금 /imu/data(0.01) 로 배관돼 있으니 여기로 갈아타도 EKF 가 보는
# 신뢰도는 그대로고, 값(bias)만 좋아진다 — 공분산 튜닝은 그 다음 문제.
DEFAULT_COV = [0.01, 0.01, 0.01]

# sensor_msgs/Imu 의 세 공분산은 모두 float64[9] (3x3 row-major). 대각은 0/4/8.
DIAG_IDX = (0, 4, 8)

# /ekf/imu/data 발행자 QoS 실측치 (bag metadata, 상단 주석 참조).
RELAY_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=10,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)


class ImuCovRelay(Node):
    def __init__(self):
        super().__init__('imu_cov_relay')

        self.declare_parameter(
            'input_topic', '/ekf/imu/data',
            ParameterDescriptor(description='구독할 sensor_msgs/Imu 토픽'))
        self.declare_parameter(
            'output_topic', '/imu/data_fixed',
            ParameterDescriptor(description='공분산을 바꿔 재발행할 토픽'))
        for name, what in (
            ('orientation_cov', 'orientation_covariance'),
            ('angular_velocity_cov', 'angular_velocity_covariance'),
            ('linear_acceleration_cov', 'linear_acceleration_covariance'),
        ):
            # dynamic_typing: `[1, 1, 1]`(INTEGER_ARRAY) 나 스칼라 `0.02` 로 넘어와도
            # 죽지 않게. _read_cov 가 float 3개로 정규화한다.
            self.declare_parameter(
                name, DEFAULT_COV,
                ParameterDescriptor(
                    description=f'{what} 대각 3개 [xx, yy, zz]. 전부 > 0 이어야 한다.',
                    dynamic_typing=True))

        self._in_topic = self.get_parameter('input_topic').value
        self._out_topic = self.get_parameter('output_topic').value
        self._cov = {
            name: self._read_cov(name)
            for name in ('orientation_cov', 'angular_velocity_cov', 'linear_acceleration_cov')
        }
        if self._in_topic == self._out_topic:
            raise ValueError(f'input_topic 과 output_topic 이 같습니다: {self._in_topic}')

        self._pub = self.create_publisher(Imu, self._out_topic, RELAY_QOS)
        self._sub = self.create_subscription(Imu, self._in_topic, self._on_imu, RELAY_QOS)
        self._count = 0

        self.get_logger().info(
            f'imu_cov_relay: {self._in_topic} -> {self._out_topic} '
            f'(QoS RELIABLE/VOLATILE, KEEP_LAST 10, use_sim_time='
            f'{self.get_parameter("use_sim_time").value})')
        for name, diag in self._cov.items():
            self.get_logger().info(f'  {name:<24} diag = {diag}')
        self._check_publisher_qos()

    # ── 파라미터 ─────────────────────────────────────────────────────
    def _read_cov(self, name):
        raw = self.get_parameter(name).value
        # 스칼라 하나만 주면 세 축에 같은 값으로 편다 (-p x_cov:=0.02).
        if isinstance(raw, (int, float)):
            raw = [float(raw)] * 3
        vals = [float(v) for v in raw]
        if len(vals) != 3:
            raise ValueError(f'{name} 은 길이 3 이어야 합니다 (받은 값: {raw!r})')
        # 0 은 robot_localization 이 "무한 신뢰"로 먹어 발산하는 바로 그 값이라
        # 여기서 막는다. 음수는 공분산이 아니다.
        bad = [v for v in vals if not v > 0.0]
        if bad:
            raise ValueError(f'{name} 에 0 또는 음수가 있습니다: {vals} — 공분산 0 은 '
                             'robot_localization 이 무한 신뢰로 해석해 발산합니다.')
        return vals

    # ── QoS 점검 (경고만) ─────────────────────────────────────────────
    def _check_publisher_qos(self):
        infos = self.get_publishers_info_by_topic(self._in_topic)
        if not infos:
            self.get_logger().info(
                f'  {self._in_topic} 발행자가 아직 없음 (CV7 드라이버가 RADXA 에서 '
                '떠 있는지 확인). 뜨면 자동으로 붙는다.')
            return
        for info in infos:
            q = info.qos_profile
            ok = (q.reliability == RELAY_QOS.reliability
                  and q.durability == RELAY_QOS.durability)
            msg = (f'  발행자 {info.node_name} QoS: reliability={q.reliability.name} '
                   f'durability={q.durability.name}')
            # 수위별로 별도 줄에서 호출 — rclpy 는 호출 지점마다 severity 를 캐싱해
            # 같은 줄을 다른 수위로 부르면 ValueError 로 죽는다 (260911 실측).
            if ok:
                self.get_logger().info(msg)
            else:
                self.get_logger().warning(msg + ' — 이 노드의 RELIABLE/VOLATILE 과 다름. '
                                          '구독이 조용히 안 붙을 수 있으니 RELAY_QOS 를 맞출 것.')

    # ── 릴레이 ───────────────────────────────────────────────────────
    def _on_imu(self, msg: Imu):
        # rclpy 는 콜백마다 새로 역직렬화한 객체를 주므로 제자리 수정이 안전하다.
        # header(stamp/frame_id)와 측정값은 그대로, 대각 공분산만 바꾼다.
        for i, v in zip(DIAG_IDX, self._cov['orientation_cov']):
            msg.orientation_covariance[i] = v
        for i, v in zip(DIAG_IDX, self._cov['angular_velocity_cov']):
            msg.angular_velocity_covariance[i] = v
        for i, v in zip(DIAG_IDX, self._cov['linear_acceleration_cov']):
            msg.linear_acceleration_covariance[i] = v
        self._pub.publish(msg)
        self._count += 1
        if self._count == 1:
            self.get_logger().info(
                f'첫 메시지 릴레이: frame_id={msg.header.frame_id!r} '
                f'stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}')


def main():
    rclpy.init()
    node = None
    rc = 0
    try:
        node = ImuCovRelay()
        rclpy.spin(node)
    except ValueError as exc:
        # 파라미터 검증 실패: 에러 로그 후 종료 (launch 가 rc=1 로 사망을 보고한다).
        rclpy.logging.get_logger('imu_cov_relay').error(str(exc))
        rc = 1
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl+C 는 정상 종료 (rc=0)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
