#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
remote_source_check — daron_bringup.launch.py source:=remote 전용 점검 프로세스.
배치 경로: ~/daron_ros/remote_source_check.py
작성: 2026-09-11

RADXA 도커가 띄운 카메라·IMU 가 이 지상국에 실제로 도달하는지 확인하고
로그로 크게 남긴다. RADXA 쪽이 늦게 뜰 수 있으니 launch 의 나머지 노드는
그대로 두고 경고만 찍는다.

[종료 코드]
  점검이 끝나면 토픽이 하나도 안 보였어도 **rc=0** 이다. 토픽 유무는 로그
  수위(INFO/WARN/ERROR/FATAL)로만 표현한다 — rc≠0 이면 launch 의 on_exit 로거가
  "죽었습니다" 로 보고해 진짜 크래시와 구분이 안 되기 때문 (260911 실측,
  d405_cuvslam_real.py 의 Ctrl+C 종료코드와 같은 이유).
  rc=1 은 진짜 예외뿐: 파라미터 타입/값 오류, 점검 코드 자체의 예외.
  어느 경로든 종료 직전에 [remote 소스 점검 결과] 블록을 반드시 찍는다.

[로그 수위 — rclpy 함정]
  rclpy 는 로그 호출 지점(파일:줄:f_lasti)마다 severity 를 캐싱하고, 같은 지점을
  다른 수위로 다시 부르면 ValueError('Logger severity cannot be changed between
  calls.') 를 던진다 (260911 실측: 결과 블록이 두 줄 찍히고 죽음). 그래서
  `(log.fatal if x else log.warning)(msg)` 나 `emit = log.xxx; emit(msg)` 처럼
  수위를 골라 한 줄에서 부르면 안 되고, if/elif 로 수위별 호출 줄을 물리적으로 분리한다.

확인 항목:
  1. ROS_DOMAIN_ID (환경변수 그대로. 없으면 0 = launch 인자와 어긋난 것)
  2. 토픽이 그래프에 보이는가 + 발행자 QoS
       /camera/camera/infra1/image_rect_raw   (sensor_msgs/Image)
       /camera/camera/infra2/image_rect_raw   (sensor_msgs/Image)
       /camera/camera/infra1/camera_info      (sensor_msgs/CameraInfo)  ★
       /camera/camera/infra2/camera_info      (sensor_msgs/CameraInfo)  ★
       /ekf/imu/data                          (sensor_msgs/Imu)  imu_cov_relay·EKF 입력 (261017: /imu/data 에서 정정)
     ★ camera_info 두 개는 cuVSLAM 노드(d405_cuvslam_real.py)가 하드코딩으로
       구독한다(:327, :333). 하나라도 안 오면 _consume_pair() 가 :383 에서 에러
       없이 return 해 /slam/odometry 가 아예 안 나온다. RADXA 쪽 README 토픽
       목록에 이 둘이 없으므로 가장 높은 수위(FATAL)로 경고하고
       camera_info_pub.py 로 대체 발행하라고 안내한다.
  3. 메시지가 실제로 들어오는가 (그래프에 보여도 QoS/도메인 문제면 안 들어온다)
  4. stamp 지연 합계 = 지상국 벽시계(time.time()) − header.stamp
       이 값은 시계 오프셋만이 아니라 "시계 오프셋 + 전송 지연(WiFi) + 처리 지연"
       의 합이다. 하나로 분리되지 않으므로 원인을 단정하지 않는다.
       판정 기준은 EKF 의 sensor_timeout(ekf.yaml:18 = 0.2s):
         |합계| > max_skew_ms(기본 100)   → 경고 (여유 절반 소진)
         |합계| > ekf_timeout_ms(기본 200) → EKF 가 이 입력을 전부 버린다
       카메라 stamp 는 cuVSLAM 을 거쳐 /slam/odometry 의 stamp 가 되므로 같은
       기준으로 EKF 에 걸린다.

timeout_s(기본 30) 안에 모든 토픽에서 메시지가 오면 그 시점에 끝나고,
못 받은 토픽이 있으면 "RADXA 쪽 확인 필요" 경고 후 끝난다.

구독 QoS 는 BEST_EFFORT/VOLATILE — 발행자가 RELIABLE 이든 TRANSIENT_LOCAL 이든
(realsense image: RELIABLE+TRANSIENT_LOCAL, camera_info: RELIABLE+VOLATILE,
CV7: RELIABLE+VOLATILE — 전부 bag metadata 실측) 붙는 가장 느슨한 조합이라
"점검 노드 QoS 때문에 안 붙는" 경우를 없앤다.

단독 실행:
  source ~/ros_env.sh
  python3 ~/daron_ros/remote_source_check.py --ros-args -p timeout_s:=30
"""

import os
import statistics
import sys
import time
import traceback

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, Imu

CAMERA_INFO_HINT = ('cuVSLAM 이 /slam/odometry 를 아예 발행하지 않습니다 → '
                    'camera_info_pub.py 로 대체 발행 필요 '
                    '(python3 ~/daron_ros/camera_info_pub.py)')

CHECKS = (
    # (토픽, 메시지 타입, 라벨, critical, 없을 때 안내)
    ('/camera/camera/infra1/image_rect_raw', Image, 'infra1_img', False, ''),
    ('/camera/camera/infra2/image_rect_raw', Image, 'infra2_img', False, ''),
    ('/camera/camera/infra1/camera_info', CameraInfo, 'infra1_info', True, CAMERA_INFO_HINT),
    ('/camera/camera/infra2/camera_info', CameraInfo, 'infra2_info', True, CAMERA_INFO_HINT),
    ('/ekf/imu/data', Imu, 'imu', False, ''),   # [261017] EKF 입력은 /imu/data 가 아니라 /ekf/imu/data (imu_cov_relay input_topic)
)
SAMPLES_PER_TOPIC = 10        # 지연은 첫 메시지 하나가 아니라 몇 개의 중앙값으로
BANNER = '=' * 72
EKF_SENSOR_TIMEOUT_S = 0.2    # ~/daron_ros/config/ekf.yaml:18 — 로그 문구용

LOOSE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)


class RemoteSourceCheck(Node):
    def __init__(self):
        super().__init__('remote_source_check')
        # dynamic_typing: `-p timeout_s:=30` 처럼 정수로 넘어와도 죽지 않게
        # (기본 정적 타이핑이면 INTEGER≠DOUBLE 로 InvalidParameterTypeException,
        #  260911 launch 실측). 읽을 때 float() 으로 맞춘다.
        _num = ParameterDescriptor(dynamic_typing=True)
        self.declare_parameter('timeout_s', 30.0, _num)
        self.declare_parameter('max_skew_ms', 100.0, _num)
        self.declare_parameter('ekf_timeout_ms', EKF_SENSOR_TIMEOUT_S * 1e3, _num)
        self.timeout_s = float(self.get_parameter('timeout_s').value)
        self.max_skew_ms = float(self.get_parameter('max_skew_ms').value)
        self.ekf_timeout_ms = float(self.get_parameter('ekf_timeout_ms').value)

        self.t0 = time.monotonic()
        self.delays = {label: [] for _, _, label, _, _ in CHECKS}   # ms, 지상국 now - stamp
        self.first_seen = {}                                        # label -> 경과 s
        self.subs = []
        for topic, msg_type, label, _, _ in CHECKS:
            self.subs.append(self.create_subscription(
                msg_type, topic, self._make_cb(label), LOOSE_QOS))

        log = self.get_logger()
        log.info(BANNER)
        log.info('[remote 소스 점검] RADXA 가 띄운 카메라·IMU 가 여기 도달하는지 확인')
        domain = os.environ.get('ROS_DOMAIN_ID')
        if domain is None:
            log.warning('  ROS_DOMAIN_ID = (미설정 → 0). launch 인자 ros_domain_id 와 '
                        '다르면 토픽이 한 개도 안 보인다.')
        else:
            log.info(f'  ROS_DOMAIN_ID = {domain}')
        log.info(f'  대기 최대 {self.timeout_s:.0f}s / stamp 지연 경고 {self.max_skew_ms:.0f}ms, '
                 f'EKF sensor_timeout {self.ekf_timeout_ms:.0f}ms')
        log.info('  ★ camera_info 두 토픽은 cuVSLAM 필수 (없으면 /slam/odometry 없음)')
        log.info(BANNER)

        self.timer = self.create_timer(1.0, self._tick)
        self.done = False

    def _make_cb(self, label):
        def _cb(msg):
            now = time.time()
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if label not in self.first_seen:
                self.first_seen[label] = time.monotonic() - self.t0
                self.get_logger().info(
                    f'  ✓ {label}: 첫 메시지 수신 (+{self.first_seen[label]:.1f}s, '
                    f'frame_id={msg.header.frame_id!r})')
            if len(self.delays[label]) < SAMPLES_PER_TOPIC:
                self.delays[label].append((now - stamp) * 1e3)
        return _cb

    def _tick(self):
        elapsed = time.monotonic() - self.t0
        all_sampled = all(len(v) >= SAMPLES_PER_TOPIC for v in self.delays.values())
        if all_sampled or elapsed >= self.timeout_s:
            self.done = True
            self.timer.cancel()
            return
        # 5 초마다 아직 못 받은 토픽을 상기시킨다 (조용히 기다리면 멈춘 줄 안다).
        if int(elapsed) % 5 == 0:
            missing = [lb for _, _, lb, _, _ in CHECKS if lb not in self.first_seen]
            if missing:
                self.get_logger().info(
                    f'  … {elapsed:.0f}s 경과, 아직 안 들어옴: {", ".join(missing)}')

    # ── 결과 블록 ────────────────────────────────────────────────────
    def _judge(self, label, critical, samples):
        """(판정 문자열, 수위 'fatal'|'error'|'warn'|'info', 요약 문구 또는 None, fatal 여부)"""
        if not samples:
            if critical:
                return 'FATAL 없음 → camera_info_pub.py 필요', 'fatal', f'{label} 없음', True
            return 'WARN 메시지 없음', 'warn', f'{label} 없음', False
        med = statistics.median(samples)
        if abs(med) > self.ekf_timeout_ms:
            return (f'ERROR |지연|>{self.ekf_timeout_ms:.0f}ms — EKF 가 이 입력을 전부 버립니다',
                    'error', f'{label} 지연 {med:+.0f}ms(EKF 폐기)', False)
        if abs(med) > self.max_skew_ms:
            return (f'WARN |지연|>{self.max_skew_ms:.0f}ms (sensor_timeout 여유 '
                    f'{self.ekf_timeout_ms - abs(med):.0f}ms 남음)',
                    'warn', f'{label} 지연 {med:+.0f}ms', False)
        return 'OK', 'info', None, False

    def report(self):
        """
        종료 직전에 반드시 한 번 찍는다. 토픽별 한 줄 + 요약 한 줄.
        어떤 토픽에서 예외가 나도 나머지 줄과 요약은 나오게 줄 단위로 감싼다.
        """
        log = self.get_logger()
        elapsed = time.monotonic() - self.t0
        log.info(BANNER)
        log.info(f'[remote 소스 점검 결과] {elapsed:.1f}s 경과, ROS_DOMAIN_ID='
                 f'{os.environ.get("ROS_DOMAIN_ID", "(미설정→0)")}')
        log.info(f'  {"토픽":<40} {"발견":<14} {"수신":<6} {"지연(ms)":<22} 판정')
        reached = 0
        problems, fatal = [], []
        for topic, _, label, critical, _ in CHECKS:
            try:
                samples = self.delays.get(label, [])
                try:
                    pubs = self.get_publishers_info_by_topic(topic)
                    found = (f'pub {len(pubs)} '
                             + ','.join(f'{p.qos_profile.reliability.name[:3]}/'
                                        f'{p.qos_profile.durability.name[:3]}' for p in pubs)
                             if pubs else 'pub 0')
                except Exception as exc:                # 그래프 조회 실패해도 줄은 찍는다
                    found = f'조회실패({type(exc).__name__})'
                if samples:
                    reached += 1
                    med = statistics.median(samples)
                    delay = f'{med:+.1f} [{min(samples):+.0f}~{max(samples):+.0f}] n={len(samples)}'
                else:
                    delay = '-'
                verdict, level, summary, is_fatal = self._judge(label, critical, samples)
                line = f'  {topic:<40} {found:<14} {len(samples):<6} {delay:<22} {verdict}'
                # 수위마다 호출 줄을 물리적으로 분리. 로거 함수를 변수에 담아 한 줄에서
                # 부르면(emit(line)) rclpy 가 그 줄의 severity 를 캐싱해 두 번째 수위에서
                # ValueError('Logger severity cannot be changed between calls.') 로 죽는다.
                if level == 'fatal':
                    log.fatal(line)
                elif level == 'error':
                    log.error(line)
                elif level == 'warn':
                    log.warning(line)
                else:
                    log.info(line)
                if summary:
                    (fatal if is_fatal else problems).append(summary)
            except Exception:
                log.error(f'  {topic:<40} (결과 줄 생성 중 예외)\n{traceback.format_exc()}')
                problems.append(f'{label} 점검 예외')

        n = len(CHECKS)
        if fatal:
            log.fatal(f'  ★★★ {n}개 중 {reached}개 도달 — cuVSLAM 필수 토픽 없음: '
                      + ' / '.join(fatal))
            log.fatal('      ' + CAMERA_INFO_HINT)
        if problems:
            log.warning(f'  ⚠⚠ {n}개 중 {reached}개 도달 — RADXA 확인 필요: ' + ' / '.join(problems))
        if fatal or problems:
            log.warning('     (중단하지 않음 — 늦게 뜨면 EKF/cuVSLAM 은 토픽이 오는 대로 붙는다. '
                        'RADXA: 도커 컨테이너, ROS_DOMAIN_ID, 시계 동기화)')
        else:
            log.info(f'  ✅ {n}개 중 {reached}개 도달 — 카메라 4 토픽 + IMU 모두 수신, '
                     'stamp 지연 허용 범위 안')
        log.info(BANNER)


def main():
    rclpy.init()
    node = None
    rc = 0
    log = rclpy.logging.get_logger('remote_source_check')
    try:
        try:
            node = RemoteSourceCheck()
        except Exception:
            # 파라미터 타입/값 오류 등 — 점검을 시작조차 못 한 진짜 예외.
            log.error('점검 노드 생성 실패:\n' + traceback.format_exc())
            rc = 1
            raise SystemExit
        try:
            while rclpy.ok() and not node.done:
                rclpy.spin_once(node, timeout_sec=0.2)
        except (KeyboardInterrupt, ExternalShutdownException):
            log.info('중단됨 (Ctrl+C) — 지금까지의 결과를 찍고 나갑니다.')
        except Exception:
            # 점검 코드 자체의 버그. 결과 블록은 그래도 찍고 rc=1.
            log.error('점검 중 예외:\n' + traceback.format_exc())
            rc = 1
        # ★ 어느 경로든 결과 블록은 반드시 찍는다 (토픽 0개여도 rc=0).
        try:
            node.report()
        except Exception:
            log.error('결과 블록 출력 중 예외:\n' + traceback.format_exc())
            rc = 1
    except SystemExit:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)   # 0 = 점검 완료(토픽 유무 무관). 1 = 점검 자체가 못 돈 진짜 예외.


if __name__ == '__main__':
    main()
