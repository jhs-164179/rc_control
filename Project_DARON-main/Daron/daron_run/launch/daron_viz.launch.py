#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DARON — RViz 시각화용 기동 런치 (daron_bringup.launch.py 의 복사본, 2026-09-20)
배치 경로: ~/daron_ros/launch/daron_viz.launch.py

원본(daron_bringup.launch.py)과의 차이는 세 가지뿐:
  * cuVSLAM 노드로 ~/d405_cuvslam_viz.py (시각화 토픽 추가 복사본)를 띄운다
  * viz_max_range 인자(기본 2.0 m)를 그 노드에 넘긴다 — 표시용 랜드마크 거리 필터,
    추정에는 영향 없음
  * map -> odom 항등 static TF 를 함께 띄운다 (튜토리얼 RViz 화면의 Fixed Frame 이
    map 이라 라벨을 맞추기 위함. EKF 는 world_frame: odom 이라 map 이 원래 없다.
    항등이므로 수치 영향 없음)
그 외(EKF, 릴레이, bag play/record, 인자)는 원본과 동일하다. 9/22 현장 촬영은 원본으로.

[2026-09-21 추가 — 4 궤적 비교 (실시간·bag 재생 동일 코드 경로)]
  ① VSLAM only            /slam/odometry            → vo_path            (기존)
  ② INS only (ZUPT 없음)   ins_odom_node.py          → /ins/odometry → /ins/path       [ins:=true]
  ③ VSLAM+INS+RL           ekf_filter_node (A)       → /odometry/filtered → slam_path  (기존)
  ④ ③ + recalib            imu_recalib_relay.py + ekf_filter_node_recalib (B)
                           → /odometry/filtered_recalib → recalib_path                 [recalib:=true]
  체인: /ekf/imu/data → imu_cov_relay → /imu/data_fixed ─┬→ EKF-A
                                                          └→ imu_recalib_relay → /imu/data_recalib → EKF-B
  EKF-B 는 이름·출력 리매핑·publish_tf=false·imu0_relative=false 네 가지만 다르다(config/ekf_recalib.yaml).
  recalib 의 본체는 **중력 기준 body-frame 정렬**(recalib_body_align): 릴레이가 g0 로 장착 자세를 잡아
  base_link 자세를 만들어 준다. 전복 뒤 Δθ 는 기록만(recalib_dpitch:=false 기본). 근거는 imu_recalib_relay.py 상단.
  recalib 은 IMU 메시지 스트림 위에서 우리 노드가 하므로 bag 재생에서도 실시간과 같이 동작한다.
  (센서 내장 필터 리셋 같은 하드웨어 측 재초기화가 아니다 — 그건 bag 으로는 불가능하다.)
  recalib_apply:=false 면 감지·이벤트만 내고 IMU 를 그대로 통과시킨다(대조군: A==B 여야 정상).
  기동 순서: rsp → cuVSLAM(+ins_odom, odom_to_path) → imu_cov_relay → imu_recalib_relay → EKF-A/B, record, bag play

[2026-09-29 추가 — ⑤ health 기반 공분산 EKF-C]  [health:=true, recalib:=true 필요]
  health_monitor.py: /ekf/imu/data·/slam/status 로 INS/SLAM 자기진단 + 자이로↔SLAM 회전 교차검사 → health
    /imu/data_recalib → /imu/data_h (× 1/h², INS FAIL 이면 버림), /slam/odometry → /slam/odometry_h
    (위치 × 1/h², 자세 분산은 INS OK 1.0 / FAIL 0.01) → ekf_filter_node_health(C) → /odometry/filtered_health
    → health_path. 재정렬은 /ekf_filter_node_health/set_pose 로만(A/B 는 /set_pose 라 안 건드림).
  C 는 B 와 입력 토픽 2개 + SLAM 자세 on 만 다르다(config/ekf_health.yaml). 판정 로직은 nodes/lib/health.py.
  기동 순서: ... → imu_recalib_relay → health_monitor → EKF-A/B/C, record, bag play
  [tf_source, 260929] odom→base_link TF 를 내는 EKF 하나를 고른다. 기본 C(최종 pose = EKF-C, 사용자 결정).
    A 로 되돌리려면 tf_source:=A. 둘이 같이 내면 TF 가 에러 없이 진동하므로 나머지는 publish_tf=false 로 덮는다.
    B 는 항상 false. /odometry/filtered(A) 토픽·slam_path 선은 비교용으로 그대로 나온다.

── 이하 원본 docstring ──
DARON — 데스크탑 통합 기동 런치 (5 프로세스)
배치 경로: ~/daron_ros/launch/daron_bringup.launch.py
작성: 2026-09-10

  1. realsense2_camera      (bag_path 지정 시 ros2 bag play 로 대체,
                             source:=remote 면 안 띄움 — RADXA 도커가 띄운다)
  2. robot_state_publisher  (xacro 를 launch 시점에 실행)
  3. d405_cuvslam_real.py   (venv 인터프리터 + PYTHONPATH 주입)
  4. imu_cov_relay.py       (/ekf/imu/data → /imu/data_fixed, imu_relay:=true 일 때)
  5. ekf_node               (ekf.launch.py include)
  6. ros2 bag record        (record:=true 일 때만)

RADXA 쪽(CV7 microstrain 드라이버 등)은 이 파일에 없다. 별도로 띄울 것.

[source 모드]
  local  (기본)   이 PC 의 D405 를 realsense2_camera 로 직접 띄운다.
  bag             bag_path 지정 시 자동. ros2 bag play 로 대체, use_sim_time 강제 true.
  remote          카메라·IMU 를 안 띄운다(RADXA 도커가 발행). rsp/cuVSLAM/EKF/녹화만.
                  use_sim_time 은 false 고정. remote_source_check.py 가 같이 떠서
                  ROS_DOMAIN_ID, 카메라 4 토픽(image/camera_info ×2)·IMU 도달 여부,
                  stamp 지연 합계(오프셋+전송+처리; >100ms 경고, >200ms 는 EKF
                  sensor_timeout 초과)를 로그로 남긴다. camera_info 가 없으면
                  FATAL + camera_info_pub.py 안내. 30s 안 보이면 "RADXA 쪽 확인
                  필요" 경고만 하고 중단하지 않는다.
                  [261008] image_relay.py 도 같이 뜬다(remote_relay:=true 기본): RADXA 가
                  camera_namespace:=radxa 로 낸 /radxa/camera/* 를 WiFi 로 한 번만 받아
                  /camera/camera/* 로 재발행 → 지상국 구독 프로세스가 몇 개든 WiFi 는 1벌.
                  [261011] remote_depth:=zstd(기본): depth 영상만 RADXA zstd 토픽으로 받아
                  video_receiver.py 가 풀어 /camera/camera/depth/image_rect_raw 로 재발행
                  (image_relay 는 depth 영상만 빼고 IR·camera_info 계속 중계). raw = 옛 동작.

사용 예:
  실시간 :  ros2 launch ~/daron_ros/launch/daron_bringup.launch.py session_name:=run_01
  bag검증:  ros2 launch ~/daron_ros/launch/daron_bringup.launch.py \
                bag_path:=~/daron_bags/straight_01 record:=false
            (bag 의 /tf_static /tf /slam/odometry /odometry/filtered 는 재생에서
             뺀다 — 지금 띄우는 노드가 같은 토픽을 발행하므로. replay_exclude 로 변경)
  원격   :  ros2 launch ~/daron_ros/launch/daron_bringup.launch.py \
                source:=remote session_name:=wifi_01

────────────────────────────────────────────────────────────────────────
[기동 순서]  RegisterEventHandler(OnProcessStart) 로 걸었다. TimerAction 은
             고정 지연이라 머신이 느릴 때 조용히 순서가 뒤집히므로 쓰지 않는다.

    robot_state_publisher ──start──▶ d405_cuvslam_real ──start──▶ imu_cov_relay ──start──▶ ekf_node
                                                                                      └──▶ bag record
                                                                                      └──▶ bag play
    (imu_relay:=false 면 릴레이 단계를 건너뛰고 cuVSLAM start 에서 바로 EKF 로 간다)

    imu_cov_relay 가 EKF/bag play 보다 먼저 떠야 하는 이유: ekf.yaml 의 imu0 가
    /imu/data_fixed 이고 이걸 내는 건 릴레이뿐이다. 릴레이가 없으면 EKF 는
    구독자만 있고 발행자가 없어 IMU 입력이 0 이 되는데, 위치는 SLAM 에서 오므로
    /odometry/filtered 는 계속 나온다 — 자세 보정만 조용히 사라지고 에러가 없다.

    ⚠ 남은 레이스: ekf_node 는 include 라 프로세스 핸들이 없어서
      "ekf 가 뜬 뒤 bag play" 를 이벤트로 걸 수 없다. 같은 on_start 배치 안에서
      ekf → bag play 순으로 나열만 해 뒀다. bag 앞부분 몇 프레임을 EKF 가
      놓치는 것이 문제가 되면 --rate 를 더 낮추거나 --start-paused 를 쓸 것.

[환경]  ~/slam_env.sh 를 읽어 확인한 값 (2026-09-10 실측, 추측 아님):
      (아래 ~ 는 실행 사용자의 홈. 261017 부터 코드도 HOME 기준)
      PATH             앞에 ~/venv/cuvslam/bin
      VIRTUAL_ENV      ~/venv/cuvslam
      PYTHONPATH       (ROS 것) + ~/Documents/cuVSLAM/python/build/cp310-cp310-linux_x86_64
                       ※ 지시받은 ".../python/build/" 가 아니라 그 아래
                          cp310-cp310-linux_x86_64 서브디렉터리다. build/ 만
                          잡으면 pycuvslam import 가 실패한다.
      CUVSLAM_BUILD_DIR ~/Documents/cuVSLAM/build/
      CUVSLAM_DATASETS  ~/datasets/
      CUVSLAM_OUTPUT    ~/Documents/cuVSLAM/build/output/
      COLCON_PREFIX_PATH ~/cuvslam_ws/install
      ROS_DOMAIN_ID     42
    LD_LIBRARY_PATH 는 slam_env.sh 가 건드리지 않는다(.so 가 RPATH 로 찾는다).
    ~/Documents/cuVSLAM/build/cuvslam_vars.sh 안의 .venv activate 줄은
    해당 .venv 가 존재하지 않아 no-op — 즉 ~/venv/cuvslam 이 그대로 유지된다.

    ⚠ ROS_DOMAIN_ID: slam_env.sh 만 42 를 export 한다. ~/.bashrc 에는 없다.
      cuVSLAM 프로세스에만 42 를 주면 나머지 4개는 도메인 0 이라 토픽이
      한 개도 안 오간다 — 에러 없이 조용히 죽는 전형적인 경우다.
      그래서 ros_domain_id 를 launch 인자로 두고 5개 전부에 같은 값을 넣는다.
────────────────────────────────────────────────────────────────────────
"""

import math
import os
import shutil
import subprocess
import xml.etree.ElementTree as ET
from datetime import datetime

import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

HOME = os.path.expanduser('~')

# ── slam_env.sh 에서 확인한 값 (상단 주석 참조) ──────────────────────────
#   [261017] /home/pakr 고정 → HOME 기준 (다른 사용자 이름의 PC 에서도 같은 배치로 동작)
VENV_DIR = os.path.join(HOME, 'venv/cuvslam')
VENV_PY = os.path.join(VENV_DIR, 'bin', 'python3')          # -> /usr/bin/python3.10
PYCUVSLAM_DIR = os.path.join(HOME, 'Documents/cuVSLAM/python/build/cp310-cp310-linux_x86_64')
CUVSLAM_BUILD_DIR = os.path.join(HOME, 'Documents/cuVSLAM/build/')
CUVSLAM_DATASETS = os.path.join(HOME, 'datasets/')
CUVSLAM_OUTPUT = os.path.join(HOME, 'Documents/cuVSLAM/build/output/')
COLCON_PREFIX_PATH = os.path.join(HOME, 'cuvslam_ws/install')

# [daron_run 260929] 스크립트·설정은 이 launch 파일 기준 상대경로 (~/daron_run/{nodes,config,launch}).
#   원본(~/daron_ros, ~/d405_cuvslam_viz.py, ~/daron.urdf.xacro)과 독립 — 폴더째 옮겨도 동작.
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
NODES = os.path.join(ROOT, 'nodes')
CONFIG = os.path.join(ROOT, 'config')

CUVSLAM_SCRIPT = os.path.join(NODES, 'd405_cuvslam_viz.py')   # [viz] 시각화 복사본 (원본: d405_cuvslam_real.py)
REMOTE_CHECK_SCRIPT = os.path.join(NODES, 'remote_source_check.py')
IMU_RELAY_SCRIPT = os.path.join(NODES, 'imu_cov_relay.py')
INS_SCRIPT = os.path.join(NODES, 'ins_odom_node.py')              # [ins]
RECALIB_SCRIPT = os.path.join(NODES, 'imu_recalib_relay.py')      # [recalib]
ODOM_TO_PATH_SCRIPT = os.path.join(NODES, 'odom_to_path.py')     # [ins/recalib] RViz 선
HEALTH_SCRIPT = os.path.join(NODES, 'health_monitor.py')         # [health]
IMAGE_RELAY_SCRIPT = os.path.join(NODES, 'image_relay.py')       # [remote 261008] WiFi 영상 1벌 중계
# [261011] remote_depth:=zstd — RADXA 가 zstd 로 보낸 depth 를 풀어 /camera/camera/depth/image_rect_raw 로 재발행
#   (depth 만. infra1/2·camera_info 는 image_relay 가 계속 raw 중계). fire·zstandard 가 cuvslam venv 에 있음.
VIDEO_RECEIVER_SCRIPT = os.path.join(os.path.dirname(NODES), 'depth_transport', 'video_receiver.py')
RELAY_STREAMS_ALL = ['infra1/image_rect_raw', 'infra1/camera_info',
                     'infra2/image_rect_raw', 'infra2/camera_info',
                     'depth/image_rect_raw', 'depth/camera_info']
DEFAULT_EKF_HEALTH_PARAMS = os.path.join(CONFIG, 'ekf_health.yaml')
DEFAULT_EKF_RECALIB_PARAMS = os.path.join(CONFIG, 'ekf_recalib.yaml')
DEFAULT_XACRO = os.path.join(CONFIG, 'daron.urdf.xacro')
DEFAULT_EKF_LAUNCH = os.path.join(ROOT, 'launch', 'ekf.launch.py')
DEFAULT_EKF_PARAMS = os.path.join(CONFIG, 'ekf.yaml')
DEFAULT_TOPIC_LIST = os.path.join(CONFIG, 'record_topics_viz.txt')   # [viz] 원본 + INS/recalib 토픽
DEFAULT_BAG_DIR = os.path.join(os.path.dirname(ROOT), 'daron_bags')   # [260929] daron_run 옆 (~/Daron/daron_bags)
URDF_OUT_DIR = '/tmp/daron_bringup'
# bag 재생 시 bag 에서 빼는 토픽. 재생 중 새로 띄우는 노드가 같은 토픽을 발행해
# 충돌하는 것들 — robot_state_publisher(/tf_static), EKF(/tf, /odometry/filtered),
# cuVSLAM(/slam/odometry), imu_cov_relay(/imu/data_fixed).
# TF 는 브로드캐스터가 둘이 되면 에러 없이 조용히 진동한다.
# [ins/recalib] 산출물도 같은 이유로 뺀다 — 지금 띄우는 노드가 다시 계산해 발행한다.
DEFAULT_REPLAY_EXCLUDE = ('/tf_static /tf /slam/odometry /odometry/filtered /imu/data_fixed '
                          '/ins/odometry /ins/status /ins/path /imu/data_recalib /odometry/filtered_recalib '
                          '/accel/filtered_recalib /ins/recalib_event /ins/recalib_status /ins/recalib_marker '
                          '/visual_slam/tracking/recalib_path '
                          # [health] 산출물 — health_monitor / EKF-C / cuVSLAM(/slam/status) 이 다시 낸다
                          '/slam/status /health/state /health/marker /imu/data_h /slam/odometry_h '
                          '/odometry/filtered_health /accel/filtered_health /visual_slam/tracking/health_path '
                          # ⚠ 녹화된 set_pose 를 재생하면 EKF-C 가 옛 재정렬 명령으로 리셋된다
                          '/ekf_filter_node_health/set_pose '
                          # EKF 진단(입력 거절 등) — 재생 중 새로 뜬 EKF 들이 다시 낸다
                          '/diagnostics '
                          # [261008] 출발 자세 — imu_recalib_relay 가 다시 낸다
                          '/ins/start_attitude '
                          # [261008] Slam 산출물 — cuVSLAM 노드가 다시 낸다
                          '/slam/reloc_status /slam/loop_closure /slam/map_pose '
                          # [261011] 루프 클로저 보정 이력 — cuVSLAM 노드가 다시 낸다
                          '/slam/map_odom_history '
                          # [261016] 로버·추종·경로계획·조작 — 사고 재구성용으로 녹화만 한다(record_topics_*.txt).
                          # ⚠ 지령 토픽을 재생하면 같은 도메인의 로버가 움직이고, 살아 있는 플래너·추종기와 발행자가 둘이 된다.
                          '/rover/diagnostics /rover/odom /rover/battery /joint_states '
                          '/rover/auto/drive_rpm /rover/auto/steer_cmd /rover/auto_enable /rover/inverted '
                          '/rover/drive_cmd /rover/steer_cmd /rover/articulation_cmd /rover/goal_position /rover/goal_relative '
                          '/rover/command /follower/debug /path_follower/status /path_follower/enable '
                          '/planner/path /planner/status /planner/waypoints /planner/route /localization/pose /goal_pose '
                          '/ui/mode /ui/waypoint_cmd /ui/waypoint_click /ui/gap_point /ui/field /gui/waypoints /gui/waypoint_goal')
# [261008] rebuild:=true — 녹화된 맵·판정·경로계획·GUI·핀 산출물도 재생에서 뺀다. 녹화 bag 으로 지금 코드
# (그리드 피쳐맵·판정·플래너)를 다시 돌려 볼 때 옛 결과(예: 9/30 TSDF 맵)가 새 결과와 섞이지 않게.
# /cda/detections(YOLO 검출)은 남긴다 — pin_node 가 다시 핀을 만든다.
REBUILD_EXCLUDE = ('/live_tsdf/map_cloud /live_tsdf/map_cloud_color /live_tsdf/map_cloud_full /live_tsdf/map_cells '
                   '/traversability/grid /traversability/cloud /traversability/why '
                   '/planner/status /planner/path /planner/waypoints /planner/route /planner/arena_grid '
                   '/localization/pose /goal_pose /ui/waypoint_cmd /gui/waypoints /gui/waypoint_goal '
                   '/pin_node/pins /pin_node/pins_json /pin_gallery/pins_info /cda/overlay')
# [261008] 장착 세트. 실시간은 설계값(level), 재생은 bag 옆 사이드카(<bag>.daron_meta.yaml)의 값,
# 없으면 10/8 이전 bag 으로 보고 cad0902(옛 동작: 카메라 -8°, IMU 출발 = 수평 가정).
MOUNTS = ('level', 'cad0902')
META_SUFFIX = '.daron_meta.yaml'


# ══════════════════════════════════════════════════════════════════════
# 헬퍼
# ══════════════════════════════════════════════════════════════════════
def _as_bool(text, name):
    v = str(text).strip().lower()
    if v in ('1', 'true', 'yes', 'on'):
        return True
    if v in ('0', 'false', 'no', 'off'):
        return False
    raise RuntimeError(f"{name} 인자는 true/false 여야 합니다 (받은 값: {text!r})")


def _double_arg(text, name):
    """
    launch 인자(문자열) → rclpy DOUBLE 파라미터용 문자열.

    `-p x:=30` 처럼 넘기면 rclpy 가 YAML 로 파싱해 INTEGER 로 만들고,
    노드가 declare_parameter('x', 30.0) 으로 DOUBLE 을 기대하면
    InvalidParameterTypeException 으로 기동 즉시 죽는다 (260911 remote_check 실측).
    항상 소수점이 들어간 형태('30.0')로 돌려준다.
    """
    try:
        v = float(text)
    except (TypeError, ValueError):
        raise RuntimeError(f'{name} 인자는 숫자여야 합니다 (받은 값: {text!r})')
    if not math.isfinite(v):
        raise RuntimeError(f'{name} 인자가 유한한 숫자가 아닙니다: {text!r}')
    out = repr(v)                       # 30.0 / 0.5 / 1e+20
    if 'e' in out or 'E' in out:
        out = f'{v:.1f}'                # YAML 1.1 은 '1e+20' 을 float 로 안 읽는다
    return out


def _ros_env(domain_id):
    """시스템 ROS(/opt/ros/humble) 프로세스용 환경 델타."""
    return {'ROS_DOMAIN_ID': str(domain_id)}


def _cuvslam_env(domain_id):
    """
    cuVSLAM 프로세스용 환경 델타.

    additional_env 는 키 단위 '덮어쓰기'라서 PATH/PYTHONPATH 처럼 누적돼야 하는
    변수는 os.environ 값을 읽어 직접 이어 붙인다. (launch 를 띄운 셸이
    /opt/ros/humble 만 source 한 상태라고 가정 — 그 셸의 ROS PYTHONPATH 를 살린다.)
    """
    pythonpath = [p for p in os.environ.get('PYTHONPATH', '').split(os.pathsep) if p]
    if PYCUVSLAM_DIR not in pythonpath:
        pythonpath.append(PYCUVSLAM_DIR)

    path = [p for p in os.environ.get('PATH', '').split(os.pathsep) if p]
    venv_bin = os.path.join(VENV_DIR, 'bin')
    if venv_bin in path:
        path.remove(venv_bin)
    path.insert(0, venv_bin)

    return {
        'ROS_DOMAIN_ID': str(domain_id),
        'PATH': os.pathsep.join(path),
        'PYTHONPATH': os.pathsep.join(pythonpath),
        'VIRTUAL_ENV': VENV_DIR,
        'VIRTUAL_ENV_PROMPT': '(cuvslam)',
        'CUVSLAM_BUILD_DIR': CUVSLAM_BUILD_DIR,
        'CUVSLAM_DATASETS': CUVSLAM_DATASETS,
        'CUVSLAM_OUTPUT': CUVSLAM_OUTPUT,
        'COLCON_PREFIX_PATH': COLCON_PREFIX_PATH,
    }


def _on_exit_logger(label, fatal_hint='', ok_hint=''):
    """
    프로세스가 죽으면 왜 죽었는지 로그를 남긴다.
    launch 는 기본적으로 자식이 조용히 사라져도 나머지를 계속 돌리기 때문에,
    이게 없으면 "토픽이 안 나오는데 에러도 없다" 상태가 된다.
    ok_hint 는 rc=0 이 '정상' 인 프로세스(스스로 끝나는 점검 등)에 그 뜻을 덧붙인다.
    """
    def _handler(event, context):
        rc = event.returncode
        if rc == 0:
            return [LogInfo(msg=f'[{label}] 정상 종료 (rc=0).' + (f' {ok_hint}' if ok_hint else ''))]
        msgs = [LogInfo(msg=f'💀 [{label}] 죽었습니다 — returncode={rc}. '
                            f'위쪽 [{label}] 출력에서 마지막 스택/에러를 보세요.')]
        if fatal_hint:
            msgs.append(LogInfo(msg=f'   ↳ {fatal_hint}'))
        return msgs
    return _handler


def _read_topic_list(path):
    if not os.path.isfile(path):
        raise RuntimeError(
            f'녹화 토픽 목록이 없습니다: {path}\n'
            '  record:=false 로 끄거나, 한 줄에 토픽 하나씩 적어 만드세요.')
    topics = []
    for line in open(path, encoding='utf-8'):
        line = line.split('#', 1)[0].strip()
        if line:
            topics.append(line)
    if not topics:
        raise RuntimeError(f'{path} 에 토픽이 한 개도 없습니다 (주석/빈 줄만 있음).')
    return topics


def _parse_topic_set(text):
    """공백/쉼표 구분 토픽 문자열 -> set. 빈 문자열이면 빈 set(아무것도 안 뺌)."""
    items = {t.strip() for t in text.replace(',', ' ').split()}
    bad = sorted(t for t in items if not t.startswith('/'))
    if bad:
        raise RuntimeError(f'replay_exclude 의 토픽은 / 로 시작해야 합니다: {bad}')
    return items


def _bag_topics(bag_path, exclude):
    """
    bag 안의 토픽 이름을 metadata.yaml 에서 읽는다.
    Humble 의 `ros2 bag play` 에는 --exclude 가 없어서(확인함) 제외를 하려면
    남길 토픽을 --topics 로 전부 나열하는 수밖에 없다.
    """
    meta = os.path.join(bag_path, 'metadata.yaml')
    if not os.path.isfile(meta):
        raise RuntimeError(f'bag metadata 가 없습니다: {meta} (bag_path 가 폴더 경로인지 확인)')
    with open(meta, encoding='utf-8') as f:
        info = yaml.safe_load(f)['rosbag2_bagfile_information']
    names = [e['topic_metadata']['name'] for e in info.get('topics_with_message_count', [])]
    kept = [n for n in names if n not in exclude]
    if not kept:
        raise RuntimeError(f'{meta} 에서 재생할 토픽을 못 찾았습니다 (전체: {names}).')
    return kept, [n for n in names if n in exclude]


def _resolve_mount(arg, source, bag_path):
    """[261008] mount 인자(auto|level|cad0902) → (mount, 이유)."""
    arg = str(arg).strip().lower()
    if arg in MOUNTS:
        return arg, 'mount 인자 명시'
    if arg != 'auto':
        raise RuntimeError(f"mount 인자는 auto/level/cad0902 여야 합니다 (받은 값: {arg!r})")
    if source != 'bag':
        return 'level', f'source={source} → 설계값'
    meta = bag_path.rstrip('/') + META_SUFFIX
    if os.path.isfile(meta):
        with open(meta, encoding='utf-8') as f:
            m = str((yaml.safe_load(f) or {}).get('mount', '')).strip().lower()
        if m in MOUNTS:
            return m, f'사이드카 {meta}'
        raise RuntimeError(f'{meta} 의 mount 값이 이상합니다: {m!r}')
    return 'cad0902', f'사이드카 없음({os.path.basename(meta)}) → 10/8 이전 bag 으로 봄'


def _urdf_joint(xml, name):
    """생성된 URDF 에서 joint origin (xyz, rpy) 를 읽는다. 없으면 RuntimeError."""
    root = ET.fromstring(xml)
    for j in root.iter('joint'):
        if j.get('name') == name:
            o = j.find('origin')
            xyz = [float(v) for v in (o.get('xyz', '0 0 0') if o is not None else '0 0 0').split()]
            rpy = [float(v) for v in (o.get('rpy', '0 0 0') if o is not None else '0 0 0').split()]
            return xyz, rpy
    raise RuntimeError(f'URDF 에 joint {name!r} 가 없습니다')


# ══════════════════════════════════════════════════════════════════════
# 본체
# ══════════════════════════════════════════════════════════════════════
def launch_setup(context, *args, **kwargs):
    def cfg(name):
        return LaunchConfiguration(name).perform(context)

    session_name = cfg('session_name').strip()
    if not session_name:
        session_name = datetime.now().strftime('%y%m%d_%H%M%S')

    bag_path = cfg('bag_path').strip()
    if bag_path:
        bag_path = os.path.abspath(os.path.expanduser(bag_path))
        if not os.path.isdir(bag_path):
            raise RuntimeError(f'bag_path 폴더가 없습니다: {bag_path}')

    source = cfg('source').strip().lower()
    if source not in ('local', 'remote'):
        raise RuntimeError(f"source 인자는 local/remote 여야 합니다 (받은 값: {cfg('source')!r})")
    if source == 'remote' and bag_path:
        raise RuntimeError('source:=remote 와 bag_path 는 같이 쓸 수 없습니다 '
                           '(remote 는 RADXA 실시간, bag 은 재생).')
    if bag_path:
        source = 'bag'
    # remote_source_check.py 가 DOUBLE 로 선언한 파라미터 — 정수 문자열로 넘기면 죽는다.
    remote_timeout_s = _double_arg(cfg('remote_wait_s'), 'remote_wait_s')
    remote_skew_ms = _double_arg(cfg('remote_max_skew_ms'), 'remote_max_skew_ms')
    # [remote 261008] RADXA 카메라 토픽 중계 (image_relay.py)
    remote_relay = cfg('remote_relay').strip().lower() in ('true', '1', 'yes')
    remote_cam_prefix = cfg('remote_cam_prefix').strip() or '/radxa/camera'
    remote_depth = cfg('remote_depth').strip().lower()
    if remote_depth not in ('raw', 'zstd'):
        raise RuntimeError(f"remote_depth 는 raw/zstd 여야 합니다 (받은 값: {cfg('remote_depth')!r})")
    remote_depth_zstd_topic = cfg('remote_depth_zstd_topic').strip()
    # [viz] d405_cuvslam_viz.py 가 DOUBLE 로 선언한 파라미터 — '2' 로 넘기면 INTEGER 로 죽는다.
    viz_max_range = _double_arg(cfg('viz_max_range'), 'viz_max_range')

    record = _as_bool(cfg('record'), 'record')
    use_sim_time = _as_bool(cfg('use_sim_time'), 'use_sim_time')
    domain_id = cfg('ros_domain_id').strip()
    xacro_in = os.path.abspath(os.path.expanduser(cfg('urdf')))
    ekf_launch = os.path.abspath(os.path.expanduser(cfg('ekf_launch')))
    ekf_params = os.path.abspath(os.path.expanduser(cfg('ekf_params')))
    topic_list_file = os.path.abspath(os.path.expanduser(cfg('record_topics')))
    bag_dir = os.path.abspath(os.path.expanduser(cfg('bag_dir')))
    replay_exclude = _parse_topic_set(cfg('replay_exclude'))
    rebuild = _as_bool(cfg('rebuild'), 'rebuild')
    if rebuild:
        replay_exclude |= _parse_topic_set(REBUILD_EXCLUDE)
    imu_relay = _as_bool(cfg('imu_relay'), 'imu_relay')
    imu_relay_in = cfg('imu_relay_in').strip()
    imu_relay_out = cfg('imu_relay_out').strip()
    # [ins/recalib]
    ins_enable = _as_bool(cfg('ins'), 'ins')
    ins_imu_topic = cfg('ins_imu_topic').strip()
    ins_static_s = _double_arg(cfg('ins_static_s'), 'ins_static_s')
    recalib_enable = _as_bool(cfg('recalib'), 'recalib')
    recalib_apply = _as_bool(cfg('recalib_apply'), 'recalib_apply')
    recalib_body_align = _as_bool(cfg('recalib_body_align'), 'recalib_body_align')
    recalib_dpitch = _as_bool(cfg('recalib_dpitch'), 'recalib_dpitch')
    recalib_params = {
        name: _double_arg(cfg(name), name)
        for name in ('recalib_flip_ratio', 'recalib_hold_s', 'recalib_static_s',
                     'recalib_gyro_thr', 'recalib_acc_std_thr', 'recalib_max_wait_s')
    }
    ekf_recalib_params = os.path.abspath(os.path.expanduser(cfg('ekf_recalib_params')))
    # [health]
    health_enable = _as_bool(cfg('health'), 'health')
    health_realign = _as_bool(cfg('health_realign'), 'health_realign')
    health_realign_thresh = _double_arg(cfg('health_realign_thresh_m'), 'health_realign_thresh_m')
    health_extrinsic = cfg('health_extrinsic').strip()
    tf_source = cfg('tf_source').strip().upper()
    if tf_source not in ('A', 'C'):
        raise RuntimeError(f"tf_source 는 A 또는 C 여야 합니다 (받은 값: {cfg('tf_source')!r})")
    ekf_health_params = os.path.abspath(os.path.expanduser(cfg('ekf_health_params')))
    if health_enable and not recalib_enable:
        raise RuntimeError('health:=true 는 recalib:=true 가 필요합니다 — EKF-C 는 /imu/data_recalib(body_align, '
                           'imu0_relative=false)을 받는다. relative=true 면 set_pose 가 IMU 자세를 0 으로 다시 잡는다.')
    if tf_source == 'C' and not health_enable:
        raise RuntimeError('tf_source:=C 는 health:=true 가 필요합니다 (EKF-C 가 없으면 odom→base_link TF 가 사라진다). '
                           'health:=false 로 돌릴 때는 tf_source:=A 를 같이 주세요.')
    bag_rate = _double_arg(cfg('bag_rate'), 'bag_rate')
    if float(bag_rate) <= 0.0:
        raise RuntimeError(f'bag_rate 는 양수여야 합니다 (받은 값: {bag_rate})')

    actions = []

    # ── bag 재생 모드면 use_sim_time 강제 ──────────────────────────────
    # ekf.yaml 주석대로, sim time 없이 옛 타임스탬프를 먹이면 sensor_timeout 에
    # 걸려 EKF 가 에러 한 줄 없이 아무것도 발행하지 않는다. 조용한 실패라
    # 인자를 존중하지 않고 덮어쓴다.
    if source == 'bag' and not use_sim_time:
        use_sim_time = True
        actions.append(LogInfo(
            msg='⚠ bag_path 가 지정돼 use_sim_time 을 강제로 true 로 올립니다 '
                '(false 면 EKF 가 sensor_timeout 에 걸려 조용히 아무것도 발행하지 않음).'))
    # remote 는 실시간이라 sim time 이 있을 수 없다. /clock 발행자가 없는데 true 면
    # 전 노드의 시계가 0 에 멈춰 EKF/cuVSLAM 이 조용히 아무것도 안 한다.
    if source == 'remote' and use_sim_time:
        use_sim_time = False
        actions.append(LogInfo(
            msg='⚠ source:=remote 는 실시간이므로 use_sim_time 을 강제로 false 로 내립니다 '
                '(/clock 발행자가 없어 true 면 전 노드 시계가 멈춤).'))

    source_desc = {
        'bag': 'bag:' + bag_path,
        'local': 'live camera (이 PC 의 D405)',
        'remote': 'remote (RADXA 도커가 카메라·IMU 발행, 여기서는 안 띄움)',
    }[source]
    actions.append(LogInfo(
        msg=f'=== DARON bringup === session={session_name} '
            f'ROS_DOMAIN_ID={domain_id} use_sim_time={use_sim_time} '
            f'record={record} source={source_desc}'))

    ros_env = _ros_env(domain_id)
    cuvslam_env = _cuvslam_env(domain_id)

    # ══════════════════════════════════════════════════════════════
    # 1. robot_state_publisher
    # ══════════════════════════════════════════════════════════════
    # xacro 를 launch 시점에 실행한다. Command() 치환으로 robot_description 에
    # 바로 넣지 않는 이유는, 지시대로 URDF '파일 경로'를 positional 인자로
    # 넘겨야 하기 때문(그 형태가 --ros-args 앞에 와야 UnknownROSArgsError 가 안 난다).
    if not os.path.isfile(xacro_in):
        raise RuntimeError(f'xacro 파일이 없습니다: {xacro_in}')
    if shutil.which('xacro') is None:
        raise RuntimeError('xacro 실행파일을 못 찾았습니다. /opt/ros/humble/setup.bash 를 source 했는지 확인하세요.')

    # [261008] 장착 세트 → xacro mount:=… . 같은 URDF 에서 카메라·IMU 장착값을 읽어 SLAM 노드·릴레이에 넘긴다
    #          (URDF 가 유일한 출처 — 노드 기본값과 어긋날 일이 없게).
    mount, mount_why = _resolve_mount(cfg('mount'), source, bag_path)
    imu_mount_source = cfg('imu_mount_source').strip().lower()
    if imu_mount_source == 'auto':
        imu_mount_source = 'tf' if mount == 'level' else 'g0'
    if imu_mount_source not in ('tf', 'g0'):
        raise RuntimeError(f"imu_mount_source 는 auto/tf/g0 여야 합니다 (받은 값: {imu_mount_source!r})")
    os.makedirs(URDF_OUT_DIR, exist_ok=True)
    urdf_out = os.path.join(URDF_OUT_DIR, f'daron_{session_name}.urdf')
    try:
        xml = subprocess.run(['xacro', xacro_in, f'mount:={mount}'],
                             check=True, capture_output=True, text=True).stdout
    except subprocess.CalledProcessError as exc:
        raise RuntimeError(f'xacro 실패 ({xacro_in}):\n{exc.stderr}') from exc
    with open(urdf_out, 'w', encoding='utf-8') as f:
        f.write(xml)
    cam_xyz, cam_rpy = _urdf_joint(xml, 'base_to_camera')
    _imu_xyz, imu_rpy = _urdf_joint(xml, 'base_to_imu')
    actions.append(LogInfo(msg=f'[urdf] xacro mount:={mount} ({mount_why}) -> {urdf_out} ({len(xml)} bytes) | '
                               f'camera xyz={cam_xyz} rpy={cam_rpy} | imu rpy={imu_rpy} | '
                               f'IMU 장착 출처={imu_mount_source}'
                               + (' (출발 기울기 보존 → /ins/start_attitude → SLAM 원점)' if imu_mount_source == 'tf'
                                  else ' (옛 동작: 출발 = 수평 가정)')))

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        emulate_tty=True,
        additional_env=ros_env,
        # ⚠ positional URDF 경로는 반드시 --ros-args 앞. launch_ros 의 Node 는
        #   arguments 를 먼저 깔고 그 뒤에 --ros-args 블록을 붙이므로 이 형태가 안전하다.
        arguments=[urdf_out],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ══════════════════════════════════════════════════════════════
    # 2. d405_cuvslam_real.py   (venv 인터프리터 + PYTHONPATH)
    # ══════════════════════════════════════════════════════════════
    # Node() 를 못 쓰는 이유: 인터프리터를 고를 수단이 없다. anaconda 의
    # python3(3.13)로 잡히면 pycuvslam(cp310 .so)이 ImportError 로 죽는다.
    if not os.path.isfile(CUVSLAM_SCRIPT):
        raise RuntimeError(f'cuVSLAM 스크립트가 없습니다: {CUVSLAM_SCRIPT}')
    if not os.path.isfile(VENV_PY):
        raise RuntimeError(f'venv 인터프리터가 없습니다: {VENV_PY}')
    if not os.path.isdir(PYCUVSLAM_DIR):
        raise RuntimeError(f'pycuvslam .so 디렉터리가 없습니다: {PYCUVSLAM_DIR}')

    # [261008] cuVSLAM Slam (루프 클로저·맵 저장·localize_in_map). 켜면 rig 는 OpenCV 축(재추정 탐색이 그 축을 가정),
    #          map->odom 은 Slam 이 내고 아래 항등 static TF 는 안 띄운다.
    slam_on = _as_bool(cfg('slam'), 'slam')
    slam_load = cfg('slam_map_load').strip()
    if slam_load:
        slam_load = os.path.abspath(os.path.expanduser(slam_load))
        if not os.path.isfile(os.path.join(slam_load, 'daron_slam_meta.json')):
            raise RuntimeError(f'slam_map_load 에 daron_slam_meta.json 이 없습니다: {slam_load}')
        if not slam_on:
            raise RuntimeError('slam_map_load 는 slam:=true 가 필요합니다')
    slam_save = cfg('slam_map_save').strip()
    if slam_save == 'auto':
        if source == 'bag':
            slam_save = ''                         # 재생은 기본 저장 안 함 (명시하면 저장)
        elif record:
            slam_save = os.path.join(bag_dir, session_name, 'slam_map')   # 종료 때 저장 — 녹화가 이미 폴더를 만든 뒤
        else:
            slam_save = os.path.join(os.path.dirname(ROOT), 'maps', f'slam_{session_name}')
    elif slam_save:
        slam_save = os.path.abspath(os.path.expanduser(slam_save))
    slam_args = []
    if slam_on:
        slam_args = ['-p', 'slam_enable:=true', '-p', 'rig_convention:=cv',
                     '-p', 'slam_map_save_dir:=' + (slam_save or "''"),
                     '-p', 'slam_map_load_dir:=' + (slam_load or "''"),
                     '-p', f'lc_history:={"true" if _as_bool(cfg("lc_resync"), "lc_resync") else "false"}',   # [261011]
                     '-p', f'lc_robot:={"true" if _as_bool(cfg("lc_robot"), "lc_robot") else "false"}']
    actions.append(LogInfo(msg=f'[slam] {"켬" if slam_on else "끔"}'
                               + (f' | 저장 {slam_save or "(안 함)"} | 재추정 {slam_load or "(새 맵)"} | rig cv | TF map->odom = Slam'
                                  if slam_on else ' | map->odom = 항등 static TF (옛 동작)')))

    cuvslam = ExecuteProcess(
        name='d405_cuvslam',
        cmd=[
            VENV_PY, CUVSLAM_SCRIPT,
            '--ros-args',
            '-p', 'enable_rerun:=False',
            # ⚠ tf_mode:=none 고정. odom_base 로 두면 EKF 도 odom->base_link 를
            #   발행해 TF 가 두 소스에서 진동한다.
            '-p', 'tf_mode:=none',
            '-p', f'use_sim_time:={"true" if use_sim_time else "false"}',
            '-p', f'viz_max_range:={viz_max_range}',   # [viz] 표시용 랜드마크 거리 필터(m)
        ] + [  # [261008] 카메라 장착값 = 생성한 URDF 의 base_to_camera
            a for k, v in zip(('cam_x', 'cam_y', 'cam_z', 'cam_roll', 'cam_pitch', 'cam_yaw'), cam_xyz + cam_rpy)
            for a in ('-p', f'{k}:={_double_arg(v, k)}')
        ] + (['-p', 'start_attitude_topic:=/ins/start_attitude'] if imu_mount_source == 'tf' else []) + slam_args,
        additional_env=cuvslam_env,
        output='screen',
        emulate_tty=True,
    )

    # ══════════════════════════════════════════════════════════════
    # 2b. imu_cov_relay.py  (시스템 python3, 나머지와 같은 ROS 환경)
    # ══════════════════════════════════════════════════════════════
    # 공분산은 노드 기본값(0.01) 그대로. 실측값을 쓰려면 imu_relay:=false 로
    # 끄고 노드를 따로 띄운다 — launch 인자로 빼지 않는다.
    # bag / local / remote 모든 모드에서 띄운다: remote 도 CV7 이 /ekf/imu/data 를
    # 내면 릴레이가 변환해야 EKF 가 받는다.
    relay = None
    if imu_relay:
        if not imu_relay_in or not imu_relay_out:
            raise RuntimeError('imu_relay_in / imu_relay_out 은 비울 수 없습니다.')
        if not os.path.isfile(IMU_RELAY_SCRIPT):
            raise RuntimeError(f'imu_cov_relay 스크립트가 없습니다: {IMU_RELAY_SCRIPT}')
        relay = ExecuteProcess(
            name='imu_cov_relay',
            cmd=['/usr/bin/python3', IMU_RELAY_SCRIPT,
                 '--ros-args',
                 '-p', f'input_topic:={imu_relay_in}',
                 '-p', f'output_topic:={imu_relay_out}',
                 '-p', f'use_sim_time:={"true" if use_sim_time else "false"}'],
            additional_env=ros_env,
            output='screen',
            emulate_tty=True,
        )
        actions.append(LogInfo(msg=f'[imu_relay] {imu_relay_in} -> {imu_relay_out} (공분산 노드 기본값)'))
    else:
        actions.append(LogInfo(
            msg='⚠ imu_relay:=false — /imu/data_fixed 발행자가 없으면 EKF 는 IMU 없이 '
                '돌고 /odometry/filtered 는 계속 나온다 (자세 보정만 조용히 빠짐). '
                '릴레이를 따로 띄웠는지 확인.'))

    # ══════════════════════════════════════════════════════════════
    # 3. ekf_node  (include)
    # ══════════════════════════════════════════════════════════════
    if not os.path.isfile(ekf_launch):
        raise RuntimeError(f'ekf.launch.py 가 없습니다: {ekf_launch}')
    ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ekf_launch),
        launch_arguments={
            'params_file': ekf_params,
            'use_sim_time': 'true' if use_sim_time else 'false',
            'publish_tf': 'true' if tf_source == 'A' else 'false',     # [tf_source]
            'ros_domain_id': domain_id,                                 # [261017] EKF-A 만 도메인이 안 넘어가던 문제
        }.items(),
    )

    # ══════════════════════════════════════════════════════════════
    # 3b. [ins] ins_odom_node.py — INS 단독 이중적분 (ZUPT 없음)
    # ══════════════════════════════════════════════════════════════
    ins_node = None
    if ins_enable:
        if not os.path.isfile(INS_SCRIPT):
            raise RuntimeError(f'ins_odom_node 스크립트가 없습니다: {INS_SCRIPT}')
        ins_node = ExecuteProcess(
            name='ins_odom',
            cmd=['/usr/bin/python3', INS_SCRIPT, '--ros-args',
                 '-p', f'imu_topic:={ins_imu_topic}',
                 '-p', f'static_seconds:={ins_static_s}',
                 '-p', f'use_sim_time:={"true" if use_sim_time else "false"}'],
            additional_env=ros_env, output='screen', emulate_tty=True)
        actions.append(LogInfo(msg=f'[ins] {ins_imu_topic} -> /ins/odometry (정지 {ins_static_s}s 캘리브, ZUPT 없음)'))

    # ══════════════════════════════════════════════════════════════
    # 3c. [recalib] imu_recalib_relay.py + EKF-B (ekf_filter_node_recalib)
    # ══════════════════════════════════════════════════════════════
    # 릴레이는 imu_cov_relay 출력(/imu/data_fixed) 뒤에 체인으로 붙는다. imu_relay:=false 면
    # /imu/data_fixed 를 밖에서 띄운 것으로 보고 그냥 구독한다.
    recalib_relay = None
    ekf_b = None
    if recalib_enable:
        if not os.path.isfile(RECALIB_SCRIPT):
            raise RuntimeError(f'imu_recalib_relay 스크립트가 없습니다: {RECALIB_SCRIPT}')
        if not os.path.isfile(ekf_recalib_params):
            raise RuntimeError(f'ekf_recalib.yaml 이 없습니다: {ekf_recalib_params}')
        recalib_relay = ExecuteProcess(
            name='imu_recalib_relay',
            cmd=['/usr/bin/python3', RECALIB_SCRIPT, '--ros-args',
                 '-p', f'input_topic:={imu_relay_out}',
                 '-p', 'output_topic:=/imu/data_recalib',
                 '-p', f'recalib_apply:={"true" if recalib_apply else "false"}',
                 '-p', f'body_align:={"true" if recalib_body_align else "false"}',
                 '-p', f'mount_source:={imu_mount_source}',                  # [261008]
                 '-p', f'imu_roll:={_double_arg(imu_rpy[0], "imu_roll")}',
                 '-p', f'imu_pitch:={_double_arg(imu_rpy[1], "imu_pitch")}',
                 '-p', f'imu_yaw:={_double_arg(imu_rpy[2], "imu_yaw")}',
                 '-p', f'apply_dpitch:={"true" if recalib_dpitch else "false"}',
                 '-p', f'flip_ratio:={recalib_params["recalib_flip_ratio"]}',
                 '-p', f'hold_s:={recalib_params["recalib_hold_s"]}',
                 '-p', f'static_s:={recalib_params["recalib_static_s"]}',
                 '-p', f'gyro_thr:={recalib_params["recalib_gyro_thr"]}',
                 '-p', f'acc_std_thr:={recalib_params["recalib_acc_std_thr"]}',
                 '-p', f'max_wait_s:={recalib_params["recalib_max_wait_s"]}',
                 '-p', f'use_sim_time:={"true" if use_sim_time else "false"}'],
            additional_env=ros_env, output='screen', emulate_tty=True)
        # EKF-B: include 대신 Node 직접 — 이름·리매핑이 A 와 달라야 해서 ekf.launch.py 를 못 쓴다.
        ekf_b = Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node_recalib',       # ⚠ ekf_recalib.yaml 최상위 키와 일치
            output='screen', emulate_tty=True, additional_env=ros_env,
            parameters=[ekf_recalib_params, {'use_sim_time': use_sim_time}],
            remappings=[('odometry/filtered', '/odometry/filtered_recalib'),
                        ('accel/filtered', '/accel/filtered_recalib')])
        actions.append(LogInfo(
            msg=f'[recalib] {imu_relay_out} -> /imu/data_recalib -> EKF-B(/odometry/filtered_recalib) '
                f'apply={recalib_apply} body_align={recalib_body_align} dpitch={recalib_dpitch} '
                + ' '.join(f'{k[8:]}={v}' for k, v in recalib_params.items())))

    # ══════════════════════════════════════════════════════════════
    # 3c-2. [health] health_monitor.py + EKF-C (ekf_filter_node_health)
    # ══════════════════════════════════════════════════════════════
    health_node = None
    ekf_c = None
    if health_enable:
        if not os.path.isfile(HEALTH_SCRIPT):
            raise RuntimeError(f'health_monitor 스크립트가 없습니다: {HEALTH_SCRIPT}')
        if not os.path.isfile(ekf_health_params):
            raise RuntimeError(f'ekf_health.yaml 이 없습니다: {ekf_health_params}')
        health_node = ExecuteProcess(
            name='health_monitor',
            cmd=['/usr/bin/python3', HEALTH_SCRIPT, '--ros-args',
                 '-p', f'imu_diag_topic:={imu_relay_in}',
                 '-p', 'imu_in_topic:=/imu/data_recalib',
                 '-p', f'realign_enable:={"true" if health_realign else "false"}',
                 '-p', f'realign_thresh_m:={health_realign_thresh}',
                 '-p', f'extrinsic_mode:={health_extrinsic}',
                 '-p', f'use_sim_time:={"true" if use_sim_time else "false"}'],
            additional_env=ros_env, output='screen', emulate_tty=True)
        ekf_c = Node(
            package='robot_localization', executable='ekf_node',
            name='ekf_filter_node_health',        # ⚠ ekf_health.yaml 최상위 키와 일치
            output='screen', emulate_tty=True, additional_env=ros_env,
            parameters=[ekf_health_params, {'use_sim_time': use_sim_time},
                        {'publish_tf': tf_source == 'C'}],          # [tf_source] 기본 C = 이 EKF 가 TF 를 낸다
            # ⚠ set_pose 는 상대 이름이라 리매핑 안 하면 A/B 와 같은 /set_pose 를 듣는다 → 재정렬이 A/B 까지 리셋.
            remappings=[('odometry/filtered', '/odometry/filtered_health'),
                        ('accel/filtered', '/accel/filtered_health'),
                        ('set_pose', '/ekf_filter_node_health/set_pose')])
        actions.append(LogInfo(
            msg=f'[health] {imu_relay_in}+/slam/status 진단 → /imu/data_h, /slam/odometry_h → EKF-C'
                f'(/odometry/filtered_health) realign={health_realign} >{health_realign_thresh} m '
                f'extrinsic={health_extrinsic}'))
    actions.append(LogInfo(msg=f'[tf_source] odom→base_link TF = EKF-{tf_source} '
                               f'({"health 기반 EKF-C" if tf_source == "C" else "기존 EKF-A"}) — 나머지 EKF 는 토픽만'))

    # ══════════════════════════════════════════════════════════════
    # 3d. [ins/recalib] odom_to_path.py — RViz 선
    # ══════════════════════════════════════════════════════════════
    path_node = None
    path_pairs = []
    if ins_enable:
        path_pairs.append(('/ins/odometry', '/ins/path'))
    if recalib_enable:
        path_pairs.append(('/odometry/filtered_recalib', '/visual_slam/tracking/recalib_path'))
    if health_enable:
        path_pairs.append(('/odometry/filtered_health', '/visual_slam/tracking/health_path'))
    if path_pairs:
        if not os.path.isfile(ODOM_TO_PATH_SCRIPT):
            raise RuntimeError(f'odom_to_path 스크립트가 없습니다: {ODOM_TO_PATH_SCRIPT}')
        path_node = ExecuteProcess(
            name='odom_to_path',
            cmd=['/usr/bin/python3', ODOM_TO_PATH_SCRIPT, '--ros-args',
                 '-p', 'odom_topics:=[' + ', '.join(o for o, _ in path_pairs) + ']',
                 '-p', 'path_topics:=[' + ', '.join(p for _, p in path_pairs) + ']',
                 '-p', f'use_sim_time:={"true" if use_sim_time else "false"}'],
            additional_env=ros_env, output='screen', emulate_tty=True)

    # ══════════════════════════════════════════════════════════════
    # 4. ros2 bag record
    # ══════════════════════════════════════════════════════════════
    recorder = None
    if record:
        topics = _read_topic_list(topic_list_file)
        out_dir = os.path.join(bag_dir, session_name)
        # record_baseline.sh 와 같은 판정: 세션 폴더가 이미 있으면 중단.
        # rosbag2 도 거부하지만, 먼저 죽여서 나머지 노드를 안 띄우는 편이 낫다.
        if os.path.exists(out_dir):
            raise RuntimeError(
                f'세션 폴더가 이미 있습니다: {out_dir}\n'
                '  session_name 을 바꾸거나 기존 폴더를 치우세요.')
        os.makedirs(bag_dir, exist_ok=True)
        # [261008] 장착 세트를 bag 옆에 남긴다 — 재생(mount:=auto)이 같은 값으로 URDF 를 다시 만든다.
        #          bag 폴더 안은 ros2 bag record 가 직접 만들어야 해서(있으면 거부) 옆 파일로 둔다.
        with open(out_dir + META_SUFFIX, 'w', encoding='utf-8') as f:
            yaml.safe_dump({'mount': mount, 'imu_mount_source': imu_mount_source,
                            'camera_xyz': cam_xyz, 'camera_rpy': cam_rpy, 'imu_rpy': imu_rpy,
                            'created': datetime.now().isoformat(timespec='seconds')},
                           f, allow_unicode=True, sort_keys=False)
        cmd = ['ros2', 'bag', 'record', '-o', out_dir]
        if use_sim_time:
            cmd.append('--use-sim-time')
        cmd += topics
        recorder = ExecuteProcess(
            name='bag_record',
            cmd=cmd,
            additional_env=ros_env,
            output='screen',
            emulate_tty=True,
        )
        actions.append(LogInfo(
            msg=f'[record] -> {out_dir} / 토픽 {len(topics)}개: {" ".join(topics)}'))

    # ══════════════════════════════════════════════════════════════
    # 5. 소스: 실카메라 / bag 재생 / remote(RADXA)
    # ══════════════════════════════════════════════════════════════
    source_proc = None       # 즉시 시작 (카메라)
    delayed_source = None    # cuVSLAM 뜬 뒤 시작 (bag play)
    remote_check = None      # 즉시 시작 (remote 점검, 끝나면 스스로 종료)
    image_relay = None       # 즉시 시작 (remote 영상 중계, 261008)
    depth_receiver = None    # 즉시 시작 (remote zstd depth 풀어 재발행, 261011)

    if source == 'remote':
        # 카메라·IMU 노드를 띄우지 않는다. 대신 점검 프로세스를 하나 띄워
        # ROS_DOMAIN_ID / 토픽 도달 / 시계 오차를 로그로 크게 남긴다.
        # 점검은 항상 rc=0 으로 끝난다 — RADXA 가 늦게 떠도 launch 는 계속 간다.
        # launch_setup 안에서 직접 기다리지 않는 이유: 여기는 동기 코드라
        # 30s 를 블록하면 rsp/cuVSLAM/EKF 기동까지 전부 미뤄진다.
        if not os.path.isfile(REMOTE_CHECK_SCRIPT):
            raise RuntimeError(f'remote 점검 스크립트가 없습니다: {REMOTE_CHECK_SCRIPT}')
        remote_check = ExecuteProcess(
            name='remote_check',
            cmd=['/usr/bin/python3', REMOTE_CHECK_SCRIPT,
                 '--ros-args',
                 '-p', f'timeout_s:={remote_timeout_s}',
                 '-p', f'max_skew_ms:={remote_skew_ms}',
                 '-p', 'use_sim_time:=false'],
            additional_env=ros_env,
            output='screen',
            emulate_tty=True,
        )
        # [remote 261008] WiFi 로 영상을 한 벌만 받는다. 지상국 구독 프로세스마다 한 벌씩 오가서
        # 구독 4~5개일 때 30 Hz → 3~6 Hz, 지연 0.2 → 0.8~1.5 s (SLAM dead → EKF-C IMU-only, TSDF 정지).
        # RADXA 는 rs_launch.py ... camera_namespace:=radxa 로 띄워야 한다 (옛 명령이면 중계할 게 없어
        # 직접 수신으로 동작하고 image_relay 가 경고만 찍는다).
        if remote_relay:
            if not os.path.isfile(IMAGE_RELAY_SCRIPT):
                raise RuntimeError(f'영상 중계 스크립트가 없습니다: {IMAGE_RELAY_SCRIPT}')
            # [261011] remote_depth:=zstd 면 depth 영상은 video_receiver 가 내므로 중계에서 뺀다
            #   (안 빼면 발행자 둘 + raw depth 가 WiFi 로 계속 옴). depth/camera_info 는 계속 중계.
            streams = [s for s in RELAY_STREAMS_ALL
                       if not (remote_depth == 'zstd' and s == 'depth/image_rect_raw')]
            image_relay = ExecuteProcess(
                name='image_relay',
                cmd=['/usr/bin/python3', IMAGE_RELAY_SCRIPT,
                     '--ros-args',
                     '-p', f'in_prefix:={remote_cam_prefix}',
                     '-p', 'out_prefix:=/camera/camera',
                     '-p', f'streams:=[{", ".join(streams)}]',
                     '-p', 'use_sim_time:=false'],
                additional_env=ros_env,
                output='screen',
                emulate_tty=True,
            )
            actions.append(LogInfo(
                msg=f'[image_relay] {remote_cam_prefix}/* -> /camera/camera/* ({len(streams)}개 스트림'
                    + (', depth 영상 제외 — zstd 수신' if remote_depth == 'zstd' else '') + ') '
                    f'(RADXA 카메라는 camera_namespace:={remote_cam_prefix.strip("/").split("/")[0]} 로 띄울 것)'))
        else:
            actions.append(LogInfo(
                msg='⚠ remote_relay:=false — 지상국 구독 프로세스마다 WiFi 로 영상이 따로 온다 (261008 지연 원인)'))
        if remote_depth == 'zstd':
            if not os.path.isfile(VIDEO_RECEIVER_SCRIPT):
                raise RuntimeError(f'zstd depth 수신 스크립트가 없습니다: {VIDEO_RECEIVER_SCRIPT}')
            depth_receiver = ExecuteProcess(
                name='depth_zstd_receiver',
                cmd=[VENV_PY, VIDEO_RECEIVER_SCRIPT,
                     '--depth_mode=zstd', '--infra1_mode=off', '--infra2_mode=off',
                     f'--depth_zstd_topic={remote_depth_zstd_topic}',
                     '--republish=True', '--republish_prefix=/camera/camera',
                     '--display=False', '--save=False', '--stats_every=5'],
                additional_env=ros_env,
                output='screen',
                emulate_tty=True,
            )
            actions.append(LogInfo(
                msg=f'[depth_zstd] {remote_depth_zstd_topic} (zstd) -> /camera/camera/depth/image_rect_raw (16UC1)'
                    + ('' if remote_relay else
                       ' ⚠ remote_relay:=false — RADXA 가 namespace 없이 depth 를 내면 발행자가 둘이 됩니다')))
    elif source == 'bag':
        # replay_exclude(기본 DEFAULT_REPLAY_EXCLUDE) 제외: 지금 살아서 도는
        # 노드가 발행하는 토픽을 bag 이 다시 뿌리면 발행자가 둘이 된다.
        #   /tf_static          robot_state_publisher 와 이중 등록
        #   /tf                 EKF 가 발행 — 브로드캐스터 둘이면 조용히 진동
        #   /slam/odometry      cuVSLAM 이 지금 계산해 발행 (bag 의 옛 값과 섞임)
        #   /odometry/filtered  EKF 산출물 (동일)
        #   /imu/data_fixed     imu_cov_relay 가 지금 bag 의 /ekf/imu/data 로부터 재생성
        kept, dropped = _bag_topics(bag_path, exclude=replay_exclude)
        actions.append(LogInfo(
            msg=f'[bag play] {bag_path} / rate {bag_rate}x / 재생 {len(kept)}개, 제외: {dropped or "없음"}'))
        delayed_source = ExecuteProcess(
            name='bag_play',
            cmd=['ros2', 'bag', 'play', bag_path,
                 '--clock', '--rate', bag_rate,
                 '--topics'] + kept,
            additional_env=ros_env,
            output='screen',
            emulate_tty=True,
        )
    else:
        # 노출/게인/프로파일은 CLAUDE.md 260903 확정값.
        # rs_launch.py include 대신 Node 를 직접 띄운다: rs_launch.py 는 지원하지
        # 않는 인자를 노란 경고만 찍고 노드에 안 넘겨서(실제로 infra_profile 이
        # 그렇게 누락된 적 있음) 조용히 기본 프로파일로 도는 사고가 난다.
        # 여기서는 파라미터가 노드로 직행한다.
        # global_settings.yaml 의 use_lifecycle_node=false 확인 → 일반 Node 로 충분.
        source_proc = Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            namespace='camera',          # rs_launch.py 기본값과 동일
            name='camera',               #  → /camera/camera/infra1/... 유지
            output='screen',
            emulate_tty=True,
            additional_env=ros_env,
            parameters=[{
                'enable_infra1': True,
                'enable_infra2': True,
                'enable_depth': True,
                'enable_color': False,
                # ⚠ depth_module.profile 은 4.58.3 에서 없어진 인자.
                #   depth_profile / infra_profile 로 분리됨.
                'depth_module.depth_profile': '848x480x30',
                'depth_module.infra_profile': '848x480x30',
                'depth_module.enable_auto_exposure': False,
                'depth_module.exposure': 8500,
                'depth_module.gain': 16,
                # depth_module.emitter_enabled 는 넣지 않는다 — D405 는 IR
                # 프로젝터가 없어 파라미터 자체가 존재하지 않는다.
                'use_sim_time': use_sim_time,
            }],
        )

    # ══════════════════════════════════════════════════════════════
    # 기동 순서 + on_exit 로거
    # ══════════════════════════════════════════════════════════════
    # EKF / bag record / bag play 는 마지막 게이트(릴레이가 있으면 릴레이,
    # 없으면 cuVSLAM)가 뜬 뒤에 시작한다. 릴레이를 cuVSLAM 과 같은 배치에 넣지
    # 않고 한 단계 뒤에 둔 이유: 같은 배치면 릴레이와 EKF 의 선후를 이벤트로
    # 보장할 수 없어 bag 앞부분 IMU 를 EKF 가 놓칠 수 있다.
    final_stage = [ekf]
    if ekf_b is not None:
        final_stage.append(ekf_b)
    if ekf_c is not None:
        final_stage.append(ekf_c)
    if recorder is not None:
        final_stage.append(recorder)
    if delayed_source is not None:
        final_stage.append(delayed_source)
    final_desc = ('EKF' + ('-A/B' if ekf_b is not None else '') + ('/C' if ekf_c is not None else '')
                  + ('/bag record' if recorder is not None else '')
                  + ('/bag play' if delayed_source is not None else ''))
    # 단계: rsp → cuVSLAM → [imu_cov_relay] → [imu_recalib_relay] → final
    n_steps = (2 + (1 if relay is not None else 0) + (1 if recalib_relay is not None else 0)
               + (1 if health_node is not None else 0))
    # cuVSLAM 과 같은 배치에서 뜨는 보조 노드 (IMU 만 필요, 순서 무관)
    with_cuvslam = [n for n in (ins_node, path_node) if n is not None]

    # [viz] map -> odom 항등 static TF (라벨용). odom -> base_link 는 여전히 EKF 단독.
    map_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_odom_identity_tf',
        output='screen',
        emulate_tty=True,
        additional_env=ros_env,
        # Humble 은 위치 인자(x y z yaw pitch roll parent child)가 deprecated 경고를 찍으므로 플래그형으로.
        # 지정하지 않은 x/y/z/roll/pitch/yaw 는 0 = 항등.
        arguments=['--frame-id', 'map', '--child-frame-id', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}],
    )
    if slam_on:
        map_odom_tf = None
    else:
        actions.append(LogInfo(msg='[viz] static TF map -> odom (identity) 기동'))

    actions += [rsp] + ([map_odom_tf] if map_odom_tf is not None else []) + [

        RegisterEventHandler(OnProcessStart(
            target_action=rsp,
            on_start=[
                LogInfo(msg=f'[1/{n_steps}] robot_state_publisher 기동 → cuVSLAM 시작'),
                cuvslam,
            ] + with_cuvslam,
        )),
    ]
    # 릴레이 체인: cuVSLAM → imu_cov_relay → imu_recalib_relay → final. 빠진 단계는 건너뛴다.
    chain = [('cuVSLAM', cuvslam)]
    if relay is not None:
        chain.append(('imu_cov_relay', relay))
    if recalib_relay is not None:
        chain.append(('imu_recalib_relay', recalib_relay))
    if health_node is not None:
        chain.append(('health_monitor', health_node))   # 릴레이 출력(/imu/data_h, /slam/odometry_h) 이 EKF-C 보다 먼저
    for i in range(len(chain)):
        label, proc = chain[i]
        if i + 1 < len(chain):
            nxt_label, nxt = chain[i + 1]
            actions.append(RegisterEventHandler(OnProcessStart(
                target_action=proc,
                on_start=[LogInfo(msg=f'[{i + 2}/{n_steps}] {label} 기동 → {nxt_label} 시작'), nxt])))
        else:
            actions.append(RegisterEventHandler(OnProcessStart(
                target_action=proc,
                on_start=[LogInfo(msg=f'[{i + 2}/{n_steps}] {label} 기동 → {final_desc} 시작')] + final_stage)))
    if relay is not None:
        actions += [
            RegisterEventHandler(OnProcessExit(
                target_action=relay,
                on_exit=_on_exit_logger(
                    'imu_cov_relay',
                    f'{imu_relay_out} 발행자가 사라졌습니다 — EKF 는 IMU 없이 계속 돌고 '
                    '/odometry/filtered 도 계속 나오니 자세 보정 누락을 눈치채기 어렵습니다. '
                    '공분산 파라미터가 0 이면 rc=1 로 죽습니다.'))),
        ]
    if recalib_relay is not None:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=recalib_relay,
            on_exit=_on_exit_logger(
                'imu_recalib_relay',
                '/imu/data_recalib 발행자가 사라졌습니다 — EKF-B 는 IMU 없이 계속 돌고 '
                '/odometry/filtered_recalib 도 계속 나오니 A/B 비교가 조용히 무의미해집니다.'))))
    if health_node is not None:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=health_node,
            on_exit=_on_exit_logger(
                'health_monitor',
                '/imu/data_h, /slam/odometry_h 발행자가 사라졌습니다 — EKF-C 는 입력 없이 추정만 계속합니다.'))))
    if ins_node is not None:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=ins_node,
            on_exit=_on_exit_logger('ins_odom', '/ins/odometry 가 멈춥니다 (INS 선 소실).'))))
    if path_node is not None:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=path_node,
            on_exit=_on_exit_logger('odom_to_path', 'RViz 의 INS/recalib Path 가 갱신되지 않습니다.'))))

    actions += [

        RegisterEventHandler(OnProcessExit(
            target_action=rsp,
            on_exit=_on_exit_logger(
                'robot_state_publisher',
                'URDF 경로가 --ros-args 뒤로 갔거나(UnknownROSArgsError) '
                'xacro 산출물이 깨졌을 수 있습니다: ' + urdf_out))),
        RegisterEventHandler(OnProcessExit(
            target_action=cuvslam,
            on_exit=_on_exit_logger(
                'd405_cuvslam',
                'ModuleNotFoundError 면 PYTHONPATH 확인: ' + PYCUVSLAM_DIR))),
    ]
    if image_relay is not None:
        actions.append(image_relay)
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=image_relay,
            on_exit=_on_exit_logger(
                'image_relay',
                '중계가 죽으면 /camera/camera/* 영상이 끊깁니다 — cuVSLAM·TSDF·GUI 전부 멈춤. '
                '위 traceback 확인 후 remote_relay:=false 로 직접 수신 가능(느림).'))))
    if depth_receiver is not None:
        actions.append(depth_receiver)
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=depth_receiver,
            on_exit=_on_exit_logger(
                'depth_zstd_receiver',
                'zstd depth 수신이 죽으면 /camera/camera/depth/image_rect_raw 가 끊깁니다 — TSDF·depth 표시 멈춤. '
                '위 traceback 확인 후 remote_depth:=raw 로 raw 중계 가능(WiFi 대역 큼).'))))
    if remote_check is not None:
        actions.append(remote_check)
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=remote_check,
            on_exit=_on_exit_logger(
                'remote_check',
                fatal_hint='rc≠0 은 점검 프로세스 자체의 이상(파라미터 오류/코드 예외)입니다 — '
                           '토픽이 안 보이는 것은 rc=0 + WARN/FATAL 로그로만 나옵니다. '
                           '위 traceback 과 [remote 소스 점검 결과] 블록을 보세요.',
                ok_hint='점검 완료 — 정상입니다. 토픽 유무는 위 [remote 소스 점검 결과] '
                        '블록의 WARN/FATAL 로 판단하세요 (rc=0 은 토픽 0개여도 나옵니다).'))))
    if source_proc is not None:
        actions.append(source_proc)
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=source_proc,
            on_exit=_on_exit_logger(
                'realsense2_camera',
                'USB 링크(lsusb -t 로 5000M) 와 카메라 점유 프로세스를 확인하세요.'))))
    if delayed_source is not None:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=delayed_source,
            on_exit=_on_exit_logger('bag_play', '재생이 끝난 것일 수도 있습니다(rc=0 이면 정상).'))))
    if recorder is not None:
        actions.append(RegisterEventHandler(OnProcessExit(
            target_action=recorder,
            on_exit=_on_exit_logger(
                'bag_record',
                '토픽 이름 오타나 디스크 부족(<5GB)을 먼저 의심하세요.'))))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'session_name', default_value='',
            description='bag 세션명. 비워두면 YYMMDD_HHMMSS 타임스탬프.'),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='전 노드 공통. bag_path 지정 시 자동으로 true 로 올라간다.'),
        DeclareLaunchArgument(
            'record', default_value='true',
            description='ros2 bag record 실행 여부 (true/false).'),
        DeclareLaunchArgument(
            'bag_path', default_value='',
            description='지정하면 카메라 대신 이 bag 을 --clock --rate <bag_rate> 로 재생.'),
        DeclareLaunchArgument(
            'bag_rate', default_value='1.0',
            description='[viz] bag 재생 배속. 원본 bringup 은 0.3 고정(근거 기록 없음). 260921 정배속 실측 후 1.0 을 기본으로.'),
        DeclareLaunchArgument(
            'source', default_value='local',
            description='local: 이 PC 의 D405 를 직접 기동 / remote: 카메라·IMU 를 안 띄우고 '
                        'RADXA 도커 발행분을 받는다 (rsp/cuVSLAM/EKF/녹화만, use_sim_time=false). '
                        'bag_path 와 remote 는 같이 못 쓴다.'),
        DeclareLaunchArgument(
            'remote_wait_s', default_value='30.0',
            description='source:=remote 점검이 카메라/IMU 토픽을 기다리는 최대 초. '
                        '넘기면 경고만 하고 중단하지 않는다.'),
        DeclareLaunchArgument(
            'remote_relay', default_value='true',
            description='[261008] source:=remote 일 때 image_relay.py 로 RADXA 영상을 WiFi 1벌만 받아 '
                        '/camera/camera/* 로 재발행. RADXA 카메라는 camera_namespace:=radxa 로 띄운다.'),
        DeclareLaunchArgument(
            'remote_depth', default_value='zstd',
            description='[261011] source:=remote 의 depth 영상 수신 방식. zstd = RADXA 송신기의 zstd CompressedImage '
                        '(remote_depth_zstd_topic)를 depth_transport/video_receiver.py 가 풀어 '
                        '/camera/camera/depth/image_rect_raw(16UC1)로 재발행, image_relay 는 depth 영상만 뺌 / '
                        'raw = 옛 동작(image_relay 가 raw depth 중계)'),
        DeclareLaunchArgument(
            'remote_depth_zstd_topic', default_value='/camera/transport/depth_zstd',
            description='[261011] remote_depth:=zstd 일 때 RADXA zstd depth 토픽'),
        DeclareLaunchArgument(
            'remote_cam_prefix', default_value='/radxa/camera',
            description='[261008] image_relay 입력 접두어 = RADXA realsense 의 /<camera_namespace>/<camera_name>.'),
        DeclareLaunchArgument(
            'remote_max_skew_ms', default_value='100.0',
            description='source:=remote 점검의 stamp 지연 합계(오프셋+전송+처리) 경고 문턱(ms). '
                        '200ms(EKF sensor_timeout) 초과는 별도로 "전부 버림" 경고.'),
        DeclareLaunchArgument(
            'ros_domain_id', default_value='42',
            description='5개 프로세스 전부에 적용. slam_env.sh 값(42)에 맞춘 기본값 — '
                        'cuVSLAM 만 42 고 나머지가 0 이면 토픽이 조용히 안 오간다.'),
        DeclareLaunchArgument(
            'urdf', default_value=DEFAULT_XACRO,
            description='xacro 입력 파일.'),
        DeclareLaunchArgument(
            'rebuild', default_value='false',
            description='[261008] bag 재생에서 녹화된 맵·판정·경로계획·GUI·핀 토픽도 뺀다 — daron_gui(map:=true)로 지금 코드가 '
                        '다시 계산한 결과만 보이게. 녹화 당시 결과를 그대로 볼 때는 false(daron_gui map:=false).'),
        DeclareLaunchArgument(
            'slam', default_value='true',
            description='[261008] cuVSLAM Slam(루프 클로저·map->odom TF·맵 저장·재추정). false = 옛 동작(VO 만, map->odom 항등).'),
        DeclareLaunchArgument(
            'lc_resync', default_value='true',
            description='[261011] 루프 클로저 뒤 보정된 과거 궤적(/slam/map_odom_history)을 낸다 — daron_3dmap 이 피쳐맵을 '
                        '다시 쌓는 데 씀(그쪽 lc_resync 도 true). slam:=true 필요. false = 옛 동작.'),
        DeclareLaunchArgument(
            'lc_robot', default_value='true',
            description='[261011 밤] 루프 클로저 때 로버 TF(map->odom)도 같은 보정 궤적을 따름 — 지도·핀·웨이포인트와 로버 위치가 맞음 '
                        '(그 순간 로버가 보정량만큼 옮겨짐). false = 로버 TF 는 Slam.track 그대로(1 % 미만 보정 안 따름).'),
        DeclareLaunchArgument(
            'slam_map_load', default_value='',
            description='[261008] 재추정할 Slam 맵 폴더(daron_slam_meta.json 있는 곳). 맵을 만든 실행의 출발 표시에 정지한 채 기동할 것.'),
        DeclareLaunchArgument(
            'slam_map_save', default_value='auto',
            description='[261008] Slam 맵 저장 폴더(종료 시). auto: 녹화면 <bag_dir>/<session>/slam_map, 실시간 녹화 없음이면 '
                        '~/Daron/maps/slam_<session>, bag 재생이면 저장 안 함. 빈 값 = 저장 안 함.'),
        DeclareLaunchArgument(
            'mount', default_value='auto',
            description='[261008] 장착 세트 auto|level|cad0902. auto: 실시간·remote = level(설계: D405·CV7 완전 수평), '
                        'bag = <bag>.daron_meta.yaml 의 값, 없으면 cad0902(10/8 이전 bag).'),
        DeclareLaunchArgument(
            'imu_mount_source', default_value='auto',
            description='[261008] 릴레이 장착 자세 출처 auto|tf|g0. auto: level→tf(URDF 값, 출발 기울기 보존 → '
                        'SLAM 원점 자세), cad0902→g0(옛 동작: 출발 = 수평 가정).'),
        DeclareLaunchArgument(
            'ekf_launch', default_value=DEFAULT_EKF_LAUNCH,
            description='include 할 ekf.launch.py.'),
        DeclareLaunchArgument(
            'ekf_params', default_value=DEFAULT_EKF_PARAMS,
            description='EKF 파라미터 yaml.'),
        DeclareLaunchArgument(
            'record_topics', default_value=DEFAULT_TOPIC_LIST,
            description='녹화 토픽 목록 파일 (한 줄에 하나, # 주석 허용).'),
        DeclareLaunchArgument(
            'bag_dir', default_value=DEFAULT_BAG_DIR,
            description='bag 출력 루트. 실제 경로는 <bag_dir>/<session_name>.'),
        DeclareLaunchArgument(
            'imu_relay', default_value='true',
            description='imu_cov_relay.py 기동 여부. ekf.yaml 의 imu0(/imu/data_fixed)를 내는 '
                        '유일한 노드 — false 면 따로 띄워야 EKF 에 IMU 가 들어간다.'),
        DeclareLaunchArgument(
            'imu_relay_in', default_value='/ekf/imu/data',
            description='릴레이 입력 토픽 (CV7 필터 IMU).'),
        DeclareLaunchArgument(
            'imu_relay_out', default_value='/imu/data_fixed',
            description='릴레이 출력 토픽. ekf.yaml 의 imu0 와 같아야 한다.'),
        DeclareLaunchArgument(
            'viz_max_range', default_value='2.0',
            description='[viz] 표시용 랜드마크 거리 필터(m). 카메라(rig)로부터 이 거리 이하인 '
                        '랜드마크만 observations_cloud/observations_image 에 그리고, '
                        'landmarks_cloud 는 한 번이라도 통과한 id 만 누적 표시. '
                        '추정(/slam/odometry)에는 영향 없음.'),
        DeclareLaunchArgument(
            'ins', default_value='true',
            description='[ins] ins_odom_node.py 기동 (INS 단독 이중적분, ZUPT 없음) → /ins/odometry, /ins/path.'),
        DeclareLaunchArgument(
            'ins_imu_topic', default_value='/imu/data',
            description='[ins] INS 단독 적분 입력. 논문(analysis_ins_only.py)과 같은 raw 토픽.'),
        DeclareLaunchArgument(
            'ins_static_s', default_value='15.0',
            description='[ins] 기동 후 정지 캘리브레이션 창 [s] (자이로 bias·|g|·초기 자세).'),
        DeclareLaunchArgument(
            'recalib', default_value='true',
            description='[recalib] imu_recalib_relay.py + EKF-B(ekf_filter_node_recalib) 기동 → '
                        '/imu/data_recalib, /odometry/filtered_recalib, recalib_path.'),
        DeclareLaunchArgument(
            'recalib_apply', default_value='true',
            description='[recalib] false 면 전복 감지·이벤트만 내고 IMU 는 그대로 통과(대조군: EKF-A == EKF-B).'),
        DeclareLaunchArgument(
            'recalib_body_align', default_value='true',
            description='[recalib] 시작 중력(g0)으로 장착 자세를 잡아 base_link 자세로 EKF-B 에 준다(imu0_relative=false). '
                        '이것이 전복 뒤 pitch 27° 오차(relative 모드의 world 축 오프셋 제거)를 없애는 본체. '
                        '260921 실측: EKF-B pitch 가 VSLAM 과 1~2° 안, z 드리프트 +0.64 → +0.02 m.'),
        DeclareLaunchArgument(
            'recalib_dpitch', default_value='false',
            description='[recalib] 전복 뒤 정지 창 중력으로 잰 Δθ(피치)를 실제 적용. 기본 false(기록만): '
                        'Δθ 는 장착 변화가 아니라 몸체 자세라 AHRS 와 이중 계산돼 12° 반대로 넘어간다(260921 실측).'),
        DeclareLaunchArgument(
            'recalib_flip_ratio', default_value='0.5',
            description='[recalib] 전복 판정 문턱. s=a·ĝ0/|g0| 가 -r 아래(뒤집힘)/+r 위(정자세). 사이는 보류.'),
        DeclareLaunchArgument(
            'recalib_hold_s', default_value='0.5',
            description='[recalib] 문턱 밖에 연속으로 머물러야 하는 시간 [s] (header stamp 기준).'),
        DeclareLaunchArgument(
            'recalib_static_s', default_value='1.0',
            description='[recalib] 재보정용 정지 창 길이 [s]. viz2_rollover 실측상 전복 뒤 2 s 짜리 정지가 없어 1 s.'),
        DeclareLaunchArgument(
            'recalib_gyro_thr', default_value='0.10',
            description='[recalib] 정지 창 평균 |ω| 상한 [rad/s].'),
        DeclareLaunchArgument(
            'recalib_acc_std_thr', default_value='0.15',
            description='[recalib] 정지 창 std|a| 상한 [m/s²].'),
        DeclareLaunchArgument(
            'recalib_max_wait_s', default_value='10.0',
            description='[recalib] 상태 변경 뒤 정지 창을 기다리는 한도 [s]. 넘기면 가장 조용했던 창으로 보정(relaxed).'),
        DeclareLaunchArgument(
            'ekf_recalib_params', default_value=DEFAULT_EKF_RECALIB_PARAMS,
            description='[recalib] EKF-B 파라미터 yaml (ekf.yaml 과 키/imu0/publish_tf 만 다름).'),
        DeclareLaunchArgument(
            'health', default_value='true',
            description='[health] health_monitor.py + EKF-C(ekf_filter_node_health) 기동 → /health/state, '
                        '/odometry/filtered_health, health_path. recalib:=true 필요.'),
        DeclareLaunchArgument(
            'tf_source', default_value='C',
            description='[260929] odom→base_link TF 를 낼 EKF: C(health 기반, 기본 — 최종 pose) / A(기존). '
                        'C 는 health:=true 필요. 한 EKF 만 TF 를 내야 한다(둘이면 조용히 진동).'),
        DeclareLaunchArgument(
            'health_realign', default_value='true',
            description='[health] health 이벤트 뒤 EKF-C 가 목표(정상 시점 EKF + SLAM 이동분)에서 벗어나면 set_pose.'),
        DeclareLaunchArgument(
            'health_realign_thresh_m', default_value='0.10',
            description='[health] 재정렬 문턱 [m] (260929 결정: 최대 10 cm).'),
        DeclareLaunchArgument(
            'health_extrinsic', default_value='online',
            description='[health] 장착 회전 R_base_imu: online(주행 초반 데이터로 추정, 기본) / tf(URDF). '
                        'URDF 피치 -17.4° 와 데이터 추정 -12~-16° 가 달라 online 이 기본.'),
        DeclareLaunchArgument(
            'ekf_health_params', default_value=DEFAULT_EKF_HEALTH_PARAMS,
            description='[health] EKF-C 파라미터 yaml (ekf_recalib.yaml 과 imu0/odom0/자세 on 만 다름).'),
        DeclareLaunchArgument(
            'replay_exclude', default_value=DEFAULT_REPLAY_EXCLUDE,
            description='bag_path 재생 시 bag 에서 뺄 토픽(공백/쉼표 구분). '
                        '지금 도는 노드가 발행하는 토픽과 겹치지 않게 하는 용도. '
                        '빈 문자열이면 전부 재생.'),

        OpaqueFunction(function=launch_setup),
    ])
