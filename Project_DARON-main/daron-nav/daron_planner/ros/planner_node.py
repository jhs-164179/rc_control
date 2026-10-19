#!/usr/bin/env python3
"""DARON 경로계획 ROS2 노드 (261004, rclpy, ROS2 Humble).

입력
  /traversability/grid  nav_msgs/OccupancyGrid  판정 격자 (odom, 5 cm, 로버 반경 1.25 m, 약 2 s)
  TF odom -> base_link                           EKF-C 위치
  /ui/mode              std_msgs/String           WAYPOINT / AUTO / MANUAL (모드 화면, 석윤님)
  /ui/waypoint_click    geometry_msgs/PointStamped  point.x = u, point.y = v [픽셀], header = 그 영상의 stamp·frame_id
  /ui/waypoint_cmd      std_msgs/String           CLEAR / UNDO / NEXT. [auto-1014] AUTO_RESET(완전 자율 방문 기록 지움) / AUTO_SKIP(지금 점 건너뜀·갇힘 정지 풂)
  /ui/field             std_msgs/String JSON [auto-1014] 경기장 놓기: {"cmd":"here","size":[4,5],"start":[-1.4,-2.07,90]} = 지금 로버(base_link)가 경기장의
                        start 자리(x, y [m], 방향 [deg])에 있다 → 그 자세로 경기장을 다시 놓음(AUTO 중엔 거부). {"cmd":"gap","xy":[x,y]} 틈(경기장 좌표). {"cmd":"show"}
                        [10/15] {"cmd":"grid","cols":3,"rows":3} 방문점 격자 (열 = x 4 m 변, 행 = y 5 m 변, AUTO 중 거부, 방문 기록 지움)
                        [10/15] {"cmd":"clear"} 경기장 풀기 → 연구실 범위 (lab_until_set 일 때만, AUTO 중 거부)
  /ui/gap_point         geometry_msgs/PointStamped [auto-1014] 틈 클릭 (RViz Publish Point 토픽을 이걸로 바꾸거나 GUI 탑뷰 클릭)
  /goal_pose            geometry_msgs/PoseStamped RViz '2D Goal Pose' / 지상국 GUI → 웨이포인트.
                        [10/11] goal_pose_heading 이 true 면 orientation = 도착 방위각 (위치 goal_tol_m 안 그리고 방향 goal_th_tol_deg 안이면 도착).
                        orientation 이 (0, 0, 0, 0) 이면 방향 없음 (위치만). false(기본)면 예전처럼 위치만
  depth 영상 + camera_info                          클릭 픽셀 -> depth -> 3D (파라미터로 토픽 지정)
  /health/state         std_msgs/String (선택)     JSON 이면 health_rules(ins=OK|DEGRADED;slam=OK) 대로, 아니면 'OK' 로 시작해야 정상. 비정상이면 격자 안 쌓음, health_grace_s(2 s) 넘으면 정지
  /traversability/why   nav_msgs/OccupancyGrid (선택) 판정 사유 (v2.2 노드부터). 정지 이유에 '앞: 낙차' 등으로 씀
  /traversability/cloud sensor_msgs/PointCloud2 (선택) 판정 칸 중심 지면 3D 점 (odom). 경로 높이·자세에 씀 [10/7 석윤, height_topic]
  map_cloud_topic       sensor_msgs/PointCloud2 (선택, 기본 끔) 지도 점군 (/live_tsdf/map_cloud). 주면 웨이포인트 번호·도착 원을
                        그 자리 바닥 높이에 그림 (표시용). [10/11 합침] /planner/path·/planner/route 높이는 path_z(위)가 정함 — 이 옵션은 경로 z 를 안 바꿈
  clicked_point_topic   geometry_msgs/PointStamped (선택, 기본 끔) RViz 'Publish Point' 로 3D 지도 위를 찍은 점 = 웨이포인트 (높이 포함)
  /rover/auto/drive_rpm, /rover/auto/steer_cmd  std_msgs/Float64 (선택, 추종기 출력) [10/11 백업] 헛바퀴·회전 부족 감시에 씀.
                        안 오면 '경로를 따라가는 중'·'경로 곡률'로 대신함
출력
  /planner/path         nav_msgs/Path   odom, 두 축 가운데 점, 5 cm, 전진만. 빈 경로 = 정지
                        [10/7] position.z = 그 자리 지면 높이 + 기준점 높이(/localization/pose z 와 같은 점),
                        orientation = 진행 방향(yaw) + 지면 기울기(pitch·roll). 탐색 자체는 평면(x, y, θ) 그대로 (path_z.py)
  /planner/status       std_msgs/String JSON {state, reason, mode, goal, waypoints, seen_ahead, plan_ms, ..., z_known, z_offset, z_offset_src}
                        [auto-1014] auto = {target(점 번호 1부터 | gap), n, visited, skipped, left, order, elapsed_s, boxed, skipped_why} (AUTO 시작 뒤)
                        state AUTO_BOXED = 완전 자율 중 갇힘 (모드 바꾸기·AUTO_SKIP 까지 정지)
  /planner/markers      [auto-1014] ns auto_pt/auto_no = 완전 자율 방문점 (흰 남음·초록 지나감·빨강 건너뜀, 큰 점 = 지금 목표, 파랑 = 틈),
                        ns field = 경기장 테두리(field_size)·출발 자리(start_in_field) 노랑. /planner/status field_pose = base_link 경기장 좌표
  /planner/waypoints    geometry_msgs/PoseArray (odom)
  /planner/route        nav_msgs/Path   목표까지 칸 경로 전체 (표시용)
  /planner/arena_grid   nav_msgs/OccupancyGrid  경기장 전체 누적 판정 + 사전 금지 칸 (표시용)
  /planner/field        std_msgs/String JSON (latched) [auto-1014] 지금 경기장: how(param|here|lab)·size·start·gap·corners_odom·gap_odom (GUI 그리기용)
                        [261015] how=lab = 경기장 놓기 전 연구실 범위 (lab_until_set), start=None

실행 (우분투 지상국):  source ~/Daron/slam_env.sh
  ~/venv/daron3d/bin/python ~/daron-nav/daron_planner/ros/planner_node.py --ros-args --params-file ~/daron-nav/daron_planner/ros/planner_params.yaml
"""
import json
import math
import os
import sys
import time
import traceback
from collections import deque
from dataclasses import replace as dc_replace

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..'))
import planner_core as P          # noqa: E402
import arena_planner as A         # noqa: E402
import waypoint_tools as W        # noqa: E402
import path_z as Z                # noqa: E402
import backup as B                # noqa: E402
import auto_wp as AW              # noqa: E402

import rclpy                                          # noqa: E402
from rclpy.node import Node                           # noqa: E402
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor   # noqa: E402
from rclpy.duration import Duration                   # noqa: E402
from rclpy.time import Time                           # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy   # noqa: E402
from nav_msgs.msg import OccupancyGrid, Path          # noqa: E402
from geometry_msgs.msg import PoseStamped, PoseArray, Pose, PointStamped, Point   # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray   # noqa: E402
from sensor_msgs.msg import Image, CameraInfo, PointCloud2   # noqa: E402
from std_msgs.msg import String, Float64              # noqa: E402
from tf2_ros import Buffer, TransformListener         # noqa: E402

MODES = ('WAYPOINT', 'AUTO', 'MANUAL')


def qos(reliable=True, latched=False, depth=1):
    """받는 쪽은 BEST_EFFORT·VOLATILE 로 둔다 = 어떤 발행 QoS 와도 맞음(QoS 불일치는 오류 없이 조용히 안 옴). 내보내는 쪽은 RELIABLE."""
    return QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=depth,
                      reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL if latched else DurabilityPolicy.VOLATILE)


def stamp_sec(st):
    return st.sec + st.nanosec * 1e-9


def goal_yaw(goal):
    """목표 튜플에서 도착 방향 [rad] (웨이포인트 (x, y, z, yaw)). 방향 없으면 None (AUTO 목표 (x, y) 등)."""
    return goal[3] if goal is not None and len(goal) >= 4 else None


class PlannerNode(Node):
    def __init__(self):
        super().__init__('daron_planner')
        d = self.declare_parameter
        # 토픽·프레임
        d('grid_topic', '/traversability/grid'); d('odom_frame', 'odom'); d('base_frame', 'base_link')
        d('mode_topic', '/ui/mode'); d('click_topic', '/ui/waypoint_click'); d('waypoint_cmd_topic', '/ui/waypoint_cmd')
        d('goal_pose_topic', '/goal_pose')
        d('depth_topic', '/camera/camera/depth/image_rect_raw'); d('camera_info_topic', '/camera/camera/depth/camera_info')
        d('depth_scale', 0.001); d('click_image_size', [0, 0])
        d('health_topic', '/health/state'); d('require_health', False); d('health_timeout_s', 3.0)
        d('health_keys', 'ins,slam'); d('health_ok_values', 'OK')   # (옛 방식) JSON health 에서 볼 키, 정상으로 칠 값 - health_rules 가 비었을 때만
        d('health_rules', 'ins=OK|DEGRADED;slam=OK')               # 키별 정상 값. ins DEGRADED 면 EKF-C 가 SLAM 위주로 감 → 주행 [10/6]
        d('health_grace_s', 2.0)                                  # 비정상이 이 시간 안이면 직전 경로 유지(격자는 안 쌓음), 넘으면 정지
        d('click_max_dz_m', 0.5)                                  # 클릭 점이 로버(base_link)보다 이만큼 넘게 높거나 낮으면 거부 (벽·배경 판)
        # 경기장 사전 정보 (경기장 좌표: 가운데 원점, +y = 페이즈2 쪽)
        d('field_size', [4.0, 5.0]); d('start_in_field', [-1.4, -2.07, 1.5707963])   # [10/8 케일럽] 좌측 하단 구석, +y 를 봄 [가정]
        d('lander_enable', False); d('lander_in_field', [0.0, 0.0]); d('edge_margin_m', 0.15); d('edge_unknown_m', 0.30)   # 랜더 위치 모름 [10/8]
        d('open_edges', '')
        d('auto_goal_in_field', [-1.5, 2.3])   # 페이즈2 틈: 위쪽 변 왼쪽 끝 (또는 [1.5, 2.3]) [가정, 10/8]
        # [auto-1014] 경기장 놓기·틈 지정 (운영자, 케일럽 10/14 '(나)'): 로버를 출발 자리에 두고 /ui/field {"cmd":"here", ...} 한 번
        #   → 그 순간 로버 자세 = 경기장 start 자리로 경기장을 다시 놓음 (SLAM 을 어디서 켰든·재추정했든 상관없음). 틈은 /ui/field gap 또는 /ui/gap_point 클릭
        d('field_topic', '/ui/field')          # std_msgs/String JSON: {"cmd":"here","size":[4,5],"start":[-1.4,-2.07,90]} (start = base_link 경기장 좌표 x, y [m], 방향 [deg])
                                               #   | {"cmd":"gap","xy":[-1.5,2.3]} (경기장 좌표) | {"cmd":"show"}
        d('gap_point_topic', '/ui/gap_point')  # geometry_msgs/PointStamped (odom_frame) — RViz 'Publish Point' 나 GUI 탑뷰 클릭으로 틈 지정
        d('field_require_set', False)          # true: /ui/field here 를 받기 전엔 AUTO 출발 안 함 (NO_FIELD). 실물 AUTO 설정에서 켬
        # [261015 케일럽 '(라)'] 경기장을 놓기 전엔 '연구실 범위', 놓은 뒤엔 경기장 — 설정 파일 하나로 평소 웨이포인트 주행과 자율을 같이
        #   true: 처음 = lab_size 사각형(SLAM 시작 자리 = odom 원점 가운데, odom 축), 경계 근처 미관측 금지 lab_edge_unknown_m, AUTO 거부(NO_FIELD)
        #         /ui/field here → field_size 경기장(경계 근처 미관측 금지 edge_unknown_m) / /ui/field clear → 다시 연구실 범위(지금 로버 자리 가운데)
        #   false(기본) = 예전 그대로 (처음부터 field_size 경기장, start_in_field 가정)
        d('lab_until_set', False); d('lab_size', [8.0, 8.0]); d('lab_edge_unknown_m', 0.0)
        # [261015 케일럽 '(가)'] 페이즈2 를 경기장 +y 변 위에 붙임 (놓은 경기장에만, 연구실 범위엔 없음). 사이 변은 벽, 입구 = 틈 x ± gap_w/2 만 열림
        #   웨이포인트는 페이즈1 ∪ 페이즈2 어디든, 자율 방문점은 페이즈1 만. 크기·위치·입구 폭 [가정] — 연습 주행 때 재서 설정 파일로 (경기장마다 한 번)
        d('p2_enable', False); d('p2_size', [3.0, 4.0]); d('gap_w', 1.2)
        d('p2_align', 'left')                  # 페이즈2 가 위쪽 변 어디에 붙나: left(왼쪽 끝 맞춤) | center(가운데) | right(오른쪽 끝 맞춤) | x(p2_x 값 그대로)
        d('p2_x', 0.0)                         #   p2_align = x 일 때만: 페이즈2 가운데 x (경기장 좌표, 페이즈1 가운데 = 0, +x = 오른쪽)
        # [261014 auto-1014] 완전 자율: 경기장에 고정한 방문점(열 x 행)을 상황에 맞춰 골라 지나간 뒤 틈으로 (auto_wp.py, 케일럽 10/14)
        #   다음 점 = 다익스트라 비용 + w_turn x 방향 바꿈 + w_order x ㄹ 순서 차이 (순서는 동점 깨기 정도), switch_gain 넘게 싸야 바꿈
        #   지나가는 점은 방향 없음(arrive_m 안 → 다음 점), 막힘·길 없음·진전 없음은 건너뜀, 움직이지 못한 채 두 점 연속 실패 = 갇힘(AUTO_BOXED 정지)
        d('auto_points', True)                 # false = 예전처럼 auto_goal_in_field 로 바로
        d('auto_grid', [3, 4])                 # 방문점 열(x, 4 m 변) x 행(y, 5 m 변)
        d('auto_margin_m', 0.8); d('auto_clear_m', 0.5); d('auto_arrive_m', 0.30)
        d('auto_w_turn', 0.55); d('auto_w_order', 0.3); d('auto_switch_gain_m', 0.5)
        d('auto_stall_s', 20.0); d('auto_fail_max', 3); d('auto_time_limit_s', 480.0)   # 시간 한도 지나면 남은 점 버리고 틈 [가정]
        d('auto_update_s', 0.5)                # 점 고르기 다시 계산 간격 (지나감 확인은 틱마다)
        d('auto_smooth_fallback', True)        # AUTO 에서만: 다듬기가 막히면 짧은 Hybrid A* 한 번 더 (시뮬 22판 틈 4 → 7/14)
        d('smooth_fallback', False); d('fallback_time_s', 0.5)   # 다른 모드 (기본 끔 = 예전 그대로)
        # 계획
        d('method', 'smooth'); d('time_limit_s', 1.0); d('hysteresis', 0.15); d('goal_tol_m', 0.10); d('goal_pass_m', 0.30)
        # 도착 방위각 [10/11 교수님 요청] — 방향 없는 목표는 위 그대로
        d('goal_pose_heading', False)        # true: /goal_pose orientation 을 도착 방향으로 씀 (false = 예전처럼 위치만)
        d('goal_th_tol_deg', 10.0)           # 도착 방향 허용 오차 θtol
        d('heading_mode', 'hybrid')          # entry(A 진입점) | hybrid(B 전 구간 Hybrid A*) | mixed(C: 멀리는 method, heading_switch_m 안은 B)
        d('heading_switch_m', 1.5)           # mixed: 목표까지 칸 경로 거리가 이 안이면 방향 맞춤 탐색
        d('heading_fail', 'position')        # 방향을 맞출 길이 없을 때 position(위치만 맞추고 도착, 이유에 방향 오차) | stop(NO_HEADING 정지)
        # 방위각 탐색 튜닝 [10/11 교수님: 튜닝값 공유] — 기본값 = 10/11 시뮬 값 (docs/DARON_261011_튜닝값.md)
        d('head_r_margin', 1.2)              # 방향 맞춤 경로 회전반경 = 최소 회전반경 x 이 값 (추종 지연·슬립 여유)
        d('head_lead_in_m', 0.15)            # 목표 바로 앞 직선 진입 길이
        d('head_tol_frac', 0.5)              # 탐색은 도착 허용 범위(위치·방향)의 이 비율 안까지 (추종 오차 여유)
        d('head_time_s', 0.5); d('head_expand', 30000)   # 탐색 한도: 시간 / 뻗는 횟수. 넘으면 '시간 안에 길 없음'
        d('head_weight', 1.5)                # 남은 길 예상 가중 (1 = 최적, 클수록 빠르지만 치우침)
        d('head_shot_every', 10)             # 곡선(Dubins) 그어 보기 간격 [뻗는 횟수]
        d('head_keep', True)                 # 직전 방향 경로 위(10 cm·15° 안)면 그대로 씀 (좌우 고리 바뀜 방지)
        # 백업 [10/11 교수님: 슬립·경사로 경로를 못 따라갈 때] — 0 이면 끔
        d('dev_replan_m', 0.0); d('dev_replan_deg', 0.0)     # 경로에서 이만큼 벗어나면 2 s 기다리지 않고 바로 다시 계획 (제안 0.15 m / 30°).
                                                             #   기본 끔: 시뮬 슬립 70 % 에서 반경을 안 키운 채(turn_adapt 끔) 켜면 오히려 나빠짐(방향까지 14 → 12/22).
                                                             #   turn_adapt 와 같이 켤 것. 이탈 값(backup.dev_m)은 꺼도 status 에 나옴
        d('head_retry_max', 2)               # 방향 목표: 마지막 접근에서 방향 못 맞추고 지나친 횟수 한도 → heading_fail 대로 (위치만 / 정지)
        d('head_retry_near_m', 8.0)          # 방향 목표: 목표 1.5 m 안에서 이만큼 넘게 돌면 같은 처리
        d('stuck_s', 4.0); d('stuck_move_m', 0.03)   # 헛바퀴: 지령(또는 경로 추종) 중 이 시간 동안 이만큼도 못 가면
        d('stuck_action', 'warn')            # warn(상태에 표시만) | stop(STUCK 정지)
        d('rpm_topic', '/rover/auto/drive_rpm'); d('steer_cmd_topic', '/rover/auto/steer_cmd')   # 추종기 출력 (없으면 대용)
        d('stuck_rpm_min', 1.0)              # |rpm| 이 이 값 넘으면 '지령 중'
        d('turn_warn_ratio', 0.8)            # 실제/지령 곡률 비율이 이 값 밑이면 상태에 '회전 부족' 표시
        d('turn_adapt_min_goal_m', 1.5)      # 목표가 이 거리 안이면 반경을 바꾸지 않음 (마지막 접근 중에 키우면 들어갈 자리를 잃음)
        d('no_heading_retry_s', 5.0)         # NO_HEADING(방향 공간 없음)으로 선 뒤 다시 찾는 간격 (서 있으면 지도가 거의 안 바뀜)
        d('turn_adapt', False)               # true: 조향 지령 기준 회전 비율로 계획 반경을 키움 (반경 = 최소 x head_r_margin / 비율, 상한 head_r_max_margin)
        d('head_r_max_margin', 2.0)
        d('wheelbase_m', 0.34); d('delta_max_deg', 43.003); d('rim_m', 0.15); d('judge_r_m', 1.25)
        d('path_in_judge_window', False)     # [10/7] true 면 /planner/path 를 판정 창(base_link 둘레 judge_r_m) 안으로 자름 (기본은 앞 2 m)
        d('grid_mode', 'accumulate')         # [261008] accumulate(옛: 받은 격자를 최근 값 우선으로 쌓음) / replace(그대로 씀 — 판정이 그리드 피쳐맵 전체를 보낼 때)
        # 갱신 규칙
        d('replan_period_s', 2.0); d('grid_timeout_s', 3.0); d('min_grid_known', 0.02)
        d('why_topic', '/traversability/why')
        d('ahead_check_m', 1.0); d('min_seen_ahead', 0.0); d('path_end_replan_m', 0.40); d('end_check_m', 0.03); d('hold_kept_path', False)
        d('hold_kept_max_dev_m', 0.15)       # [261016] hold_kept_path 여도 경로에서 이만큼 벗어났으면 새로 계획한 경로를 보냄 (추종기 e_abort 0.30 보다 작게:
                                             #   예전엔 '칸 경로 0.30 m 안 = 유지'와 추종기 '|e_y| > 0.30 = HOLD'가 맞물려 서로 기다리며 멈췄다 — 10/14 로그 10.9 s)
        d('hold_kept_max_s', 3.0)            # [261016] hold_kept_path 여도 마지막 송신 뒤 이만큼 지나면 새로 계획한 경로를 보냄
                                             #   (늦게 뜬·재시작한 추종기가 경로를 영영 못 받던 문제. /planner/path 는 VOLATILE)
        d('default_mode', 'MANUAL'); d('waypoint_queue', False); d('click_remove_m', 0.0)
        d('tick_hz', 5.0); d('path_republish_hz', 1.0); d('arena_publish_hz', 1.0)
        # 경로 높이 [10/7] 경로 탐색은 평면(x, y, θ) 그대로, 낸 경로 점마다 지면 높이·자세를 채움 (path_z.py)
        d('height_topic', '/traversability/cloud')   # 판정 노드의 지면 3D 점 (판정 격자 칸 중심). 비우면 z = 로버 높이로 평평
        d('path_z_offset_m', -1.0)           # 기준점(두 축 가운데)이 지면 위로 떠 있는 높이. 음수 = 주행 중 (TF z - 발밑 지면 z) 중앙값으로 자동
        d('path_z_fit_r_m', 0.15)            # 경로 점 둘레 이 반경 안 칸으로 지면 평면 맞춤 (바퀴 간격 절반 정도: 차체가 걸치는 범위)
        d('log_dir', '')                     # 비우면 기록 안 함. 주면 plans.jsonl(계획마다 한 줄) + arena_####.npz(10번마다)
        d('map_cloud_topic', '')             # [10/7] 예 /live_tsdf/map_cloud. 비우면 높이 표시 끔 (예전처럼 경로 z = 0, 웨이포인트 = 찍은 z)
        d('height_period_s', 1.0)            # 지도 점군으로 높이 격자를 다시 만드는 최소 간격
        d('clicked_point_topic', '')         # [10/7] 예 /clicked_point (RViz Publish Point). 비우면 끔
        g = lambda n: self.get_parameter(n).value

        self.odom, self.base = g('odom_frame'), g('base_frame')
        fw, fl = g('field_size')
        ap = A.ArenaParams(field_w=float(fw), field_l=float(fl), edge_margin_m=g('edge_margin_m'), edge_unknown_m=g('edge_unknown_m'),
                           open_edges=tuple(e for e in g('open_edges').split(',') if e),
                           lander_xy=tuple(g('lander_in_field')) if g('lander_enable') else None,
                           judge_r_m=g('judge_r_m'), rim_m=g('rim_m'),
                           merge='replace' if g('grid_mode') == 'replace' else 'latest',
                           p2_enable=bool(g('p2_enable')), p2_w=float(g('p2_size')[0]), p2_l=float(g('p2_size')[1]),
                           p2_x=self._p2_x(str(g('p2_align')), float(g('p2_x')), float(fw), float(g('p2_size')[0])),
                           gap_x=float(g('auto_goal_in_field')[0]), gap_w=float(g('gap_w')))
        self.T = A.from_start_pose(tuple(g('start_in_field')))
        self.field_wl = (float(fw), float(fl)); self.start_field = tuple(float(v) for v in g('start_in_field'))   # [auto-1014] RViz 경기장 테두리·출발 자리
        self.field_req = bool(g('field_require_set'))
        self.field_info = dict(how='param', size=list(self.field_wl), start=[self.start_field[0], self.start_field[1], round(math.degrees(self.start_field[2]), 1)],
                               gap=[float(v) for v in g('auto_goal_in_field')], t=None, carried=None,
                               grid=[int(v) for v in g('auto_grid')])
        self.edge_unknown_field = float(g('edge_unknown_m'))            # 경기장을 놓으면 쓰는 값 (연구실 범위는 lab_edge_unknown_m)
        self.lab_on = bool(g('lab_until_set'))
        self.lab_wl = tuple(float(v) for v in g('lab_size'))
        self.lab_edge_unknown = float(g('lab_edge_unknown_m'))
        self.p2_on = bool(g('p2_enable'))
        self.p2_align, self.p2_x_param = str(g('p2_align')).lower(), float(g('p2_x'))
        if self.lab_on:                                                # [261015] 놓기 전 = 연구실 범위 (예전 live_lab 8x8·출발 0,0,0·미관측 금지 끔과 같은 격자)
            ap = dc_replace(ap, field_w=self.lab_wl[0], field_l=self.lab_wl[1], edge_unknown_m=self.lab_edge_unknown, p2_enable=False)
            self.T = (0.0, 0.0, 0.0)
            self.field_info.update(how='lab', size=list(self.lab_wl), start=None)
        self.arena_wl = self.lab_wl if self.lab_on else self.field_wl   # 지금 경로계획 격자의 사각형 (연구실 범위 또는 경기장)
        self.amap = A.ArenaMap(ap, self.T)
        self.field_info['p2'] = self.p2_info()
        rp = P.RoverParams(wheelbase_m=g('wheelbase_m'), delta_max_deg=g('delta_max_deg'))
        pp = P.PlanParams(goal_tol_m=g('goal_tol_m'))
        app = A.ArenaPlanParams(method=g('method'), time_limit_s=g('time_limit_s'), hysteresis=g('hysteresis'), goal_pass_m=g('goal_pass_m'),
                                window_m=float(g('judge_r_m')) if g('path_in_judge_window') else 0.0,
                                heading_mode=g('heading_mode'), th_tol_deg=float(g('goal_th_tol_deg')), switch_m=float(g('heading_switch_m')),
                                heading_fail=g('heading_fail'), head_r_margin=float(g('head_r_margin')), lead_in_m=float(g('head_lead_in_m')),
                                head_tol_frac=float(g('head_tol_frac')), head_time_s=float(g('head_time_s')), head_expand=int(g('head_expand')),
                                head_weight=float(g('head_weight')), shot_every=int(g('head_shot_every')), head_keep=bool(g('head_keep')),
                                head_r_max_margin=float(g('head_r_max_margin')),
                                smooth_fallback=bool(g('smooth_fallback')), fallback_time_s=float(g('fallback_time_s')))
        self.planner = A.ArenaPlanner(app, rp, pp)
        self.rp = self.planner.rp
        self.p_goal_tol, self.p_goal_pass = g('goal_tol_m'), g('goal_pass_m')
        self.p_th_tol = math.radians(float(g('goal_th_tol_deg')))
        self.goal_pose_heading = bool(g('goal_pose_heading'))
        self.last_arrival = None            # 마지막 도착 기록 (위치·방향 오차) — /planner/status
        self.auto_goal = tuple(self.amap.field_to_odom(np.array(g('auto_goal_in_field'), float)))
        # 완전 자율 (auto_points)
        self.auto_points = bool(g('auto_points'))
        ac, ar = (int(v) for v in g('auto_grid'))
        self.auto_ap = AW.AutoParams(field_w=float(fw), field_l=float(fl), margin_m=float(g('auto_margin_m')), n_cols=ac, n_rows=ar,
                                     clear_m=float(g('auto_clear_m')), arrive_m=float(g('auto_arrive_m')), w_turn=float(g('auto_w_turn')),
                                     w_order=float(g('auto_w_order')), switch_gain_m=float(g('auto_switch_gain_m')),
                                     fail_max=int(g('auto_fail_max')), stall_s=float(g('auto_stall_s')),
                                     time_limit_s=float(g('auto_time_limit_s')), gap_xy=tuple(float(v) for v in g('auto_goal_in_field')))
        self.auto_update_s = float(g('auto_update_s'))
        self.fb_auto, self.fb_other = bool(g('auto_smooth_fallback')), bool(g('smooth_fallback'))
        self.sel = None                     # AW.AutoSelector — 처음 AUTO 틱에 로버 자리로 만듦 (AUTO_RESET 이면 다시)
        self.auto_cur = None                # 지금 겨누는 자리 odom (x, y)
        self.auto_tgt = None                # 점 번호(0부터) 또는 'gap'
        self.auto_boxed = None              # 갇힘 정지 이유 (모드를 바꾸거나 AUTO_RESET/AUTO_SKIP 까지 유지)
        self.auto_t_upd = -1e9
        self.auto_log_n = 0
        self.p = {n: g(n) for n in ('replan_period_s', 'grid_timeout_s', 'min_grid_known', 'ahead_check_m', 'min_seen_ahead',
                                     'path_end_replan_m', 'end_check_m', 'hold_kept_path', 'hold_kept_max_dev_m', 'hold_kept_max_s', 'require_health', 'health_timeout_s', 'depth_scale', 'waypoint_queue', 'click_remove_m',
                                     'health_keys', 'health_ok_values', 'click_max_dz_m', 'health_rules', 'health_grace_s')}
        self.click_size = g('click_image_size')
        # 백업 [10/11]
        self.dev_m, self.dev_th = float(g('dev_replan_m')), math.radians(float(g('dev_replan_deg')))
        self.retry = B.HeadingRetry(max_miss=int(g('head_retry_max')), max_near_m=float(g('head_retry_near_m')),
                                    zone_m=float(g('goal_pass_m')))
        self.stuck = B.StuckWatch(float(g('stuck_s')), float(g('stuck_move_m')))
        self.stuck_action, self.stuck_rpm_min = g('stuck_action'), float(g('stuck_rpm_min'))
        self.turn = B.TurnWatch(1.0 / self.planner.rp.r_min)
        self.turn_warn = float(g('turn_warn_ratio'))
        self.turn_adapt = bool(g('turn_adapt'))
        self.no_heading_retry_s = float(g('no_heading_retry_s'))
        self.turn_adapt_min_goal_m = float(g('turn_adapt_min_goal_m'))
        if self.turn_adapt:                                          # 반경마다 Dubins 표(약 1 s)를 틱 밖에서 미리 (리뷰 10/11: 틱 안에서 만들면 1 s 멈춤)
            import threading
            threading.Thread(target=self.planner.prebuild_tables, daemon=True).start()
        self.cmd_rpm, self.cmd_steer = None, None          # (값, 받은 시각)
        self.backup = dict(dev_m=None, dev_deg=None, dev_replans=0, retry=None, stuck_s=0.0, turn_ratio=None, turn_src=None)
        self.dev_flag = None                                # 이번 틱 이탈 이유 (다시 계획 이유에 붙임)
        self.prev_tick_mid = None
        self.hold = None                                    # (목표 키, 상태) 백업이 세운 정지 — 목표·모드가 바뀔 때까지 유지

        # 상태
        self.mode = g('default_mode') if g('default_mode') in MODES else 'MANUAL'
        self.apply_mode_opts()
        self.waypoints = []                 # odom (x, y, z, yaw) — yaw = 도착 방향 [rad] 또는 None (위치만) [10/11]
        self.path_msg = None                # 지금 내보낸 경로 (None = 정지)
        self.path_pub_t = -1e9              # 그 경로를 (새로) 보낸 시각 — hold_kept_max_s [261016]
        self.band_exit_ok = False           # [261016] 경계 금지 띠 안 출발을 운영자가 허용함 (/ui/field band_exit 또는 here+band_exit). 띠를 나오면 끝
        self.band = dict(in_band=False, depth_m=0.0)
        self.path_pts = []
        self.goal_key = None
        self.last_plan_t = -1e9
        self.new_info = False
        self.last_grid_t = None
        self.grid_known = 0.0
        self.health = None                  # (값, 받은 시각)
        self.depths = deque(maxlen=15)
        self.K = None
        self.state, self.reason, self.plan_ms, self.seen_ahead = 'INIT', '시작', 0.0, 1.0
        self.last_click = None
        self.stopped_published = False
        self.retry_after = 0.0              # 계획 실패 뒤 다시 시도할 시각 (새 정보가 오면 바로)
        self.why = None                     # 최신 판정 사유 격자 (P.Grid, cost 자리에 사유 코드)
        self.path_goal = None               # 지금 경로를 낸 계획의 목표 (odom xy, 옮긴 목표)
        self.health_bad_since = None        # health 가 비정상이 된 시각 (health_grace_s 동안은 직전 경로 유지)
        self.why_raw = None                 # (시각 키, 행 배열, res, origin_p, origin_q) - 같은 시각 비용 격자와 짝지어 '낙차 의심' 기억
        self.last_int = None                # 마지막으로 경기장 지도에 쌓은 비용 격자 (시각 키, 로버 위치)
        self.log_dir = g('log_dir')
        self.n_plan = 0
        if self.log_dir:
            os.makedirs(self.log_dir, exist_ok=True)
            fn = os.path.join(self.log_dir, 'plans.jsonl')
            if os.path.exists(fn):                                     # [261016] launch respawn 으로 다시 뜨면 번호를 이어서 — arena_####.npz 를 덮지 않게
                with open(fn) as fh:
                    self.n_plan = sum(1 for _ in fh)
                self.get_logger().warn('기록 폴더에 이전 실행 기록 %d 건 — 이어서 씀 (respawn?)' % self.n_plan)
            self.log_f = open(fn, 'a')
        self.res_warned = False
        self.cloud_msg, self.hg, self.hg_t, self.hg_dirty = None, None, -1e9, False   # 지도 점군 → 바닥 높이 격자
        self.height_on = bool(g('map_cloud_topic'))
        self.height_period = float(g('height_period_s'))
        self.hl = Z.HeightLayer(self.amap)
        self.z_fit_r = float(g('path_z_fit_r_m'))
        self.z_off_fixed = float(g('path_z_offset_m'))
        self.z_off_samples = deque(maxlen=100)    # 자동: (TF 기준점 z - 발밑 지면 z), 틱(5 Hz)마다
        self.rover_z = 0.0                         # 마지막 기준점 z (지면을 하나도 모를 때 경로 z)
        self.path_z_known = 0.0

        self.tf = Buffer(cache_time=Duration(seconds=10.0))
        self.tfl = TransformListener(self.tf, self, spin_thread=True)   # 콜백 안에서 기다려도 TF 가 계속 들어오게
        self.create_subscription(OccupancyGrid, g('grid_topic'), self.on_grid, qos(reliable=False, depth=2))
        self.create_subscription(String, g('mode_topic'), self.on_mode, qos(reliable=False))   # BEST_EFFORT·VOLATILE = 어떤 발행 QoS 와도 맞음 (화면이 1 Hz 재발행)
        self.create_subscription(PointStamped, g('click_topic'), self.on_click, qos(reliable=False, depth=5))
        self.create_subscription(String, g('waypoint_cmd_topic'), self.on_wp_cmd, qos(reliable=False, depth=5))
        self.create_subscription(PoseStamped, g('goal_pose_topic'), self.on_goal_pose, qos(reliable=False, depth=5))   # RViz 2D Goal Pose (시험용)
        if g('field_topic'):
            self.create_subscription(String, g('field_topic'), self.on_field, qos(reliable=False, depth=5))
        if g('gap_point_topic'):
            self.create_subscription(PointStamped, g('gap_point_topic'), self.on_gap_point, qos(reliable=False, depth=5))
        self.pub_field = self.create_publisher(String, '/planner/field', qos(latched=True))   # 지금 경기장 (GUI 그리기용, 바뀔 때만)
        self.create_subscription(Image, g('depth_topic'), self.on_depth, qos(reliable=False, depth=2))
        self.create_subscription(CameraInfo, g('camera_info_topic'), self.on_info, qos(reliable=False, depth=2))
        if g('health_topic'):
            self.create_subscription(String, g('health_topic'), self.on_health, qos(reliable=False, depth=5))
        if g('height_topic'):
            self.create_subscription(PointCloud2, g('height_topic'), self.on_height, qos(reliable=False, depth=2))
        if g('why_topic'):
            self.create_subscription(OccupancyGrid, g('why_topic'), self.on_why, qos(reliable=False, depth=2))   # 판정 v2.2 노드부터 (없으면 이유만 덜 자세함)
        if self.height_on:
            self.create_subscription(PointCloud2, g('map_cloud_topic'), self.on_cloud, qos(reliable=False, depth=1))
        if g('clicked_point_topic'):
            self.create_subscription(PointStamped, g('clicked_point_topic'), self.on_point3d, qos(reliable=False, depth=5))
        if g('rpm_topic'):
            self.create_subscription(Float64, g('rpm_topic'), lambda m: setattr(self, 'cmd_rpm', (float(m.data), self.tnow())), qos(reliable=False, depth=5))
        if g('steer_cmd_topic'):
            self.create_subscription(Float64, g('steer_cmd_topic'), lambda m: setattr(self, 'cmd_steer', (float(m.data), self.tnow())), qos(reliable=False, depth=5))
        self.pub_path = self.create_publisher(Path, '/planner/path', qos())
        self.pub_status = self.create_publisher(String, '/planner/status', qos(depth=5))
        self.pub_wp = self.create_publisher(PoseArray, '/planner/waypoints', qos(latched=True))
        self.pub_route = self.create_publisher(Path, '/planner/route', qos())
        self.pub_arena = self.create_publisher(OccupancyGrid, '/planner/arena_grid', qos(latched=True))
        self.create_timer(1.0 / g('tick_hz'), self.tick)
        if g('path_republish_hz') > 0:                                # 0 = 같은 경로를 다시 안 보냄 (10/7: 현성님 추종기는 경로를 받을 때마다 속도·조향을 처음부터 다시 올림)
            self.create_timer(1.0 / g('path_republish_hz'), self.republish)
        self.create_timer(1.0 / g('arena_publish_hz'), self.publish_arena)
        self.create_timer(0.5, self.publish_status)
        self.pub_mk = self.create_publisher(MarkerArray, '/planner/markers', qos(depth=2))
        self.create_timer(0.2, self.publish_markers)                  # RViz: 차체·기준점(두 축 가운데)·목표 도착 범위
        self.publish_field()
        self.get_logger().info('daron_planner 시작: method=%s, 모드=%s, 출발(경기장)=%s, T_odom_field=%s, 자동 목표 odom=%s' % (
            app.method, self.mode, g('start_in_field'), ['%.3f' % v for v in self.T], ['%.2f' % v for v in self.auto_goal]))

    # ------------------------------------------------------------ 입력
    def now(self):
        return self.get_clock().now()

    def tnow(self):
        """판단용 시각 [s] = 노드 시계 (녹화 재생에서 use_sim_time:=true 면 녹화 시각)."""
        return self.get_clock().now().nanoseconds * 1e-9

    def lookup(self, target, source, stamp=None, timeout=0.1):
        """target <- source 변환 (t (3,), q (4,), stamp). 실패하면 None."""
        try:
            tr = self.tf.lookup_transform(target, source, stamp or Time(), timeout=Duration(seconds=timeout))
        except Exception as e:                                       # noqa: BLE001
            if stamp is not None:
                return self.lookup(target, source, None, timeout)    # 그 시각이 없으면 최신으로
            self.get_logger().warn('TF %s<-%s 실패: %s' % (target, source, e), throttle_duration_sec=2.0)
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        T, Q = np.array([t.x, t.y, t.z]), np.array([q.x, q.y, q.z, q.w])
        if not (np.all(np.isfinite(T)) and np.all(np.isfinite(Q))):   # [261016] NaN TF → path_z.ground_at 의 int(round(nan)) 로 노드가 죽었다
            self.get_logger().warn('TF %s<-%s 가 유한값이 아님 (NaN/inf) — 실패로 처리' % (target, source), throttle_duration_sec=2.0)
            return None
        return T, Q, tr.header.stamp

    def on_grid(self, msg):
        info = msg.info
        cost = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        o, q = info.origin.position, info.origin.orientation
        if abs(info.resolution - self.amap.res) > 1e-6 and not self.res_warned:
            self.get_logger().warn('판정 격자 %.3f m 와 경기장 지도 %.3f m 칸 크기가 다름: 가까운 칸 값으로 옮겨 적음 (얇은 불가 띠를 놓칠 수 있음)'
                                   % (info.resolution, self.amap.res))
            self.res_warned = True
        known = float((cost >= 0).sum()) / max(1, int(math.pi * (self.amap.ap.judge_r_m / info.resolution) ** 2))
        self.grid_known = known
        self.last_grid_t = self.tnow()
        if not self.health_ok()[0]:
            return                                                   # SLAM 이 나쁠 때 격자는 쌓지 않음
        if known < self.p['min_grid_known']:
            self.get_logger().info('판정 격자 관측 %.0f%% < %.0f%%: 누적 안 함' % (100 * known, 100 * self.p['min_grid_known']),
                                   throttle_duration_sec=5.0)
            return
        tf = self.lookup(self.odom, self.base, msg.header.stamp)
        rover_xy = tuple(tf[0][:2]) if tf else None
        n = self.amap.integrate(cost, info.resolution, (o.x, o.y, o.z), (q.x, q.y, q.z, q.w), rover_xy, stamp_sec(msg.header.stamp))
        self.new_info = self.new_info or n > 0
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.last_int = (key, rover_xy)
        if self.why_raw is not None and self.why_raw[0] == key:      # 사유 격자가 먼저 온 경우
            self.amap.mark_why(*self.why_raw[1:], rover_xy)

    def on_height(self, msg):
        if not self.health_ok()[0]:
            return                                                   # 격자와 같은 규칙: SLAM 이 나쁠 때는 안 쌓음
        if msg.header.frame_id and msg.header.frame_id != self.odom:
            self.get_logger().warn('높이 점 frame %s != %s: 안 씀' % (msg.header.frame_id, self.odom), throttle_duration_sec=10.0)
            return
        p = W.cloud_xyz(msg.fields, msg.point_step, msg.data, msg.width, msg.height, bool(getattr(msg, 'is_bigendian', False)))   # [10/11 합침] sensor_msgs_py 대신 (같은 디코더, 가짜 ROS 시험 가능)
        self.hl.integrate(p, stamp_sec(msg.header.stamp))

    def z_offset(self):
        if self.z_off_fixed >= 0:
            return self.z_off_fixed, 'param'
        if len(self.z_off_samples) >= 5:
            return float(np.median(self.z_off_samples)), 'auto'
        return None, 'none'                                           # 모름: 경로 첫 점을 지금 로버 높이에 맞춤 (path_z.fill_path)

    def on_why(self, msg):
        info = msg.info
        w = np.asarray(msg.data, dtype=np.int16).reshape(info.height, info.width)
        o, q = info.origin.position, info.origin.orientation
        self.why = P.Grid(w, info.resolution, (o.x, o.y, o.z), (q.x, q.y, q.z, q.w))
        key = (msg.header.stamp.sec, msg.header.stamp.nanosec)
        self.why_raw = (key, w, info.resolution, (o.x, o.y, o.z), (q.x, q.y, q.z, q.w))
        if self.last_int is not None and self.last_int[0] == key:    # 같은 시각 비용 격자를 이미 쌓았으면 바로 (쌓지 않은 격자의 사유는 버림)
            if self.amap.mark_why(*self.why_raw[1:], self.last_int[1]):
                self.new_info = True

    WHY_KOR = {2: '경사', 3: '턱', 4: '경사·턱', 5: '낙차', 7: '벽', 8: '벽 옆', 9: '크레이터', 10: '낙차 의심'}

    def ahead_reason(self, t, yaw):
        """로버 앞 0.2~0.9 m, 좌우 0.2 m 안에서 막는 것: 경기장 끝·랜더(사전 금지) / 판정 사유. 없으면 ''."""
        pts = [(t[0] + f * math.cos(yaw) - l * math.sin(yaw), t[1] + f * math.sin(yaw) + l * math.cos(yaw))
               for f in np.arange(0.2, 0.95, 0.05) for l in (-0.2, -0.1, 0.0, 0.1, 0.2)]
        a = self.amap
        F = a.odom_to_field(np.array(pts))
        cnt = {}
        eb = a.edge_block()
        for fx, fy in F:
            c = int(math.floor((fx - a.o_field[0]) / a.res)); r = int(math.floor((fy - a.o_field[1]) / a.res))
            if 0 <= r < a.ny and 0 <= c < a.nx:
                if a.edge_band[r, c] or eb[r, c]:
                    cnt['경기장 끝'] = cnt.get('경기장 끝', 0) + 1
                elif a.prior[r, c]:
                    cnt['랜더'] = cnt.get('랜더', 0) + 1
        if self.why is not None:
            g = self.why
            for x, y in pts:
                rc = g.cell(g.odom_to_g(np.array([x, y, t[2]])))
                if g.inside(rc):
                    k = self.WHY_KOR.get(int(g.cost[rc]))
                    if k:
                        cnt[k] = cnt.get(k, 0) + 1
        if not cnt:
            return ''
        top = sorted(cnt.items(), key=lambda kv: -kv[1])[:2]
        return ' (앞: %s)' % ', '.join(k for k, _ in top)

    def on_mode(self, msg):
        m = msg.data.strip().upper()
        if m not in MODES:
            self.get_logger().warn('모르는 모드: %r' % msg.data)
            return
        if m != self.mode:
            self.get_logger().info('모드 %s -> %s' % (self.mode, m))
            self.mode = m
            self.apply_mode_opts()
            self.planner.reset()
            self.hold = None
            self.auto_boxed = None                                    # 갇힘 정지는 모드를 바꾸면 풂 (수동으로 빼고 AUTO 로 돌아오는 흐름)
            if m == 'AUTO' and self.sel is not None:
                self.sel.resume(self.tnow())                          # 지나간 점·시간 한도는 그대로, 진전 없음 시계만 새로
                self.auto_cur, self.auto_t_upd = None, -1e9
            self.stop('모드 바뀜', 'MODE_CHANGED')                    # 바뀌는 순간 일단 정지, 다음 틱에 새로 계획
            self.last_plan_t = -1e9

    # ------------------------------------------------------------ 경기장 놓기·틈 지정 [auto-1014]
    def publish_field(self):
        f = dict(self.field_info)
        f['T_odom_field'] = [round(float(v), 4) for v in self.T]
        f['corners_odom'] = [[round(float(a), 3), round(float(b), 3)] for a, b in self.amap.field_to_odom(np.array(self.amap.outline()[:-1]))]
        f['gap_odom'] = [round(float(v), 3) for v in self.auto_goal[:2]]
        self.pub_field.publish(String(data=json.dumps(f, ensure_ascii=False)))
        if self.log_dir:
            with open(os.path.join(self.log_dir, 'field.json'), 'w') as fh:
                json.dump(f, fh, ensure_ascii=False)

    @staticmethod
    def _p2_x(align, x_param, field_w, p2_w):
        """[261015] 페이즈2 가운데 x: left = 왼쪽 끝 맞춤, center = 0, right = 오른쪽 끝 맞춤, x = 설정값."""
        a = str(align).lower()
        if a == 'left':
            return -field_w / 2 + p2_w / 2
        if a == 'right':
            return field_w / 2 - p2_w / 2
        if a == 'center':
            return 0.0
        return float(x_param)

    def p2_info(self):
        """[261015] 붙인 페이즈2 (status·/planner/field 용). 연구실 범위·p2 꺼짐이면 None.
        pass_w = 입구에서 기준점이 지나갈 수 있는 폭 (입구 폭 - 2 x (반폭 + edge_margin)). 0.1 m 아래면 경고."""
        a = self.amap.ap
        if not a.p2_enable:
            return None
        op = self.amap.opening()
        pw = None if op is None else round((op[1] - op[0]) - 2 * (a.half_width_m + a.edge_margin_m), 3)
        return dict(size=[a.p2_w, a.p2_l], x=round(a.p2_x, 3), align=self.p2_align, gap_w=a.gap_w,
                    opening=None if op is None else [round(op[0], 3), round(op[1], 3)], pass_w=pw)

    def set_gap(self, gxy_field, src):
        """틈(페이즈2 입구) = AUTO 마지막 목표. 경기장 좌표. 방문 기록은 그대로."""
        g_ = (float(gxy_field[0]), float(gxy_field[1]))
        self.auto_ap = dc_replace(self.auto_ap, gap_xy=g_)
        self.auto_goal = tuple(self.amap.field_to_odom(np.array(g_)))
        if self.sel is not None:
            self.sel.gap = np.asarray(self.auto_goal, float)
            if self.auto_tgt == 'gap':
                self.auto_cur = (float(self.auto_goal[0]), float(self.auto_goal[1]))
        self.field_info['gap'] = [round(g_[0], 3), round(g_[1], 3)]
        self.amap.set_gap_x(g_[0])                                    # [261015] 페이즈2 를 붙였으면 입구도 틈 x 로
        self.field_info['p2'] = self.p2_info()
        if self.amap.ap.p2_enable and self.amap.opening() is None:
            self.get_logger().warn('틈 x %.2f 가 페이즈2 폭 밖 → 페이즈2 입구 없음 (사이 변 전부 벽)' % g_[0])
        elif self.amap.ap.p2_enable and self.field_info['p2']['pass_w'] < 0.1:
            self.get_logger().warn('페이즈2 입구 %s 가 좁음 (지나갈 폭 %.2f m) — 틈이 페이즈2 끝에 걸쳤는지, gap_w 확인' % (
                self.field_info['p2']['opening'], self.field_info['p2']['pass_w']))
        self.planner.reset(); self.last_plan_t = -1e9; self.goal_key = None
        if self.state == 'ARRIVED' and self.mode == 'AUTO':
            self.state = 'OK'                                         # 틈에 도착한 뒤 틈을 옮기면 다시 감
        self.get_logger().info('틈 지정 (%s): 경기장 (%.2f, %.2f) = odom (%.2f, %.2f)' % (src, g_[0], g_[1], self.auto_goal[0], self.auto_goal[1]))
        self.publish_field()

    def set_field_here(self, size=None, start=None):
        """지금 로버(base_link) 자세 = 경기장 start 자리로 경기장을 다시 놓음. AUTO 중에는 안 받음. 반환 (성공, 이유)."""
        if self.mode == 'AUTO':
            return False, 'AUTO 중에는 경기장을 다시 놓지 않음 (MANUAL/WAYPOINT 로 바꾼 뒤)'
        tf = self.lookup(self.odom, self.base)
        if tf is None:
            return False, '위치(TF) 없음'
        size = (float(size[0]), float(size[1])) if size else self.field_wl
        if start:
            start = (float(start[0]), float(start[1]), math.radians(float(start[2])))
        else:
            start = self.start_field
        hx, hy = size[0] / 2, size[1] / 2
        if not (-hx < start[0] < hx and -hy < start[1] < hy):
            return False, '출발 자리 (%.2f, %.2f) 가 경기장 %.1fx%.1f 밖' % (start[0], start[1], size[0], size[1])
        R = W.quat_to_R(tf[1])
        rp_ = (float(tf[0][0]), float(tf[0][1]), math.atan2(R[1, 0], R[0, 0]))
        T = A.field_from_rover_pose(rp_, start)
        new = A.ArenaMap(dc_replace(self.amap.ap, field_w=size[0], field_l=size[1], edge_unknown_m=self.edge_unknown_field,
                                    p2_enable=self.p2_on, gap_x=float(self.field_info['gap'][0]),
                                    p2_x=self._p2_x(self.p2_align, self.p2_x_param, size[0], self.amap.ap.p2_w)), T)   # 경기장 폭이 바뀌어도 맞춤 유지
        n = new.carry_from(self.amap)
        hl = Z.HeightLayer(new); hl.carry_from(self.hl)
        gap_f = self.field_info['gap']
        self.amap, self.hl, self.T = new, hl, T
        self.field_wl, self.start_field, self.arena_wl = size, start, size
        self.auto_ap = dc_replace(self.auto_ap, field_w=size[0], field_l=size[1])
        self.sel, self.auto_cur, self.auto_tgt, self.auto_boxed, self.auto_t_upd, self.auto_log_n = None, None, None, None, -1e9, 0
        self.field_info.update(how='here', size=[size[0], size[1]], start=[round(start[0], 3), round(start[1], 3), round(math.degrees(start[2]), 1)],
                               t=round(time.time(), 1), carried=n, rover_odom=[round(v, 3) for v in rp_[:2]] + [round(math.degrees(rp_[2]), 1)])
        self.set_gap(gap_f, '경기장 다시 놓음')                       # 틈은 경기장 좌표 그대로 → odom 다시 계산 (publish_field 포함)
        self.get_logger().info('경기장 놓음: %.1fx%.1f m, 로버(base_link) odom (%.2f, %.2f, %.0f°) = 경기장 (%.2f, %.2f, %.0f°), 칸 %d 옮김, 방문 기록 지움' % (
            size[0], size[1], rp_[0], rp_[1], math.degrees(rp_[2]), start[0], start[1], math.degrees(start[2]), n))
        return True, '경기장 놓음'

    def set_field_lab(self):
        """[261015] 경기장 풀기 → 연구실 범위 (lab_size, 지금 로버 자리 가운데, odom 축, 미관측 금지 lab_edge_unknown_m). AUTO 중 거부.
        lab_until_set 이 꺼져 있으면 거부 (예전 동작 그대로 두기)."""
        if not self.lab_on:
            return False, 'lab_until_set 꺼짐 — 연구실 범위로 못 돌아감 (설정 파일에서 켤 것)'
        if self.mode == 'AUTO':
            return False, 'AUTO 중에는 경기장을 풀지 않음 (MANUAL/WAYPOINT 로 바꾼 뒤)'
        tf = self.lookup(self.odom, self.base)
        cx, cy = (float(tf[0][0]), float(tf[0][1])) if tf is not None else (0.0, 0.0)
        T = (cx, cy, 0.0)
        new = A.ArenaMap(dc_replace(self.amap.ap, field_w=self.lab_wl[0], field_l=self.lab_wl[1], edge_unknown_m=self.lab_edge_unknown,
                                    p2_enable=False), T)
        n = new.carry_from(self.amap)
        hl = Z.HeightLayer(new); hl.carry_from(self.hl)
        gap_f = self.field_info['gap']
        self.amap, self.hl, self.T, self.arena_wl = new, hl, T, self.lab_wl
        self.sel, self.auto_cur, self.auto_tgt, self.auto_boxed, self.auto_t_upd, self.auto_log_n = None, None, None, None, -1e9, 0
        self.field_info.update(how='lab', size=list(self.lab_wl), start=None, t=round(time.time(), 1), carried=n, rover_odom=None)
        self.set_gap(gap_f, '경기장 풀기')                            # 틈 숫자(경기장 좌표)는 그대로 — 다시 놓으면 그 경기장 기준
        self.get_logger().info('경기장 풀기: 연구실 범위 %.1fx%.1f m, 가운데 = 로버 odom (%.2f, %.2f), 칸 %d 옮김' % (self.lab_wl[0], self.lab_wl[1], cx, cy, n))
        return True, '연구실 범위로'

    def set_grid(self, cols, rows):
        """[10/15] 방문점 격자 (열 = 4 m 변 x 방향, 행 = 5 m 변 y 방향) 바꾸기. AUTO 중 거부. 방문 기록 지움 (다음 AUTO 때 새로 만듦)."""
        if self.mode == 'AUTO':
            return False, 'AUTO 중에는 격자를 안 바꿈 (MANUAL/WAYPOINT 로 바꾼 뒤)'
        try:
            cols, rows = int(cols), int(rows)
        except (TypeError, ValueError):
            return False, '열·행 숫자 아님 (%r, %r)' % (cols, rows)
        if not (1 <= cols <= 8 and 1 <= rows <= 8):
            return False, '열·행은 1~8 (%d x %d)' % (cols, rows)
        self.auto_ap = dc_replace(self.auto_ap, n_cols=cols, n_rows=rows)
        self.sel, self.auto_cur, self.auto_tgt, self.auto_boxed, self.auto_t_upd, self.auto_log_n = None, None, None, None, -1e9, 0
        self.field_info['grid'] = [cols, rows]
        self.get_logger().info('방문점 격자 %d열 x %d행 (방문 기록 지움)' % (cols, rows))
        return True, '격자 %dx%d' % (cols, rows)

    def on_field(self, msg):
        try:
            c = json.loads(msg.data)
            cmd = str(c.get('cmd', '')).lower()
        except Exception as e:                                        # noqa: BLE001
            self.get_logger().warn('/ui/field JSON 못 읽음: %s (%r)' % (e, msg.data))
            return
        if cmd == 'here':
            ok, why = self.set_field_here(c.get('size'), c.get('start'))
            if ok:
                self.band_exit_ok = bool(c.get('band_exit'))           # [261016] 제어창이 '띠 안 출발' 경고에 '예'를 했을 때만 True
            if 'gap' in c and ok:
                self.set_gap(c['gap'], '/ui/field here')
            if not ok:
                self.get_logger().warn('경기장 놓기 거부: %s' % why)
            self.field_info['last_cmd'] = dict(cmd='here', ok=ok, why=why, t=round(time.time(), 1))
            self.publish_field()
        elif cmd == 'band_exit':                                      # [261016] 띠 안에서 막힘 → 운영자 확인 → 나가는 칸만 열기
            self.band_exit_ok = True
            self.get_logger().warn('경계 금지 띠 나가기 허용 (운영자 확인) — 띠를 나오면 자동으로 끝')
            self.last_plan_t, self.retry_after = -1e9, 0.0
            self.publish_status()
        elif cmd == 'clear':
            self.band_exit_ok = False
            ok, why = self.set_field_lab()
            if not ok:
                self.get_logger().warn('경기장 풀기 거부: %s' % why)
            self.field_info['last_cmd'] = dict(cmd='clear', ok=ok, why=why, t=round(time.time(), 1))
            self.publish_field()
        elif cmd == 'gap' and c.get('xy') is not None:
            self.set_gap(c['xy'], '/ui/field gap')
        elif cmd == 'grid':
            ok, why = self.set_grid(c.get('cols'), c.get('rows'))
            self.field_info['last_cmd'] = dict(cmd='grid', ok=ok, why=why, t=round(time.time(), 1))
            if not ok:
                self.get_logger().warn('방문점 격자 바꾸기 거부: %s' % why)
            self.publish_field()
        elif cmd == 'show':
            self.publish_field()
        else:
            self.get_logger().warn('/ui/field 모르는 명령: %r' % msg.data)

    def on_gap_point(self, msg):
        """틈 클릭 (RViz Publish Point / GUI 탑뷰). odom_frame 이 아니면 TF 로 옮김. 경기장 밖 0.5 m 넘으면 거부."""
        if self.field_info.get('how') == 'lab':                       # [261015] 연구실 범위엔 경기장 좌표가 없음 → 틈 클릭은 경기장을 놓은 뒤
            self.get_logger().warn('틈 클릭 거부: 경기장을 아직 안 놓음 (연구실 범위) — [여기가 출발점] 먼저')
            self.field_info['last_cmd'] = dict(cmd='gap_click', ok=False, why='경기장 안 놓음 — 먼저 [여기가 출발점]', t=round(time.time(), 1))
            self.publish_field()
            return
        p = np.array([msg.point.x, msg.point.y, msg.point.z])
        if msg.header.frame_id and msg.header.frame_id != self.odom:
            tf = self.lookup(self.odom, msg.header.frame_id)
            if tf is None:
                self.get_logger().warn('틈 클릭: %s -> %s TF 없음' % (msg.header.frame_id, self.odom))
                return
            p = W.quat_to_R(tf[1]) @ p + tf[0]
        f = self.amap.odom_to_field(p[:2])
        hx, hy = self.arena_wl[0] / 2 + 0.5, self.arena_wl[1] / 2 + 0.5
        if not (-hx <= f[0] <= hx and -hy <= f[1] <= hy):
            self.get_logger().warn('틈 클릭 거부: 경기장 좌표 (%.2f, %.2f) 가 경기장에서 0.5 m 넘게 밖' % (f[0], f[1]))
            return
        self.set_gap(f, '클릭')

    def apply_mode_opts(self):
        """모드마다 다른 계획 옵션: AUTO 에서만 smooth_fallback (auto_smooth_fallback). 다른 모드는 smooth_fallback(기본 끔) 그대로."""
        self.planner.app.smooth_fallback = self.fb_auto if self.mode == 'AUTO' else self.fb_other

    def on_health(self, msg):
        self.health = (msg.data, self.tnow())

    def health_ok(self):
        if self.health is None:
            return (not self.p['require_health']), '건강 상태 토픽 없음'
        v, t = self.health
        if self.tnow() - t > self.p['health_timeout_s']:
            return False, '건강 상태 끊김'
        s = v.strip()
        okv = {x.strip().upper() for x in str(self.p['health_ok_values']).split(',') if x.strip()}
        if s.startswith('{'):
            # 실제 health_monitor 는 JSON ({"ins": "OK", "slam": "OK", ...}) [사실, 10/6 우분투 녹화 재생]. 'OK' 로 시작하는지만 보면 늘 정지
            try:
                h = json.loads(s)
            except ValueError:
                return False, 'SLAM 상태 해석 실패'
            rules = {}
            for item in str(self.p['health_rules']).split(';'):          # 'ins=OK|DEGRADED;slam=OK'
                if '=' in item:
                    k, vs = item.split('=', 1)
                    rules[k.strip()] = {x.strip().upper() for x in vs.split('|') if x.strip()}
            if not rules:
                rules = {k.strip(): okv for k in str(self.p['health_keys']).split(',') if k.strip()}
            vals = {k: str(h.get(k, '?')).upper() for k in rules}
            return all(vals[k] in rules[k] for k in rules), 'SLAM 상태 ' + ' / '.join('%s %s' % kv for kv in vals.items())
        return s.upper().startswith('OK'), 'SLAM 상태 %s' % v

    def on_depth(self, msg):
        self.depths.append(msg)

    def on_info(self, msg):
        self.K = list(msg.k)

    # ------------------------------------------------------------ 바닥 높이 (RViz 3D 지도 위에 그리기, 10/7 교수님 요청)
    def on_cloud(self, msg):
        self.cloud_msg = msg
        self.hg_dirty = True

    def height_grid(self):
        """최신 지도 점군으로 만든 높이 격자 (height_period_s 마다 한 번만 다시 만듦). 없으면 None."""
        if not self.height_on:
            return None
        if self.hg_dirty and self.cloud_msg is not None and time.time() - self.hg_t >= self.height_period:
            m = self.cloud_msg
            self.hg_dirty, self.hg_t = False, time.time()
            try:
                P_ = W.cloud_xyz(m.fields, m.point_step, m.data, m.width, m.height, bool(getattr(m, 'is_bigendian', False)))
                fr = m.header.frame_id
                if fr and fr != self.odom and len(P_):
                    tf = self.lookup(self.odom, fr, m.header.stamp)
                    if tf is None:
                        return self.hg
                    P_ = P_ @ W.quat_to_R(tf[1]).T + tf[0]
                if len(P_):
                    self.hg = W.HeightGrid(P_)
            except Exception as e:                                   # noqa: BLE001
                self.get_logger().warn('지도 점군 높이 실패: %s' % e, throttle_duration_sec=5.0)
        return self.hg

    def ground_z(self, x, y, fallback):
        """(x, y) 바닥 높이 (지도 점군 칸 중앙값, 0.15 m 안 가장 가까운 칸). 모르면 fallback."""
        hg = self.height_grid()
        if hg is not None:
            z = hg.lookup(float(x), float(y))
            if math.isfinite(z):
                return z
        return float(fallback)

    def tf_rover_z(self):
        tf = self.lookup(self.odom, self.base)
        return float(tf[0][2]) if tf is not None else 0.0

    def on_point3d(self, msg):
        """RViz 'Publish Point' 로 3D 지도 위를 찍은 점 → 웨이포인트 (높이 포함)."""
        pt = np.array([msg.point.x, msg.point.y, msg.point.z])
        if msg.header.frame_id and msg.header.frame_id != self.odom:
            tf = self.lookup(self.odom, msg.header.frame_id)
            if tf is None:
                self.last_click = dict(ok=False, reason='RViz 점 TF 없음')
                return
            pt = W.transform_point(tf[0], tf[1], pt)
        tb = self.lookup(self.odom, self.base)
        dz = float(pt[2] - tb[0][2]) if tb is not None else 0.0
        if abs(dz) > self.p['click_max_dz_m']:
            res = dict(ok=False, reason='찍은 점이 로버보다 %+.2f m %s → 바닥이 아님' % (dz, '높음' if dz > 0 else '낮음'),
                       point=[float(v) for v in pt])
        else:
            ok, why = self.add_waypoint(pt, 'RViz 점')
            res = dict(ok=ok, reason=why, point=[float(v) for v in pt])
        self.last_click = res
        self.get_logger().info('RViz 점 %s' % json.dumps(res, ensure_ascii=False))
        self.publish_status()

    def on_click(self, msg):
        res = dict(ok=False, reason='', pixel=[msg.point.x, msg.point.y])
        if self.K is None or not self.depths:
            res['reason'] = 'depth 영상·camera_info 없음'
        else:
            ts = stamp_sec(msg.header.stamp)
            dm = min(self.depths, key=lambda m: abs(stamp_sec(m.header.stamp) - ts)) if ts > 0 else self.depths[-1]
            u, v = msg.point.x, msg.point.y
            if self.click_size[0] > 0:                               # 화면 영상 크기가 depth 와 다르면 비율로 맞춤
                u *= dm.width / float(self.click_size[0]); v *= dm.height / float(self.click_size[1])
            depth = W.decode_depth(dm.encoding, dm.height, dm.width, dm.step, dm.data, self.p['depth_scale'])
            tf = self.lookup(self.odom, dm.header.frame_id, dm.header.stamp)
            if tf is None:
                res['reason'] = 'TF %s<-%s 없음' % (self.odom, dm.header.frame_id)
            else:
                pt, why = W.click_to_odom(u, v, depth, self.K, tf[0], tf[1])
                tb = self.lookup(self.odom, self.base, dm.header.stamp) if pt is not None else None
                dz = float(pt[2] - tb[0][2]) if (pt is not None and tb is not None) else 0.0
                if pt is None:
                    res['reason'] = why
                elif abs(dz) > self.p['click_max_dz_m']:                  # 배경 판·벽을 찍은 경우 (10/6 우분투: 1 m 높이 점이 웨이포인트가 됨)
                    res.update(reason='클릭한 점이 로버보다 %+.2f m %s → 바닥이 아님' % (dz, '높음' if dz > 0 else '낮음'),
                               point=[float(v_) for v_ in pt])
                else:
                    ok, why2 = self.add_waypoint(pt, '클릭')
                    res.update(ok=ok, reason=why2, point=[float(v_) for v_ in pt])
        self.last_click = res
        self.get_logger().info('클릭 %s' % json.dumps(res, ensure_ascii=False))
        self.publish_status()

    def add_waypoint(self, pt, source, yaw=None):
        r = float(self.p['click_remove_m'])
        if r > 0 and self.waypoints:                                 # 이미 찍은 점 근처를 다시 찍으면 그 점을 지움 (RViz 운용)
            d = [math.hypot(w[0] - pt[0], w[1] - pt[1]) for w in self.waypoints]
            i = int(np.argmin(d))
            if d[i] <= r:
                self.waypoints.pop(i)
                self.planner.reset(); self.last_plan_t = -1e9
                self.hold = None; self.retry.reset(None)
                self.publish_waypoints()
                return True, '%s: %d번 웨이포인트 지움 (남은 %d개)' % (source, i + 1, len(self.waypoints))
        f = self.amap.odom_to_field(np.asarray(pt[:2], float))
        if not bool(self.amap.inside_field(f)):                       # [261015] 페이즈2 를 붙였으면 페이즈1 ∪ 페이즈2
            return False, '%s 위치가 경기장 밖' % source
        if self.height_on:                                           # RViz 2D Goal Pose 는 z = 0 평면 → 그 자리 바닥 높이로 (모르면 로버 높이)
            z0 = float(pt[2]) if source in ('클릭', 'RViz 점') else self.tf_rover_z()
            pt = (float(pt[0]), float(pt[1]), self.ground_z(pt[0], pt[1], z0))
        wp = (float(pt[0]), float(pt[1]), float(pt[2]), None if yaw is None else float(yaw))
        if self.p['waypoint_queue']:
            self.waypoints.append(wp)
        else:
            self.waypoints = [wp]
        self.planner.reset()
        self.hold = None; self.retry.reset(None)                     # 같은 목표를 다시 줘도 백업 정지가 풀리고 재시도 횟수는 처음부터
        self.last_plan_t = -1e9
        self.goal_key = None                                         # [261017] 마지막 점 도착(ARRIVED) 뒤 같은 좌표를 다시 넣어도 계획하게
        self.publish_waypoints()
        return True, '%s 웨이포인트 추가%s' % (source, '' if yaw is None else ' (방향 %.0f°)' % math.degrees(yaw))

    def on_goal_pose(self, msg):
        p = msg.pose.position
        pt = np.array([p.x, p.y, p.z])
        o = msg.pose.orientation
        yaw = None
        if self.goal_pose_heading and (o.x * o.x + o.y * o.y + o.z * o.z + o.w * o.w) > 0.25:   # (0,0,0,0) = 방향 없음
            yaw = math.atan2(2 * (o.w * o.z + o.x * o.y), 1 - 2 * (o.y * o.y + o.z * o.z))
        if msg.header.frame_id and msg.header.frame_id != self.odom:
            tf = self.lookup(self.odom, msg.header.frame_id)
            if tf is None:
                self.last_click = dict(ok=False, reason='RViz 목표 TF 없음')
                return
            pt = W.transform_point(tf[0], tf[1], pt)
            if yaw is not None:
                Rf = W.quat_to_R(tf[1])
                yaw = P._wrap(yaw + math.atan2(Rf[1, 0], Rf[0, 0]))
        ok, why = self.add_waypoint(pt, 'RViz', yaw)
        self.last_click = dict(ok=ok, reason=why, point=[float(v) for v in pt],
                               yaw_deg=None if yaw is None else round(math.degrees(yaw), 1))
        self.publish_status()

    def on_wp_cmd(self, msg):
        c = msg.data.strip().upper()
        if c == 'CLEAR':
            self.waypoints = []
        elif c == 'UNDO' and self.waypoints:
            self.waypoints.pop()
        elif c == 'NEXT' and self.waypoints:
            self.waypoints.pop(0)
        elif c == 'AUTO_RESET':                                       # [auto-1014] 방문 기록·시간 한도를 지우고 다음 AUTO 틱에 지금 자리에서 새로
            self.sel, self.auto_cur, self.auto_tgt, self.auto_boxed, self.auto_t_upd, self.auto_log_n = None, None, None, None, -1e9, 0
            self.get_logger().info('자율: 방문 기록 지움 (AUTO_RESET)')
        elif c == 'AUTO_SKIP' and self.sel is not None:               # [auto-1014] 지금 점 건너뛰기 (갇힘 정지도 풂)
            self.sel.skip_target(self.tnow())
            self.auto_cur, self.auto_boxed, self.auto_t_upd = None, None, -1e9
            self.sel.resume(self.tnow())
        else:
            return
        self.planner.reset(); self.last_plan_t = -1e9
        self.hold = None; self.retry.reset(None)
        self.publish_waypoints()

    # ------------------------------------------------------------ 출력
    def make_path(self, pts, stamp):
        m = Path()
        m.header.frame_id = self.odom
        m.header.stamp = stamp
        pts3, known = Z.fill_path(self.hl, pts, self.z_offset()[0], self.rover_z, self.z_fit_r, exclude=self.amap.cost >= 100)
        for x, y, z, yaw, _d, q in pts3:
            ps = PoseStamped()
            ps.header = m.header
            ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = x, y, z
            (ps.pose.orientation.x, ps.pose.orientation.y,
             ps.pose.orientation.z, ps.pose.orientation.w) = (float(v) for v in q)
            m.poses.append(ps)
        self._mk_z_known = known                                     # 지면을 찾은 점 비율 (ROS 메시지엔 속성을 못 붙임)
        return m

    def stop(self, reason, state):
        changed = state != self.state
        self.state, self.reason = state, reason
        if self.path_msg is not None or not self.stopped_published:
            m = Path(); m.header.frame_id = self.odom; m.header.stamp = self.now().to_msg()
            self.pub_path.publish(m)
            self.stopped_published = True
        self.path_msg, self.path_pts = None, []
        if changed:
            self.publish_status()                                     # 상태가 바뀌면 0.5 s 타이머를 기다리지 않고 바로 (10/7 우분투: 도착 표시가 최대 0.5 s 늦게 보임)

    def on_callback_error(self, exc):
        """[261016] 콜백 예외 — 예전엔 프로세스가 죽고(finally 가 빈 경로만 보냄) 제어창은 '노드 없음'·버튼 잠김.
        이제는 정지(빈 경로, 상태 ERROR)하고 계속 돈다. 원인이 남아 있으면 다음 틱도 같은 자리에서 서 있는다."""
        self.n_errors = getattr(self, 'n_errors', 0) + 1
        where = traceback.extract_tb(exc.__traceback__)[-1] if exc.__traceback__ else None
        key = (type(exc).__name__, where.filename if where else '', where.lineno if where else 0)
        seen = getattr(self, '_err_seen', {})
        self._err_seen = seen
        if key not in seen or time.monotonic() - seen[key] > 10.0:     # 같은 자리 예외는 10 s 에 한 번만 전체 traceback
            seen[key] = time.monotonic()
            self.get_logger().error('콜백 예외 #%d → 정지:\n%s' % (self.n_errors, ''.join(
                traceback.format_exception(type(exc), exc, exc.__traceback__))))
        try:
            self.stop('내부 오류 → 정지: %s: %s' % (type(exc).__name__, str(exc)[:120]), 'ERROR')
        except Exception as e2:                                         # noqa: BLE001 — 정지 자체가 실패해도 루프는 계속
            self.get_logger().error('정지 발행 실패: %r' % e2, throttle_duration_sec=5.0)

    def republish(self):
        if self.path_msg is not None:
            self.pub_path.publish(self.path_msg)                    # 같은 stamp 로 다시 (추종기 '경로 끊김' 감시용)

    def log_plan(self, t, yaw, goal, res, blocked):
        self.n_plan += 1
        if not self.log_dir:
            return
        P_ = res.get('path') or []
        rec = dict(t=round(self.tnow(), 3), wall=round(time.time(), 3), n=self.n_plan, mode=self.mode,
                   pose=[round(float(t[0]), 3), round(float(t[1]), 3), round(float(yaw), 3)],
                   goal=[round(float(v), 3) for v in goal[:2]], goal_yaw=None if goal_yaw(goal) is None else round(goal_yaw(goal), 3),
                   e_th_deg=None if res.get('e_th') is None else round(math.degrees(res['e_th']), 1),
                   local=res.get('local'), heading_dropped=bool(res.get('heading_dropped')), status=res['status'], reason=res['reason'],
                   ahead=self.ahead_reason(t, yaw), ms=round(self.plan_ms, 1), ms_parts={k: round(v, 1) for k, v in res.get('ms', {}).items()},
                   kept=bool(res.get('kept')), blocked=bool(blocked), method=self.planner.app.method,
                   route_cells=len(res.get('route') or []), kmax=round(float(res.get('kmax', 0.0)), 3),
                   path=[[round(p[0], 3), round(p[1], 3)] for p in P_[::4]] + ([[round(P_[-1][0], 3), round(P_[-1][1], 3)]] if P_ else []),
                   grid_known=round(self.grid_known, 3), map_known=round(self.amap.stats()['known'], 3), updates=self.amap.n_updates,
                   auto=None if self.mode != 'AUTO' or self.auto_tgt is None else ('gap' if self.auto_tgt == 'gap' else int(self.auto_tgt) + 1),
                   local_used=res.get('local'))
        self.log_f.write(json.dumps(rec, ensure_ascii=False) + '\n')
        self.log_f.flush()
        if self.n_plan % 10 == 1:
            a = self.amap
            np.savez_compressed(os.path.join(self.log_dir, 'arena_%04d.npz' % self.n_plan), cost=a.cost, prior=a.prior,
                                edge_zone=a.edge_zone, sus=a.sus, o_field=a.o_field, res=a.res, T=np.array(a.T), t=self.tnow())

    def publish_waypoints(self):
        m = PoseArray(); m.header.frame_id = self.odom; m.header.stamp = self.now().to_msg()
        for x, y, z, yaw in self.waypoints:
            p = Pose(); p.position.x, p.position.y, p.position.z = float(x), float(y), float(z)
            if yaw is None:
                p.orientation.w = 1.0                                 # 방향 없음 (방향 여부는 /planner/status waypoint_yaw_deg 의 null)
            else:
                p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.poses.append(p)
        self.pub_wp.publish(m)

    def publish_arena(self):
        a = self.amap
        m = OccupancyGrid()
        m.header.frame_id = self.odom; m.header.stamp = self.now().to_msg()
        m.info.resolution = a.res; m.info.width = a.nx; m.info.height = a.ny
        (px, py, pz), (qx, qy, qz, qw) = a.origin_odom()
        m.info.origin.position.x, m.info.origin.position.y = px, py
        m.info.origin.orientation.z, m.info.origin.orientation.w = qz, qw
        m.data = np.clip(a.merged_cost(), -1, 100).astype(np.int8).reshape(-1).tolist()
        self.pub_arena.publish(m)

    def field_pose(self):
        """[auto-1014] base_link 의 경기장 좌표 [x, y, yaw°] (출발 자리 start_in_field 와 같은 기준) — 로버를 테이프 구석에 두고 맞는지 보는 용."""
        tf = self.lookup(self.odom, self.base, timeout=0.0)
        if tf is None:
            return None
        R = W.quat_to_R(tf[1])
        f = self.amap.odom_to_field(np.array(tf[0][:2]))
        yaw = math.degrees(P._wrap(math.atan2(R[1, 0], R[0, 0]) - self.T[2]))
        return [round(float(f[0]), 3), round(float(f[1]), 3), round(yaw, 1)]

    def publish_status(self):
        goal = self.current_goal()
        gyw = goal_yaw(goal) if goal is not None else None
        s = dict(state=self.state, reason=self.reason, mode=self.mode,
                 goal=[round(float(v), 3) for v in goal[:2]] if goal is not None else None,
                 goal_yaw_deg=None if gyw is None else round(math.degrees(gyw), 1),
                 heading=None if gyw is None else ('위치만(방향 못 맞춤)' if not self.planner.heading_active(goal[:2], gyw) else self.planner.app.heading_mode),
                 goal_tol_m=self.p_goal_tol, goal_th_tol_deg=round(math.degrees(self.p_th_tol), 1),
                 waypoint_yaw_deg=[None if w[3] is None else round(math.degrees(w[3]), 1) for w in self.waypoints],
                 last_arrival=self.last_arrival,
                 waypoints=len(self.waypoints), seen_ahead=round(self.seen_ahead, 2), grid_known=round(self.grid_known, 2),
                 plan_ms=round(self.plan_ms, 1), method=self.planner.app.method, path_points=len(self.path_pts),
                 click=self.last_click, stamp=round(time.time(), 2),
                 z_known=round(self.path_z_known, 2), z_offset=None if self.z_offset()[0] is None else round(self.z_offset()[0], 3),
                 z_offset_src=self.z_offset()[1], backup=self.backup, auto=self.auto_info(), field_pose=self.field_pose(),
                 field=dict({k: self.field_info.get(k) for k in ('how', 'size', 'start', 'gap', 't', 'grid', 'p2')},
                            band_m=round(self.amap.ap.half_width_m + self.amap.ap.edge_margin_m, 3),   # [261016] 제어창 '띠 안 출발' 경고용
                            mid_from_base_m=round(self.rp.mid_from_base, 3)),
                 band=dict(self.band, exit_ok=self.band_exit_ok))
        self.pub_status.publish(String(data=json.dumps(s, ensure_ascii=False)))

    # ------------------------------------------------------------ RViz 표시 (10/6 실시간 시험: 로버의 어느 점이 목표에 닿아야 하는지 안 보임)
    def publish_markers(self):
        def mk(ns, i, typ, rgba, scale, xyz=(0.0, 0.0, 0.0), yaw=0.0, pts=None, text=''):
            m = Marker()
            m.header.frame_id = self.odom
            m.header.stamp = self.now().to_msg()
            m.ns, m.id, m.type, m.action = ns, i, typ, Marker.ADD
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in xyz)
            m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
            m.color.r, m.color.g, m.color.b, m.color.a = (float(v) for v in rgba)
            if pts is not None:
                m.points = [Point(x=float(a), y=float(b), z=float(c)) for a, b, c in pts]
            m.text = text
            return m

        def gone(ns, i):
            m = Marker()
            m.header.frame_id = self.odom
            m.ns, m.id, m.action = ns, i, Marker.DELETE
            return m

        out = []
        tf = self.lookup(self.odom, self.base)
        if tf is not None:
            t, q, _ = tf
            R = W.quat_to_R(q)
            yaw = math.atan2(R[1, 0], R[0, 0])
            c, s_ = math.cos(yaw), math.sin(yaw)
            mid = (t[0] + self.rp.mid_from_base * c, t[1] + self.rp.mid_from_base * s_, t[2])
            hl, hw = self.rp.length_m / 2, self.rp.half_width_m
            box = [(mid[0] + a * c - b * s_, mid[1] + a * s_ + b * c, mid[2] + 0.02)
                   for a, b in ((hl, hw), (hl, -hw), (-hl, -hw), (-hl, hw), (hl, hw))]
            out.append(mk('rover', 0, Marker.LINE_STRIP, (1, 1, 1, 0.9), (0.012, 0, 0), pts=box))          # 차체 윤곽 (계획기가 쓰는 크기)
            out.append(mk('rover', 1, Marker.SPHERE, (1, 0.85, 0, 1), (0.07, 0.07, 0.07), mid))             # 기준점 = 두 축 가운데 (이 점이 목표에 닿아야 도착)
            out.append(mk('rover', 2, Marker.SPHERE, (0.6, 0.6, 0.6, 0.9), (0.04, 0.04, 0.04), t))          # base_link = 뒷축 가운데
            out.append(mk('rover', 3, Marker.TEXT_VIEW_FACING, (1, 1, 1, 1), (0, 0, 0.07),
                          (mid[0], mid[1], mid[2] + 0.30), text='ref  %s' % self.state))
        wz = (tf[0][2] if tf is not None else 0.0) + 0.15
        # [auto-1014] 경기장 테두리(field_size, 노랑 선) + 출발 자리(start_in_field 의 base_link, 노랑 화살표) — 바닥 테이프와 맞는지 보는 용
        fw_, fl_ = self.arena_wl
        lab = self.field_info.get('how') == 'lab'                      # [261015] 연구실 범위: 회색 테두리, 출발 화살표·틈은 숨김(투명)
        ya = 0.0 if lab else 0.9
        cor = self.amap.field_to_odom(np.array(self.amap.outline()))   # [261015] 페이즈2 를 붙였으면 같이
        out.append(mk('field', 0, Marker.LINE_STRIP, (0.6, 0.6, 0.6, 0.6) if lab else (1, 0.9, 0.1, 0.9), (0.02 if lab else 0.025, 0, 0),
                      pts=[(c[0], c[1], wz - 0.14) for c in cor]))
        so = self.amap.field_to_odom(np.array(self.start_field[:2]))
        out.append(mk('field', 1, Marker.ARROW, (1, 0.9, 0.1, ya), (0.30, 0.04, 0.04), (so[0], so[1], wz - 0.13),
                      yaw=self.start_field[2] + self.T[2]))
        out.append(mk('field', 2, Marker.TEXT_VIEW_FACING, (0.7, 0.7, 0.7, 1) if lab else (1, 0.9, 0.1, 1), (0, 0, 0.10), (cor[0][0], cor[0][1], wz),
                      text=('lab area %.1fx%.1f m (field not set)' % (fw_, fl_)) if lab else
                      ('field %.1fx%.1f m%s' % (fw_, fl_, '' if self.field_info.get('how') == 'here' else ' (not set)'))))
        out.append(mk('field', 3, Marker.CUBE, (0.3, 0.6, 1.0, 0.0 if lab else 0.85), (0.2, 0.2, 0.03), (self.auto_goal[0], self.auto_goal[1], wz - 0.13)))   # 틈 (AUTO 마지막 목표)
        out.append(mk('field', 4, Marker.TEXT_VIEW_FACING, (0.3, 0.6, 1.0, 0.0 if lab else 1), (0, 0, 0.09), (self.auto_goal[0], self.auto_goal[1], wz + 0.02), text='gap'))
        op = self.amap.opening()                                      # [261015] 페이즈2 입구 (초록 선). 없으면 투명
        hy_ = self.amap.ap.field_l / 2
        oo = self.amap.field_to_odom(np.array([(op[0], hy_), (op[1], hy_)] if op else [(0.0, hy_), (0.01, hy_)]))
        out.append(mk('field', 5, Marker.LINE_STRIP, (0.2, 0.9, 0.3, 0.95 if op else 0.0), (0.05, 0, 0), pts=[(c[0], c[1], wz - 0.13) for c in oo]))
        if self.height_on:                                           # 지도가 자라면 웨이포인트 높이도 다시 (처음 찍을 때 못 본 곳)
            changed = False
            for i, w in enumerate(self.waypoints):
                z = self.ground_z(w[0], w[1], w[2])
                if abs(z - w[2]) > 0.01:
                    self.waypoints[i] = (w[0], w[1], z, w[3]); changed = True
            if changed:
                self.publish_waypoints()
        for i in range(20):                                          # 웨이포인트 순서 번호 (줄 세우기 때 어느 점이 먼저인지)
            if self.mode == 'WAYPOINT' and i < len(self.waypoints):
                w = self.waypoints[i]
                out.append(mk('wp', i, Marker.TEXT_VIEW_FACING, (1, 0.85, 0.2, 1), (0, 0, 0.10),
                              (w[0], w[1], (w[2] + 0.15) if self.height_on else wz), text=str(i + 1)))
                if w[3] is not None:                                 # [10/11] 웨이포인트 방향 화살표 (노랑)
                    out.append(mk('wp_dir', i, Marker.ARROW, (1, 0.85, 0.2, 0.9), (0.25, 0.03, 0.03),
                                  (w[0], w[1], (w[2] + 0.02) if self.height_on else wz - 0.13), yaw=w[3]))
                else:
                    out.append(gone('wp_dir', i))
            else:
                out += [gone('wp', i), gone('wp_dir', i)]
        # [auto-1014] 완전 자율 방문점: 흰 = 남음, 초록 = 지나감, 빨강 = 건너뜀, 노랑 테두리 = 지금 목표. 번호 = 점 번호(1부터)
        sel = self.sel
        for i in range(30):
            if sel is not None and i < len(sel.pts):
                p = sel.pts[i]
                s_ = sel.state[i]
                col = (0.2, 0.8, 0.3, 0.9) if s_ == sel.VISITED else (0.9, 0.25, 0.2, 0.9) if s_ == sel.SKIPPED else (1, 1, 1, 0.8)
                big = 0.16 if (self.auto_tgt == i and self.mode == 'AUTO') else 0.10
                out.append(mk('auto_pt', i, Marker.SPHERE, col, (big, big, 0.03), (p[0], p[1], wz - 0.13)))
                out.append(mk('auto_no', i, Marker.TEXT_VIEW_FACING, col[:3] + (1,), (0, 0, 0.09), (p[0], p[1], wz + 0.02), text=str(i + 1)))
            else:
                out += [gone('auto_pt', i), gone('auto_no', i)]
        goal = self.current_goal()
        if goal is not None:
            gz = (tf[0][2] if tf is not None else 0.0) + 0.01
            if self.height_on:
                gz = self.ground_z(goal[0], goal[1], gz - 0.01) + 0.01
            out.append(mk('goal', 10, Marker.CYLINDER, (0.1, 0.9, 0.3, 0.55),
                          (2 * self.p_goal_tol, 2 * self.p_goal_tol, 0.01), (goal[0], goal[1], gz)))          # 도착 범위 (기준점이 이 안)
            gyw = goal_yaw(goal) if self.planner.heading_active(goal[:2], goal_yaw(goal)) else None
            if gyw is None:
                ring = [(goal[0] + self.p_goal_pass * math.cos(a), goal[1] + self.p_goal_pass * math.sin(a), gz)
                        for a in np.linspace(0, 2 * math.pi, 37)]
                out.append(mk('goal', 11, Marker.LINE_STRIP, (0.1, 0.9, 0.3, 0.6), (0.008, 0, 0), pts=ring))    # 지나침 도착 범위 (뒤쪽만)
                out += [gone('goal', 14), gone('goal', 15)]
            else:                                                    # [10/11] 방향 목표: 지나침 = 목표 방향선을 따라 앞으로 pass_m, 옆 ±tol
                c_, s2 = math.cos(gyw), math.sin(gyw)
                L_, r_ = self.p_goal_pass, self.p_goal_tol
                box2 = [(goal[0] + a * c_ - b * s2, goal[1] + a * s2 + b * c_, gz) for a, b in ((0, r_), (L_, r_), (L_, -r_), (0, -r_))]
                out.append(mk('goal', 11, Marker.LINE_STRIP, (0.1, 0.9, 0.3, 0.6), (0.008, 0, 0), pts=box2))
                out.append(mk('goal', 14, Marker.ARROW, (0.1, 0.9, 0.3, 0.95), (0.30, 0.035, 0.035), (goal[0], goal[1], gz + 0.01), yaw=gyw))   # 목표 방향
                fan = [(goal[0] + 0.30 * math.cos(gyw + a), goal[1] + 0.30 * math.sin(gyw + a), gz + 0.01) for a in (-self.p_th_tol, self.p_th_tol)]
                out.append(mk('goal', 15, Marker.LINE_LIST, (0.1, 0.9, 0.3, 0.7), (0.008, 0, 0),
                              pts=[(goal[0], goal[1], gz + 0.01), fan[0], (goal[0], goal[1], gz + 0.01), fan[1]]))     # 허용 방향 ±θtol
            pg = self.path_goal
            if pg is not None and math.hypot(pg[0] - goal[0], pg[1] - goal[1]) > 0.02:
                out.append(mk('goal', 12, Marker.SPHERE, (1, 0.5, 0, 0.9), (0.06, 0.06, 0.06), (pg[0], pg[1], gz)))   # 옮긴 목표
                out.append(mk('goal', 13, Marker.LINE_LIST, (1, 0.5, 0, 0.8), (0.008, 0, 0),
                              pts=[(goal[0], goal[1], gz), (pg[0], pg[1], gz)]))
            else:
                out += [gone('goal', 12), gone('goal', 13)]
        else:
            out += [gone('goal', i) for i in (10, 11, 12, 13, 14, 15)]
        ma = MarkerArray()
        ma.markers = out
        self.pub_mk.publish(ma)

    # ------------------------------------------------------------ 주기 판단
    def current_goal(self):
        if self.mode == 'AUTO':
            return self.auto_cur if self.auto_points else self.auto_goal
        if self.mode == 'WAYPOINT' and self.waypoints:
            return self.waypoints[0]
        return None

    def arrived(self, passed=False, goal_dist=0.0, moved=0.0, near=False, e_th=None, dropped=False):
        self.last_arrival = dict(t=round(time.time(), 2), dist=round(float(goal_dist), 3),
                                 e_th_deg=None if e_th is None else round(math.degrees(e_th), 1),
                                 passed=bool(passed), heading_dropped=bool(dropped), moved=round(float(moved), 3))
        self.get_logger().info('도착 %s' % json.dumps(self.last_arrival, ensure_ascii=False))
        if self.mode == 'AUTO' and self.auto_points and self.sel is not None and self.auto_tgt not in (None, 'gap'):
            i = self.auto_tgt                                         # 방문점: 서지 않고 다음 점으로 (틈만 진짜 도착)
            self.sel.mark_visited(i, self.tnow(), '경로계획 도착')
            self.auto_log()
            self.auto_cur, self.auto_tgt, self.auto_t_upd = None, None, -1e9
            self.planner.reset()
            self.state, self.reason = 'OK', '자율: 점 %d 도착, 다음 점으로' % (i + 1)
            self.last_plan_t = -1e9
            self.goal_key = None
            return
        if self.mode == 'WAYPOINT' and self.waypoints:
            self.waypoints.pop(0)
            self.publish_waypoints()
            self.planner.reset()
            if self.waypoints:
                self.state, self.reason = 'OK', '웨이포인트 도착, 다음으로'
                self.last_plan_t = -1e9
                self.goal_key = None
                return
        why = '도착'
        if self.mode == 'AUTO' and self.auto_points and self.auto_tgt == 'gap':
            sm = self.sel.summary() if self.sel is not None else {}
            why = '자율 끝: 틈 도착 (점 %s/%s 지나감, %s 건너뜀)' % (sm.get('visited'), sm.get('n'), sm.get('skipped'))
            self.get_logger().info(why)                               # [10/15 우분투] 로그에도 (전엔 status 이유에만)
        if e_th is not None:
            why += ' (위치 %.2f m, 방향 %+.0f°)' % (goal_dist, math.degrees(e_th))
        if dropped:
            why += (' (방향 재시도 한도 — %s: 위치만 맞춤)' % ('놓침 %d번' % self.retry.miss if self.retry.miss >= self.retry.max_miss > 0
                                                        else '목표 근처 %.1f m 돎' % self.retry.near_dist)
                    if self.retry.dropped else ' (목표 방향으로 들어갈 공간 없음: 위치만 맞춤)')
        if passed:
            why += ' (목표를 %.2f m 지나침 — 전진만이라 되돌아가지 않음)' % goal_dist
        if moved:
            why += ' (목표가 불가·금지 칸이라 %.2f m 옮긴 곳 — 더 가까이는 못 감)' % moved
        if near:
            why += ' (옮긴 목표 %.2f m 앞에서 멈춤: 더 다가가면 부딪힘)' % goal_dist
        self.stop(why, 'ARRIVED')

    def sample_z_offset(self, t, R, mid):
        """기준점 높이(pose_relay 와 같은 점: base_link 에서 몸체 x 축으로 mid_from_base) - 발밑 지면 높이."""
        self.rover_z = float(t[2] + self.rp.mid_from_base * R[2, 0])
        gz = self.hl.ground_at(mid, self.z_fit_r)
        if gz is not None:
            self.z_off_samples.append(self.rover_z - gz[0])

    def remaining_path(self, mid_xy):
        if not self.path_pts:
            return 0.0, 0
        P_ = np.array([(p[0], p[1]) for p in self.path_pts])
        i = int(np.argmin(np.hypot(P_[:, 0] - mid_xy[0], P_[:, 1] - mid_xy[1])))
        seg = np.hypot(*np.diff(P_[i:], axis=0).T) if i < len(P_) - 1 else np.zeros(0)
        return float(seg.sum()), i

    def backup_watch(self, mid, yaw, goal, gyw, key, pg, rem, idx):
        """[10/11 교수님: 슬립·경사 백업] 틱마다. 반환 '이탈 이유'(바로 다시 계획) / '' / None(정지시킴 → tick 끝)."""
        tn = self.tnow()
        bk = self.backup
        warn = []
        moved = self.prev_tick_mid is not None and math.hypot(mid[0] - self.prev_tick_mid[0], mid[1] - self.prev_tick_mid[1]) > 1e-3
        self.prev_tick_mid = (float(mid[0]), float(mid[1]))
        # 1) 경로 이탈
        dev = ''
        if self.path_msg is not None and len(self.path_pts) >= 2:
            dd, dth, j = B.deviation(self.path_pts, mid, yaw, idx)
            bk['dev_m'], bk['dev_deg'] = round(dd, 3), round(math.degrees(dth), 1)
            if ((self.dev_m > 0 and dd > self.dev_m) or (self.dev_th > 0 and dth > self.dev_th)) and tn - self.last_plan_t >= 1.0:   # 새 경로마다 추종기가 속도를 0부터 올림(10/7) → 1 s 간격
                dev = '경로 이탈 %.0f cm·%.0f°' % (100 * dd, math.degrees(dth))
        else:
            bk['dev_m'] = bk['dev_deg'] = None
            j = 0
        # 2) 방향 재시도 한도 (한 번 포기한 목표는 목표가 바뀔 때까지 포기 유지 — SLAM 이상 등으로 planner.reset() 돼도)
        if (gyw is not None and self.retry.dropped and self.retry.key == key and self.planner.app.heading_fail != 'stop'
                and self.planner.heading_active(goal[:2], gyw)):
            self.planner.heading_dropped = self.planner.goal_key(goal[:2], gyw)
        if gyw is not None and self.planner.heading_active(goal[:2], gyw):
            sc = self.planner.r_margin_now / max(1.0, self.planner.app.head_r_margin)   # 반경을 키웠으면 거리 한도도 같은 비율로
            drop, why = self.retry.update(key, mid, pg, rem if self.path_msg is not None else 1e9, scale=sc)
            bk['retry'] = self.retry.info()
            if drop:
                if self.planner.app.heading_fail == 'stop':
                    self.stop('방향 재시도 한도: %s → 정지 (목표 방향 %.0f°, 목표를 다시 주면 재시도)' % (why, math.degrees(gyw)), 'NO_HEADING')
                    self.hold = (key, 'NO_HEADING', self.reason)
                    self.goal_key = key
                    return None
                self.planner.heading_dropped = self.planner.goal_key(goal[:2], gyw)
                self.planner.prev_head = None
                self.get_logger().warn('방향 재시도 한도: %s → 위치만 맞춤' % why)
                dev = dev or '방향 재시도 한도(%s) → 위치만' % why
        elif gyw is None:
            bk['retry'] = None
        # 3) 헛바퀴
        rpm_ok = self.cmd_rpm is not None and tn - self.cmd_rpm[1] <= 0.5
        active = (abs(self.cmd_rpm[0]) > self.stuck_rpm_min) if rpm_ok else (self.path_msg is not None and rem > 0.10)
        stuck, dur = self.stuck.update(tn, mid, active and self.state not in ('ARRIVED',))
        bk['stuck_s'] = round(dur, 1)
        if stuck:
            warn.append('헛바퀴 %.0f s (%s)' % (dur, 'rpm 지령' if rpm_ok else '경로 추종 중'))
            if self.stuck_action == 'stop':
                self.stop('헛바퀴: %s 중 %.0f s 동안 %.0f cm 미만 이동 → 정지' % ('rpm 지령' if rpm_ok else '경로 추종',
                                                                              self.stuck.window_s, 100 * self.stuck.min_move_m), 'STUCK')
                self.hold = (key, 'STUCK', self.reason)
                self.goal_key = key
                self.stuck.reset()
                return None
        # 4) 회전 부족 (기록만)
        steer_ok = self.cmd_steer is not None and tn - self.cmd_steer[1] <= 0.5
        if steer_ok:
            kc, bk['turn_src'] = math.tan(math.radians(self.cmd_steer[0])) / self.rp.wheelbase_m, 'steer_cmd'
        elif self.path_msg is not None and len(self.path_pts) >= 3:
            kc, bk['turn_src'] = B.path_curvature_at(self.path_pts, j), 'path'
        else:
            kc = None
        if moved:
            r = self.turn.update(mid, yaw, kc)
            bk['turn_ratio'] = None if r is None else round(r, 2)
            bk['turn_n'] = self.turn.n
        if bk.get('turn_ratio') is not None and bk.get('turn_n', 0) >= 3 and bk['turn_ratio'] < self.turn_warn:
            warn.append('회전 부족 (실제/지령 %.2f)' % bk['turn_ratio'])
        near_goal = math.hypot(mid[0] - pg[0], mid[1] - pg[1]) <= self.turn_adapt_min_goal_m
        if (self.turn_adapt and not near_goal and bk.get('turn_src') == 'steer_cmd' and bk.get('turn_ratio') is not None
                and bk.get('turn_n', 0) >= 3):                      # 목표 가까이서는 반경을 안 바꿈 (우분투 2-1: 0.85 m 앞에서 키우자 들어갈 자리가 없어 INFEASIBLE)
            m0 = self.planner.r_margin_now
            m1 = self.planner.set_turn_ratio(bk['turn_ratio'])          # 조향 지령 기준일 때만 (경로 곡률 대용은 부족을 못 봄)
            if abs(m1 - m0) > 1e-9:
                self.get_logger().info('회전 비율 %.2f → 계획 반경 배수 %.1f → %.1f' % (bk['turn_ratio'], m0, m1))
                dev = dev or '회전 부족 → 반경 x%.1f' % m1
        bk['r_margin'] = round(self.planner.r_margin_now, 2)
        bk['warn'] = warn
        return dev

    # ------------------------------------------------------------ 완전 자율 [auto-1014]
    def auto_log(self):
        """선택기 기록(지나감·건너뜀·목표 바꿈·틈)을 노드 로그로."""
        if self.sel is None:
            return
        for t_, kind, i, why in self.sel.log[self.auto_log_n:]:
            name = '' if kind == 'gap' else ('점 %d ' % (i + 1))
            txt = {'visit': '지나감', 'skip': '건너뜀', 'target': '다음 목표', 'gap': '틈으로'}.get(kind, kind)
            self.get_logger().info('자율: %s%s %s' % (name, txt, ('(%s)' % why) if why else ''))
        self.auto_log_n = len(self.sel.log)

    def auto_step(self, mid, yaw):
        """AUTO 다음 목표 (odom x, y). 선택기는 auto_update_s 마다 (점 근처를 지나는 중이면 바로) 다시 계산."""
        tn = self.tnow()
        if self.sel is None:
            f = self.amap.odom_to_field(np.array(mid[:2], float))
            fyaw = float(P._wrap(yaw - self.T[2]))                    # odom 방향 = 경기장 방향 + T 회전
            self.sel = AW.AutoSelector(self.auto_ap, start=(float(f[0]), float(f[1]), fyaw), rp=self.rp, pp=self.planner.pp,
                                       to_odom=lambda p: self.amap.field_to_odom(np.asarray(p, float)))
            self.auto_log_n = 0
            self.get_logger().info('자율 시작: 방문점 %d개 (%dx%d, 경계 %.1f m 안쪽), 출발(경기장) (%.2f, %.2f, %.0f°), 틈 %s' % (
                len(self.sel.pts), self.auto_ap.n_cols, self.auto_ap.n_rows, self.auto_ap.margin_m, f[0], f[1], math.degrees(fyaw),
                list(self.auto_ap.gap_xy)))
        sel, ar = self.sel, self.auto_ap.arrive_m
        near = any(sel.state[i] == sel.LEFT and math.hypot(p[0] - mid[0], p[1] - mid[1]) <= ar for i, p in enumerate(sel.pts))
        near = near or (sel.tgt_xy is not None and sel.target != 'gap' and math.hypot(sel.tgt_xy[0] - mid[0], sel.tgt_xy[1] - mid[1]) <= ar)
        if self.auto_cur is None or near or tn - self.auto_t_upd >= self.auto_update_s:
            tgt, xy = sel.update(self.amap, np.array(mid[:2], float), yaw, tn)
            self.auto_t_upd = tn
            self.auto_log()
            if xy is None:                                            # [261016] 로버 자리가 막힘 — 점은 그대로 두고 이번엔 목표 없음
                self.auto_cur = None
                return None
            if tgt != self.auto_tgt:
                self.planner.reset()                                  # 목표가 바뀜 = 직전 길에 매이지 않음 (웨이포인트 도착 때와 같음)
                self.auto_cur = (float(xy[0]), float(xy[1]))
            elif self.auto_cur is None or math.hypot(xy[0] - self.auto_cur[0], xy[1] - self.auto_cur[1]) > 0.05:
                self.auto_cur = (float(xy[0]), float(xy[1]))         # 설 자리가 5 cm 넘게 바뀔 때만 (지도 갱신마다 목표가 흔들려 재계획하지 않게)
            self.auto_tgt = tgt
        return self.auto_cur

    def auto_info(self):
        if self.sel is None:
            return None
        sm = self.sel.summary()
        el = None if self.sel.t0 is None else round(self.tnow() - self.sel.t0, 1)
        return dict(target=None if self.auto_tgt is None else ('gap' if self.auto_tgt == 'gap' else int(self.auto_tgt) + 1),
                    n=sm['n'], visited=sm['visited'], skipped=sm['skipped'], left=sm['left'],
                    order=[i + 1 for i in sm['order']], elapsed_s=el, time_limit_s=self.auto_ap.time_limit_s,
                    boxed=self.auto_boxed is not None,
                    skipped_why={str(i + 1): w for i, w in enumerate(sm['why']) if self.sel.state[i] == self.sel.SKIPPED})

    def tick(self):
        if self.mode == 'MANUAL':
            self.stop('키보드 모드: 경로 정지', 'MANUAL')
            return
        goal = self.current_goal()
        auto_sel = self.mode == 'AUTO' and self.auto_points            # 목표는 위치를 안 뒤에 고름 (아래)
        if goal is None and self.state == 'ARRIVED' and self.mode == 'WAYPOINT':
            if '다음 웨이포인트' not in self.reason:                # 도착 이유(위치·방향 오차, 위치만 맞춤 등)는 남김 [10/11]
                self.reason = self.reason + ' — 다음 웨이포인트를 찍어 주세요'
            return
        if goal is None and not auto_sel:
            self.stop('목표 없음 (웨이포인트를 찍어 주세요)' if self.mode == 'WAYPOINT' else '목표 없음', 'NO_GOAL')
            return
        ok, why = self.health_ok()
        if not ok:
            if self.health_bad_since is None:
                self.health_bad_since = self.tnow()
            bad_for = self.tnow() - self.health_bad_since
            if (self.path_msg is not None and bad_for < self.p['health_grace_s']
                    and self.health is not None and self.tnow() - self.health[1] <= self.p['health_timeout_s']):
                # 짧은 이상은 넘김: 직전 경로 유지, 새 계획·격자 누적은 안 함 (석윤님: 2 s 정상이면 OK 로 돌아옴)
                self.state, self.reason = 'OK', '%s → 직전 경로 유지 (%.1f/%.1f s)' % (why, bad_for, self.p['health_grace_s'])
                return
            if self.state != 'SLAM_BAD':
                self.planner.reset()                                  # 회복 뒤엔 직전 길에 매이지 말고 새로 (흔들려 자리가 바뀌었을 수 있음)
            self.stop(why + ' → 정지', 'SLAM_BAD')
            return
        self.health_bad_since = None
        if self.last_grid_t is None or self.tnow() - self.last_grid_t > self.p['grid_timeout_s']:
            self.stop('판정 격자 끊김 → 정지', 'JUDGE_LOST')
            return
        tf = self.lookup(self.odom, self.base)
        if tf is None:
            self.stop('위치(TF) 없음 → 정지', 'TF_LOST')
            return
        t, q, st = tf
        R = W.quat_to_R(q)
        yaw = math.atan2(R[1, 0], R[0, 0])
        mid = t[:2] + self.rp.mid_from_base * np.array([math.cos(yaw), math.sin(yaw)])
        self.sample_z_offset(t, R, mid)
        # [261016] 경계 금지 띠: 안이면 허용(운영자 확인) 전엔 서 있고, 띠를 나오면 허용을 끝낸다
        depth = A.band_depth(self.amap.grid(), mid)
        self.band = dict(in_band=depth > 0.0, depth_m=round(depth, 3))
        if depth <= 0.0 and self.band_exit_ok:
            self.band_exit_ok = False
            self.get_logger().info('경계 금지 띠 밖으로 나옴 → 띠 나가기 허용 끝')
        self.planner.band_exit = self.band_exit_ok
        if self.sel is not None:
            self.sel.band_exit = self.band_exit_ok
        if depth > 0.0 and not self.band_exit_ok and (auto_sel or goal is not None):
            self.stop('로버(차체 가운데)가 경계 금지 띠 안 (안쪽 끝까지 %.2f m) → 제어창에서 [띠 나가기] 확인, 또는 수동으로 빼기' % depth,
                      'START_BLOCKED')
            return
        if auto_sel and (self.field_req or self.lab_on) and self.field_info.get('how') != 'here':
            self.stop('경기장 위치 미지정%s → AUTO 안 함 (로버를 출발 자리에 두고 /ui/field here)' % (' (연구실 범위)' if self.field_info.get('how') == 'lab' else ''), 'NO_FIELD')
            return
        if auto_sel:
            if self.auto_boxed is not None:                           # 갇힘: 모드 바꿈·AUTO_SKIP·AUTO_RESET 까지 정지 유지
                if self.state != 'AUTO_BOXED':
                    self.stop(self.auto_boxed, 'AUTO_BOXED')
                return
            goal = self.auto_step(mid, yaw)
            if goal is None:
                self.stop('자율: 목표 없음', 'NO_GOAL')
                return
        gyw = goal_yaw(goal)
        key = (self.mode, tuple(np.round(goal[:2], 3)), None if gyw is None else round(gyw, 3))
        if self.state == 'ARRIVED' and key == self.goal_key:
            return                                                   # 도착한 목표 그대로면 대기
        if self.hold is not None:
            if self.hold[0] == key:                                  # 백업 정지(방향 재시도 한도·헛바퀴): 목표·모드를 바꿀 때까지 유지
                if self.state != self.hold[1]:                       #   (사이에 판정 끊김·SLAM 이상 등으로 상태가 바뀌었어도 다시 그 정지로 — 리뷰 10/11)
                    self.stop(self.hold[2], self.hold[1])
                return
            self.hold = None
        if self.path_msg is None and key == self.goal_key and not self.new_info and self.tnow() < self.retry_after:
            return                                                   # 방금 실패했고 바뀐 게 없음
        if self.state == 'NO_HEADING' and key == self.goal_key and self.tnow() < self.retry_after:
            return                                                   # [10/11 우분투] 방향 공간 없음 정지: 새 격자가 와도 no_heading_retry_s 마다만 다시 찾음 (매 1 s 0.5 s 탐색 낭비)
        rem, idx = self.remaining_path(mid)
        pg = self.path_goal if self.path_goal is not None else goal[:2]  # 계획기가 실제로 쓴 목표 (금지 띠 안 목표는 안쪽으로 옮겨짐)
        end_far = bool(self.path_pts) and math.hypot(self.path_pts[-1][0] - pg[0], self.path_pts[-1][1] - pg[1]) > 0.25
        blocked = self.path_msg is not None and A.path_blocked(self.amap, self.path_pts, self.rp, self.p['ahead_check_m'], idx,
                                                               rover_mid_odom=mid, rover_yaw=yaw, band_exit=self.band_exit_ok)
        self.seen_ahead = A.seen_fraction_ahead(self.amap, self.path_pts, self.p['ahead_check_m'], idx) if self.path_pts else 0.0
        # 도착은 틱(5 Hz)마다 확인: 전엔 계획할 때만 봐서 추종기가 경로 끝에서 선 뒤 다음 격자까지 ARRIVED 가 늦었음 (10/7 우분투 0.35~1.75 s)
        if self.path_msg is not None and key == self.goal_key:
            ha = self.planner.heading_active(goal[:2], gyw)
            ok_a, ai = A.arrival(mid, yaw, pg, gyw if ha else None, self.p_goal_tol, self.p_goal_pass, self.p_th_tol)   # 계획기와 같은 기준 [10/11]
            if ok_a:
                moved = math.hypot(pg[0] - goal[0], pg[1] - goal[1])
                ex = P._wrap(yaw - gyw) if gyw is not None else None
                self.arrived(passed=ai['passed'], goal_dist=ai['dist'], moved=moved if moved > 0.02 else 0.0,
                             e_th=ex, dropped=gyw is not None and not ha)
                return
        dev = self.backup_watch(mid, yaw, goal, gyw, key, pg, rem, idx)   # 도착 확인 뒤에 (도착하는 틱에 한도가 걸려 NO_HEADING 이 되지 않게 — 리뷰 10/11)
        if dev is None:
            return
        # 경로 끝에 섰는데(남은 길 end_check_m 미만) 도착 범위 밖이면 다음 격자를 기다리지 말고 다시 계획
        at_end = self.path_msg is not None and rem < self.p['end_check_m'] and self.tnow() - self.last_plan_t >= 0.5
        urgent = (self.path_msg is None or blocked or key != self.goal_key or (end_far and rem < self.p['path_end_replan_m']) or at_end
                  or bool(dev))
        periodic = self.new_info and self.tnow() - self.last_plan_t >= self.p['replan_period_s']
        if not (urgent or periodic):
            return
        t0 = time.time()
        res = self.planner.plan(self.amap, t, R, goal, gyw)
        self.plan_ms = (time.time() - t0) * 1000
        self.log_plan(t, yaw, goal, res, blocked)
        self.last_plan_t = self.tnow()
        self.new_info = False
        self.goal_key = key
        if res['status'] == A.Status.ARRIVED:
            ex = None if gyw is None else P._wrap(yaw - gyw)
            self.arrived(passed=bool(res.get('passed')), goal_dist=res.get('goal_dist_m', 0.0), moved=res.get('goal_moved_m') or 0.0,
                         near=bool(res.get('near_moved')), e_th=ex, dropped=gyw is not None and bool(res.get('heading_dropped')))
            return
        if res['status'] != A.Status.OK:
            if res['status'] == A.Status.TIMEOUT and self.path_msg is not None and not blocked:
                self.state, self.reason = 'OK', '계산 시간 초과 → 직전 경로 유지'
                return
            if auto_sel and self.auto_tgt not in (None, 'gap'):
                i = self.auto_tgt
                act = self.sel.plan_failed(res['status'], self.tnow())
                self.auto_log()
                if act == 'boxed':                                    # 점 탓이 아니라 로버가 갇힘 (전진만으로 못 빠져나옴) → 운영자
                    self.auto_boxed = ('자율: 갇힘 — 움직이지 못한 채 점 두 곳 연속 %s%s → 정지 (수동으로 빼고 AUTO 다시, 또는 AUTO_SKIP)'
                                       % (res['reason'], self.ahead_reason(t, yaw)))
                    self.get_logger().warn(self.auto_boxed)
                    self.stop(self.auto_boxed, 'AUTO_BOXED')
                    return
                if act == 'skip':                                     # 그 점을 건너뜀 → 다음 틱에 다른 점으로 바로
                    self.stop('자율: 점 %d %s → 건너뜀, 다른 점으로' % (i + 1, res['reason']), res['status'])
                    self.auto_cur, self.auto_t_upd = None, -1e9
                    self.retry_after = self.tnow()
                    return
            self.stop('%s%s → 정지' % (res['reason'], self.ahead_reason(t, yaw)), res['status'])
            self.retry_after = self.tnow() + (self.no_heading_retry_s if res['status'] == A.Status.NO_HEADING else 1.0)
            return
        if auto_sel:
            self.sel.plan_ok()
        if dev:
            self.backup['dev_replans'] += 1
        off_path = (self.backup['dev_m'] or 0.0) > float(self.p['hold_kept_max_dev_m'])     # [261016] 상호 대기 방지
        held_long = self.tnow() - self.path_pub_t > float(self.p['hold_kept_max_s'])           # [261016] 늦게 뜬 추종기
        if (self.p['hold_kept_path'] and res.get('kept') and not blocked and not dev and self.path_msg is not None
                and not (end_far and rem < self.p['path_end_replan_m']) and rem > 0.3 and not off_path and not held_long):
            # 직전 길을 유지하는 계획이면 경로를 다시 보내지 않음 (10/7 우분투 ①: 현성님 추종기는 경로를 받을 때마다 속도·조향을 0 부터 다시 올림)
            self.path_goal = res.get('plan_goal_odom')              # 옮긴 목표는 새로 본 만큼 바뀔 수 있음 → 도착·끝 판정은 새 값으로
            self.state, self.reason = 'OK', '정상 (직전 길 유지, 경로 그대로)'
            return
        self.path_pts = res['path']
        self.path_goal = res.get('plan_goal_odom')                    # 10/6 우분투: 원래 목표와 비교하면 옮긴 목표 0.4 m 앞부터 틱마다 재계획
        self.path_msg = self.make_path(res['path'], st)               # stamp = 계획에 쓴 위치(TF)의 시각
        self.path_z_known = self._mk_z_known
        self.pub_path.publish(self.path_msg)
        self.path_pub_t = self.tnow()
        self.stopped_published = False
        if res.get('route'):
            g = res['g']
            rp_ = [(*g.g_to_odom(g.center(rc))[:2], 0.0, 0.0, 1) for rc in res['route']]   # make_path 가 높이를 채움 (path_z)
            self.pub_route.publish(self.make_path(rp_, st))
        if self.p['min_seen_ahead'] > 0:
            sa = A.seen_fraction_ahead(self.amap, self.path_pts, self.p['ahead_check_m'], 0)
            if sa < self.p['min_seen_ahead']:
                self.stop('경로 앞 관측 %.0f%% < 기준 → 정지' % (100 * sa), 'UNSEEN_AHEAD')
                return
        self.state = 'OK'
        self.reason = '정상 (%s%s%s)' % ('막힘으로 다시 계획, ' if blocked else '', (dev + ' → 다시 계획, ') if dev else '',
                                       '직전 길 유지' if res.get('kept') else '새 길')


def main():
    rclpy.init()
    node = PlannerNode()
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    try:
        while rclpy.ok():
            try:
                ex.spin_once(timeout_sec=0.5)
            except (KeyboardInterrupt, ExternalShutdownException):
                raise
            except Exception as e:                                    # noqa: BLE001 — [261016] 콜백 예외 하나로 죽지 않게 (정지하고 계속)
                node.on_callback_error(e)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.stop('노드 종료', 'SHUTDOWN')
        except Exception:                                             # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():                                                # Ctrl-C 때 rclpy 가 이미 내려가 있으면 다시 부르지 않음 (이중 shutdown 오류)
            rclpy.shutdown()


if __name__ == '__main__':
    main()
