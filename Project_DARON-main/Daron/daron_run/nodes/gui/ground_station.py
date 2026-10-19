#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
ground_station — 지상국 제어창 (PyQt5 + rclpy). 작성 261006.  설계: ~/Daron/지상국 GUI 기능 계획.md

RViz 와 별도 프로세스다 — RViz 가 굳어도(261005 로그 §3 tf2 교착) 이 창의 상태·웨이포인트는 계속 동작한다.

화면
  상태      SLAM(/slam/status) · health(/health/state) · 전복(/ins/recalib_status, _event) · 경로계획(/planner/status)
            주요 토픽 수신 주기 (raw 구독 — 영상도 역직렬화하지 않음)
  자세      롤/피치 (attitude_topic 의 orientation). 경고 15° / 차단 25° = traversability.yaml slope_warn/block_deg
  웨이포인트  탑뷰·3D RViz 의 2D Goal Pose → click_topic. 목록 삭제·순서 변경·일시정지.
            플래너에는 지금 목표 하나만 준다 (gui/waypoint_seq.py — 이유는 거기 설명).
            /gui/waypoints (MarkerArray) 로 번호·순서선을 RViz 에 그린다.
            [261011] 도착 방위 = 2D Goal Pose 드래그 방향 (목록 '방위' 열, 플래너 목표 orientation 에 실음 —
              플래너는 아직 위치만 씀). 마커 = 원판(반경 wp_pos_tol_m) + 화살표(방위) + 부채꼴(±wp_yaw_tol_deg).
              도착 순간 위치·방위 오차는 기록만 (arrived 의 arrival_error, 판정 아님).
            [261011] '영상에서 찍기' 버튼 → D405 infra1 창 (gui/image_window.py, 계산 gui/image_click.py).
              누르고 드래그 = 위치 + 방향. 같은 목록에 source=image 로 추가. image_click:=false 면 버튼·구독 없음.
  크레이터 핀  pin_gallery 의 ~/pins_info 표. 행 클릭 또는 RViz 핀 클릭(/gui/pin_selected) → 정보 창
            (대표 이미지, 지름, 좌표, 검출 수, 확정 여부, 로버로부터 거리)
            + 림 선명도·가시 범위와 림 오버레이 (pin_gallery 가 top 박스 crop 으로 계산, 261007)
            + 열화도 순번 (261013): 정보 창 '순번 k / N' 버튼 → 전체 순서 창 (D 작을수록 열화 덜 됨 = 1번, 행 클릭 → 그 크레이터).
              D 를 못 만든 크레이터는 순서에 없음 (정보 창에 '계산 안 됨' + 사유).
            + 크레이터 형상 (261009): 상태(관측 중/계산 중/완료/관측 부족) · 직경 · 최대 내벽 경사 · 예측 깊이 · 깊이/직경 · 관측 수.
              pin_gallery 가 관측이 끝난 뒤 대표 프레임으로 한 번 계산한 결과를 읽기만 한다.
              3D mesh 는 정보 창 안 (gui/crater_mesh_view.py): 마우스 회전·휠 확대·가운데 이동, 만들어진 mesh 만 불러옴
  로버 제어   [261010] '수동' 버튼 = 키보드 조종 ON/OFF (gui/rover_panel.py · gui/rover_teleop.py). 제어팀 console.py 와 같은 규약으로
            <rover_ns>/steer_cmd·drive_cmd·articulation_cmd·inverted·command·heartbeat 송신, /joint_states·<rover_ns>/diagnostics 표시.
            ON 일 때만 보낸다 (OFF 면 침묵 → console.py 와 충돌 없음). 키는 이 창(또는 [상세] 창)이 활성일 때만, 다른 창으로 가면 즉시 정지.
            CSV xm430_/l12_/drive_*.csv (console.py 형식) → session_dir, 처음 ON 때 생성.
            [261012] '웨이포인트' 버튼 = 경로 추종기 ON/OFF (gui/follower_link.py): <follower_ns>/enable 송신, <follower_ns>/status 표시.
            추종기(rover/server/path_follower.py)는 daron_gui.launch.py 가 같이 띄운다. 창을 닫을 때 켜져 있으면 끈다.
            [261014] '자율' 버튼 = 완전 자율 ON/OFF (gui/auto_link.py, daron-nav auto-1014 경로계획 필요). 끄기 = 정지(MANUAL), 수동을 켜면 자율 꺼짐.
  완전 자율  [261014] 경기장 놓기(크기·출발 구석·거리·방향 → [여기가 출발점] = /ui/field here), 틈 숫자/[틈 찍기](다음 2D Goal Pose 한 번 = 틈,
            웨이포인트로 안 넣음 → /ui/gap_point), [점 건너뛰기]·[자율 기록 지우기], 상태(경기장 놓임·로버 경기장 좌표·목표 점·지나감·건너뜀·갇힘).
            경기장 테두리·출발·틈·방문점 그림은 경로계획 /planner/markers (탑뷰 RViz) — gui/auto_panel.py

시간: 끊김 판단·웨이포인트 대기는 벽시계(time.monotonic). bag 재생(use_sim_time)이어도 같다.

[261008] 웨이포인트 기록·불러오기
  좌표계   waypoint_frame (기본 odom, 나중에 map). 저장·불러오기·플래너 목표·마커가 전부 이 좌표계.
           click_topic 의 frame_id 가 다르면 **거부 + 경고** (TF 로 몰래 바꾸지 않음 — 전에는 TF 변환했다).
           ⚠ 3D 창(daron_viz3d.rviz, Fixed Frame map)에서 찍은 2D Goal Pose 는 waypoint_frame 이 odom 인 동안 거부된다.
  기록     <session_dir>/waypoints.jsonl 에 사건 한 줄씩 (줄마다 flush+fsync). 키 순서대로:
             {"event":"session_start", "session", "frame", "map_id", "session_dir", "pose_topic", "odom_topic", "t_ros", "t_wall"}
             {"event":"added",   "id", "goal":{"x","y","z"[,"yaw_deg"]}, ["source":"loaded:<파일>"|"session"|"image"|"rviz", "loaded_id", "loaded_session"],
                                 ["image":{"pixel","pixel_used","depth_m","stamp","yaw_source","dz_m"}], "frame", "session", "t_ros", "t_wall"}
             {"event":"arrived", "id", "goal", "arrived_pose":{"x","y","z","yaw_deg"}|null, "pose_frame", "pose_age_s",
                                 "arrival_error":{"pos_err_m","yaw_err_deg"}|null  ([261011] 기록만 — 판정 아님. pose_frame ≠ frame 이면 null)
                                 "odom_pose":{...}|null, "health":<최근 /health/state JSON>|null, "health_age_s", "frame", "session", "t_ros", "t_wall"}
             {"event":"removed", "id", "goal", ...}  {"event":"cleared", "ids", "goals", ...}  {"event":"reordered", "order", ...}
           arrived_pose = 도착 순간의 최근 arrival_pose_topic(/localization/pose — pose_relay 의 두 축 가운데 점),
           odom_pose = odom_topic(base_link). 도착 판정은 WaypointSeq 의 ARRIVED 분기 그대로 (on_arrived 콜백).
           t_ros = 노드 시계(use_sim_time 이면 bag 시각), t_wall = 벽시계. id 는 같은 파일의 기존 최대 id 다음부터.
           폴더가 없으면 첫 웨이포인트 때 만든다 (daron_viz 녹화가 '폴더 있음'으로 막히지 않게 — waypoint_seq.JsonlLog).
  불러오기  '웨이포인트 불러오기' → waypoints.jsonl 선택 (기본 위치 = 세션 폴더들의 부모).
           id 별 도착(파랑 원판)/미도착(주황 원판)을 /gui/waypoints 의 ns gui_loaded_waypoints 로 표시, 라벨 L<id>.
           '전부 다시 넣기'·'선택 다시 넣기' → 보통 추가 경로로 목록 끝에 (added 에 source 기록). 좌표 그대로.
           파일 좌표계 ≠ waypoint_frame → 거부. map_id·세션이 다르면 경고 창 후 불러옴.
           load_waypoints_file 파라미터 = 기동 시 표시만 (목록에 자동으로 넣지 않음).
"""

import json
import re
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.serialization import deserialize_message
from rclpy.time import Time
from geometry_msgs.msg import Point, PoseArray, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as PathMsg
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2
from tf2_ros import Buffer, TransformException, TransformListener
from std_msgs.msg import Int32, String
from visualization_msgs.msg import Marker, MarkerArray

from PyQt5 import QtCore, QtGui, QtWidgets
from PyQt5.QtCore import Qt

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from gui.image_click import DepthBuffer, compute_click, decode_depth, scale_uv, yaw_of_quat  # noqa: E402
from gui.image_window import ImageClickWindow, stamp_ns  # noqa: E402
from gui.follower_link import FollowerLink  # noqa: E402
from gui.auto_link import AutoLink  # noqa: E402  [261014] 완전 자율·경기장 놓기
from gui.auto_panel import AutoPanel  # noqa: E402
from gui.rover_panel import RoverPanel  # noqa: E402
from gui.rover_teleop import DEFAULT_MAX_STEER_DEG, RoverLink  # noqa: E402
from gui.waypoint_seq import (WP_LOG_NAME, JsonlLog, WaypointSeq, arrival_error, arrived_unique,  # noqa: E402
                              check_loaded, max_logged_id, reconstruct, wp_goal, wrap_pi)
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / '3dmap'))
import kf_resync as KR  # noqa: E402  [261011] 루프 클로저 보정 이력 (numpy 만)

ROOT = Path(__file__).resolve().parent.parent.parent      # ~/Daron/daron_run
MSG_TYPES = {'Image': Image, 'Odometry': Odometry, 'Imu': Imu, 'OccupancyGrid': OccupancyGrid,
             'String': String, 'Path': PathMsg, 'PointCloud2': PointCloud2}
DEFAULT_HZ_TOPICS = [
    '/camera/camera/infra1/image_rect_raw|Image',
    '/camera/camera/depth/image_rect_raw|Image',
    '/slam/odometry|Odometry',
    '/odometry/filtered_health|Odometry',
    '/imu/data_recalib|Imu',
    '/traversability/grid|OccupancyGrid',
    '/cda/detections|String',
    '/planner/path|Path',
]
SENSOR_QOS = QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
LATCHED = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.TRANSIENT_LOCAL)

GREEN, YELLOW, RED, GRAY, BLUE = '#2e9d4a', '#d39b00', '#d0342c', '#8a8a8a', '#2f6fd0'
LEVEL_COLOR = {'OK': GREEN, 'DEGRADED': YELLOW, 'FAIL': RED}
PLANNER_COLOR = {'OK': GREEN, 'ARRIVED': BLUE, 'NO_GOAL': GRAY, 'MANUAL': GRAY, 'AUTO_BOXED': RED, 'NO_FIELD': RED}   # [261014] 자율 갇힘·경기장 미지정


def read_slope_limits(path):
    """traversability.yaml 의 slope_warn_deg / slope_block_deg. 못 읽으면 15 / 25."""
    warn, block = 15.0, 25.0
    try:
        import yaml
        with open(path) as f:
            d = yaml.safe_load(f)

        def find(o, k):
            if isinstance(o, dict):
                if k in o:
                    return o[k]
                for v in o.values():
                    r = find(v, k)
                    if r is not None:
                        return r
            return None
        warn = float(find(d, 'slope_warn_deg') or warn)
        block = float(find(d, 'slope_block_deg') or block)
    except Exception:  # noqa: BLE001
        pass
    return warn, block


def quat_to_rp(x, y, z, w):
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return math.degrees(roll), math.degrees(pitch)


# ============================================================ ROS 쪽
class Latest:
    """콜백 스레드가 쓰고 Qt 스레드가 읽는 값 + 받은 시각 (대입 한 번이라 잠금 불필요)."""

    def __init__(self):
        self.v, self.t = None, None

    def set(self, v):
        self.v, self.t = v, time.monotonic()

    def age(self):
        return None if self.t is None else time.monotonic() - self.t


class GroundNode(Node):
    def __init__(self):
        super().__init__('ground_station')
        P = self.declare_parameter
        # [261008] 웨이포인트 좌표계 (전 odom_frame). 클릭·저장·불러오기·플래너 목표·마커 전부 이 좌표계
        self.waypoint_frame = str(P('waypoint_frame', 'map').value)    # [261008 밤] 기본 map (Slam 재추정·루프 클로저 반영 좌표)
        DYN = ParameterDescriptor(dynamic_typing=True)          # 숫자만인 이름(0930 등)이 INTEGER 로 와도 받게
        self.session_dir = os.path.abspath(os.path.expanduser(str(P('session_dir', '', DYN).value).strip()
                                                              or os.path.join('~', 'daron_gui_runs',
                                                                              time.strftime('%Y%m%d_%H%M%S'))))
        self.session_name = str(P('session_name', '', DYN).value).strip() or os.path.basename(self.session_dir)
        self.map_id = str(P('map_id', '', DYN).value).strip()
        self.load_file = os.path.expanduser(str(P('load_waypoints_file', '', DYN).value).strip())
        self.pose_topic = P('arrival_pose_topic', '/localization/pose').value
        self.click_topic = P('click_topic', '/gui/waypoint_goal').value
        self.goal_topic = P('planner_goal_topic', '/goal_pose').value
        self.wp_cmd_topic = P('planner_wp_cmd_topic', '/ui/waypoint_cmd').value
        self.select_topic = P('select_topic', '/gui/pin_selected').value
        self.odom_topic = odom_topic = P('odom_topic', '/odometry/filtered_health').value
        att_topic = P('attitude_topic', '/imu/data_recalib').value
        pins_topic = P('pins_topic', '/pin_gallery/pins_info').value
        hz_topics = P('hz_topics', DEFAULT_HZ_TOPICS).value
        self.trav_params = P('trav_params', str(ROOT / 'config' / 'traversability.yaml')).value
        self.window_geometry = str(P('window_geometry', '').value).strip()    # [261008] "WxH+X+Y" (녹화용 배치)
        # [261011] 웨이포인트 마커: 원판 반경 = 도착 위치 허용, 부채꼴 = 방위 허용 (지금은 표시용 — 플래너 판정은 나중)
        self.wp_pos_tol = float(P('wp_pos_tol_m', 0.10).value)
        self.wp_yaw_tol = math.radians(float(P('wp_yaw_tol_deg', 10.0).value))
        self.wp_arrow_len = float(P('wp_arrow_len_m', 0.30).value)
        # [261011] 영상 클릭 (gui/image_click.py)
        self.image_click_on = bool(P('image_click', True).value)
        self.image_topic = P('image_topic', '/camera/camera/infra1/image_rect_raw').value
        self.depth_topic = P('depth_topic', '/camera/camera/depth/image_rect_raw').value
        self.depth_info_topic = P('depth_info_topic', '/camera/camera/depth/camera_info').value
        self.depth_scale = float(P('depth_scale', 0.001).value)                # pin_node·live_tsdf·pin_gallery 와 같은 값
        self.base_frame = P('base_frame', 'base_link').value
        self.click_max_dz = float(P('image_click_max_dz_m', 0.5).value)        # 플래너 click_max_dz_m 과 같은 값
        self.click_min_drag = float(P('image_click_min_drag_px', 15.0).value)
        self.depth_match_s = float(P('image_depth_match_s', 0.034).value)      # infra1·depth 짝 허용 (1 프레임)
        # [261010] 로버 키보드 조종 (gui/rover_teleop.py). max_steer 는 로버도 자체 한계로 다시 자른다
        self.rover = RoverLink(self, ns=str(P('rover_ns', '/rover').value),
                               max_steer=float(P('max_steer_deg', DEFAULT_MAX_STEER_DEG).value),
                               log_dir=self.session_dir)
        self.follower = FollowerLink(self, ns=str(P('follower_ns', '/path_follower').value))   # [261012] 웨이포인트 버튼

        self.slam, self.health, self.recalib, self.recalib_event = Latest(), Latest(), Latest(), Latest()
        self.planner, self.planner_wps, self.pose, self.att, self.pins = Latest(), Latest(), Latest(), Latest(), Latest()
        self.clicks = []                                        # Qt 스레드가 꺼내 간다 (list.append/pop 은 원자적)
        self.selected = Latest()
        self.loc_pose = Latest()                                # [261008] 도착 기록용 (x, y, z, yaw, frame_id)
        self.reloc = Latest()                                   # [261008 밤] /slam/reloc_status (재추정 상태 JSON)
        self.hz = {}                                            # topic -> deque of monotonic
        self.auto = AutoLink(self, self.planner, frame=self.waypoint_frame, wp_cmd_topic=self.wp_cmd_topic)   # [261014] 자율 버튼·경기장 놓기

        J = self._json_into
        self.create_subscription(String, '/slam/status', J(self.slam), 10)
        self.create_subscription(String, '/health/state', J(self.health), 10)
        self.create_subscription(String, '/ins/recalib_status', lambda m: self.recalib.set(m.data), 10)
        self.create_subscription(String, '/ins/recalib_event', lambda m: self.recalib_event.set(m.data), 10)
        self.create_subscription(String, '/planner/status', J(self.planner), 10)
        self.create_subscription(String, '/slam/reloc_status', self._on_reloc, LATCHED)   # [261008 밤]
        self.create_subscription(PoseArray, '/planner/waypoints', self.on_planner_wps, LATCHED)
        self.create_subscription(Odometry, odom_topic, self.on_odom, SENSOR_QOS)
        self.create_subscription(Imu, att_topic, self.on_att, SENSOR_QOS)
        self.create_subscription(String, pins_topic, J(self.pins), LATCHED)
        self.create_subscription(PoseStamped, self.click_topic, lambda m: self.clicks.append(m), 10)
        self.create_subscription(Int32, self.select_topic, lambda m: self.selected.set(m.data), 10)
        self.create_subscription(PoseStamped, self.pose_topic, self.on_loc_pose, 10)
        for spec in hz_topics:
            topic, _, typ = spec.partition('|')
            if typ not in MSG_TYPES:
                self.get_logger().warn(f'hz_topics: 모르는 타입 {spec!r}')
                continue
            self.hz[topic] = []
            self.create_subscription(MSG_TYPES[typ], topic, self._hz_cb(topic), SENSOR_QOS, raw=True)

        # [261011] 영상 클릭: 창이 열려 있을 때만 역직렬화 (raw 구독 — 닫혀 있으면 콜백이 바로 돌아감)
        self.img_active = False
        self.img_latest, self.depth_info = Latest(), Latest()
        self.depth_buf = DepthBuffer(keep=45)                   # 1.5 s @30 Hz
        self.tf_buf = None
        if self.image_click_on:
            img_qos = QoSProfile(depth=2, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(Image, self.image_topic, self._on_img_raw, img_qos, raw=True)
            self.create_subscription(Image, self.depth_topic, self._on_depth_raw,
                                     QoSProfile(depth=5, history=HistoryPolicy.KEEP_LAST,
                                                reliability=ReliabilityPolicy.BEST_EFFORT), raw=True)
            self.create_subscription(CameraInfo, self.depth_info_topic, lambda m: self.depth_info.set(m), SENSOR_QOS)
        # [261011] TF 는 항상 (영상 클릭 + 로버 map 위치 + 루프 클로저 기준 map<-odom)
        self.tf_buf = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buf, self)
        # [261011] 루프 클로저 재동기화: 보정 이력 + 지도 칸(마지막 관측 시각 = 지도 클릭 웨이포인트의 기준 시각)
        self.odom_frame = str(P('odom_frame', 'odom').value)
        self.lc_on = bool(P('lc_resync', True).value)
        self.lc_tol_ns = int(float(P('lc_resync_tol_s', 0.1).value) * 1e9)
        self.lc_hist = Latest()
        self.cells_raw = Latest()
        if self.lc_on:
            self.create_subscription(PathMsg, str(P('lc_history_topic', '/slam/map_odom_history').value),
                                     lambda m: self.lc_hist.set(KR.history_from_path_msg(m)), LATCHED)
            self.create_subscription(PointCloud2, str(P('cells_topic', '/live_tsdf/map_cells').value),
                                     lambda raw: self.cells_raw.set(raw), 1, raw=True)

        self.pub_goal = self.create_publisher(PoseStamped, self.goal_topic, 10)
        self.pub_wp_cmd = self.create_publisher(String, self.wp_cmd_topic, 10)
        self.pub_select = self.create_publisher(Int32, self.select_topic, 10)
        self.pub_wp_mk = self.create_publisher(MarkerArray, '/gui/waypoints', LATCHED)
        self.get_logger().info(f'웨이포인트 입력 {self.click_topic} → 플래너 {self.goal_topic} / {self.wp_cmd_topic}, '
                               f'포즈 {odom_topic}, 자세 {att_topic}, 핀 {pins_topic}')
        if self.image_click_on:
            self.get_logger().info(f'[261011] 영상 클릭 {self.image_topic} + {self.depth_topic} ({self.depth_info_topic}), '
                                   f'TF {self.waypoint_frame} ← depth 프레임, 바닥 검사 ±{self.click_max_dz} m ({self.base_frame})')
        self.get_logger().info(f'[261008] 웨이포인트 좌표계 {self.waypoint_frame}, 기록 '
                               f'{os.path.join(self.session_dir, WP_LOG_NAME)} (세션 {self.session_name}, '
                               f'map_id {self.map_id!r}), 도착 포즈 {self.pose_topic}')

    def now_ros(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------------- [261011] map 좌표 로버 · 루프 클로저 기준 ----------------
    def _T(self, target, source):
        try:
            tr = self.tf_buf.lookup_transform(target, source, Time())
        except TransformException:
            return None
        t, q = tr.transform.translation, tr.transform.rotation
        T = KR.np.eye(4)
        T[:3, :3] = KR.quats_to_mats([(q.x, q.y, q.z, q.w)])[0]
        T[:3, 3] = (t.x, t.y, t.z)
        return T

    def rover_wp(self):
        """로버 (x, y, z, yaw) — waypoint_frame 좌표 (TF waypoint_frame <- base). 웨이포인트·핀과 같은 좌표라 거리·마커가 맞다.
        TF 가 없으면 None (호출 쪽이 옛 동작 = odom 포즈로)."""
        T = self._T(self.waypoint_frame, self.base_frame)
        if T is None:
            return None
        return (float(T[0, 3]), float(T[1, 3]), float(T[2, 3]), math.atan2(T[1, 0], T[0, 0]))

    def lc_anchor(self, x, y, source, stamp_s=None):
        """웨이포인트 기준 (t_ns, M_ref, 출처) 또는 None. 루프 클로저 보정 이력이 오면 이 시각의 보정량만큼 옮긴다.
        image  = 클릭한 영상 stamp (그 프레임 포즈로 계산했으므로)
        rviz·session = 그 자리 지도 칸의 마지막 관측 시각 (/live_tsdf/map_cells 의 age). 칸이 없으면 지금 시각.
        M_ref = 그 시각에 지도가 쓰던 map<-odom: 받은 보정 이력에 있으면 그 값, 없으면 지금 TF."""
        if not self.lc_on:
            return None
        t, src = None, 'now'
        if source == 'image' and stamp_s is not None:
            t, src = int(round(float(stamp_s) * 1e9)), 'image'
        elif source in ('rviz', 'session'):
            t = self._cell_time(x, y)
            if t is not None:
                src = 'cell'
        if t is None:
            t = self.get_clock().now().nanoseconds
        h = self.lc_hist.v
        M = h.lookup(t, self.lc_tol_ns) if h is not None else None
        if M is None:
            M = self._T(self.waypoint_frame, self.odom_frame)
        return None if M is None else (t, M, src)

    def _cell_time(self, x, y, r=0.05):
        raw = self.cells_raw.v
        if raw is None:
            return None
        try:
            from sensor_msgs_py import point_cloud2
            m = deserialize_message(raw, PointCloud2)
            if m.header.frame_id != self.waypoint_frame or 'age' not in [f.name for f in m.fields]:
                return None
            a = point_cloud2.read_points_numpy(m, field_names=('x', 'y', 'age'), skip_nans=True)
        except Exception:  # noqa: BLE001
            return None
        if len(a) == 0:
            return None
        d = KR.np.hypot(a[:, 0] - x, a[:, 1] - y)
        i = int(KR.np.argmin(d))
        if d[i] > r:
            return None
        return m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec + int(round(float(a[i, 2]) * 1e9))

    def _on_reloc(self, m):
        """[261008 밤] 재추정 상태. map_id 인자를 안 줬으면 재추정 결과의 map_id 를 이 세션 map_id 로 쓴다."""
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        self.reloc.set(d)
        if not self.map_id and d.get('map_id'):
            self.map_id = str(d['map_id'])

    def _json_into(self, latest):
        def cb(m):
            try:
                latest.set(json.loads(m.data))
            except ValueError:
                pass
        return cb

    def _hz_cb(self, topic):
        def cb(_raw):
            q = self.hz[topic]
            now = time.monotonic()
            q.append(now)
            while q and q[0] < now - 3.0:
                q.pop(0)
        return cb

    def hz_of(self, topic, window=2.0):
        now = time.monotonic()
        n = sum(1 for t in list(self.hz.get(topic, [])) if t >= now - window)
        return n / window

    def on_planner_wps(self, m):
        self.planner_wps.set([(p.position.x, p.position.y) for p in m.poses])

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.pose.set((p.x, p.y, p.z, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))))

    def on_att(self, m):
        q = m.orientation
        if m.orientation_covariance[0] < 0 or (q.x == q.y == q.z == q.w == 0):
            return                                              # 자세 없는 IMU 메시지
        self.att.set(quat_to_rp(q.x, q.y, q.z, q.w))

    def on_loc_pose(self, m):
        p, q = m.pose.position, m.pose.orientation
        self.loc_pose.set((p.x, p.y, p.z, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
                           m.header.frame_id))

    # ---- 클릭 좌표. [261008] 좌표계가 waypoint_frame 이 아니면 거부 (전에는 TF 로 odom 변환 — 저장 좌표가 섞이지 않게)
    def click_xyz(self, msg):
        """→ (x, y, z, yaw|None) 또는 거부 사유 문자열. frame_id 가 비었으면 waypoint_frame 으로 본다 (전과 같음).
        [261011] yaw = 2D Goal Pose 드래그 방향 (orientation 이 전부 0 이면 None)."""
        p, q = msg.pose.position, msg.pose.orientation
        frame = msg.header.frame_id or self.waypoint_frame
        if frame != self.waypoint_frame:
            return (f'클릭 좌표계 {frame!r} ≠ waypoint_frame {self.waypoint_frame!r} → 거부 '
                    f'(RViz Fixed Frame 을 {self.waypoint_frame} 로)')
        return p.x, p.y, p.z, yaw_of_quat(q.x, q.y, q.z, q.w)

    # ---- [261011] 영상 클릭
    def set_image_active(self, on):
        self.img_active = bool(on)
        if not on:
            self.img_latest.v = None
            self.depth_buf.clear()

    def _on_img_raw(self, raw):
        if self.img_active:
            self.img_latest.set(deserialize_message(raw, Image))

    def _on_depth_raw(self, raw):
        if self.img_active:
            m = deserialize_message(raw, Image)
            self.depth_buf.push(stamp_ns(m), m)

    def _tf_at(self, target, source, stamp):
        """→ ((t), (q)) 또는 사유 문자열. 영상 stamp 시각의 TF (WiFi 지연 동안 움직여도 맞게)."""
        try:
            tr = self.tf_buf.lookup_transform(target, source, Time.from_msg(stamp), timeout=Duration(seconds=0.2))
        except TransformException as e:
            return f'TF {target} ← {source} 없음 ({type(e).__name__})'
        t, q = tr.transform.translation, tr.transform.rotation
        return (t.x, t.y, t.z), (q.x, q.y, q.z, q.w)

    def image_click(self, img, press_uv, release_uv):
        """화면에 고정한 infra1 프레임 img 위의 누른 점·뗀 점(화소, 드래그 짧으면 None) → compute_click 결과 dict."""
        fail = dict(ok=False, notes=[], point=None, yaw=None, yaw_source=None, uv_used=None, depth_m=None)
        if img is None:
            return dict(fail, reason='영상 없음')
        dm, dt = self.depth_buf.nearest(stamp_ns(img), self.depth_match_s)
        if dm is None:
            return dict(fail, reason='같은 시각 depth 없음' + (f' (가장 가까운 것 {dt * 1000:+.0f} ms)' if dt is not None else
                                                         f' — {self.depth_topic} 안 들어옴 (depth 꺼짐?)'))
        info = self.depth_info.v
        if info is None:
            return dict(fail, reason=f'{self.depth_info_topic} 없음')
        try:
            depth = decode_depth(dm.encoding, dm.height, dm.width, dm.step, dm.data, self.depth_scale)
        except ValueError as e:
            return dict(fail, reason=str(e))
        tf = self._tf_at(self.waypoint_frame, dm.header.frame_id, dm.header.stamp)
        if isinstance(tf, str):
            return dict(fail, reason=tf)
        base = self._tf_at(self.waypoint_frame, self.base_frame, dm.header.stamp)
        sc = (img.width, img.height), (dm.width, dm.height)
        pu = scale_uv(press_uv[0], press_uv[1], *sc)
        ru = scale_uv(release_uv[0], release_uv[1], *sc) if release_uv is not None else None
        res = compute_click(pu, ru, depth, list(info.k), tf, None if isinstance(base, str) else base[0],
                            max_dz=self.click_max_dz, min_drag_px=self.click_min_drag)
        res.update(stamp=dm.header.stamp.sec + dm.header.stamp.nanosec * 1e-9, depth_dt_s=dt)
        return res

    # ---- 출력
    def send(self, act):
        if act[0] == 'clear':
            self.pub_wp_cmd.publish(String(data='CLEAR'))
        else:
            m = PoseStamped()
            m.header.frame_id = self.waypoint_frame
            m.header.stamp = self.get_clock().now().to_msg()
            m.pose.position.x, m.pose.position.y, m.pose.position.z = act[1], act[2], act[3]
            yaw = act[4] if len(act) > 4 else None             # [261011] 도착 방위 (플래너는 아직 위치만 씀)
            if yaw is None:
                # [261016] 방향 없음 = (0,0,0,0) — 플래너(goal_pose_heading) 규약. 예전 w=1.0(항등)은 'map +x 로 도착'으로 계획됐다.
                # Quaternion 기본값이 w=1 이라 0 을 직접 넣는다.
                m.pose.orientation.w = 0.0
            else:
                m.pose.orientation.z, m.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            self.pub_goal.publish(m)

    def publish_wp_markers(self, items, active_id, rover, loaded=None, clear_loaded=False):
        """loaded: [261008] 불러온 웨이포인트 (reconstruct 항목) — 같은 배열의 ns gui_loaded_waypoints 로.
        clear_loaded: 한 번이라도 불러온 걸 그렸으면 True — 그 ns 도 지운다 (DELETEALL 이 ns 별인지와 무관하게 남지 않게).
        아무것도 안 불러왔으면 배열 내용이 전과 같다."""
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        def mk(kind, i, rgba, sx, sy, sz, ns='gui_waypoints'):
            m = Marker()
            m.header.frame_id, m.header.stamp = self.waypoint_frame, stamp
            m.ns, m.id, m.type, m.action = ns, i, kind, Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = sx, sy, sz
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            return m
        # 마커 id 는 1 부터: 같은 배열에서 DELETEALL 뒤에 id 0 을 ADD 하면 RViz(Humble)가 디스플레이를 Error 로 표시 (261006 실측)
        # DELETEALL 은 전부 ADD 앞에 (Humble 의 DELETEALL 이 ns 를 안 가리면 뒤에 둔 것이 앞의 ADD 를 지운다)
        for ns in ('gui_waypoints',) + (('gui_loaded_waypoints',) if clear_loaded else ()):
            clear = Marker()
            clear.header.frame_id, clear.header.stamp = self.waypoint_frame, stamp
            clear.ns, clear.action = ns, Marker.DELETEALL
            clear.pose.orientation.w = 1.0
            arr.markers.append(clear)
        if items:
            line = mk(Marker.LINE_STRIP, 1, (0.2, 0.6, 1.0, 0.8), 0.01, 0.01, 0.01)   # 폭 = x, y·z 는 0 이면 RViz 오류
            pts = ([rover[:3]] if rover else []) + [(it['x'], it['y'], it['z']) for it in items]
            line.points = [Point(x=float(a), y=float(b), z=float(c) + 0.01) for a, b, c in pts]
            arr.markers.append(line)
        # [261011] 웨이포인트 = 원판(반경 wp_pos_tol — 도착 위치 허용) + 번호 + 화살표(도착 방위) + 부채꼴(±wp_yaw_tol).
        #   도착 때 로버가 원판 안인지·로버 방향이 부채꼴 안인지 RViz 에서 눈으로 확인. 방위 없는 항목은 원판·번호만.
        #   id = 2 + 4k + (0 원판, 1 번호, 2 화살표, 3 부채꼴)
        for k, it in enumerate(items):
            act = it['id'] == active_id
            col = (0.1, 0.85, 0.3) if act else (0.2, 0.5, 1.0)
            base = 2 + 4 * k
            d = 2 * self.wp_pos_tol
            c = mk(Marker.CYLINDER, base, col + (0.45,), d, d, 0.01)
            c.pose.position.x, c.pose.position.y, c.pose.position.z = it['x'], it['y'], it['z'] + 0.005
            t = mk(Marker.TEXT_VIEW_FACING, base + 1, (1.0, 1.0, 1.0, 1.0), 0.12, 0.12, 0.12)
            t.pose.position.x, t.pose.position.y, t.pose.position.z = it['x'], it['y'], it['z'] + 0.12
            t.text = str(k + 1)
            arr.markers += [c, t]
            if it.get('yaw') is not None:
                arr.markers += self._heading_markers(mk, base + 2, it, col, 'gui_waypoints', self.wp_yaw_tol)
        # [261008] 불러온 웨이포인트: 작은 원판 — 도착 파랑 / 미도착 주황, 라벨 L<id>. [261011] 방위 있으면 화살표 (부채꼴 없음)
        #   id = 1 + 3k + (0 원판, 1 라벨, 2 화살표)
        for k, w in enumerate(loaded or []):
            col = (0.15, 0.3, 0.95, 0.85) if w['status'] == 'arrived' else (1.0, 0.55, 0.0, 0.9)
            c = mk(Marker.CYLINDER, 1 + 3 * k, col, 0.16, 0.16, 0.02, ns='gui_loaded_waypoints')
            c.pose.position.x, c.pose.position.y, c.pose.position.z = w['x'], w['y'], w['z'] + 0.01
            t = mk(Marker.TEXT_VIEW_FACING, 2 + 3 * k, col[:3] + (1.0,), 0.1, 0.1, 0.1, ns='gui_loaded_waypoints')
            t.pose.position.x, t.pose.position.y, t.pose.position.z = w['x'] + 0.1, w['y'] + 0.1, w['z'] + 0.05
            t.text = f'L{w["id"]}'
            arr.markers += [c, t]
            if w.get('yaw') is not None:
                arr.markers += self._heading_markers(mk, 3 + 3 * k, w, col[:3], 'gui_loaded_waypoints', None, alpha=0.8)
        self.pub_wp_mk.publish(arr)

    def _heading_markers(self, mk, mid, it, rgb, ns, tol, alpha=1.0):
        """[261011] 화살표(id mid) + 부채꼴(id mid+1, tol 이 있을 때). 원판 가운데에서 yaw 방향으로."""
        yaw, L = it['yaw'], self.wp_arrow_len
        qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
        a = mk(Marker.ARROW, mid, tuple(rgb) + (alpha,), L, 0.03, 0.03, ns=ns)   # scale.x 길이, y·z 굵기
        a.pose.position.x, a.pose.position.y, a.pose.position.z = it['x'], it['y'], it['z'] + 0.03
        a.pose.orientation.z, a.pose.orientation.w = qz, qw
        out = [a]
        if tol:
            f = mk(Marker.TRIANGLE_LIST, mid + 1, tuple(rgb) + (0.3,), 1.0, 1.0, 1.0, ns=ns)
            f.pose.position.x, f.pose.position.y, f.pose.position.z = it['x'], it['y'], it['z'] + 0.015
            f.pose.orientation.z, f.pose.orientation.w = qz, qw
            n = 8
            angs = [-tol + 2 * tol * i / n for i in range(n + 1)]
            for a0, a1 in zip(angs[:-1], angs[1:]):              # 원판 가운데를 꼭짓점으로 하는 부채꼴 (반경 = 화살표 길이)
                f.points += [Point(x=0.0, y=0.0, z=0.0),
                             Point(x=L * math.cos(a0), y=L * math.sin(a0), z=0.0),
                             Point(x=L * math.cos(a1), y=L * math.sin(a1), z=0.0)]
            out.append(f)
        return out


# ============================================================ Qt 위젯
def fit_columns(table):
    """열 너비 = 내용, 마지막 열이 남은 폭 (가로 스크롤 없이 전부 보이게)"""
    h = table.horizontalHeader()
    for c in range(table.columnCount() - 1):
        h.setSectionResizeMode(c, QtWidgets.QHeaderView.ResizeToContents)
    h.setSectionResizeMode(table.columnCount() - 1, QtWidgets.QHeaderView.Stretch)


_UNSET = object()
RIM_REASON = {'bbox_too_small': '박스가 작음 (한 변 20 px 미만) — 더 가까이서 검출되면 계산',
              'insufficient_image_contrast': '영상 대비 부족',
              'insufficient_valid_observations': '림 경계 관측 부족',
              'no_connected_rim_candidates': '연결된 림 경계 없음',
              'no_accepted_lorentzian_fit': '모델 적합 통과 구간 없음',
              'empty_crop': '박스가 영상 밖'}


def rim_width(rim, key, status_key, reason_key, n=None):
    """폭 한 줄. 못 구하면 0 이 아니라 '—' + 사유 (전달본: 확보 못한 폭은 null)."""
    v = rim.get(key)
    if v is None or rim.get(status_key) == 'pending':
        why = RIM_REASON.get(rim.get(reason_key), rim.get(reason_key) or '')
        return f'— <span style="color:{GRAY}">{why}</span>'
    return f'<b>{v:.3f}</b>' + (f' <span style="color:{GRAY}">({n}구간)</span>' if n is not None else '')


def rim_cell(rim):
    """핀 표의 '림' 열 = 선명도 · 가시 범위. pending·error 는 0 이 아니라 '대기'·'오류'"""
    if not rim:
        return '—'
    if rim.get('status') == 'valid':
        return f'{rim["rim_sharpness_raw"]:.2f} · {100 * rim["rim_completeness"]:.0f}%'
    return '대기' if rim.get('status') == 'pending' else '오류'


def rim_html(rim):
    if not rim:
        return f'<span style="color:{GRAY}">— (대표 이미지 없음)</span>'
    st = rim.get('status')
    if st != 'valid':
        why = RIM_REASON.get(rim.get('reason'), rim.get('reason') or '')
        return f'<span style="color:{GRAY}">{"계산 대기" if st == "pending" else "계산 오류"}: {why}</span>'
    if not rim.get('accepted_sectors'):
        return f'<b>림 확인 안 됨</b> (0/{rim["total_sectors"]} 방향)'
    return (f'선명도 <b>{rim["rim_sharpness_raw"]:.2f}</b><br>'
            f'가시 범위 <b>{100 * rim["rim_completeness"]:.0f}%</b> ({rim["accepted_sectors"]}/{rim["total_sectors"]})<br>'
            f'최장 연속 호 {100 * rim["visible_arc_fraction"]:.0f}%<br>'
            + (f'경계 폭 관측 {rim_width(rim, "median_edge_width_r", "width_status", "width_reason")}'
               f' · 모델 {rim_width(rim, "lorentzian_median_width_r", "model_width_status", "model_width_reason", rim.get("lorentzian_fitted_sectors"))}<br>'
               if 'median_edge_width_r' in rim else '') +
            f'<span style="color:{GRAY}">상대값 — 확률·연령 아님 · 폭 단위 = 박스 반경 비<br>경계 품질 {rim["observation_quality"]:.2f}</span>')


CRATER_STATE = {'ACTIVE': ('관측 중', BLUE), 'FINALIZING': ('계산 중', YELLOW), 'COMPLETE': ('완료', GREEN),
                'INSUFFICIENT_OBSERVATION': ('관측 부족', RED)}


def crater_html(c):
    """[261009] 크레이터 칸. 관측이 끝난 뒤 한 번 계산한 값만 보여 준다 (pin_gallery 가 계산, 여기선 읽기만)."""
    if not c:
        return '— (크레이터 분석 꺼짐 또는 pin_gallery 결과 없음)'
    name, color = CRATER_STATE.get(c.get('state'), (c.get('state') or '?', GRAY))
    ago = c.get('last_observed_ago_s')
    head = (badge(name, color) + f' &nbsp; 관측 {c.get("observation_count", 0)}회'
            + (f' · 마지막 검출 {ago:.1f} s 전' if ago is not None else ''))
    note = {'ACTIVE': '관측이 끝나면(검출이 끊기면) 대표 이미지를 확정하고 한 번 계산합니다.',
            'FINALIZING': '대표 이미지 확정 — 직경 · 경사 · 깊이 · 3D mesh 계산 중'}.get(c.get('state'), '')
    rows = ''
    if c.get('geometry_status') == 'COMPLETE':
        rows = (f'<table cellspacing="3">'
                f'<tr><td>직경 (Diameter)</td><td><b>{c["diameter_m"] * 100:.1f} cm</b></td></tr>'
                f'<tr><td>최대 내벽 경사</td><td><b>{c["max_inner_wall_slope_deg"]:.1f}°</b> (관측 구간)</td></tr>'
                f'<tr><td>예측 깊이</td><td><b>{c["predicted_depth_m"] * 100:.1f} cm</b> (모델 추정)</td></tr>'
                f'<tr><td>깊이/직경</td><td><b>{c["depth_to_diameter_ratio"]:.3f}</b></td></tr>'
                f'<tr><td>관측 수 (계산 시점)</td><td><b>{c.get("observation_count_at_geometry", "—")}</b></td></tr>'
                f'</table>')
        if c.get('state') == 'ACTIVE':
            note = '다시 관측 중 — 아래는 지난 계산값. 대표 이미지가 바뀌면 관측이 끝난 뒤 다시 계산합니다.'
    elif c.get('geometry_status') == 'INSUFFICIENT_OBSERVATION' or c.get('state') == 'INSUFFICIENT_OBSERVATION':
        rows = f'계산 안 함: <b>{c.get("reason")}</b>'
    return head + (f'<br>{note}' if note else '') + (f'<br>{rows}' if rows else '')


# [261013] 열화도 — pin_gallery 의 degradation (gui/degradation_bridge.py) 을 읽기만 한다
DEG_REASON = {'acquisition_unverified': '촬영 조건(자세·조명) 미확인', 'identity_unconfirmed': '핀 미확정',
              'insufficient_time_blocks': '5 s 구간 부족', 'insufficient_time_span': '관측 시간 부족',
              'unstable_observed_slope': '경사 시간 변동 큼',
              'missing_or_nonfinite_slope_deg': '경사 측정 실패', 'missing_geometry_flags': '경사 측정 실패',
              'missing_or_nonfinite_rim_score': '림 측정 실패', 'rim_measurement_not_valid': '림 측정 실패',
              'quality_wall_sectors': '내벽 방향 부족', 'missing_quality_wall_sectors': '내벽 방향 부족',
              'quality_wall_windows': '내벽 구간 부족', 'missing_quality_wall_windows': '내벽 구간 부족',
              'missing_quality_background_delta_deg': '지형 보정 비교 불가',
              'quality_background_delta_deg': '지형 보정에 민감 (plane/quadratic 차 > 5°)',
              'quality_bbox_min_side_px': '박스 작음 (32 px 미만)', 'truncated_or_unknown_bbox': '박스가 화면 가장자리',
              'quality_depth_valid_fraction': 'depth 유효 부족', 'quality_saturation_fraction': '영상 포화',
              'geometry_weak_height_signal_relative_to_ground_residual': '높이 신호 약함 (지면 잔차 수준)',
              'geometry_weak_observed_depression_evidence': '함몰 근거 약함',
              'geometry_wall_signal_not_resolved_above_background': '내벽 신호 약함'}
DEG_META = ('acquisition_unverified', 'identity_unconfirmed')      # 보류 사유일 뿐 D 계산은 막지 않음


def deg_reason_txt(reasons, n=3):
    """사유 → 한글. dict(사유: 프레임 수) 또는 list. 보류용 메타 사유는 뺀다."""
    items = reasons.items() if isinstance(reasons, dict) else ((r, None) for r in reasons)
    seen, out = set(), []
    for r, c in sorted(items, key=lambda kv: -(kv[1] or 0)):
        if r in DEG_META:
            continue
        t = DEG_REASON.get(r, r)
        if t not in seen:
            seen.add(t)
            out.append(t + (f' ×{c}' if c else ''))
    return ', '.join(out[:n]) + (' …' if len(out) > n else '')


def deg_html(d):
    """정보 창 열화도 칸 (순번은 버튼, 여기는 값·상태)."""
    if not d:
        return f'<span style="color:{GRAY}">— (열화도 꺼짐 또는 pin_gallery 결과 없음)</span>'
    st = d.get('status')
    if st == 'observing':
        return '관측 중 — 검출이 끊기면 대표·구간 프레임으로 경사 P95 · 림 선명도를 측정합니다.'
    if st == 'measuring':
        return '측정 중 — 경사 P95 · 림 선명도'
    if st == 'no_index':
        why = deg_reason_txt(d.get('reasons') or {})
        return (f'<b>계산 안 됨</b> — 전체 순서에서 제외<br><span style="color:{GRAY}">'
                + (f'측정 프레임 {d["raw_count"]}장 · ' if d.get('raw_count') else '')
                + (why or d.get('note') or '경사·림 두 값이 함께 나온 프레임 없음') + '</span>')
    tag = badge('확정', GREEN) if st == 'ranked' else badge('보류', YELLOW)
    hold = deg_reason_txt(d.get('reasons') or [])
    meta = [DEG_REASON[r] for r in d.get('reasons') or [] if r in DEG_META]
    return (f'{tag} &nbsp; D = <b>{d["D"]:.1f}</b> <span style="color:{GRAY}">(0~100, 작을수록 열화 덜 됨)</span>'
            f'<table cellspacing="3">'
            f'<tr><td>관측 내벽 경사 P95</td><td><b>{d["slope_deg"]:.1f}°</b> → 기여 {d["slope_contribution"]:.1f}</td></tr>'
            f'<tr><td>림 선명도 S</td><td><b>{d["rim_score"]:.3f}</b> → 기여 {d["rim_contribution"]:.1f}</td></tr>'
            f'<tr><td>5 s 구간</td><td>{d["blocks"]}개 · {d["span_seconds"]:.0f} s</td></tr></table>'
            f'<span style="color:{GRAY}">값 = 5 s 구간별 프레임의 중앙값 (아래 림 칸은 대표 1장)</span><br>'
            + (f'<span style="color:{GRAY}">보류 사유: {", ".join(x for x in [hold] + meta if x)}</span><br>' if st != 'ranked' else '')
            + (f'<span style="color:{GRAY}">기준 범위 밖 값 — 0/100 에서 잘림</span><br>' if d.get('reference_extrapolation') else '')
            + ('<span style="color:{0}">다시 관측 중 — 끝나면 새 프레임을 넣어 다시 계산</span>'.format(BLUE) if d.get('measuring') else ''))


def deg_ref_txt(info):
    """전체 순서 창 머리글의 기준 범위 설명."""
    ref = (info or {}).get('reference')
    if not ref:
        return info.get('note') if info else None
    rng = f'경사 {ref["slope_low"]:.1f}~{ref["slope_high"]:.1f}° · 림 S {ref["rim_low"]:.3f}~{ref["rim_high"]:.3f}'
    if info.get('provisional_reference'):
        return f'기준 = <b>임시</b> (이번 미션 크레이터 범위: {rng}) — 크레이터가 늘면 값·순서가 바뀔 수 있음'
    return f'기준 = 고정 <b>{ref["reference_id"]}</b> ({rng})'


def yaw_txt(it):
    """[261011] 목록 '방위' 열 — waypoint_frame 기준 yaw, 없으면 —"""
    return '—' if it.get('yaw') is None else f'{math.degrees(it["yaw"]):+.0f}'


def done_txt(d):
    """[261011] 도착 줄: 좌표 + 도착 순간 오차 (기록만, 판정 아님)"""
    s = f'({d["x"]:+.2f}, {d["y"]:+.2f})'
    e = d.get('err')
    if e:
        s += f' Δ{e["pos_err_m"] * 100:.0f}cm' + (f' {e["yaw_err_deg"]:+.0f}°' if e['yaw_err_deg'] is not None else '')
    return s


def badge(text, color):
    return f'<span style="background:{color};color:white;padding:1px 6px;border-radius:3px;"><b>{text}</b></span>'


class AttitudeBar(QtWidgets.QWidget):
    """-span..+span 막대. 경고/차단 구간 색, 바늘 = 현재 각."""

    def __init__(self, name, warn, block, span=40.0):
        super().__init__()
        self.name, self.warn, self.block, self.span = name, warn, block, span
        self.value = None
        self.setMinimumHeight(46)

    def set_value(self, v):
        self.value = v
        self.update()

    def paintEvent(self, _e):
        p = QtGui.QPainter(self)
        p.setRenderHint(QtGui.QPainter.Antialiasing)
        w, h = self.width(), self.height()
        x0, x1, y0, y1 = 70, w - 70, 8, h - 14

        def X(a):
            return x0 + (max(-self.span, min(self.span, a)) + self.span) / (2 * self.span) * (x1 - x0)
        zones = [(-self.span, -self.block, RED), (-self.block, -self.warn, YELLOW), (-self.warn, self.warn, GREEN),
                 (self.warn, self.block, YELLOW), (self.block, self.span, RED)]
        for a, b, c in zones:
            col = QtGui.QColor(c)
            col.setAlpha(90)
            p.fillRect(QtCore.QRectF(X(a), y0, X(b) - X(a), y1 - y0), col)
        p.setPen(QtGui.QPen(QtGui.QColor('#555'), 1))
        p.drawRect(QtCore.QRectF(x0, y0, x1 - x0, y1 - y0))
        f = p.font()
        f.setPointSize(8)
        p.setFont(f)
        for a in (-self.block, -self.warn, 0, self.warn, self.block):
            p.drawText(QtCore.QRectF(X(a) - 20, y1, 40, 14), Qt.AlignCenter, f'{a:+.0f}' if a else '0')
        f.setPointSize(10)
        p.setFont(f)
        p.drawText(QtCore.QRectF(0, y0, x0 - 6, y1 - y0), Qt.AlignRight | Qt.AlignVCenter, self.name)
        if self.value is None:
            p.drawText(QtCore.QRectF(x1 + 6, y0, 70, y1 - y0), Qt.AlignLeft | Qt.AlignVCenter, '—')
            return
        v = self.value
        c = RED if abs(v) >= self.block else YELLOW if abs(v) >= self.warn else '#222'
        p.setPen(QtGui.QPen(QtGui.QColor(c), 3))
        p.drawLine(QtCore.QPointF(X(v), y0 - 3), QtCore.QPointF(X(v), y1 + 1))
        p.drawText(QtCore.QRectF(x1 + 6, y0, 70, y1 - y0), Qt.AlignLeft | Qt.AlignVCenter, f'{v:+.1f}°')


class CraterDialog(QtWidgets.QDialog):
    """핀 하나의 정보 창 (비모달, 하나를 재사용). 열려 있는 동안 거리·검출 수가 계속 갱신된다."""

    def __init__(self, parent, on_order=None):
        super().__init__(parent)
        self.setWindowTitle('크레이터 정보')
        self.pid = None
        self.on_order = on_order                                # [261013] 열화도 순번 클릭 → 전체 순서 창
        self._img_path = self._rim_path = _UNSET
        lay = QtWidgets.QVBoxLayout(self)
        lay.setSizeConstraint(QtWidgets.QLayout.SetMinimumSize)  # 글이 길어지면 창이 늘어난다 (잘리지 않게)
        self.img = QtWidgets.QLabel('이미지 없음')
        self.img.setAlignment(Qt.AlignCenter)
        self.img.setMinimumSize(424, 240)
        self.img.setStyleSheet('background:#222;color:#bbb;')
        self.info = QtWidgets.QLabel()
        self.info.setTextFormat(Qt.RichText)
        self.info.setTextInteractionFlags(Qt.TextSelectableByMouse)
        lay.addWidget(self.img)
        lay.addWidget(self.info)
        deg_box = QtWidgets.QGroupBox('열화도 (관측 내벽 경사 P95 0.5 + 림 선명도 0.5)')   # [261013]
        dl = QtWidgets.QHBoxLayout(deg_box)
        self.deg_btn = QtWidgets.QPushButton('순번 —')
        self.deg_btn.setToolTip('클릭 → 전체 순서 (열화 덜 된 것 = 1번)')
        self.deg_btn.setMinimumWidth(120)
        self.deg_btn.setStyleSheet('QPushButton{font-size:16px;font-weight:bold;padding:6px 10px;}')
        self.deg_btn.clicked.connect(lambda: self.on_order and self.on_order(self.pid))
        self.deg_info = QtWidgets.QLabel('—')
        self.deg_info.setTextFormat(Qt.RichText)
        self.deg_info.setWordWrap(True)
        self.deg_info.setTextInteractionFlags(Qt.TextSelectableByMouse)
        dl.addWidget(self.deg_btn, 0, Qt.AlignTop)
        dl.addWidget(self.deg_info, 1)
        lay.addWidget(deg_box)
        rim_box = QtWidgets.QGroupBox('림 선명도 (대표 박스 crop · 파랑 = 모델폭 · 노랑 = 관측폭만 · 빨강 = 제외)')
        rl = QtWidgets.QHBoxLayout(rim_box)
        self.rim_img = QtWidgets.QLabel('—')
        self.rim_img.setAlignment(Qt.AlignCenter)
        self.rim_img.setFixedSize(200, 150)
        self.rim_img.setStyleSheet('background:#222;color:#bbb;')
        self.rim_info = QtWidgets.QLabel()
        self.rim_info.setTextFormat(Qt.RichText)
        self.rim_info.setWordWrap(True)
        self.rim_info.setTextInteractionFlags(Qt.TextSelectableByMouse)
        rl.addWidget(self.rim_img)
        rl.addWidget(self.rim_info, 1)
        lay.addWidget(rim_box)
        self.rim_warn = QtWidgets.QLabel()
        self.rim_warn.setWordWrap(True)
        self.rim_warn.setStyleSheet(f'color:{YELLOW};')
        lay.addWidget(self.rim_warn)
        cr_box = QtWidgets.QGroupBox('크레이터 경사·깊이 (대표 프레임 depth)')     # [261009]
        cl = QtWidgets.QVBoxLayout(cr_box)
        self.crater_info = QtWidgets.QLabel('—')
        self.crater_info.setTextFormat(Qt.RichText)
        self.crater_info.setWordWrap(True)
        self.crater_info.setTextInteractionFlags(Qt.TextSelectableByMouse)
        cl.addWidget(self.crater_info)
        try:                                                    # 3D mesh 를 이 창 안에 (VTK 가 없으면 글만)
            from gui.crater_mesh_view import CraterMeshView
            self.mesh_view = CraterMeshView(cr_box)
            self.mesh_view.setVisible(False)
            cl.addWidget(self.mesh_view)
        except Exception as e:  # noqa: BLE001
            self.mesh_view = None
            print(f'[ground_station] 3D mesh 보기 끔 (VTK 불러오기 실패): {e}', file=sys.stderr)
        lay.addWidget(cr_box)
        self.resize(460, 820)

    def show_pin(self, pid):
        self.pid = pid
        self._img_path = self._rim_path = _UNSET                # None(이미지 없음)도 '바뀜'으로 보게 — 앞 핀 그림이 남지 않게
        self.show()
        self.raise_()

    def refresh(self, pins, rover, frame):
        if self.pid is None or not self.isVisible():
            return
        p = pins.get(self.pid)
        if p is None:                                           # 다른 핀에 합쳐졌으면 그 핀으로
            host = next((q for q in pins.values() if self.pid in q.get('merged_from', [])), None)
            if host is None:
                self.info.setText(f'핀 #{self.pid} 없음 (pin_gallery 가 아직 못 받았거나 사라짐)')
                return
            self.pid = host['id']
            p = host
        self.setWindowTitle(f'크레이터 #{p["id"]}')
        path = p.get('image')
        if path != self._img_path:
            pm = QtGui.QPixmap(path) if path and os.path.exists(path) else QtGui.QPixmap()
            if pm.isNull():
                self.img.setText('이미지 없음 (영상 버퍼에서 못 찾음)' if path is None and p.get('top') else '이미지 없음')
            else:
                self.img.setPixmap(pm.scaled(self.img.width(), self.img.height(), Qt.KeepAspectRatio,
                                             Qt.SmoothTransformation))
            self._img_path = path
        rpath = p.get('rim_image')
        if rpath != self._rim_path:
            pm = QtGui.QPixmap(rpath) if rpath and os.path.exists(rpath) else QtGui.QPixmap()
            if pm.isNull():
                self.rim_img.setText('—')
            else:                                               # crop 이 작아도 점·선이 보이게 확대
                self.rim_img.setPixmap(pm.scaled(self.rim_img.width(), self.rim_img.height(), Qt.KeepAspectRatio,
                                                 Qt.SmoothTransformation))
            self._rim_path = rpath
        dg = p.get('degradation')                               # [261013]
        ordered = bool(dg and dg.get('status') in ('ranked', 'preview'))
        self.deg_btn.setText(f'순번 {dg["order"]} / {dg["total"]}' if ordered else '순번 —')
        self.deg_btn.setEnabled(ordered or bool(dg))            # 계산 안 된 핀도 전체 순서는 볼 수 있게
        self.deg_info.setText(deg_html(dg))
        self.deg_info.setMinimumHeight(self.deg_info.heightForWidth(max(self.deg_info.width(), 300)))
        rim = p.get('rim')
        self.rim_info.setText(rim_html(rim))
        self.rim_warn.setText('⚠ 대표 박스가 화면 가장자리에 걸림 — 림이 잘려 가시 범위가 낮게 나올 수 있음'
                              if rim and rim.get('top_on_edge') else '')
        self.rim_warn.setVisible(bool(rim and rim.get('top_on_edge')))
        cr = p.get('crater')
        self.crater_info.setText(crater_html(cr))
        # 줄바꿈 QLabel 은 높이를 낮게 잡아 마지막 줄이 잘린다 → 글 높이만큼 최소 높이
        self.crater_info.setMinimumHeight(self.crater_info.heightForWidth(max(self.crater_info.width(), 400)))
        mesh = cr.get('mesh_path') if cr and cr.get('geometry_status') == 'COMPLETE' else None
        mesh = mesh if mesh and os.path.exists(mesh) else None
        if self.mesh_view is not None:                          # 만들어진 mesh 를 불러와 그리기만 (계산은 pin_gallery 워커)
            self.mesh_view.setVisible(mesh is not None)
            if mesh is not None:
                self.mesh_view.load(mesh)
        x, y, z = p['xyz']
        dist = f'{math.hypot(x - rover[0], y - rover[1]):.2f} m' if rover else '— (로버 포즈 없음)'
        conf = badge('확정', RED) if p.get('confirmed') else badge('후보', YELLOW)
        top = p.get('top') or {}
        edge = f' (가장자리 {p["n_edge"]})' if p.get('n_edge') else ''
        self.info.setText(
            f'<table cellspacing="4">'
            f'<tr><td>종류</td><td><b>{p.get("cls", "")}</b> &nbsp; {conf}</td></tr>'
            f'<tr><td>지름</td><td><b>{2 * p["radius_m"] * 100:.1f} cm</b></td></tr>'
            f'<tr><td>좌표 ({frame})</td><td><b>x {x:+.3f} &nbsp; y {y:+.3f} &nbsp; z {z:+.3f}</b> m</td></tr>'
            f'<tr><td>검출 수</td><td><b>{p["n"]}</b>{edge}</td></tr>'
            f'<tr><td>로버로부터 거리</td><td><b>{dist}</b> (수평)</td></tr>'
            f'<tr><td>score</td><td>평균 {p.get("score_mean", 0):.2f} &nbsp; 최고 {top.get("score", 0):.2f}</td></tr>'
            f'</table>')


class DegradationOrderDialog(QtWidgets.QDialog):
    """[261013] 열화도 전체 순서 (비모달, 하나를 재사용). D 를 만든 크레이터만 — 계산 안 된 크레이터는 여기 없다.
    행 클릭 → 그 크레이터 정보 창."""
    COLS = ['순번', '핀', 'D', '경사 P95 [°]', '림 S', '경사 기여', '림 기여', '구간', '상태']

    def __init__(self, parent, on_pick):
        super().__init__(parent)
        self.setWindowTitle('열화도 전체 순서')
        self.on_pick = on_pick
        self.pid = None
        self._rows = []
        lay = QtWidgets.QVBoxLayout(self)
        self.head = QtWidgets.QLabel()
        self.head.setTextFormat(Qt.RichText)
        self.head.setWordWrap(True)
        lay.addWidget(self.head)
        self.table = QtWidgets.QTableWidget(0, len(self.COLS))
        self.table.setHorizontalHeaderLabels(self.COLS)
        self.table.verticalHeader().setVisible(False)
        fit_columns(self.table)
        self.table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.table.cellClicked.connect(self._clicked)
        lay.addWidget(self.table, 1)
        self.foot = QtWidgets.QLabel()
        self.foot.setTextFormat(Qt.RichText)
        self.foot.setWordWrap(True)
        self.foot.setStyleSheet(f'color:{GRAY};')
        lay.addWidget(self.foot)
        self.resize(620, 420)

    def show_for(self, pid):
        self.pid = pid
        self.show()
        self.raise_()

    def _clicked(self, row, _col):
        if 0 <= row < len(self._rows):
            self.pid = self._rows[row]
            self.on_pick(self.pid)

    def refresh(self, info):
        if not self.isVisible():
            return
        order = (info or {}).get('order') or []
        self.head.setText('<b>D 가 작을수록 열화 덜 됨 → 1번</b> &nbsp; D = 50·(1−u<sub>경사</sub>) + 50·(1−u<sub>림</sub>)<br>'
                          + (deg_ref_txt(info) or '아직 순서 없음 — 관측이 끝나 경사·림 두 값이 나온 크레이터가 2개 이상이어야 합니다.'))
        self._rows = [o['pin'] for o in order]
        if self.table.rowCount() != len(order):
            self.table.setRowCount(len(order))
        sel = None
        for r, o in enumerate(order):
            vals = (str(o['order']), f'#{o["pin"]}', f'{o["D"]:.1f}', f'{o["slope_deg"]:.1f}', f'{o["rim_score"]:.3f}',
                    f'{o["slope_contribution"]:.1f}', f'{o["rim_contribution"]:.1f}',
                    f'{o["blocks"]} · {o["span_seconds"]:.0f} s', '확정' if o['status'] == 'ranked' else '보류')
            for c, v in enumerate(vals):
                cell = self.table.item(r, c)
                if cell is None:
                    cell = QtWidgets.QTableWidgetItem(v)
                    self.table.setItem(r, c, cell)
                elif cell.text() != v:
                    cell.setText(v)
                if c == 8:
                    cell.setForeground(QtGui.QColor(GREEN if o['status'] == 'ranked' else YELLOW))
            if o['pin'] == self.pid:
                sel = r
        if sel is not None and self.table.currentRow() != sel:
            self.table.selectRow(sel)
        n_rank = sum(o['status'] == 'ranked' for o in order)
        pol = (info or {}).get('policy') or {}
        self.foot.setText(
            f'{len(order)}개 (확정 {n_rank} · 보류 {len(order) - n_rank}) · 계산 안 된 크레이터는 목록에 없음<br>'
            f'보류 = 값은 계산됐지만 확정 조건 전 — {pol.get("block_seconds", 5):.0f} s 구간 {pol.get("min_blocks", 5)}개·'
            f'{pol.get("min_span_seconds", 20):.0f} s 이상 반복 관측 + 핀 확정 + 촬영 조건 확인'
            + ('' if (info or {}).get('acquisition_verified') else ' (현재 촬영 조건 미확인 설정 → 전부 보류)')
            + '<br>상대 순서 — 연령·확률 아님. 가중치 1:1 은 운영 기준이며 최적값 검증이 아님.')


class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, node: GroundNode):
        super().__init__()
        self.node = node
        self.seq = WaypointSeq()
        self._lc_seen = None                                    # [261011] 마지막으로 반영한 보정 이력 (받은 시각)
        self.warn, self.block = read_slope_limits(node.trav_params)
        self.setWindowTitle('DARON 지상국')
        self._last_mk = 0.0
        self._last_sel_t = None
        self._pins = {}

        # [261008] 웨이포인트 기록 — 같은 폴더에 이어 쓰면 id 를 이어 매긴다 (불러올 때 id 가 겹치지 않게)
        self.log_path = os.path.join(node.session_dir, WP_LOG_NAME)
        last = max_logged_id(self.log_path)
        if last:
            self.seq.continue_ids(last)
        self.seq.on_arrived = self._log_arrived
        self.wplog = JsonlLog(self.log_path)
        self.wplog.write(self._ev('session_start', map_id=node.map_id, session_dir=node.session_dir,
                                  pose_topic=node.pose_topic, odom_topic=node.odom_topic, first_id=last + 1),
                         mkdir=False)                           # 폴더가 없으면 첫 웨이포인트 때 같이 씀 (JsonlLog 설명)
        self.loaded = None                                      # 불러온 기록 {path, items, warns, requeued}
        self._loaded_drawn = False

        root = QtWidgets.QWidget()
        grid = QtWidgets.QGridLayout(root)
        grid.addWidget(self._build_status(), 0, 0)
        grid.addWidget(self._build_waypoints(), 0, 1)
        grid.addWidget(self._build_pins(), 0, 2)
        self.rover = RoverPanel(node.rover, node.follower, auto=node.auto, on_auto=self._on_auto_mode)   # [261010] 수동, [261012] 웨이포인트, [261014] 자율
        grid.addWidget(self.rover, 1, 0, 1, 3)
        self.auto_panel = AutoPanel(node.auto)                  # [261014] 경기장 놓기·틈·자율 상태
        grid.addWidget(self.auto_panel, 2, 0, 1, 3)
        self._paused_before_auto = None
        grid.setColumnStretch(0, 4)
        grid.setColumnStretch(1, 2)
        grid.setColumnStretch(2, 5)                             # 4:2:5 — 핀 표 '림' 열 자리 (261007, 웨이포인트 표는 여유 있음)
        self.setCentralWidget(root)
        self.dialog = CraterDialog(self, on_order=self._show_deg_order)
        self.deg_dialog = DegradationOrderDialog(self, on_pick=self._pick_from_order)    # [261013]
        self.resize(1320, 860)                                  # [261014] 자율 칸만큼 (760 → 860)
        m = re.fullmatch(r'(\d+)x(\d+)\+(-?\d+)\+(-?\d+)', node.window_geometry or '')
        if m:                                                   # [261008] 녹화 때 창 겹침 방지
            w_, h_, x_, y_ = (int(v) for v in m.groups())
            self.setGeometry(x_, y_, w_, h_)

        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.tick)
        self.timer.start(100)
        self._fill_log_state()
        if node.load_file:                                      # 기동 시 표시만 (목록에 자동으로 넣지 않음)
            self._load(node.load_file, interactive=False)

    # ---------------- 화면 구성 ----------------
    def _build_status(self):
        box = QtWidgets.QGroupBox('상태')
        lay = QtWidgets.QVBoxLayout(box)
        self.lbl = {}
        form = QtWidgets.QFormLayout()
        for k, name in (('slam', 'SLAM'), ('reloc', '맵 재추정'), ('health', 'health'), ('roll', '전복'), ('planner', '경로계획')):
            self.lbl[k] = QtWidgets.QLabel('—')
            self.lbl[k].setTextFormat(Qt.RichText)
            self.lbl[k].setWordWrap(True)
            form.addRow(name, self.lbl[k])
        lay.addLayout(form)
        att = QtWidgets.QGroupBox(f'자세 (경고 {self.warn:.0f}° / 차단 {self.block:.0f}°)')
        al = QtWidgets.QVBoxLayout(att)
        self.bar_roll = AttitudeBar('롤', self.warn, self.block)
        self.bar_pitch = AttitudeBar('피치', self.warn, self.block)
        self.lbl_att = QtWidgets.QLabel()
        self.lbl_att.setTextFormat(Qt.RichText)
        al.addWidget(self.bar_roll)
        al.addWidget(self.bar_pitch)
        al.addWidget(self.lbl_att)
        lay.addWidget(att)
        self.hz_table = QtWidgets.QTableWidget(len(self.node.hz), 2)
        self.hz_table.setHorizontalHeaderLabels(['토픽', 'Hz'])
        self.hz_table.verticalHeader().setVisible(False)
        self.hz_table.horizontalHeader().setSectionResizeMode(0, QtWidgets.QHeaderView.Stretch)
        self.hz_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.hz_table.setSelectionMode(QtWidgets.QAbstractItemView.NoSelection)
        for r, t in enumerate(self.node.hz):
            self.hz_table.setItem(r, 0, QtWidgets.QTableWidgetItem(t))
            self.hz_table.setItem(r, 1, QtWidgets.QTableWidgetItem('—'))
        lay.addWidget(QtWidgets.QLabel('수신 주기 (최근 2 s)'))
        lay.addWidget(self.hz_table)
        return box

    def _build_waypoints(self):
        box = QtWidgets.QGroupBox('웨이포인트 — 탑뷰/3D RViz 에서 2D Goal Pose (G, 드래그 = 도착 방향)')
        lay = QtWidgets.QVBoxLayout(box)
        self.wp_table = QtWidgets.QTableWidget(0, 6)
        self.wp_table.setHorizontalHeaderLabels(['#', 'x [m]', 'y [m]', '방위 [°]', '거리 [m]', '상태'])
        self.wp_table.verticalHeader().setVisible(False)
        fit_columns(self.wp_table)
        self.wp_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.wp_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.wp_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        lay.addWidget(self.wp_table)
        row = QtWidgets.QHBoxLayout()
        b_up, b_dn = QtWidgets.QPushButton('▲ 위로'), QtWidgets.QPushButton('▼ 아래로')
        b_del, b_clr = QtWidgets.QPushButton('삭제'), QtWidgets.QPushButton('전체 삭제')
        self.b_pause = QtWidgets.QPushButton('일시정지')
        self.b_pause.setCheckable(True)
        b_up.clicked.connect(lambda: self._move(-1))
        b_dn.clicked.connect(lambda: self._move(+1))
        b_del.clicked.connect(self._delete)
        b_clr.clicked.connect(self._clear_all)
        self.b_pause.toggled.connect(self._pause)
        for b in (b_up, b_dn, b_del, b_clr, self.b_pause):
            row.addWidget(b)
        lay.addLayout(row)
        self.img_win = None
        if self.node.image_click_on:                            # [261011] 영상 클릭 창
            b_img = QtWidgets.QPushButton('영상에서 찍기 (D405)')
            b_img.setToolTip('infra1 영상 창을 띄움 — 누르고 드래그 = 위치 + 도착 방향. 바닥 0.07~1.8 m 안만')
            b_img.clicked.connect(self._open_image_click)
            lay.addWidget(b_img)
        self.lbl_wp = QtWidgets.QLabel()
        self.lbl_wp.setTextFormat(Qt.RichText)
        self.lbl_wp.setWordWrap(True)
        lay.addWidget(self.lbl_wp)
        self.lbl_done = QtWidgets.QLabel()
        self.lbl_done.setStyleSheet(f'color:{GRAY};')
        lay.addWidget(self.lbl_done)

        # [261008] 불러오기 — 지난 세션의 waypoints.jsonl 을 지도에 표시하고 골라서 다시 넣는다
        row2 = QtWidgets.QHBoxLayout()
        b_load = QtWidgets.QPushButton('웨이포인트 불러오기')
        b_load.setToolTip(f'waypoints.jsonl 선택 (좌표계 {self.node.waypoint_frame} 만). 지도에 표시만 하고 목록엔 안 넣음')
        self.b_requeue_all = QtWidgets.QPushButton('전부 다시 넣기')
        self.b_requeue_sel = QtWidgets.QPushButton('선택 다시 넣기')
        self.b_unload = QtWidgets.QPushButton('불러온 것 닫기')
        b_own = QtWidgets.QPushButton('이번 세션 도착 목록')                     # [261008 밤]
        b_own.setToolTip('지금 세션에서 도착한 웨이포인트를 표·지도에 표시 → 선택/전부 다시 넣기 (좌표 그대로). 켜 둔 동안 도착하면 자동 갱신')
        b_own.clicked.connect(self._load_own)
        b_load.clicked.connect(self._load_dialog)
        self.b_requeue_all.clicked.connect(self._requeue_all)
        self.b_requeue_sel.clicked.connect(self._requeue_sel)
        self.b_unload.clicked.connect(self._unload)
        for b in (b_own, b_load, self.b_requeue_all, self.b_requeue_sel, self.b_unload):
            row2.addWidget(b)
        lay.addLayout(row2)
        self.loaded_table = QtWidgets.QTableWidget(0, 6)
        self.loaded_table.setHorizontalHeaderLabels(['L#', 'x [m]', 'y [m]', '방위 [°]', '거리 [m]', '상태'])
        self.loaded_table.verticalHeader().setVisible(False)
        fit_columns(self.loaded_table)
        self.loaded_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.loaded_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.loaded_table.setSelectionMode(QtWidgets.QAbstractItemView.ExtendedSelection)
        lay.addWidget(self.loaded_table)
        self.lbl_loaded = QtWidgets.QLabel()
        self.lbl_loaded.setTextFormat(Qt.RichText)
        self.lbl_loaded.setWordWrap(True)
        lay.addWidget(self.lbl_loaded)
        self.lbl_log = QtWidgets.QLabel()
        self.lbl_log.setTextFormat(Qt.RichText)
        self.lbl_log.setWordWrap(True)
        self.lbl_log.setTextInteractionFlags(Qt.TextSelectableByMouse)
        lay.addWidget(self.lbl_log)
        self._show_loaded()
        return box

    def _build_pins(self):
        box = QtWidgets.QGroupBox('크레이터 핀 — 행 또는 RViz 핀 클릭 → 정보 창')
        lay = QtWidgets.QVBoxLayout(box)
        self.pin_table = QtWidgets.QTableWidget(0, 7)
        self.pin_table.setHorizontalHeaderLabels(['#', '종류', '지름 [cm]', '확정', '검출', '림', '거리 [m]'])
        self.pin_table.verticalHeader().setVisible(False)
        fit_columns(self.pin_table)
        self.pin_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.pin_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.pin_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.pin_table.cellClicked.connect(self._pin_clicked)
        lay.addWidget(self.pin_table)
        self.lbl_pins = QtWidgets.QLabel()
        self.lbl_pins.setStyleSheet(f'color:{GRAY};')
        self.lbl_pins.setWordWrap(True)
        lay.addWidget(self.lbl_pins)
        return box

    # ---------------- 버튼 ----------------
    def _sel_row(self):
        rows = self.wp_table.selectionModel().selectedRows()
        return rows[0].row() if rows else None

    def _move(self, d):
        r = self._sel_row()
        if r is None:
            return
        before = [it['id'] for it in self.seq.items]
        self.seq.move(r, d)
        order = [it['id'] for it in self.seq.items]
        if order != before:
            self._log('reordered', order=order)
        self._fill_wps()
        j = min(max(r + d, 0), len(self.seq.items) - 1)
        self.wp_table.selectRow(j)

    def _delete(self):
        r = self._sel_row()
        if r is not None and r < len(self.seq.items):
            it = self.seq.items[r]
            self.seq.remove(r)
            self._log('removed', id=it['id'], goal=wp_goal(it))
            self._fill_wps()

    def _clear_all(self):
        if self.seq.items and QtWidgets.QMessageBox.question(
                self, '전체 삭제', f'웨이포인트 {len(self.seq.items)}개를 모두 지울까요?') != QtWidgets.QMessageBox.Yes:
            return
        gone = list(self.seq.items)
        self.seq.clear_all()
        if gone:
            self._log('cleared', ids=[it['id'] for it in gone], goals=[wp_goal(it) for it in gone])
        self._fill_wps()

    # ---------------- [261008] 기록 ----------------
    def _ev(self, event, **kw):
        n = self.node
        d = {'event': event}
        d.update(kw)
        d.update(frame=n.waypoint_frame, session=n.session_name, t_ros=n.now_ros(), t_wall=time.time())
        return d

    def _log(self, event, **kw):
        try:
            self.wplog.write(self._ev(event, **kw))
        except Exception as e:  # noqa: BLE001 — 기록 실패로 웨이포인트 동작이 멈추면 안 됨
            self.wplog.error = str(e)
        if self.wplog.error:
            self.node.get_logger().error(f'웨이포인트 기록 실패 {self.log_path}: {self.wplog.error}')
        self._fill_log_state()

    def _add_wp(self, x, y, z, yaw=None, **extra):
        """목록 추가는 전부 여기로 (지도 클릭·영상 클릭·다시 넣기) — added 기록. yaw [rad] 또는 None.
        [261011] 이번 세션 지도 위 항목은 루프 클로저 기준(시각·map<-odom)을 붙인다. 이전 세션(loaded:)은 바탕 지도라 안 옮김."""
        it = self.seq.add(x, y, z, yaw)
        src = str(extra.get('source', ''))
        if not src.startswith('loaded:'):
            a = self.node.lc_anchor(x, y, src, (extra.get('image') or {}).get('stamp'))
            if a is not None:
                it['lc'] = [a[0], a[1]]
                extra['anchor'] = {'t_ns': a[0], 'src': a[2]}
        self._log('added', id=it['id'], goal=wp_goal(it), **extra)
        self._last_mk = 0.0                                     # 다음 틱에 마커 바로
        return it

    def _apply_lc(self):
        """[261011] 새 루프 클로저 보정 이력 → 이번 세션 웨이포인트(목록·도착)를 기준 시각의 보정량만큼 옮김 + 'moved' 기록.
        지금 플래너에 준 목표가 옮겨지면 시퀀서가 CLEAR 후 새 좌표로 다시 보낸다. 피쳐맵·핀과 같은 이력·같은 식."""
        h = self.node.lc_hist
        if h.v is None or h.t == self._lc_seen:
            return
        self._lc_seen = h.t
        moves = []
        for it in list(self.seq.items) + list(self.seq.done):
            lc = it.get('lc')
            if not lc:
                continue
            r = KR.correct_point(h.v, lc[0], lc[1], (it['x'], it['y'], it['z']), self.node.lc_tol_ns)
            if r is None:
                continue
            p, M, D = r
            lc[1] = M
            dyaw = math.atan2(D[1, 0], D[0, 0])
            dpos = math.dist(p, (it['x'], it['y'], it['z']))
            if dpos < 1e-4 and abs(dyaw) < 1e-5:
                continue
            frm = wp_goal(it)
            yaw = None if it.get('yaw') is None else it['yaw'] + dyaw
            if any(x is it for x in self.seq.items):
                self.seq.relocate(it['id'], p[0], p[1], p[2], yaw)
            else:
                it.update(x=float(p[0]), y=float(p[1]), z=float(p[2]), yaw=None if yaw is None else wrap_pi(yaw))
            moves.append({'id': it['id'], 'from': frm, 'to': wp_goal(it), 'dpos_m': round(dpos, 4),
                          'dyaw_deg': round(math.degrees(dyaw), 3)})
        if moves:
            dmax = max(m['dpos_m'] for m in moves)
            self._log('moved', reason='lc_resync', moves=moves)
            if not self.seq.msg.startswith('웨이포인트 #'):
                self.seq.msg = f'루프 클로저 보정: 웨이포인트 {len(moves)}개 옮김 (최대 {dmax * 100:.1f} cm)'
            self.node.get_logger().info(f'[LC 재동기화] 웨이포인트 {len(moves)}개 옮김, 최대 {dmax * 100:.2f} cm')
            self._last_mk = 0.0

    # ---------------- [261011] 영상 클릭 ----------------
    def _open_image_click(self):
        if self.img_win is None:
            self.img_win = ImageClickWindow(self, self.node, self._image_result, self.node.click_min_drag)
        self.img_win.show()
        self.img_win.raise_()
        self.img_win.activateWindow()

    def _image_result(self, r):
        """ImageClickWindow 의 성공 결과 → 목록 추가 (z = 찍은 바닥점 높이)."""
        x, y, z = r['point']
        info = {'pixel': [round(r['press_uv'][0], 1), round(r['press_uv'][1], 1)],
                'pixel_used': list(r['uv_used']) if r.get('uv_used') else None,
                'depth_m': r.get('depth_m'), 'stamp': r.get('stamp'), 'yaw_source': r.get('yaw_source'),
                'dz_m': r.get('dz')}
        it = self._add_wp(x, y, z, r.get('yaw'), source='image', image=info)
        self.node.get_logger().info(f'[영상 클릭] #{it["id"]} ({x:+.3f}, {y:+.3f}, {z:+.3f}) 방위 {yaw_txt(it)}° '
                                    f'{json.dumps(info, ensure_ascii=False)}')
        self._fill_wps()
        return it

    def _log_arrived(self, it):
        """WaypointSeq.on_arrived — 도착 판정은 시퀀서의 ARRIVED 분기 (여기서 따로 판정하지 않는다)."""
        n = self.node
        lp, lp_age = n.loc_pose.v, n.loc_pose.age()
        op = n.pose.v
        h = n.health.v
        err = None                                              # [261011] 기록만 — 도착 판정은 시퀀서(플래너 ARRIVED)
        if lp is not None and lp[4] == n.waypoint_frame:
            err = arrival_error(it, lp)
        it['err'] = err
        self._log('arrived', id=it['id'], goal=wp_goal(it), arrival_error=err,
                  arrived_pose=None if lp is None else
                  {'x': lp[0], 'y': lp[1], 'z': lp[2], 'yaw_deg': math.degrees(lp[3])},
                  pose_frame=None if lp is None else lp[4],
                  pose_age_s=None if lp_age is None else round(lp_age, 3),
                  odom_pose=None if op is None else
                  {'x': op[0], 'y': op[1], 'z': op[2], 'yaw_deg': math.degrees(op[3])},
                  health=h, health_age_s=None if n.health.age() is None else round(n.health.age(), 3))
        if self.loaded and self.loaded.get('own'):           # [261008 밤] 이번 세션 도착 목록을 켜 둔 동안 자동 갱신
            self._load_own(quiet=True)

    def _fill_log_state(self):
        w = self.wplog
        if w.error:
            txt = badge('기록 실패', RED) + f' {w.error}'
        elif w.opened:
            txt = f'기록 {self.log_path}'
        else:
            txt = f'기록 대기 — 폴더 없음, 첫 웨이포인트 때 만듦: {self.log_path}'
        self.lbl_log.setText(f'<small style="color:{GRAY}">{txt}</small>' if not w.error else f'<small>{txt}</small>')

    # ---------------- [261008] 불러오기 ----------------
    def _load_dialog(self):
        start = os.path.dirname(self.node.session_dir)          # 세션 폴더들의 부모 (bag_dir 등)
        if not os.path.isdir(start):
            start = os.path.expanduser('~')
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, '웨이포인트 불러오기', start,
                                                        f'웨이포인트 기록 ({WP_LOG_NAME} *.jsonl);;모든 파일 (*)')
        if path:
            self._load(path, interactive=True)

    def _load(self, path, interactive):
        """interactive=False (load_waypoints_file): 창 없이 로그·표 아래 글로만 알림."""
        lg = self.node.get_logger()
        try:
            rec = reconstruct(path)
        except (OSError, UnicodeDecodeError) as e:
            err = f'못 읽음: {e}'
        else:
            err, warns = check_loaded(rec, self.node.waypoint_frame, self.node.map_id, self.node.session_name)
        if err:
            lg.error(f'웨이포인트 불러오기 거부 {path}: {err}')
            if interactive:
                QtWidgets.QMessageBox.critical(self, '웨이포인트 불러오기 거부', f'{path}\n\n{err}')
            else:
                self.lbl_loaded.setText(badge('불러오기 거부', RED) + f' {os.path.basename(path)}: {err}')
            return False
        for w in warns:
            lg.warn(f'웨이포인트 불러오기 {path}: {w}')
        if warns and interactive and QtWidgets.QMessageBox.warning(
                self, '다른 세션 좌표', '\n\n'.join(warns) + '\n\n그래도 불러올까요? (지도에 표시만, 목록엔 안 넣음)',
                QtWidgets.QMessageBox.Ok | QtWidgets.QMessageBox.Cancel) != QtWidgets.QMessageBox.Ok:
            return False
        items = [w for w in rec['waypoints'] if w['status'] != 'removed']
        self.loaded = dict(path=os.path.abspath(path), items=items, warns=warns, requeued={})
        lg.info(f'웨이포인트 불러옴 {path}: {len(items)}개 (도착 {sum(w["status"] == "arrived" for w in items)})')
        self._show_loaded()
        self._last_mk = 0.0                                     # 다음 틱에 바로 그림
        return True

    def _load_own(self, quiet=False):
        """[261008 밤] 진행 중인 세션의 기록에서 도착한 웨이포인트만 불러와 표·지도에 (대화상자·좌표계 검사 없음 — 같은 세션)."""
        if not os.path.isfile(self.log_path):
            if not quiet:
                self.lbl_loaded.setText(badge('없음', GRAY) + ' 이번 세션에 아직 도착 기록이 없음')
            return False
        try:
            rec = reconstruct(self.log_path)
        except (OSError, UnicodeDecodeError) as e:
            self.lbl_loaded.setText(badge('못 읽음', RED) + f' {e}')
            return False
        items = arrived_unique(rec)
        prev = self.loaded if (self.loaded and self.loaded.get('own')) else None
        self.loaded = dict(path=os.path.abspath(self.log_path), items=items, warns=[], own=True,
                           requeued=prev['requeued'] if prev else {})
        if not quiet:
            self.node.get_logger().info(f'이번 세션 도착 목록: {len(items)}개')
        self._show_loaded()
        self._last_mk = 0.0
        return True

    def _unload(self):
        self.loaded = None
        self._show_loaded()
        self._last_mk = 0.0

    def _requeue(self, items):
        L = self.loaded
        for w in items:                                         # 보통 추가 경로 (added 기록). 좌표는 파일 값 그대로
            self._add_wp(w['x'], w['y'], w['z'], w.get('yaw'), source=('session' if L.get('own') else f'loaded:{L["path"]}'),
                         loaded_id=w['id'], loaded_session=w['session'])
            L['requeued'][w['key']] = L['requeued'].get(w['key'], 0) + 1
        self._fill_wps()
        self._show_loaded()

    def _requeue_all(self):
        if not self.loaded:
            return
        items = self.loaded['items']
        n_arr = sum(w['status'] == 'arrived' for w in items)
        if QtWidgets.QMessageBox.question(
                self, '전부 다시 넣기', f'{"이번 세션에서 도착한" if self.loaded.get("own") else "불러온"} 웨이포인트 '
                f'{len(items)}개 (도착했던 것 {n_arr}개 포함)를 '
                f'파일 순서대로 목록 끝에 넣을까요?') != QtWidgets.QMessageBox.Yes:
            return
        self._requeue(items)

    def _requeue_sel(self):
        if not self.loaded:
            return
        rows = sorted(i.row() for i in self.loaded_table.selectionModel().selectedRows())
        self._requeue([self.loaded['items'][r] for r in rows if r < len(self.loaded['items'])])

    def _show_loaded(self, rover=None):
        L = self.loaded
        on = L is not None
        for w in (self.loaded_table, self.b_requeue_all, self.b_requeue_sel, self.b_unload):
            w.setVisible(on)
        if not on:
            self.loaded_table.setRowCount(0)
            if not self.lbl_loaded.text().startswith('<span'):  # 기동 시 거부 알림은 남김
                self.lbl_loaded.setText('')
            return
        items = L['items']
        if self.loaded_table.rowCount() != len(items):
            self.loaded_table.setRowCount(len(items))
        for r, w in enumerate(items):
            d = f'{math.hypot(w["x"] - rover[0], w["y"] - rover[1]):.2f}' if rover else '—'
            st = '도착' if w['status'] == 'arrived' else '미도착'
            if w.get('n_arrived', 1) > 1:
                st += f' ×{w["n_arrived"]}'
            k = L['requeued'].get(w['key'])
            if k:
                st += f' · 다시 넣음{"" if k == 1 else f" ×{k}"}'
            for c, v in enumerate((f'L{w["id"]}', f'{w["x"]:+.2f}', f'{w["y"]:+.2f}', yaw_txt(w), d, st)):
                cell = self.loaded_table.item(r, c)
                if cell is None:
                    cell = QtWidgets.QTableWidgetItem(v)
                    self.loaded_table.setItem(r, c, cell)
                elif cell.text() != v:
                    cell.setText(v)
                if c == 5:
                    cell.setForeground(QtGui.QBrush(QtGui.QColor(BLUE if w['status'] == 'arrived' else '#d07000')))
        n_arr = sum(w['status'] == 'arrived' for w in items)
        if L.get('own'):
            self.lbl_loaded.setText(f'이번 세션 도착 {len(items)}곳 (파랑, 지도 라벨 L#) — 골라서 다시 넣기. 도착하면 자동 갱신')
            return
        txt = (f'불러옴 {os.path.basename(os.path.dirname(L["path"]))}/{os.path.basename(L["path"])}: '
               f'{len(items)}개 (도착 {n_arr} 파랑 · 미도착 {len(items) - n_arr} 주황, 지도 라벨 L#)')
        self.lbl_loaded.setText(txt + ''.join('<br>' + badge('주의', YELLOW) + ' ' + w for w in L['warns']))

    def _pause(self, on):
        self.seq.set_paused(on)
        self.b_pause.setText('재개' if on else '일시정지')

    def _on_auto_mode(self, on):
        """[261014] 자율 켬 → 지상국 웨이포인트 목록 일시정지 (자율 중 목표를 밀어 넣지 않게), 끔 → 자율 전 상태로."""
        if on:
            if self._paused_before_auto is None:
                self._paused_before_auto = self.seq.paused
            self.b_pause.setChecked(True)                       # → _pause(True)
            self.seq.msg = '자율 중 — 웨이포인트 목록 일시정지'
        elif self._paused_before_auto is not None:
            if not self._paused_before_auto:
                self.b_pause.setChecked(False)
            self._paused_before_auto = None

    def _pin_clicked(self, row, _col):
        it = self.pin_table.item(row, 0)
        if it is None:
            return
        pid = int(it.text())
        self.node.pub_select.publish(Int32(data=pid))           # pin_gallery 가 RViz 마커를 강조
        self.dialog.show_pin(pid)

    def _show_deg_order(self, pid):
        """[261013] 정보 창 열화도 순번 클릭 → 전체 순서"""
        self.deg_dialog.show_for(pid)
        self.deg_dialog.refresh((self.node.pins.v or {}).get('degradation'))

    def _pick_from_order(self, pid):
        """[261013] 전체 순서 행 클릭 → 그 크레이터 정보 창 + RViz 강조"""
        self.node.pub_select.publish(Int32(data=pid))
        self.dialog.show_pin(pid)

    # ---------------- 주기 ----------------
    def tick(self):
        n = self.node
        now = time.monotonic()
        rover = n.rover_wp()                                    # [261011] 웨이포인트·핀과 같은 좌표(map). TF 없으면 옛 동작
        if rover is None:
            rover = n.pose.v if (n.pose.age() is not None and n.pose.age() < 2.0) else None
        self._apply_lc()

        # 클릭 → 목록 ([261008] 좌표계가 waypoint_frame 이 아니면 거부 — TF 변환 안 함)
        while n.clicks:
            msg = n.clicks.pop(0)
            r = n.click_xyz(msg)
            if isinstance(r, str):
                self.seq.msg = r
                n.get_logger().warn(r)
                continue
            x, y, z, yaw = r
            if self.auto_panel.take_gap_click(x, y, z):         # [261014] [틈 찍기] 중이면 이 클릭은 틈 (웨이포인트로 안 넣음)
                continue
            if n.auto.auto_on():                                # [261015] 자율 중 2D Goal Pose 는 버림 (실수 클릭이 나중에 웨이포인트로 남지 않게)
                self.seq.msg = '자율 중 — 2D Goal Pose 무시 (틈을 옮기려면 [틈 찍기] 먼저, 웨이포인트는 자율을 끈 뒤)'
                n.get_logger().warn(self.seq.msg)
                continue
            self._add_wp(x, y, rover[2] if rover else z, yaw, source='rviz')   # 2D Goal Pose 의 z 는 항상 0 → 표시용으로 로버 높이

        # 플래너와 동기화
        pl = n.planner.v if n.planner.v is not None else {}
        wps = n.planner_wps.v
        if wps is None and n.planner.v is not None and pl.get('waypoints') == 0:
            wps = []                    # 플래너는 목록이 바뀔 때만 /planner/waypoints 를 낸다 — 기동 후 한 번도 안 바뀌었으면 상태의 개수로
        for act in self.seq.tick(now, wps, pl.get('state', ''), self._click_reason(pl)):
            n.send(act)
        if self.seq.paused != self.b_pause.isChecked():         # 시퀀서가 스스로 멈춘 경우 버튼도 맞춤
            self.b_pause.blockSignals(True)
            self.b_pause.setChecked(self.seq.paused)
            self.b_pause.setText('재개' if self.seq.paused else '일시정지')
            self.b_pause.blockSignals(False)

        # RViz 선택 → 정보 창
        if n.selected.t is not None and n.selected.t != self._last_sel_t:
            self._last_sel_t = n.selected.t
            self.dialog.show_pin(n.selected.v)

        if now - self._last_mk > 0.5:
            self._last_mk = now
            loaded = self.loaded['items'] if self.loaded else None
            n.publish_wp_markers(self.seq.items, self.seq.sent['id'] if self.seq.sent else None, rover,
                                 loaded, clear_loaded=self._loaded_drawn)
            self._loaded_drawn = self._loaded_drawn or bool(loaded)
            self._fill_status(rover)
            self._log_map_id()
            if self.loaded:
                self._show_loaded(rover)                        # 거리 열
        self._fill_wps(rover, pl, wps)
        self._fill_pins(rover)

    @staticmethod
    def _click_reason(pl):
        c = pl.get('click') or {}
        return c.get('reason', '') if not c.get('ok', True) else ''

    def _age_txt(self, lt, stale=2.0):
        a = lt.age()
        if a is None:
            return badge('없음', GRAY), True
        if a > stale:
            return badge(f'끊김 {a:.0f}s', GRAY), True
        return '', False

    RELOC_TXT = {'NEW_MAP': ('새 맵', GRAY), 'PENDING': ('재추정 중', YELLOW), 'CONFIRMING': ('확인 중', YELLOW),
                 'ACCEPTED': ('이전 맵에 맞춤', GREEN), 'REJECTED': ('거부 → 새 맵', RED), 'FAILED': ('실패 → 새 맵', RED)}

    def _log_map_id(self):
        """[261008 밤] 재추정으로 map_id 가 정해지면 기록에 한 줄 (불러올 때 map_id 비교용). 상태가 확정된 뒤 한 번."""
        d = self.node.reloc.v or {}
        if getattr(self, '_map_id_logged', None) == (d.get('state'), d.get('map_id')):
            return
        if d.get('state') in ('NEW_MAP', 'ACCEPTED', 'REJECTED', 'FAILED') and d.get('map_id'):
            self._map_id_logged = (d.get('state'), d.get('map_id'))
            self.wplog.write(self._ev('map_id', map_id=d['map_id'], reloc=d['state']), mkdir=False)

    def _fill_reloc(self):
        """[261008 밤] /slam/reloc_status — 이전 맵 좌표로 이어 쓰는지 운용자가 출발 전에 확인."""
        d = self.node.reloc.v
        if d is None:
            self.lbl['reloc'].setText(badge('없음 (Slam 꺼짐?)', GRAY))
            return
        txt, col = self.RELOC_TXT.get(d.get('state', ''), (d.get('state', '?'), YELLOW))
        extra = f' map_id {d.get("map_id", "")}' if d.get('map_id') else ''
        if d.get('confirm_delta_m') is not None:
            extra += f' (두 추정 차 {d["confirm_delta_m"] * 100:.1f} cm)'
        if d.get('state') in ('PENDING', 'CONFIRMING'):
            extra += ' — 출발점에서 정지한 채 기다릴 것'
        if d.get('lc_count'):
            extra += f' | 루프 클로저 {d["lc_count"]}'
        self.lbl['reloc'].setText(badge(txt, col) + extra)

    def _fill_status(self, rover):
        n = self.node
        self._fill_reloc()
        tag, dead = self._age_txt(n.slam)
        if dead:
            self.lbl['slam'].setText(tag)
        else:
            s = n.slam.v
            jump = s.get('relock_jump_m')
            self.lbl['slam'].setText((badge('추적', GREEN) if s.get('tracked') else badge('잃음', RED))
                                     + f' 특징점 {s.get("features_l", "?")}'
                                     + (f' · 재잠금 점프 {jump:.2f} m' if jump else ''))
        tag, dead = self._age_txt(n.health)
        if dead:
            self.lbl['health'].setText(tag)
        else:
            h = n.health.v
            why = '; '.join((h.get('ins_reasons') or []) + (h.get('slam_reasons') or []))
            self.lbl['health'].setText(
                'INS ' + badge(h.get('ins', '?'), LEVEL_COLOR.get(h.get('ins'), GRAY)) + ' &nbsp;SLAM '
                + badge(h.get('slam', '?'), LEVEL_COLOR.get(h.get('slam'), GRAY))
                + (f'<br><small>{why}</small>' if why else ''))
        tag, dead = self._age_txt(n.recalib, stale=3.0)
        if dead:
            self.lbl['roll'].setText(tag)
        else:
            st = dict(kv.split('=', 1) for kv in n.recalib.v.split() if '=' in kv).get('state', '?')
            col = {'upright': GREEN, 'inverted': RED}.get(st, GRAY)
            name = {'upright': '정상', 'inverted': '전복', 'unknown': '모름'}.get(st, st)
            ev = ''
            if n.recalib_event.v and n.recalib_event.age() < 60:
                ev = f'<br><small>{n.recalib_event.age():.0f}s 전: {n.recalib_event.v}</small>'
            self.lbl['roll'].setText(badge(name, col) + ev)
        tag, dead = self._age_txt(n.planner)
        if dead:
            self.lbl['planner'].setText(tag)
        else:
            pl = n.planner.v
            st = pl.get('state', '?')
            self.lbl['planner'].setText(badge(st, PLANNER_COLOR.get(st, YELLOW)) + f' 모드 {pl.get("mode", "?")}'
                                        + f'<br><small>{pl.get("reason", "")}</small>')
        # 자세
        a = n.att.v if (n.att.age() is not None and n.att.age() < 1.0) else None
        self.bar_roll.set_value(a[0] if a else None)
        self.bar_pitch.set_value(a[1] if a else None)
        if a is None:
            self.lbl_att.setText(badge('자세 데이터 없음', GRAY))
        else:
            m = max(abs(a[0]), abs(a[1]))
            self.lbl_att.setText(badge('차단 각 넘음', RED) if m >= self.block else
                                 badge('경고', YELLOW) if m >= self.warn else badge('정상', GREEN))
        for r, t in enumerate(n.hz):
            hz = n.hz_of(t)
            it = self.hz_table.item(r, 1)
            it.setText(f'{hz:.1f}')
            it.setForeground(QtGui.QBrush(QtGui.QColor(RED if hz == 0 else '#222')))

    def _fill_wps(self, rover=None, pl=None, wps=None):
        items = self.seq.items
        sel = self._sel_row()
        if self.wp_table.rowCount() != len(items):
            self.wp_table.setRowCount(len(items))
        for r, it in enumerate(items):
            d = f'{math.hypot(it["x"] - rover[0], it["y"] - rover[1]):.2f}' if rover else '—'
            st = self.seq.item_state(it) if r == 0 else '대기'
            if r == 0 and self.seq.paused:
                st = '일시정지'
            for c, v in enumerate((str(r + 1), f'{it["x"]:+.2f}', f'{it["y"]:+.2f}', yaw_txt(it), d, st)):
                cell = self.wp_table.item(r, c)
                if cell is None:
                    self.wp_table.setItem(r, c, QtWidgets.QTableWidgetItem(v))
                elif cell.text() != v:
                    cell.setText(v)
        if sel is not None and sel < len(items):
            self.wp_table.selectRow(sel)
        if pl is None:
            return
        lines = []
        if self.seq.msg:
            lines.append(badge('알림', YELLOW) + ' ' + self.seq.msg)
        if wps is None:
            lines.append(badge('플래너 없음', GRAY) + ' /planner/waypoints 를 못 받음 — 경로계획 노드가 떠 있는지 확인')
        elif pl.get('mode') and pl.get('mode') != 'WAYPOINT':
            lines.append(badge('주의', YELLOW) + f' 플래너 모드가 {pl.get("mode")} — WAYPOINT 가 아니면 목표를 줘도 안 감')
        self.lbl_wp.setText('<br>'.join(lines))
        if self.seq.done:
            self.lbl_done.setText('도착: ' + ', '.join(done_txt(d) for d in self.seq.done[-5:]))

    def _fill_pins(self, rover):
        n = self.node
        info = n.pins.v or {}
        pins = {int(p['id']): p for p in info.get('pins', [])}
        self._pins = pins
        frame = info.get('frame', 'odom')
        rows = sorted(pins)
        if self.pin_table.rowCount() != len(rows):
            self.pin_table.setRowCount(len(rows))
        for r, pid in enumerate(rows):
            p = pins[pid]
            x, y, _ = p['xyz']
            d = f'{math.hypot(x - rover[0], y - rover[1]):.2f}' if rover else '—'
            vals = (str(pid), p.get('cls', ''), f'{2 * p["radius_m"] * 100:.1f}', '확정' if p.get('confirmed') else '후보',
                    str(p['n']), rim_cell(p.get('rim')), d)
            for c, v in enumerate(vals):
                cell = self.pin_table.item(r, c)
                if cell is None:
                    self.pin_table.setItem(r, c, QtWidgets.QTableWidgetItem(v))
                elif cell.text() != v:
                    cell.setText(v)
        n_conf = sum(1 for p in pins.values() if p.get('confirmed'))
        self.lbl_pins.setText(f'핀 {len(pins)}개 (확정 {n_conf}) · 이미지 {info.get("out_dir", "—")}'
                              if n.pins.v is not None else '/pin_gallery/pins_info 없음 — pin_gallery·pin_node 확인')
        self.dialog.refresh(pins, rover, frame)
        self.deg_dialog.refresh(info.get('degradation'))


# ============================================================ 실행
def main():
    rclpy.init()
    node = GroundNode()
    ex = SingleThreadedExecutor()
    ex.add_node(node)

    def spin():
        try:
            ex.spin()
        except (ExternalShutdownException, Exception):  # noqa: BLE001 — 종료 중 context 내려감
            pass
    th = threading.Thread(target=spin, daemon=True)
    th.start()

    app = QtWidgets.QApplication(sys.argv[:1])
    win = MainWindow(node)
    win.show()
    signal.signal(signal.SIGINT, lambda *_: app.quit())         # launch Ctrl+C
    signal.signal(signal.SIGTERM, lambda *_: app.quit())
    beat = QtCore.QTimer()
    beat.timeout.connect(lambda: None)                         # Qt 루프 중에도 파이썬 시그널 처리
    beat.start(200)
    rc = app.exec_()
    win.rover.shutdown()                                      # [261010] 조종 ON 이었으면 0 지령 + stop (OFF 면 아무것도 안 보냄), [261012] 자율 ON 이면 끔
    win.wplog.close()
    ex.shutdown()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
