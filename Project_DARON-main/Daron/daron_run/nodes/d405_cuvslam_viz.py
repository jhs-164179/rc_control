#!/usr/bin/env python3
"""External cuVSLAM consumer for OmniLRS rover_3 D405 ROS2 stereo topics.

=== 시각화 전용 복사본 (2026-09-20, ICRA 써플 영상용) ===
원본 ~/d405_cuvslam_real.py 를 복사해 RViz 발행만 덧붙였다. 추정 로직(track 호출,
공분산, 재획득 재정렬, /slam/odometry 내용)은 원본과 동일하다. 추가된 코드는 전부
"[viz]" 주석으로 표시했고, TF 는 발행하지 않는다(odom->base_link 는 EKF 가 낸다).
9/22 현장 촬영은 원본으로 진행하므로 원본을 고치지 말 것.

[261008] cuVSLAM Slam (루프 클로저 · 맵 저장 · localize_in_map) — slam_enable:=true 일 때만. 근거: tools/slam_offline.py,
  ~/Daron/analysis_slam_1008 (오프라인 시험: 다른 세션 출발점 재추정 14/14 성공 100~140 ms, Slam 추가 비용 +0.1 ms/프레임, +128 MiB).
  * EKF 입력(/slam/odometry)은 그대로 VO — 끊김이 없어야 한다. Slam 보정은 TF map->odom 으로만 낸다(REP-105).
      T_map_base = Slam(rig) · rig_from_base,  map_from_odom = T_map_base · T_odom_base(VO)⁻¹
  * 맵 좌표계 = 이 실행의 odom(출발 = 중력 정렬 원점)과 같게 시작: 첫 slam.track 직후 set_slam_pose(현재 VO rig 포즈).
    (첫 track 전에 부르면 첫 PGO 때까지 반영이 안 되다가 점프한다 — 오프라인 실측)
  * rig_convention: localize_in_map 의 탐색 격자·각도 탐색이 OpenCV 축 rig 를 가정한다(ROS 축이면 '수직' 반경이 좌우,
    각도 탐색이 pitch 가 된다 — 45° 헤딩 오차·0.5 m 좌우 오차에서 실패). cv 로 rig 를 만들고 API 경계에서
    T_ros = Q·T_cv·Q⁻¹ (Q = [0.5,-0.5,0.5,-0.5]) 로 바꾼다. VO 는 ros rig 와 5~8 mm 차이.
  * 재추정(slam_map_load_dir): 출발점(맵을 만든 실행의 출발 표시)에 정지한 채 기동. 저장된 출발 rig 포즈를 추정값으로
    localize_in_map → 성공하면 추정값을 바꿔(reloc_confirm_*) 한 번 더 → 두 결과의 map←odom 이 2 cm·1° 안이면 ACCEPTED,
    아니면 REJECTED(이 실행 자기 맵으로 되돌림). 주행 중 재추정은 '성공'이어도 수 cm~수십 cm 틀린 사례가 있어 안 한다.
  * 출력: /slam/reloc_status (String JSON, TRANSIENT_LOCAL), /slam/loop_closure (String JSON, 보정 이벤트),
          /slam/map_pose (Odometry, map 프레임), TF map->odom (map_tf:=true).

[261011] lc_history — 루프 클로저 뒤 과거 프레임 전체의 보정된 map<-odom 을 /slam/map_odom_history 로 (피쳐맵 재동기화용)
  * slam.track 출력(=TF map->odom)은 PGO 결과 중 1 % 넘는 것만 반영한다(오프라인 함정 2). get_all_slam_poses() 는
    프레임마다 PGO 가 반영된 포즈를 준다(trav_obstacle: 2762/2762 프레임, max_map_size 300 과 무관, track 과 최대 2.1 cm·0.16°).
  * 프레임마다 (stamp, VO odom<-base, 그때 낸 map<-odom) 를 기록 → 새 루프 클로저가 보이면(1 s 마다 확인,
    lc_history_min_interval_s 간격) 트래킹 스레드에서 get_all_slam_poses() 만 부르고, 변환·발행은 별도 스레드:
        M_corr(t) = Slam_all(t) · rig_from_base · (VO odom<-base(t))⁻¹      (실시간 mfo_now 와 같은 식)
    nav_msgs/Path: header.frame_id = map, poses[i].header = (그 프레임 stamp, odom), pose = M_corr(t). TRANSIENT_LOCAL.
  * 재추정 PENDING/CONFIRMING 프레임은 기록 안 함(맵 좌표가 확정 전). REJECTED 로 되돌리면 기록을 비운다.
  * 그때 낸 map<-odom 과의 차이가 lc_history_min_m·deg 보다 작으면 안 낸다. /slam/odometry(EKF 입력)는 그대로.
  * [261011 밤] lc_robot (기본 켬) — 로버 TF 도 같은 보정을 따른다: 이력을 낼 때 마지막 프레임의
        오프셋 Off = M_corr(t_last) · M_raw(t_last)⁻¹   (M_raw = Slam.track 기반 실시간 map<-odom)
    을 구해 이후 TF map->odom = Off · M_raw 로 낸다 → 루프 클로저 순간 로버가 보정량만큼 옮겨지고 지도·핀·웨이포인트와 맞는다.
    이중 적용 방지: Slam.track 이 PGO 를 직접 반영해 M_raw 가 lc_track_jump_m·deg 넘게 바뀌면 Off = I 로 되돌리고 이력을 다시 낸다.
    이력을 계산하는 사이 M_raw 가 바뀌었으면 그 Off 는 버린다.
"""

import array
import bisect
import copy
import json
import math
import os
import threading
import time
from collections import OrderedDict

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, QuaternionStamped, TransformStamped
from nav_msgs.msg import Odometry, Path
from rclpy.clock import Clock
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy,
                       qos_profile_sensor_data)
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster
from visualization_msgs.msg import Marker

# [viz] 튜토리얼(Isaac ROS Visual SLAM)의 rviz 설정을 재활용하기 위해 토픽명을 그대로 맞춘다.
VIZ_LANDMARKS_TOPIC = "/visual_slam/vis/landmarks_cloud"
VIZ_OBSERVATIONS_TOPIC = "/visual_slam/vis/observations_cloud"
VIZ_VO_PATH_TOPIC = "/visual_slam/tracking/vo_path"
VIZ_SLAM_PATH_TOPIC = "/visual_slam/tracking/slam_path"
VIZ_IMAGE_TOPIC = "/visual_slam/vis/observations_image"
VIZ_OUTAGE_MARKER_TOPIC = "/visual_slam/vis/outage_marker"
VIZ_PATH_MAX_POSES = 3000
# [261005] Path 발행 주기(벽시계). 포즈는 매 프레임 쌓고 발행만 묶는다 — RViz Humble 의 tf2_ros 교착
# (Path 수신마다 waitForTransform 잠금, TF 수신 스레드와 역순 잠금) 확률을 줄이고 누적 Path 재직렬화 부하도 줄인다.
VIZ_PATH_PERIOD_S = 0.2
VIZ_LANDMARK_PERIOD_S = 0.5          # 2 Hz (벽시계 기준 — 재생 배속과 무관)
VIZ_LANDMARK_SLOW_PERIOD_S = 1.0     # get_final_landmarks() 가 30 ms 를 넘으면 1 Hz 로
VIZ_LANDMARK_SLOW_THRESHOLD_MS = 30.0


def cuvslam_DataLayer_Map():
    import pycuvslam as cuvslam
    return cuvslam.Slam.DataLayer.Map


def _make_cloud(points: np.ndarray, frame_id: str, stamp_ns: int) -> PointCloud2:
    """[viz] Nx3 float 배열 -> PointCloud2(x,y,z float32). 빈 배열이면 폭 0 인 메시지."""
    msg = PointCloud2()
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.header.frame_id = frame_id
    msg.height = 1
    msg.width = int(points.shape[0])
    msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * msg.width
    msg.is_dense = True
    data = np.ascontiguousarray(points, dtype=np.float32).tobytes()
    msg.data = array.array("B", data)   # tobytes() 직접 대입보다 빠름(기존 성능 메모)
    return msg


def _gray(msg: Image) -> np.ndarray:
    enc = msg.encoding.lower()
    channels = 1 if enc in ("mono8", "8uc1") else 4 if "rgba" in enc or "bgra" in enc else 3
    rows = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    image = rows[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
    if channels == 1:
        return np.ascontiguousarray(image[:, :, 0])
    rgb = image[:, :, :3]
    if enc.startswith("bgr"):
        rgb = rgb[:, :, ::-1]
    gray = (
        0.299 * rgb[:, :, 0]
        + 0.587 * rgb[:, :, 1]
        + 0.114 * rgb[:, :, 2]
    )
    return np.ascontiguousarray(gray.astype(np.uint8))


# nav_msgs/Odometry covariance is a row-major 6x6 (x, y, z, roll, pitch, yaw);
# the diagonal lives at indices 0, 7, 14, 21, 28, 35.
TRACKED_VARIANCE = 0.01  # ~10cm / ~0.1rad standard deviation
LOST_VARIANCE = 1e6  # "do not use this estimate" for nav2 / robot_localization


def _diagonal_covariance(position_var: float, orientation_var: float) -> list:
    covariance = [0.0] * 36
    for i in range(3):
        covariance[i * 7] = position_var
    for i in range(3, 6):
        covariance[i * 7] = orientation_var
    return covariance


def _quaternion_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)


def _quaternion_multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate_vector(q, v):
    """Rotate v by q: q * (v, 0) * q^-1."""
    rotated = _quaternion_multiply(
        _quaternion_multiply(q, (float(v[0]), float(v[1]), float(v[2]), 0.0)),
        _quaternion_conjugate(q),
    )
    return np.array(rotated[:3], dtype=np.float64)


def _rotation_vector(q):
    """Unit quaternion -> axis * angle (rad)."""
    x, y, z, w = q
    if w < 0.0:  # take the shorter of the two equivalent rotations
        x, y, z, w = -x, -y, -z, -w
    axis_norm = float(np.sqrt(x * x + y * y + z * z))
    if axis_norm < 1e-12:
        # angle -> 0, where 2 * atan2(n, w) / n -> 2 / w -> 2.
        return np.array([2.0 * x, 2.0 * y, 2.0 * z], dtype=np.float64)
    angle = 2.0 * float(np.arctan2(axis_norm, w))
    return np.array([x, y, z], dtype=np.float64) * (angle / axis_norm)


def _color_from_id(identifier: int):
    return [
        (int(identifier) * 17) % 256,
        (int(identifier) * 31) % 256,
        (int(identifier) * 47) % 256,
    ]


def _quaternion_to_matrix(quaternion) -> np.ndarray:
    x, y, z, w = (float(v) for v in quaternion)
    norm = x * x + y * y + z * z + w * w
    if norm < 1e-12:
        return np.eye(3)
    s = 2.0 / norm
    return np.array([
        [1.0 - s * (y * y + z * z), s * (x * y - z * w), s * (x * z + y * w)],
        [s * (x * y + z * w), 1.0 - s * (x * x + z * z), s * (y * z - x * w)],
        [s * (x * z - y * w), s * (y * z + x * w), 1.0 - s * (x * x + y * y)],
    ])


def _matrix_to_quaternion(matrix: np.ndarray):
    """3x3 rotation matrix -> (x, y, z, w). Shepperd's method: pick the branch with
    the largest denominator so no near-180-degree rotation divides by ~0."""
    trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * s
        x = (matrix[2, 1] - matrix[1, 2]) / s
        y = (matrix[0, 2] - matrix[2, 0]) / s
        z = (matrix[1, 0] - matrix[0, 1]) / s
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / s
        x = 0.25 * s
        y = (matrix[0, 1] + matrix[1, 0]) / s
        z = (matrix[0, 2] + matrix[2, 0]) / s
    elif matrix[1, 1] > matrix[2, 2]:
        s = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / s
        x = (matrix[0, 1] + matrix[1, 0]) / s
        y = 0.25 * s
        z = (matrix[1, 2] + matrix[2, 1]) / s
    else:
        s = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / s
        x = (matrix[0, 2] + matrix[2, 0]) / s
        y = (matrix[1, 2] + matrix[2, 1]) / s
        z = 0.25 * s
    return (x, y, z, w)


def _rpy_to_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """URDF/REP-103 convention: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ])


def _make_transform(rotation_matrix: np.ndarray, translation) -> np.ndarray:
    transform = np.eye(4)
    transform[:3, :3] = rotation_matrix
    transform[:3, 3] = np.asarray(translation, dtype=np.float64)
    return transform


def _invert_transform(transform: np.ndarray) -> np.ndarray:
    rotation = transform[:3, :3]
    inverse = np.eye(4)
    inverse[:3, :3] = rotation.T
    inverse[:3, 3] = -rotation.T @ transform[:3, 3]
    return inverse


class D405CuVSLAM(Node):
    def __init__(self):
        super().__init__("d405_cuvslam")
        # 2026-09-03 실측. 하드코딩 18mm 대비 1.35% 스케일 오차가 있었음.
        # TODO: 근본 해결은 CameraInfo.P[3]에서 읽는 것.
        #       run_cuvslam_offline.py는 이미 -P[3]/P[0]로 역산 중.
        #       실시간 노드도 같은 방식으로 바꿀 것.
        self.declare_parameter("baseline_m", 0.017761)
        self.declare_parameter("sync_tolerance_ms", 5.0)
        self.declare_parameter("warmup_seconds", 6.0)
        self.declare_parameter("infra1_topic", "/camera/camera/infra1/image_rect_raw")
        self.declare_parameter("infra2_topic", "/camera/camera/infra2/image_rect_raw")

        # 규약 §2-2 토픽/프레임 이름. 전부 파라미터라 launch에서 덮어쓸 수 있다.
        self.declare_parameter("odom_topic", "/slam/odometry")
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("base_frame_id", "base_link")
        self.declare_parameter("map_frame_id", "map")

        # base_link -> camera_link 정적 변환. 기본값은 daron.urdf.xacro mount:=cad0902 값
        # (준서님 Fusion 측정, 2026-09-02). [261008] daron_viz.launch.py 는 생성한 URDF 의
        # base_to_camera 를 읽어 이 6개를 항상 넘긴다(기본 mount:=level → rpy 0) — 단독 실행 때만 기본값이 쓰인다.
        self.declare_parameter("cam_x", 0.244427)
        self.declare_parameter("cam_y", 0.0)
        self.declare_parameter("cam_z", 0.034352)
        self.declare_parameter("cam_roll", 0.0)
        self.declare_parameter("cam_pitch", -0.13963)
        self.declare_parameter("cam_yaw", 0.0)

        # 규약 §2-2: 최종 구성에서는 cuVSLAM이 map->odom만,
        # EKF가 odom->base_link를 발행한다. EKF 착수(9/6) 전까지
        # 임시로 odom_base를 쓴다. 둘을 동시에 켜면 TF가 진동한다.
        self.declare_parameter("tf_mode", "odom_base")
        # Dynamic self-occlusion mask stand-in: ratio oscillates with frame number instead
        # of staying fixed for the process lifetime, to check KillTracksWithinMask()
        # stability under a mask that changes shape every frame (not just once at startup).
        # ratio(frame) = base + amplitude * sin(2*pi*frame/period_frames), clipped to [0,1].
        # 2026-09-07 기본값 꺼짐으로 변경. 켜려면 파라미터로 명시.
        self.declare_parameter("mask_base_ratio", 0.0)
        self.declare_parameter("mask_amplitude", 0.0)
        self.declare_parameter("mask_period_frames", 90.0)
        self._last_mask_ratio = 0.0
        # Diagnostic-only: dump (id,u,v) of observations_left at each 30-frame checkpoint
        # so per-landmark inter-frame displacement can be checked offline. Empty = disabled.
        self.declare_parameter("landmark_log_path", "")
        # Diagnostic-only: dump get_final_landmarks() (id,x,y,z world coords, RIGHT_HAND_Z_UP)
        # at each 30-frame checkpoint, to check offline for planar structure (floor/wall
        # clusters in a Z histogram) vs an unstructured point cloud. Empty = disabled.
        self.declare_parameter("landmark3d_log_path", "")
        # Rerun spawns a GUI viewer; disable it for headless runs and bag replays.
        self.declare_parameter("enable_rerun", True)
        # 인위적 추적 소실 주입 (실험용). "시작,끝"[;"시작,끝"...] [s] — 노드 시계(use_sim_time 이면
        # bag 재생 시각) 기준, 노드가 첫 스테레오 쌍을 받은 순간부터의 경과 초. 그 구간에서는 cuVSLAM 이
        # 유효 pose 를 내도 소실로 취급해 기존 lost 경로(마지막 pose 를 LOST_VARIANCE 로 재발행 →
        # 구간 끝에 RELOCK 재정렬)를 그대로 탄다. 빈 문자열 = 끔 (그 경우 코드 경로가 기존과 같다).
        # launch 가 -p 를 고정해 두어 환경변수 DARON_FAULT_INJECT_S 로도 줄 수 있다 (파라미터가 우선).
        self.declare_parameter("fault_inject_s", os.environ.get("DARON_FAULT_INJECT_S", ""))
        # [261008] 출발 자세(중력 기준). imu_recalib_relay(mount_source:=tf)가 첫 출력 base 자세
        # R_world←base(t0)(yaw 0)를 TRANSIENT_LOCAL 로 한 번 낸다. 받으면 odom 원점 자세를 그걸로 잡아
        #   T_odom_base(t) = R0 · inv(T_world_base(t0)) · T_world_base(t)
        # → odom z = 중력, 기울어진 곳에서 출발해도 SLAM·EKF 가 같은 기준. 빈 문자열 = 옛 동작(출발 base = odom).
        # 첫 락 뒤 start_attitude_wait_s(센서 시각) 동안 못 받으면 항등으로 시작하고 경고한다.
        self.declare_parameter("start_attitude_topic", "")
        self.declare_parameter("start_attitude_wait_s", 5.0)
        # [261008] Slam (상단 docstring)
        self.declare_parameter("rig_convention", "ros")        # ros | cv
        self.declare_parameter("slam_enable", False)
        self.declare_parameter("slam_sync", False)
        self.declare_parameter("map_tf", True)                 # slam_enable 일 때 TF map->odom 발행
        self.declare_parameter("slam_map_save_dir", "")        # 비우면 저장 안 함. 종료 시 + ~/save_slam_map
        self.declare_parameter("slam_map_load_dir", "")        # 비우면 새 맵
        self.declare_parameter("reloc_h_radius", 1.0)
        self.declare_parameter("reloc_h_step", 0.25)
        self.declare_parameter("reloc_v_radius", 0.25)
        self.declare_parameter("reloc_v_step", 0.0625)
        self.declare_parameter("reloc_angular_step_deg", 5.0)
        self.declare_parameter("reloc_confirm_dx", 0.3)        # 확인용 둘째 추정값 = 출발 포즈 + (dx 전방, dyaw)
        self.declare_parameter("reloc_confirm_dyaw_deg", 30.0)
        self.declare_parameter("reloc_agree_m", 0.02)
        self.declare_parameter("reloc_agree_deg", 1.0)
        self.declare_parameter("reloc_max_tries", 8)
        self.declare_parameter("reloc_retry_frames", 15)
        self.declare_parameter("lc_event_m", 0.01)             # map<-odom 이 이만큼(또는 0.3°) 바뀌면 보정 이벤트
        self.declare_parameter("lc_history", False)            # [261011] /slam/map_odom_history 발행 (상단 docstring)
        self.declare_parameter("lc_history_min_interval_s", 2.0)
        self.declare_parameter("lc_history_min_m", 0.002)
        self.declare_parameter("lc_history_min_deg", 0.05)
        self.declare_parameter("lc_robot", True)               # [261011 밤] 로버 TF 도 보정 궤적을 따름 (lc_history 필요)
        self.declare_parameter("lc_track_jump_m", 0.002)       # Slam.track 이 PGO 를 직접 반영한 것으로 볼 M_raw 변화
        self.declare_parameter("lc_track_jump_deg", 0.1)
        self._left = OrderedDict()
        self._right = OrderedDict()
        self._info = [None, None]
        self._tracker = None
        self._last_stamp = 0
        self._frames = 0
        self._trajectory = []
        self._pairs_received = 0
        self._paired_stamp = [0, 0]     # [261016] 마지막으로 짝지어 쓴 (left, right) stamp — 이하로 오는 프레임은 버림
        self._stale_frames = 0
        self._warmup_start_stamp = None
        self._warmup_logged_second = -1
        self._first_pose = None
        # base_link 기준 원점의 역변환 inv(T_world_base(t0)). _first_pose(rig 기준)와
        # 별도로 둔다 -- _first_pose는 Rerun의 landmark/궤적 표시가 계속 rig 좌표계를
        # 쓰기 때문에 의미를 바꿀 수 없다.
        self._odom_from_world = None
        # Most recent successful (relative_position, rotation xyzw); republished with
        # LOST_VARIANCE while tracking is down so the topic keeps a valid timestamp.
        self._last_pose = None
        # True while cuVSLAM is returning world_from_rig=None after a first lock.
        # cuVSLAM restarts in a fresh coordinate system after a loss of tracking
        # (pycuvslam.pyi:752 "same coordinate system until a loss of tracking"),
        # so the lost->tracked edge must re-anchor _odom_from_world; see _track().
        self._tracking_lost = False
        self._fault_windows = []        # [(start_s, end_s), ...] 비어 있으면 주입 없음
        self._fault_t0_ns = None        # 첫 스테레오 쌍 시점의 노드 시계 [ns]
        self._fault_active = False      # 직전 프레임이 주입 구간이었는가 (start/end 로그용)
        spec = str(self.get_parameter("fault_inject_s").value).strip()
        if spec:
            try:
                for part in spec.split(";"):
                    part = part.strip()
                    if not part:
                        continue
                    a, b = (float(v) for v in part.split(","))
                    if b <= a:
                        raise ValueError(f"end <= start in {part!r}")
                    self._fault_windows.append((a, b))
                self._fault_windows.sort()
            except ValueError as exc:
                raise SystemExit(f"fault_inject_s 는 '시작,끝[;시작,끝...]' 초 형식이어야 합니다: {spec!r} ({exc})")
            self.get_logger().warning(
                "FAULT INJECT armed: tracking forced LOST for "
                + ", ".join(f"t={a:g}..{b:g}s" for a, b in self._fault_windows)
                + " (node clock since first stereo pair)")
        # (stamp_ns, relative_position) of the previous published fix, for the twist
        # finite difference.
        self._twist_reference = None
        self._rr = None
        if not bool(self.get_parameter("enable_rerun").value):
            self.get_logger().info("Rerun disabled (enable_rerun=False)")
        else:
            try:
                import rerun as rr
                import rerun.blueprint as rrb

                self._rr = rr
                rr.init("d405_cuvslam_ros", spawn=True)
                rr.log("/world", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True)
                rr.log(
                    "/world/axes",
                    rr.Arrows3D(
                        vectors=[[1, 0, 0], [0, 1, 0], [0, 0, 1]],
                        colors=[[255, 0, 0], [0, 255, 0], [0, 0, 255]],
                        labels=["X forward", "Y left", "Z up"],
                    ),
                    static=True,
                )
                rr.send_blueprint(
                    rrb.Blueprint(
                        rrb.Horizontal(
                            rrb.Spatial3DView(name="Odometry 3D", origin="/world"),
                            rrb.Vertical(
                                rrb.Spatial2DView(
                                    name="D405 Left Features",
                                    origin="/world/features/left",
                                ),
                                rrb.Spatial2DView(
                                    name="D405 Right Features",
                                    origin="/world/features/right",
                                ),
                            ),
                            column_shares=[2, 1],
                        )
                    )
                )
                self.get_logger().info("Rerun viewer launched")
            except Exception as exc:
                self.get_logger().error(f"Failed to launch Rerun: {exc}")
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._odom_frame_id = str(self.get_parameter("odom_frame_id").value)
        self._base_frame_id = str(self.get_parameter("base_frame_id").value)
        self._map_frame_id = str(self.get_parameter("map_frame_id").value)

        # cuVSLAM이 돌려주는 world_from_rig는 rig(= 좌우 이미저의 중점, X-forward/
        # Y-left/Z-up)의 pose다. camera_link는 왼쪽 이미저에 있으므로 rig는 거기서
        # y로 -baseline/2 만큼 떨어져 있다 (2026-09-03 TF 실측:
        # camera_link -> camera_infra2_optical_frame = [0, -0.017761, 0], 회전 동일).
        #   T_base_rig = T_base_cam * T_cam_rig,  T_world_base = T_world_rig * inv(T_base_rig)
        half = float(self.get_parameter("baseline_m").value) * 0.5
        base_from_camera = _make_transform(
            _rpy_to_matrix(
                float(self.get_parameter("cam_roll").value),
                float(self.get_parameter("cam_pitch").value),
                float(self.get_parameter("cam_yaw").value),
            ),
            (
                float(self.get_parameter("cam_x").value),
                float(self.get_parameter("cam_y").value),
                float(self.get_parameter("cam_z").value),
            ),
        )
        camera_from_rig = _make_transform(np.eye(3), (0.0, -half, 0.0))
        self._base_from_rig = base_from_camera @ camera_from_rig
        self._rig_from_base = _invert_transform(self._base_from_rig)

        tf_mode = str(self.get_parameter("tf_mode").value)
        if tf_mode not in ("none", "odom_base", "map_odom"):
            self.get_logger().error(
                f"Unknown tf_mode={tf_mode!r} (expected none/odom_base/map_odom); "
                "falling back to 'none' rather than publishing a wrong TF tree"
            )
            tf_mode = "none"
        self._tf_mode = tf_mode
        self._tf_broadcaster = (
            None if tf_mode == "none" else TransformBroadcaster(self)
        )
        self.get_logger().info(
            f"frames: {self._odom_frame_id} -> {self._base_frame_id}, "
            f"topic={self._odom_topic}, tf_mode={self._tf_mode}, "
            f"base_from_rig t="
            f"({self._base_from_rig[0, 3]:+.6f},{self._base_from_rig[1, 3]:+.6f},"
            f"{self._base_from_rig[2, 3]:+.6f})m"
        )
        self._odom_pub = self.create_publisher(Odometry, self._odom_topic, 10)
        # [261008] 출발 자세
        self._start_topic = str(self.get_parameter("start_attitude_topic").value).strip()
        self._start_wait_ns = int(float(self.get_parameter("start_attitude_wait_s").value) * 1e9)
        self._start_R = None                 # world(중력)←base(t0), yaw 0
        self._start_wait_from = None         # 첫 락 시각 [ns] (기다림 시작)
        self._start_waiting_logged = False
        if self._start_topic:
            latched = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                 reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.create_subscription(QuaternionStamped, self._start_topic, self._on_start_attitude, latched)
            self.get_logger().info(f"start attitude: {self._start_topic} 를 받아 odom 원점 자세로 씀 "
                                   f"(최대 {self._start_wait_ns * 1e-9:.1f} s 대기)")
        # [261008] Slam 상태
        self._cv = str(self.get_parameter("rig_convention").value).strip().lower() == "cv"
        self._Q = _quaternion_to_matrix((0.5, -0.5, 0.5, -0.5))        # ROS rig 축 <- OpenCV rig 축
        self._slam_on = bool(self.get_parameter("slam_enable").value)
        self._slam = None
        self._slam_started = False
        self._map_from_odom = np.eye(4)          # 마지막으로 낸(=확정된) map<-odom
        self._mfo_prev = None
        self._reloc = "OFF"                      # OFF | NEW_MAP | PENDING | CONFIRMING | ACCEPTED | REJECTED | FAILED
        self._reloc_info = {}
        self._loc_result = None                  # 콜백 결과 (pose|None, err)
        self._loc_busy = False
        self._loc_first = None                   # 첫 성공 map<-odom
        self._loc_tries = 0
        self._loc_next_frame = 0
        self._start_rig_map = None               # 이 실행 출발 rig 포즈 (map, ROS 축)
        self._guess0 = None                      # 재추정 추정값 (불러온 맵의 출발 rig 포즈)
        self._hold_frames = 0
        self._loaded_meta = None
        self._seen_lc = set()
        self._lc_count = 0
        # [261011] lc_history — 프레임 기록 (stamp, VO odom<-base, 그때 낸 map<-odom)
        self._hist_on = self._slam_on and bool(self.get_parameter("lc_history").value)
        self._hist_t, self._hist_ob, self._hist_mfo = [], [], []
        self._hist_raw = []                      # [261011 밤] 같은 프레임의 M_raw (Off 계산용)
        self._robot_on = self._hist_on and bool(self.get_parameter("lc_robot").value)
        self._lc_off = np.eye(4)                 # TF 오프셋 Off (트래킹 스레드가 읽고 이력 스레드가 바꿈 — 대입 한 번)
        self._mfo_raw_prev = None
        self._robot_moves = 0
        self._hist_lc_sent = 0                   # 마지막 발행 때의 _lc_count
        self._hist_due = False
        self._hist_last = -1e9                   # time.monotonic()
        self._hist_busy = False
        self._hist_n = 0
        self._save_dir = str(self.get_parameter("slam_map_save_dir").value).strip()
        self._load_dir = str(self.get_parameter("slam_map_load_dir").value).strip()
        if self._slam_on and not self._cv:
            self.get_logger().warning("slam_enable 인데 rig_convention=ros — localize_in_map 탐색이 축을 잘못 잡는다"
                                      " (헤딩 오차 15° 넘으면 실패). rig_convention:=cv 권장")
        latched_s = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1, reliability=ReliabilityPolicy.RELIABLE,
                               durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._reloc_pub = self.create_publisher(String, "/slam/reloc_status", latched_s)
        self._lc_pub = self.create_publisher(String, "/slam/loop_closure", 10)
        self._map_pose_pub = self.create_publisher(Odometry, "/slam/map_pose", 10)
        self._hist_pub = (self.create_publisher(Path, "/slam/map_odom_history", latched_s) if self._hist_on else None)
        # [261008 밤] map->odom 은 /tf_static 으로 — 바뀔 때만(루프 클로저·재추정) 다시 낸다.
        # 움직이는 TF(/tf 20 Hz)로 내면 RViz Humble 의 tf2 교착(Path 표시가 TF 를 기다리며 TF 수신 스레드와 잠금을
        # 엇갈려 잡음)이 잦아져 3D 창이 멈췄다(10/8 실측: 메인 스레드 futex 대기, CPU 시간 정지). static 이면 시각 보간이
        # 필요 없어 기다릴 일이 없다. 대신 지난 시각 조회도 지금 보정값을 쓴다(보정은 드물고 작아 문제 없음).
        self._map_tf = (StaticTransformBroadcaster(self)
                        if self._slam_on and bool(self.get_parameter("map_tf").value) else None)
        self._map_tf_sent = None
        if self._map_tf is not None:
            # 기동 즉시 항등을 한 번 내고(Slam 시작 전에도 map 이 있게), 0.5 s 마다 바뀌었는지 보고 바뀌었으면 다시 냄
            self.create_timer(0.5, self._map_tf_timer)
        if self._slam_on:
            from std_srvs.srv import Trigger
            self.create_service(Trigger, "~/save_slam_map", self._srv_save_map)
            self.get_logger().info(
                f"Slam on: rig={'cv' if self._cv else 'ros'} sync={bool(self.get_parameter('slam_sync').value)} "
                f"save={self._save_dir or '(안 함)'} load={self._load_dir or '(새 맵)'} map_tf={self._map_tf is not None}")
        # [health 260929] 프레임마다 트래킹 상태·특징점 수 — health_monitor 의 SLAM 자기진단 입력.
        #   JSON {stamp_ns, tracked, features_l, relock_jump_m|null}. stamp 은 /slam/odometry 와 같은 값.
        self._status_pub = self.create_publisher(String, "/slam/status", 10)
        self.create_subscription(
            CameraInfo,
            "/camera/camera/infra1/camera_info",
            lambda msg: self._set_info(0, msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/camera/camera/infra2/camera_info",
            lambda msg: self._set_info(1, msg),
            qos_profile_sensor_data,
        )
        infra1_topic = self.get_parameter("infra1_topic").value
        infra2_topic = self.get_parameter("infra2_topic").value
        self.create_subscription(
            Image,
            infra1_topic,
            lambda msg: self._image(0, msg),
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            infra2_topic,
            lambda msg: self._image(1, msg),
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"Waiting for rover_3 D405 stereo topics: {infra1_topic}, {infra2_topic}"
        )
        base_ratio = float(self.get_parameter("mask_base_ratio").value)
        amplitude = float(self.get_parameter("mask_amplitude").value)
        period_frames = float(self.get_parameter("mask_period_frames").value)
        if base_ratio > 0.0 or amplitude > 0.0:
            self.get_logger().info(
                f"dynamic native mask active: base_ratio={base_ratio:.0%}, "
                f"amplitude={amplitude:.0%}, period_frames={period_frames:.0f} "
                f"(range [{max(0.0, base_ratio - amplitude):.0%}, "
                f"{min(1.0, base_ratio + amplitude):.0%}], passed via track(masks=...))"
            )

        # ── [viz] RViz 발행 (여기부터 __init__ 끝까지 추가분) ──────────────────
        # 항상 켜져 있다(viz_enable 파라미터 없음 — 이 파일 자체가 시각화 전용 복사본).
        self.declare_parameter("ekf_odom_topic", "/odometry/filtered")
        # [viz] 랜드마크 거리 필터(표시용). get_last_landmarks() 의 coords 는 rig 프레임이라
        #       norm(coords) = 카메라로부터의 거리. 이 값 이하인 id 만 관측/영상에 그리고,
        #       landmarks_cloud 는 "한 번이라도 통과한 id" 화이트리스트로 거른다.
        #       추정(track/공분산/재정렬/odometry)에는 관여하지 않는다.
        self.declare_parameter("viz_max_range", 2.0)
        self._viz_max_range = float(self.get_parameter("viz_max_range").value)
        self._viz_near_ids_ever: set[int] = set()    # landmarks_cloud 화이트리스트(누적)
        self._viz_range_last_log = 0.0               # time.monotonic()
        self._viz_landmarks_pub = self.create_publisher(PointCloud2, VIZ_LANDMARKS_TOPIC, 1)
        self._viz_observations_pub = self.create_publisher(PointCloud2, VIZ_OBSERVATIONS_TOPIC, 1)
        self._viz_vo_path_pub = self.create_publisher(Path, VIZ_VO_PATH_TOPIC, 1)
        self._viz_slam_path_pub = self.create_publisher(Path, VIZ_SLAM_PATH_TOPIC, 1)
        self._viz_image_pub = self.create_publisher(Image, VIZ_IMAGE_TOPIC, 1)
        self._viz_outage_pub = self.create_publisher(Marker, VIZ_OUTAGE_MARKER_TOPIC, 1)
        self._viz_vo_path = Path()
        self._viz_vo_path.header.frame_id = self._odom_frame_id
        self._viz_slam_path = Path()
        self._viz_slam_path.header.frame_id = self._odom_frame_id
        ekf_odom_topic = str(self.get_parameter("ekf_odom_topic").value)
        self.create_subscription(Odometry, ekf_odom_topic, self._viz_on_ekf_odom, 10)
        # get_final_landmarks() 는 누적 dict 라 프레임이 쌓일수록 커진다 -> track() 경로가
        # 아니라 별도 타이머에서 부른다. 벽시계 Clock 을 따로 주는 이유: use_sim_time 이면
        # 노드 시계가 bag 배속(0.3x)을 따라가 "2 Hz" 가 0.6 Hz 로 보이기 때문.
        self._viz_wall_clock = Clock()
        self._viz_landmark_period = VIZ_LANDMARK_PERIOD_S
        self._viz_landmark_timer = self.create_timer(
            self._viz_landmark_period, self._viz_publish_landmarks, clock=self._viz_wall_clock)
        self._viz_landmark_slowed = False
        self._viz_path_dirty = {}               # id(publisher) -> (path, publisher), 다음 타이머에서 발행
        self._viz_path_timer = self.create_timer(
            VIZ_PATH_PERIOD_S, self._viz_flush_paths, clock=self._viz_wall_clock)
        self._viz_landmark_last_log = 0.0        # time.monotonic()
        self._viz_obs_last_log = 0.0
        self._viz_obs_empty_warned = False
        self._viz_outage_active = False
        self._viz_outage_marker_seq = 0
        self.get_logger().info(
            "[viz] publishing " + ", ".join((
                VIZ_LANDMARKS_TOPIC + f" ({1.0 / self._viz_landmark_period:.0f} Hz wall)",
                VIZ_OBSERVATIONS_TOPIC, VIZ_VO_PATH_TOPIC,
                VIZ_SLAM_PATH_TOPIC + f" (<- {ekf_odom_topic})",
                VIZ_IMAGE_TOPIC, VIZ_OUTAGE_MARKER_TOPIC))
            + f"; all in frame '{self._odom_frame_id}', no TF broadcast; "
            f"range filter max_range={self._viz_max_range:.2f} m")

    # ── [viz] 발행 헬퍼 ────────────────────────────────────────────────────
    # ── [261008] 축 변환 · Slam ───────────────────────────────────────────
    def _from_api(self, pose) -> np.ndarray:
        """cuvslam.Pose -> ROS 축 4x4."""
        T = _make_transform(_quaternion_to_matrix(pose.rotation), np.asarray(pose.translation, dtype=np.float64))
        if not self._cv:
            return T
        Q = _make_transform(self._Q, (0.0, 0.0, 0.0))
        return Q @ T @ Q.T

    def _to_api(self, T: np.ndarray):
        import pycuvslam as cuvslam
        if self._cv:
            Q = _make_transform(self._Q, (0.0, 0.0, 0.0))
            T = Q.T @ T @ Q
        return cuvslam.Pose(translation=[float(v) for v in T[:3, 3]],
                            rotation=[float(v) for v in _matrix_to_quaternion(T[:3, :3])])

    def _publish_reloc(self, **extra) -> None:
        self._reloc_info.update(extra)
        d = dict(state=self._reloc, tries=self._loc_tries, lc_count=self._lc_count, **self._reloc_info)
        self._reloc_pub.publish(String(data=json.dumps(d)))

    def _on_loc(self, pose, err_msg="") -> None:
        self._loc_result = (pose, str(err_msg))

    def _start_localize(self, guess_map: np.ndarray, images) -> None:
        import pycuvslam as cuvslam
        g = self.get_parameter
        st = cuvslam.Slam.LocalizationSettings(
            horizontal_search_radius=float(g("reloc_h_radius").value),
            vertical_search_radius=float(g("reloc_v_radius").value),
            horizontal_step=float(g("reloc_h_step").value),
            vertical_step=float(g("reloc_v_step").value),
            angular_step_rads=math.radians(float(g("reloc_angular_step_deg").value)))
        self._loc_busy = True
        self._loc_tries += 1
        self._slam.localize_in_map(self._load_dir, self._to_api(guess_map), list(images), st, self._on_loc)

    @staticmethod
    def _T_delta(A: np.ndarray, B: np.ndarray):
        D = np.linalg.inv(A) @ B
        ang = math.degrees(math.acos(max(-1.0, min(1.0, (float(np.trace(D[:3, :3])) - 1.0) / 2.0))))
        return float(np.linalg.norm(A[:3, 3] - B[:3, 3])), ang

    def _confirm_guess(self) -> np.ndarray:
        g = self.get_parameter
        dyaw = math.radians(float(g("reloc_confirm_dyaw_deg").value))
        off = _make_transform(_rpy_to_matrix(0.0, 0.0, dyaw), (float(g("reloc_confirm_dx").value), 0.0, 0.0))
        return self._guess0 @ off

    def _slam_step(self, stamp: int, odom_from_base: np.ndarray, gray_left, gray_right) -> None:
        """VO 성공 프레임마다: Slam.track → (첫 프레임) 맵 좌표계 지정·재추정 시작 → 재추정 상태기계 → map<-odom TF."""
        odom_from_rig = odom_from_base @ self._base_from_rig
        try:
            sp = self._slam.track(self._tracker.get_state())
        except Exception as exc:
            self.get_logger().error(f"Slam.track failed: {type(exc).__name__}: {exc}", throttle_duration_sec=5.0)
            return
        if not self._slam_started:
            self._slam_started = True
            self._start_rig_map = odom_from_rig.copy()
            try:   # 첫 호출은 내부 기록을 켜기만 하고 빈 값을 준다(오프라인 실측) — 루프 클로저 집계용
                self._slam.get_pose_graph()
                self._slam.get_landmarks(cuvslam_DataLayer_Map())
            except Exception:
                pass
            self._slam.set_slam_pose(self._to_api(odom_from_rig))     # 맵 좌표계 = 이 실행 odom
            if self._load_dir:
                meta = os.path.join(self._load_dir, "daron_slam_meta.json")
                try:
                    with open(meta, encoding="utf-8") as f:
                        self._loaded_meta = json.load(f)
                    self._guess0 = np.array(self._loaded_meta["start_rig_map"], dtype=np.float64).reshape(4, 4)
                except Exception as exc:
                    self._reloc = "FAILED"
                    self._publish_reloc(map_dir=self._load_dir, error=f"meta 읽기 실패: {exc}")
                    self.get_logger().error(f"재추정 불가 — {meta}: {exc}. 새 맵으로 진행")
                    return
                self._reloc = "PENDING"
                self._start_localize(self._guess0, (gray_left, gray_right))
                self._publish_reloc(map_dir=self._load_dir, map_id=self._loaded_meta.get("map_id", ""))
                self.get_logger().info(f"재추정 시작: {self._load_dir} (map_id={self._loaded_meta.get('map_id', '')})")
            else:
                self._reloc = "NEW_MAP"
                self._publish_reloc(map_id=self._new_map_id())
            return
        T_map_rig = self._from_api(sp)
        mfo_now = T_map_rig @ self._rig_from_base @ np.linalg.inv(odom_from_base)
        # [261011 밤] Slam.track 이 PGO 를 직접 반영했으면(M_raw 변화) 오프셋을 버리고 새 이력으로 다시 맞춘다 (이중 적용 방지)
        if self._robot_on and self._mfo_raw_prev is not None:
            dmr, dar = self._T_delta(self._mfo_raw_prev, mfo_now)
            if dmr > float(self.get_parameter("lc_track_jump_m").value) or dar > float(self.get_parameter("lc_track_jump_deg").value):
                if not np.allclose(self._lc_off, np.eye(4)):
                    self.get_logger().info(f"[lc_robot] Slam.track 보정 반영({dmr * 100:.2f} cm / {dar:.3f}°) → 오프셋 초기화, 이력 다시")
                self._lc_off = np.eye(4)
                self._hist_due = True
        self._mfo_raw_prev = mfo_now

        # 재추정 상태기계 (콜백은 slam.track 안에서 온다)
        if self._loc_result is not None:
            pose, err = self._loc_result
            self._loc_result = None
            self._loc_busy = False
            if self._reloc == "PENDING":
                if pose is not None:
                    self._loc_first = self._from_api(pose) @ np.linalg.inv(odom_from_rig)
                    self._reloc = "CONFIRMING"
                    self._start_localize(self._confirm_guess(), (gray_left, gray_right))
                    self.get_logger().info("재추정 1차 성공 — 추정값을 바꿔 확인 중")
                elif self._loc_tries >= int(self.get_parameter("reloc_max_tries").value):
                    self._reloc = "FAILED"
                    self.get_logger().warning(f"재추정 실패 {self._loc_tries}회 ({err}) — 새 맵으로 진행")
                else:
                    self._loc_next_frame = self._frames + int(self.get_parameter("reloc_retry_frames").value)
                self._publish_reloc(last_error=err if pose is None else "")
            elif self._reloc == "CONFIRMING":
                ok = False
                if pose is not None:
                    second = self._from_api(pose) @ np.linalg.inv(odom_from_rig)
                    dm, da = self._T_delta(self._loc_first, second)
                    ok = (dm <= float(self.get_parameter("reloc_agree_m").value)
                          and da <= float(self.get_parameter("reloc_agree_deg").value))
                    self._reloc_info.update(confirm_delta_m=round(dm, 4), confirm_delta_deg=round(da, 3))
                if ok:
                    self._reloc = "ACCEPTED"
                    self._map_from_odom = second
                    mfo_now = second
                    self.get_logger().info(f"재추정 ACCEPTED (두 추정 차이 {dm * 100:.1f} cm / {da:.2f}°) — "
                                           f"map = {self._loaded_meta.get('map_id', '')} 좌표계")
                else:
                    self._reloc = "REJECTED"
                    # 마지막 확정 map<-odom 기준으로 되돌림 (set_slam_pose 는 다음 프레임부터 반영 → 2 프레임 유지)
                    self._slam.set_slam_pose(self._to_api(self._map_from_odom @ odom_from_rig))
                    self._hold_frames = 2
                    self._hist_t, self._hist_ob, self._hist_mfo, self._hist_raw = [], [], [], []   # [261011] 되돌린 맵 — 기록 다시
                    self.get_logger().warning(f"재추정 REJECTED ({err or '두 추정 불일치'}) — 이 실행 자기 맵으로 되돌림")
                self._publish_reloc(last_error=err)
        if self._reloc == "PENDING" and not self._loc_busy and self._frames >= self._loc_next_frame:
            self._start_localize(self._guess0, (gray_left, gray_right))
            self._publish_reloc()

        # 확정 전(PENDING/CONFIRMING)·되돌린 직후 프레임은 마지막 확정 map<-odom 유지
        if self._reloc in ("PENDING", "CONFIRMING") or self._hold_frames > 0:
            self._hold_frames = max(0, self._hold_frames - 1)
            mfo_pub = self._map_from_odom
        else:
            mfo_pub = self._lc_off @ mfo_now if self._robot_on else mfo_now   # [261011 밤] lc_robot
        if self._mfo_prev is not None:
            dm, da = self._T_delta(self._mfo_prev, mfo_pub)
            if dm > float(self.get_parameter("lc_event_m").value) or da > 0.3:
                self._lc_pub.publish(String(data=json.dumps(dict(
                    stamp_ns=int(stamp), dpos_m=round(dm, 4), dang_deg=round(da, 3), reloc=self._reloc))))
        self._mfo_prev = mfo_pub
        self._map_from_odom = mfo_pub
        if self._frames % 30 == 0:
            try:
                for ps in self._slam.get_loop_closure_poses():
                    k = int(ps.timestamp_ns)
                    if k not in self._seen_lc:
                        self._seen_lc.add(k)
                        self._lc_count += 1
            except Exception:
                pass
            if self._hist_on and self._lc_count > self._hist_lc_sent:
                self._hist_due = True
        if self._frames % 150 == 0:          # 약 5 s 마다 상태 재발행 (lc_count 갱신)
            self._publish_reloc()
        self._publish_map_outputs(stamp, mfo_pub @ odom_from_base, mfo_pub)
        # [261011] lc_history 기록·발행 (재추정 확정 전 프레임은 맵 좌표가 정해지지 않아 기록 안 함)
        if self._hist_on and self._reloc not in ("PENDING", "CONFIRMING") and self._hold_frames == 0:
            self._hist_t.append(int(stamp))
            self._hist_ob.append(odom_from_base.copy())
            self._hist_mfo.append(mfo_pub.copy())
            self._hist_raw.append(mfo_now.copy())
            if (self._hist_due and not self._hist_busy
                    and time.monotonic() - self._hist_last >= float(self.get_parameter("lc_history_min_interval_s").value)):
                self._start_history(stamp)

    def _start_history(self, stamp: int) -> None:
        """[261011] 트래킹 스레드: get_all_slam_poses() 만 부르고 나머지는 별도 스레드 (상단 docstring)."""
        self._hist_due = False
        self._hist_last = time.monotonic()
        self._hist_lc_sent = self._lc_count
        t0 = time.monotonic()
        try:
            allp = self._slam.get_all_slam_poses(0)
        except Exception as exc:
            self.get_logger().error(f"get_all_slam_poses 실패: {type(exc).__name__}: {exc}")
            return
        ms_get = (time.monotonic() - t0) * 1000
        n = len(self._hist_t)
        rec = (self._hist_t[:n], self._hist_ob[:n], self._hist_mfo[:n], self._hist_raw[:n])
        self._hist_busy = True
        threading.Thread(target=self._build_history, args=(allp, rec, ms_get, stamp, self._lc_count),
                         daemon=True).start()

    def _build_history(self, allp, rec, ms_get, stamp, lc_count) -> None:
        try:
            t0 = time.monotonic()
            poses = {int(ps.timestamp_ns): ps.pose for ps in allp}
            path = Path()
            path.header.frame_id = self._map_frame_id
            path.header.stamp.sec, path.header.stamp.nanosec = stamp // 1_000_000_000, stamp % 1_000_000_000
            dmax = amax = 0.0
            n_miss = 0
            last = None                                     # (M_corr, M_raw, M_pub) 마지막 프레임 — lc_robot 오프셋
            for st, ob, mfo_used, mfo_raw in zip(*rec):
                p = poses.get(st)
                if p is None:
                    n_miss += 1
                    continue
                M = self._from_api(p) @ self._rig_from_base @ _invert_transform(ob)
                last = (M, mfo_raw, mfo_used)
                dm, da = self._T_delta(mfo_used, M)
                dmax, amax = max(dmax, dm), max(amax, da)
                ps = PoseStamped()
                ps.header.frame_id = self._odom_frame_id
                ps.header.stamp.sec, ps.header.stamp.nanosec = st // 1_000_000_000, st % 1_000_000_000
                ps.pose.position.x, ps.pose.position.y, ps.pose.position.z = (float(v) for v in M[:3, 3])
                (ps.pose.orientation.x, ps.pose.orientation.y,
                 ps.pose.orientation.z, ps.pose.orientation.w) = (float(v) for v in _matrix_to_quaternion(M[:3, :3]))
                path.poses.append(ps)
            g = self.get_parameter
            small = dmax < float(g("lc_history_min_m").value) and amax < float(g("lc_history_min_deg").value)
            robot_txt = ""
            if path.poses and not small:
                self._hist_pub.publish(path)
                self._hist_n += 1
                # [261011 밤] lc_robot: 로버 TF 를 같은 보정으로. 계산하는 사이 track 이 바뀌었으면(raw 변화) 이번 Off 는 버림
                if self._robot_on and last is not None:
                    M_last, raw_last, pub_last = last
                    jm, ja = self._T_delta(raw_last, self._mfo_raw_prev) if self._mfo_raw_prev is not None else (0.0, 0.0)
                    if jm <= float(g("lc_track_jump_m").value) and ja <= float(g("lc_track_jump_deg").value):
                        off = M_last @ _invert_transform(raw_last)
                        rm, ra = self._T_delta(pub_last, M_last)      # 마지막 프레임에서 로버가 옮겨지는 양
                        self._lc_off = off
                        self._robot_moves += 1
                        robot_txt = f" · 로버 위치 보정 {rm * 100:.2f} cm / {ra:.3f}°"
                    else:
                        robot_txt = " · 로버 보정 보류(track 이 그사이 바뀜)"
            self.get_logger().info(
                f"[lc_history{'' if small else ' #' + str(self._hist_n)}] 루프 클로저 {lc_count} · 프레임 {len(path.poses)}"
                f" (all_slam_poses {len(allp)}, 못 찾음 {n_miss}) · 실시간 대비 최대 {dmax * 100:.2f} cm / {amax:.3f}°"
                f" · get_all_slam_poses {ms_get:.0f} ms + 변환 {(time.monotonic() - t0) * 1000:.0f} ms"
                + (" → 작아서 안 냄" if small else "") + robot_txt)
        except Exception as exc:
            self.get_logger().error(f"lc_history 실패: {type(exc).__name__}: {exc}")
        finally:
            self._hist_busy = False

    def _publish_map_outputs(self, stamp: int, map_from_base: np.ndarray, map_from_odom: np.ndarray) -> None:
        sec, nsec = stamp // 1_000_000_000, stamp % 1_000_000_000
        m = Odometry()
        m.header.stamp.sec, m.header.stamp.nanosec = sec, nsec
        m.header.frame_id = self._map_frame_id
        m.child_frame_id = self._base_frame_id
        q = _matrix_to_quaternion(map_from_base[:3, :3])
        m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z = (float(v) for v in map_from_base[:3, 3])
        (m.pose.pose.orientation.x, m.pose.pose.orientation.y,
         m.pose.pose.orientation.z, m.pose.pose.orientation.w) = (float(v) for v in q)
        self._map_pose_pub.publish(m)
        # TF map->odom 은 _map_tf_timer 가 낸다 (self._map_from_odom)

    def _map_tf_timer(self) -> None:
        T = self._map_from_odom
        if self._map_tf_sent is not None:
            dm, da = self._T_delta(self._map_tf_sent, T)
            if dm < 0.001 and da < 0.05:             # 1 mm · 0.05° 안이면 그대로 (static 은 한 번 내면 남음)
                return
        self._map_tf_sent = T.copy()
        now = self.get_clock().now()
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id, t.child_frame_id = self._map_frame_id, self._odom_frame_id
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in T[:3, 3])
        qo = _matrix_to_quaternion(T[:3, :3])
        (t.transform.rotation.x, t.transform.rotation.y,
         t.transform.rotation.z, t.transform.rotation.w) = (float(v) for v in qo)
        self._map_tf.sendTransform(t)

    def _new_map_id(self) -> str:
        """저장 폴더 이름. <세션>/slam_map 처럼 이름이 slam_map 이면 세션 폴더 이름을 쓴다(세션마다 다르게)."""
        if not self._save_dir:
            return time.strftime("slam_%y%m%d_%H%M%S")
        d = self._save_dir.rstrip("/")
        base = os.path.basename(d)
        return os.path.basename(os.path.dirname(d)) if base == "slam_map" else base

    def save_slam_map(self, timeout_s: float = 15.0) -> str:
        """Slam 맵 + daron_slam_meta.json 저장. 반환: 결과 문자열."""
        if self._slam is None or not self._slam_started:
            return "Slam 미동작 — 저장 안 함"
        if not self._save_dir:
            return "slam_map_save_dir 비어 있음 — 저장 안 함"
        import threading
        done = threading.Event()
        res = {}

        def cb(ok, *a):
            res["ok"] = bool(ok)
            done.set()
        os.makedirs(self._save_dir, exist_ok=True)
        t0 = time.monotonic()
        self._slam.save_map(self._save_dir, cb)
        if not done.wait(timeout_s):
            return f"save_map 콜백 {timeout_s:.0f} s 안에 안 옴: {self._save_dir}"
        accepted = self._reloc == "ACCEPTED" and self._loaded_meta is not None
        meta = dict(
            map_id=(self._loaded_meta.get("map_id") if accepted else self._new_map_id()),
            saved_from_session=self._new_map_id(),
            # 다음 실행의 재추정 추정값 = 이 맵 좌표계에서의 출발 표시 rig 포즈. 재추정으로 이어 쓴 맵이면 원래 출발점을 물려받는다
            start_rig_map=(self._loaded_meta["start_rig_map"] if accepted
                           else [float(v) for v in self._start_rig_map.reshape(-1)]),
            parent_map_dir=(self._load_dir if accepted else ""),
            reloc_state=self._reloc, rig_convention="cv" if self._cv else "ros",
            lc_count=self._lc_count, created=time.strftime("%Y-%m-%dT%H:%M:%S"))
        with open(os.path.join(self._save_dir, "daron_slam_meta.json"), "w", encoding="utf-8") as f:
            json.dump(meta, f, ensure_ascii=False, indent=1)
        return f"saved ok={res.get('ok')} {self._save_dir} ({(time.monotonic() - t0) * 1000:.0f} ms, map_id={meta['map_id']})"

    def _srv_save_map(self, req, res):
        try:
            res.message = self.save_slam_map()
            res.success = res.message.startswith("saved ok=True")
        except Exception as exc:
            res.success, res.message = False, f"{type(exc).__name__}: {exc}"
        return res

    def _on_start_attitude(self, msg: QuaternionStamped) -> None:
        q = msg.quaternion
        R = _quaternion_to_matrix((q.x, q.y, q.z, q.w))
        if self._odom_from_world is not None:
            if self._start_R is None:
                self.get_logger().warning("start attitude 가 odom 원점을 잡은 뒤 도착 — 무시 (이번 실행은 항등 출발)")
            return
        self._start_R = R
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, float(R[2, 2])))))
        self.get_logger().info(f"start attitude 수신: 출발 기울기 {tilt:.2f}°")

    def _viz_range_filter(self, current_landmarks) -> set:
        """[viz] rig 프레임 랜드마크 중 norm(coords) <= viz_max_range 인 id 집합(near_ids).

        통과한 id 는 _viz_near_ids_ever 에 누적(landmarks_cloud 화이트리스트). 1초마다 로그."""
        near_ids = set()
        n_total = 0
        for lm in current_landmarks or ():
            n_total += 1
            c = lm.coords
            d = math.sqrt(float(c[0]) ** 2 + float(c[1]) ** 2 + float(c[2]) ** 2)
            if d <= self._viz_max_range:
                near_ids.add(int(lm.id))
        self._viz_near_ids_ever.update(near_ids)
        now = time.monotonic()
        if now - self._viz_range_last_log >= 1.0:
            self._viz_range_last_log = now
            pct = (100.0 * len(near_ids) / n_total) if n_total else 0.0
            self.get_logger().info(
                f"[viz] range filter: kept {len(near_ids)}/{n_total} ({pct:.0f}%), "
                f"max_range={self._viz_max_range:.1f}")
        return near_ids

    def _viz_publish_landmarks(self) -> None:
        """[viz] 누적 랜드마크(world 프레임) -> odom 프레임 점구름. 타이머(벽시계) 전용."""
        if self._tracker is None or self._odom_from_world is None:
            return
        t0 = time.perf_counter()
        try:
            landmarks = self._tracker.get_final_landmarks()
        except Exception as exc:
            self.get_logger().warning(f"[viz] get_final_landmarks failed: {exc}")
            return
        t1 = time.perf_counter()
        n_all = len(landmarks)
        # [viz] 거리 필터: 한 번이라도 near_ids 에 든 id 만 남긴다 (화이트리스트 누적).
        whitelist = self._viz_near_ids_ever
        kept = [coords for lid, coords in landmarks.items() if int(lid) in whitelist]
        n = len(kept)
        if n:
            p_world = np.fromiter(
                (c for coords in kept for c in coords),
                dtype=np.float64, count=3 * n).reshape(n, 3)
            if self._cv:
                p_world = p_world @ self._Q.T                       # [261008] cv 월드 -> ROS 축
            rotation = self._odom_from_world[:3, :3]
            translation = self._odom_from_world[:3, 3]
            p_odom = p_world @ rotation.T + translation
        else:
            p_odom = np.zeros((0, 3), dtype=np.float32)
        self._viz_landmarks_pub.publish(
            _make_cloud(p_odom, self._odom_frame_id, self._last_stamp))
        t2 = time.perf_counter()
        get_ms, pub_ms = (t1 - t0) * 1e3, (t2 - t1) * 1e3
        now = time.monotonic()
        if now - self._viz_landmark_last_log >= 10.0:
            self._viz_landmark_last_log = now
            self.get_logger().info(
                f"[viz] landmarks: {n}/{n_all} points (range whitelist), "
                f"get={get_ms:.1f}ms, pub={pub_ms:.1f}ms")
        if get_ms > VIZ_LANDMARK_SLOW_THRESHOLD_MS and not self._viz_landmark_slowed:
            self._viz_landmark_slowed = True
            self._viz_landmark_period = VIZ_LANDMARK_SLOW_PERIOD_S
            self._viz_landmark_timer.cancel()
            self._viz_landmark_timer = self.create_timer(
                self._viz_landmark_period, self._viz_publish_landmarks,
                clock=self._viz_wall_clock)
            self.get_logger().warning(
                f"[viz] landmarks: get_final_landmarks took {get_ms:.1f}ms "
                f"(> {VIZ_LANDMARK_SLOW_THRESHOLD_MS:.0f}ms, {n_all} points) -> "
                f"lowering {VIZ_LANDMARKS_TOPIC} to {1.0 / self._viz_landmark_period:.0f} Hz")

    def _viz_publish_observations(self, stamp: int, odom_from_rig: np.ndarray,
                                  current_landmarks, observations_left, near_ids: set) -> None:
        """[viz] 현재 프레임 랜드마크(rig 프레임) 중 왼쪽 관측 id 와 겹치는 것만 odom 으로 옮겨 발행.
        near_ids(거리 필터 통과 id) 밖의 것은 발행하지 않는다."""
        obs_ids = {int(o.id) for o in observations_left}
        matched = [lm for lm in current_landmarks if int(lm.id) in obs_ids]
        n_total = len(current_landmarks)
        if not matched and n_total:
            if not self._viz_obs_empty_warned:
                self._viz_obs_empty_warned = True
                self.get_logger().warning(
                    f"[viz] observations: no id overlap between get_last_observations(0) "
                    f"({len(obs_ids)}) and get_last_landmarks() ({n_total}); "
                    "publishing all current landmarks for such frames")
            matched = current_landmarks
        matched = [lm for lm in matched if int(lm.id) in near_ids]   # [viz] 거리 필터
        now = time.monotonic()
        if now - self._viz_obs_last_log >= 1.0:
            self._viz_obs_last_log = now
            pct = (100.0 * len(matched) / n_total) if n_total else 0.0
            self.get_logger().info(
                f"[viz] observations: matched {len(matched)}/{n_total} ({pct:.0f}%)")
        if matched:
            p_rig = np.array([[float(v) for v in lm.coords] for lm in matched], dtype=np.float64)
            if self._cv:
                p_rig = p_rig @ self._Q.T                           # [261008] cv rig -> ROS 축
            p_odom = p_rig @ odom_from_rig[:3, :3].T + odom_from_rig[:3, 3]
        else:
            p_odom = np.zeros((0, 3), dtype=np.float32)
        self._viz_observations_pub.publish(_make_cloud(p_odom, self._odom_frame_id, stamp))

    def _viz_publish_image(self, stamp: int, gray_left: np.ndarray, observations_left,
                           tracked: bool, near_ids: set) -> None:
        """[viz] 좌측 IR + 관측점 원(추적 중 초록 / lost 빨강 + 빨간 테두리) -> bgr8.
        원은 obs.id 가 near_ids(거리 필터 통과)에 있을 때만 그린다."""
        try:
            import cv2
        except ImportError:
            return
        img = cv2.cvtColor(gray_left, cv2.COLOR_GRAY2BGR)
        color = (0, 255, 0) if tracked else (0, 0, 255)
        for obs in observations_left or ():
            if int(obs.id) not in near_ids:
                continue
            cv2.circle(img, (int(round(float(obs.u))), int(round(float(obs.v)))), 3, color, 1,
                       lineType=cv2.LINE_AA)
        if not tracked:
            h, w = img.shape[:2]
            cv2.rectangle(img, (0, 0), (w - 1, h - 1), (0, 0, 255), 6)
        msg = Image()
        msg.header.stamp.sec = stamp // 1_000_000_000
        msg.header.stamp.nanosec = stamp % 1_000_000_000
        msg.header.frame_id = "camera_infra1_optical_frame"
        msg.height, msg.width = int(img.shape[0]), int(img.shape[1])
        msg.encoding = "bgr8"
        msg.is_bigendian = 0
        msg.step = msg.width * 3
        msg.data = array.array("B", img.tobytes())   # tobytes() 직접 대입은 ~17 ms/프레임
        self._viz_image_pub.publish(msg)

    def _viz_append_path(self, path: Path, publisher, odom_msg: Odometry, add_pose: bool) -> None:
        """[viz] Odometry pose 를 Path 에 누적(최대 VIZ_PATH_MAX_POSES, 넘으면 앞에서 삭제). 발행은 타이머."""
        if add_pose:
            ps = PoseStamped()
            ps.header = odom_msg.header
            ps.pose = odom_msg.pose.pose
            path.poses.append(ps)
            if len(path.poses) > VIZ_PATH_MAX_POSES:
                del path.poses[: len(path.poses) - VIZ_PATH_MAX_POSES]
        path.header.stamp = odom_msg.header.stamp
        self._viz_path_dirty[id(publisher)] = (path, publisher)   # 발행은 _viz_flush_paths (VIZ_PATH_PERIOD_S)

    def _viz_flush_paths(self) -> None:
        """[viz] 바뀐 Path 만 발행 (벽시계 타이머 — sim time 이 멈춰도 마지막 경로까지 나간다)."""
        dirty, self._viz_path_dirty = self._viz_path_dirty, {}
        for path, publisher in dirty.values():
            publisher.publish(path)

    def _viz_on_ekf_odom(self, msg: Odometry) -> None:
        """[viz] /odometry/filtered -> slam_path(융합 경로)."""
        self._viz_append_path(self._viz_slam_path, self._viz_slam_path_pub, msg, True)

    def _viz_publish_outage_marker(self, stamp: int, active: bool) -> None:
        """[viz] fault_inject 창 시작/끝 표시. 창 동안 텍스트(id 0), 시작/끝 지점에 구(id 1/2)."""
        if self._last_pose is None:
            return
        position, _ = self._last_pose
        base = Marker()
        base.header.stamp.sec = stamp // 1_000_000_000
        base.header.stamp.nanosec = stamp % 1_000_000_000
        base.header.frame_id = self._odom_frame_id
        base.ns = "outage"
        base.pose.orientation.w = 1.0
        base.pose.position.x = float(position[0])
        base.pose.position.y = float(position[1])
        base.pose.position.z = float(position[2])
        text = Marker()
        text.header = base.header
        text.ns = base.ns
        text.id = 0
        if active:
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose = copy.deepcopy(base.pose)   # base.pose 는 아래 구(dot)에도 쓰므로 복사
            text.pose.position.z = float(position[2]) + 0.4
            text.scale.z = 0.12
            text.color.r, text.color.g, text.color.b, text.color.a = 1.0, 0.3, 0.3, 1.0
            text.text = "visual updates withheld"
        else:
            text.action = Marker.DELETE
        self._viz_outage_pub.publish(text)
        dot = Marker()
        dot.header = base.header
        dot.ns = base.ns
        dot.id = 1 if active else 2
        dot.type = Marker.SPHERE
        dot.action = Marker.ADD
        dot.pose = base.pose
        dot.scale.x = dot.scale.y = dot.scale.z = 0.06
        dot.color.r, dot.color.g, dot.color.b, dot.color.a = (1.0, 0.3, 0.3, 1.0) if active else (0.3, 1.0, 0.3, 1.0)
        self._viz_outage_pub.publish(dot)

    @staticmethod
    def _stamp_ns(msg) -> int:
        return int(msg.header.stamp.sec) * 1_000_000_000 + int(
            msg.header.stamp.nanosec
        )

    def _set_info(self, index: int, msg: CameraInfo) -> None:
        self._info[index] = msg

    def _image(self, index: int, msg: Image) -> None:
        stamp = self._stamp_ns(msg)
        if stamp <= self._paired_stamp[index]:
            # [261016] 이미 짝지어 쓴 시각 이하 = 중복·지각 프레임 → 버린다. 예전에는 다시 짝지어져 _track 이
            # stamp+1 ns 로 cuVSLAM 에 넣었다(261015 WiFi bag: /slam/odometry 7.7 % 가 직전+1 ns, 정지 중 3.26·40.25 m 점프).
            self._stale_frames += 1
            self.get_logger().warning(
                f"stale/duplicate {'left' if index == 0 else 'right'} frame dropped "
                f"(stamp <= last paired, total={self._stale_frames})", throttle_duration_sec=5.0)
            return
        queue = self._left if index == 0 else self._right
        queue[stamp] = msg
        while len(queue) > 12:
            del queue[min(queue)]
        self._consume_pair()

    def _consume_pair(self) -> None:
        if not self._left or not self._right or any(v is None for v in self._info):
            return
        tolerance = int(
            float(self.get_parameter("sync_tolerance_ms").value) * 1_000_000
        )
        # [261016] 짝이 있는 가장 최신 쌍 하나만 쓰고 그 이하는 양쪽 다 버린다. 예전에는 가장 오래된 left 만 보고,
        # 짝이 없는데 right 맨 앞이 더 오래됐으면 아무것도 안 버려 left 큐 12 장이 찰 때까지 막혔다
        # (수신율 60 % 재현: 지연 평균 358 ms, 최대 967 ms — 'VO 지연 0.3~0.4 s' 의 상당 부분).
        rights = sorted(self._right)
        for left_stamp in sorted(self._left, reverse=True):
            i = bisect.bisect_left(rights, left_stamp)
            right_stamp = min(rights[max(0, i - 1):i + 1], key=lambda value: abs(value - left_stamp))
            if abs(right_stamp - left_stamp) <= tolerance:
                break
        else:
            return
        left, right = self._left[left_stamp], self._right[right_stamp]
        for queue, cut in ((self._left, left_stamp), (self._right, right_stamp)):
            for old in [s for s in queue if s <= cut]:
                del queue[old]
        self._paired_stamp = [left_stamp, right_stamp]
        stamp = max(left_stamp, right_stamp)
        self._pairs_received += 1
        if self._warmup_start_stamp is None:
            self._warmup_start_stamp = stamp
            if self._fault_windows:
                # 첫 스테레오 쌍 수신 시점의 노드 시계 = fault_inject_s 의 0 초. (warmup 이전이라
                # _track() 에서 잡으면 warmup_seconds 만큼 늦어진다 — 260915 실측 6 s 어긋남)
                now_ns = self.get_clock().now().nanoseconds
                self._fault_t0_ns = now_ns if now_ns > 0 else stamp
        elapsed = max(0.0, (stamp - self._warmup_start_stamp) * 1e-9)
        warmup = float(self.get_parameter("warmup_seconds").value)
        if elapsed < warmup:
            second = int(elapsed)
            if second != self._warmup_logged_second:
                self._warmup_logged_second = second
                self.get_logger().info(
                    f"Robot settling: {elapsed:.1f}/{warmup:.1f}s "
                    f"(discarded stereo pairs={self._pairs_received})"
                )
            return
        if self._tracker is None:
            self.get_logger().info(
                f"Robot settling complete after {elapsed:.1f}s; "
                "starting odometry from a fresh origin"
            )
        self._track(stamp, left, right)

    def _make_tracker(self):
        import pycuvslam as cuvslam

        half = float(self.get_parameter("baseline_m").value) * 0.5
        rotation = [0.5, -0.5, 0.5, -0.5]  # optical -> X-forward/Y-left/Z-up
        cameras = []
        for index, y in enumerate((half, -half)):
            info = self._info[index]
            width, height = int(info.width), int(info.height)
            fx, fy, cx, cy = (
                float(info.k[0]),
                float(info.k[4]),
                float(info.k[2]),
                float(info.k[5]),
            )
            camera = cuvslam.Camera()
            camera.size = (width, height)
            camera.focal = (fx, fy)
            camera.principal = (cx, cy)
            camera.distortion = cuvslam.Distortion(
                cuvslam.Distortion.Model.Pinhole
            )
            if self._cv:   # [261008] OpenCV 축 rig: 원점 = 두 카메라 중앙(ROS rig 와 같은 점), 왼쪽 x = -b/2
                camera.rig_from_camera = cuvslam.Pose(translation=[-y, 0.0, 0.0], rotation=[0.0, 0.0, 0.0, 1.0])
            else:
                camera.rig_from_camera = cuvslam.Pose(
                    translation=[0.0, y, 0.0], rotation=rotation
                )
            cameras.append(camera)
        rig = cuvslam.Rig()
        rig.cameras = cameras
        odometry = cuvslam.Odometry.Config(
            async_sba=False,
            enable_observations_export=True,
            enable_final_landmarks_export=True,
            rectified_stereo_camera=True,
            odometry_mode=cuvslam.Odometry.OdometryMode.Multicamera,
        )
        info0 = self._info[0]
        self.get_logger().info(
            f"cuVSLAM initialized: {info0.width}x{info0.height}, "
            f"fx={float(info0.k[0]):.2f}, fy={float(info0.k[4]):.2f}, "
            f"baseline={half * 2000:.1f} mm"
        )
        if self._rr is not None:
            try:
                intrinsics = np.array(
                    [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]]
                )
                for path in ("/world/rig/left", "/world/rig/right"):
                    self._rr.log(
                        path,
                        self._rr.Pinhole(
                            image_from_camera=intrinsics,
                            width=width,
                            height=height,
                            image_plane_distance=0.5,
                        ),
                        static=True,
                    )
            except Exception as exc:
                self.get_logger().warning(f"Rerun camera setup failed: {exc}")
        odom = cuvslam.Odometry(rig, odometry)
        if self._slam_on:
            self._slam = cuvslam.Slam(rig, odom.get_primary_cameras(),
                                      cuvslam.Slam.Config(sync_mode=bool(self.get_parameter("slam_sync").value)))
            self.get_logger().info(f"cuVSLAM Slam 생성 (primary={list(odom.get_primary_cameras())})")
        return odom

    def _build_mask(self, h: int, w: int) -> np.ndarray | None:
        base_ratio = float(self.get_parameter("mask_base_ratio").value)
        amplitude = float(self.get_parameter("mask_amplitude").value)
        period_frames = float(self.get_parameter("mask_period_frames").value)
        if base_ratio <= 0.0 and amplitude <= 0.0:
            return None
        if period_frames > 0.0:
            phase = 2.0 * np.pi * self._frames / period_frames
            ratio = base_ratio + amplitude * np.sin(phase)
        else:
            ratio = base_ratio
        ratio = float(np.clip(ratio, 0.0, 1.0))
        self._last_mask_ratio = ratio
        # 0 = usable, nonzero = excluded (cuvslam::sof mask convention, verified against
        # selection.cpp/selection_v2.cpp). Rebuilt every frame since ratio now moves
        # continuously with self._frames instead of being fixed for the process lifetime.
        mask = np.zeros((h, w), dtype=np.uint8)
        px = int(w * ratio)
        if px:
            mask[:, :px] = 1
            mask[:, w - px:] = 1
        return mask

    def _publish_status(self, stamp: int, tracked: bool, features_l: int, relock_jump=None) -> None:
        msg = String()
        msg.data = json.dumps({"stamp_ns": int(stamp), "tracked": bool(tracked),
                               "features_l": int(features_l),
                               "relock_jump_m": None if relock_jump is None else float(relock_jump)})
        self._status_pub.publish(msg)

    def _publish_odometry(self, stamp: int, position, rotation, tracked: bool) -> None:
        rotation = tuple(float(v) for v in rotation)
        msg = Odometry()
        msg.header.stamp.sec = stamp // 1_000_000_000
        msg.header.stamp.nanosec = stamp % 1_000_000_000
        msg.header.frame_id = self._odom_frame_id
        msg.child_frame_id = self._base_frame_id
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z = (
            float(v) for v in position
        )
        (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ) = (float(v) for v in rotation)
        pose_variance = TRACKED_VARIANCE if tracked else LOST_VARIANCE
        msg.pose.covariance = _diagonal_covariance(pose_variance, pose_variance)

        # Twist by finite difference against the previous published fix, expressed in
        # child_frame_id (body) as nav_msgs/Odometry requires: the world-frame position
        # delta is rotated by the conjugate of the current orientation, and the angular
        # rate comes from q_prev^-1 * q_now, which is already a body-frame delta.
        linear = (0.0, 0.0, 0.0)
        angular = (0.0, 0.0, 0.0)
        twist_variance = LOST_VARIANCE
        if tracked and self._twist_reference is not None:
            (
                reference_stamp,
                reference_position,
                reference_rotation,
            ) = self._twist_reference
            dt = (stamp - reference_stamp) * 1e-9
            if dt > 0.0:
                world_velocity = (
                    np.asarray(position, dtype=np.float64) - reference_position
                ) / dt
                linear = tuple(
                    _rotate_vector(_quaternion_conjugate(rotation), world_velocity)
                )
                angular = tuple(
                    _rotation_vector(
                        _quaternion_multiply(
                            _quaternion_conjugate(reference_rotation), rotation
                        )
                    )
                    / dt
                )
                twist_variance = TRACKED_VARIANCE
        msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z = (
            float(v) for v in linear
        )
        (
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ) = (float(v) for v in angular)
        msg.twist.covariance = _diagonal_covariance(twist_variance, twist_variance)
        self._odom_pub.publish(msg)
        self._publish_tf(msg)
        # [viz] VO 경로: 추적 중인 pose 만 누적(lost 프레임 = LOST_VARIANCE 재발행분은 제외), 발행은 매번.
        self._viz_append_path(self._viz_vo_path, self._viz_vo_path_pub, msg, tracked)

        if tracked:
            self._last_pose = (
                np.asarray(position, dtype=np.float64).copy(),
                rotation,
            )
            self._twist_reference = (
                stamp,
                np.asarray(position, dtype=np.float64).copy(),
                rotation,
            )
        else:
            # Drop the reference so the first fix after a dropout does not report a
            # velocity spanning the whole outage.
            self._twist_reference = None

    def _publish_tf(self, msg: Odometry) -> None:
        # 규약 §2-2: 최종 구성에서는 cuVSLAM이 map->odom만,
        # EKF가 odom->base_link를 발행한다. EKF 착수(9/6) 전까지
        # 임시로 odom_base를 쓴다. 둘을 동시에 켜면 TF가 진동한다.
        if self._tf_broadcaster is None:
            return
        transform = TransformStamped()
        transform.header.stamp = msg.header.stamp
        if self._tf_mode == "odom_base":
            transform.header.frame_id = self._odom_frame_id
            transform.child_frame_id = self._base_frame_id
        else:
            # map_odom: EKF 착수 전까지는 실행 경로가 아니다. EKF가 붙으면 여기서
            # 내보낼 값은 이 pose 자체가 아니라 보정항
            # T_map_odom = T_map_base * inv(T_odom_base)이고, T_odom_base는 EKF가
            # 발행한다. 그때 이 분기를 그렇게 바꿀 것.
            transform.header.frame_id = self._map_frame_id
            transform.child_frame_id = self._odom_frame_id
        transform.transform.translation.x = msg.pose.pose.position.x
        transform.transform.translation.y = msg.pose.pose.position.y
        transform.transform.translation.z = msg.pose.pose.position.z
        transform.transform.rotation = msg.pose.pose.orientation
        self._tf_broadcaster.sendTransform(transform)

    def _track(self, stamp: int, left: Image, right: Image) -> None:
        if self._tracker is None:
            self._tracker = self._make_tracker()
        if stamp <= self._last_stamp:
            # [261016] 예전 max(stamp, last+1) 은 처리한 영상을 +1 ns 로 다시 넣었다 — _consume_pair 가 단조 증가를
            # 보장하므로 여기 오면 버그. 같은 영상을 dt≈0 으로 다시 넣지 않는다.
            self.get_logger().error(f"non-increasing stamp {stamp} <= {self._last_stamp} — frame skipped")
            return
        self._last_stamp = stamp
        gray_left, gray_right = _gray(left), _gray(right)
        mask = self._build_mask(*gray_left.shape)
        masks = [mask, mask] if mask is not None else None
        if self._rr is not None:
            try:
                set_time = getattr(
                    self._rr,
                    "set_time_nanos",
                    getattr(self._rr, "set_time_ns", None),
                )
                if set_time is not None:
                    set_time("timestamp", stamp)
                self._rr.log("/world/features/left", self._rr.Image(gray_left))
                self._rr.log("/world/features/right", self._rr.Image(gray_right))
            except Exception as exc:
                self.get_logger().warning(f"Rerun image logging failed: {exc}")
                self._rr = None
        try:
            estimate = self._tracker.track(stamp, [gray_left, gray_right], masks=masks)
        except Exception as exc:
            self.get_logger().error(f"cuVSLAM track failed: {type(exc).__name__}: {exc}")
            return
        self._frames += 1
        fault = False
        if self._fault_windows:
            now_ns = self.get_clock().now().nanoseconds
            if self._fault_t0_ns is None:      # 방어용 — 정상 경로에서는 _consume_pair 에서 이미 잡힘
                self._fault_t0_ns = now_ns if now_ns > 0 else stamp
            t_rel = (now_ns - self._fault_t0_ns) * 1e-9
            fault = any(a <= t_rel < b for a, b in self._fault_windows)
            if fault != self._fault_active:
                self.get_logger().warning(
                    f"FAULT INJECT {'start' if fault else 'end'} at t={t_rel:.2f} s (frame={self._frames})")
                self._fault_active = fault
                self._viz_publish_outage_marker(stamp, fault)   # [viz] 창 시작/끝 표시
        if estimate.world_from_rig is None or fault:
            if self._frames % 30 == 0 and not fault:
                self.get_logger().warning(
                    f"ODOMETRY NOT LOCKED: tracked_frames={self._frames}, "
                    f"left_contrast_std={float(gray_left.std()):.1f}, "
                    f"right_contrast_std={float(gray_right.std()):.1f}"
                )
            if self._last_pose is not None:
                if not self._tracking_lost:
                    self.get_logger().warning(
                        f"TRACKING LOST at frame={self._frames}; holding last odom pose "
                        "with LOST_VARIANCE until relock")
                self._tracking_lost = True
                last_position, last_rotation = self._last_pose
                self._publish_odometry(stamp, last_position, last_rotation, False)
            # Before the first lock there is no origin yet, so a relative pose is
            # undefined and nothing is published at all.
            # [viz] 영상은 락 전/lost 에도 내보낸다(빨간 테두리). 관측은 있으면 그대로 그린다.
            try:
                lost_obs = self._tracker.get_last_observations(0)
                lost_landmarks = self._tracker.get_last_landmarks()
            except Exception:
                lost_obs, lost_landmarks = [], []
            near_ids = self._viz_range_filter(lost_landmarks)   # [viz] 조회만, 추정 무관
            self._publish_status(stamp, False, len(lost_obs))
            self._viz_publish_image(stamp, gray_left, lost_obs, False, near_ids)
            return
        pose = estimate.world_from_rig.pose
        world_from_rig = self._from_api(pose)                       # [261008] ROS 축 4x4 (cv rig 면 변환)
        position = world_from_rig[:3, 3].copy()
        rig_quat = _matrix_to_quaternion(world_from_rig[:3, :3])
        if self._first_pose is None:
            self._first_pose = position.copy()
            self.get_logger().info("ODOMETRY LOCKED; origin initialized")
        # rig 기준 상대 위치. Rerun의 궤적/landmark 표시 전용으로 그대로 둔다.
        relative_position = position - self._first_pose

        # 발행용 pose는 base_link 기준으로 옮긴다. 이것을 안 하면 244mm/8도만큼
        # 어긋난 채로 나가고, 로버가 회전할 때 레버암만큼 위치 오차가 더 생긴다.
        world_from_base = (
            world_from_rig
            @ self._rig_from_base
        )
        # 첫 락 시점의 base pose를 SE(3) 전체로 잡아 odom 원점으로 쓴다.
        #   T_odom_base(t) = inv(T_world_base(t0)) @ T_world_base(t)
        # translation만 빼면 자세가 world 절대값으로 남아 odom 축이 카메라 장착각만큼
        # 기울어진다 (2026-09-07 imu_straight_01: 평지 2m 직진인데 최종 x=+1.519,
        # z=-0.256m, atan(0.256/1.519)=9.57deg — URDF base_to_camera pitch -8.00deg와
        # 정합). EKF가 이 위치를 중력 기준 IMU 자세와 같은 프레임으로 섞으므로 필수.
        relock_jump = None
        if self._odom_from_world is None:
            # [261008] 출발 자세(중력 기준)를 기다린다 — 원점을 잡기 전 프레임은 발행하지 않는다(로버는 정지 중).
            if self._start_topic and self._start_R is None:
                if self._start_wait_from is None:
                    self._start_wait_from = stamp
                if stamp - self._start_wait_from < self._start_wait_ns:
                    if not self._start_waiting_logged:
                        self.get_logger().info(f"ODOMETRY LOCKED; {self._start_topic} 기다리는 중 (원점 보류)")
                        self._start_waiting_logged = True
                    self._publish_status(stamp, True, len(self._tracker.get_last_observations(0)))
                    return
                self.get_logger().error(
                    f"{self._start_topic} 를 {self._start_wait_ns * 1e-9:.1f} s 동안 못 받음 — 출발 자세 항등으로 시작 "
                    "(odom 이 출발 경사만큼 기울 수 있음. imu_recalib_relay mount_source:=tf 인지 확인)")
                self._start_R = np.eye(3)
            start = np.eye(4)
            if self._start_R is not None:
                start[:3, :3] = self._start_R
            self._odom_from_world = start @ _invert_transform(world_from_base)
            self._tracking_lost = False
        elif self._tracking_lost:
            # lost -> tracked edge. cuVSLAM may have restarted in a new world frame, so
            # re-anchor: the first relocked base pose must land exactly on the last
            # published odom pose (continuity, no jump into the EKF):
            #   T_odom_world_new = T_odom_base(last) @ inv(T_world_base_new)
            # Motion during the blackout is lost (unobservable here); the EKF keeps
            # integrating IMU over that gap on its own.
            last_position, last_rotation = self._last_pose
            # jump = what would have been published with the stale origin vs. last pose
            stale_position = (self._odom_from_world @ world_from_base)[:3, 3]
            jump = float(np.linalg.norm(stale_position - last_position))
            odom_from_base_last = _make_transform(
                _quaternion_to_matrix(last_rotation), last_position)
            self._odom_from_world = odom_from_base_last @ _invert_transform(world_from_base)
            self._tracking_lost = False
            self.get_logger().warning(f"RELOCK: re-anchored, jump={jump:.3f} m")
            relock_jump = jump
        odom_from_base = self._odom_from_world @ world_from_base
        base_relative_position = odom_from_base[:3, 3]
        base_rotation = _matrix_to_quaternion(odom_from_base[:3, :3])

        observations_left = self._tracker.get_last_observations(0)
        observations_right = self._tracker.get_last_observations(1)
        current_landmarks = self._tracker.get_last_landmarks()
        self._publish_odometry(stamp, base_relative_position, base_rotation, True)
        self._publish_status(stamp, True, len(observations_left), relock_jump)
        if self._slam is not None:
            self._slam_step(stamp, odom_from_base, gray_left, gray_right)
        # [viz] 현재 관측 랜드마크(rig 프레임) -> odom.  odom_from_rig = odom_from_base @ base_from_rig
        #       (= _odom_from_world @ world_from_rig; 위에서 만든 행렬 재사용, 새 추정 없음)
        near_ids = self._viz_range_filter(current_landmarks)   # [viz] 거리 필터(표시용)
        self._viz_publish_observations(
            stamp, odom_from_base @ self._base_from_rig, current_landmarks, observations_left,
            near_ids)
        self._viz_publish_image(stamp, gray_left, observations_left, True, near_ids)
        self._trajectory.append(relative_position.tolist())
        if self._rr is not None:
            try:
                for path, observations in (
                    ("/world/features/left/keypoints", observations_left),
                    ("/world/features/right/keypoints", observations_right),
                ):
                    self._rr.log(
                        path,
                        self._rr.Points2D(
                            [[float(obs.u), float(obs.v)] for obs in observations],
                            colors=[
                                _color_from_id(obs.id) for obs in observations
                            ],
                            radii=4.0,
                        ),
                    )
                quat = self._rr.Quaternion(xyzw=[float(v) for v in rig_quat])
                self._rr.log(
                    "/world/rig",
                    self._rr.Transform3D(
                        translation=relative_position.tolist(),
                        rotation=quat,
                    ),
                )
                if len(self._trajectory) >= 2:
                    self._rr.log(
                        "/world/trajectory",
                        self._rr.LineStrips3D(
                            [self._trajectory],
                            colors=[[0, 200, 255]],
                            radii=0.03,
                        ),
                    )
                if current_landmarks:
                    self._rr.log(
                        "/world/rig/current_features",
                        self._rr.Points3D(
                            [
                                [float(v) for v in landmark.coords]
                                for landmark in current_landmarks
                            ],
                            colors=[
                                _color_from_id(landmark.id)
                                for landmark in current_landmarks
                            ],
                            radii=0.025,
                        ),
                    )
                if self._frames % 10 == 0:
                    global_landmarks = self._tracker.get_final_landmarks()
                    if global_landmarks:
                        map_points = [
                            (
                                np.asarray(coords, dtype=np.float64)
                                - self._first_pose
                            ).tolist()
                            for coords in global_landmarks.values()
                        ]
                        self._rr.log(
                            "/world/map/features",
                            self._rr.Points3D(
                                map_points,
                                colors=[
                                    _color_from_id(identifier)
                                    for identifier in global_landmarks.keys()
                                ],
                                radii=0.025,
                            ),
                        )
            except Exception as exc:
                self.get_logger().warning(f"Rerun pose logging failed: {exc}")
        if self._frames % 30 == 0:
            landmark_log_path = self.get_parameter("landmark_log_path").value
            if landmark_log_path:
                try:
                    with open(landmark_log_path, "a") as f:
                        f.write(json.dumps({
                            "frame": self._frames,
                            "mask_ratio": self._last_mask_ratio,
                            "obs": [
                                [int(o.id), float(o.u), float(o.v)]
                                for o in observations_left
                            ],
                        }) + "\n")
                except Exception as exc:
                    self.get_logger().warning(f"landmark_log write failed: {exc}")
            landmark3d_log_path = self.get_parameter("landmark3d_log_path").value
            if landmark3d_log_path and self._frames % 300 == 0:
                # Throttled to every 300 frames (~10s): get_final_landmarks() is the full
                # cumulative map (tens of thousands of points in past runs) and dumping it
                # every 30-frame checkpoint would bloat the log for little extra insight.
                try:
                    global_landmarks = self._tracker.get_final_landmarks()
                    with open(landmark3d_log_path, "a") as f:
                        f.write(json.dumps({
                            "frame": self._frames,
                            "mask_ratio": self._last_mask_ratio,
                            "landmarks": [
                                [int(lid), float(c[0]), float(c[1]), float(c[2])]
                                for lid, c in global_landmarks.items()
                            ],
                        }) + "\n")
                except Exception as exc:
                    self.get_logger().warning(f"landmark3d_log write failed: {exc}")
            p = relative_position
            distance = float(np.linalg.norm(p))
            if observations_left:
                us = np.array([obs.u for obs in observations_left])
                vs = np.array([obs.v for obs in observations_left])
                spread = (
                    f"left_u_mean/std={us.mean():.1f}/{us.std():.1f}, "
                    f"left_v_mean/std={vs.mean():.1f}/{vs.std():.1f}, "
                    f"left_u_range=[{us.min():.0f},{us.max():.0f}], "
                    f"left_v_range=[{vs.min():.0f},{vs.max():.0f}], "
                )
            else:
                spread = "left_u_mean/std=n/a, left_v_mean/std=n/a, "
            self.get_logger().info(
                f"ODOMETRY OK: frame={self._frames}, "
                f"relative=({p[0]:+.3f},{p[1]:+.3f},{p[2]:+.3f})m, "
                f"distance_from_origin={distance:.3f}m, "
                f"features_L/R={len(observations_left)}/{len(observations_right)}, "
                f"map_points={len(self._tracker.get_final_landmarks())}, "
                f"mask_ratio={self._last_mask_ratio:.1%}, "
                f"left_contrast_std={float(gray_left.std()):.1f}, "
                f"right_contrast_std={float(gray_right.std()):.1f}, "
                f"{spread}"
                f"published={self._odom_topic}"
            )


def main():
    rclpy.init()
    node = D405CuVSLAM()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # Ctrl+C: rclpy 의 SIGINT 핸들러가 이미 context 를 shutdown 한 뒤 spin 이
        # 빠져나온다. 정상 종료이므로 삼키고 exit code 0 으로 나간다 — 예외로
        # 두면 launch 의 on_exit 로거가 rc=1 을 "프로세스 사망"으로 보고한다.
        pass
    finally:
        if node._slam is not None and node._save_dir:          # [261008] 종료 시 Slam 맵 저장
            try:
                print("[d405_cuvslam] " + node.save_slam_map(), flush=True)
            except Exception as exc:
                print(f"[d405_cuvslam] Slam 맵 저장 실패: {type(exc).__name__}: {exc}", flush=True)
        node.destroy_node()
        # 위 경로에서는 context 가 이미 내려가 있어 shutdown() 을 다시 부르면
        # RCLError(context already shutdown) 가 난다. 살아 있을 때만 부른다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
