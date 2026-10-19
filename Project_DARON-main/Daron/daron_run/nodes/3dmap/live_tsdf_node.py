#!/usr/bin/env python3
"""실시간 TSDF 매핑 노드 (ROS 2 Humble, rclpy).

입력: depth 이미지 + camera_info + nav_msgs/Odometry (cuVSLAM 또는 INS 융합) + /tf_static
출력: ~/map_cloud (PointCloud2 xyz, odom 프레임 — 주행가능성 격자 입력)
      발행은 로버 반경 publish_radius(기본 1.25 m = 판정 1 m + 판정 주변 참조 25 cm)만. 저장은 전체 (260929)
      ~/map_cloud_color (PointCloud2 xyzrgb — 거리색. RViz 표시용)
      ~/map_cloud_full (PointCloud2 xyzrgb 회색 — 전체 맵, full_publish_period 마다. RViz 표시 전용,
                        TRANSIENT_LOCAL. 반경 밖은 로버가 돌아오기 전엔 안 변하므로 느린 주기로 충분 (260930)
      ~/save_map 서비스 (mesh/cloud/traj 저장)
      ~/set_range_color 서비스 (std_srvs/SetBool — 거리색 토글)

[261008] backend — 저장 방식 선택 (포즈 게이트·키프레임·depth 전처리·발행 주기는 공통)
  tsdf  옛 동작. Open3D TSDF 에 지나온 영역 전체를 dense 로 누적, 2 s 마다 전체 추출 후 반경 crop.
  grid  그리드 피쳐맵(feature_grid.py). 5 cm 칸마다 cost 계산 축(높이 평균·분산·하/상단·점 수)만 누적.
        dense 전체 맵을 쌓지 않는다(교수님 10/8 지시 — dense 는 주행 뒤 bag 으로 오프라인 refine).
        ~/map_cells (PointCloud2: x y z zlo zhi var w np nf, 칸 중심·맵 좌표) — 판정 노드 input_mode:=cells 입력
        ~/map_cloud 는 칸 중심 (x, y, z평균) 점으로 계속 낸다(옛 판정 노드·RViz 호환).
        저장: map_<tag>_featuregrid.npz (+ 칸 중심 PLY, 궤적).

[261008 밤] map_frame (기본 '' = 옛 동작: odom 좌표로 쌓음)
  'map' 이면 키프레임 포즈를 T_map_odom(TF, 그 시각) · T_odom_base · T_base_depth 로 map 좌표에 쌓고 map 으로 발행한다.
  map->odom 은 cuVSLAM Slam(루프 클로저·재추정) 또는 slam:=false 면 항등 static TF.
  재추정 대기: /slam/reloc_status 가 확정(NEW_MAP/ACCEPTED/REJECTED/FAILED)될 때까지 적분하지 않는다 — 확정 전에 쌓으면
  재추정 성공 뒤 map 이 바뀌어 앞부분이 어긋난다. reloc_wait_s 동안 상태가 한 번도 안 오면(Slam 없음) 그냥 시작.
  feature_map_load (grid): ACCEPTED 면 이전 세션 피쳐맵(npz)을 불러와 이어서 쌓는다.
    'auto' = 재추정한 Slam 맵 폴더의 상위(세션 폴더)에서 가장 새 map_*_featuregrid.npz, 경로 = 그 파일, '' = 안 함.
    파일 meta 의 map_id 가 재추정 map_id 와 다르면 안 불러온다.

[261011] lc_resync (grid) — 루프 클로저 때 피쳐맵을 보정된 궤적으로 다시 쌓는다 (계산 kf_resync.py)
  왜: 칸 통계만 누적하므로 PGO 로 과거 포즈가 바뀌어도 쌓인 칸을 옮길 수 없었다. trav_obstacle 오프라인: 루프 클로저 8회,
      실시간 포즈(track)엔 보정 0 (cuVSLAM 이 1 % 미만 보정을 버림) / get_all_slam_poses 와 최대 2.1 cm·0.16° 차이.
  키프레임마다 솎은 depth(원본 uint16, zlib) + 적분 포즈 + 그때 map<-odom 을 보관한다(상한 lc_store_max_mb).
  cuVSLAM 노드가 /slam/map_odom_history(Path: 프레임마다 보정된 map<-odom)를 내면, 백그라운드에서
    T_wc_new = M_new(t) · M_used⁻¹ · T_wc_used 로 처음부터(원래 순서) 다시 쌓고 끝나면 통째로 바꾼다.
    재적분 중 새로 들어온 키프레임은 교체 직전 잠금 안에서 이어 쌓는다. 불러온 이전 세션 피쳐맵은 원재료가 없어 바탕으로 둔다.
  보정이 lc_resync_min_m·deg 보다 작으면 건너뜀. 궤적 파일(traj_tum)·npz 도 보정값으로 저장된다.
  웨이포인트·핀·로버 위치는 이 노드 밖이라 바뀌지 않는다.

의존: pip install open3d scipy  /  apt: ros-humble-sensor-msgs-py ros-humble-tf2-ros ros-humble-std-srvs
"""
import collections
import copy
import json
import os
import queue
import threading
import time

import numpy as np
import open3d as o3d
import rclpy
import tf2_ros
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from scipy.spatial.transform import Rotation as R, Slerp
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String
from std_srvs.srv import SetBool, Trigger

from depth_color import range_to_rgb
from feature_grid import FeatureGrid
import kf_resync as KR
from depth_filter import speckle_keep
from ground_filter import ground_connected_mask, height_clip_mask

_RGB_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
]
_GRAY = np.array([110, 110, 110], np.uint8)     # 거리값이 없는 점(범위 밖 / 시야 밖 / 거리색 끔)
# [261008] 그리드 피쳐맵 칸 메시지 필드 (전부 float32)
_CELL_KEYS = ("x", "y", "z", "zlo", "zhi", "var", "w", "np", "nf")
# [261011] 끝에 age = 칸 마지막 관측 stamp − header.stamp [s] (≤ 0). 지상국이 웨이포인트 기준 시각으로 씀(루프 클로저 재동기화).
#          float32 로 절대 시각을 못 담아 header 기준 차이로 낸다. 판정 노드 등은 필드 이름으로 읽어 영향 없음.
_CELL_FIELDS = [PointField(name=k, offset=4 * i, datatype=PointField.FLOAT32, count=1)
                for i, k in enumerate(_CELL_KEYS + ("age",))]


def pack_rgb(rgb):
    """(N,3) uint8 -> PointCloud2 rgb 필드용 float32"""
    rgb = np.asarray(rgb, np.uint32)
    return ((rgb[:, 0] << 16) | (rgb[:, 1] << 8) | rgb[:, 2]).astype(np.uint32).view(np.float32)


def write_range_ply(path, xyz, rgb, rng, comments):
    """x y z + red green blue + range_m(광축 거리) 를 담은 binary PLY.
    open3d 로 읽으면 xyz+색이 읽히고, range_m 은 plyfile 등으로 읽는다."""
    dt = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                   ("red", "u1"), ("green", "u1"), ("blue", "u1"), ("range_m", "<f4")])
    a = np.empty(len(xyz), dt)
    a["x"], a["y"], a["z"] = np.asarray(xyz, np.float32).T
    a["red"], a["green"], a["blue"] = np.asarray(rgb, np.uint8).T
    a["range_m"] = rng
    hdr = ("ply\nformat binary_little_endian 1.0\n"
           + "".join(f"comment {c}\n" for c in comments)
           + f"element vertex {len(a)}\n"
           + "property float x\nproperty float y\nproperty float z\n"
           + "property uchar red\nproperty uchar green\nproperty uchar blue\n"
           + "property float range_m\nend_header\n")
    with open(path, "wb") as f:
        f.write(hdr.encode("ascii"))
        f.write(a.tobytes())


def stamp_ns(h):
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


def tf_msg_to_T(tf):
    T = np.eye(4)
    q, t = tf.transform.rotation, tf.transform.translation
    T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
    T[:3, 3] = [t.x, t.y, t.z]
    return T


class LiveTsdf(Node):
    def __init__(self):
        super().__init__("live_tsdf")
        P = self.declare_parameter
        self.depth_topic = P("depth_topic", "/camera/camera/depth/image_rect_raw").value
        self.info_topic = P("info_topic", "/camera/camera/depth/camera_info").value
        self.odom_topic = P("odom_topic", "/slam/odometry").value
        self.depth_scale = P("depth_scale", 1e-3).value           # m per LSB
        self.voxel = P("voxel", 0.01).value
        self.dmin = P("dmin", 0.07).value
        # 이보다 먼 depth 픽셀은 적분 전에 버린다(integrate_loop). 원거리일수록 오차가 z² 로 커져
        # 구조물 가장자리에서 공중으로 뿌려지는 점이 많아지므로 1.0 m 로 제한 (260928: 1.2 -> 1.0)
        self.dmax = P("dmax", 1.0).value
        self.every = P("every", 2).value
        self.max_gap = int(P("max_gap_ms", 50.0).value * 1e6)
        self.max_wait = int(P("max_wait_ms", 500.0).value * 1e6)
        self.kf_trans = P("keyframe_trans", 0.03).value
        self.kf_rot = P("keyframe_rot_deg", 3.0).value
        self.cov_trace_max = P("cov_trace_max", 0.0).value         # 0 = 비활성
        self.pub_period = P("publish_period", 2.0).value
        self.out_dir = P("out_dir", os.path.expanduser("~/maps")).value
        # 종료(Ctrl+C) 시 자동 저장. false 면 파일을 안 만든다(save_map 서비스는 그대로 동작)
        self.save_on_exit = bool(P("save_on_exit", True).value)
        # 지면 연결 필터 (ground_filter.py). 발행 클라우드와 *_ground.ply 저장에 적용.
        # TSDF 볼륨 자체는 건드리지 않는다 — 원본 *_cloud.ply / *_mesh.ply 는 그대로 저장된다.
        self.gf_on = P("ground_filter", True).value
        self.gf_voxel = P("gf_voxel", 0.03).value
        self.gf_band = P("gf_seed_band", 0.10).value
        self.gf_win = P("gf_local_win", 1.0).value
        self.gf_last = None
        # 지면 위 높이 제한 [m] (ground_filter.height_clip_mask). 0 = 끔.
        # 발행 클라우드(RViz + 주행가능성 격자 입력)와 *_ground.ply 에 적용. TSDF·원본 저장은 그대로.
        # 켜면 실제 구조물 윗부분도 잘린다 — 260928 test_3dmap 분석 README §12-14
        self.max_height = float(P("max_height", 0.0, ParameterDescriptor(dynamic_typing=True)).value)
        self.hc_last = None
        # depth 고립 조각 제거 (depth_filter.py). 적분 **전에** 적용 — 한 번 TSDF 에 들어간
        # 공중 점은 나중 프레임이 지워주지 못하므로(truncation 밖) 입구에서 막는다.
        # 면적 기준 [m²]. 0 이면 끔. 260928: 25 cm² 로 스프레이 섬 전부 제거, 바닥 손실 0.3%
        # dynamic_typing: launch 에서 0 (INTEGER) 을 넘겨도 죽지 않게 (DOUBLE 선언에 INTEGER 는 거부됨)
        self.speckle_area = float(P("speckle_min_area", 0.0025,
                                    ParameterDescriptor(dynamic_typing=True)).value)
        # 거리색 (depth_color.py). depth 영상과 같은 turbo·같은 [dmin, dmax]·같은 거리 정의
        # (현재 카메라 광축 거리 Z). 켜진 상태에서 save_map 하면 *_range.ply 에 거리값이 있는
        # 점만 저장한다. 끄면 map_cloud_color 는 전부 회색, *_range.ply 는 안 만든다.
        # 토글: ros2 service call /live_tsdf/set_range_color std_srvs/srv/SetBool "{data: false}"
        #   또는 ros2 param set /live_tsdf range_color false
        self.range_on = bool(P("range_color", True).value)
        self.range_last = None
        mask_path = P("chassis_mask", "").value
        # ── [health 260929] 포즈 게이트 ─────────────────────────────────────────
        # TSDF 는 포즈 점프를 스스로 못 고친다(sdf_trunc 4 cm, 가중 상한 없음 — 진짜 면 뒤 4 cm 넘는 가짜 면은
        # 영영 안 지워진다). 그래서 점프가 맵에 들어가지 않게 막는다. health_topic 이 비면 게이트 끔(기존 동작).
        #   NORMAL  INS·SLAM 모두 OK 가 health_window_s 동안 유지 → odom_topic 포즈로 적분, 앵커 갱신
        #   ANCHOR  INS 가 OK 아님(또는 이벤트 직후) & SLAM OK → 앵커 EKF · 앵커 SLAM⁻¹ · 지금 SLAM 으로 적분
        #           (health_monitor 재정렬 목표와 같은 식 — 기존 맵과 이어진다)
        #   HOLD    SLAM DEGRADED → 키프레임을 모아 두고 SLAM OK 가 되면 ANCHOR 포즈로 적분, FAIL 이면 버림
        #   SLAM FAIL(LOST) 프레임은 버린다 — 그때 EKF 포즈는 IMU 추측항법이라 맵에 넣을 수 없다.
        self.health_topic = P("health_topic", "").value
        self.slam_topic = P("slam_topic", "/slam/odometry").value
        self.health_window = float(P("health_window_s", 30.0, ParameterDescriptor(dynamic_typing=True)).value)
        self.hold_max = int(P("hold_max", 120).value)     # 키프레임 수 (≈ 0.8 MB/장)
        # 첫 odom 뒤 이 시간은 적분 안 함 — EKF 출력 첫 ~2 s 는 원점에서 최대 0.9 m 떠 있다(260929, bag 11개)
        self.warmup_ns = int(float(P("warmup_s", 2.0, ParameterDescriptor(dynamic_typing=True)).value) * 1e9)
        # 발행 반경 [m]. 0 = 전체 발행(기존). 맵이 커지면 전체 추출·전송이 맵 크기에 비례해 느려진다
        self.pub_radius = float(P("publish_radius", 1.25, ParameterDescriptor(dynamic_typing=True)).value)
        # [260930] 전체 맵 표시 주기 [s]. 0 = 끔. publish_radius > 0 일 때만 의미 있다(0 이면 map_cloud 가 이미 전체).
        # TSDF 가 바뀌는 곳은 카메라 시야 dmax 안뿐 — 반경 밖은 고정이라 매번 다시 보낼 필요가 없다.
        # 반경 안은 map_cloud_color 가 2 s 마다 덮어 그리므로 전체 맵이 늦어도 로버 주변은 최신이다.
        self.full_period = float(P("full_publish_period", 30.0, ParameterDescriptor(dynamic_typing=True)).value)
        # [261008] 저장 방식 tsdf | grid (상단 docstring)
        self.backend = str(P("backend", "tsdf").value).strip().lower()
        if self.backend not in ("tsdf", "grid"):
            raise ValueError(f"backend 는 tsdf/grid 여야 합니다: {self.backend!r}")
        self.grid_cell = float(P("grid_cell", 0.025, ParameterDescriptor(dynamic_typing=True)).value)
        self.grid_stride = max(1, int(P("grid_stride", 2).value))          # depth 픽셀 솎기 (2 → 1/4)
        self.grid_w_max = float(P("grid_w_max", 2000.0, ParameterDescriptor(dynamic_typing=True)).value)
        self.grid_d_ref = float(P("grid_d_ref", 0.5, ParameterDescriptor(dynamic_typing=True)).value)
        self.grid_min_np = int(P("grid_min_np", 5).value)                   # 발행할 칸의 최소 점 수
        # [261008 밤] map 좌표계 · 재추정 대기 · 피쳐맵 불러오기 (상단 docstring)
        self.map_frame = str(P("map_frame", "").value).strip()
        self.reloc_topic = str(P("reloc_topic", "/slam/reloc_status").value)
        self.reloc_wait_ns = int(float(P("reloc_wait_s", 20.0, ParameterDescriptor(dynamic_typing=True)).value) * 1e9)
        self.fm_load = str(P("feature_map_load", "auto").value).strip()
        # 판정 노드가 맵 전체를 판정할 때(judge_extent:=map) 칸은 전부 보낸다. 0 = 전부, >0 = 반경 [m], <0 = publish_radius
        self.cells_radius = float(P("cells_radius", -1.0, ParameterDescriptor(dynamic_typing=True)).value)
        # [261011] 루프 클로저 재동기화 (상단 docstring)
        self.lc_resync = bool(P("lc_resync", False).value)
        self.lc_history_topic = str(P("lc_history_topic", "/slam/map_odom_history").value)
        self.lc_tol_ns = int(float(P("lc_resync_tol_s", 0.1, ParameterDescriptor(dynamic_typing=True)).value) * 1e9)
        self.lc_min_m = float(P("lc_resync_min_m", 0.002, ParameterDescriptor(dynamic_typing=True)).value)
        self.lc_min_deg = float(P("lc_resync_min_deg", 0.05, ParameterDescriptor(dynamic_typing=True)).value)
        self.lc_store_mb = float(P("lc_store_max_mb", 1500.0, ParameterDescriptor(dynamic_typing=True)).value)
        self.reloc = None                                # 마지막 /slam/reloc_status (dict)
        self.map_id = ""
        self.T_mo_last = np.eye(4)                       # 마지막으로 쓴 T_map_odom
        self.fm_loaded = None

        self.mask = None
        if mask_path:
            m = np.asarray(o3d.io.read_image(mask_path))
            self.mask = (m[..., 0] if m.ndim == 3 else m) > 0

        self.vol = self.fg = None
        if self.backend == "tsdf":
            self.vol = o3d.pipelines.integration.ScalableTSDFVolume(
                voxel_length=self.voxel, sdf_trunc=4 * self.voxel,
                color_type=o3d.pipelines.integration.TSDFVolumeColorType.NoColor)
        else:
            self.fg = FeatureGrid(cell=self.grid_cell, d_ref=self.grid_d_ref, w_max=self.grid_w_max)
        self.grid_last = None                            # (적분 ms, 칸 수)
        self.lock = threading.Lock()
        # [261011] 루프 클로저 재동기화 — 키프레임 원재료 · 불러온 바탕 맵 · 마지막 보정 이력
        if self.lc_resync and (self.fg is None or not self.map_frame):
            self.get_logger().warning("lc_resync 는 backend=grid · map_frame 지정일 때만 — 끔")
            self.lc_resync = False
        self.kf_store = KR.KeyframeStore(self.lc_store_mb * 1e6) if self.lc_resync else None
        self.fg_base = None                              # 불러온 이전 세션 피쳐맵 (원재료 없음 → 재적분 바탕)
        self.lc_hist = None
        self.lc_evt = threading.Event()
        self.lc_count = 0
        self.lc_last = None                              # 마지막 재동기화 요약 (report 용)
        self.lc_full_warned = False
        self.odom = collections.deque(maxlen=4000)    # (t_ns, p(3), q(4), valid)
        self.pending = collections.deque(maxlen=60)   # (t_ns, depth_u16, frame_id, recv_mono_ns)
        self.work = queue.Queue(maxsize=4)
        self.K = self.intr = None
        self.img_wh = None
        self.base = self.world = None
        self.T_base_depth = None
        self.T_kf = None
        self.traj = []
        self.stats = collections.Counter()
        self.depth_count = 0
        self.health = None                               # 최근 /health/state (dict)
        self.event_until = -1.0                          # [s] 이 시각까지는 '이벤트 직후'
        self.anchor = None                               # (T_ob_odom, T_ob_slam) 마지막 NORMAL 프레임
        self.hold = collections.deque(maxlen=max(1, self.hold_max))   # (t, depth, T_ob_slam)
        self.recover_q = collections.deque()             # HOLD 해제 뒤 적분 대기 (t, depth, T_wc, T_mo)
        self.odom_t0 = None
        self.pose_mode = "NORMAL"
        self.slam_odom = (collections.deque(maxlen=4000)
                          if self.health_topic and self.slam_topic != self.odom_topic else None)

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)

        self.create_subscription(CameraInfo, self.info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 50)
        self.create_subscription(Image, self.depth_topic, self.on_depth, qos_profile_sensor_data)
        if self.map_frame:
            from rclpy.qos import HistoryPolicy, ReliabilityPolicy
            self.create_subscription(String, self.reloc_topic, self.on_reloc,
                                     QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                                reliability=ReliabilityPolicy.RELIABLE,
                                                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if self.health_topic:
            self.create_subscription(String, self.health_topic, self.on_health, 10)
            if self.slam_odom is not None:
                self.create_subscription(Odometry, self.slam_topic, self.on_slam_odom, 50)
        if self.lc_resync:
            from rclpy.qos import HistoryPolicy, ReliabilityPolicy
            self.create_subscription(Path, self.lc_history_topic, self.on_lc_history,
                                     QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                                reliability=ReliabilityPolicy.RELIABLE,
                                                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.pub = self.create_publisher(PointCloud2, "~/map_cloud", 1)
        self.pub_color = self.create_publisher(PointCloud2, "~/map_cloud_color", 1)
        self.pub_cells = self.create_publisher(PointCloud2, "~/map_cells", 1) if self.fg is not None else None
        # 전체 맵: RViz 를 나중에 켜도 마지막 것을 바로 받게 TRANSIENT_LOCAL (RViz 쪽 Durability 도 맞춰야 함)
        self.pub_full = self.create_publisher(
            PointCloud2, "~/map_cloud_full",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.full_on = self.full_period > 0 and self.pub_radius > 0
        self.full_next = 0.0                              # time.monotonic() 기준 다음 전체 발행 시각
        self.full_q = queue.Queue(maxsize=1)              # (pts, stamp) — 지면필터가 전체 맵에 돌아 느리므로 별도 스레드
        self.full_last = None
        self.extract_last = None
        self.create_service(Trigger, "~/save_map", self.on_save)
        self.create_service(SetBool, "~/set_range_color", self.on_set_range_color)
        self.add_on_set_parameters_callback(self.on_params)
        self.create_timer(0.02, self.process_pending)
        self.create_timer(self.pub_period, self.publish_cloud)
        self.create_timer(5.0, self.report)

        self.worker = threading.Thread(target=self.integrate_loop, daemon=True)
        self.worker.start()
        if self.full_on:
            threading.Thread(target=self.full_loop, daemon=True).start()
        if self.lc_resync:
            threading.Thread(target=self.resync_loop, daemon=True).start()
        self.get_logger().info(f"odom={self.odom_topic} depth={self.depth_topic} scale={self.depth_scale} "
                               f"backend={self.backend}"
                               + (f" (cell {self.grid_cell} m, stride {self.grid_stride}, w_max {self.grid_w_max})"
                                  if self.fg is not None else ""))
        self.get_logger().info(
            f"health gate={'on ' + self.health_topic + ' (slam=' + self.slam_topic + ')' if self.health_topic else 'off'} "
            f"warmup={self.warmup_ns * 1e-9:g}s publish_radius={self.pub_radius:g}m "
            f"full_map={'every %gs' % self.full_period if self.full_on else 'off'}")
        self.get_logger().info(
            f"lc_resync={'on ' + self.lc_history_topic if self.lc_resync else 'off'}"
            + (f" (원재료 상한 {self.lc_store_mb:g} MB, 최소 보정 {self.lc_min_m * 100:g} cm/{self.lc_min_deg:g}°)"
               if self.lc_resync else ""))

    # ---------------- callbacks ----------------
    def on_info(self, m):
        if self.K is None:
            self.K = np.array(m.k, dtype=np.float64).reshape(3, 3)
            self.img_wh = (int(m.width), int(m.height))
            self.intr = o3d.camera.PinholeCameraIntrinsic(
                m.width, m.height, self.K[0, 0], self.K[1, 1], self.K[0, 2], self.K[1, 2])
            self.get_logger().info(f"K=\n{self.K}")

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        valid = True
        if self.cov_trace_max > 0:
            valid = float(np.trace(np.array(m.pose.covariance).reshape(6, 6))) <= self.cov_trace_max
        self.odom.append((stamp_ns(m.header), np.array([p.x, p.y, p.z]),
                          np.array([q.x, q.y, q.z, q.w]), valid))
        if self.odom_t0 is None:
            self.odom_t0 = stamp_ns(m.header)
        if self.base is None:
            self.base, self.world = m.child_frame_id, m.header.frame_id
            self.get_logger().info(f"odom frames: {self.world} -> {self.base}")

    # ---------------- [261008 밤] map 좌표계 ----------------
    @property
    def out_frame(self):
        return self.map_frame or self.world

    FINAL_RELOC = ("NEW_MAP", "ACCEPTED", "REJECTED", "FAILED")

    def on_reloc(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        prev = (self.reloc or {}).get("state")
        self.reloc = d
        self.map_id = str(d.get("map_id", "") or self.map_id)
        if d.get("state") != prev:
            self.get_logger().info(f"재추정 상태 {prev} -> {d.get('state')} (map_id={self.map_id})")
        if d.get("state") == "ACCEPTED" and self.fg is not None and self.fm_loaded is None:
            self._load_feature_map(d)

    def _load_feature_map(self, d):
        """ACCEPTED: 이전 세션 피쳐맵을 불러온다 (적분 시작 전에만)."""
        self.fm_loaded = ""
        if not self.fm_load:
            return
        path = self.fm_load
        if path == "auto":
            import glob
            sess = os.path.dirname(str(d.get("map_dir", "")).rstrip("/"))
            files = sorted(glob.glob(os.path.join(sess, "map_*_featuregrid.npz")))
            if not files:
                self.get_logger().warning(f"피쳐맵 불러오기: {sess} 에 map_*_featuregrid.npz 없음 — 새로 쌓음")
                return
            path = files[-1]
        try:
            g = FeatureGrid.load(path)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"피쳐맵 불러오기 실패 {path}: {e}")
            return
        mid = str(g.meta.get("map_id", ""))
        if mid and self.map_id and mid != self.map_id:
            self.get_logger().warning(f"피쳐맵 map_id {mid} != 재추정 {self.map_id} — 안 불러옴 ({path})")
            return
        if abs(g.cell - self.grid_cell) > 1e-9:
            self.get_logger().warning(f"피쳐맵 칸 {g.cell} != grid_cell {self.grid_cell} — 안 불러옴")
            return
        with self.lock:
            if self.stats["integrated"] > 0:
                self.get_logger().warning("이미 적분을 시작해 피쳐맵을 안 불러옴")
                return
            self.fg = g
            if self.lc_resync:
                self.fg_base = copy.deepcopy(g)          # 재적분 바탕 (원재료 없음)
        self.fm_loaded = path
        self.get_logger().info(f"피쳐맵 불러옴: {path} (칸 {g.n_cells()}, 프레임 {g.n_frames}, map_id={mid})")

    def _map_ready(self, t):
        """재추정이 확정됐거나, Slam 이 없는 것으로 보일 때 True."""
        if not self.map_frame:
            return True
        st = (self.reloc or {}).get("state")
        if st in self.FINAL_RELOC:
            return True
        if st is None and self.odom_t0 is not None and t - self.odom_t0 > self.reloc_wait_ns:
            if not getattr(self, "_reloc_timeout_logged", False):
                self._reloc_timeout_logged = True
                self.get_logger().warning(f"{self.reloc_topic} 를 {self.reloc_wait_ns * 1e-9:.0f} s 동안 못 받음 — "
                                          "Slam 없음으로 보고 적분 시작 (map->odom 은 TF 그대로)")
            return True
        return False

    def _T_map_odom(self, t_ns):
        """TF map<-odom (그 시각, 없으면 최신). 실패하면 None."""
        if not self.map_frame or self.map_frame == self.world:
            return np.eye(4)
        for tt in (Time(nanoseconds=int(t_ns)), Time()):
            try:
                tf = self.tf_buf.lookup_transform(self.map_frame, self.world, tt)
                self.T_mo_last = tf_msg_to_T(tf)
                return self.T_mo_last
            except Exception:  # noqa: BLE001
                continue
        return None

    def on_depth(self, m):
        self.depth_count += 1
        if self.depth_count % self.every:
            return
        if m.encoding not in ("16UC1", "mono16"):
            self.get_logger().error(f"depth encoding {m.encoding} 미지원")
            return
        d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, : m.width].copy()
        self.pending.append((stamp_ns(m.header), d, m.header.frame_id, time.monotonic_ns()))

    # ---------------- pose ----------------
    def on_health(self, m):
        try:
            self.health = json.loads(m.data)
        except ValueError:
            pass

    def on_slam_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.slam_odom.append((stamp_ns(m.header), np.array([p.x, p.y, p.z]),
                               np.array([q.x, q.y, q.z, q.w]), m.pose.covariance[0] < 1.0))

    def interp(self, t, od=None):
        od = self.odom if od is None else od
        for i in range(len(od) - 1, 0, -1):
            t1 = od[i][0]
            t0 = od[i - 1][0]
            if t0 <= t <= t1:
                # [261001] stamp 가 샘플과 정확히 같으면 보간이 아니므로 간격 검사를 안 한다.
                # 포즈가 프레임마다 같은 stamp 로 오는 오프라인 궤적(프레임 드롭 26%, 20260922_170303)에서
                # 드롭 직후 프레임이 'gap' 으로 다 버려지던 것. 30 Hz EKF odom 에선 거의 안 걸린다.
                for j in (i, i - 1):
                    if od[j][0] == t:
                        if not od[j][3]:
                            return None, "cov"
                        T = np.eye(4)
                        T[:3, :3] = R.from_quat(od[j][2]).as_matrix()
                        T[:3, 3] = od[j][1]
                        return T, "ok"
                if t1 - t0 > self.max_gap or t1 == t0:
                    return None, "gap"
                if not (od[i][3] and od[i - 1][3]):
                    return None, "cov"
                a = (t - t0) / (t1 - t0)
                rot = Slerp([0.0, 1.0], R.from_quat([od[i - 1][2], od[i][2]]))([a])
                T = np.eye(4)
                T[:3, :3] = rot.as_matrix()[0]
                T[:3, 3] = (1 - a) * od[i - 1][1] + a * od[i][1]
                return T, "ok"
            if t1 < t:
                break
        return None, "range"

    # ---------------- [health] 포즈 게이트 ----------------
    def _anchor_pose(self, T_s):
        A_e, A_s = self.anchor
        return A_e @ np.linalg.inv(A_s) @ T_s

    def gate_pose(self, t, d, T_ob):
        """이 depth 프레임에 쓸 포즈 (T_odom_base) 또는 None(보류·폐기). 상단 [health] 주석의 상태표."""
        h = self.health
        ins = h.get("ins", "OK") if h else "OK"
        slam = h.get("slam", "OK") if h else "OK"
        now = t * 1e-9
        if ins != "OK" or slam != "OK":
            self.event_until = now + self.health_window
        if slam == "FAIL":
            if self.hold:
                self.stats["hold_drop"] += len(self.hold)
                self.hold.clear()
            self._set_mode("FAIL_DROP")
            self.stats["skip_slam_fail"] += 1
            return None
        if self.slam_odom is None:
            T_s = T_ob                                   # odom_topic 이 곧 SLAM
        else:
            T_s, why = self.interp(t, self.slam_odom)
            if T_s is None:
                self.stats[f"skip_slam_{why}"] += 1
                return None
        if ins == "OK" and slam == "OK" and now >= self.event_until:
            self.anchor = (T_ob, T_s)
            self._set_mode("NORMAL")
            return T_ob
        if self.anchor is None:                          # 정상 구간이 한 번도 없었다 — 앵커 없이 그대로
            self._set_mode("NO_ANCHOR")
            return T_ob
        if slam == "DEGRADED":
            # 버퍼도 키프레임만 (depth 한 장 ≈ 0.8 MB). 판정 포즈는 나중에 쓸 앵커 포즈와 같은 것
            cand = self._anchor_pose(T_s) @ self.T_base_depth
            if not self.is_keyframe(cand):
                self.stats["not_kf"] += 1
                return None
            self.T_kf = cand
            if len(self.hold) == self.hold.maxlen:
                self.stats["hold_overflow"] += 1
            self.hold.append((t, d, T_s))
            self._set_mode("HOLD")
            return None
        # SLAM OK — 모아 둔 게 있으면 앵커 포즈로 풀어 준다
        while self.hold:
            ht, hd, hT_s = self.hold.popleft()
            # [261011] 4번째 = 적분 때 쓴 map<-odom (HOLD 해제분은 지금까지처럼 map<-odom 을 안 곱함 → 항등)
            self.recover_q.append((ht, hd, self._anchor_pose(hT_s) @ self.T_base_depth, np.eye(4)))
            self.stats["hold_released"] += 1
        self._set_mode("ANCHOR")
        self.stats["anchor_pose"] += 1
        return self._anchor_pose(T_s)

    def _set_mode(self, mode):
        if mode != self.pose_mode:
            self.get_logger().info(f"맵 포즈 모드 {self.pose_mode} → {mode} "
                                   f"(health {self.health.get('ins') if self.health else '-'}/"
                                   f"{self.health.get('slam') if self.health else '-'})")
            self.pose_mode = mode

    def is_keyframe(self, T):
        if self.T_kf is None:
            return True
        d = np.linalg.inv(self.T_kf) @ T
        rot = np.degrees(np.linalg.norm(R.from_matrix(d[:3, :3]).as_rotvec()))
        return np.linalg.norm(d[:3, 3]) > self.kf_trans or rot > self.kf_rot

    def process_pending(self):
        if self.intr is None or not self.odom:
            return
        latest = self.odom[-1][0]
        while self.pending:
            t, d, frame, recv = self.pending[0]
            if latest < t:                              # 해당 시각 odom 미도착 -> 대기
                if time.monotonic_ns() - recv < self.max_wait:
                    return
                self.pending.popleft()                  # max_wait 초과 -> 폐기
                self.stats["drop_wait"] += 1
                continue
            self.pending.popleft()
            if self.T_base_depth is None:
                try:
                    tf = self.tf_buf.lookup_transform(self.base, frame, Time())
                    self.T_base_depth = tf_msg_to_T(tf)
                    self.get_logger().info(f"T_{self.base}_{frame}=\n{np.round(self.T_base_depth, 4)}")
                except Exception as e:  # noqa: BLE001
                    self.get_logger().warn(f"TF {self.base}->{frame} 대기: {e}", throttle_duration_sec=2.0)
                    self.stats["drop_tf"] += 1
                    continue
            if self.odom_t0 is not None and t - self.odom_t0 < self.warmup_ns:
                self.stats["skip_warmup"] += 1
                continue
            T_ob, why = self.interp(t)
            if T_ob is None:
                self.stats[f"skip_{why}"] += 1
                continue
            if self.health_topic:
                T_ob = self.gate_pose(t, d, T_ob)
                if T_ob is None:
                    continue
            if not self._map_ready(t):                       # [261008 밤] 재추정 확정 전에는 안 쌓음
                self.stats["skip_reloc_wait"] += 1
                continue
            T_mo = self._T_map_odom(t)
            if T_mo is None:
                self.stats["skip_map_tf"] += 1
                continue
            T_wc = T_mo @ T_ob @ self.T_base_depth
            if not self.is_keyframe(T_wc):
                self.stats["not_kf"] += 1
                continue
            try:
                self.work.put_nowait((t, d, T_wc, T_mo))
                self.T_kf = T_wc
            except queue.Full:
                self.stats["drop_busy"] += 1

    # ---------------- integration ----------------
    def integrate_loop(self):
        while rclpy.ok():
            try:
                t, d, T_wc, T_mo = self.work.get(timeout=0.05)
            except queue.Empty:
                if not self.recover_q:
                    continue
                t, d, T_wc, T_mo = self.recover_q.popleft()  # HOLD 해제분 — 순서 무관(TSDF 가중 평균)
            dm = d.astype(np.float32) * self.depth_scale
            if self.mask is not None and self.mask.shape == dm.shape:
                dm[self.mask] = 0
            dm[(dm < self.dmin) | (dm > self.dmax)] = 0
            if self.speckle_area > 0:
                keep = speckle_keep(dm, self.K[0, 0], self.speckle_area)
                self.stats["speckle_px"] += int((dm > 0).sum() - keep.sum())
                dm[~keep] = 0
            if self.fg is not None:
                self._integrate_grid(t, dm, T_wc, d, T_mo)
                continue
            h, w = dm.shape
            rgbd = o3d.geometry.RGBDImage.create_from_color_and_depth(
                o3d.geometry.Image(np.zeros((h, w, 3), np.uint8)), o3d.geometry.Image(dm),
                depth_scale=1.0, depth_trunc=self.dmax, convert_rgb_to_intensity=False)
            with self.lock:
                self.vol.integrate(rgbd, self.intr, np.linalg.inv(T_wc))
                self.traj.append((t, T_wc))
            self.stats["integrated"] += 1

    def _integrate_grid(self, t, dm, T_wc, d_raw=None, T_mo=None):
        """[261008] depth(전처리 끝, m) → 맵 좌표 점 → 피쳐맵 칸 누적.
        [261011] lc_resync 면 같은 잠금 안에서 원재료(솎은 원본 depth, 전처리로 지운 화소 = 0)도 보관 — 맵과 보관 목록이 항상 같은 순서."""
        t0 = time.monotonic()
        k = self.grid_stride
        sub = dm[::k, ::k]
        pw = KR.points_from_sub(sub, self.K, k, T_wc)
        if pw is None:
            self.stats["grid_empty"] += 1
            return
        q = (np.where(sub > 0, d_raw[::k, ::k], 0).astype(np.uint16)
             if self.kf_store is not None and d_raw is not None else None)
        with self.lock:
            n = self.fg.integrate(pw, T_wc[:3, 3], t * 1e-9)
            self.traj.append((t, T_wc))
            if q is not None and not self.kf_store.add(t, q, T_wc, np.eye(4) if T_mo is None else T_mo):
                self.stats["lc_store_full"] += 1
        self.stats["integrated"] += 1
        self.grid_last = ((time.monotonic() - t0) * 1000, n)

    def _publish_grid(self):
        """[261008] grid: 반경 안 칸 → ~/map_cells + ~/map_cloud(칸 중심) + 색 클라우드, 전체는 full 주기로."""
        t0 = time.monotonic()
        center = None
        if self.odom:
            p_o = self.odom[-1][1]
            center = (self.T_mo_last[:3, :3] @ p_o + self.T_mo_last[:3, 3])[:2]   # map 좌표 (map_frame 없으면 항등)
        rad_cells = self.pub_radius if self.cells_radius < 0 else self.cells_radius
        with self.lock:
            c_all = self.fg.cells(center if rad_cells > 0 else None, rad_cells if center is not None else 0.0,
                                  min_np=self.grid_min_np)
            full = (self.fg.cells(min_np=self.grid_min_np)
                    if self.full_on and time.monotonic() >= self.full_next else None)
        self.extract_last = dict(n=len(c_all["x"]), ms=(time.monotonic() - t0) * 1000)
        if full is not None and len(full["x"]):
            if self.gf_on or self.max_height > 0:
                gf_keep = self.gf_last
                full = {k_: v_[self._cells_keep(full)] for k_, v_ in full.items()}   # 칸 단위 필터(복셀 = 칸)
                self.gf_last = gf_keep
            try:
                self.full_q.put_nowait((np.column_stack([full["x"], full["y"], full["z"]]).astype(np.float32),
                                        self.get_clock().now().to_msg()))
                self.full_next = time.monotonic() + self.full_period
            except queue.Full:
                self.stats["full_skip_busy"] += 1
        if len(c_all["x"]) and (self.gf_on or self.max_height > 0):
            t1 = time.monotonic()
            keep = self._cells_keep(c_all)
            c_all = {k_: v_[keep] for k_, v_ in c_all.items()}
            if self.gf_last is not None:
                self.gf_last["ms"] = (time.monotonic() - t1) * 1000
        hdr = Header(frame_id=self.out_frame, stamp=self.get_clock().now().to_msg())
        age = c_all["t"] - (hdr.stamp.sec + hdr.stamp.nanosec * 1e-9)
        cells = np.column_stack([c_all[k_] for k_ in _CELL_KEYS] + [age]).astype(np.float32)
        self.pub_cells.publish(point_cloud2.create_cloud(hdr, _CELL_FIELDS, cells))
        if center is not None and self.pub_radius > 0 and rad_cells != self.pub_radius:   # 표시·호환 클라우드는 반경만
            cells = cells[np.hypot(cells[:, 0] - center[0], cells[:, 1] - center[1]) <= self.pub_radius]
        pts = cells[:, :3]
        self.pub.publish(point_cloud2.create_cloud_xyz32(hdr, pts))
        rgb = np.tile(_GRAY, (len(pts), 1))
        if self.range_on and len(pts):
            r = self._range_color(pts)
            if r is not None:
                rgb[r["valid"]] = r["rgb"][r["valid"]]
                self.range_last = dict(n=int(r["valid"].sum()), total=len(pts))
        data = np.column_stack([pts.astype(np.float32), pack_rgb(rgb)]).astype(np.float32)
        self.pub_color.publish(point_cloud2.create_cloud(hdr, _RGB_FIELDS, data))

    def _cells_keep(self, c):
        """[261008] 칸 → 지면 연결 필터·높이 제한. 칸마다 zlo~zhi 를 칸 크기 간격 기둥 점으로 펼쳐
        (복셀 = 칸 크기라 이웃 칸이 26-연결로 이어진다) ground_connected_mask 에 넣고, 기둥 점이 하나라도
        남은 칸을 남긴다. 높이 제한은 칸 중심 높이로."""
        n = len(c["x"])
        keep = np.ones(n, bool)
        if self.gf_on:
            v = self.grid_cell
            lo, hi = np.minimum(c["zlo"], c["z"]), np.maximum(c["zhi"], c["z"])
            m = np.clip(np.ceil((hi - lo) / v).astype(np.int64) + 1, 1, 12)
            idx = np.repeat(np.arange(n), m)
            off = np.arange(len(idx)) - np.repeat(np.cumsum(m) - m, m)
            z = np.minimum(lo[idx] + off * v, hi[idx])
            P = np.column_stack([c["x"][idx], c["y"][idx], z])
            with self.lock:
                cams = np.array([T[:3, 3] for _, T in self.traj]) if self.traj else None
            k_pts, info = ground_connected_mask(P, voxel=v, seed_band=self.gf_band, local_win=self.gf_win,
                                                traj=cams, min_seed_voxels=12)
            keep = np.zeros(n, bool)
            keep[idx[k_pts]] = True
            info["removed"] = int(n - keep.sum())
            self.gf_last = info
        if self.max_height > 0:
            k2, self.hc_last = height_clip_mask(np.column_stack([c["x"], c["y"], c["z"]]), self.max_height)
            keep &= k2
        return keep

    # ---------------- output ----------------
    def publish_cloud(self):
        if self.world is None or self.stats["integrated"] == 0:
            return
        if self.fg is not None:
            self._publish_grid()
            return
        t0 = time.monotonic()
        with self.lock:
            pc = self.vol.extract_point_cloud()
        pc = pc.voxel_down_sample(2 * self.voxel)
        pts = np.asarray(pc.points, dtype=np.float32)
        # 전체 볼륨 추출 비용 (맵 크기에 비례 — 반경 발행이어도 여기는 전체다. 260930 이동 윈도 볼륨 판단용)
        self.extract_last = dict(n=len(pts), ms=(time.monotonic() - t0) * 1000)
        now = time.monotonic()
        if self.full_on and now >= self.full_next and len(pts):
            # 추출은 어차피 여기서 전체를 한다 — 그 결과를 넘기기만 한다. 앞 것이 아직 처리 중이면 이번은 건너뜀
            try:
                self.full_q.put_nowait((pts.copy(), self.get_clock().now().to_msg()))
                self.full_next = now + self.full_period
            except queue.Full:
                self.stats["full_skip_busy"] += 1
        if self.pub_radius > 0 and len(pts) and self.odom:
            # 로버(최신 포즈) 반경만 발행 — 저장(save)은 전체 볼륨 그대로
            c = (self.T_mo_last[:3, :3] @ self.odom[-1][1] + self.T_mo_last[:3, 3]).astype(np.float32)
            pts = pts[np.linalg.norm(pts - c, axis=1) <= self.pub_radius]
        if self.gf_on and len(pts):
            t0 = time.monotonic()
            keep, info = self._ground_mask(pts)
            pts = pts[keep]
            info["ms"] = (time.monotonic() - t0) * 1000
            self.gf_last = info
        if self.max_height > 0 and len(pts):
            keep, self.hc_last = height_clip_mask(pts, self.max_height)
            pts = pts[keep]
        hdr = Header(frame_id=self.out_frame, stamp=self.get_clock().now().to_msg())
        self.pub.publish(point_cloud2.create_cloud_xyz32(hdr, pts))

        rgb = np.tile(_GRAY, (len(pts), 1))
        if self.range_on and len(pts):
            r = self._range_color(pts)
            if r is not None:
                rgb[r["valid"]] = r["rgb"][r["valid"]]
                self.range_last = dict(n=int(r["valid"].sum()), total=len(pts))
        data = np.column_stack([pts.astype(np.float32), pack_rgb(rgb)]).astype(np.float32)
        self.pub_color.publish(point_cloud2.create_cloud(hdr, _RGB_FIELDS, data))

    def full_loop(self):
        """전체 맵 발행 (별도 스레드). 저장(save)과 같은 _keep_mask(지면 연결 + 높이 제한)를 쓴다.
        색은 회색 고정 — 반경 안은 map_cloud_color 가 거리색으로 겹쳐 그린다."""
        while rclpy.ok():
            pts, stamp = self.full_q.get()
            t0 = time.monotonic()
            if (self.gf_on or self.max_height > 0) and self.fg is None:   # grid 는 _publish_grid 에서 칸 단위로 이미 거름
                pts = pts[self._keep_mask(pts)]
            data = np.column_stack([pts, pack_rgb(np.tile(_GRAY, (len(pts), 1)))]).astype(np.float32)
            self.pub_full.publish(point_cloud2.create_cloud(
                Header(frame_id=self.out_frame, stamp=stamp), _RGB_FIELDS, data))
            self.full_last = dict(n=len(pts), ms=(time.monotonic() - t0) * 1000)

    # ---------------- 거리색 ----------------
    def _camera_pose_now(self):
        """최신 odom 기준 T_world_camera(depth 광학 프레임). 아직 없으면 None."""
        if not self.odom or self.T_base_depth is None:
            return None, None
        t, p, q, _ = self.odom[-1]
        T_ob = np.eye(4)
        T_ob[:3, :3] = R.from_quat(q).as_matrix()
        T_ob[:3, 3] = p
        return self.T_mo_last @ T_ob @ self.T_base_depth, t

    def _range_color(self, pts):
        """맵 점 -> 현재 카메라 광축 거리 Z 와 turbo 색.
        valid = depth 영상에서 그 점이 색을 가질 조건과 같다:
          카메라 앞(Z>0) & dmin<=Z<=dmax & 영상 안으로 투영됨.
        (가림은 따지지 않는다 — 벽 뒤 점도 시야 안이면 자기 Z 로 칠해진다)"""
        T_wc, t = self._camera_pose_now()
        if T_wc is None or self.K is None or self.img_wh is None:
            return None
        T_cw = np.linalg.inv(T_wc)
        pc = np.asarray(pts, np.float64) @ T_cw[:3, :3].T + T_cw[:3, 3]
        Z = pc[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            u = self.K[0, 0] * pc[:, 0] / Z + self.K[0, 2]
            v = self.K[1, 1] * pc[:, 1] / Z + self.K[1, 2]
        w, h = self.img_wh
        rgb, in_range = range_to_rgb(Z, self.dmin, self.dmax)
        valid = in_range & (Z > 0) & (u >= 0) & (u < w) & (v >= 0) & (v < h)
        return dict(rgb=rgb, valid=valid, Z=Z, T_wc=T_wc, t=t)

    def on_set_range_color(self, req, res):
        self.range_on = bool(req.data)
        res.success = True
        res.message = f"range_color = {self.range_on}"
        self.get_logger().info(res.message)
        return res

    def on_params(self, params):
        for p in params:
            if p.name == "range_color":
                self.range_on = bool(p.value)
                self.get_logger().info(f"range_color = {self.range_on}")
        return SetParametersResult(successful=True)

    def _ground_mask(self, pts):
        # 궤적 시드: 로버가 지나간 자리 아래 = 확실한 지면 (ground_filter.py 260928 사고 참고)
        with self.lock:
            cams = np.array([T[:3, 3] for _, T in self.traj]) if self.traj else None
        return ground_connected_mask(pts, voxel=self.gf_voxel, seed_band=self.gf_band,
                                     local_win=self.gf_win, traj=cams)

    def _keep_mask(self, pts):
        """저장용: 지면 연결 필터(켜져 있으면) & 높이 제한(켜져 있으면)"""
        keep = self._ground_mask(pts)[0] if self.gf_on else np.ones(len(pts), bool)
        if self.max_height > 0:
            keep &= height_clip_mask(pts, self.max_height)[0]
        return keep

    def save(self):
        os.makedirs(self.out_dir, exist_ok=True)
        tag = time.strftime("%Y%m%d_%H%M%S")
        base = os.path.join(self.out_dir, f"map_{tag}")
        if self.fg is not None:
            return self._save_grid(base)
        with self.lock:
            mesh = self.vol.extract_triangle_mesh()
            cloud = self.vol.extract_point_cloud()
            traj = list(self.traj)
        mesh.compute_vertex_normals()
        o3d.io.write_triangle_mesh(base + "_mesh.ply", mesh)
        o3d.io.write_point_cloud(base + "_cloud.ply", cloud)
        if self.gf_on or self.max_height > 0:
            keep = self._keep_mask(np.asarray(cloud.points))
            o3d.io.write_point_cloud(base + "_cloud_ground.ply", cloud.select_by_index(np.nonzero(keep)[0]))
            vkeep = self._keep_mask(np.asarray(mesh.vertices))
            gmesh = o3d.geometry.TriangleMesh(mesh)
            gmesh.remove_vertices_by_mask(~vkeep)
            o3d.io.write_triangle_mesh(base + "_mesh_ground.ply", gmesh)
        if self.range_on:
            pts = np.asarray(cloud.points)
            if len(pts):
                pts = pts[self._keep_mask(pts)]
            r = self._range_color(pts) if len(pts) else None
            if r is not None:
                v = r["valid"]
                Tw = r["T_wc"]
                write_range_ply(
                    base + "_range.ply", pts[v], r["rgb"][v], r["Z"][v].astype(np.float32),
                    comments=[
                        "DARON live_tsdf range-colored map (depth_color.range_to_rgb, turbo)",
                        f"range_m = optical-axis depth Z from camera at stamp_ns {r['t']}",
                        f"dmin {self.dmin} dmax {self.dmax} frame {self.out_frame}",
                        "T_world_camera " + " ".join(f"{x:.6f}" for x in Tw.ravel()),
                    ])
        with open(base + "_traj_tum.txt", "w") as f:
            for t, T in traj:
                q = R.from_matrix(T[:3, :3]).as_quat()
                f.write(f"{t * 1e-9:.9f} {T[0, 3]:.6f} {T[1, 3]:.6f} {T[2, 3]:.6f} "
                        f"{q[0]:.8f} {q[1]:.8f} {q[2]:.8f} {q[3]:.8f}\n")
        return base

    def _save_grid(self, base):
        """[261008] grid: 피쳐맵 npz + 칸 중심 PLY(보기용) + 궤적."""
        with self.lock:
            self.fg.save(base + "_featuregrid.npz", meta=dict(
                frame=self.out_frame, map_id=self.map_id, reloc=(self.reloc or {}).get("state", ""),
                loaded_from=self.fm_loaded or ""))
            c = self.fg.cells(min_np=self.grid_min_np)
            traj = list(self.traj)
        pc = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.column_stack([c["x"], c["y"], c["z"]])))
        o3d.io.write_point_cloud(base + "_cells.ply", pc)
        self._write_traj(base, traj)
        return base

    def _write_traj(self, base, traj):
        with open(base + "_traj_tum.txt", "w") as f:
            for t, T in traj:
                q = R.from_matrix(T[:3, :3]).as_quat()
                f.write(f"{t * 1e-9:.9f} {T[0, 3]:.6f} {T[1, 3]:.6f} {T[2, 3]:.6f} "
                        f"{q[0]:.8f} {q[1]:.8f} {q[2]:.8f} {q[3]:.8f}\n")

    # ---------------- [261011] 루프 클로저 재동기화 ----------------
    def on_lc_history(self, m):
        """/slam/map_odom_history: poses[i] = 프레임 stamp 의 보정된 map<-odom. 받기만 하고 계산은 resync_loop."""
        h = KR.history_from_path_msg(m)
        if h is not None:
            self.lc_hist = h
            self.lc_evt.set()

    def resync_loop(self):
        while rclpy.ok():
            if not self.lc_evt.wait(0.5):
                continue
            self.lc_evt.clear()
            hist = self.lc_hist
            if hist is None or self.K is None:
                continue
            try:
                self._resync(hist)
            except Exception as e:  # noqa: BLE001
                self.stats["lc_resync_error"] += 1
                self.get_logger().error(f"[LC 재동기화] 실패: {type(e).__name__}: {e}")

    def _new_grid(self):
        return FeatureGrid(cell=self.grid_cell, d_ref=self.grid_d_ref, w_max=self.grid_w_max)

    def _resync(self, hist):
        """보관한 키프레임 전체를 보정 포즈로 새 피쳐맵에 다시 쌓고 교체 (상단 docstring)."""
        if self.kf_store.full:
            if not self.lc_full_warned:
                self.lc_full_warned = True
                self.get_logger().warning(f"[LC 재동기화] 원재료가 상한 {self.lc_store_mb:g} MB 를 넘어 보관을 멈춤 — "
                                          "이후 재동기화 안 함 (lc_store_max_mb 를 키울 것)")
            self.stats["lc_skip_full"] += 1
            return
        t0 = time.monotonic()
        with self.lock:
            n0 = len(self.kf_store)
            recs = self.kf_store.recs[:n0]
            base = self.fg_base
        mm, md, nh = KR.max_move(recs, hist, self.lc_tol_ns)
        if nh == 0 or (mm < self.lc_min_m and md < self.lc_min_deg):
            self.stats["lc_skip_small"] += 1
            self.get_logger().info(f"[LC 재동기화] 건너뜀: 키프레임 {n0} 중 이력 있음 {nh}, "
                                   f"최대 보정 {mm * 100:.2f} cm / {md:.3f}° (< {self.lc_min_m * 100:g} cm·{self.lc_min_deg:g}°)")
            return
        g, upd, st = KR.rebuild(base, self._new_grid, recs, hist, self.K, self.grid_stride, self.depth_scale,
                                self.lc_tol_ns)
        t1 = time.monotonic()
        with self.lock:                                  # 재적분 중 새로 들어온 키프레임을 이어 쌓고 통째로 교체
            if len(self.kf_store) < n0 or (n0 and self.kf_store.recs[0] is not recs[0]):   # 추가 전용이라 정상이면 안 걸림
                self.get_logger().warning("[LC 재동기화] 보관 목록이 바뀜 — 이번 결과 버림")
                return
            tail = self.kf_store.recs[n0:]
            upd_t, st_t = KR.integrate_recs(g, tail, hist, self.K, self.grid_stride, self.depth_scale, self.lc_tol_ns)
            for rec, (T_new, M_new) in zip(recs + tail, upd + upd_t):
                if T_new is not None:
                    rec[3], rec[4] = T_new, M_new
            self.fg = g
            self.traj = [(rec[0], rec[3]) for rec in self.kf_store.recs]
        t2 = time.monotonic()
        self.lc_count += 1
        self.lc_last = dict(n=n0 + len(tail), miss=st["miss"] + st_t["miss"], cm=st["moved_max_m"] * 100,
                            deg=st["moved_max_deg"], ms=(t2 - t0) * 1000, lock_ms=(t2 - t1) * 1000)
        self.stats["lc_resync"] += 1
        self.get_logger().info(
            f"[LC 재동기화 #{self.lc_count}] 키프레임 {n0}+{len(tail)} (이력 없음 {self.lc_last['miss']}) "
            f"최대 이동 {self.lc_last['cm']:.2f} cm / {self.lc_last['deg']:.3f}° · 재적분 {(t1 - t0) * 1000:.0f} ms "
            f"+ 교체 {self.lc_last['lock_ms']:.0f} ms · 칸 {g.n_cells()} · 원재료 {self.kf_store.nbytes / 1e6:.1f} MB")

    def on_save(self, req, res):
        try:
            res.message = self.save()
            res.success = True
        except Exception as e:  # noqa: BLE001
            res.success, res.message = False, str(e)
        return res

    def report(self):
        gf = ""
        if self.gf_last:
            g = self.gf_last
            gf = (f" | ground_filter: 성분 {g['components']} 중 {g['kept_components']} 유지"
                  f"({g.get('seeds', '')}), "
                  f"제거 {g['removed']}점, {g['ms']:.0f}ms")
            if g.get("skipped"):
                gf = f" | ground_filter 생략: {g['skipped']}"
        if self.max_height > 0 and self.hc_last:
            gf += f" | 높이제한 {self.max_height} m: 제거 {self.hc_last['removed']}점"
        rc = f" | range_color {'ON' if self.range_on else 'OFF'}"
        if self.range_on and self.range_last:
            rc += f" (거리값 있는 점 {self.range_last['n']}/{self.range_last['total']})"
        if self.extract_last:
            rc += f" | 전체추출 {self.extract_last['n']}점 {self.extract_last['ms']:.0f}ms"
        if self.full_last:
            rc += f" | full_map {self.full_last['n']}점 {self.full_last['ms']:.0f}ms"
        if self.fg is not None:
            rc += (f" | grid 칸 {self.fg.n_cells()} 배열 {self.fg.shape} {self.fg.nbytes() / 1e6:.2f} MB"
                   + (f", 적분 {self.grid_last[0]:.0f}ms/{self.grid_last[1]}칸" if self.grid_last else ""))
        if self.kf_store is not None:
            rc += (f" | LC 재동기화 {self.lc_count}회 원재료 {len(self.kf_store)}장 {self.kf_store.nbytes / 1e6:.1f} MB"
                   + (" (상한 도달)" if self.kf_store.full else "")
                   + (f", 마지막 {self.lc_last['cm']:.1f} cm {self.lc_last['ms']:.0f} ms" if self.lc_last else ""))
        hg = (f" | pose {self.pose_mode} hold={len(self.hold)} recover={len(self.recover_q)}"
              if self.health_topic else "")
        if self.map_frame:
            hg += f" | frame {self.map_frame} reloc={(self.reloc or {}).get('state')} map_id={self.map_id}"
        self.get_logger().info(f"stats {dict(self.stats)} pending={len(self.pending)} "
                               f"queue={self.work.qsize()}{gf}{rc}{hg}")


def main():
    rclpy.init()
    node = LiveTsdf()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.stats["integrated"] > 0 and node.save_on_exit:
            print("saved:", node.save())
        elif not node.save_on_exit:
            print("save_on_exit=false: 맵 파일 저장 안 함")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
