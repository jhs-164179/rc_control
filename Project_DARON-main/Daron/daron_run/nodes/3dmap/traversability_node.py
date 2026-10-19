#!/usr/bin/env python3
"""주행가능성 격자 노드 (ROS 2 Humble, rclpy).

입력: /live_tsdf/map_cloud (PointCloud2, odom) + /slam/odometry + /imu/data + /tf_static
      + /health/state (std_msgs/String JSON, 261006) + (depth_confirm 일 때) depth 영상·camera_info
출력: ~/grid  (nav_msgs/OccupancyGrid  — RViz "Map" 디스플레이)
      ~/why   (nav_msgs/OccupancyGrid  — ~/grid 와 같은 header·info, data = judge_core 사유 코드. 261006)
      ~/cloud (PointCloud2 XYZRGB      — RViz "PointCloud2", 3D 맵 위에 색으로 겹침)

중력 정렬:
  IMU 가속도는 imu_link 기준이므로 odom 으로 옮겨야 한다.
      g_odom = R_odom_base @ R_base_imu @ (-a_imu_hat)
  R_odom_base 는 **맵과 같은 포즈 소스**(기본 /slam/odometry)에서 가져온다.
  R_base_imu 는 /tf_static 에서 가져온다(URDF base_to_imu).
  IMU 는 중력 방향 산출에만 쓰고 자세 추정에는 관여하지 않는다 — README §8의
  "한 맵 안에서 포즈 소스를 섞지 않는다"를 지키기 위한 구분이다.

[261006 판정 노드 작업지시서 2·3절] 전부 파라미터, 기본값이면 격자는 전과 같다 (발행 중단 게이트만 새로 켜짐)
  발행 중단: require_health 이면 /health/state 가 FAIL(ins 또는 slam)이거나 health_timeout_s 넘게 끊기면,
            그리고 gravity_mode=imu 이면 /imu/data 가 imu_timeout_s 넘게 끊기면 grid·why·cloud 를 내지 않는다.
  trend_mode / n_crit / s_crit_deg : judge_core 상수 (기동 때 한 번)
  h_crit_m / wall_span_m : judge_core H_CRIT·WALL_SPAN (기동 때 한 번, 261007 실험용 완화)
  drop_rules   : 격자를 로버 중심 3.2 m 정사각형(64x64)으로 고정 + inside(반경 1.25 m) + cam_pose 를 judge 에 넘김
  drop_suspect : JC.DROP_SUSPECT (drop_rules 필요)
  depth_confirm: JC.DEPTH_CONFIRM + 격자 지도 시각에 가장 가까운 depth 한 장 (drop_suspect 필요)
  up_corr      : IMU '위'(base_link 프레임)에 JC.up_correct 고정 보정각
  [261008] input_mode : cloud(옛 동작, /live_tsdf/map_cloud 점군을 칸에 넣어 통계) /
                        cells(그리드 피쳐맵 /live_tsdf/map_cells 의 칸 채널을 그대로 — 높이 평균·하/상단·분산·점 수).
                        cells 는 점을 다시 칸에 넣지 않고 목표 칸마다 원래 칸을 역사상으로 읽는다
                        (feature_grid.heightmap_from_cells). 판정기·게이트·출력은 두 모드 같다.
  [261008 밤] map_frame : 칸(map_cells)이 map 좌표면 'map'. 로버 위치·자세·중력·depth 포즈를 TF map<-odom 으로 옮겨 같이 쓴다.
              judge_extent : window(옛 동작, 로버 주변만) / map(받은 칸 전체를 판정 — 플래너는 쌓지 않고 그대로 씀).
              map 범위에서 judge_core 낙차 규칙(NEG/의심)은 지금 카메라 기준이라 로버 주변 창(inside) 안에서만 돈다.
              그 결과는 '낙차 근거' 층(map 5 cm 칸, drop_mem)에 기억하고, 창 밖 미관측 칸에 다시 입힌다(관측되면 지움) —
              판정 근거도 그리드 피쳐로 들고 다니는 셈. builtin 은 낙차 규칙이 없어 층도 없다.

의존: numpy, scipy (open3d 불필요)
"""
import collections
import hashlib
import json
import math
import struct
import threading
import time

import numpy as np
import rclpy
import tf2_ros
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid, Odometry
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, qos_profile_sensor_data
from scipy.spatial.transform import Rotation as R
from scipy.spatial.transform import Slerp
from sensor_msgs.msg import CameraInfo, Image, Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, String

import judge_adapters
from judge_adapters import get_judge
from feature_grid import heightmap_from_cells
from trav_core import (BLOCKED, UNKNOWN, TravParams, _cell_stats, compute_grid, gravity_align_R, inflate_blocked,
                       summarize)

_RGB_FIELDS = [
    PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
]

# drop_rules 격자 (261006 작업지시서 3-c): 로버 중심 정사각형 한 변 64칸 x 5 cm = 3.2 m, 판정 창 반경 1.25 m
DROP_GRID_N = 64
DROP_INSIDE_R = 1.25
# URDF base_link -> camera_link (tf_static 을 못 받을 때만 씀. 10/5 녹화 tf_static 과 같은 값)
CAM_XYZ_FALLBACK = np.array([0.244427, 0.0, 0.034352])


def cost_to_rgb(cost):
    """0(green) -> 65(yellow) -> 100(red). unknown 은 호출 전에 걸러진다."""
    c = np.clip(cost.astype(np.float32), 0, BLOCKED)
    blocked = cost >= BLOCKED
    t = np.clip(c / 65.0, 0, 1)                     # warn 구간 비율
    r = (255 * t).astype(np.uint32)
    g = (200 * (1 - 0.25 * t)).astype(np.uint32)
    b = np.zeros_like(r)
    r[blocked], g[blocked], b[blocked] = 230, 30, 30
    packed = (r << 16) | (g << 8) | b
    return np.frombuffer(packed.astype(np.uint32).tobytes(), np.float32)


def why_to_rgb(why, cost):
    """judge_core 사유(why)별 색 (260930). why 를 안 내는 판정기(builtin)는 cost_to_rgb.
    PASS: 비용 0 초록 -> 50 노랑 -> 99 주황. 불가(100)는 사유별 색, 보류(-1)는 회색.
    사유 없이 100 인 칸(팽창으로 막힌 PASS 등)은 빨강(안전망)."""
    import judge_core as JC
    c = cost.astype(np.float32)
    rgb = np.stack([np.interp(c, [0, 50, 99], ch)
                    for ch in ((101, 242, 240), (188, 208, 138), (80, 61, 40))], axis=1)
    rgb[cost < 0] = (60, 60, 60)                          # UNK (높이가 없어 보통 안 그려짐)
    rgb[(why == JC.SPARSE) & (cost < 0)] = (180, 180, 180)
    rgb[cost >= BLOCKED] = (212, 68, 60)                  # 안전망
    for code, col in ((JC.SLOPE, (212, 68, 60)), (JC.STEP, (47, 109, 222)), (JC.BOTH, (47, 109, 222)),
                      (JC.WALL, (18, 39, 87)), (JC.NEARWALL, (110, 127, 184)), (JC.NEG, (234, 51, 247))):
        rgb[why == code] = col
    r, g, b = (np.rint(rgb[:, k]).astype(np.uint32) for k in range(3))
    packed = (r << 16) | (g << 8) | b
    return np.frombuffer(packed.astype(np.uint32).tobytes(), np.float32)


def stamp_ns(h):
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


class TravNode(Node):
    def __init__(self):
        super().__init__("traversability")
        P = self.declare_parameter

        def PN(name, default):
            """숫자 파라미터: 정수·실수 어느 쪽으로 넘겨도 받는다 (rclpy INTEGER != DOUBLE 함정, 260911 로그)"""
            return P(name, default, ParameterDescriptor(dynamic_typing=True)).value

        self.cloud_topic = P("cloud_topic", "/live_tsdf/map_cloud").value
        self.input_mode = str(P("input_mode", "cloud").value).strip().lower()      # [261008] cloud | cells
        if self.input_mode not in ("cloud", "cells"):
            raise ValueError(f"input_mode 는 cloud/cells 여야 합니다: {self.input_mode!r}")
        self.cells_topic = P("cells_topic", "/live_tsdf/map_cells").value
        self.map_frame = str(P("map_frame", "").value).strip()                    # [261008 밤]
        self.judge_extent = str(P("judge_extent", "window").value).strip().lower()
        if self.judge_extent not in ("window", "map"):
            raise ValueError(f"judge_extent 는 window/map 여야 합니다: {self.judge_extent!r}")
        self.drop_mem = {}                       # (ix, iy) map 5 cm 칸 -> why (NEG/NEG_SUS)
        self.odom_frame_id = None
        self.cells = None
        self.odom_topic = P("odom_topic", "/slam/odometry").value
        self.imu_topic = P("imu_topic", "/imu/data").value
        self.period = P("update_period", 2.0).value
        self.gravity_mode = P("gravity_mode", "imu").value
        # 판정기 교체 지점. 이름 목록은 judge_adapters.JUDGES
        self.judge_name = P("judge", "judge_core").value   # [261008] 기본 judge_core (옛 builtin)
        self.judge = get_judge(self.judge_name)
        # judge_core 낙차(NEG) 의심 검사. 미관측 셀마다 파이썬 루프라 실시간에서는 끈다 (260928)
        judge_adapters.NEG_CHECK = bool(P("neg_check", False).value)

        # [261006 2절] 발행 중단 게이트
        self.health_topic = P("health_topic", "/health/state").value
        self.require_health = bool(P("require_health", True).value)
        self.health_timeout = float(PN("health_timeout_s", 2.0))
        self.imu_timeout = float(PN("imu_timeout_s", 0.5))
        # [261006 3절] judge_core 옵션. 기본은 지금과 같은 결과
        self.trend_mode = str(P("trend_mode", "plane").value)
        self.n_crit = int(PN("n_crit", 8))
        self.s_crit = float(PN("s_crit_deg", 20.0))
        # [261007] 턱·벽 기준도 파라미터로 (실험 중 완화용). 기본값 = judge_core.py 상수
        self.h_crit = float(PN("h_crit_m", 0.04))
        self.wall_span = float(PN("wall_span_m", 0.15))
        self.drop_rules = bool(P("drop_rules", False).value)
        self.drop_suspect = bool(P("drop_suspect", False).value)
        self.depth_confirm = bool(P("depth_confirm", False).value)
        self.up_corr = bool(P("up_corr", False).value)
        self.up_corr_pitch = float(PN("up_corr_pitch", 0.71))
        self.up_corr_roll = float(PN("up_corr_roll", -3.27))
        self.cam_frame = P("cam_frame", "camera_link").value
        self.depth_topic = P("depth_topic", "/camera/camera/depth/image_rect_raw").value
        self.depth_info_topic = P("depth_info_topic", "/camera/camera/depth/camera_info").value

        self.p = TravParams(
            cell_size=P("cell_size", 0.05).value,
            min_points_per_cell=P("min_points_per_cell", 2).value,
            max_range_m=P("max_range_m", 8.0).value,
            slope_warn_deg=P("slope_warn_deg", 15.0).value,
            slope_block_deg=P("slope_block_deg", 25.0).value,
            step_warn_m=P("step_warn_m", 0.025).value,
            step_block_m=P("step_block_m", 0.0449).value,
            rough_warn_m=P("rough_warn_m", 0.01).value,
            rough_block_m=P("rough_block_m", 0.02).value,
            inflate_radius_m=P("inflate_radius_m", 0.0).value,
        )

        self.lock = threading.Lock()
        self.cloud = None            # (N,3) odom
        self.cloud_stamp = None      # 지도 시각 (map_cloud header stamp, ns)
        self.world = self.base = None
        self.R_ob = None             # odom <- base 회전
        self.t_ob = np.zeros(3)      # 로버 위치 (max_range 중심)
        self.g_odom = None           # 중력(아래) 방향, odom
        self.imu_frame = None
        self.R_bi = None             # base <- imu 회전 (static)
        self.n_imu = 0
        self.imu_rx = None           # /imu/data 마지막 수신 (노드 시계 — bag 재생이면 sim time). 가속도 크기로 버린 샘플도 셈
        self.health = None           # 마지막 /health/state (dict)
        self.health_rx = None
        self.gate_prev = None        # 직전 발행 중단 사유 키 (None = 발행 중)
        self.gate_log_t = {}         # 발행 중단 이유 키 -> 마지막 로그 시각
        self.T_bc = None             # base <- camera_link (static, cam_pose 용)
        self.T_bd = {}               # base <- depth 광학 프레임 (static, frame_id 별)
        self.depth_buf = collections.deque(maxlen=90)     # (stamp_ns, frame_id, depth_m [::2, ::2]) 약 3 s @30 Hz
        self.odom_buf = collections.deque(maxlen=300)     # (stamp_ns, p, q) 약 10 s @30 Hz (depth 시각 자세 보간용)
        self.K_depth = None

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)

        self._setup_judge_core()

        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        if self.input_mode == "cells":
            self.create_subscription(PointCloud2, self.cells_topic, self.on_cells, 1)
        else:
            self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, 1)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 10)
        if self.gravity_mode == "imu":
            self.create_subscription(Imu, self.imu_topic, self.on_imu, qos_profile_sensor_data)
        if self.require_health:
            self.create_subscription(String, self.health_topic, self.on_health, 10)
        if self.depth_on:
            self.create_subscription(Image, self.depth_topic, self.on_depth, qos_profile_sensor_data)
            self.create_subscription(CameraInfo, self.depth_info_topic, self.on_depth_info, qos_profile_sensor_data)
        self.pub_grid = self.create_publisher(OccupancyGrid, "~/grid", latched)
        self.pub_why = self.create_publisher(OccupancyGrid, "~/why", latched)
        self.pub_cloud = self.create_publisher(PointCloud2, "~/cloud", 1)
        self.create_timer(self.period, self.update)

        self.get_logger().info(
            f"input={self.input_mode}:{self.cells_topic if self.input_mode == 'cells' else self.cloud_topic} "
            f"odom={self.odom_topic} gravity={self.gravity_mode} map_frame={self.map_frame or '-'} "
            f"extent={self.judge_extent} "
            f"judge={self.judge_name} neg_check={judge_adapters.NEG_CHECK} "
            f"min_pts={self.p.min_points_per_cell} inflate={self.p.inflate_radius_m} "
            f"cell={self.p.cell_size} "
            f"slope_block={self.p.slope_block_deg}deg "
            f"step_block={self.p.step_block_m * 1000:.0f}mm"
            + (" (builtin 전용, judge_core 는 안 씀)" if self.judge_name == "judge_core" else ""))
        self.get_logger().info(
            f"발행 중단 게이트: require_health={self.require_health} ({self.health_topic}, FAIL 또는 "
            f"{self.health_timeout:g} s 끊김) | IMU {self.imu_timeout:g} s 끊김"
            + ("" if self.gravity_mode == "imu" else " (gravity_mode=fixed 라 IMU 검사 안 함)")
            + f" | up_corr={self.up_corr}"
            + (f" (pitch {self.up_corr_pitch:+.2f} roll {self.up_corr_roll:+.2f} deg)" if self.up_corr else ""))
        if self.judge_name == "judge_core":
            import judge_core as JC     # 판정에 쓰이는 것과 같은 모듈 (judge_adapters 가 넣은 sys.path)
            with open(JC.__file__, "rb") as f:
                md5 = hashlib.md5(f.read()).hexdigest()
            self.get_logger().info(
                f"judge_core 기준값: S_CRIT={JC.S_CRIT}deg H_CRIT={JC.H_CRIT * 1000:.0f}mm "
                f"WALL_SPAN={JC.WALL_SPAN * 100:.0f}cm | {JC.__file__} md5={md5}")
            self.get_logger().info(
                f"judge_core v{getattr(JC, 'VERSION', '?')} 옵션: trend_mode={JC.TREND_MODE} "
                f"(R_TREND={JC.R_TREND} MIN_TREND={JC.MIN_TREND}) N_CRIT={JC.N_CRIT} S_CRIT={JC.S_CRIT} "
                f"drop_rules={self.drop_rules} DROP_SUSPECT={getattr(JC, 'DROP_SUSPECT', None)} "
                f"DEPTH_CONFIRM={getattr(JC, 'DEPTH_CONFIRM', None)}")

    def _setup_judge_core(self):
        """[261006 3-a·g·h·i] judge_core 모듈 상수를 기동 때 한 번 바꾼다. 기본값이면 v2.1 과 같은 결과."""
        self.depth_on = False
        self.opt_tag = ""
        if self.judge_name != "judge_core":
            if (self.trend_mode != "plane" or self.drop_rules or self.drop_suspect or self.depth_confirm
                    or self.n_crit != 8 or self.s_crit != 20.0
                    or self.h_crit != 0.04 or self.wall_span != 0.15):
                self.get_logger().warn(
                    f"judge={self.judge_name}: trend_mode/n_crit/s_crit_deg/h_crit_m/wall_span_m/drop_rules/drop_suspect/depth_confirm 는 "
                    f"judge_core 전용이라 무시합니다.")
            self.drop_rules = self.drop_suspect = self.depth_confirm = False
            return
        import judge_core as JC
        if self.trend_mode == "median":
            JC.TREND_MODE = "median"
            JC.R_TREND = 4
            JC.MIN_TREND = 14
        elif self.trend_mode != "plane":
            raise SystemExit(f"trend_mode 는 plane | median: {self.trend_mode!r}")
        JC.N_CRIT = self.n_crit
        JC.S_CRIT = self.s_crit
        JC.H_CRIT = self.h_crit
        JC.WALL_SPAN = self.wall_span     # judge_adapters·_grid_fixed 둘 다 JC.WALL_SPAN 을 읽음
        if abs(self.p.cell_size - JC.RES) > 1e-9 and self.drop_rules:
            raise SystemExit(f"drop_rules 는 cell_size {JC.RES} 전제 (지금 {self.p.cell_size})")
        if self.drop_suspect and not self.drop_rules:
            self.get_logger().warn("drop_suspect 는 drop_rules(inside·cam_pose)가 있어야 동작합니다 — 지금은 효과 없음")
        if self.depth_confirm and not (self.drop_suspect and self.drop_rules):
            self.get_logger().warn("depth_confirm 은 drop_rules + drop_suspect 가 있어야 동작합니다 — 지금은 효과 없음")
        JC.DROP_SUSPECT = self.drop_suspect
        JC.DEPTH_CONFIRM = self.depth_confirm
        self.depth_on = self.depth_confirm and self.drop_suspect and self.drop_rules
        tags = []
        if self.trend_mode != "plane":
            tags.append(f"trend={self.trend_mode}")
        if self.n_crit != 8:
            tags.append(f"N{self.n_crit}")
        if self.s_crit != 20.0:
            tags.append(f"S{self.s_crit:g}")
        if self.h_crit != 0.04:
            tags.append(f"H{self.h_crit * 100:g}cm")
        if self.wall_span != 0.15:
            tags.append(f"W{self.wall_span * 100:g}cm")
        tags += [n for n, on in (("drop", self.drop_rules), ("sus", self.drop_suspect),
                                 ("depth", self.depth_on)) if on]
        self.opt_tag = ",".join(tags)

    # ---------------- callbacks ----------------
    def on_cloud(self, m):
        pts = point_cloud2.read_points_numpy(m, field_names=("x", "y", "z"), skip_nans=True)
        with self.lock:
            self.cloud = np.asarray(pts, dtype=np.float64)
            self.cloud_stamp = stamp_ns(m.header)
            self.world = m.header.frame_id

    def on_cells(self, m):
        """[261008] 그리드 피쳐맵 칸 (x y z zlo zhi var w np nf). self.cloud 에는 칸 중심 xyz 를 둔다(게이트·개수 검사 공용)."""
        keys = ("x", "y", "z", "zlo", "zhi", "var", "np")
        a = point_cloud2.read_points_numpy(m, field_names=keys, skip_nans=True)
        a = np.asarray(a, dtype=np.float64).reshape(-1, len(keys))
        with self.lock:
            self.cells = {k: a[:, i] for i, k in enumerate(keys)}
            self.cloud = a[:, :3].copy()
            self.cloud_stamp = stamp_ns(m.header)
            self.world = m.header.frame_id

    def _hmap_cells(self, cells, R_go, center_g):
        """[261008] cells → build_heightmap 과 같은 dict (builtin 경로). 범위·격자 크기 규칙도 같게."""
        pg = np.column_stack([cells["x"], cells["y"], cells["z"]]) @ R_go.T
        p = self.p
        if center_g is not None and p.max_range_m > 0:
            pg = pg[np.linalg.norm(pg[:, :2] - np.asarray(center_g), axis=1) <= p.max_range_m]
        if pg.shape[0] < 1:
            return None
        c = p.cell_size
        lo_xy = np.floor(pg[:, :2].min(axis=0) / c) * c - c
        hi_xy = np.ceil(pg[:, :2].max(axis=0) / c) * c + c
        nx, ny = (np.round((hi_xy - lo_xy) / c).astype(int) + 1)
        nx, ny = int(max(nx, 1)), int(max(ny, 1))
        cnt, zmean, zstd, zmin, zmax = heightmap_from_cells(cells, R_go, lo_xy, (nx, ny), c)
        valid = cnt >= p.min_points_per_cell
        if not valid.any():
            return None
        return dict(origin=lo_xy, cell=c, shape=(nx, ny), count=cnt, valid=valid,
                    center_xy=None if center_g is None else np.asarray(center_g, float),
                    height=np.where(valid, zmean, np.nan), span=np.where(valid, zmax - zmin, np.nan),
                    zstd=np.where(valid, zstd, np.nan))

    def on_odom(self, m):
        q = m.pose.pose.orientation
        p = m.pose.pose.position
        with self.lock:
            self.base = m.child_frame_id
            self.odom_frame_id = m.header.frame_id
            self.R_ob = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
            self.t_ob = np.array([p.x, p.y, p.z])
            if self.depth_on:
                self.odom_buf.append((stamp_ns(m.header), self.t_ob.copy(), np.array([q.x, q.y, q.z, q.w])))

    def on_imu(self, m):
        self.imu_rx = self.get_clock().now()
        a = np.array([m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z])
        n = np.linalg.norm(a)
        if not (9.0 < n < 10.6):        # 가감속 중이면 중력 추정을 오염시키므로 버린다
            return
        self.imu_frame = m.header.frame_id or "imu_link"
        if self.R_bi is None:
            if self.base is None:
                return
            try:
                tf = self.tf_buf.lookup_transform(self.base, self.imu_frame, rclpy.time.Time())
            except Exception:
                return
            r = tf.transform.rotation
            self.R_bi = R.from_quat([r.x, r.y, r.z, r.w]).as_matrix()
            self.get_logger().info(f"R_base_imu 확보 ({self.base} <- {self.imu_frame})")
        with self.lock:
            if self.R_ob is None:
                return
            if self.up_corr:
                # [261006 3-e] '위'를 base_link 프레임에서 고정 보정각만큼 돌린 뒤 odom 으로 (judge_core.up_correct)
                import judge_core as JC
                up_b = JC.up_correct(self.R_bi @ (a / n), self.up_corr_pitch, self.up_corr_roll)
                g = self.R_ob @ (-up_b)
            else:
                g = self.R_ob @ self.R_bi @ (-a / n)     # odom 기준 '아래' 방향
            self.g_odom = g if self.g_odom is None else 0.98 * self.g_odom + 0.02 * g
            self.n_imu += 1

    def on_health(self, m):
        try:
            h = json.loads(m.data)
        except ValueError:
            self.get_logger().warn(f"{self.health_topic} JSON 아님: {m.data[:80]!r}", throttle_duration_sec=5.0)
            return
        with self.lock:
            self.health, self.health_rx = h, self.get_clock().now()

    def on_depth_info(self, m):
        if self.K_depth is None:
            self.K_depth = np.array(m.k, float).reshape(3, 3)

    def on_depth(self, m):
        if m.encoding in ("16UC1", "mono16"):
            d = np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width]
            d = d[::2, ::2].astype(np.float32) / 1000.0       # mm -> m, 0 = 값 없음
        elif m.encoding == "32FC1":
            d = np.frombuffer(m.data, np.float32).reshape(m.height, m.step // 4)[:, :m.width][::2, ::2].copy()
            d[~np.isfinite(d)] = 0.0
        else:
            self.get_logger().warn(f"depth encoding {m.encoding} 모름", throttle_duration_sec=10.0)
            return
        with self.lock:
            self.depth_buf.append((stamp_ns(m.header), m.header.frame_id, d))

    # ---------------- 발행 중단 게이트 (261006 2절) ----------------
    def _gate_reasons(self):
        """발행하면 안 되는 이유 [(키, 설명)]. 빈 목록이면 발행."""
        now = self.get_clock().now()
        out = []
        if self.require_health:
            with self.lock:
                h, rx = self.health, self.health_rx
            if rx is None:
                out.append(("health_none", f"{self.health_topic} 미수신"))
            else:
                age = (now - rx).nanoseconds * 1e-9
                if age > self.health_timeout:
                    out.append(("health_stale", f"{self.health_topic} {age:.1f} s 끊김 (> {self.health_timeout:g} s)"))
                for k in ("ins", "slam"):
                    if h.get(k) == "FAIL":
                        out.append((f"{k}_fail", f"{k.upper()} FAIL ({','.join(h.get(k + '_reasons') or []) or '-'})"))
        if self.gravity_mode == "imu" and self.imu_rx is not None:
            age = (now - self.imu_rx).nanoseconds * 1e-9
            if age > self.imu_timeout:
                out.append(("imu_stale", f"{self.imu_topic} {age:.2f} s 끊김 (> {self.imu_timeout:g} s)"))
        return out

    def _gate(self):
        """True 면 이번 주기는 발행하지 않는다. 같은 이유는 5 s 에 한 번만 로그."""
        rs = self._gate_reasons()
        now = self.get_clock().now()
        if not rs:
            if self.gate_prev is not None:
                self.get_logger().info(f"판정 격자 발행 재개 (t={now.nanoseconds * 1e-9:.2f}, "
                                       f"멈춘 이유였던 것: {', '.join(self.gate_prev)})")
                self.gate_prev = None
            return False
        # 이유(키)마다 마지막 로그 시각. 5 s 안에 찍은 적 없는 이유가 하나라도 있으면 한 줄 (지금 이유 전부)
        fresh = [k for k, _ in rs if k not in self.gate_log_t
                 or not 0 <= (now - self.gate_log_t[k]).nanoseconds < 5_000_000_000]
        if fresh:
            self.get_logger().warn(f"판정 격자 발행 중단 (t={now.nanoseconds * 1e-9:.2f}) — "
                                   f"{' / '.join(s for _, s in rs)}")
            for k, _ in rs:
                self.gate_log_t[k] = now
        self.gate_prev = tuple(k for k, _ in rs)
        return True

    # ---------------- drop_rules 용 (261006 3-c·i) ----------------
    def _static_tf(self, target, source):
        tf = self.tf_buf.lookup_transform(target, source, rclpy.time.Time())
        tr, q = tf.transform.translation, tf.transform.rotation
        T = np.eye(4)
        T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3, 3] = (tr.x, tr.y, tr.z)
        return T

    def _cam_pose(self, R_go, R_ob, t_ob):
        """지금 카메라 (x, y, yaw), 격자(중력) 좌표계. 위치 = 로버 포즈로 옮긴 base->camera_link,
        yaw = camera_link +x 를 격자 평면에 투영한 각도."""
        if self.T_bc is None and self.base is not None:
            try:
                self.T_bc = self._static_tf(self.base, self.cam_frame)
                self.get_logger().info(f"cam_pose: {self.base} <- {self.cam_frame} 확보 "
                                       f"(이동 {np.round(self.T_bc[:3, 3], 4).tolist()})")
            except Exception:
                pass
        if self.T_bc is None:
            t_bc, R_bc = CAM_XYZ_FALLBACK, np.eye(3)
            self.get_logger().warn(f"{self.cam_frame} TF 없음 — URDF 상수로 cam_pose", throttle_duration_sec=10.0)
        else:
            t_bc, R_bc = self.T_bc[:3, 3], self.T_bc[:3, :3]
        c = R_go @ (t_ob + R_ob @ t_bc)
        f = R_go @ R_ob @ R_bc @ np.array([1.0, 0.0, 0.0])
        return float(c[0]), float(c[1]), float(math.atan2(f[1], f[0]))

    def _odom_at(self, t_ns):
        """odom_topic 을 t 에서 보간한 4x4 (위치 선형, 회전 slerp — live_tsdf interp 와 같은 방식). 못 하면 None"""
        od = list(self.odom_buf)
        for i in range(len(od) - 1, 0, -1):
            t0, t1 = od[i - 1][0], od[i][0]
            if t0 <= t_ns <= t1:
                a = 0.0 if t1 == t0 else (t_ns - t0) / (t1 - t0)
                T = np.eye(4)
                T[:3, :3] = Slerp([0.0, 1.0], R.from_quat([od[i - 1][2], od[i][2]]))([a]).as_matrix()[0]
                T[:3, 3] = (1 - a) * od[i - 1][1] + a * od[i][1]
                return T
            if t1 < t_ns:
                break
        if od and 0 <= t_ns - od[-1][0] <= 50_000_000:     # 아직 다음 odom 이 안 옴: 50 ms 안이면 마지막 포즈
            T = np.eye(4)
            T[:3, :3] = R.from_quat(od[-1][2]).as_matrix()
            T[:3, 3] = od[-1][1]
            return T
        return None

    def _T_map_odom(self):
        """[261008 밤] 최신 TF map<-odom. map_frame 없거나 칸이 odom 이면 항등, 실패하면 None."""
        if not self.map_frame or self.world != self.map_frame or not self.odom_frame_id \
                or self.odom_frame_id == self.map_frame:
            return np.eye(4)
        try:
            tf = self.tf_buf.lookup_transform(self.map_frame, self.odom_frame_id, rclpy.time.Time())
        except Exception:
            return None
        q, t = tf.transform.rotation, tf.transform.translation
        T = np.eye(4)
        T[:3, :3] = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    def _depth_pts(self, R_go, t_map, T_mo=None):
        """[261006 3-i] 지도 시각에 가장 가까운 depth 한 장 -> 격자 좌표 점. (pts, 설명) / (None, 이유)"""
        import judge_core as JC
        with self.lock:
            buf = list(self.depth_buf)
        if self.K_depth is None or not buf or t_map is None:
            return None, "depth·camera_info 없음"
        ts = np.array([b[0] for b in buf])
        i = int(np.argmin(np.abs(ts - t_map)))          # 같은 거리면 이전 것 (argmin 은 첫 번째)
        t_d, fid, d = buf[i]
        if abs(t_d - t_map) > 200_000_000:
            return None, f"지도 시각 ±0.2 s 안 depth 없음 ({(t_d - t_map) * 1e-9:+.2f} s)"
        with self.lock:
            T_ob = self._odom_at(t_d)
        if T_ob is None:
            return None, "depth 시각 odom 없음"
        if fid not in self.T_bd:
            try:
                self.T_bd[fid] = self._static_tf(self.base, fid)
            except Exception as e:
                return None, f"{self.base}<-{fid} TF 없음 ({type(e).__name__})"
        G = np.eye(4)
        G[:3, :3] = R_go
        K2 = self.K_depth.copy()
        K2[0, 0] /= 2; K2[1, 1] /= 2; K2[0, 2] /= 2; K2[1, 2] /= 2    # [::2, ::2] 영상: u_full = 2 u
        pts = JC.depth_to_points(d, K2, G @ (np.eye(4) if T_mo is None else T_mo) @ T_ob @ self.T_bd[fid], step=1)
        return pts, f"depth {(t_d - t_map) * 1e-3:+.0f} us, {len(pts)}점"

    def _grid_fixed(self, pts_g, center_xy, cam_pose, depth_pts, cells=None, R_go=None):
        """[261006 3-c] 로버 중심 3.2 m 정사각형 격자 + inside·cam_pose 를 judge_core 에 넘김.
        반환은 compute_grid 와 같은 꼴 (배열 (ny, nx))."""
        import judge_core as JC
        c = self.p.cell_size
        n = DROP_GRID_N
        cx, cy = float(center_xy[0]), float(center_xy[1])
        org = np.array([math.floor((cx - n * c / 2) / c) * c, math.floor((cy - n * c / 2) / c) * c])
        nx = ny = n
        if self.judge_extent == "map" and pts_g is not None and len(pts_g):
            # [261008 밤] 받은 칸 전체 + 로버 창(3.2 m)을 덮는 격자 (낙차 규칙 여유 30 cm 포함)
            lo = np.minimum(pts_g[:, :2].min(axis=0) - 0.35, org)
            hi = np.maximum(pts_g[:, :2].max(axis=0) + 0.35, org + n * c)
            org = np.floor(lo / c) * c
            nx, ny = (int(v) for v in np.ceil((hi - org) / c))
        if cells is not None:                                            # [261008] 피쳐맵 칸 채널
            cnt, zmean, zstd, zmin, zmax = heightmap_from_cells(cells, R_go, org, (nx, ny), c)
        else:
            cnt, zmean, zstd, zmin, zmax = _cell_stats(pts_g, c, org, (nx, ny))
        valid = cnt >= self.p.min_points_per_cell
        if not valid.any():
            return None
        hmap = dict(origin=org, cell=c, shape=(nx, ny), count=cnt, valid=valid,
                    center_xy=np.array([cx, cy]),
                    height=np.where(valid, zmean, np.nan),
                    span=np.where(valid, zmax - zmin, np.nan),
                    zstd=np.where(valid, zstd, np.nan))
        Y, X = np.mgrid[0:ny, 0:nx]
        inside = np.hypot(org[0] + (X + .5) * c - cx, org[1] + (Y + .5) * c - cy) <= DROP_INSIDE_R
        tall = np.nan_to_num(hmap["span"]) > JC.WALL_SPAN
        cam_xy = (cx, cy) if judge_adapters.NEG_CHECK else None       # 옛 한 장면용 규칙 (어댑터와 같게)
        r = JC.judge(np.ascontiguousarray(hmap["height"].T, dtype=float), cam_xy=cam_xy, tall=tall.T,
                     origin=(float(org[0]), float(org[1])), inside=inside.T, cam_pose=cam_pose,
                     depth_pts=depth_pts)
        res = dict(cost=np.asarray(r["cost"]).T, slope=np.asarray(r["sl"]).T, step=np.asarray(r["step"]).T,
                   rough=np.asarray(r["rough"]).T, why=np.asarray(r["why"]).T, judge_stats=JC.stats(r))
        if self.judge_extent == "map" and R_go is not None:
            self._apply_drop_mem(res, valid, inside, org, c, R_go, JC)
        cost = inflate_blocked(res["cost"], valid, self.p, c)
        out = dict(hmap)
        out.update(res)
        out["cost"] = cost.astype(np.int8)
        out["inside"] = inside
        return out

    def _apply_drop_mem(self, res, valid, inside, org, c, R_go, JC):
        """[261008 밤] 낙차 근거 층: 창 안에서 나온 NEG/의심을 map 5 cm 칸에 기억, 관측되면 지움, 창 밖 미관측 칸에 다시 입힘."""
        ny, nx = valid.shape
        Y, X = np.mgrid[0:ny, 0:nx]
        pg = np.stack([org[0] + (X + .5) * c, org[1] + (Y + .5) * c, np.zeros_like(X, float)], -1)
        pm = pg @ R_go                                                    # 중력 -> map (R_go.T @ p)
        kx = np.floor(pm[..., 0] / c).astype(np.int64)
        ky = np.floor(pm[..., 1] / c).astype(np.int64)
        why, cost = res["why"], res["cost"]
        mark = inside & ((why == JC.NEG) | (why == JC.NEG_SUS))
        for a, b, w in zip(kx[mark], ky[mark], why[mark]):
            self.drop_mem[(int(a), int(b))] = int(w)
        for a, b in zip(kx[valid], ky[valid]):                          # 관측된 칸은 근거 지움
            self.drop_mem.pop((int(a), int(b)), None)
        if not self.drop_mem:
            return
        tgt = ~inside & ~valid & (cost < 0)
        for i, j in zip(*np.nonzero(tgt)):
            w = self.drop_mem.get((int(kx[i, j]), int(ky[i, j])))
            if w is not None:
                why[i, j] = w
                cost[i, j] = 100 if w == JC.NEG else JC.SUS_COST

    # ---------------- main ----------------
    def update(self):
        if self._gate():
            return
        with self.lock:
            pts = None if self.cloud is None else self.cloud.copy()
            cells = None if self.cells is None else {k: v.copy() for k, v in self.cells.items()}
            world, g, center = self.world, self.g_odom, self.t_ob.copy()
            R_ob, t_map = self.R_ob, self.cloud_stamp
        if pts is None or pts.shape[0] < (20 if self.input_mode == "cells" else 100):
            return
        if self.gravity_mode == "imu":
            if g is None:
                self.get_logger().warn(
                    f"중력 방향 미확보 (IMU {self.n_imu}샘플). {self.imu_topic} 와 "
                    f"base<-imu static TF 를 확인하세요. gravity_mode:=fixed 로 우회 가능.",
                    throttle_duration_sec=10.0)
                return
        else:
            g = np.array([0.0, 0.0, -1.0])

        T_mo = None
        if self.map_frame and world == self.map_frame:                   # [261008 밤] 로버·중력을 map 좌표로
            T_mo = self._T_map_odom()
            if T_mo is None:
                self.get_logger().warn(f"TF {self.map_frame}<-{self.odom_frame_id} 없음 — 판정 보류",
                                       throttle_duration_sec=5.0)
                return
            center = T_mo[:3, :3] @ center + T_mo[:3, 3]
            g = T_mo[:3, :3] @ g
            if R_ob is not None:
                R_ob = T_mo[:3, :3] @ R_ob
        R_go = gravity_align_R(g)
        t0 = time.monotonic()
        dinfo = ""
        if self.drop_rules:
            if R_ob is None:
                return
            cam = self._cam_pose(R_go, R_ob, center)
            dpts = None
            if self.depth_on:
                dpts, dinfo = self._depth_pts(R_go, t_map, T_mo)
                dinfo = f" | {dinfo}"
            grid = self._grid_fixed(pts @ R_go.T, (R_go @ center)[:2], cam, dpts,
                                    cells=cells if self.input_mode == "cells" else None, R_go=R_go)
        elif self.input_mode == "cells":
            hmap = self._hmap_cells(cells, R_go, (R_go @ center)[:2])
            grid = None if hmap is None else compute_grid(None, self.p, judge=self.judge, hmap=hmap)
        else:
            grid = compute_grid(pts @ R_go.T, self.p, center_xy=(R_go @ center)[:2],
                                judge=self.judge)
        ms = (time.monotonic() - t0) * 1000
        if grid is None:
            return

        R_og = R_go.T
        m = self.publish_grid(grid, R_og, world)
        self.publish_why(grid, m)
        self.publish_cloud(grid, R_og, world)
        tilt = math.degrees(math.acos(np.clip(np.dot(-g / np.linalg.norm(g), [0, 0, 1]), -1, 1)))
        extra = ""
        js = grid.get("judge_stats")
        if js:
            extra = (f" | lethal {js['lethal']:.1%} (slope {js['slope_only']:.1%} /"
                     f" step {js['step_only']:.1%} / both {js['both']:.1%}"
                     f" / near_wall {js['near_wall']:.1%})"
                     f" wall {js['wall_cells']} neg {js['neg_cells']}")
            if "why" in grid:
                import judge_core as JC
                extra += f" sus {int((grid['why'] == getattr(JC, 'NEG_SUS', -99)).sum())}"
        tag = self.judge_name
        if self.judge_name == "judge_core":
            import judge_core as JC
            tag = f"judge_core v{getattr(JC, 'VERSION', '?')}" + (f" {self.opt_tag}" if self.opt_tag else "")
        self.get_logger().info(
            f"[{tag}] {summarize(grid)} | odom 기울기 {tilt:.1f}deg{extra} | 격자+판정 {ms:.0f} ms{dinfo}")

    def publish_grid(self, gd, R_og, world):
        nx, ny = gd["shape"]
        valid = gd["valid"]
        z0 = float(np.nanmedian(gd["height"][valid])) if valid.any() else 0.0
        org = R_og @ np.array([gd["origin"][0], gd["origin"][1], z0])
        q = R.from_matrix(R_og).as_quat()

        m = OccupancyGrid()
        m.header = Header(frame_id=world, stamp=self.get_clock().now().to_msg())
        m.info.resolution = float(gd["cell"])
        m.info.width, m.info.height = int(nx), int(ny)
        m.info.origin = Pose()
        m.info.origin.position.x, m.info.origin.position.y, m.info.origin.position.z = map(float, org)
        (m.info.origin.orientation.x, m.info.origin.orientation.y,
         m.info.origin.orientation.z, m.info.origin.orientation.w) = map(float, q)
        m.data = gd["cost"].astype(np.int8).ravel().tolist()
        self.pub_grid.publish(m)
        return m

    def publish_why(self, gd, grid_msg):
        """[261006 3-d] 사유 코드 격자. ~/grid 와 같은 header·info. why 를 안 내는 판정기(builtin)면 안 냄."""
        if "why" not in gd:
            return
        import judge_core as JC
        m = OccupancyGrid()
        m.header = grid_msg.header
        m.info = grid_msg.info
        m.data = JC.to_occupancy_data(np.asarray(gd["why"]).T)     # why (ny,nx) -> [x, y] 로 넘겨 행 = y
        self.pub_why.publish(m)

    def publish_cloud(self, gd, R_og, world):
        nx, ny = gd["shape"]
        valid = gd["valid"]
        if not valid.any():
            return
        xs = gd["origin"][0] + (np.arange(nx) + 0.5) * gd["cell"]
        ys = gd["origin"][1] + (np.arange(ny) + 0.5) * gd["cell"]
        X, Y = np.meshgrid(xs, ys)
        pg = np.stack([X[valid], Y[valid], gd["height"][valid]], axis=1)
        po = pg @ R_og.T
        if "why" in gd:
            rgb = why_to_rgb(gd["why"][valid], gd["cost"][valid])
        else:
            rgb = cost_to_rgb(gd["cost"][valid])
        data = np.column_stack([po.astype(np.float32), rgb]).astype(np.float32)
        hdr = Header(frame_id=world, stamp=self.get_clock().now().to_msg())
        self.pub_cloud.publish(point_cloud2.create_cloud(hdr, _RGB_FIELDS, data))


def main():
    rclpy.init()
    n = TravNode()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
