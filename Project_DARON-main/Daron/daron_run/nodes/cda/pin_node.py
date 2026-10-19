#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
pin_node — 검출(CDA/YOLO) bbox 를 맵(odom) 좌표 핀으로 바꿔 RViz MarkerArray 로 낸다. 작성 261001.

입력
  det_topic   (std_msgs/String JSON, 영상 1장 = 메시지 1개)
              {"stamp_ns": <infra1 header.stamp>, "camera": "infra1", "source": "fake|yolo",
               "dets": [{"box": [x1, y1, x2, y2], "score": 0.9, "class": "crater"}, ...]}
              box 는 원본 848x480 픽셀. camera 가 infra1 이 아니면 버린다(D405 depth 가 infra1 픽셀 기준).
  depth_topic + info_topic   같은 stamp 의 depth 를 buffer_s 동안 들고 있다가 검출과 맞춘다 (추론 지연 흡수)
  odom_topic  live_tsdf 맵과 같은 포즈 소스(기본 /odometry/filtered_health). stamp 보간, 외삽 안 함
  /tf_static  base → depth 광학 프레임
  health_topic  SLAM FAIL 이면 그 검출은 버린다 (live_tsdf 도 그 프레임을 맵에 안 넣는다)
출력
  ~/pins (visualization_msgs/MarkerArray, TRANSIENT_LOCAL)  원판(크기=반지름) + 막대 + 머리 + 라벨
  ~/pins_json (std_msgs/String, TRANSIENT_LOCAL)            핀 목록 (planner 등 다른 노드용)
              핀마다 top = score 최고 검출 {stamp_ns, box, score} (261006, 지상국 크레이터 정보 창 이미지용)
  종료 시 out_dir/pins_<시각>.json (out_dir 가 비면 저장 안 함)

계산은 nodes/lib/pins.py — 오프라인 도구(~/Daron/yolo/tools/pins_offline.py)와 같은 코드.
⚠ live_tsdf 가 ANCHOR/HOLD(INS 비정상) 모드일 때는 맵이 odom_topic 이 아닌 앵커 포즈로 쌓인다 —
  그 구간 핀은 맵과 어긋날 수 있다. 지금은 SLAM FAIL 만 거른다.

[261011] map_frame (기본 '' = 옛 동작: odom 좌표 핀)
  'map' 이면 검출점을 그 시각에 쓰던 TF map<-odom 으로 map 좌표에 놓고 핀·마커·json 을 map 으로 낸다 (피쳐맵·웨이포인트와 같은 좌표).
  lc_resync 면 /slam/map_odom_history(루프 클로저 보정 이력)를 받아, 검출마다 p_new = M_corr(t) · M_ref⁻¹ · p 로 옮기고
  핀 중심을 다시 계산한다 (지난 핀·지금 쌓이는 핀 모두. 핀 id·소속·반지름·top 은 그대로 — lib/pins.py reposition).
  피쳐맵 재동기화(live_tsdf lc_resync)와 같은 이력·같은 식이라 핀이 다시 쌓인 지도와 같이 움직인다.

[261011 밤] 재추정 대기 · 이전 세션 핀 불러오기 (map_frame 일 때)
  /slam/reloc_status 가 확정(NEW_MAP/ACCEPTED/REJECTED/FAILED)될 때까지 검출을 안 쓴다 — 확정 전 map 좌표는 바뀔 수 있다
  (live_tsdf 와 같은 규칙, reloc_wait_s 동안 상태가 안 오면 Slam 없음으로 보고 시작).
  pins_load: ACCEPTED 면 이전 세션 핀을 미리 넣는다 (lib/pins.py seed). 'auto' = 재추정한 Slam 맵 폴더 상위(세션 폴더)의 최신
  pins_*.json, 경로 = 그 파일, '' = 안 함. 파일 frame 이 map_frame 과 다르거나(옛 odom 파일) map_id 가 다르면 안 불러온다.
  → 새 검출이 같은 판정으로 기존 핀에 합쳐져, 이전에 관측한 크레이터를 다시 봐도 새 크레이터로 안 늘어난다.
  저장 json 에 map_id·loaded_from 을 남긴다 (다음 세션 확인용).
"""

import collections
import json
import os
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Odometry, Path as PathMsg
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / '3dmap'))
from lib.pins import OdomBuffer, PinSet, box_on_edge, box_to_point  # noqa: E402
import kf_resync as KR  # noqa: E402  [261011] 보정 이력 (numpy 만)
from lib.rotations import quat_to_mat  # noqa: E402


def stamp_ns(h):
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


class PinNode(Node):
    def __init__(self):
        super().__init__('pin_node')
        P = self.declare_parameter
        self.det_topic = P('det_topic', '/cda/detections').value
        self.depth_topic = P('depth_topic', '/camera/camera/depth/image_rect_raw').value
        self.info_topic = P('info_topic', '/camera/camera/depth/camera_info').value
        self.odom_topic = P('odom_topic', '/odometry/filtered_health').value
        self.health_topic = P('health_topic', '/health/state').value
        self.dmin = float(P('dmin', 0.07).value)
        self.dmax = float(P('dmax', 1.0).value)            # live_tsdf dmax 와 같게 — 맵에 없는 거리엔 핀도 없음
        self.min_valid = float(P('min_valid', 0.3).value)
        self.depth_scale = float(P('depth_scale', 0.001).value)
        self.buffer_s = float(P('buffer_s', 3.0).value)    # depth 보관 (추론 지연 상한)
        self.wait_s = float(P('wait_s', 1.0).value)        # 검출 뒤 depth·odom 이 따라오길 기다리는 상한
        self.out_dir = P('out_dir', '').value
        self.stick_h = float(P('stick_h', 0.15).value)
        # [261001] YOLO 팀 연동 시험 임계값 0.25 / 테두리 박스는 위치 평균에서 제외 / 포즈 빠진 구간 건너 보간 금지
        self.score_min = float(P('score_min', 0.25).value)
        self.edge_margin = float(P('edge_margin', 2.0).value)
        self.max_odom_gap_ns = int(float(P('max_odom_gap', 0.15).value) * 1e9)
        self.pins = PinSet(join_m=float(P('join_m', 0.10).value), min_count=int(P('min_count', 3).value),
                           cand_n=int(P('cand_n', 10).value),                    # [261009] 형상 계산 후보 (lib/pins.py)
                           cand_zmin=float(P('cand_zmin', 0.35).value), cand_zmax=float(P('cand_zmax', 1.0).value),
                           block_s=float(P('block_s', 5.0).value),               # [261013] 열화도 반복 관측 구간 (lib/pins.py)
                           block_n=int(P('block_n', 12).value))
        # [261011] map 좌표 · 루프 클로저 재동기화 (상단 docstring)
        self.map_frame = str(P('map_frame', '').value).strip()
        self.lc_resync = bool(P('lc_resync', False).value) and bool(self.map_frame)
        self.lc_tol_ns = int(float(P('lc_resync_tol_s', 0.1).value) * 1e9)
        lc_topic = str(P('lc_history_topic', '/slam/map_odom_history').value)
        self.lc_count = 0
        # [261011 밤] 재추정 대기 · 이전 세션 핀 (상단 docstring)
        self.pins_load = str(P('pins_load', '').value).strip()
        reloc_topic = str(P('reloc_topic', '/slam/reloc_status').value)
        self.reloc_wait_s = float(P('reloc_wait_s', 20.0).value)
        self.reloc = None
        self.map_id = ''
        self.pins_loaded = None                            # None = 아직 안 봄, '' = 안 불러옴, 경로 = 불러옴
        self.t_start = time.monotonic()

        self.K = None
        self.depth = collections.OrderedDict()             # stamp_ns -> (u16 HxW, frame_id)
        self.odom = OdomBuffer()
        self.world = self.base = None
        self.T_bd = {}
        self.health = None
        self.pending = collections.deque()                 # (det_msg dict, recv_monotonic)
        self.stats = collections.Counter()
        self.last_pub = None

        self.tf_buf = tf2_ros.Buffer()
        self.tf_lis = tf2_ros.TransformListener(self.tf_buf, self)
        self.create_subscription(String, self.det_topic, self.on_det, 50)
        self.create_subscription(Image, self.depth_topic, self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, self.info_topic, self.on_info, qos_profile_sensor_data)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 100)
        if self.health_topic:
            self.create_subscription(String, self.health_topic, self.on_health, 10)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_mk = self.create_publisher(MarkerArray, '~/pins', latched)
        self.pub_js = self.create_publisher(String, '~/pins_json', latched)
        if self.map_frame:
            from rclpy.qos import HistoryPolicy, ReliabilityPolicy
            self.create_subscription(String, reloc_topic, self.on_reloc,
                                     QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                                reliability=ReliabilityPolicy.RELIABLE,
                                                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        if self.lc_resync:
            from rclpy.qos import HistoryPolicy, ReliabilityPolicy
            self.create_subscription(PathMsg, lc_topic, self.on_lc_history,
                                     QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                                                reliability=ReliabilityPolicy.RELIABLE,
                                                durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.02, self.process)
        self.create_timer(5.0, self.report)
        self.get_logger().info(
            f'det={self.det_topic} odom={self.odom_topic} dmax={self.dmax} join={self.pins.join_m} '
            f'min_count={self.pins.min_count} score_min={self.score_min} edge_margin={self.edge_margin}px '
            f'buffer={self.buffer_s}s out_dir={self.out_dir or "(저장 안 함)"} '
            f'map_frame={self.map_frame or "(odom 그대로)"} lc_resync={"on " + lc_topic if self.lc_resync else "off"}')

    # ---------------- 입력 ----------------
    def on_info(self, m):
        if self.K is None:
            self.K = np.array(m.k, dtype=np.float64).reshape(3, 3)

    def on_depth(self, m):
        if m.encoding not in ('16UC1', 'mono16'):
            self.stats['depth_bad_encoding'] += 1
            return
        t = stamp_ns(m.header)
        self.depth[t] = (np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width].copy(),
                         m.header.frame_id)
        while self.depth and next(iter(self.depth)) < t - int(self.buffer_s * 1e9):
            self.depth.popitem(last=False)

    def on_odom(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.odom.append(stamp_ns(m.header), [p.x, p.y, p.z], [q.x, q.y, q.z, q.w])
        if self.world is None:
            self.world, self.base = m.header.frame_id, m.child_frame_id

    def on_health(self, m):
        try:
            self.health = json.loads(m.data)
        except ValueError:
            pass

    def on_det(self, m):
        try:
            d = json.loads(m.data)
            int(d['stamp_ns'])
        except (ValueError, KeyError, TypeError):
            self.stats['det_bad_json'] += 1
            return
        if d.get('camera', 'infra1') != 'infra1':
            self.stats['det_not_infra1'] += 1
            return
        self.stats['det_msgs'] += 1
        if d.get('dets'):
            self.pending.append((d, time.monotonic()))

    # ---------------- 처리 ----------------
    def _T_base_depth(self, frame):
        if frame not in self.T_bd:
            try:
                tf = self.tf_buf.lookup_transform(self.base, frame, Time())
            except tf2_ros.TransformException:
                return None
            T = np.eye(4)
            q, t = tf.transform.rotation, tf.transform.translation
            T[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
            T[:3, 3] = [t.x, t.y, t.z]
            self.T_bd[frame] = T
        return self.T_bd[frame]

    @property
    def out_frame(self):
        """[261011] 핀 좌표계: map_frame 이 있으면 map, 아니면 odom_topic 의 frame (옛 동작)."""
        return self.map_frame or self.world

    def _M_map_odom(self):
        """지금 TF map<-odom (static — Slam 보정 또는 항등). 없으면 None."""
        try:
            tf = self.tf_buf.lookup_transform(self.map_frame, self.world, Time())
        except tf2_ros.TransformException:
            return None
        T = np.eye(4)
        q, t = tf.transform.rotation, tf.transform.translation
        T[:3, :3] = quat_to_mat(q.x, q.y, q.z, q.w)
        T[:3, 3] = [t.x, t.y, t.z]
        return T

    FINAL_RELOC = ('NEW_MAP', 'ACCEPTED', 'REJECTED', 'FAILED')

    def on_reloc(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            return
        self.reloc = d
        self.map_id = str(d.get('map_id', '') or self.map_id)
        if d.get('state') == 'ACCEPTED' and self.pins_loaded is None:
            self._load_pins(d)
        elif d.get('state') in self.FINAL_RELOC and self.pins_loaded is None:
            self.pins_loaded = ''

    def _map_ready(self):
        if not self.map_frame:
            return True
        if self.reloc is not None:
            return self.reloc.get('state') in self.FINAL_RELOC
        return time.monotonic() - self.t_start > self.reloc_wait_s    # Slam 없음으로 봄

    def _load_pins(self, d):
        """ACCEPTED: 이전 세션 핀을 미리 넣는다 (검출 시작 전에만)."""
        self.pins_loaded = ''
        if not self.pins_load:
            return
        path = self.pins_load
        if path == 'auto':
            import glob
            sess = os.path.dirname(str(d.get('map_dir', '')).rstrip('/'))
            files = sorted(glob.glob(os.path.join(sess, 'pins_*.json')))
            if not files:
                self.get_logger().warning(f'이전 핀 불러오기: {sess} 에 pins_*.json 없음 — 새로 시작')
                return
            path = files[-1]
        try:
            with open(os.path.expanduser(path)) as f:
                js = json.load(f)
        except (OSError, ValueError) as e:
            self.get_logger().error(f'이전 핀 불러오기 실패 {path}: {e}')
            return
        if js.get('frame') != self.map_frame:
            self.get_logger().warning(f'이전 핀 frame {js.get("frame")!r} != {self.map_frame!r} (옛 odom 파일?) — 안 불러옴 ({path})')
            return
        mid = str(js.get('map_id', ''))
        if mid and self.map_id and mid != self.map_id:
            self.get_logger().warning(f'이전 핀 map_id {mid} != 재추정 {self.map_id} — 안 불러옴 ({path})')
            return
        if self.pins._p:
            self.get_logger().warning('이미 검출을 쌓기 시작해 이전 핀을 안 불러옴')
            return
        n = 0
        for p in js.get('pins', []):
            self.pins.seed(p, loaded=dict(file=path, id=p.get('id'), top=p.get('top'),
                                          session=os.path.basename(os.path.dirname(path))))
            n += 1
        self.pins_loaded = path
        self.stats['pins_loaded'] = n
        self.get_logger().info(f'이전 세션 핀 {n}개 불러옴: {path} (map_id={mid or "-"})')
        if n:
            self.publish()

    def on_lc_history(self, m):
        """[261011] 루프 클로저 보정 이력 → 검출마다 옮기고 핀 중심 다시 계산 → 다시 발행."""
        hist = KR.history_from_path_msg(m)
        if hist is None:
            return

        def fn(src, p):
            r = KR.correct_point(hist, src[0], src[1], p, self.lc_tol_ns)
            if r is None:
                return None
            src[1] = r[1]                                   # 다음 보정의 기준
            return r[0]
        n, dmax = self.pins.reposition(fn)
        self.lc_count += 1
        self.stats['lc_resync'] += 1
        self.get_logger().info(f'[LC 재동기화 #{self.lc_count}] 검출 {n}개 옮김, 핀 중심 최대 {dmax * 100:.2f} cm')
        if self.pins._p:
            self.publish()

    def process(self):
        changed = False
        keep = collections.deque()
        while self.pending:
            d, t_recv = self.pending.popleft()
            r = self._try(d)
            if r == 'wait':
                if time.monotonic() - t_recv < self.wait_s:
                    keep.append((d, t_recv))
                else:
                    self.stats['drop_timeout'] += len(d['dets'])
            elif r:
                changed = True
        self.pending = keep
        if changed:
            self.publish()

    def _try(self, d):
        """'wait' = 재료가 아직 안 옴, True = 핀 갱신, False = 버림"""
        t = int(d['stamp_ns'])
        if self.K is None or self.world is None or self.odom.latest is None or self.odom.latest < t:
            return 'wait'
        if t not in self.depth:
            newest = next(reversed(self.depth)) if self.depth else -1
            if newest < t:
                return 'wait'
            self.stats['drop_no_depth'] += len(d['dets'])   # 이미 지나갔거나 buffer_s 밖
            return False
        if self.health and self.health.get('slam') == 'FAIL':
            self.stats['drop_slam_fail'] += len(d['dets'])
            return False
        if not self._map_ready():                           # [261011 밤] 재추정 확정 전 map 좌표는 바뀔 수 있음
            self.stats['skip_reloc_wait'] += len(d['dets'])
            return False
        T_ob = self.odom.pose(t, max_gap_ns=self.max_odom_gap_ns)
        if T_ob is None:
            self.stats['drop_no_pose'] += len(d['dets'])
            return False
        dm, frame = self.depth[t]
        T_bd = self._T_base_depth(frame)
        if T_bd is None:
            return 'wait'
        changed = False
        M_mo = None
        H, W = dm.shape
        for det in d['dets']:
            if float(det.get('score', 1.0)) < self.score_min:
                self.stats['drop_low_score'] += 1
                continue
            edge = box_on_edge(det['box'], W, H, self.edge_margin)
            st, Z, pc, radius = box_to_point(dm, det['box'], self.K, self.dmin, self.dmax, self.min_valid,
                                             self.depth_scale)
            if st != 'ok':
                self.stats['drop_depth_invalid'] += 1
                continue
            w = (T_ob @ T_bd @ np.append(pc, 1.0))[:3]
            src = None
            if self.map_frame:                              # [261011] 그 시각 TF map<-odom 으로 map 좌표에
                if M_mo is None:
                    M_mo = self._M_map_odom()
                    if M_mo is None:
                        return 'wait'
                w = M_mo[:3, :3] @ w + M_mo[:3, 3]
                src = [t, M_mo] if self.lc_resync else None
            pid, newly = self.pins.add(w, float(det.get('score', 1.0)), radius, t, str(det.get('class', '')),
                                       pos_weight=0.0 if edge else 1.0, box=det['box'], z=Z, src=src)
            self.stats['ok'] += 1
            self.stats['ok_edge'] += int(edge)
            changed = True
            if newly:
                c = self.pins.center(pid)
                self.get_logger().info(f'핀 확정 #{pid} {det.get("class", "")} '
                                       f'({c[0]:+.3f}, {c[1]:+.3f}, {c[2]:+.3f}) {self.out_frame}')
        return changed

    # ---------------- 출력 ----------------
    def publish(self):
        pins = self.pins.pins(self.out_frame)
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        clear = Marker()                    # 합쳐져 사라진 핀의 마커를 지우려고 매번 전체를 다시 그린다
        clear.header.frame_id, clear.header.stamp = self.out_frame, stamp
        clear.ns, clear.action = 'cda_pins', Marker.DELETEALL
        arr.markers.append(clear)
        for p in pins:
            x, y, z = p['xyz']
            r = max(p['radius_m'], 0.01)
            conf = p['confirmed']
            rgba = (0.9, 0.1, 0.1, 1.0) if conf else (1.0, 0.6, 0.0, 0.5)

            def mk(kind, k):
                m = Marker()
                m.header.frame_id, m.header.stamp = self.out_frame, stamp
                m.ns, m.id, m.type, m.action = 'cda_pins', p['id'] * 4 + k, kind, Marker.ADD
                m.pose.orientation.w = 1.0
                m.color.r, m.color.g, m.color.b, m.color.a = rgba
                return m
            disc = mk(Marker.CYLINDER, 0)
            disc.pose.position.x, disc.pose.position.y, disc.pose.position.z = x, y, z
            disc.scale.x = disc.scale.y = 2 * r
            disc.scale.z = 0.004
            disc.color.a = 0.6 if conf else 0.3
            stick = mk(Marker.CYLINDER, 1)
            stick.pose.position.x, stick.pose.position.y = x, y
            stick.pose.position.z = z + self.stick_h / 2
            stick.scale.x = stick.scale.y = 0.006
            stick.scale.z = self.stick_h
            head = mk(Marker.SPHERE, 2)
            head.pose.position.x, head.pose.position.y = x, y
            head.pose.position.z = z + self.stick_h
            head.scale.x = head.scale.y = head.scale.z = 0.03
            text = mk(Marker.TEXT_VIEW_FACING, 3)
            text.pose.position.x, text.pose.position.y = x, y
            text.pose.position.z = z + self.stick_h + 0.04
            text.scale.z = 0.03
            text.color.r = text.color.g = text.color.b = 1.0
            text.text = f"{p['cls']}#{p['id']} n={p['n']} {p['score_mean']:.2f}" + ('' if conf else ' ?')
            arr.markers += [disc, stick, head, text]
        self.pub_mk.publish(arr)
        self.pub_js.publish(String(data=json.dumps(dict(frame=self.out_frame, pins=pins))))
        self.last_pub = pins

    def report(self):
        n_conf = sum(p['confirmed'] for p in self.last_pub) if self.last_pub else 0
        self.get_logger().info(f'stats {dict(self.stats)} pending={len(self.pending)} depth_buf={len(self.depth)} '
                               f'pins={len(self.last_pub or [])} (확정 {n_conf})')

    def save(self):
        if not self.out_dir or not self.last_pub:
            return None
        os.makedirs(os.path.expanduser(self.out_dir), exist_ok=True)
        path = os.path.join(os.path.expanduser(self.out_dir), f'pins_{time.strftime("%Y%m%d_%H%M%S")}.json')
        with open(path, 'w') as f:
            json.dump(dict(frame=self.out_frame, odom_topic=self.odom_topic, det_topic=self.det_topic, lc_resync=self.lc_count,
                           map_id=self.map_id, loaded_from=self.pins_loaded or '',
                           join_m=self.pins.join_m, min_count=self.pins.min_count, stats=dict(self.stats),
                           pins=self.last_pub), f, indent=1)
        return path


def main():
    rclpy.init()
    node = None
    try:
        node = PinNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001
        # Ctrl+C 가 launch 와 터미널 양쪽에서 와서 context 가 먼저 내려가면 spin 이 RCLError
        # (failed to initialize wait set) 로 끝난다 — 260911 로그 §2 의 이중 shutdown. 종료 중이면 무시
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            p = node.save()
            if p:
                print('pins saved:', p)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
