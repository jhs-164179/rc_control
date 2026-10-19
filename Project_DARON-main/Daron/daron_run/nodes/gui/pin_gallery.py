#!/usr/bin/python3
# -*- coding: utf-8 -*-
"""
pin_gallery — CDA 핀을 RViz 에서 클릭할 수 있는 InteractiveMarker 로 그리고, 핀마다 대표 이미지를 저장한다. 작성 261006.

pin_node 는 그대로 두고(검증 끝난 노드) 그 출력만 읽는다.
입력
  pins_topic   /pin_node/pins_json  핀 목록. 핀마다 top = score 최고 검출 {stamp_ns, box, score}
  image_topic  infra1 (mono8). stamp 로 buffer_s 동안 들고 있다가 top 프레임을 찾는다
  select_topic /gui/pin_selected (Int32)  제어창 표에서 고른 핀 → 마커 강조
출력
  InteractiveMarker 서버 'pin_gallery'  (RViz: InteractiveMarkers 디스플레이, 네임스페이스 /pin_gallery)
      핀 클릭(BUTTON_CLICK) → select_topic 에 핀 id 발행 → 제어창이 정보 창을 띄운다
  ~/pins_info (String JSON, TRANSIENT_LOCAL)  핀 목록 + image(저장 경로) — 제어창이 읽는다
  out_dir/det_<stamp>.jpg   대표 이미지 (top 프레임 전체 + top 박스). 핀이 합쳐져도 stamp 로 다시 찾는다
  out_dir/rim_<stamp>.png   림 선명도 오버레이 (top 박스 crop, 파랑 = 모델폭, 노랑 = 관측폭만, 빨강 = 제외)   ┐ rim:=true 일 때 (261007)
  out_dir/rim_<stamp>.json  림 선명도 전체 결과 (sector_measurements 포함)            ┘
  pins_info 핀마다 rim = 요약(status, reason, rim_sharpness_raw, rim_completeness, ...) + top_on_edge, rim_image

림 선명도: rim/analyze_rim.py + rim/rim_sharpness_core/ (전달본 ~/Daron/rim_sharpness_gui_hybrid_20261007(2) 복사, 수정 없음,
  ESF v2 + Lorentzian 모델폭, 10/7 교체 — 이전 v1 은 backup/rim_v1_261007). 영상만 쓴다.
  오버레이는 지상국 정보 창 썸네일 크기(rim_overlay_size, 200x150 안에 비율 유지)로 그린다 — 작은 crop 을 그려 놓고 키우면 점이 깨짐(전달본 README).
  대표 crop = top 박스(score 최고 검출)를 박스 그리기 전 원본 프레임에서 자른 것. 실패(pending)는 0점이 아니다.

크레이터 경사·깊이·메시 (crater:=true, 261009): crater/crater_analysis/ (전달본 ~/Daron/crater_slope_depth_mesh_code_only 복사, 수정 없음).
  **관측이 끝난 크레이터만 한 번 계산한다** (CDA 매 프레임 계산 금지 — 요청서 261009).
  관측 중(ACTIVE)   핀의 top(score 최고 검출 = pin_node 대표 이미지 선택 그대로)이 바뀔 때마다 그 stamp 의
                    infra1 영상·raw depth(16UC1)·depth K·같은 프레임 박스 전부를 스냅샷으로 복사해 둔다
                    (버퍼에서 지워져도 대표 프레임과 같은 시각 depth 를 쓰게). geometry 는 계산하지 않는다.
  관측 종료         지금 센서 시각 − last_observed(핀 detections 의 마지막 stamp) > crater_finalize_timeout_s
                    → 워커 스레드 1개에서 계산 (FINALIZING)
  후보 (261009 밤)  score 최고 프레임은 계산에 안 맞는 경우가 많다(real_output: 최고 score 성공 0/11 묶음).
                    pin_node 의 cands(가장자리 아님·Z 0.35~1.0 m, score 상위 10 — real_output 에서 후보 4~5장에 1장꼴 성공) + 마지막에 top 을 score 순으로 시도,
                    처음 성공한 프레임 = 확정 대표 이미지(rep_box.jpg, 정보 창 이미지도 이걸로). 결과는 프레임별 캐시 —
                    다시 관측돼 더 높은 후보가 생기면 그것만 새로 시도. 전부 실패 = INSUFFICIENT_OBSERVATION(사유 모음).
                    지금 센서 시각 = 마지막으로 받은 영상·depth·검출 stamp + 그 뒤 흐른 벽시계 (bag 이 멈춰도 끝남)
  계산              직경 = pin_node 와 같은 bbox 가로폭 × depth(lib/pins.box_to_point) → analyze_crater(diameter_m=) 로 넘김
                    (box_to_point 가 실패하면 전달본 bbox 접원 직경, diameter_source 에 기록). 경사·깊이·메시 = 전달본 그대로.
  결과              COMPLETE | INSUFFICIENT_OBSERVATION (전달본이 거부 = 유효 depth·내벽 범위 부족 — 값을 만들지 않음, reason 기록)
  다시 관측         새 검출이 오면 ACTIVE 로. 끝났을 때 top stamp 가 같으면 다시 계산하지 않음(관측 수만 갱신), 바뀌었으면 다시 계산.
  Z 축              시험장 = 실제 크레이터 지형 Z × crater_z_exaggeration(기본 5). 측정값을 그대로 쓰고 메시도 측정 높이 그대로(배율 1).
                    전달본은 고정 d/D 같은 '일반 크레이터' 가정이 없고 측정 내벽으로 포물면을 맞추므로, 추정도 자동으로 시험장(×5) 척도다.
                    배율은 결과에 기록만 한다 (다시 곱하면 ×25 가 됨).
  out_dir/crater_<stamp>_b<x1>_<y1>/   전달본 출력 그대로(result.json·completed_mesh.npz·*.obj ...) + geometry.json(아래 키)
                            + 대표 입력 rep_infra1.png · rep_depth.png(16UC1) · camera.json · boxes.json (전달본 CLI 로 다시 돌릴 수 있게)
  pins_info 핀마다 crater = {state, last_observed_ago_s, observation_count, representative_stamp, representative_bbox,
                            [geometry_status, diameter_m, max_inner_wall_slope_deg, predicted_depth_m, depth_to_diameter_ratio,
                             mesh_path, result_dir, reason, ...]}
  out_dir/pins_info.json    pins_info 와 같은 내용 (바뀔 때마다 덮어씀)

열화도 순번 (degradation:=true, 261013): gui/degradation/ (전달본 crater_degradation_equalweight_20261013 복사, 수정 없음)
  + gui/degradation_bridge.py (연결부). D = 50·(1−u경사P95) + 50·(1−u림S), 작을수록 열화 덜 됨 → 순번 1번.
  측정       관측이 끝난 핀(ACTIVE 아님)의 cands + top + blocks(pin_node 의 5 s 구간별 검출) 프레임마다 한 번,
             크레이터 워커에서 (형상 계산 뒤). 같은 stamp 스냅샷의 depth → 관측 내벽 경사 P95 (전달본 measure_observed),
             IR crop → rim_sharpness_raw. 프레임 키(stamp, 박스)별 1회 — 다시 관측돼 새 프레임이 생기면 그것만.
  기록       out_dir/degradation_observations.sqlite (전달본 ObservationStore — 실패한 측정도 사유와 함께 남김)
  순서       전달본 equalweight.evaluate. ranked(확정) / preview(보류) 를 D 순으로 한 줄에, 상태를 같이 표시.
             확정 = 5 s 구간 5개·20 s 이상 반복 관측 + identity_confirmed(핀 확정) + acquisition_verified(파라미터).
             기준 범위: degradation_reference(전달본 calibrate 로 만든 reference.json) 가 없으면 이번 미션 크레이터들의
             최소·최대로 임시 기준 (크레이터가 늘면 값·순서가 바뀜 — 정보 창에 표시).
             **D 를 못 만든 크레이터(경사·림 중 하나라도 없음, 품질 게이트 탈락)는 순서에 넣지 않는다** — 핀별 사유만.
  pins_info  최상위 degradation = {order:[{order, pin, status, D, slope_deg, rim_score, ...}], reference, ...}
             핀마다 degradation = {status: observing|measuring|ranked|preview|no_index, order, total, D, ..., reasons}
  out_dir/degradation_ranking.json  마지막 순서 결과

마커는 바뀐 핀만 다시 넣고, update_period_s 보다 자주 반영하지 않는다 (RViz 부하).
"""

import collections
import concurrent.futures
import json
import os
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from interactive_markers import InteractiveMarkerServer
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Int32, String
from visualization_msgs.msg import InteractiveMarker, InteractiveMarkerControl, InteractiveMarkerFeedback, Marker

sys.path.insert(0, str(Path(__file__).resolve().parent / 'rim'))
from analyze_rim import analyze_representative_crop  # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parent / 'crater'))
from crater_analysis import Box as CraterBox, Camera as CraterCamera, analyze_crater, warmup as crater_warmup  # noqa: E402
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))      # nodes/ (lib 패키지)
from lib.pins import box_to_point  # noqa: E402  (pin_node 와 같은 직경 계산)
sys.path.insert(0, str(Path(__file__).resolve().parent))
import degradation_bridge as DB  # noqa: E402  [261013] 열화도 순번

ACTIVE, FINALIZING, COMPLETE, INSUFFICIENT = 'ACTIVE', 'FINALIZING', 'COMPLETE', 'INSUFFICIENT_OBSERVATION'


def rep_key(top):
    """대표 프레임 키 = (stamp, bbox). 한 프레임에 크레이터가 여럿이면 stamp 가 같으므로 박스까지 봐야 한다."""
    return int(top['stamp_ns']), tuple(round(float(v), 1) for v in top['box'])

RIM_KEYS = ('algorithm', 'status', 'reason', 'rim_sharpness_raw', 'rim_completeness', 'visible_arc_fraction',
            'accepted_sectors', 'total_sectors', 'observation_quality', 'input_width', 'input_height',
            # hybrid v2 (10/7): 관측폭·모델폭 (단위 = bbox 반경 정규화, 못 구하면 None — 0 으로 바꾸지 않음)
            'median_edge_width_r', 'width_status', 'width_reason',
            'lorentzian_median_width_r', 'model_width_status', 'model_width_reason',
            'lorentzian_fitted_sectors', 'observed_only_sectors', 'rejected_selected_sectors')


def stamp_ns(h):
    return h.stamp.sec * 1_000_000_000 + h.stamp.nanosec


class PinGallery(Node):
    def __init__(self):
        super().__init__('pin_gallery')
        P = self.declare_parameter
        self.pins_topic = P('pins_topic', '/pin_node/pins_json').value
        image_topic = P('image_topic', '/camera/camera/infra1/image_rect_raw').value
        self.select_topic = P('select_topic', '/gui/pin_selected').value
        self.buffer_s = float(P('buffer_s', 5.0).value)        # pin_node 는 검출 뒤 최대 ~1 s(wait_s) 안에 핀을 낸다
        out_dir = P('out_dir', '').value
        self.update_period = float(P('update_period_s', 0.5).value)
        self.stick_h = float(P('stick_h', 0.15).value)
        self.rim_on = bool(P('rim', True).value)
        self.edge_margin = float(P('edge_margin', 2.0).value)  # pin_node 와 같은 값 — top 박스가 잘렸는지 표시용
        self.rim_ov_box = [int(v) for v in P('rim_overlay_size', [200, 150]).value]   # ground_station 정보 창 썸네일 크기
        self.crater_on = bool(P('crater', True).value)                                  # [261009] 크레이터 경사·깊이·메시
        depth_topic = P('depth_topic', '/camera/camera/depth/image_rect_raw').value
        depth_info_topic = P('depth_info_topic', '/camera/camera/depth/camera_info').value
        det_topic = P('det_topic', '/cda/detections').value
        self.depth_scale = float(P('depth_scale', 0.001).value)                        # pin_node·live_tsdf 와 같은 값
        self.finalize_s = float(P('crater_finalize_timeout_s', 2.5).value)            # 마지막 검출 뒤 이만큼 없으면 관측 종료
        self.z_exag = float(P('crater_z_exaggeration', 5.0).value)                     # 시험장 Z 증폭 (기록만, 위 설명)
        self.dmin = float(P('dmin', 0.07).value)                                       # 직경 = pin_node box_to_point 와 같은 값
        self.dmax = float(P('dmax', 1.0).value)
        self.min_valid = float(P('min_valid', 0.3).value)
        self.diam_src = str(P('crater_diameter_source', 'pin').value)                  # pin = 요청서(기존 bbox 가로폭×depth) | code = 전달본 bbox 접원
        if self.diam_src not in ('pin', 'code'):
            raise ValueError(f'crater_diameter_source 는 pin/code: {self.diam_src!r}')
        self.out_dir = os.path.expanduser(out_dir or time.strftime('~/daron_gui_runs/%Y%m%d_%H%M%S/pins'))
        os.makedirs(self.out_dir, exist_ok=True)
        # [261013] 열화도 순번 (위 설명). 측정은 크레이터 워커에서 하므로 crater:=true 일 때만
        self.deg_on = bool(P('degradation', True).value) and self.crater_on
        ref_path = os.path.expanduser(str(P('degradation_reference', '').value))
        pol_path = os.path.expanduser(str(P('degradation_policy', '').value))
        self.deg_verified = bool(P('degradation_acquisition_verified', False).value)   # 자세·조명·내벽 구간 비교 가능성을 확인했을 때만 true
        self.deg_mission = str(P('degradation_mission_id', '').value) or os.path.basename(os.path.dirname(self.out_dir.rstrip('/')))
        self.deg_protocol = dict(protocol_id=str(P('degradation_protocol_id', 'daron-d405-848x480-v1').value),
                                 comparison_group=str(P('degradation_comparison_group', 'k-src-2026').value),
                                 view_cell=str(P('degradation_view_cell', 'fixed_mount_exp8500g16').value),
                                 support_id=str(P('degradation_support_id', 'observed_inner_wall').value))
        self.deg_key = tuple(self.deg_protocol[k] for k in ('protocol_id', 'comparison_group', 'view_cell', 'support_id'))
        self.deg_policy = DB.load_policy(pol_path or None)
        self.deg_ref = DB.load_reference(ref_path) if ref_path else None
        if self.deg_ref is not None and tuple(self.deg_ref.protocol_key) != self.deg_key:
            raise ValueError(f'degradation_reference 의 protocol_key {list(self.deg_ref.protocol_key)} ≠ 이 노드 {list(self.deg_key)}')

        self.images = collections.OrderedDict()                # stamp_ns -> Image
        self.saved = {}                                        # stamp_ns -> 파일 경로
        self.rim = {}                                          # stamp_ns -> 림 요약 dict (RIM_KEYS + top_on_edge)
        self.rim_img = {}                                      # stamp_ns -> 오버레이 경로
        self.pending = {}                                      # stamp_ns -> 핀 id (핀이 영상보다 먼저 온 경우)
        self.pins = {}                                         # id -> 핀 dict (pins_json 그대로)
        self.frame = 'odom'
        self.want = {}                                         # id -> 마커 서명 (다음 반영 때 그릴 것)
        self.shown = {}                                        # id -> 지금 서버에 들어간 서명
        self.selected = None
        self.stats = collections.Counter()
        # [261009] 크레이터 분석 — 결과는 워커 스레드가 쓰고 apply() 타이머가 발행한다
        self.depth = collections.OrderedDict()                 # stamp_ns -> u16 HxW
        self.dets = collections.OrderedDict()                  # stamp_ns -> [box, ...] (같은 프레임 크레이터 박스 전부)
        self.K_depth = None
        self.snap = {}                                         # rep_key -> 대표 프레임 스냅샷 dict, None = 재료 놓침
        self.snap_wait = {}                                    # rep_key -> 핀 id (영상·depth 가 아직 안 옴)
        self.geo = {}                                          # rep_key -> geometry 결과 dict (COMPLETE / INSUFFICIENT)
        self.geo_running = set()                               # rep_key
        self.pin_state = {}                                    # 핀 id -> 지난번 상태 (바뀌면 발행)
        self.clock = (None, 0.0)                               # (마지막 센서 stamp_ns, 그때 벽시계)
        self.crater_lock = threading.Lock()
        self.crater_dirty = False
        self.crater_pool = concurrent.futures.ThreadPoolExecutor(max_workers=1, thread_name_prefix='crater')
        # [261013] 열화도 — deg_done·deg_running·deg_result 는 crater_lock 으로 (워커가 씀)
        self.deg_done = set()                                  # rep_key (측정 끝, 성공·실패 무관)
        self.deg_running = set()                               # rep_key
        self.deg_result = None                                 # DB.rank_order 결과
        self.deg_aliases = {}                                  # 마지막으로 순서 계산에 쓴 핀 합침 (바뀌면 다시 계산)
        self.deg_store = None                                  # ObservationStore — sqlite 연결은 워커 스레드에서만 연다
        if self.crater_on:                                     # 워커 첫 작업 = 가속 워밍업 (분석보다 항상 먼저, 기동은 안 막음)
            self.crater_pool.submit(self._crater_warmup)

        # 영상: cda_overlay 와 같은 이유로 큐를 길게 (sensor_data 기본 depth 5 면 짝 영상이 밀려남)
        img_qos = QoSProfile(depth=30, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Image, image_topic, self.on_image, img_qos)
        self.create_subscription(String, self.pins_topic, self.on_pins, latched)
        self.create_subscription(Int32, self.select_topic, self.on_select, 10)
        if self.crater_on:
            self.create_subscription(Image, depth_topic, self.on_depth, img_qos)
            self.create_subscription(CameraInfo, depth_info_topic, self.on_depth_info, img_qos)
            self.create_subscription(String, det_topic, self.on_det, 50)
        self.pub_sel = self.create_publisher(Int32, self.select_topic, 10)
        self.pub_info = self.create_publisher(String, '~/pins_info', latched)
        self.server = InteractiveMarkerServer(self, 'pin_gallery')
        self.create_timer(self.update_period, self.apply)
        self.create_timer(5.0, self.report)
        self.get_logger().info(f'{self.pins_topic} + {image_topic} -> InteractiveMarker /pin_gallery, '
                               f'이미지 {self.out_dir}, 림 선명도 {"켬" if self.rim_on else "끔"}, '
                               f'크레이터 분석 {"켬 (" + depth_topic + f", 관측 종료 {self.finalize_s} s)" if self.crater_on else "끔"}, '
                               f'열화도 {"켬 (기준 " + (ref_path if self.deg_ref else "임시=미션 범위") + f", mission {self.deg_mission})" if self.deg_on else "끔"}')
        self._publish_info()                                   # 핀이 생기기 전에도 제어창이 '노드 있음, 핀 0개'로 보게

    # ---------------- 입력 ----------------
    def on_image(self, m):
        t = stamp_ns(m.header)
        self.images[t] = m
        self._tick_clock(t)
        if self.crater_on:
            for k in [k for k in self.snap_wait if k[0] == t]:
                self._snapshot(self.pins.get(self.snap_wait[k]))
        while self.images and next(iter(self.images)) < t - int(self.buffer_s * 1e9):
            self.images.popitem(last=False)
        pid = self.pending.pop(t, None)
        if pid is not None and pid in self.pins:
            self._save_top(self.pins[pid])
            self._publish_info()
        for k in [k for k in self.pending if k < t - int(self.buffer_s * 1e9)]:
            del self.pending[k]
            self.stats['image_missing'] += 1

    def on_select(self, m):
        if m.data != self.selected:
            self.selected = m.data
            self._refresh_want()

    def on_pins(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            self.stats['bad_json'] += 1
            return
        self.frame = d.get('frame') or self.frame
        self.pins = {int(p['id']): p for p in d.get('pins', [])}
        for p in self.pins.values():
            self._save_top(p)
            self._snapshot(p)
        if self.crater_on:                                     # 어느 핀의 후보도 아닌 스냅샷은 버린다 (계산 중인 것은 둠)
            keep = {rep_key(e) for p in self.pins.values() for e in self._entries(p)}
            with self.crater_lock:                             # [261013] 열화도 측정 전인 프레임도 둔다
                keep |= {k for p in self.pins.values() for k in map(rep_key, self._deg_entries(p))
                         if k not in self.deg_done} | self.deg_running
            for k in [k for k in self.snap if k not in keep and k not in self.geo_running]:
                del self.snap[k]
            for k in [k for k in self.snap_wait if k not in keep]:
                del self.snap_wait[k]
        self._refresh_want()
        self._publish_info()

    def on_depth(self, m):
        if m.encoding not in ('16UC1', 'mono16'):
            self.stats['depth_bad_encoding'] += 1
            return
        t = stamp_ns(m.header)
        self.depth[t] = np.frombuffer(m.data, np.uint16).reshape(m.height, m.step // 2)[:, :m.width].copy()
        while self.depth and next(iter(self.depth)) < t - int(self.buffer_s * 1e9):
            self.depth.popitem(last=False)
        self._tick_clock(t)
        for k in [k for k in self.snap_wait if k[0] == t]:
            self._snapshot(self.pins.get(self.snap_wait[k]))

    def on_depth_info(self, m):
        if self.K_depth is None:
            self.K_depth = np.array(m.k, dtype=np.float64).reshape(3, 3)

    def on_det(self, m):
        try:
            d = json.loads(m.data)
            t = int(d['stamp_ns'])
            self.dets[t] = [[float(v) for v in det['box']] for det in d.get('dets', [])]
        except (ValueError, KeyError, TypeError):
            self.stats['bad_det_json'] += 1
            return
        while self.dets and next(iter(self.dets)) < t - int(self.buffer_s * 1e9):
            self.dets.popitem(last=False)
        self._tick_clock(t)

    # ---------------- 크레이터: 관측 중 스냅샷, 관측 종료 후 1회 계산 (261009) ----------------
    def _tick_clock(self, t):
        if self.clock[0] is None or t > self.clock[0]:
            self.clock = (t, time.monotonic())

    def _now_ns(self):
        """지금 센서 시각 = 마지막 센서 stamp + 그 뒤 흐른 벽시계 (재생이 끝나거나 카메라가 멈춰도 관측 종료가 난다)."""
        t, w = self.clock
        return None if t is None else t + int((time.monotonic() - w) * 1e9)

    @staticmethod
    def _entries(p):
        """형상 계산 후보 (시도 순서). pin_node 의 cands(가장자리 아님·Z 범위, score 내림차순) + 마지막에 top.
        cands 가 없는 옛 pin_node 면 top 하나 — 이전 동작."""
        out = list(p.get('cands') or [])
        top = p.get('top')
        if top and all(rep_key(e) != rep_key(top) for e in out):
            out.append(top)
        return out

    def _deg_entries(self, p):
        """[261013] 열화도 측정 프레임 = 형상 후보(cands + top) + 5 s 구간별 검출(blocks). 같은 프레임은 한 번."""
        if not self.deg_on:
            return []
        out = self._entries(p)
        seen = {rep_key(e) for e in out}
        for e in p.get('blocks') or []:
            if rep_key(e) not in seen:
                seen.add(rep_key(e))
                out.append(e)
        return out

    def _snapshot(self, p):
        """핀의 후보 프레임마다 영상·raw depth·K·같은 프레임 박스를 복사해 둔다. 계산은 하지 않는다."""
        if not self.crater_on or not p:
            return
        with self.crater_lock:
            done = set(self.deg_done)
        geo = {rep_key(e) for e in self._entries(p)}
        for e in self._deg_entries(p) or self._entries(p):
            if rep_key(e) in geo or rep_key(e) not in done:     # 열화도만 쓰는 프레임은 측정이 끝났으면 다시 안 잡음
                self._snapshot_one(p['id'], e)

    def _snapshot_one(self, pid, e):
        key = rep_key(e)
        t = key[0]
        if key in self.snap:
            return
        img, dm = self.images.get(t), self.depth.get(t)
        if img is None or dm is None or self.K_depth is None:
            newest = min(next(reversed(self.images)) if self.images else -1, next(reversed(self.depth)) if self.depth else -1)
            if t > newest or self.K_depth is None:
                self.snap_wait[key] = pid                       # 아직 안 옴 → on_image / on_depth 에서 다시
            else:
                self.snap[key] = None                           # 버퍼 밖 → 이 프레임으로는 계산 불가
                self.snap_wait.pop(key, None)
                self.stats['snap_missing'] += 1
            return
        self.snap_wait.pop(key, None)
        self.snap[key] = dict(key=key, stamp_ns=t, box=[float(v) for v in e['box']], score=float(e['score']),
                              image=self._to_bgr(img), depth=dm.copy(), K=self.K_depth.copy(),
                              boxes=[list(b) for b in (self.dets.get(t) or [e['box']])])
        self.stats['snap'] += 1

    def _pick(self, p):
        """후보를 순서대로 훑어 (성공 결과, 아직 안 해 본 후보 키 목록, 실패 결과 목록, 계산 중 여부)."""
        todo, fails, running = [], [], False
        with self.crater_lock:
            for e in self._entries(p):
                k = rep_key(e)
                g = self.geo.get(k)
                if k in self.geo_running:
                    running = True
                if g is not None and g['geometry_status'] == COMPLETE:
                    return g, todo, fails, running              # todo 가 남아 있으면(더 높은 후보 미시도) 아직 확정 아님
                if g is not None:
                    fails.append(g)
                elif k not in self.geo_running:
                    todo.append(k)
        return None, todo, fails, running

    def _crater_view(self, p, now):
        """핀 하나의 crater 정보 (pins_info 용). 상태 판단도 여기서."""
        top = p.get('top')
        t = int(top['stamp_ns']) if top else None
        dets = p.get('detections') or ([t] if t else [])
        last = max(dets) if dets else None
        n_cand = len(p.get('cands') or [])
        view = dict(observation_count=int(p.get('n', 0)), candidate_count=n_cand,
                    representative_stamp=t, representative_bbox=top['box'] if top else None,
                    representative_score=float(top['score']) if top else None,
                    last_observed_ago_s=None if (now is None or last is None) else round((now - last) / 1e9, 2))
        g, todo, fails, running = self._pick(p)
        if last is not None and now is not None and now - last <= self.finalize_s * 1e9:
            view['state'] = ACTIVE
        elif g is not None and not todo and not running:
            view['state'] = COMPLETE
        elif todo or running:
            view['state'] = FINALIZING
        elif fails:
            view['state'] = INSUFFICIENT
        else:
            view['state'] = INSUFFICIENT
            view['reason'] = 'no_representative_image'
        if g is not None:                                       # 다시 관측 중이어도 지난 결과는 보여 준다
            view.update({k: v for k, v in g.items() if k != 'observation_count'})
            view['observation_count_at_geometry'] = g['observation_count']
        elif fails and view['state'] == INSUFFICIENT:
            reasons = collections.Counter(f.get('reason') for f in fails)
            view.update(geometry_status=INSUFFICIENT, tried=len(fails),
                        reason=(f'후보 {len(fails)}장 모두 실패: ' if len(fails) > 1 else '')
                        + ', '.join(f'{r} ×{c}' if c > 1 else str(r) for r, c in reasons.most_common()))
        return view, todo, last

    def _finalize_tick(self):
        """관측이 끝난 핀을 찾아 후보 계산을 워커에 넣는다. 상태가 바뀐 핀이 있으면 발행."""
        if not self.crater_on:
            return False
        now = self._now_ns()
        changed = False
        for pid, p in self.pins.items():
            view, todo, last = self._crater_view(p, now)
            if view['state'] != self.pin_state.get(pid):
                self.get_logger().info(f'크레이터 #{pid} {self.pin_state.get(pid, "-")} -> {view["state"]} '
                                       f'(후보 {view["candidate_count"]}, 관측 {view["observation_count"]})'
                                       + (f' {view.get("reason")}' if view['state'] == INSUFFICIENT else ''))
                self.pin_state[pid] = view['state']
                changed = True
            if view['state'] != ACTIVE:
                self._deg_submit(pid, p)                        # [261013] 관측이 끝난 핀의 열화도 측정 (형상 계산 뒤 같은 워커)
            if view['state'] != FINALIZING or not todo:
                continue
            chain = []
            ent = {rep_key(e): e for e in self._entries(p)}
            for k in todo:
                if k in self.snap_wait:
                    self._snapshot_one(pid, ent[k])             # 다시 확인 (그 stamp depth 가 끝내 안 왔으면 None)
                    if k in self.snap_wait:
                        break                                   # 이 후보 재료가 아직 오는 중 — 순서를 지키려고 기다림
                chain.append((k, self.snap.get(k)))
            if not chain:
                continue
            with self.crater_lock:
                self.geo_running.update(k for k, _ in chain)
            self.crater_pool.submit(self._geo_chain, chain, int(p.get('n', 0)))
        for pid in [k for k in self.pin_state if k not in self.pins]:
            del self.pin_state[pid]
        if self.deg_on:                                         # 핀이 합쳐지면 측정 없이도 순서를 다시 (합친 id 로 묶음)
            aliases = self._aliases()
            with self.crater_lock:
                redo = aliases != self.deg_aliases and bool(self.deg_done) and not self.deg_running
                if redo:
                    self.deg_aliases = aliases
            if redo:
                self.crater_pool.submit(self._deg_chain, None, [], aliases)
        return changed

    # ---------------- 열화도: 관측 종료 후 프레임별 측정 → 순서 (261013) ----------------
    def _aliases(self):
        return {str(m): str(p['id']) for p in self.pins.values() for m in p.get('merged_from') or []}

    def _deg_submit(self, pid, p):
        if not self.deg_on:
            return
        chain = []
        with self.crater_lock:
            busy = self.deg_done | self.deg_running
        for e in self._deg_entries(p):
            k = rep_key(e)
            if k in busy:
                continue
            if k in self.snap_wait:
                self._snapshot_one(pid, e)
                if k in self.snap_wait:
                    continue                                    # 재료가 아직 오는 중 — 다음 tick
            if k not in self.snap:
                self._snapshot_one(pid, e)
            chain.append((k, self.snap.get(k)))
        if not chain:
            return
        aliases = self._aliases()
        with self.crater_lock:
            self.deg_running.update(k for k, _ in chain)
            self.deg_aliases = aliases
        self.crater_pool.submit(self._deg_chain, (pid, bool(p.get('confirmed'))), chain, aliases)

    def _deg_chain(self, who, chain, aliases):
        """워커 스레드: 프레임마다 경사 P95·림 S 측정 → 관측 기록 → 전체 순서 다시. who=None 이면 순서만 다시."""
        t0 = time.monotonic()
        try:
            if self.deg_store is None:
                from crater_rank.store import ObservationStore
                self.deg_store = ObservationStore(os.path.join(self.out_dir, 'degradation_observations.sqlite'))
            ok = 0
            for key, snap in chain:
                try:
                    if snap is None:                            # 그 stamp 의 영상·depth 가 버퍼에 없었음 → 측정 불가 (기록할 값 없음)
                        self.stats['deg_snap_missing'] += 1
                        continue
                    geo, rim, q, err = DB.measure_frame(snap['image'], snap['depth'], snap['K'], snap['box'], snap['boxes'],
                                                        depth_scale=self.depth_scale, edge_margin=self.edge_margin)
                    obs = DB.observation(mission_id=self.deg_mission, crater_id=who[0], stamp_ns=snap['stamp_ns'],
                                         box=snap['box'], geometry=geo, rim=rim, quality=q, errors=err,
                                         protocol=self.deg_protocol, identity_confirmed=who[1],
                                         acquisition_verified=self.deg_verified)
                    self.deg_store.add(obs)
                    both = obs['slope_deg'] is not None and obs['rim_score'] is not None
                    ok += both
                    self.stats['deg_obs_both' if both else 'deg_obs_partial'] += 1
                except Exception as e:  # noqa: BLE001  한 프레임 실패가 나머지·핀·형상을 막지 않게
                    self.stats['deg_error'] += 1
                    self.get_logger().warn(f'열화도 측정 오류 stamp={key[0]}: {type(e).__name__}: {e}')
                finally:
                    with self.crater_lock:
                        self.deg_done.add(key)
                        self.deg_running.discard(key)
            res = DB.rank_order(self.deg_store.observations(), self.deg_key, self.deg_policy, self.deg_ref, aliases)
            with open(os.path.join(self.out_dir, 'degradation_ranking.json'), 'w', encoding='utf-8') as f:
                json.dump(res, f, ensure_ascii=False, indent=1)
            with self.crater_lock:
                self.deg_result = res
                self.crater_dirty = True
            if who is not None:
                self.get_logger().info(f'열화도 #{who[0]} 프레임 {len(chain)}장 측정 (두 값 {ok}장), 순서 {len(res["order"])}개 '
                                       f'({time.monotonic() - t0:.2f} s)' + (f' — {res["note"]}' if res.get('note') else ''))
        except Exception as e:  # noqa: BLE001
            with self.crater_lock:
                self.deg_running.difference_update(k for k, _ in chain)
                self.deg_done.update(k for k, _ in chain)
            self.stats['deg_error'] += 1
            self.get_logger().error(f'열화도 순서 계산 오류: {type(e).__name__}: {e}')

    def _deg_view(self, pid, state, res, ordered):
        """핀 하나의 degradation (pins_info 용). ordered = _deg_order 의 이 핀 줄 (순서에 없으면 None)."""
        with self.crater_lock:
            keys = {rep_key(e) for e in self._deg_entries(self.pins[pid])}
            running = bool(keys & self.deg_running)
            pending = bool(keys - self.deg_done)
        if ordered is not None:
            return dict(ordered, measuring=running or state == ACTIVE)
        row = (res or {}).get('per_crater', {}).get(str(pid))
        if state == ACTIVE:                                     # 다시 관측 중 — 끝나면 새 프레임까지 넣어 다시 계산
            return dict(status='observing')
        if running or (pending and row is None):
            return dict(status='measuring')
        if row is None or row.get('status') != 'no_index':
            return dict(status='no_index', reasons={}, note='측정한 프레임 없음 (대표 프레임 영상·depth 를 놓침)')
        return dict(row, note=row.get('note') or (res or {}).get('note'))

    def _crater_warmup(self):
        """워커 스레드: numba 커널 미리 컴파일. numba 가 없으면 numpy 경로 (계산 결과 같음). 백엔드는 CRATER_BACKEND(기본 auto)."""
        t0 = time.monotonic()
        try:
            st = crater_warmup()
        except Exception as e:  # noqa: BLE001  워밍업이 실패해도 분석은 그대로 돈다
            self.get_logger().warn(f'크레이터 가속 워밍업 실패: {type(e).__name__}: {e}')
            return
        self.get_logger().info(f'크레이터 가속 backend={st["active"]} numba={st["numba_version"]} '
                               f'({time.monotonic() - t0:.1f} s)'
                               + (f' fallback: {st["fallback_reason"]}' if st['fallback_reason'] else ''))

    def _geo_chain(self, chain, n_obs):
        """워커 스레드: 후보를 score 순으로 하나씩, 처음 성공하면 멈춘다. 프레임마다 결과를 캐시(같은 후보 재계산 없음)."""
        for i, (key, snap) in enumerate(chain):
            if snap is None:                                    # 그 프레임과 같은 시각 depth 가 없음 → 만들지 않음
                self._geo_set(key, dict(geometry_status=INSUFFICIENT, reason='representative_depth_missing',
                                        observation_count=n_obs, representative_stamp=key[0]))
                continue
            out = os.path.join(self.out_dir, f'crater_{key[0]}_b{int(key[1][0])}_{int(key[1][1])}')
            if self._geo_run(snap, n_obs, out) == COMPLETE:
                with self.crater_lock:                          # 나머지는 안 해 봄 (캐시 없음 → 앞 후보가 성공이라 필요 없음)
                    self.geo_running.difference_update(k for k, _ in chain[i + 1:])
                    self.crater_dirty = True
                return

    def _geo_run(self, snap, n_obs, out):
        """프레임 하나로 직경 → 경사 → 깊이 → 메시 (전달본 analyze_crater). 반환 geometry_status."""
        t, t0 = snap['stamp_ns'], time.monotonic()
        dm, K, box = snap['depth'], snap['K'], snap['box']
        base = dict(observation_count=n_obs, representative_stamp=t, representative_bbox=box,
                    representative_score=snap['score'], terrain_z_exaggeration=self.z_exag, z_scale_applied=1.0)
        try:
            os.makedirs(out, exist_ok=True)
            cv2.imwrite(os.path.join(out, 'rep_infra1.png'), snap['image'])
            cv2.imwrite(os.path.join(out, 'rep_depth.png'), dm)
            with open(os.path.join(out, 'camera.json'), 'w') as f:
                json.dump(dict(width=int(dm.shape[1]), height=int(dm.shape[0]), K=K.tolist()), f)
            st, _, _, radius = box_to_point(dm, box, K, self.dmin, self.dmax, self.min_valid, self.depth_scale)
            diam = 2 * radius if (st == 'ok' and self.diam_src == 'pin') else None
            tb = CraterBox(*box)
            objs = [CraterBox(*b) for b in snap['boxes']]
            idx = max(range(len(objs)), key=lambda i: objs[i].iou(tb)) if objs else -1
            if idx < 0 or objs[idx].iou(tb) < 0.99:             # 같은 프레임 박스 목록에 이 박스가 없으면 더한다
                objs.append(tb)
                idx = len(objs) - 1
            with open(os.path.join(out, 'boxes.json'), 'w') as f:
                json.dump([list(b.xyxy) for b in objs], f)
            depth_m = dm.astype(np.float64) * self.depth_scale
            depth_m[dm == 0] = np.nan
            depth_m[dm >= 60000] = np.nan                       # 센서 무효값 (CDA 노드와 같은 규칙). 전달본은 거리 상한이 없다
            res = analyze_crater(depth_m, CraterCamera(int(dm.shape[1]), int(dm.shape[0]), K), objs, idx, out,
                                 frame_id=t, diameter_m=diam, preview=False)
            img = snap['image'].copy()                          # 확정 대표 이미지 = 계산에 쓴 프레임 + 박스
            x1, y1, x2, y2 = (int(round(v)) for v in box)
            cv2.rectangle(img, (x1, y1), (x2, y2), (60, 220, 60), 2)
            cv2.imwrite(os.path.join(out, 'rep_box.jpg'), img, [cv2.IMWRITE_JPEG_QUALITY, 90])
            g = dict(base, geometry_status=COMPLETE, reason=None,
                     diameter_m=res['diameter_m'],
                     diameter_source='pin_bbox_width_x_depth' if diam is not None else
                     ('crater_code_bbox_circle' if self.diam_src == 'code' else f'crater_code_bbox_circle (pin 실패: {st})'),
                     diameter_pin_m=2 * radius if st == 'ok' else None,
                     diameter_code_bbox_m=self._code_bbox_diameter(out),
                     max_inner_wall_slope_deg=res['observed_max_inner_wall_slope_deg'],
                     predicted_depth_m=res['estimated_depth_m'],
                     depth_to_diameter_ratio=res['depth_to_diameter_ratio'],
                     depth_is_model_estimated=res['depth_is_model_estimated'],
                     quality_flags=res['quality_flags'],
                     mesh_path=os.path.join(out, 'completed_mesh.npz'), result_dir=out,
                     representative_image=os.path.join(out, 'rep_box.jpg'))
        except ValueError as e:                                 # 전달본의 거부 = 이 프레임으론 관측 부족. 값을 만들지 않는다
            g = dict(base, geometry_status=INSUFFICIENT, reason=str(e), result_dir=out)
        except Exception as e:  # noqa: BLE001  분석이 죽어도 핀·이미지·림은 계속
            g = dict(base, geometry_status=INSUFFICIENT, reason=f'error {type(e).__name__}: {e}', result_dir=out)
            self.get_logger().error(f'크레이터 계산 오류 stamp={t}: {type(e).__name__}: {e}')
        g['elapsed_s'] = round(time.monotonic() - t0, 2)
        try:
            with open(os.path.join(out, 'geometry.json'), 'w', encoding='utf-8') as f:
                json.dump(g, f, ensure_ascii=False, indent=2)
        except OSError as e:
            self.get_logger().warn(f'geometry.json 저장 실패: {e}')
        self._geo_set(snap['key'], g)
        self.get_logger().info(f'크레이터 geometry {g["geometry_status"]} stamp={t} {g["elapsed_s"]} s'
                               + (f' ({g["reason"]})' if g.get('reason') else ''))
        return g['geometry_status']

    @staticmethod
    def _code_bbox_diameter(out):
        """전달본이 bbox 접원으로 구한 직경 (비교용 기록). metrics.json 의 bbox_diameter_m."""
        try:
            with open(os.path.join(out, 'metrics.json'), encoding='utf-8') as f:
                return json.load(f).get('bbox_diameter_m')
        except (OSError, ValueError):
            return None

    def _geo_set(self, key, g):
        with self.crater_lock:
            self.geo[key] = g
            self.geo_running.discard(key)
            self.stats['geo_' + g['geometry_status']] += 1
            self.crater_dirty = True

    # ---------------- 이미지 ----------------
    def _save_top(self, p):
        top = p.get('top')
        if not top:
            self.stats['no_top'] += 1               # pin_node 가 261006 이전 버전이면 top 이 없다
            return
        t = int(top['stamp_ns'])
        if t in self.saved:
            return
        img = self.images.get(t)
        if img is None:
            newest = next(reversed(self.images)) if self.images else -1
            if t > newest:
                self.pending[t] = p['id']           # 영상이 아직 안 옴 → on_image 에서 저장
            else:
                self.stats['image_missing'] += 1    # buffer_s 밖이거나 영상을 못 받음 → 이전 이미지 유지
            return
        bgr = self._to_bgr(img)
        if bgr is None:
            self.stats['bad_encoding'] += 1
            return
        if self.rim_on and t not in self.rim:
            self._rim(t, bgr, top['box'])            # 박스를 그리기 전에 — 그린 테두리가 림 엣지로 잡히지 않게
        x1, y1, x2, y2 = (int(round(v)) for v in top['box'])
        color = (60, 220, 60)                       # 확정 여부로 색을 나누지 않음 — 저장 시점(대개 후보 단계)과 달라지므로
        cv2.rectangle(bgr, (x1, y1), (x2, y2), color, 2)
        label = f"#{p['id']} {p.get('cls', '')} {float(top['score']):.2f}"
        cv2.putText(bgr, label, (x1, max(y1 - 6, 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
        cv2.putText(bgr, label, (x1, max(y1 - 6, 14)), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 1, cv2.LINE_AA)
        path = os.path.join(self.out_dir, f'det_{t}.jpg')
        if cv2.imwrite(path, bgr, [cv2.IMWRITE_JPEG_QUALITY, 90]):
            self.saved[t] = path
            self.stats['saved'] += 1
        else:
            self.stats['save_fail'] += 1

    def _rim(self, t, bgr, box):
        h, w = bgr.shape[:2]
        x1, y1, x2, y2 = (int(round(v)) for v in box)
        x1, y1, x2, y2 = max(x1, 0), max(y1, 0), min(x2, w), min(y2, h)
        on_edge = self.edge_margin > 0 and (x1 <= self.edge_margin or y1 <= self.edge_margin or
                                            x2 >= w - 1 - self.edge_margin or y2 >= h - 1 - self.edge_margin)
        if x2 - x1 < 1 or y2 - y1 < 1:
            self.rim[t] = dict(status='pending', reason='empty_crop', top_on_edge=on_edge)
            self.stats['rim_pending'] += 1
            return
        try:
            data, overlay = analyze_representative_crop(bgr[y1:y2, x1:x2], color_order='bgr', include_overlay=True,
                                                        overlay_size=self._fit(x2 - x1, y2 - y1))
        except Exception as e:  # noqa: BLE001  림 계산이 죽어도 핀·이미지는 계속
            self.rim[t] = dict(status='error', reason=f'{type(e).__name__}: {e}', top_on_edge=on_edge)
            self.stats['rim_error'] += 1
            self.get_logger().warn(f'림 계산 실패 stamp={t}: {e}')
            return
        self.rim[t] = {k: data.get(k) for k in RIM_KEYS}
        self.rim[t]['top_on_edge'] = on_edge
        self.stats['rim_' + data['status']] += 1
        try:
            with open(os.path.join(self.out_dir, f'rim_{t}.json'), 'w') as f:
                json.dump(dict(data, stamp_ns=t, box_px=[x1, y1, x2, y2], top_on_edge=on_edge), f, ensure_ascii=False)
        except OSError as e:
            self.get_logger().warn(f'rim_{t}.json 저장 실패: {e}')
        path = os.path.join(self.out_dir, f'rim_{t}.png')
        if overlay is not None and cv2.imwrite(path, overlay):
            self.rim_img[t] = path

    def _fit(self, w, h):
        """crop (w, h) 를 썸네일 상자 안에 비율 유지로 맞춘 (너비, 높이)."""
        k = min(self.rim_ov_box[0] / w, self.rim_ov_box[1] / h)
        return max(1, int(round(w * k))), max(1, int(round(h * k)))

    @staticmethod
    def _to_bgr(m):
        if m.encoding == 'mono8':
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width]
            return cv2.cvtColor(a, cv2.COLOR_GRAY2BGR)
        if m.encoding in ('bgr8', 'rgb8'):
            a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3].reshape(m.height, m.width, 3)
            return a.copy() if m.encoding == 'bgr8' else cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
        return None

    def _deg_order(self):
        """[261013] 전체 순서 = D 를 만든 크레이터만, 살아 있는 핀 id 로, 1번부터 다시 번호 (합침 직후 옛 id 가 남지 않게)."""
        with self.crater_lock:
            res = self.deg_result
        alive = {str(k) for k in self.pins}
        order = [dict(o, pin=int(o['crater_id'])) for o in (res or {}).get('order', []) if o['crater_id'] in alive]
        for i, o in enumerate(order, 1):
            o['order'], o['total'] = i, len(order)
        return res, order

    def _publish_info(self):
        pins = []
        now = self._now_ns()
        res, order = self._deg_order() if self.deg_on else (None, [])
        by_pin = {o['pin']: o for o in order}
        for pid in sorted(self.pins):
            p = dict(self.pins[pid])
            top = p.get('top') or {}
            t = int(top['stamp_ns']) if top else None
            p['image'] = self.saved.get(t)
            p['rim'] = self.rim.get(t)
            p['rim_image'] = self.rim_img.get(t)
            if self.crater_on:
                p['crater'] = self._crater_view(self.pins[pid], now)[0]
                if p['crater'].get('representative_image'):     # 확정 대표 이미지 = 형상 계산에 쓴 프레임
                    p['image'] = p['crater']['representative_image']
            if self.deg_on:
                p['degradation'] = self._deg_view(pid, p['crater']['state'], res, by_pin.get(pid))
            p.pop('cands', None)
            p.pop('blocks', None)
            p.pop('detections', None)               # 길어지기만 하고 제어창엔 필요 없음
            pins.append(p)
        info = dict(frame=self.frame, out_dir=self.out_dir, pins=pins)
        if self.deg_on:                                         # [261013] 전체 순서 (정보 창 '전체 순서' 표)
            info['degradation'] = dict(
                order=order, reference=(res or {}).get('reference'),
                provisional_reference=(res or {}).get('provisional_reference', self.deg_ref is None),
                note=(res or {}).get('note'), version=DB.VERSION, weights=DB.WEIGHTS, mission_id=self.deg_mission,
                acquisition_verified=self.deg_verified, unique_observations=(res or {}).get('unique_observations', 0),
                policy=dict(block_seconds=self.deg_policy.block_seconds, min_blocks=self.deg_policy.min_blocks,
                            min_span_seconds=self.deg_policy.min_span_seconds))
        js = json.dumps(info, ensure_ascii=False)
        self.pub_info.publish(String(data=js))
        try:
            with open(os.path.join(self.out_dir, 'pins_info.json'), 'w') as f:
                f.write(js)
        except OSError as e:
            self.get_logger().warn(f'pins_info.json 저장 실패: {e}')

    # ---------------- 마커 ----------------
    def _refresh_want(self):
        self.want = {pid: (tuple(round(v, 3) for v in p['xyz']), round(p['radius_m'], 3), bool(p['confirmed']),
                           int(p['n']), pid == self.selected, p.get('cls', ''))
                     for pid, p in self.pins.items()}

    def apply(self):
        moved = self._finalize_tick()               # [261009] 관측 종료 판단 → 계산 시작
        with self.crater_lock:
            dirty, self.crater_dirty = self.crater_dirty, False
        if dirty or moved:
            self._publish_info()                    # 상태 변화·워커 결과를 여기서 발행
        changed = False
        for pid in [k for k in self.shown if k not in self.want]:
            self.server.erase(f'pin_{pid}')         # 합쳐져 사라진 핀
            del self.shown[pid]
            changed = True
        for pid, sig in self.want.items():
            if self.shown.get(pid) != sig:
                self.server.insert(self._marker(pid, sig), feedback_callback=self.on_feedback)
                self.shown[pid] = sig
                changed = True
        if changed:
            self.server.applyChanges()

    def _marker(self, pid, sig):
        (x, y, z), r, conf, n, sel, cls = sig
        r = max(r, 0.01)
        rgba = (0.9, 0.1, 0.1, 1.0) if conf else (1.0, 0.6, 0.0, 0.6)
        im = InteractiveMarker()
        im.header.frame_id = self.frame
        im.name = f'pin_{pid}'
        im.description = f'{cls}#{pid} n={n}' + ('' if conf else ' ?')
        im.pose.position.x, im.pose.position.y, im.pose.position.z = float(x), float(y), float(z)
        im.pose.orientation.w = 1.0
        im.scale = 0.15

        def mk(kind, sx, sy, sz, dz, color):
            m = Marker()
            m.type = kind
            m.scale.x, m.scale.y, m.scale.z = float(sx), float(sy), float(sz)
            m.pose.position.z = float(dz)
            m.pose.orientation.w = 1.0
            m.color.r, m.color.g, m.color.b, m.color.a = color
            return m
        head_c = (0.1, 0.9, 1.0, 1.0) if sel else rgba
        ctl = InteractiveMarkerControl()
        ctl.name = 'click'
        ctl.interaction_mode = InteractiveMarkerControl.BUTTON
        ctl.always_visible = True
        ctl.markers = [mk(Marker.CYLINDER, 2 * r, 2 * r, 0.004, 0.0, rgba[:3] + ((0.6 if conf else 0.3),)),
                       mk(Marker.CYLINDER, 0.006, 0.006, self.stick_h, self.stick_h / 2, rgba),
                       mk(Marker.SPHERE, *((0.05,) * 3 if sel else (0.035,) * 3), self.stick_h, head_c)]
        im.controls.append(ctl)
        return im

    def on_feedback(self, fb):
        if fb.event_type != InteractiveMarkerFeedback.BUTTON_CLICK:
            return
        try:
            pid = int(fb.marker_name.split('_', 1)[1])
        except (IndexError, ValueError):
            return
        self.selected = pid
        self._refresh_want()
        self.pub_sel.publish(Int32(data=pid))
        self.get_logger().info(f'핀 #{pid} 클릭')

    def report(self):
        self.get_logger().info(f'stats {dict(self.stats)} pins={len(self.pins)} image_buf={len(self.images)} '
                               f'saved={len(self.saved)} rim={len(self.rim)} snap={len(self.snap)} geo={len(self.geo)} depth_buf={len(self.depth)}')


def main():
    rclpy.init()
    node = None
    try:
        node = PinGallery()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception:  # noqa: BLE001
        # launch·터미널 이중 Ctrl+C 로 context 가 먼저 내려간 경우 (pin_node 와 같은 처리)
        if rclpy.ok():
            raise
    finally:
        if node is not None:
            node.crater_pool.shutdown(wait=False, cancel_futures=True)   # 줄 선 분석은 버림 (도는 1개는 끝까지)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
