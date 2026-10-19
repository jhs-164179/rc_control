#!/usr/bin/python3
"""lc_viz_node — 루프 클로저 보정을 3D RViz 에 보이게 하는 표시 전용 노드 (261015).

**구독만 하고 표시 토픽(/lc_viz/*)만 낸다.** 피쳐맵·핀·웨이포인트·로버 TF·판정·플래너는 이 노드와 무관하게 이미 보정된다
(CHANGES_261011_lcsync.md). 끄려면 daron_gui.launch.py lc_viz:=false.

cuVSLAM 노드가 루프 클로저 뒤 /slam/map_odom_history 를 내면, 그 직전 상태를 '보정 전'으로 기억해 show_s 동안 보여 준다.
burst_s 안에 이어 오는 이력(보통 2 s 간격 2~3개)은 같은 보정으로 묶는다.

  /lc_viz/map_removed     보정 전엔 칸이 있었는데 지금 그 자리(xy 같은 칸, 높이 dz_tol 안)에 칸이 없는 곳 = 옛 위치 (빨강 칸)
  /lc_viz/map_now         보정 후 피쳐맵 칸 (하늘색) — 원본 RViz '3D Map (full)' 은 30 s 마다라 보정 직후엔 옛 지도가 보이므로
                          show_s 동안 지금 칸을 같이 낸다 (빨강 = 전, 하늘색 = 후)
  /lc_viz/pins_before     1 cm 넘게 옮겨진 크레이터 핀의 보정 전 자리 (보라) + 새 자리까지 화살표
  /lc_viz/rover_fix       같은 stamp 의 로버: 보정 전 map 위치(빨강) / 보정 후 TF 로 다시 본 위치(초록) + 선
  /lc_viz/path_realtime   그때그때 TF map<-odom x /slam/odometry (1 cm 이상 움직일 때만 쌓음)
  /lc_viz/path_corrected  이력 M_corr(t) x /slam/odometry(t) + 마지막 이력 뒤 실시간 = 지금 알고 있는 최선의 전체 궤적
빨강 칸에는 새 관측에 따른 칸 변화도 조금 섞인다 → 보정 뒤 show_s 만 표시.
"""
import bisect
import copy

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros

LATCH = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST, reliability=ReliabilityPolicy.RELIABLE,
                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
_NP_OF = {PointField.FLOAT32: np.float32, PointField.FLOAT64: np.float64}


def T_of(p, q):
    x, y, z, w = q
    R = np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                  [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                  [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = p
    return T


def T_pose(m):
    return T_of([m.position.x, m.position.y, m.position.z],
                [m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w])


def ns(st):
    return st.sec * 1_000_000_000 + st.nanosec


def pt(v):
    return Point(x=float(v[0]), y=float(v[1]), z=float(v[2]))


def cloud_xyz(m):
    """PointCloud2 → (N,3) float32. 필드 오프셋으로 바로 읽음 (점마다 파이썬 루프 없음)."""
    f = {x.name: x for x in m.fields}
    n = m.width * m.height
    if n == 0 or not all(k in f for k in 'xyz'):
        return np.zeros((0, 3), np.float32)
    buf = np.frombuffer(m.data, np.uint8).reshape(n, m.point_step)
    out = np.empty((n, 3), np.float32)
    for i, k in enumerate('xyz'):
        dt = np.dtype(_NP_OF.get(f[k].datatype, np.float32)).newbyteorder('>' if m.is_bigendian else '<')
        out[:, i] = buf[:, f[k].offset:f[k].offset + dt.itemsize].copy().view(dt).ravel()
    return out[np.isfinite(out).all(axis=1)]


def _safe(fn):
    """표시 전용 노드라 콜백 예외로 죽지 않게 — 경고만 남기고 다음 메시지를 기다린다."""
    def wrap(self, *a):
        try:
            return fn(self, *a)
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f'[lc_viz] {fn.__name__} 예외 (표시만 건너뜀): {type(e).__name__}: {e}',
                                      throttle_duration_sec=5.0)
    wrap.__name__ = fn.__name__
    return wrap


def removed_cells(old, cur, cell, dz_tol):
    """old 칸 중 cur 의 같은 xy 칸에 없거나 높이가 dz_tol 이상 다른 것. (N,3) float32 둘 다."""
    if not len(old):
        return old
    if not len(cur):
        return old

    def key(a):
        ij = np.floor(a[:, :2] / cell).astype(np.int64)
        return (ij[:, 0] << 32) ^ (ij[:, 1] & 0xFFFFFFFF)
    kc, ko = key(cur), key(old)
    order = np.argsort(kc, kind='stable')
    kc_s, zc_s = kc[order], cur[order, 2]
    i = np.searchsorted(kc_s, ko)
    i_c = np.minimum(i, len(kc_s) - 1)
    hit = (i < len(kc_s)) & (kc_s[i_c] == ko)
    same = hit & (np.abs(zc_s[i_c] - old[:, 2]) < dz_tol)
    return old[~same]


class LcViz(Node):
    def __init__(self):
        super().__init__('lc_viz')
        P = self.declare_parameter
        self.map_frame = str(P('map_frame', 'map').value)
        self.cell = float(P('cell', 0.025).value)                 # live_tsdf grid_cell 과 같게
        self.dz_tol = float(P('dz_tol', 0.04).value)
        self.burst_s = float(P('burst_s', 8.0).value)
        self.show_s = float(P('show_s', 10.0).value)
        self.path_step = float(P('path_step_m', 0.01).value)
        cells_topic = str(P('cells_topic', '/live_tsdf/map_cells').value)
        odom_topic = str(P('odom_topic', '/slam/odometry').value)  # 이력 M_corr 와 짝인 VO (odom<-base)
        hist_topic = str(P('history_topic', '/slam/map_odom_history').value)
        pins_topic = str(P('pins_topic', '/pin_node/pins').value)
        self.tf_buf = tf2_ros.Buffer()
        self.tf_l = tf2_ros.TransformListener(self.tf_buf, self)
        self.cells = np.zeros((0, 3), np.float32)
        self.cells_hdr = None
        self.pins = {}                                  # (ns,id) -> Marker (지금)
        self.odom_t, self.odom_T = [], []               # stamp ns 오름차순, odom<-base
        self.rt = Path()
        self.rt.header.frame_id = self.map_frame
        self.rt_pos = {}                                # stamp ns -> 그때 실시간 map 위치
        self.last_rt = None                             # (stamp ns, VO 4x4, map 4x4)
        self.corr, self.corr_last_ns = None, 0
        self.snap = None
        self.n_hist = self.n_fix = 0
        self.create_subscription(PointCloud2, cells_topic, self.on_cells, 1)
        self.create_subscription(Odometry, odom_topic, self.on_odom, 50)
        self.create_subscription(Path, hist_topic, self.on_hist, LATCH)
        self.create_subscription(MarkerArray, pins_topic, self.on_pins, LATCH)
        self.pub_removed = self.create_publisher(PointCloud2, '/lc_viz/map_removed', LATCH)
        self.pub_now = self.create_publisher(PointCloud2, '/lc_viz/map_now', LATCH)
        self.pub_pins = self.create_publisher(MarkerArray, '/lc_viz/pins_before', LATCH)
        self.pub_fix = self.create_publisher(MarkerArray, '/lc_viz/rover_fix', LATCH)
        self.pub_rt = self.create_publisher(Path, '/lc_viz/path_realtime', 1)
        self.pub_corr = self.create_publisher(Path, '/lc_viz/path_corrected', LATCH)
        self.create_timer(0.5, self.tick)
        self.get_logger().info(f'lc_viz: {hist_topic} 오면 보정 전 칸·핀·로버를 {self.show_s:g} s 표시 → /lc_viz/* '
                               f'(칸 {self.cell * 100:g} cm, 묶음 {self.burst_s:g} s). 표시 전용 — 다른 노드에 영향 없음')

    def now_s(self):
        """ROS 시계 [s] — 실시간은 벽시계와 같고, bag 재생(use_sim_time)이면 bag 시각이라 배속과 무관하게 show_s 가 일정."""
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- 입력 ----------
    @_safe
    def on_cells(self, m):
        self.cells = cloud_xyz(m)
        self.cells_hdr = m.header
        self.publish_removed()

    @_safe
    def on_pins(self, arr):
        cur = {}
        for mk in arr.markers:
            if mk.action == Marker.DELETEALL:
                cur = {}
                continue
            cur[(mk.ns, mk.id)] = mk
        self.pins = cur
        self.publish_pins()

    @_safe
    def on_odom(self, m):
        k = ns(m.header.stamp)
        if self.odom_t and k <= self.odom_t[-1]:
            return
        T = T_pose(m.pose.pose)
        self.odom_t.append(k)
        self.odom_T.append(T)
        try:
            tf = self.tf_buf.lookup_transform(self.map_frame, m.header.frame_id, Time())
        except Exception:
            return
        tr, q = tf.transform.translation, tf.transform.rotation
        M = T_of([tr.x, tr.y, tr.z], [q.x, q.y, q.z, q.w])
        Pm = M @ T
        self.last_rt = (k, T, Pm)
        if (not self.rt.poses or np.linalg.norm(Pm[:3, 3] - self.rt_pos[ns(self.rt.poses[-1].header.stamp)])
                >= self.path_step):
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.header.stamp = m.header.stamp
            ps.pose.position = pt(Pm[:3, 3])
            ps.pose.orientation.w = 1.0
            self.rt.poses.append(ps)
            self.rt.header.stamp = m.header.stamp
            self.rt_pos[k] = Pm[:3, 3].copy()
        s = self.snap
        if s is not None and s.get('rover') is not None:
            if self.now_s() - s['t0'] < self.show_s:
                _, T0, P0 = s['rover']
                P1 = M @ T0                       # 같은 stamp 의 VO 를 지금 map<-odom 으로 다시 본 위치
                s['rover_d'] = max(s.get('rover_d', 0.0), float(np.linalg.norm(P1[:3, 3] - P0[:3, 3])))
                arr = MarkerArray(markers=[Marker(action=Marker.DELETEALL),
                                           self.arrow(1, P0, (1.0, 0.15, 0.15)), self.arrow(2, P1, (0.2, 1.0, 0.3))])
                ln = Marker()
                ln.header.frame_id = self.map_frame
                ln.ns, ln.id, ln.type, ln.action = 'fix', 3, Marker.LINE_STRIP, Marker.ADD
                ln.pose.orientation.w = 1.0
                ln.scale.x = 0.012
                ln.color.r = ln.color.g = ln.color.b = ln.color.a = 1.0
                ln.points = [pt(P0[:3, 3] + [0, 0, 0.15]), pt(P1[:3, 3] + [0, 0, 0.15])]
                arr.markers.append(ln)
                self.pub_fix.publish(arr)
            elif not s.get('rover_cleared'):
                s['rover_cleared'] = True
                self.pub_fix.publish(MarkerArray(markers=[Marker(action=Marker.DELETEALL)]))

    def arrow(self, id_, Pm, rgb):
        a = Marker()
        a.header.frame_id = self.map_frame
        a.ns, a.id, a.type, a.action = 'fix', id_, Marker.ARROW, Marker.ADD
        a.pose.orientation.w = 1.0
        p0 = Pm[:3, 3] + [0, 0, 0.15]
        h = np.array([Pm[0, 0], Pm[1, 0], 0.0])       # 진행 방향을 수평면에 투영 (기울어 있어도 읽히게)
        h = h / (np.linalg.norm(h) or 1.0)
        a.points = [pt(p0), pt(p0 + 0.3 * h)]
        a.scale.x, a.scale.y, a.scale.z = 0.04, 0.09, 0.1
        a.color.r, a.color.g, a.color.b, a.color.a = rgb[0], rgb[1], rgb[2], 1.0
        return a

    def odom_at(self, k, tol=20_000_000):
        i = bisect.bisect_left(self.odom_t, k)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(self.odom_t) and abs(self.odom_t[j] - k) <= tol:
                if best is None or abs(self.odom_t[j] - k) < abs(self.odom_t[best] - k):
                    best = j
        return None if best is None else self.odom_T[best]

    # ---------- 이력 ----------
    @_safe
    def on_hist(self, h):
        self.n_hist += 1
        now = self.now_s()
        new_fix = self.snap is None or now - self.snap['t0'] > self.burst_s
        if new_fix:
            self.log_fix_summary()              # 앞 묶음이 show_s 전에 바뀌어도 요약은 남김
            self.n_fix += 1
            self.snap = dict(t0=now, cells=self.cells.copy(), pins=copy.deepcopy(self.pins), rover=self.last_rt,
                             fix=self.n_fix)
        out = Path()
        out.header.frame_id = self.map_frame
        out.header.stamp = h.header.stamp
        miss, dmax = 0, 0.0
        for p in h.poses:
            k = ns(p.header.stamp)
            T = self.odom_at(k)
            if T is None:
                miss += 1
                continue
            Pm = T_pose(p.pose) @ T
            ps = PoseStamped()
            ps.header.frame_id = self.map_frame
            ps.header.stamp = p.header.stamp
            ps.pose.position = pt(Pm[:3, 3])
            ps.pose.orientation.w = 1.0
            out.poses.append(ps)
            r = self.rt_pos.get(k)
            if r is not None:
                dmax = max(dmax, float(np.linalg.norm(Pm[:3, 3] - r)))
        self.corr = out
        self.corr_last_ns = ns(out.poses[-1].header.stamp) if out.poses else 0
        self.publish_paths()
        self.publish_removed()
        self.publish_pins()
        self.get_logger().info(f'[lc_viz] 이력 #{self.n_hist} (보정 묶음 #{self.n_fix}{" 새" if new_fix else ""}) '
                               f'프레임 {len(h.poses)} (odom 못 찾음 {miss}) · 궤적 보정 최대 {dmax * 100:.1f} cm')

    # ---------- 출력 ----------
    @_safe
    def tick(self):
        self.publish_paths()
        if self.snap is not None and self.now_s() - self.snap['t0'] > self.show_s:
            self.log_fix_summary()

    def log_fix_summary(self):
        s = self.snap
        if s is None or s.get('logged'):
            return
        s['logged'] = True
        self.get_logger().info(f'[lc_viz] 보정 묶음 #{s["fix"]}: 빨강 칸 최대 {s.get("removed_n", 0)}개 · '
                               f'핀 {s.get("pins_moved", 0)}개 옮겨짐 · 로버 {s.get("rover_d", 0.0) * 100:.1f} cm')

    def publish_paths(self):
        if self.pub_rt.get_subscription_count():
            self.pub_rt.publish(self.rt)
        if self.corr is not None and self.pub_corr.get_subscription_count():
            out = Path()
            out.header = self.corr.header
            out.poses = list(self.corr.poses) + [p for p in self.rt.poses if ns(p.header.stamp) > self.corr_last_ns]
            self.pub_corr.publish(out)

    def publish_removed(self):
        s = self.snap
        if s is None or self.cells_hdr is None or not len(s['cells']):
            return
        if self.now_s() - s['t0'] > self.show_s:
            if not s.get('removed_cleared'):
                s['removed_cleared'] = True
                empty = point_cloud2.create_cloud_xyz32(self.cells_hdr, np.zeros((0, 3), np.float32))
                self.pub_removed.publish(empty)
                self.pub_now.publish(empty)
            return
        rem = removed_cells(s['cells'], self.cells, self.cell, self.dz_tol)
        s['removed_n'] = max(s.get('removed_n', 0), int(len(rem)))
        self.pub_removed.publish(point_cloud2.create_cloud_xyz32(self.cells_hdr, rem))
        self.pub_now.publish(point_cloud2.create_cloud_xyz32(self.cells_hdr, self.cells))

    def publish_pins(self):
        s = self.snap
        if s is None:
            return
        arr = MarkerArray(markers=[Marker(action=Marker.DELETEALL)])
        if self.now_s() - s['t0'] > self.show_s:
            if not s.get('pins_cleared'):
                s['pins_cleared'] = True
                self.pub_pins.publish(arr)
            return
        old, cur = s['pins'], self.pins
        moved = 0
        for (nsp, mid), mk in old.items():
            if mid % 4 != 0:                    # pin_node: id*4+k, k=0 원판(핀 중심)
                continue
            now_mk = cur.get((nsp, mid))
            if now_mk is None:
                continue
            a = np.array([mk.pose.position.x, mk.pose.position.y, mk.pose.position.z])
            b = np.array([now_mk.pose.position.x, now_mk.pose.position.y, now_mk.pose.position.z])
            if np.linalg.norm(a - b) < 0.01:
                continue
            moved += 1
            for k in (0, 2):                    # 원판 + 머리 구 (핀 본체가 빨강이라 잔상은 연보라)
                src = old.get((nsp, mid + k))
                if src is None:
                    continue
                g = Marker()
                g.header.frame_id = src.header.frame_id or self.map_frame
                g.ns, g.id, g.type, g.action = 'pin_before', mid + k, src.type, Marker.ADD
                g.pose, g.scale = src.pose, src.scale
                if k == 0:
                    g.scale.z = 0.01
                g.color.r, g.color.g, g.color.b, g.color.a = 0.7, 0.55, 1.0, 0.8
                arr.markers.append(g)
            ln = Marker()
            ln.header.frame_id = mk.header.frame_id or self.map_frame
            ln.ns, ln.id, ln.type, ln.action = 'pin_move', mid, Marker.ARROW, Marker.ADD
            ln.pose.orientation.w = 1.0
            ln.points = [pt(a + [0, 0, 0.05]), pt(b + [0, 0, 0.05])]
            ln.scale.x, ln.scale.y, ln.scale.z = 0.008, 0.02, 0.02
            ln.color.r = ln.color.g = ln.color.b = ln.color.a = 1.0
            arr.markers.append(ln)
        s['pins_moved'] = max(s.get('pins_moved', 0), moved)
        self.pub_pins.publish(arr)


def main():
    rclpy.init()
    n = LcViz()
    try:
        rclpy.spin(n)
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
