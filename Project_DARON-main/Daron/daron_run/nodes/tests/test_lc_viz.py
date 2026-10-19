"""lc_viz_node (261015) — 보정 전 칸 비교·PointCloud2 읽기. ROS 실행 없이: python3 -m pytest nodes/tests/test_lc_viz.py"""
import os
import sys

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'gui'))
from sensor_msgs.msg import PointField  # noqa: E402
from sensor_msgs_py import point_cloud2  # noqa: E402
from std_msgs.msg import Header  # noqa: E402
import lc_viz_node as LV  # noqa: E402

CELL = 0.025


def _loop_removed(old, cur, cell, dz_tol):
    """시연용 lc_ghost3d 의 칸별 파이썬 루프 구현 (기준)."""
    d = {(int(np.floor(x / cell)), int(np.floor(y / cell))): z for x, y, z in cur}
    keep = [abs(d.get((int(np.floor(x / cell)), int(np.floor(y / cell))), 1e9) - z) >= dz_tol for x, y, z in old]
    return old[np.array(keep, bool)]


def _grid(n=40, z0=0.0):
    ij = np.stack(np.meshgrid(np.arange(-n, n), np.arange(-n // 2, n)), -1).reshape(-1, 2)
    xy = (ij + 0.5) * CELL
    return np.column_stack([xy, np.full(len(xy), z0)]).astype(np.float32)


def test_identity_no_removed():
    g = _grid()
    assert len(LV.removed_cells(g, g, CELL, 0.04)) == 0


def test_shift_matches_loop_and_counts_edge():
    rng = np.random.default_rng(0)
    old = _grid()
    old[:, 2] = rng.normal(0, 0.005, len(old)).astype(np.float32)   # 옮겨진 칸끼리 높이차 < 4 cm (5σ 넘음)
    cur = old.copy()
    cur[:, 0] += 3 * CELL                       # 3칸 평행이동 → 왼쪽 가장자리 3줄이 옛 자리로 남음
    a = LV.removed_cells(old, cur, CELL, 0.04)
    b = _loop_removed(old, cur, CELL, 0.04)
    assert np.array_equal(np.sort(a, axis=0), np.sort(b, axis=0))
    assert len(a) == 3 * len(np.unique(old[:, 1]))


def test_height_change_counts_and_negative_coords():
    old = _grid()
    cur = old.copy()
    sel = (old[:, 0] < -0.3) & (old[:, 1] < 0)  # 음수 좌표 구역만 5 cm 올림
    cur[sel, 2] += 0.05
    a = LV.removed_cells(old, cur, CELL, 0.04)
    assert len(a) == int(sel.sum())
    assert np.array_equal(np.sort(a, axis=0), np.sort(_loop_removed(old, cur, CELL, 0.04), axis=0))


def test_empty_current_returns_all():
    old = _grid(5)
    assert len(LV.removed_cells(old, np.zeros((0, 3), np.float32), CELL, 0.04)) == len(old)


def test_cloud_xyz_reads_cells_message():
    """live_tsdf ~/map_cells 와 같은 모양(float32 필드 10개)에서 x,y,z 만 뽑는다."""
    keys = ("x", "y", "z", "zlo", "zhi", "var", "w", "np", "nf", "age")
    fields = [PointField(name=k, offset=4 * i, datatype=PointField.FLOAT32, count=1) for i, k in enumerate(keys)]
    data = np.arange(30 * len(keys), dtype=np.float32).reshape(30, len(keys))
    m = point_cloud2.create_cloud(Header(frame_id='map'), fields, data)
    xyz = LV.cloud_xyz(m)
    assert xyz.shape == (30, 3)
    assert np.array_equal(xyz, data[:, :3])


def test_node_end_to_end_without_spin():
    """노드에 메시지를 직접 넣어 이력 → 빨강 칸·핀 잔상·로버 화살표까지 예외 없이 도는지 (261015 KeyError 재발 방지)."""
    import rclpy
    from builtin_interfaces.msg import Time as TimeMsg
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from nav_msgs.msg import Odometry, Path
    from visualization_msgs.msg import Marker, MarkerArray
    rclpy.init()
    try:
        n = LV.LcViz()
        tf = TransformStamped()
        tf.header.frame_id, tf.child_frame_id = 'map', 'odom'
        tf.transform.rotation.w = 1.0
        n.tf_buf.set_transform_static(tf, 'test')

        def stamp(k):
            return TimeMsg(sec=int(k), nanosec=int((k % 1) * 1e9))
        for i in range(30):                          # x 로 1 cm 씩 가는 로버
            o = Odometry()
            o.header.frame_id, o.child_frame_id = 'odom', 'base_link'
            o.header.stamp = stamp(100 + i * 0.1)
            o.pose.pose.position.x = 0.01 * i
            o.pose.pose.orientation.w = 1.0
            n.on_odom(o)
        fields = [PointField(name=k, offset=4 * j, datatype=PointField.FLOAT32, count=1) for j, k in enumerate('xyz')]
        g = _grid(10)
        n.on_cells(point_cloud2.create_cloud(Header(frame_id='map'), fields, g))
        pin = MarkerArray()
        for k in range(4):
            mk = Marker()
            mk.header.frame_id, mk.ns, mk.id, mk.type = 'map', 'cda_pins', k, Marker.CYLINDER
            mk.pose.position.x = 0.1
            mk.pose.orientation.w = 1.0
            pin.markers.append(mk)
        n.on_pins(pin)
        h = Path()                                   # 이력: 모든 프레임 map<-odom 을 x +5 cm
        h.header.frame_id = 'map'
        for i in range(30):
            ps = PoseStamped()
            ps.header.stamp = stamp(100 + i * 0.1)
            ps.pose.position.x = 0.05
            ps.pose.orientation.w = 1.0
            h.poses.append(ps)
        n.on_hist(h)
        g2 = g.copy()
        g2[:, 0] += 0.05
        n.on_cells(point_cloud2.create_cloud(Header(frame_id='map'), fields, g2))
        for mk in pin.markers:
            mk.pose.position.x = 0.15
        n.on_pins(pin)
        n.tick()
        assert n.snap is not None and n.snap['fix'] == 1
        assert n.snap['removed_n'] == 2 * len(np.unique(g[:, 1]))     # 5 cm = 2칸 → 왼쪽 2줄이 옛 자리
        assert n.snap['pins_moved'] == 1
        assert n.corr is not None and len(n.corr.poses) == 30
        n.destroy_node()
    finally:
        rclpy.try_shutdown()
