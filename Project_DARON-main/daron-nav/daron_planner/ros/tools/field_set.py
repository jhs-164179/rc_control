#!/usr/bin/env python3
"""경기장 놓기·틈 지정 (261014, 브랜치 auto-1014). 로버를 출발 자리에 둔 뒤 한 번 실행 → 경로계획이 '지금 로버 자세'를 기준으로 경기장을 놓는다.

왜: 방문점·경계 금지 띠·틈은 경기장 좌표로 정해져 있음. 예전에는 'SLAM 을 켠 자리 = 출발 자리'라고 가정했는데,
   다른 곳에서 켰거나 옛 지도로 재추정하면 경기장이 통째로 어긋남 (케일럽 10/14: '아주 큰일') → 운영자가 직접 놓음.

경기장 좌표: 가운데 원점, x = 폭(--size 첫 값) 방향, y = 길이(둘째 값) 방향, +y = 페이즈2 쪽 변.
출발 자리: 로버 base_link(뒷축 가운데)가 --corner 구석에서 x 로 --offset 첫 값, y 로 둘째 값 들어간 곳, 로버는 --facing 쪽을 봄.
   기본 = 4 x 5 m, 왼쪽 아래(bl) 구석에서 0.6 m · 0.43 m, +y(긴 변 방향 = 페이즈2 쪽)를 봄 [가정, 대회 출발 구역은 연습 주행 때 확인]
   예) 로버 앞이 짧은 변(4 m) 방향이면 --facing +x  (= 경기장을 '가로'로 놓음)

실행 (우분투 지상국, 경로계획과 같은 ROS 환경·도메인. 맥에서 SSH 접속한 터미널도 됨). AUTO 중에는 거부됨 → MANUAL/WAYPOINT 에서:
    source ~/Daron/slam_env.sh
    /usr/bin/python3 field_set.py                         # 기본값으로 놓음
    /usr/bin/python3 field_set.py --corner bl --offset 0.6 0.43 --facing +y --size 4 5 --gap -1.5 2.3
    /usr/bin/python3 field_set.py --gap 1.5 2.3 --gap-only # 틈만 옮김 (AUTO 중에도 됨)
    /usr/bin/python3 field_set.py --dry                   # ROS 없이 보낼 JSON·그림만
    /usr/bin/python3 field_set.py --clear                 # [10/15] 경기장 풀기 → 연구실 범위 (live_lab.yaml lab_until_set)
확인: RViz /planner/markers 노란 테두리가 바닥 테이프와 겹치는지, /planner/status field_pose 가 출발 자리 값인지, field.how = here.
"""
import argparse
import json
import math
import sys
import time

FACING = {'+y': 90.0, '-y': -90.0, '+x': 0.0, '-x': 180.0}


def start_pose(size, corner, offset, facing):
    W, L = size
    dx, dy = offset
    x = -W / 2 + dx if corner[1] == 'l' else W / 2 - dx
    y = -L / 2 + dy if corner[0] == 'b' else L / 2 - dy
    return [round(x, 3), round(y, 3), FACING[facing]]


def sketch(size, start, gap):
    """위에서 본 경기장 (가로 = x, 세로 = y, 위 = +y 페이즈2 쪽). R = 로버, 화살표 = 보는 방향, G = 틈."""
    W, L = size
    nx, ny = 21, int(round(21 * L / W * 0.5))
    g = [[' '] * nx for _ in range(ny)]
    for i in range(nx):
        g[0][i] = g[ny - 1][i] = '-'
    for j in range(ny):
        g[j][0] = g[j][nx - 1] = '|'

    def cell(x, y):
        i = int(round((x + W / 2) / W * (nx - 1)))
        j = int(round((L / 2 - y) / L * (ny - 1)))
        return min(max(j, 0), ny - 1), min(max(i, 0), nx - 1)
    if gap is not None:
        j, i = cell(*gap); g[j][i] = 'G'
    j, i = cell(start[0], start[1])
    g[j][i] = {90.0: '^', -90.0: 'v', 0.0: '>', 180.0: '<'}.get(start[2], 'R')
    return '\n'.join(''.join(r) for r in g)


def main():
    ap = argparse.ArgumentParser(description='경기장 놓기·틈 지정 (/ui/field)')
    ap.add_argument('--size', nargs=2, type=float, default=[4.0, 5.0], metavar=('W', 'L'), help='경기장 폭(x) 길이(y) [m], 기본 4 5')
    ap.add_argument('--corner', choices=['bl', 'br', 'tl', 'tr'], default='bl', help='출발 구석: b/t = 아래(-y)/위(+y), l/r = 왼(-x)/오른(+x). 기본 bl')
    ap.add_argument('--offset', nargs=2, type=float, default=[0.6, 0.43], metavar=('DX', 'DY'), help='구석에서 base_link 까지 x, y 거리 [m]')
    ap.add_argument('--facing', choices=list(FACING), default='+y', help='로버가 보는 방향 (경기장 축). 기본 +y')
    ap.add_argument('--gap', nargs=2, type=float, default=None, metavar=('X', 'Y'), help='틈(페이즈2 입구) 경기장 좌표 [m]')
    ap.add_argument('--gap-only', action='store_true', help='경기장은 그대로, 틈만')
    ap.add_argument('--grid', nargs=2, type=int, default=None, metavar=('COLS', 'ROWS'),
                    help='[10/15] 방문점 격자만 바꿈: 열(x, 4 m 변) 행(y, 5 m 변). AUTO 중 거부, 방문 기록 지움')
    ap.add_argument('--clear', action='store_true', help='[10/15] 경기장 풀기 → 연구실 범위 (경로계획 lab_until_set 일 때, AUTO 중 거부)')
    ap.add_argument('--topic', default='/ui/field')
    ap.add_argument('--dry', action='store_true', help='보내지 않고 JSON·그림만')
    a = ap.parse_args()
    if a.clear:
        msg = dict(cmd='clear')
    elif a.grid is not None:
        msg = dict(cmd='grid', cols=a.grid[0], rows=a.grid[1])
    elif a.gap_only:
        if a.gap is None:
            sys.exit('--gap-only 에는 --gap X Y 가 필요')
        msg = dict(cmd='gap', xy=a.gap)
    else:
        st = start_pose(a.size, a.corner, a.offset, a.facing)
        msg = dict(cmd='here', size=a.size, start=st)
        if a.gap is not None:
            msg['gap'] = a.gap
        print('경기장 %.1f x %.1f m, 로버 base_link = 경기장 (%.2f, %.2f), 보는 방향 %s (%.0f°)' % (a.size[0], a.size[1], st[0], st[1], a.facing, st[2]))
        print(sketch(a.size, st, a.gap))
    data = json.dumps(msg)
    print('보냄 %s: %s' % (a.topic, data))
    if a.dry:
        return
    import rclpy
    from std_msgs.msg import String
    rclpy.init()
    n = rclpy.create_node('field_set')
    pub = n.create_publisher(String, a.topic, 10)
    got = []
    n.create_subscription(String, '/planner/field', lambda m: got.append(m.data), 10)
    t0 = time.time()
    while time.time() - t0 < 1.0:                                     # 구독자 연결 기다림
        rclpy.spin_once(n, timeout_sec=0.1)
    pub.publish(String(data=data))
    t0 = time.time()
    n_before = len(got)
    while time.time() - t0 < 3.0 and len(got) <= n_before:
        rclpy.spin_once(n, timeout_sec=0.1)
    if len(got) > n_before:
        f = json.loads(got[-1])
        lc = f.get('last_cmd') or {}
        print('경로계획 응답: how=%s size=%s start=%s gap=%s grid=%s %s' % (f.get('how'), f.get('size'), f.get('start'), f.get('gap'), f.get('grid'),
                                                                  ('' if not lc else '(%s: %s)' % ('OK' if lc.get('ok') else '거부', lc.get('why')))))
    else:
        print('경로계획 응답 없음 (3 s) — 노드가 떠 있는지, 같은 ROS_DOMAIN_ID 인지 확인')
    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
