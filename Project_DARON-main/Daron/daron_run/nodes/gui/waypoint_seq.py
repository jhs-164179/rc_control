"""지상국 웨이포인트 목록 ↔ 경로계획 노드 동기화 (ROS 의존 없음). 작성 261006.

목록은 지상국(제어창)이 들고, 플래너에는 **지금 갈 목표 하나만** 준다.
  - 플래너 waypoint_queue 기본값이 false(새 목표가 목록을 덮어씀)이고 실행 중에 못 바꾼다.
    하나씩 주면 queue 가 true 든 false 든 똑같이 동작한다.
  - 플래너는 중간 삭제·순서 변경 명령이 없다(CLEAR / UNDO / NEXT 뿐). 지상국이 목록을 편집하고
    맨 앞이 바뀌면 CLEAR → (플래너 목록이 빈 것 확인) → 새 목표 를 보낸다. 서로 다른 토픽이라
    순서가 보장되지 않으므로 CLEAR 가 반영된 걸 /planner/waypoints 로 확인한 뒤에 목표를 보낸다.
  - 도착: 플래너가 스스로 목록에서 빼고(/planner/waypoints 가 빔) 상태를 ARRIVED 로 바꾼다.
    상태는 0.5 s 주기 발행이라 목록이 먼저 빌 수 있다 → arrive_wait_s 동안 ARRIVED 를 기다린다.
  - 밖에서 목표가 바뀐 경우(다른 RViz 의 2D Goal Pose 등)는 싸우지 않고 일시정지 + 경고.

tick() 이 돌려주는 동작: ('clear',) 또는 ('goal', x, y, z, yaw). 좌표는 전부 waypoint_frame.

[261011] 도착 방위각 yaw [rad, waypoint_frame 기준, None = 방향 지정 없음]
  - 항목 dict 에 'yaw'. 2D Goal Pose 드래그 방향(지도 클릭) 또는 영상 클릭 드래그 방향(gui/image_click.py).
  - 플래너는 아직 위치만 쓴다(방위 도착 판정은 나중 — 플래너 쪽 작업). 지상국은 목표 orientation 에 실어 보내고,
    도착 순간의 위치·방위 오차를 기록만 한다 (arrival_error — 판정 아님).
  - jsonl goal 에 'yaw_deg' (없으면 키 없음 → 옛 파일과 같은 모양). 불러올 때 없으면 yaw None.

[261011 밤] 루프 클로저 재동기화 — relocate(): 지도가 보정 궤적으로 다시 쌓이면 지상국이 이번 세션 웨이포인트를 같이 옮긴다.
  지금 플래너에 준 목표가 옮겨지면 CLEAR → 새 좌표로 다시 보낸다(아래 tick 첫머리). 기록은 'moved' 사건, reconstruct 가 반영.

[261008] 도착 기록·불러오기 (아래 '기록' 절, 역시 ROS 의존 없음 — tests/test_waypoint_log.py)
  - 도착 판정은 위 tick() 의 ARRIVED 분기 하나뿐이다. 지상국은 on_arrived 콜백으로 그 순간을 받아 기록한다.
  - <session_dir>/waypoints.jsonl 에 사건 한 줄씩 (JsonlLog). 형식은 ground_station.py 독스트링.
  - reconstruct(): jsonl → 웨이포인트 목록 (id 별 도착/미도착/삭제). check_loaded(): 좌표계·세션 검사.
  - 좌표는 json 의 float repr 그대로 저장·복원 (반올림 없음 — 같은 자리 재방문용).
"""
from __future__ import annotations

import itertools
import json
import math
import os

IDLE, CLEARING, SENDING, ACTIVE = 'IDLE', 'CLEARING', 'SENDING', 'ACTIVE'


class WaypointSeq:
    def __init__(self, tol_m=0.02, clear_timeout_s=1.0, accept_timeout_s=1.5, arrive_wait_s=1.5, max_clear_retry=3):
        self.tol = tol_m
        self.clear_timeout = clear_timeout_s
        self.accept_timeout = accept_timeout_s
        self.arrive_wait = arrive_wait_s
        self.max_clear_retry = max_clear_retry
        self.items: list[dict] = []         # {id, x, y, z, yaw}  yaw [rad] 또는 None [261011]
        self.done: list[dict] = []          # 도착한 것 (표시용)
        self.paused = False
        self.phase = IDLE
        self.sent = None                    # 플래너에 준 항목 (dict 사본)
        self.owns = False                   # 플래너 목표를 지상국이 넣었는가 (밖에서 넣은 목표는 안 지움)
        self.msg = ''                       # 사용자에게 보일 마지막 사건
        self._t = 0.0                       # phase 진입 시각
        self._clear_n = 0
        self._empty_since = None
        self._ids = itertools.count(1)
        self.on_arrived = None              # [261008] 도착 콜백 f(item) — 도착 판정은 tick() 의 ARRIVED 분기 하나뿐
        self._resend = False                # [261011] 보낸 목표가 옮겨짐 → 다음 tick 에 CLEAR 후 다시 보냄

    def continue_ids(self, last_id: int):
        """[261008] 같은 세션 폴더의 기록을 이어 쓸 때 id 가 겹치지 않게 last_id 다음부터 매긴다."""
        self._ids = itertools.count(int(last_id) + 1)

    # ---------------- 편집 (제어창 버튼) ----------------
    def add(self, x, y, z=0.0, yaw=None) -> dict:
        it = dict(id=next(self._ids), x=float(x), y=float(y), z=float(z),
                  yaw=None if yaw is None else wrap_pi(float(yaw)))
        self.items.append(it)
        return it

    def remove(self, idx: int):
        if 0 <= idx < len(self.items):
            del self.items[idx]

    def move(self, idx: int, d: int):
        j = idx + d
        if 0 <= idx < len(self.items) and 0 <= j < len(self.items):
            self.items[idx], self.items[j] = self.items[j], self.items[idx]

    def clear_all(self):
        self.items.clear()

    def relocate(self, it_id, x, y, z, yaw):
        """[261011] 루프 클로저 재동기화로 항목 좌표를 바꾼다. 플래너에 준 목표면 다시 보낸다. 반환: 찾았는가."""
        for it in self.items:
            if it['id'] == it_id:
                it.update(x=float(x), y=float(y), z=float(z), yaw=None if yaw is None else wrap_pi(float(yaw)))
                if self.sent is not None and self.sent['id'] == it_id and self.phase in (SENDING, ACTIVE):
                    self._resend = True
                return True
        return False

    def set_paused(self, p: bool):
        self.paused = bool(p)
        if not p:
            self.msg = ''

    # ---------------- 상태 ----------------
    @property
    def desired(self):
        return self.items[0] if self.items and not self.paused else None

    def item_state(self, it) -> str:
        if self.sent is not None and it['id'] == self.sent['id']:
            return {SENDING: '보내는 중', ACTIVE: '진행 중'}.get(self.phase, '')
        return '대기'

    def _match(self, wps, it) -> bool:
        return (wps is not None and len(wps) == 1
                and math.hypot(wps[0][0] - it['x'], wps[0][1] - it['y']) <= self.tol)

    def _go(self, phase, now):
        self.phase, self._t = phase, now

    # ---------------- 주기 ----------------
    def tick(self, now: float, planner_wps, planner_state: str = '', planner_reason: str = '') -> list:
        """planner_wps: /planner/waypoints 의 [(x, y), ...] (아직 못 받았으면 None)."""
        out = []
        want = self.desired

        if self._resend:                                    # [261011] 보낸 목표가 옮겨짐 → 비우고 새 좌표로
            self._resend = False
            if self.phase in (SENDING, ACTIVE):
                self.msg = f'웨이포인트 #{self.sent["id"]} 위치 보정(루프 클로저) → 플래너에 다시 보냄'
                return self._start_clear(now, out)

        if self.phase == CLEARING:
            if planner_wps is not None and len(planner_wps) == 0:
                self.owns = False
                self._go(IDLE, now)
            elif now - self._t > self.clear_timeout:
                if self._clear_n >= self.max_clear_retry:
                    self.paused, self.msg = True, '플래너가 CLEAR 에 응답 없음 → 일시정지'
                    self._go(IDLE, now)
                    return out
                self._clear_n += 1
                self._t = now
                out.append(('clear',))
                return out
            else:
                return out

        if self.phase == SENDING:
            if want is not None and want['id'] == self.sent['id'] and self._match(planner_wps, self.sent):
                self._go(ACTIVE, now)
                self._empty_since = None
                return out
            if want is None or want['id'] != self.sent['id']:
                return self._start_clear(now, out)          # 보내는 사이 편집됨
            if now - self._t > self.accept_timeout:
                self.paused = True
                self.msg = f'플래너가 웨이포인트 #{self.sent["id"]} 를 안 받음' + (f' ({planner_reason})' if planner_reason else '') + ' → 일시정지'
                self.sent = None
                self._go(IDLE, now)
            return out

        if self.phase == ACTIVE:
            if want is None or want['id'] != self.sent['id']:
                return self._start_clear(now, out)
            if planner_wps is not None and len(planner_wps) == 0:
                if planner_state == 'ARRIVED':
                    arrived = self.items.pop(0)
                    self.done.append(arrived)
                    self.msg = f'웨이포인트 #{self.sent["id"]} 도착'
                    if self.on_arrived is not None:
                        self.on_arrived(arrived)
                    self.sent, self.owns, self._empty_since = None, False, None
                    self._go(IDLE, now)
                    want = self.desired                     # 같은 틱에 다음 것을 보낸다
                else:
                    if self._empty_since is None:
                        self._empty_since = now
                    if now - self._empty_since <= self.arrive_wait:
                        return out
                    self.paused, self.msg = True, '플래너 목표가 밖에서 지워짐 → 일시정지 (재개하면 다시 보냄)'
                    self.sent, self.owns, self._empty_since = None, False, None
                    self._go(IDLE, now)
                    return out
            elif planner_wps is not None and not self._match(planner_wps, self.sent):
                self.paused, self.msg = True, '플래너 목표가 밖에서 바뀜(다른 RViz 의 2D Goal Pose?) → 일시정지 (재개하면 제어창 목록으로 되돌림)'
                self.sent, self.owns = None, False
                self._go(IDLE, now)
                return out
            else:
                self._empty_since = None
                return out

        # IDLE
        if planner_wps is None:
            return out                                      # 플래너를 아직 못 봄
        if want is None:
            if self.owns and len(planner_wps) > 0:
                return self._start_clear(now, out)          # 일시정지·전체 삭제 → 플래너도 멈춤
            return out
        if len(planner_wps) > 0:
            self.owns = True                                # 지상국이 목록을 쥐므로 남의 목표도 비운다
            return self._start_clear(now, out)
        self.sent = dict(want)
        self.owns = True
        self._go(SENDING, now)
        out.append(('goal', want['x'], want['y'], want['z'], want.get('yaw')))
        return out

    def _start_clear(self, now, out):
        self.sent = None
        self._clear_n = 0
        self._go(CLEARING, now)
        out.append(('clear',))
        return out


# ============================================================ [261008] 기록·불러오기
WP_LOG_NAME = 'waypoints.jsonl'
REMOVED_EVENTS = ('removed', 'cleared')


def wrap_pi(a: float) -> float:
    """[261011] 각도 → (-π, π]"""
    a = math.fmod(a + math.pi, 2 * math.pi)
    if a <= 0:
        a += 2 * math.pi
    return a - math.pi


def wp_goal(it) -> dict:
    g = {'x': it['x'], 'y': it['y'], 'z': it['z']}
    if it.get('yaw') is not None:                           # [261011] 방향 지정한 것만 (옛 기록과 같은 모양 유지)
        g['yaw_deg'] = math.degrees(it['yaw'])
    return g


def arrival_error(it, pose):
    """[261011] 도착 순간 오차 (기록용 — 판정 아님). pose = (x, y, z, yaw[rad]).
    → {'pos_err_m', 'yaw_err_deg'(방향 지정 없으면 None)} — 부호: 로버 yaw - 목표 yaw, (-180, 180]."""
    out = {'pos_err_m': math.hypot(pose[0] - it['x'], pose[1] - it['y']), 'yaw_err_deg': None}
    if it.get('yaw') is not None:
        out['yaw_err_deg'] = math.degrees(wrap_pi(pose[3] - it['yaw']))
    return out


class JsonlLog:
    """사건 한 줄 = JSON 한 줄. 줄마다 flush + fsync (지상국이 꺼져도 앞 줄은 남게).

    폴더가 없으면 mkdir=False 인 줄(세션 시작 헤더)은 메모리에 들고 있다가 처음 mkdir=True 인 줄에서 폴더를 만든다.
    이유: 기본 session_dir 은 daron_viz 녹화 폴더(bag_dir/session_name)인데, daron_viz 는 그 폴더가 이미 있으면
    녹화를 거부한다 (daron_3dmap._resolve_out_dir 와 같은 사정). 지상국을 먼저 띄워도 녹화가 막히지 않게.
    쓰기 실패(디스크 등)는 예외를 올리지 않고 error 에 남긴다 — 다음 줄 쓸 때 밀린 것부터 다시 쓴다.
    """

    def __init__(self, path):
        self.path = str(path)
        self.error = ''
        self._f = None
        self._buf: list[str] = []

    @property
    def opened(self) -> bool:
        return self._f is not None

    @property
    def pending(self) -> int:
        return len(self._buf)

    def write(self, obj: dict, mkdir: bool = True) -> bool:
        self._buf.append(json.dumps(obj, ensure_ascii=False))
        try:
            if self._f is None:
                d = os.path.dirname(self.path) or '.'
                if not os.path.isdir(d):
                    if not mkdir:
                        return False
                    os.makedirs(d, exist_ok=True)
                self._f = open(self.path, 'a', encoding='utf-8')
            while self._buf:
                self._f.write(self._buf[0] + '\n')
                self._buf.pop(0)
            self._f.flush()
            os.fsync(self._f.fileno())
            self.error = ''
            return True
        except OSError as e:
            self.error = str(e)
            return False

    def close(self):
        if self._f is not None:
            self._f.close()
            self._f = None


def read_jsonl(src):
    """src: 파일 경로 또는 줄 목록. → (사건 dict 목록, 못 읽은 줄 번호 목록). 각 사건에 '_line'(1 부터)."""
    if isinstance(src, (str, os.PathLike)):
        with open(src, encoding='utf-8') as f:
            lines = f.read().splitlines()
    else:
        lines = list(src)
    evs, bad = [], []
    for n, line in enumerate(lines, 1):
        if not line.strip():
            continue
        try:
            e = json.loads(line)
        except ValueError:
            bad.append(n)                                   # 꺼지는 순간 잘린 마지막 줄 등
            continue
        if not isinstance(e, dict) or 'event' not in e:
            bad.append(n)
            continue
        e['_line'] = n
        evs.append(e)
    return evs, bad


def max_logged_id(path) -> int:
    """기존 기록의 가장 큰 웨이포인트 id (없으면 0)."""
    if not os.path.exists(path):
        return 0
    try:
        evs, _ = read_jsonl(path)
    except OSError:
        return 0
    ids = [e['id'] for e in evs if isinstance(e.get('id'), int)]
    ids += [i for e in evs for i in (e.get('ids') or []) if isinstance(i, int)]
    return max(ids, default=0)


def reconstruct(src) -> dict:
    """waypoints.jsonl → {'waypoints': [...], 'headers': [...], 'bad': [줄 번호]}.

    waypoints 는 'added' 순서. 항목 = {key, id, x, y, z, yaw, frame, session, status, arrived_pose, source, line}
      yaw [rad] = goal.yaw_deg (261011 부터, 없으면 None)
      status: 'arrived' | 'pending'(미도착) | 'removed'(삭제·전체 삭제)
      key = (헤더 블록 번호, id) — 지상국은 같은 폴더에 이어 쓸 때 id 를 이어 매기지만(continue_ids),
            손으로 합친 파일 등에서 id 가 겹쳐도 섞이지 않게. 도착·삭제는 같은 블록 → 없으면 가장 최근 같은 id 에 붙인다.
    좌표는 json 이 읽은 float 그대로 (반올림 없음).
    """
    evs, bad = read_jsonl(src)
    headers, wps, by_key = [], [], {}
    block, hdr = 0, {}

    def find(i):
        if not isinstance(i, int):
            return None
        if (block, i) in by_key:
            return by_key[(block, i)]
        for b in range(block - 1, -1, -1):
            if (b, i) in by_key:
                return by_key[(b, i)]
        return None

    for e in evs:
        ev = e.get('event')
        if ev == 'session_start':
            block += 1
            hdr = e
            headers.append(e)
        elif ev == 'map_id' and headers:                    # [261008 밤] 재추정으로 나중에 정해진 map_id → 이 블록 헤더에 반영
            hdr['map_id'] = e.get('map_id', hdr.get('map_id'))
        elif ev == 'added':
            g = e.get('goal') or {}
            try:
                x, y, z = float(g['x']), float(g['y']), float(g.get('z', 0.0))
                yaw = None if g.get('yaw_deg') is None else math.radians(float(g['yaw_deg']))
            except (KeyError, TypeError, ValueError):
                bad.append(e['_line'])
                continue
            w = dict(key=(block, e.get('id')), id=e.get('id'), x=x, y=y, z=z, yaw=yaw,
                     frame=e.get('frame', hdr.get('frame')), session=e.get('session', hdr.get('session')),
                     status='pending', arrived_pose=None, source=e.get('source'), line=e['_line'])
            if w['key'] in by_key:                          # 같은 블록에 같은 id 가 또 — 뒤의 것을 따로 둔다
                w['key'] = (block, e.get('id'), e['_line'])
            by_key[w['key']] = w
            wps.append(w)
        elif ev == 'arrived':
            w = find(e.get('id'))
            if w is not None:
                w['status'], w['arrived_pose'] = 'arrived', e.get('arrived_pose')
        elif ev == 'moved':                                 # [261011] 루프 클로저 재동기화로 옮김
            for mv in e.get('moves') or []:
                w = find(mv.get('id'))
                to = mv.get('to') or {}
                if w is None:
                    continue
                try:
                    w['x'], w['y'], w['z'] = float(to['x']), float(to['y']), float(to.get('z', w['z']))
                    if 'yaw_deg' in to:
                        w['yaw'] = None if to['yaw_deg'] is None else math.radians(float(to['yaw_deg']))
                except (KeyError, TypeError, ValueError):
                    bad.append(e['_line'])
        elif ev in REMOVED_EVENTS:
            for i in ([e.get('id')] if ev == 'removed' else (e.get('ids') or [])):
                w = find(i)
                if w is not None and w['status'] == 'pending':
                    w['status'] = 'removed'
    return {'waypoints': wps, 'headers': headers, 'bad': sorted(bad)}


def arrived_unique(rec: dict) -> list:
    """[261008 밤] 기록에서 도착한 웨이포인트만, 같은 좌표(1e-6 m)·방위는 한 줄로(마지막 도착 기준, 도착 횟수 n_arrived).
    진행 중인 세션에서 '이번 세션 도착 목록'으로 다시 넣을 때 — 다시 넣어 또 도착하면 같은 자리가 여러 번 기록되므로."""
    by_xy = {}
    for w in rec['waypoints']:
        if w['status'] != 'arrived':
            continue
        k = (round(w['x'], 6), round(w['y'], 6),
             None if w.get('yaw') is None else round(math.degrees(w['yaw']), 3))   # [261011] 방위가 다르면 다른 줄
        prev = by_xy.pop(k, None)                       # 다시 넣으면 순서상 마지막 도착 위치로
        w = dict(w, n_arrived=(prev['n_arrived'] if prev else 0) + 1)
        by_xy[k] = w
    return list(by_xy.values())


def check_loaded(rec: dict, frame: str, map_id: str, session: str):
    """불러온 기록 검사 → (거부 사유 또는 None, 경고 목록).

    좌표계가 다르면 거부 (변환하지 않는다). map_id·세션이 다르면 경고만 — 좌표는 각 세션 기동 시점의 odom 원점 기준이라
    같은 출발점에서 시작했거나 재추정(relocalization)이 성공했을 때만 같은 자리다.
    """
    wps = [w for w in rec['waypoints'] if w['status'] != 'removed']
    if not wps:
        return '불러올 웨이포인트가 없음 (added 사건 없음 또는 전부 삭제됨)', []
    frames = {w['frame'] for w in wps}
    if frames != {frame}:
        got = ', '.join(repr(f) for f in sorted(frames, key=str))
        return f'파일 좌표계 {got} ≠ 현재 waypoint_frame {frame!r} → 거부 (변환하지 않음)', []
    warns = []
    f_maps = {str(h.get('map_id', '')) for h in rec['headers']}
    f_sess = {h.get('session') for h in rec['headers']} | {w['session'] for w in wps}
    if f_maps != {str(map_id)} or f_sess != {session}:
        warns.append('다른 세션 좌표 — 같은 출발점·재추정 성공 시에만 같은 자리'
                     f' (파일 세션 {", ".join(sorted(map(str, f_sess)))} / map_id {", ".join(sorted(f_maps)) or "없음"}'
                     f' ↔ 현재 {session} / {map_id or "없음"})')
    if rec['bad']:
        warns.append(f'못 읽은 줄 {len(rec["bad"])}개 건너뜀 (줄 {", ".join(map(str, rec["bad"][:5]))}'
                     + (' …' if len(rec['bad']) > 5 else '') + ')')
    return None, warns
