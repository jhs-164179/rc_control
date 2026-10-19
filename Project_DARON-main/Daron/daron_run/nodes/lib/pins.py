"""검출 bbox → 맵 좌표 핀 (ROS 의존 없음). 오프라인(yolo/tools/pins_offline.py)과 실시간(cda/pin_node.py)이 같이 쓴다.

한 검출 → 점 하나 (box_to_point):
  bbox 안쪽 50% 영역 유효 depth [dmin, dmax] 중앙값 = Z, bbox 중심 (u,v) 역투영 → 카메라 광학 프레임 점.
  반지름 ≈ (x2-x1)·Z/fx/2.  D405 depth 는 infra1 과 같은 픽셀 기준이라 bbox 를 그대로 겹친다.
여러 검출 → 핀 (PinSet): 같은 class 의 기존 핀 중심과 거리 < max(join_m, 핀 반지름 + 검출 반지름) 이면
  같은 크레이터(추정 원판이 겹침)로 보고 합침(score 가중 평균), 아니면 새 핀. n >= min_count 이면 확정.
  핀이 자라며 다른 핀과 원판이 겹치게 되면 두 핀을 합친다(작은 쪽 id 는 alias 로 남음 → resolve()).
  (261001: 고정 10 cm 기준은 지름 15~25 cm 크레이터를 먼 거리 검출에서 여러 핀으로 쪼갰다 — 20260922_170303 트랙 2)
  화면 가장자리에 걸린 박스(box_on_edge)는 크레이터가 잘려 박스 중심 ≠ 크레이터 중심 → 핀 연결·횟수에는 쓰고
  위치 평균에는 안 쓴다(pos_weight=0). 가장자리 검출만 있는 핀은 그 평균을 임시 위치로 쓴다. (261001)
  핀마다 score 가 가장 높은 검출 하나(stamp, box, score)를 'top' 으로 들고 있다 — 지상국 크레이터 정보 창이
  그 프레임에 박스를 그려 보여 준다. 위치·합치기 계산에는 안 쓴다. (261006)
  [261009] 핀마다 'cands' = 크레이터 형상(경사·깊이·메시) 계산 후보 — 가장자리에 안 걸리고 Z 가 [cand_zmin, cand_zmax]
  인 검출 중 score 상위 cand_n(기본 10) 개 {stamp, box, score, z}. score 최고(top)가 계산에 맞지 않는 경우가 많아서
  (real_output: 최고 score 프레임 성공 0/11, 0.35~0.7 m·가장자리 아님만 성공) 지상국이 위에서부터 시도한다.
  [261013] 핀마다 'blocks' = 열화도 반복 관측용. cands 와 같은 조건(가장자리 아님·Z 범위)의 검출을 stamp 의 block_s(기본 5 s)
  구간마다 하나 — 박스 짧은 변이 큰 것, 같으면 score 높은 것 — 최근 block_n(기본 12) 구간 {stamp, box, score, z}.
  cands 는 score 상위라 시간상 몰려 있다(같은 5 s 구간) → 열화도 전달본의 '5 s 구간당 한 쌍 · 구간 5개 이상' 을 못 채움.
  경사·림 값이 높은 프레임을 고르지 않는다 (전달본 README §6 — 품질 기준으로만 선정).
작성 261001 — rock_data 수동 어노테이션 7장으로 검증 (같은 돌 흩어짐 ≤ 1.8 cm, yolo/README.md)
"""
from __future__ import annotations

import numpy as np

from .rotations import quat_to_mat


def slerp(q0, q1, f: float) -> np.ndarray:
    q0, q1 = np.asarray(q0, np.float64), np.asarray(q1, np.float64)
    d = float(q0 @ q1)
    if d < 0:
        q1, d = -q1, -d
    if d > 0.9995:
        q = q0 + f * (q1 - q0)
    else:
        th = np.arccos(d)
        q = (np.sin((1 - f) * th) * q0 + np.sin(f * th) * q1) / np.sin(th)
    return q / np.linalg.norm(q)


class OdomBuffer:
    """(t_ns, p, q[x,y,z,w]) 를 시간순으로 쌓고 임의 stamp 의 T_world_base 를 보간한다."""

    def __init__(self, maxlen: int = 20000):
        self.maxlen = maxlen
        self.t: list[int] = []
        self.p: list = []
        self.q: list = []

    def append(self, t_ns: int, p, q):
        if self.t and t_ns <= self.t[-1]:
            return                                  # 역순·중복 stamp 는 버림
        self.t.append(int(t_ns))
        self.p.append(np.asarray(p, np.float64))
        self.q.append(np.asarray(q, np.float64))
        if len(self.t) > self.maxlen:
            del self.t[0], self.p[0], self.q[0]

    @property
    def latest(self) -> int | None:
        return self.t[-1] if self.t else None

    def pose(self, t_ns: int, max_gap_ns: int | None = None) -> np.ndarray | None:
        """T_world_base (4x4). 버퍼 범위 밖이면 None (외삽 안 함).
        max_gap_ns: 앞뒤 샘플 간격이 이보다 크면 None — 포즈가 빠진 구간(튄 포즈 제거·SLAM 끊김)을 건너
        보간하지 않는다. stamp 가 샘플과 정확히 같으면 간격과 무관하게 그 샘플."""
        if len(self.t) < 2 or not (self.t[0] <= t_ns <= self.t[-1]):
            return None
        k = int(np.searchsorted(self.t, t_ns))
        if k < len(self.t) and self.t[k] == t_ns:
            T = np.eye(4)
            T[:3, :3] = quat_to_mat(*self.q[k])
            T[:3, 3] = self.p[k]
            return T
        k = min(max(k, 1), len(self.t) - 1)
        if max_gap_ns is not None and self.t[k] - self.t[k - 1] > max_gap_ns:
            return None
        f = (t_ns - self.t[k - 1]) / max(1, self.t[k] - self.t[k - 1])
        T = np.eye(4)
        T[:3, :3] = quat_to_mat(*slerp(self.q[k - 1], self.q[k], f))
        T[:3, 3] = self.p[k - 1] + f * (self.p[k] - self.p[k - 1])
        return T


def box_on_edge(box, width: int, height: int, margin: float) -> bool:
    """박스가 영상 테두리 margin 픽셀 안에 닿는가 (margin <= 0 이면 항상 False)."""
    if margin <= 0:
        return False
    x1, y1, x2, y2 = (float(v) for v in box)
    return x1 <= margin or y1 <= margin or x2 >= width - 1 - margin or y2 >= height - 1 - margin


def box_to_point(depth_u16, box, K, dmin=0.07, dmax=1.0, min_valid=0.3, depth_scale=1e-3):
    """반환 (status, Z, p_cam(3,), radius_m). status 가 'ok' 가 아니면 나머지는 None."""
    x1, y1, x2, y2 = (float(v) for v in box)
    fx, fy, cx, cy = K[0][0], K[1][1], K[0][2], K[1][2]
    u, v, hw, hh = (x1 + x2) / 2, (y1 + y2) / 2, (x2 - x1) / 4, (y2 - y1) / 4
    H, W = depth_u16.shape
    r0, r1 = max(0, int(v - hh)), min(H, int(v + hh) + 1)
    c0, c1 = max(0, int(u - hw)), min(W, int(u + hw) + 1)
    inner = depth_u16[r0:r1, c0:c1].astype(np.float64) * depth_scale
    ok = inner[(inner >= dmin) & (inner <= dmax)]
    if inner.size == 0 or len(ok) < min_valid * inner.size:
        return f"depth_invalid({len(ok)}/{inner.size})", None, None, None
    Z = float(np.median(ok))
    return "ok", Z, np.array([(u - cx) * Z / fx, (v - cy) * Z / fy, Z]), (x2 - x1) * Z / fx / 2


class PinSet:
    def __init__(self, join_m: float = 0.10, min_count: int = 3,
                 cand_n: int = 10, cand_zmin: float = 0.35, cand_zmax: float = 1.0,
                 block_s: float = 5.0, block_n: int = 12):
        self.join_m, self.min_count = join_m, min_count
        self.cand_n, self.cand_zmin, self.cand_zmax = cand_n, cand_zmin, cand_zmax
        self.block_ns, self.block_n = int(block_s * 1e9), block_n
        self._p: list[dict] = []
        # [261011] 검출마다 (넣은 핀 id, 위치, 위치 가중, 가장자리 가중, src) — 루프 클로저 재동기화(reposition)용
        self._recs: list[list] = []

    def add(self, p_world, score: float, radius_m: float, stamp_ns: int, cls: str = "",
            pos_weight: float = 1.0, box=None, z: float | None = None, src=None) -> tuple[int, bool]:
        """검출 하나를 넣는다. 반환 (핀 id, 새로 확정됐는지).
        pos_weight=0 (가장자리 박스): 연결·횟수에만 쓰고 위치·반지름 평균에는 안 쓴다.
        box: 주면 score 최고 검출을 'top' 으로 기록 (같은 score 면 먼저 온 것).
        z: box 와 같이 주면 형상 계산 후보(cands) 판단에 쓴다 (가장자리 아님 + Z 범위).
        src: [261011] reposition 에 넘겨 줄 값 (pin_node: odom 좌표 점·map<-odom 기준). None 이면 그 검출은 안 옮김."""
        p = np.asarray(p_world, np.float64)
        best, bd = None, np.inf
        for pin in self._p:
            if pin["cls"] != cls or pin["alias"] is not None:
                continue
            d = float(np.linalg.norm(self._center(pin) - p))
            if d < max(self.join_m, pin["radius_m"] + radius_m) and d < bd:
                best, bd = pin, d
        if best is None:
            best = dict(id=len(self._p), cls=cls, _sum=np.zeros(3), _w=0.0, _r=0.0, alias=None,
                        _sum_e=np.zeros(3), _w_e=0.0, _r_e=0.0, n=0, n_edge=0, scores=[], det=[], top=None, cands=[],
                        blocks={})
            self._p.append(best)
        was = best["n"] >= self.min_count
        if pos_weight > 0:
            w = score * pos_weight
            best["_sum"] += w * p
            best["_w"] += w
            best["_r"] += w * radius_m
        else:
            best["_sum_e"] += score * p
            best["_w_e"] += score
            best["_r_e"] += score * radius_m
            best["n_edge"] += 1
        self._recs.append([best["id"], p.copy(), score * pos_weight if pos_weight > 0 else 0.0,
                           0.0 if pos_weight > 0 else score, src])
        best["n"] += 1
        best["scores"].append(float(score))
        best["det"].append(int(stamp_ns))
        if box is not None and (best["top"] is None or score > best["top"]["score"]):
            best["top"] = dict(stamp_ns=int(stamp_ns), box=[round(float(v), 1) for v in box], score=float(score))
        if (box is not None and z is not None and pos_weight > 0 and self.cand_n > 0
                and self.cand_zmin <= z <= self.cand_zmax):
            best["cands"] = self._top_cands(best["cands"] + [dict(stamp_ns=int(stamp_ns), box=[round(float(v), 1) for v in box],
                                                                  score=float(score), z=round(float(z), 3))])
            if self.block_ns > 0 and self.block_n > 0:
                self._put_block(best, dict(stamp_ns=int(stamp_ns), box=[round(float(v), 1) for v in box],
                                           score=float(score), z=round(float(z), 3)))
        best["radius_m"] = best["_r"] / best["_w"] if best["_w"] > 0 else best["_r_e"] / best["_w_e"]
        best = self._merge_overlaps(best)
        return best["id"], (not was) and best["n"] >= self.min_count

    def _merge_overlaps(self, pin):
        """pin 과 원판이 겹치는 같은 class 핀을 합친다. 검출이 많은 쪽이 남는다."""
        while True:
            c = self._center(pin)
            other = next((q for q in self._p if q is not pin and q["alias"] is None and q["cls"] == pin["cls"]
                          and np.linalg.norm(self._center(q) - c) < max(self.join_m, q["radius_m"] + pin["radius_m"])),
                         None)
            if other is None:
                return pin
            keep, gone = (pin, other) if pin["n"] >= other["n"] else (other, pin)
            for k in ("_sum", "_sum_e"):
                keep[k] = keep[k] + gone[k]
            for k in ("_w", "_r", "_w_e", "_r_e", "n", "n_edge"):
                keep[k] += gone[k]
            keep["scores"] += gone["scores"]
            keep["det"] += gone["det"]
            if gone["top"] is not None and (keep["top"] is None or gone["top"]["score"] > keep["top"]["score"]):
                keep["top"] = gone["top"]
            keep["cands"] = self._top_cands(keep.get("cands", []) + gone.get("cands", []))
            for e in gone.get("blocks", {}).values():
                self._put_block(keep, e)
            keep["radius_m"] = keep["_r"] / keep["_w"] if keep["_w"] > 0 else keep["_r_e"] / keep["_w_e"]
            gone["alias"] = keep["id"]
            pin = keep

    def seed(self, pin: dict, loaded: dict | None = None) -> int:
        """[261011 밤] 이전 세션 핀(pins_*.json 의 한 항목)을 미리 넣는다. 반환: 새 id.
        새 검출은 add() 의 같은 판정(거리 < max(join_m, 반지름 합))으로 이 핀에 합쳐진다 → 다시 관측한 크레이터가 새 핀이 안 됨.
        위치 가중 = score_mean × 위치 검출 수 (이전 세션 평균을 그만큼의 무게로). 검출별 기록은 없어 reposition 대상 아님(src None).
        top·cands 는 비운다 — 대표 이미지·형상 계산은 이번 세션 프레임이 있어야 해서, 이번 세션 첫 검출이 top 이 된다.
        loaded: 출처 메타 (파일, 이전 id, 이전 top) → pins() 의 'loaded'."""
        xyz = np.asarray(pin["xyz"], np.float64)
        n, n_edge = int(pin.get("n", 1)), int(pin.get("n_edge", 0))
        r = float(pin.get("radius_m", 0.0))
        sm = float(pin.get("score_mean", 1.0))
        edge_only = bool(pin.get("edge_only", False))
        w = max(sm * max(n - n_edge, 1), 1e-6)
        new = dict(id=len(self._p), cls=str(pin.get("cls", "")), alias=None, n=n, n_edge=n_edge,
                   _sum=np.zeros(3), _w=0.0, _r=0.0, _sum_e=np.zeros(3), _w_e=0.0, _r_e=0.0,
                   scores=[sm] * max(n, 1), det=[], top=None, cands=[], blocks={}, radius_m=r, loaded=loaded or {})
        if edge_only:
            new["_sum_e"], new["_w_e"], new["_r_e"] = w * xyz, w, w * r
        else:
            new["_sum"], new["_w"], new["_r"] = w * xyz, w, w * r
        self._p.append(new)
        self._recs.append([new["id"], xyz.copy(), 0.0 if edge_only else w, w if edge_only else 0.0, None])
        return new["id"]

    def reposition(self, fn) -> tuple[int, float]:
        """[261011] 검출 위치를 fn(src, p) → 새 p (또는 None = 그대로) 로 바꾸고 핀 중심을 다시 계산한다.
        핀 소속·id·반지름·횟수·top 은 그대로 (지상국 이미지·크레이터 계산이 id 로 묶여 있음). 옮긴 뒤 겹치면 평소처럼 합친다.
        반환: (옮긴 검출 수, 살아 있는 핀 중심의 최대 이동 [m])."""
        before = {p["id"]: self._center(p) for p in self._p if p["alias"] is None}
        moved = 0
        for r in self._recs:
            if r[4] is None:
                continue
            q = fn(r[4], r[1])
            if q is not None:
                r[1] = np.asarray(q, np.float64)
                moved += 1
        for p in self._p:
            p["_sum"], p["_sum_e"] = np.zeros(3), np.zeros(3)
        for pid, pt, w, we, _ in self._recs:
            keep = self._p[self.resolve(pid)]
            keep["_sum"] += w * pt
            keep["_sum_e"] += we * pt
        for p in list(self._p):
            if p["alias"] is None:
                self._merge_overlaps(p)
        dmax = max((float(np.linalg.norm(self._center(self._p[self.resolve(i)]) - c)) for i, c in before.items()),
                   default=0.0)
        return moved, dmax

    @staticmethod
    def _block_rank(e):
        x1, y1, x2, y2 = e["box"]
        return min(x2 - x1, y2 - y1), e["score"], -e["stamp_ns"]

    def _put_block(self, pin, e):
        """[261013] block_s 구간마다 하나 (박스 짧은 변 → score → 먼저 온 것). 최근 block_n 구간만."""
        b = pin.setdefault("blocks", {})
        k = e["stamp_ns"] // self.block_ns
        if k not in b or self._block_rank(e) > self._block_rank(b[k]):
            b[k] = e
        for old in sorted(b)[:-self.block_n]:
            del b[old]

    def _top_cands(self, cands):
        """score 내림차순 상위 cand_n (같은 score 면 먼저 온 것)."""
        return sorted(cands, key=lambda c: (-c["score"], c["stamp_ns"]))[:self.cand_n]

    def resolve(self, pid: int) -> int:
        """합쳐져 사라진 핀 id → 지금 살아 있는 핀 id"""
        while self._p[pid]["alias"] is not None:
            pid = self._p[pid]["alias"]
        return pid

    @staticmethod
    def _center(pin) -> np.ndarray:
        return pin["_sum"] / pin["_w"] if pin["_w"] > 0 else pin["_sum_e"] / pin["_w_e"]

    def center(self, pid: int) -> np.ndarray:
        return self._center(self._p[self.resolve(pid)])

    def pins(self, frame: str = "") -> list[dict]:
        return [dict(id=p["id"], cls=p["cls"], frame=frame, xyz=[round(float(v), 4) for v in self.center(p["id"])],
                     radius_m=round(float(p["radius_m"]), 4), n=p["n"], n_edge=p["n_edge"],
                     edge_only=p["_w"] == 0,
                     score_mean=round(float(np.mean(p["scores"])), 3),
                     confirmed=p["n"] >= self.min_count, detections=sorted(p["det"]), top=p["top"], cands=p.get("cands", []),
                     blocks=[p["blocks"][k] for k in sorted(p.get("blocks", {}))],
                     merged_from=sorted(q["id"] for q in self._p if q["alias"] is not None
                                        and self.resolve(q["id"]) == p["id"]),
                     loaded=p.get("loaded") or None)
                for p in self._p if p["alias"] is None]
