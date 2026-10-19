#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
susp_log.py — 실험 로그.

CSV 는 순수 데이터만 담고, 파라미터는 같은 이름의 .meta.json 에 따로 쓴다.
CSV 안에 주석 헤더를 섞으면 pandas/엑셀에서 매번 걸린다.

파일 이름   <prefix>_<YYYYmmdd_HHMMSS>.csv / .meta.json
한 줄 = 한 제어 주기. event 열은 비어 있다가 상태 전환·마커에서만 채워진다.
"""

from __future__ import annotations

import csv
import datetime
import json
import os

HEADER = [
    "t", "state", "dir", "goal_deg", "pos_deg", "err_deg",
    "vel_dps", "cur_mA", "torque_mNm", "gc_lsb", "event",
]


class Recorder:
    def __init__(self, log_dir: str, prefix: str, meta: dict = None,
                 enabled: bool = True):
        self.enabled = enabled
        self.path = None
        self._f = None
        self._w = None
        self.rows = 0
        if not enabled:
            return
        os.makedirs(log_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.path = os.path.join(log_dir, f"{prefix}_{stamp}.csv")
        self._f = open(self.path, "w", newline="", encoding="utf-8")
        self._w = csv.writer(self._f)
        self._w.writerow(HEADER)
        self.meta_path = self.path[:-4] + ".meta.json"
        self.meta = dict(meta or {})
        self.meta.setdefault("created", stamp)
        self._flush_meta()

    def _flush_meta(self):
        if not self.enabled:
            return
        with open(self.meta_path, "w", encoding="utf-8") as f:
            json.dump(self.meta, f, ensure_ascii=False, indent=2)

    def update_meta(self, **kw):
        if not self.enabled:
            return
        self.meta.update(kw)
        self._flush_meta()

    def write(self, t, state, direction, goal_deg, pos_deg, vel_dps,
              cur_a, gc_lsb, event=""):
        if not self.enabled:
            return
        self._w.writerow([
            f"{t:.4f}", state, direction,
            f"{goal_deg:.4f}", f"{pos_deg:.4f}", f"{goal_deg - pos_deg:.4f}",
            f"{vel_dps:.2f}", f"{cur_a * 1000.0:.1f}",
            f"{cur_a * 1.304 * 1000.0:.1f}", gc_lsb, event,
        ])
        self.rows += 1

    def close(self):
        if self._f:
            self._f.close()
            self._f = None
        return self.path

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


def read_csv(path: str):
    """CSV -> (rows: list[dict], meta: dict). 숫자 열은 float 로 바꾼다."""
    num = {"t", "goal_deg", "pos_deg", "err_deg", "vel_dps",
           "cur_mA", "torque_mNm", "gc_lsb", "dir"}
    rows = []
    with open(path, encoding="utf-8") as f:
        for r in csv.DictReader(f):
            for k in list(r):
                if k in num and r[k] != "":
                    r[k] = float(r[k])
            rows.append(r)
    meta = {}
    mp = path[:-4] + ".meta.json"
    if os.path.exists(mp):
        with open(mp, encoding="utf-8") as f:
            meta = json.load(f)
    return rows, meta
