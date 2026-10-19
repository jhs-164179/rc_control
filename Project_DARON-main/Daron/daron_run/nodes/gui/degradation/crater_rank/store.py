"""Restart-safe observation journal and a bounded GUI-worker candidate queue."""
import json
import sqlite3
import threading
import math


class ObservationStore:
    def __init__(self, path):
        self.db=sqlite3.connect(str(path))
        self.db.execute('CREATE TABLE IF NOT EXISTS observations (mission TEXT, id TEXT, body TEXT NOT NULL, PRIMARY KEY(mission,id))')

    def add(self, observation):
        mission, oid = observation['mission_id'], observation['observation_id']
        if not all(isinstance(x,str) and x for x in (mission,oid)):
            raise ValueError('invalid observation identity')
        blob=json.dumps(observation,ensure_ascii=False,sort_keys=True,allow_nan=False)
        with self.db:
            existing=self.db.execute('SELECT body FROM observations WHERE mission=? AND id=?',(mission,oid)).fetchone()
            if existing:
                if existing[0]!=blob: raise ValueError('conflicting duplicate observation')
                return False
            self.db.execute('INSERT INTO observations VALUES (?,?,?)',(mission,oid,blob))
        return True

    def observations(self):
        return [json.loads(r[0]) for r in self.db.execute('SELECT body FROM observations ORDER BY mission,id')]

    def close(self): self.db.close()


class CandidateQueue:
    """Latest/best acquisition token per physical crater, never image-score best.

    Call offer from detector, pop from a separate measurement worker. A token
    references a frozen synchronized IR/depth pair; it is NOT a mutable GUI
    image. Cadence is per crater; priority gives least-recently-served IDs first.
    No dependency on NumPy/Numba/ROS. A queue instance belongs to one mission.
    """
    def __init__(self, min_interval_s=5.0, max_pending=32):
        if not math.isfinite(min_interval_s) or min_interval_s<=0 or type(max_pending) is not int or max_pending<1:
            raise ValueError('invalid queue settings')
        self.interval=min_interval_s;self.max_pending=max_pending
        self.pending={};self.last={};self.lock=threading.Lock()

    def offer(self, crater_id, timestamp_s, token, acquisition_quality):
        if not isinstance(crater_id,str) or not crater_id or not math.isfinite(timestamp_s) or timestamp_s<0 or not math.isfinite(acquisition_quality):
            raise ValueError('invalid candidate')
        with self.lock:
            if timestamp_s < self.last.get(crater_id,-math.inf)+self.interval: return False
            old=self.pending.get(crater_id)
            if old and (acquisition_quality,timestamp_s) <= (old[2],old[0]): return False
            if not old and len(self.pending)>=self.max_pending: return False
            self.pending[crater_id]=(timestamp_s,token,acquisition_quality)
            return True

    def pop(self):
        with self.lock:
            if not self.pending: return None
            cid=min(self.pending,key=lambda x:(self.last.get(x,-math.inf),self.pending[x][0],x))
            stamp,token,quality=self.pending.pop(cid);self.last[cid]=stamp
            return dict(crater_id=cid,timestamp_s=stamp,token=token,acquisition_quality=quality)
