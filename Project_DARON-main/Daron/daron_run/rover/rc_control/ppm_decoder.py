
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GPIO 기반 독립 PPM decoder. ROS 의존성이 없음. libgpiod Python v1 API.

한 GPIO 라인은 한 프로세스에서만 점유해야 한다.
"""
import threading
import time


class PPMDecoder:
    def __init__(self, chip='gpiochip3', line=20, num_channels=8,
                 sync_us=3000, min_us=800, max_us=2200):
        import gpiod
        self._chip = gpiod.Chip(chip)
        self._line = self._chip.get_line(line)
        try:
            self._line.request(consumer='daron_ppm', type=gpiod.LINE_REQ_EV_RISING_EDGE)
        except Exception:
            self._chip.close()
            raise
        self.num_channels = num_channels
        self.sync_us = sync_us
        self.min_us = min_us
        self.max_us = max_us
        self._pending = []
        self._synced = False
        self._last_edge_ns = None
        self._lock = threading.Lock()
        self._latest = None
        self._last_frame_at = None
        self._valid_count = 0
        self._invalid_count = 0
        self._stop = threading.Event()
        self._thread = None
        self._closed = False

    def start(self):
        if self._closed:
            raise RuntimeError('PPMDecoder already closed')
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name='ppm-gpio', daemon=True)
        self._thread.start()

    def _invalidate(self):
        with self._lock:
            self._latest = None
            self._last_frame_at = None
        self._invalid_count += 1

    def _process(self, us, now):
        if us > self.sync_us:
            if self._synced:
                if len(self._pending) == self.num_channels:
                    with self._lock:
                        self._latest = tuple(self._pending)
                        self._last_frame_at = now
                        self._valid_count += 1
                else:
                    self._invalidate()
            self._pending = []
            self._synced = True
            return
        if not self._synced:
            return
        if self.min_us <= us <= self.max_us and len(self._pending) < self.num_channels:
            self._pending.append(int(round(us)))
        else:
            self._invalidate()
            self._pending = []
            self._synced = False

    def _run(self):
        while not self._stop.is_set():
            try:
                if not self._line.event_wait(sec=0.05):
                    continue
                ev = self._line.event_read()
                edge = ev.timestamp
                if self._last_edge_ns is not None:
                    delta = (edge - self._last_edge_ns) / 1000.0
                    if delta > 0:
                        self._process(delta, time.monotonic())
                self._last_edge_ns = edge
            except Exception:
                # 실패한 스레드가 종료되면 최신 프레임도 timeout으로 무효 처리됨.
                self._invalidate()
                self._stop.set()

    def snapshot(self, timeout_s=0.3):
        """(list[int] | None, frame_time | None). 신선하지 않으면 None."""
        now = time.monotonic()
        with self._lock:
            ts = self._last_frame_at
            if ts is None or now - ts > timeout_s:
                return None, ts
            return list(self._latest), ts

    def get_channels(self, timeout_s=0.3):
        return self.snapshot(timeout_s)[0]

    def close(self):
        if self._closed:
            return
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._line.release()
        self._chip.close()
        self._closed = True

    stop = close
