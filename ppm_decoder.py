
#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import threading
from datetime import timedelta

import gpiod
from gpiod.line import Edge


class PPMDecoder:
    def __init__(
        self,
        chip="gpiochip3",
        line=20,
        num_channels=8,
        sync_us=3000,
        min_us=800,
        max_us=2200,
    ):
        self.chip = chip if chip.startswith("/dev/") else f"/dev/{chip}"
        self.line = line
        self.num_channels = num_channels
        self.sync_us = sync_us
        self.min_us = min_us
        self.max_us = max_us

        self._request = gpiod.request_lines(
            self.chip,
            consumer="daron_ppm",
            config={
                self.line: gpiod.LineSettings(
                    edge_detection=Edge.RISING
                )
            },
            event_buffer_size=256,
        )

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
        self._error = None

    def start(self):
        if self._closed:
            raise RuntimeError("PPMDecoder already closed")
        if self._thread is not None:
            return

        self._thread = threading.Thread(
            target=self._run,
            name="ppm-gpio",
            daemon=True,
        )
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

        if (
            self.min_us <= us <= self.max_us
            and len(self._pending) < self.num_channels
        ):
            self._pending.append(int(round(us)))
        else:
            self._invalidate()
            self._pending = []
            self._synced = False

    def _run(self):
        try:
            while not self._stop.is_set():
                if not self._request.wait_edge_events(
                    timeout=timedelta(milliseconds=50)
                ):
                    continue

                events = self._request.read_edge_events()

                for ev in events:
                    edge = ev.timestamp_ns

                    if self._last_edge_ns is not None:
                        delta = (
                            edge - self._last_edge_ns
                        ) / 1000.0

                        if delta > 0:
                            self._process(
                                delta,
                                time.monotonic()
                            )

                    self._last_edge_ns = edge

        except Exception as exc:
            self._error = exc
            self._invalidate()
            print(f"[PPM ERROR] {exc}", flush=True)
            self._stop.set()

    def snapshot(self, timeout_s=0.3):
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
            if self._thread.is_alive():
                raise RuntimeError(
                    "PPM worker did not stop; GPIO remains allocated"
                )

        self._request.release()
        self._closed = True

    stop = close
