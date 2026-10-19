#!/usr/bin/env python3
"""D405 ground-station receiver supporting RAW and Zstd per stream.

Each stream can independently use one of:
- raw  : sensor_msgs/msg/Image
- zstd : sensor_msgs/msg/CompressedImage

Typical experiments:
A. Depth Zstd + IR1 Raw  + IR2 Raw
B. Depth Zstd + IR1 Zstd + IR2 Raw
C. Depth Zstd + IR1 Zstd + IR2 Zstd

The receiver keeps only the latest message for each stream to avoid queueing
old live frames during congestion.

[261011] mode=off: 그 스트림은 구독하지 않는다.
[261011] republish=True: zstd 스트림을 풀어서 sensor_msgs/Image 로
<republish_prefix>/<stream>/image_rect_raw (기본 /camera/camera/...) 에 다시 낸다.
RViz2(Humble)엔 zstd image_transport 가 없어서 CompressedImage 를 못 보므로,
RViz·live_tsdf·depth_colorize·cuVSLAM 이 원래 토픽 그대로 받게 하는 용도.
header(stamp, frame_id) 는 송신 메시지 것을 그대로 쓴다. 표시 루프와 별개로
콜백에서 바로 발행하므로 display_fps 와 무관하게 수신 속도 그대로 나간다.
⚠ image_relay(daron_viz source:=remote) 가 같은 스트림을 중계하면 발행자가 둘이 된다
  — 그 스트림은 image_relay streams 에서 빼야 한다.
  실행: ~/venv/cuvslam/bin/python3 video_receiver.py --republish=True --display=False
"""

import array
import re
import os
import shlex
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
import fire
import numpy as np
import rclpy
import zstandard as zstd

from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import CompressedImage, Image

os.environ["QT_QPA_FONTDIR"] = "/usr/share/fonts/truetype/dejavu"


DEFAULT_DEPTH_RAW_TOPIC = "/camera/camera/depth/image_rect_raw"
DEFAULT_DEPTH_ZSTD_TOPIC = "/camera/transport/depth_zstd"

DEFAULT_INFRA1_RAW_TOPIC = "/radxa/camera/infra1/image_rect_raw"
DEFAULT_INFRA1_ZSTD_TOPIC = "/camera/transport/ir1_zstd"

DEFAULT_INFRA2_RAW_TOPIC = "/radxa/camera/infra2/image_rect_raw"
DEFAULT_INFRA2_ZSTD_TOPIC = "/camera/transport/ir2_zstd"


class DataReceiver(Node):
    """Receive, decode, display, save, and monitor D405 streams."""

    def __init__(
        self,
        output_dir="saved_data",
        display=True,
        save=False,
        display_fps=15.0,
        depth_save_every=30,
        infra_save_every=30,
        png_compression=1,
        min_depth_mm=100.0,
        max_depth_mm=5000.0,
        depth_unit_mm=1.0,
        depth_width=848,
        depth_height=480,
        depth_mode="zstd",
        infra1_mode="zstd",
        infra2_mode="zstd",
        depth_raw_topic=DEFAULT_DEPTH_RAW_TOPIC,
        depth_zstd_topic=DEFAULT_DEPTH_ZSTD_TOPIC,
        infra1_raw_topic=DEFAULT_INFRA1_RAW_TOPIC,
        infra1_zstd_topic=DEFAULT_INFRA1_ZSTD_TOPIC,
        infra2_raw_topic=DEFAULT_INFRA2_RAW_TOPIC,
        infra2_zstd_topic=DEFAULT_INFRA2_ZSTD_TOPIC,
        republish=False,
        republish_prefix="/camera/camera",
    ):
        """Initialize receiver state.

        Args:
            output_dir: Root save directory.
            display: Show OpenCV viewer when True.
            save: Save decoded frames when True.
            display_fps: GUI refresh rate.
            depth_save_every: Save one depth frame every N decoded frames.
            infra_save_every: Save one IR frame every N decoded frames.
            png_compression: PNG compression level 0..9.
            min_depth_mm: Display minimum depth.
            max_depth_mm: Display maximum depth.
            depth_unit_mm: Millimeters per raw depth unit.
            depth_width: Fallback depth width.
            depth_height: Fallback depth height.
            depth_mode: "raw", "zstd" or "off".
            infra1_mode: "raw", "zstd" or "off".
            infra2_mode: "raw", "zstd" or "off".
            *_raw_topic: Raw ROS Image topic.
            *_zstd_topic: Zstd CompressedImage topic.
            republish: Re-publish decoded zstd streams as sensor_msgs/Image.
            republish_prefix: Output prefix, <prefix>/<stream>/image_rect_raw.
        """
        super().__init__("data_receiver")

        self.output_dir = Path(output_dir)
        self.display = bool(display)
        self.save = bool(save)
        self.display_fps = max(1.0, float(display_fps))
        self.depth_save_every = max(1, int(depth_save_every))
        self.infra_save_every = max(1, int(infra_save_every))
        self.png_compression = int(np.clip(int(png_compression), 0, 9))

        self.min_depth_mm = float(min_depth_mm)
        self.max_depth_mm = float(max_depth_mm)
        self.depth_unit_mm = float(depth_unit_mm)
        self.depth_width = int(depth_width)
        self.depth_height = int(depth_height)

        if self.max_depth_mm <= self.min_depth_mm:
            raise ValueError("max_depth_mm must be greater than min_depth_mm")

        self.modes = {
            "depth": self._normalize_mode(depth_mode),
            "infra1": self._normalize_mode(infra1_mode),
            "infra2": self._normalize_mode(infra2_mode),
        }

        self.raw_topics = {
            "depth": str(depth_raw_topic),
            "infra1": str(infra1_raw_topic),
            "infra2": str(infra2_raw_topic),
        }
        self.zstd_topics = {
            "depth": str(depth_zstd_topic),
            "infra1": str(infra1_zstd_topic),
            "infra2": str(infra2_zstd_topic),
        }

        self.latest_messages = {
            "depth": None,
            "infra1": None,
            "infra2": None,
        }

        self.frames = {
            "color": None,
            "depth": None,
            "infra1": None,
            "infra2": None,
        }

        self.stats = {
            "depth": self.make_stat(),
            "infra1": self.make_stat(),
            "infra2": self.make_stat(),
        }

        self.latest_report = {
            "display_fps": 0.0,
            "total_mb_s": 0.0,
            "total_mbps": 0.0,
        }

        self.message_lock = threading.Lock()
        self.stats_lock = threading.Lock()

        self.last_stats_time = time.monotonic()
        self.display_frame_count = 0
        self.save_counts = {
            "depth": 0,
            "infra1": 0,
            "infra2": 0,
        }

        # One Zstd decompressor per decode worker thread.
        # A single shared ZstdDecompressor can be unsafe when multiple
        # streams are decompressed concurrently by ThreadPoolExecutor.
        self._zstd_tls = threading.local()
        self.zstd_format_cache = {}

        if self.save:
            self.output_dir.mkdir(parents=True, exist_ok=True)
            for key in ("depth", "infra1", "infra2"):
                (self.output_dir / key).mkdir(exist_ok=True)

        realtime_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

        # [261011] zstd 스트림만 재발행. RELIABLE/VOLATILE 5 = image_relay 출력과 같은 QoS
        # (BEST_EFFORT·RELIABLE 구독자 모두 붙는다).
        self.republishers = {}
        if bool(republish):
            republish_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            prefix = str(republish_prefix).rstrip("/")
            for key in ("depth", "infra1", "infra2"):
                if self.modes[key] != "zstd":
                    continue
                out_topic = f"{prefix}/{key}/image_rect_raw"
                self.republishers[key] = self.create_publisher(Image, out_topic, republish_qos)
                self.get_logger().info(f"  republish {key}: {self._active_topic(key)} -> {out_topic}")

        self._create_stream_subscription("depth", realtime_qos)
        self._create_stream_subscription("infra1", realtime_qos)
        self._create_stream_subscription("infra2", realtime_qos)

        self.get_logger().info(
            "D405 receiver started\n"
            f"  depth : mode={self.modes['depth']} | topic={self._active_topic('depth')}\n"
            f"  infra1: mode={self.modes['infra1']} | topic={self._active_topic('infra1')}\n"
            f"  infra2: mode={self.modes['infra2']} | topic={self._active_topic('infra2')}\n"
            f"  display={self.display}, target={self.display_fps:.1f} FPS\n"
            f"  save={self.save}, output={self.output_dir}"
        )

    @staticmethod
    def _normalize_mode(mode):
        """Validate and normalize one stream mode."""
        value = str(mode).strip().lower()
        if value not in ("raw", "zstd", "off"):
            raise ValueError(f"mode must be 'raw', 'zstd' or 'off', got: {mode}")
        return value

    def _active_topic(self, key):
        """Return the topic currently selected for a stream."""
        return self.zstd_topics[key] if self.modes[key] == "zstd" else self.raw_topics[key]

    def _create_stream_subscription(self, key, qos):
        """Create Image or CompressedImage subscriber according to stream mode."""
        topic = self._active_topic(key)

        # [261011] off = 구독 안 함 (daron_viz remote_depth:=zstd 는 depth 만 받고 IR 은 image_relay 가 중계)
        if self.modes[key] == "off":
            return

        if self.modes[key] == "zstd":
            self.create_subscription(
                CompressedImage,
                topic,
                lambda msg, stream=key: self.image_callback(stream, msg),
                qos,
            )
        else:
            self.create_subscription(
                Image,
                topic,
                lambda msg, stream=key: self.image_callback(stream, msg),
                qos,
            )

    @staticmethod
    def make_stat():
        """Create empty receive statistics."""
        return {
            "count": 0,
            "bytes": 0,
            "delay_ms": float("nan"),
            "hz": 0.0,
            "mb_s": 0.0,
            "mbps": 0.0,
        }

    @staticmethod
    def format_delay(delay_ms):
        """Format delay for printing."""
        if not np.isfinite(delay_ms):
            return "  n/a"
        return f"{delay_ms:6.1f}"

    def image_callback(self, key, msg):
        """Keep only newest message and update receive statistics."""
        receive_wall_time = time.time()

        stamp = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        delay_ms = (
            (receive_wall_time - stamp) * 1000.0
            if stamp > 0.0
            else float("nan")
        )

        data_size = len(msg.data)

        with self.message_lock:
            self.latest_messages[key] = msg

        with self.stats_lock:
            stat = self.stats[key]
            stat["count"] += 1
            stat["bytes"] += data_size
            stat["delay_ms"] = delay_ms

        if key in self.republishers:
            try:
                self.republish_zstd(key, msg)
            except Exception as exc:
                self.get_logger().error(f"{key} republish failed: {exc}")

    def republish_zstd(self, key, msg):
        """Decompress one zstd CompressedImage and publish it as sensor_msgs/Image.

        압축 해제한 바이트를 reshape 없이 그대로 data 에 넣는다 (step·endian 은 송신 메타데이터 그대로).
        """
        raw, metadata = self.decompress_zstd(msg)

        out = Image()
        out.header = msg.header
        out.height = metadata["height"]
        out.width = metadata["width"]
        out.encoding = metadata["encoding"]
        out.is_bigendian = int(metadata["bigendian"])
        out.step = metadata["step"]
        out.data = array.array("B", raw[: metadata["height"] * metadata["step"]])
        self.republishers[key].publish(out)

    def take_latest_messages(self):
        """Atomically take latest pending messages and clear slots."""
        with self.message_lock:
            messages = self.latest_messages.copy()
            for key in self.latest_messages:
                self.latest_messages[key] = None
        return messages

    def parse_zstd_format(self, format_string):
        """Parse generic zstd_sender metadata from CompressedImage.format."""
        text = str(format_string or "")

        if text in self.zstd_format_cache:
            return self.zstd_format_cache[text]

        pairs = dict(
            (key.lower(), value)
            for key, value in re.findall(
                r"([A-Za-z_][A-Za-z0-9_]*)\s*=\s*([^;,\|\s]+)",
                text,
            )
        )

        encoding = pairs.get("encoding", "16UC1")
        width = int(pairs.get("width", self.depth_width))
        height = int(pairs.get("height", self.depth_height))

        if encoding in ("mono8", "8UC1"):
            bytes_per_pixel = 1
        elif encoding in ("16UC1", "mono16"):
            bytes_per_pixel = 2
        else:
            raise ValueError(f"Unsupported Zstd encoding: {encoding}")

        step = int(pairs.get("step", width * bytes_per_pixel))
        bigendian = int(pairs.get("bigendian", pairs.get("is_bigendian", "0")))
        raw_size = int(pairs.get("raw_size", pairs.get("size", height * step)))

        metadata = {
            "encoding": encoding,
            "width": width,
            "height": height,
            "step": step,
            "bigendian": bool(bigendian),
            "raw_size": raw_size,
        }

        self.zstd_format_cache[text] = metadata
        return metadata

    def _get_zstd_decompressor(self):
        """Return one ZstdDecompressor dedicated to the current worker thread."""
        decompressor = getattr(self._zstd_tls, "decompressor", None)
        if decompressor is None:
            decompressor = zstd.ZstdDecompressor()
            self._zstd_tls.decompressor = decompressor
        return decompressor

    def decompress_zstd(self, msg):
        """Decompress one Zstd CompressedImage using a thread-local decoder."""
        metadata = self.parse_zstd_format(msg.format)

        decompressor = self._get_zstd_decompressor()
        raw = decompressor.decompress(
            bytes(msg.data),
            max_output_size=metadata["raw_size"],
        )

        if len(raw) != metadata["raw_size"]:
            raise ValueError(
                f'Decompressed size mismatch: {len(raw)} != {metadata["raw_size"]}'
            )

        return raw, metadata

    def decode_raw_depth(self, msg):
        """Convert raw sensor_msgs/Image depth to uint16 2-D array."""
        if msg.encoding not in ("16UC1", "mono16"):
            raise ValueError(f"Unexpected raw depth encoding: {msg.encoding}")

        dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
        row_elements = int(msg.step) // dtype.itemsize
        expected = int(msg.height) * row_elements

        depth = np.frombuffer(
            msg.data,
            dtype=dtype,
            count=expected,
        ).reshape(int(msg.height), row_elements)

        return depth[:, : int(msg.width)].astype(np.uint16, copy=True)

    def decode_raw_infra(self, msg):
        """Convert raw sensor_msgs/Image IR to mono8 2-D array."""
        if msg.encoding not in ("mono8", "8UC1"):
            raise ValueError(f"Unexpected raw IR encoding: {msg.encoding}")

        raw = np.frombuffer(msg.data, dtype=np.uint8)
        required = int(msg.height) * int(msg.step)

        if raw.size < required:
            raise ValueError(f"IR buffer too small: {raw.size} < {required}")

        image = raw[:required].reshape(int(msg.height), int(msg.step))
        return image[:, : int(msg.width)].copy()

    def decode_zstd_depth(self, msg):
        """Decompress Zstd depth to uint16 2-D array."""
        raw, metadata = self.decompress_zstd(msg)

        if metadata["encoding"] not in ("16UC1", "mono16"):
            raise ValueError(f"Unexpected Zstd depth encoding: {metadata['encoding']}")

        dtype = np.dtype(">u2" if metadata["bigendian"] else "<u2")
        row_elements = metadata["step"] // dtype.itemsize
        expected = metadata["height"] * row_elements

        depth = np.frombuffer(
            raw,
            dtype=dtype,
            count=expected,
        ).reshape(metadata["height"], row_elements)

        return depth[:, : metadata["width"]].astype(np.uint16, copy=True)

    def decode_zstd_infra(self, msg):
        """Decompress Zstd IR to mono8 2-D array."""
        raw, metadata = self.decompress_zstd(msg)

        if metadata["encoding"] not in ("mono8", "8UC1"):
            raise ValueError(f"Unexpected Zstd IR encoding: {metadata['encoding']}")

        row_elements = metadata["step"]
        expected = metadata["height"] * row_elements

        image = np.frombuffer(
            raw,
            dtype=np.uint8,
            count=expected,
        ).reshape(metadata["height"], row_elements)

        return image[:, : metadata["width"]].copy()

    def decode_message(self, key, msg):
        """Decode one message according to stream type and selected mode."""
        mode = self.modes[key]

        if key == "depth":
            return (
                self.decode_zstd_depth(msg)
                if mode == "zstd"
                else self.decode_raw_depth(msg)
            )

        if key in ("infra1", "infra2"):
            return (
                self.decode_zstd_infra(msg)
                if mode == "zstd"
                else self.decode_raw_infra(msg)
            )

        return None

    def depth_to_display(self, depth):
        """Colorize depth for viewer only."""
        depth_mm = depth.astype(np.float32) * self.depth_unit_mm
        valid = depth != 0

        clipped = np.clip(
            depth_mm,
            self.min_depth_mm,
            self.max_depth_mm,
        )

        normalized = (
            (clipped - self.min_depth_mm)
            / (self.max_depth_mm - self.min_depth_mm)
            * 255.0
        ).astype(np.uint8)

        image = cv2.applyColorMap(normalized, cv2.COLORMAP_TURBO)
        image[~valid] = 0
        return image

    @staticmethod
    def infra_to_display(infra):
        """Convert mono8 IR to BGR for display."""
        return cv2.cvtColor(infra, cv2.COLOR_GRAY2BGR)

    @staticmethod
    def message_stamp_ns(msg):
        """Return ROS message timestamp in nanoseconds."""
        return (
            int(msg.header.stamp.sec) * 1_000_000_000
            + int(msg.header.stamp.nanosec)
        )

    def save_frame(self, key, raw_image, msg):
        """Save selected raw reconstructed frames losslessly as PNG."""
        if not self.save or raw_image is None:
            return

        self.save_counts[key] += 1
        every = (
            self.depth_save_every
            if key == "depth"
            else self.infra_save_every
        )

        if self.save_counts[key] % every != 0:
            return

        stamp_ns = self.message_stamp_ns(msg)
        if stamp_ns <= 0:
            stamp_ns = time.time_ns()

        path = self.output_dir / key / f"{stamp_ns}.png"

        ok = cv2.imwrite(
            str(path),
            raw_image,
            [cv2.IMWRITE_PNG_COMPRESSION, self.png_compression],
        )

        if not ok:
            self.get_logger().error(f"Failed to save {key}: {path}")

    def update_frames_parallel(self, decode_pool):
        """Decode latest messages in parallel and update save/display buffers."""
        messages = self.take_latest_messages()
        futures = {}

        for key, msg in messages.items():
            if msg is not None:
                futures[key] = (
                    msg,
                    decode_pool.submit(self.decode_message, key, msg),
                )

        for key, (msg, future) in futures.items():
            try:
                raw = future.result()

                if raw is None:
                    continue

                self.save_frame(key, raw, msg)

                if key == "depth":
                    self.frames[key] = self.depth_to_display(raw)
                else:
                    self.frames[key] = self.infra_to_display(raw)

            except Exception as exc:
                self.get_logger().error(f"{key} worker failed: {exc}")

    @staticmethod
    def resize_frame(image, width, height, nearest=False):
        """Resize image for viewer tile."""
        if image.shape[:2] == (height, width):
            return image.copy()

        interpolation = (
            cv2.INTER_NEAREST
            if nearest
            else cv2.INTER_AREA
        )

        return cv2.resize(
            image,
            (width, height),
            interpolation=interpolation,
        )

    def make_tile(self, image, title, text_color, width, height, nearest=False):
        """Build one labeled viewer tile."""
        if image is None:
            tile = np.zeros((height, width, 3), dtype=np.uint8)

            cv2.putText(
                tile,
                # f"{title} (Waiting...)",
                f"",
                (20, height // 2),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 0, 255),
                2,
                cv2.LINE_AA,
            )

            return tile

        tile = self.resize_frame(
            image,
            width,
            height,
            nearest=nearest,
        )

        cv2.rectangle(
            tile,
            (0, 0),
            (width, 48),
            (0, 0, 0),
            thickness=-1,
        )

        cv2.putText(
            tile,
            title,
            (15, 33),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            text_color,
            2,
            cv2.LINE_AA,
        )

        return tile

    def build_grid(self):
        """Build 2x2 monitoring grid."""
        target_height = self.depth_height
        target_width = self.depth_width

        for key in ("depth", "infra1", "infra2"):
            frame = self.frames[key]
            if frame is not None:
                target_height, target_width = frame.shape[:2]
                break

        color_tile = self.make_tile(
            self.frames["color"],
            # "RGB (disabled)",
            "",
            (0, 255, 0),
            target_width,
            target_height,
        )

        depth_tile = self.make_tile(
            self.frames["depth"],
            f"Depth ({self.modes['depth']})",
            (0, 255, 255),
            target_width,
            target_height,
            nearest=True,
        )

        infra1_tile = self.make_tile(
            self.frames["infra1"],
            f"Left IR ({self.modes['infra1']})",
            (255, 255, 255),
            target_width,
            target_height,
        )

        infra2_tile = self.make_tile(
            self.frames["infra2"],
            f"Right IR ({self.modes['infra2']})",
            (255, 255, 255),
            target_width,
            target_height,
        )

        top = np.hstack((color_tile, depth_tile))
        bottom = np.hstack((infra1_tile, infra2_tile))
        return np.vstack((top, bottom))
    # def build_grid(self):
    #     """Build 1x3 monitoring grid: Depth | IR1 | IR2."""

    #     target_height = self.depth_height
    #     target_width = self.depth_width

    #     for key in ("depth", "infra1", "infra2"):
    #         frame = self.frames[key]

    #         if frame is not None:
    #             target_height, target_width = frame.shape[:2]
    #             break

    #     depth_tile = self.make_tile(
    #         self.frames["depth"],
    #         f"Depth ({self.modes['depth']})",
    #         (0, 255, 255),
    #         target_width,
    #         target_height,
    #         nearest=True,
    #     )

    #     infra1_tile = self.make_tile(
    #         self.frames["infra1"],
    #         f"Left IR ({self.modes['infra1']})",
    #         (255, 255, 255),
    #         target_width,
    #         target_height,
    #     )

    #     infra2_tile = self.make_tile(
    #         self.frames["infra2"],
    #         f"Right IR ({self.modes['infra2']})",
    #         (255, 255, 255),
    #         target_width,
    #         target_height,
    #     )

    #     return np.hstack(
    #         (
    #             depth_tile,
    #             infra1_tile,
    #             infra2_tile,
    #         )
    #     )

    def calculate_stats(self):
        """Calculate per-stream FPS and bandwidth once per second."""
        now = time.monotonic()
        elapsed = now - self.last_stats_time

        if elapsed < 1.0:
            return False

        total_bytes = 0

        with self.stats_lock:
            for stat in self.stats.values():
                stat["hz"] = stat["count"] / elapsed
                stat["mb_s"] = (
                    stat["bytes"] / 1_000_000.0
                ) / elapsed
                stat["mbps"] = stat["mb_s"] * 8.0

                total_bytes += stat["bytes"]

                stat["count"] = 0
                stat["bytes"] = 0

        total_mb_s = (
            total_bytes / 1_000_000.0
        ) / elapsed

        self.latest_report = {
            "display_fps": self.display_frame_count / elapsed,
            "total_mb_s": total_mb_s,
            "total_mbps": total_mb_s * 8.0,
        }

        self.display_frame_count = 0
        self.last_stats_time = now
        return True

    def print_stats(self):
        """Print mode, FPS, delay, and payload bandwidth."""
        with self.stats_lock:
            snapshot = {
                key: value.copy()
                for key, value in self.stats.items()
            }

        print(
            "\n"
            f'[D405 Monitor - {time.strftime("%H:%M:%S")}]\n'
            f'  Depth [{self.modes["depth"]:4s}] | '
            f'Hz: {snapshot["depth"]["hz"]:5.1f} | '
            f'Delay: {self.format_delay(snapshot["depth"]["delay_ms"])} ms | '
            f'Payload: {snapshot["depth"]["mb_s"]:6.2f} MB/s\n'
            f'  IR1   [{self.modes["infra1"]:4s}] | '
            f'Hz: {snapshot["infra1"]["hz"]:5.1f} | '
            f'Delay: {self.format_delay(snapshot["infra1"]["delay_ms"])} ms | '
            f'Payload: {snapshot["infra1"]["mb_s"]:6.2f} MB/s\n'
            f'  IR2   [{self.modes["infra2"]:4s}] | '
            f'Hz: {snapshot["infra2"]["hz"]:5.1f} | '
            f'Delay: {self.format_delay(snapshot["infra2"]["delay_ms"])} ms | '
            f'Payload: {snapshot["infra2"]["mb_s"]:6.2f} MB/s\n'
            f'  ------------------------------------------------------------\n'
            f'  Total : {self.latest_report["total_mb_s"]:6.2f} MB/s '
            f'({self.latest_report["total_mbps"]:6.1f} Mbps)',
            flush=True,
        )


def main(
    output_dir="saved_data",
    display=True,
    save=False,
    display_fps=15.0,
    depth_save_every=30,
    infra_save_every=30,
    png_compression=1,
    min_depth_mm=100.0,
    max_depth_mm=5000.0,
    depth_unit_mm=1.0,
    depth_width=848,
    depth_height=480,
    depth_mode="zstd",
    infra1_mode="zstd",
    infra2_mode="zstd",
    depth_raw_topic=DEFAULT_DEPTH_RAW_TOPIC,
    depth_zstd_topic=DEFAULT_DEPTH_ZSTD_TOPIC,
    infra1_raw_topic=DEFAULT_INFRA1_RAW_TOPIC,
    infra1_zstd_topic=DEFAULT_INFRA1_ZSTD_TOPIC,
    infra2_raw_topic=DEFAULT_INFRA2_RAW_TOPIC,
    infra2_zstd_topic=DEFAULT_INFRA2_ZSTD_TOPIC,
    republish=False,
    republish_prefix="/camera/camera",
    decode_workers=3,
    ros_threads=4,
    ros_args="",
    stats_every=1,
):
    """Run receiver with Fire-style arguments.

    stats_every: 통계를 N 초마다 출력 (launch 안에서 돌릴 때 로그 줄이기용, 261011).
    """
    rclpy_args = (
        shlex.split(str(ros_args))
        if ros_args
        else []
    )

    rclpy.init(args=rclpy_args)

    node = DataReceiver(
        output_dir=output_dir,
        display=display,
        save=save,
        display_fps=display_fps,
        depth_save_every=depth_save_every,
        infra_save_every=infra_save_every,
        png_compression=png_compression,
        min_depth_mm=min_depth_mm,
        max_depth_mm=max_depth_mm,
        depth_unit_mm=depth_unit_mm,
        depth_width=depth_width,
        depth_height=depth_height,
        depth_mode=depth_mode,
        infra1_mode=infra1_mode,
        infra2_mode=infra2_mode,
        depth_raw_topic=depth_raw_topic,
        depth_zstd_topic=depth_zstd_topic,
        infra1_raw_topic=infra1_raw_topic,
        infra1_zstd_topic=infra1_zstd_topic,
        infra2_raw_topic=infra2_raw_topic,
        infra2_zstd_topic=infra2_zstd_topic,
        republish=republish,
        republish_prefix=republish_prefix,
    )

    ros_executor = MultiThreadedExecutor(
        num_threads=max(1, int(ros_threads))
    )

    ros_executor.add_node(node)

    ros_thread = threading.Thread(
        target=ros_executor.spin,
        daemon=True,
    )

    ros_thread.start()

    decode_pool = ThreadPoolExecutor(
        max_workers=max(1, int(decode_workers)),
        thread_name_prefix="d405_decode",
    )

    window_name = "D405 RAW/Zstd Stream View"

    if node.display:
        cv2.namedWindow(
            window_name,
            cv2.WINDOW_NORMAL,
        )

    frame_period = 1.0 / node.display_fps
    next_frame_time = time.monotonic()
    stats_every = max(1, int(stats_every))
    stats_count = 0

    try:
        while rclpy.ok():
            # 표시·저장 둘 다 꺼져 있으면(republish 전용) 여기서 다시 풀 필요 없다
            if node.display or node.save:
                node.update_frames_parallel(
                    decode_pool
                )

            if node.display:
                grid = node.build_grid()

                cv2.imshow(
                    window_name,
                    grid,
                )

                node.display_frame_count += 1

                key = cv2.waitKey(1) & 0xFF

                if key in (ord("q"), 27):
                    break

            if node.calculate_stats():
                stats_count += 1
                if stats_count % stats_every == 0:
                    node.print_stats()

            next_frame_time += frame_period

            remaining = (
                next_frame_time
                - time.monotonic()
            )

            if remaining > 0.0:
                time.sleep(remaining)
            else:
                next_frame_time = time.monotonic()

    except KeyboardInterrupt:
        pass

    finally:
        decode_pool.shutdown(
            wait=True,
            cancel_futures=True,
        )

        ros_executor.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

        ros_thread.join(
            timeout=2.0
        )

        if node.display:
            cv2.destroyAllWindows()


if __name__ == "__main__":
    fire.Fire(main)