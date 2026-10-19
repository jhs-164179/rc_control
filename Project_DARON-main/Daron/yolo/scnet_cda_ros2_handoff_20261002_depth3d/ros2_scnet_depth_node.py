"""ROS 2 Humble node: infra1 -> SCNet -> aligned-depth profiling -> JSON.

The two image topics are paired by their exact header stamp. Inference runs in
a worker thread and the pending pair is overwritten, so latency stays bounded
when inference is slower than the 30 Hz camera stream.
"""
from __future__ import annotations

import json
import threading
import time
from collections import OrderedDict
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from depth_profile import CameraIntrinsics, DepthProfileConfig
from cda_output import make_payload
from scnet_depth_filter import SCNetDepthCraterDetector


INFRA_TOPIC = "/camera/camera/infra1/image_rect_raw"
DEPTH_TOPIC = "/camera/camera/depth/image_rect_raw"
OUTPUT_TOPIC = "/cda/detections"
CAMERA_INFO_TOPIC = "/camera/camera/infra1/camera_info"


def stamp_ns(message: Image) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class SCNetDepthNode(Node):
    def __init__(self) -> None:
        super().__init__("scnet_depth_detector")
        self.declare_parameter("confidence", 0.25)
        self.declare_parameter("nms_iou", 0.4)
        self.declare_parameter("weights", str(Path(__file__).resolve().parent / "weights/scnet_best.pt"))
        self.declare_parameter("tau_depth_m", 0.015)
        self.declare_parameter("tau_conf", 0.65)
        self.declare_parameter("depression_polarity", "positive")
        self.declare_parameter("publish_rejected", False)
        self.declare_parameter("device", "auto")
        self.declare_parameter("half", True)
        self.declare_parameter("cuda_graph", True)
        self.declare_parameter("max_buffered_stamps", 30)
        self.declare_parameter("min_depth_m", 0.07)
        self.declare_parameter("max_depth_m", 0.0)  # zero: no upper metric cutoff
        self.declare_parameter("reliable_depth_max_m", 1.0)  # rejection policy only
        self.declare_parameter("profile_long_range", True)
        self.declare_parameter("allow_long_range_rejection", False)
        self.declare_parameter("depth_error_coefficient", 0.012)
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("invalid_raw_min", 60000)
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.invalid_raw_min = int(self.get_parameter("invalid_raw_min").value)
        if not np.isfinite(self.depth_scale) or self.depth_scale <= 0:
            raise ValueError('depth_scale must be finite and positive')
        if not 0 <= self.invalid_raw_min <= 65536:
            raise ValueError('invalid_raw_min must be 0..65536')
        max_depth = float(self.get_parameter("max_depth_m").value)

        config = DepthProfileConfig(
            mode="ground_corrected",
            n_strips=10,
            tau_depth_m=float(self.get_parameter("tau_depth_m").value),
            tau_conf=float(self.get_parameter("tau_conf").value),
            recall_first=True,
            depth_error_coefficient=float(self.get_parameter("depth_error_coefficient").value),
            reliable_depth_max_m=float(self.get_parameter("reliable_depth_max_m").value),
            profile_long_range=bool(self.get_parameter("profile_long_range").value),
            allow_long_range_rejection=bool(self.get_parameter("allow_long_range_rejection").value),
            min_depth_m=float(self.get_parameter("min_depth_m").value),
            max_depth_m=None if max_depth == 0 else max_depth,
            min_valid_ratio=0.35,
            min_depression_ratio=0.12,
            min_coherent_depression_ratio=0.08,
            min_center_depression_ratio_of_tau=0.60,
            min_bowl_amplitude_ratio_of_tau=0.45,
            depression_polarity=str(self.get_parameter("depression_polarity").value),
        )
        config.validate()
        intrinsics = CameraIntrinsics(fx=431.94, fy=431.94, cx=419.34, cy=228.48)
        self.system = SCNetDepthCraterDetector(
            weights=str(self.get_parameter("weights").value),
            device=str(self.get_parameter("device").value),
            half=bool(self.get_parameter("half").value),
            cuda_graph=bool(self.get_parameter("cuda_graph").value),
            confidence=float(self.get_parameter("confidence").value),
            nms_iou=float(self.get_parameter("nms_iou").value),
            depth_config=config,
            intrinsics=intrinsics,
        )
        self.publish_rejected = bool(self.get_parameter("publish_rejected").value)
        self.max_buffer = int(self.get_parameter("max_buffered_stamps").value)
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(String, OUTPUT_TOPIC, 10)
        self.create_subscription(Image, INFRA_TOPIC, self._on_infra, qos_profile_sensor_data)
        self.create_subscription(Image, DEPTH_TOPIC, self._on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, CAMERA_INFO_TOPIC, self._on_camera_info, qos_profile_sensor_data)

        self._infra: OrderedDict[int, Image] = OrderedDict()
        self._depth: OrderedDict[int, Image] = OrderedDict()
        self._pending: tuple[int, Image, Image] | None = None
        self._lock = threading.Lock()
        self._event = threading.Event()
        self._stopping = False
        self._paired=0;self._dropped=0;self._processed=0;self._failed=0
        self._statistics_started=time.monotonic();self._last_report=self._statistics_started
        self._last_report_count=0
        self._processing_ms=[]
        self._worker = threading.Thread(target=self._worker_loop, daemon=True)
        self._worker.start()
        self.get_logger().info(
            f"listening: {INFRA_TOPIC} + {DEPTH_TOPIC}; publishing: {OUTPUT_TOPIC}"
        )

    def _trim(self, buffer: OrderedDict[int, Image]) -> None:
        while len(buffer) > self.max_buffer:
            buffer.popitem(last=False)

    def _pair_if_ready(self, stamp: int) -> None:
        if stamp not in self._infra or stamp not in self._depth:
            return
        # Keep only the newest complete pair. This intentionally drops queued
        # frames when GPU inference cannot follow the 30 Hz input rate.
        self._paired+=1
        if self._pending is not None: self._dropped+=1
        self._pending = (stamp, self._infra.pop(stamp), self._depth.pop(stamp))
        for key in list(self._infra):
            if key <= stamp:
                self._infra.pop(key, None)
        for key in list(self._depth):
            if key <= stamp:
                self._depth.pop(key, None)
        self._event.set()

    def _on_infra(self, message: Image) -> None:
        stamp = stamp_ns(message)
        with self._lock:
            self._infra[stamp] = message
            self._trim(self._infra)
            self._pair_if_ready(stamp)

    def _on_depth(self, message: Image) -> None:
        stamp = stamp_ns(message)
        with self._lock:
            self._depth[stamp] = message
            self._trim(self._depth)
            self._pair_if_ready(stamp)

    def _on_camera_info(self, message: CameraInfo) -> None:
        if len(message.k) == 9 and message.k[0] > 0 and message.k[4] > 0:
            self.system.intrinsics = CameraIntrinsics(
                fx=float(message.k[0]),
                fy=float(message.k[4]),
                cx=float(message.k[2]),
                cy=float(message.k[5]),
            )

    def _worker_loop(self) -> None:
        while not self._stopping:
            self._event.wait(timeout=0.25)
            with self._lock:
                pair = self._pending
                self._pending = None
                self._event.clear()
            if pair is None:
                continue
            stamp, infra_message, depth_message = pair
            started=time.perf_counter()
            try:
                mono = self.bridge.imgmsg_to_cv2(infra_message, desired_encoding="mono8")
                color = cv2.cvtColor(mono, cv2.COLOR_GRAY2BGR)
                depth_raw = self.bridge.imgmsg_to_cv2(depth_message, desired_encoding="16UC1")
                if depth_raw.ndim == 3 and depth_raw.shape[2] == 1:
                    depth_raw = depth_raw[:, :, 0]
                if depth_raw.ndim != 2:
                    raise ValueError(f"16UC1 depth shape is invalid: {depth_raw.shape}")
                valid = depth_raw > 0
                if self.invalid_raw_min:
                    valid &= depth_raw < self.invalid_raw_min
                depth_m = np.full(depth_raw.shape, np.nan, dtype=np.float32)
                depth_m[valid] = depth_raw[valid].astype(np.float32) * self.depth_scale
                accepted, rejected = self.system.infer(color, depth_m)
                payload = make_payload(stamp, accepted, rejected, self.publish_rejected)
                message = String()
                message.data = json.dumps(payload, ensure_ascii=False, separators=(",", ":"))
                self.publisher.publish(message)
                self._processed+=1
                self._processing_ms.append((time.perf_counter()-started)*1000)
            except Exception as exc:  # keep the stream alive and expose the failing stamp
                self._failed+=1
                self.get_logger().error(f"stamp {stamp} 처리 실패: {exc}")
            now=time.monotonic()
            if now-self._last_report>=5 and self._processing_ms:
                values=np.asarray(self._processing_ms)
                self.get_logger().info(
                    f"CDA completed={self._processed}, output_fps={(self._processed-self._last_report_count)/max(now-self._last_report,1e-6):.2f}, "
                    f"processing median/p95={np.median(values):.2f}/{np.percentile(values,95):.2f} ms, "
                    f"pending_drops={self._dropped}/{self._paired}, errors={self._failed}")
                self._processing_ms.clear();self._last_report=now;self._last_report_count=self._processed

    def destroy_node(self) -> bool:
        self._stopping = True
        self._event.set()
        if hasattr(self, "_worker"):
            self._worker.join(timeout=2.0)
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = SCNetDepthNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
