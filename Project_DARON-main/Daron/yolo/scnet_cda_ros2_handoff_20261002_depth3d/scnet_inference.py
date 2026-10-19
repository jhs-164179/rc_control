"""SCNet inference entry point for downstream depth profiling.

Python usage:
    detector = SCNetDetector()
    detections = detector.infer(bgr_frame)

Each detection is expressed in the original input image coordinate system.
"""
from __future__ import annotations

import argparse
import json
import os
from dataclasses import asdict, dataclass
from pathlib import Path

import cv2
import numpy as np


ROOT = Path(__file__).resolve().parent
os.environ.setdefault("YOLO_CONFIG_DIR", str(ROOT / ".config/ultralytics"))
os.environ.setdefault("YOLO_OFFLINE", "true")
os.environ.setdefault("YOLO_AUTOINSTALL", "false")

# This import must happen before loading the checkpoint because best.pt contains
# the custom SCNetModel class reference.
import scnet_model  # noqa: E402,F401
import torch  # noqa: E402
from ultralytics import YOLO  # noqa: E402


@dataclass(frozen=True)
class Detection:
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_id: int

    @property
    def center(self) -> tuple[float, float]:
        return ((self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2)


class SCNetDetector:
    """Loads SCNet once and performs frame-by-frame inference."""

    def __init__(
        self,
        weights: str | Path = ROOT / "weights/scnet_best.pt",
        device: str = "auto",
        confidence: float = 0.25,
        imgsz: int = 864,
        nms_iou: float = 0.6,
        max_det: int = 100,
        half: bool | None = None,
        cuda_graph: bool = True,
    ) -> None:
        self.device = "0" if device == "auto" and torch.cuda.is_available() else ("cpu" if device == "auto" else device)
        if self.device != "cpu" and not torch.cuda.is_available():
            raise RuntimeError("CUDA 장치를 요청했지만 torch.cuda.is_available()이 False입니다.")
        self.confidence = confidence
        self.imgsz = imgsz
        self.nms_iou = nms_iou
        self.max_det = max_det
        # CUDA FP16 is optional; CPU inference always uses FP32.
        self.half = self.device != "cpu" if half is None else bool(half) and self.device != "cpu"
        self.model = YOLO(str(weights))
        self.use_cuda_graph=bool(cuda_graph) and self.device!='cpu'
        self.graph_forward=None

    def infer(self, bgr_frame: np.ndarray) -> list[Detection]:
        """Return detections in the original BGR frame's pixel coordinates."""
        result = self.model.predict(
            source=bgr_frame,
            device=self.device,
            conf=self.confidence,
            iou=self.nms_iou,
            imgsz=self.imgsz,
            max_det=self.max_det,
            rect=True,
            half=self.half,
            batch=1,
            agnostic_nms=False,
            verbose=False,
            save=False,
        )[0]
        if self.use_cuda_graph and self.graph_forward is None and self.model.predictor.model.pt:
            from cuda_graph import GraphForward
            backend=self.model.predictor.model
            self.graph_forward=GraphForward(backend.forward)
            backend.forward=self.graph_forward
        if result.boxes is None or len(result.boxes) == 0:
            return []
        rows = result.boxes.data.detach().cpu().numpy()
        return [Detection(float(x1), float(y1), float(x2), float(y2), float(conf), int(cls)) for x1, y1, x2, y2, conf, cls in rows]


def draw(frame: np.ndarray, detections: list[Detection]) -> np.ndarray:
    canvas = frame.copy()
    for det in detections:
        cv2.rectangle(canvas, (round(det.x1), round(det.y1)), (round(det.x2), round(det.y2)), (0, 200, 255), 2)
        cv2.putText(
            canvas,
            f"crater {det.confidence:.3f}",
            (max(0, round(det.x1)), max(16, round(det.y1) - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (0, 200, 255),
            1,
            cv2.LINE_AA,
        )
    return canvas


def run_video(args: argparse.Namespace) -> None:
    detector = SCNetDetector(
        args.weights,
        device=args.device,
        confidence=args.conf,
        imgsz=args.imgsz,
        nms_iou=args.nms_iou,
        max_det=args.max_det,
        half=not args.fp32,
        cuda_graph=not args.no_cuda_graph,
    )
    capture = cv2.VideoCapture(str(args.source))
    if not capture.isOpened():
        raise RuntimeError(f"영상을 열 수 없습니다: {args.source}")
    fps = float(capture.get(cv2.CAP_PROP_FPS)) or 30.0
    width = int(capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(capture.get(cv2.CAP_PROP_FRAME_HEIGHT))
    writer = None
    if args.output_video:
        Path(args.output_video).parent.mkdir(parents=True, exist_ok=True)
        writer = cv2.VideoWriter(str(args.output_video), cv2.VideoWriter_fourcc(*"mp4v"), fps, (width, height))
        if not writer.isOpened():
            raise RuntimeError(f"결과 영상을 만들 수 없습니다: {args.output_video}")
    Path(args.output_jsonl).parent.mkdir(parents=True, exist_ok=True)
    frame_index = 0
    with Path(args.output_jsonl).open("w", encoding="utf-8") as output:
        while True:
            ok, frame = capture.read()
            if not ok:
                break
            detections = detector.infer(frame)
            record = {
                "frame": frame_index,
                "timestamp_sec": frame_index / fps,
                "image_width": width,
                "image_height": height,
                "detections": [asdict(item) | {"center_x": item.center[0], "center_y": item.center[1]} for item in detections],
            }
            output.write(json.dumps(record, ensure_ascii=False) + "\n")
            if writer is not None:
                writer.write(draw(frame, detections))
            frame_index += 1
            if args.limit and frame_index >= args.limit:
                break
            if frame_index % 500 == 0:
                print(f"processed {frame_index} frames", flush=True)
    capture.release()
    if writer is not None:
        writer.release()
    print(f"완료: {frame_index} frames -> {args.output_jsonl}")


def main() -> None:
    parser = argparse.ArgumentParser(description="YOLO-SCNet 영상 추론 및 Depth profiling 연동용 JSONL 출력")
    parser.add_argument("--source", type=Path, default=ROOT / "data/video4_infra1.mp4")
    parser.add_argument("--weights", type=Path, default=ROOT / "weights/scnet_best.pt")
    parser.add_argument("--output-jsonl", type=Path, default=ROOT / "results/scnet_detections.jsonl")
    parser.add_argument("--output-video", type=Path, default=None)
    parser.add_argument("--device", default="auto", help="auto, cpu 또는 CUDA 번호")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--imgsz", type=int, default=864)
    parser.add_argument("--nms-iou", type=float, default=0.4)
    parser.add_argument("--max-det", type=int, default=100)
    parser.add_argument("--limit", type=int, default=0)
    parser.add_argument("--fp32",action="store_true",help="Disable CUDA FP16")
    parser.add_argument("--no-cuda-graph",action="store_true",help="Use eager PyTorch inference")
    run_video(parser.parse_args())


if __name__ == "__main__":
    main()
