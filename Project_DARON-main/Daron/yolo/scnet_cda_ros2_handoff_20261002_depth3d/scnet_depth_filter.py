"""SCNet + depth-profile crater filtering.

This module does not alter SCNet weights or the existing inference class.  It
wraps SCNetDetector and validates every returned bbox with depth_profile.py.

Expected inputs per frame:
  - color_frame: OpenCV BGR copy of the mono infra1 frame
  - aligned_depth_m: depth aligned to the infra1 image, in meters
  - optional camera intrinsics

Output:
  - accepted crater candidates, including bbox, pixel center, SCNet confidence,
    depth-profile diagnostics, and optional camera-frame XYZ.
  - rejected candidates for debugging / threshold tuning.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Iterable

import cv2
import numpy as np

from depth_profile import (
    CameraIntrinsics,
    DepthProfileConfig,
    DepthValidationResult,
    validate_depth_candidate,
)
from scnet_inference import Detection, SCNetDetector


@dataclass(frozen=True)
class CraterCandidate:
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_id: int
    center_x: float
    center_y: float
    depth: DepthValidationResult

    @property
    def bbox(self) -> tuple[float, float, float, float]:
        return self.x1, self.y1, self.x2, self.y2

    def to_dict(self) -> dict:
        d = asdict(self)
        d["depth"] = self.depth.to_dict()
        return d


class SCNetDepthCraterDetector:
    """Detect crater candidates with SCNet, then reject depth-inconsistent FP."""

    def __init__(
        self,
        weights: str | Path | None = None,
        *,
        device: str = "auto",
        confidence: float = 0.25,
        imgsz: int = 864,
        nms_iou: float = 0.6,
        max_det: int = 100,
        half: bool | None = None,
        cuda_graph: bool = True,
        depth_config: DepthProfileConfig | None = None,
        intrinsics: CameraIntrinsics | None = None,
    ) -> None:
        kwargs = dict(
            device=device,
            confidence=confidence,
            imgsz=imgsz,
            nms_iou=nms_iou,
            max_det=max_det,
            half=half,
            cuda_graph=cuda_graph,
        )
        if weights is not None:
            kwargs["weights"] = weights
        self.detector = SCNetDetector(**kwargs)
        self.depth_config = depth_config or DepthProfileConfig()
        self.intrinsics = intrinsics

    def validate_detections(
        self,
        depth_m: np.ndarray,
        detections: Iterable[Detection],
    ) -> tuple[list[CraterCandidate], list[CraterCandidate]]:
        if depth_m.ndim != 2:
            raise ValueError("aligned depth must be HxW")

        accepted: list[CraterCandidate] = []
        rejected: list[CraterCandidate] = []

        for det in detections:
            result = validate_depth_candidate(
                depth_m,
                (det.x1, det.y1, det.x2, det.y2),
                det.confidence,
                config=self.depth_config,
                intrinsics=self.intrinsics,
            )
            cx, cy = det.center
            candidate = CraterCandidate(
                det.x1,
                det.y1,
                det.x2,
                det.y2,
                det.confidence,
                det.class_id,
                cx,
                cy,
                result,
            )
            (accepted if result.is_crater else rejected).append(candidate)

        return accepted, rejected

    def infer(
        self,
        color_frame: np.ndarray,
        aligned_depth_m: np.ndarray,
    ) -> tuple[list[CraterCandidate], list[CraterCandidate]]:
        if color_frame.shape[:2] != aligned_depth_m.shape[:2]:
            raise ValueError(
                "infra1과 depth 크기가 다릅니다. infra1-aligned depth를 전달해야 합니다: "
                f"rgb={color_frame.shape[:2]}, depth={aligned_depth_m.shape[:2]}"
            )
        detections = self.detector.infer(color_frame)
        return self.validate_detections(aligned_depth_m, detections)


def draw_filtered(
    frame: np.ndarray,
    accepted: Iterable[CraterCandidate],
    rejected: Iterable[CraterCandidate] = (),
    *,
    show_rejected: bool = True,
) -> np.ndarray:
    """Accepted candidates are labeled CRATER; rejected candidates are optional."""
    canvas = frame.copy()

    if show_rejected:
        for item in rejected:
            p1 = (round(item.x1), round(item.y1))
            p2 = (round(item.x2), round(item.y2))
            cv2.rectangle(canvas, p1, p2, (120, 120, 120), 1)
            cv2.putText(
                canvas,
                f"reject {item.depth.reason}",
                (max(0, p1[0]), max(16, p1[1] - 4)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.38,
                (120, 120, 120),
                1,
                cv2.LINE_AA,
            )

    for item in accepted:
        p1 = (round(item.x1), round(item.y1))
        p2 = (round(item.x2), round(item.y2))
        cv2.rectangle(canvas, p1, p2, (0, 255, 0), 2)
        label = f"CRATER {item.confidence:.2f} ({item.center_x:.0f},{item.center_y:.0f})"
        if item.depth.representative_depth_m is not None:
            label += f" z={item.depth.representative_depth_m:.2f}m"
        cv2.putText(
            canvas,
            label,
            (max(0, p1[0]), max(16, p1[1] - 5)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.43,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

    return canvas


if __name__ == "__main__":
    raise SystemExit(
        "이 파일은 RGB와 동일 시각의 aligned depth frame을 함께 받아 사용하는 모듈입니다. "
        "README_DEPTH.md의 예제를 참고하세요."
    )
