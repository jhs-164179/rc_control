"""Shared `/cda/detections` JSON contract."""
from __future__ import annotations

from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from scnet_depth_filter import CraterCandidate


def _round_or_none(value, digits=6):
    return None if value is None else round(float(value), digits)


def candidate_to_dict(item: CraterCandidate, accepted: bool) -> dict:
    depth = item.depth
    record = {
        "box": [round(item.x1, 2), round(item.y1, 2), round(item.x2, 2), round(item.y2, 2)],
        "score": round(item.confidence, 6),
        "class": "crater",
        "depth_profile_pass": accepted,
        "depth_reason": depth.reason,
        "depth_decision_state": depth.decision_state,
        "depth_m": _round_or_none(depth.representative_depth_m),
        "depth_valid_ratio": round(depth.valid_ratio, 6),
        "depth_reliable_valid_ratio": _round_or_none(depth.reliable_valid_ratio),
        "depth_filter_applied": bool(depth.filter_applied),
        "depth_profile_computed": bool(depth.profile_computed),
        "depth_rejection_allowed": bool(depth.rejection_allowed),
        "depth_reliable_range_max_m": float(depth.reliable_range_max_m),
        "depth_recommended_range": bool(
            depth.representative_depth_m is not None
            and depth.representative_depth_m <= depth.reliable_range_max_m
        ),
        "depth_high_accuracy_range": bool(
            depth.representative_depth_m is not None
            and 0.07 <= depth.representative_depth_m <= 0.5
        ),
        "depression_ratio": _round_or_none(depth.depression_ratio),
        "coherent_depression_ratio": _round_or_none(depth.coherent_depression_ratio),
        "center_depression_m": _round_or_none(depth.center_depression_m),
        "bowl_amplitude_m": _round_or_none(depth.bowl_amplitude_m),
        "center_valid_ratio": _round_or_none(depth.center_valid_ratio),
        "max_profile_delta_m": round(depth.max_positive_delta_m, 6),
        "effective_depth_threshold_m": _round_or_none(depth.effective_tau_depth_m),
        "ground_plane_inlier_ratio": _round_or_none(depth.ground_plane_inlier_ratio),
        "ground_plane_rmse_m": _round_or_none(depth.ground_plane_rmse_m),
    }
    if depth.center_camera_xyz_m is not None:
        record["camera_xyz_m"] = [round(value, 6) for value in depth.center_camera_xyz_m]
    return record


def make_payload(stamp_ns: int, accepted, rejected=(), publish_rejected: bool = False) -> dict:
    detections = [candidate_to_dict(item, True) for item in accepted]
    if publish_rejected:
        detections.extend(candidate_to_dict(item, False) for item in rejected)
    return {"stamp_ns": int(stamp_ns), "camera": "infra1", "source": "yolo", "dets": detections}
