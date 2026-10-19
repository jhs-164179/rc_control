"""Depth validation for SCNet crater candidates.

The implementation keeps the core idea of Esuga-Mopah et al. (2026):
YOLO/SCNet first proposes a bounding box and Depth is evaluated only inside
that candidate region.  The paper's 10-strip raw-depth test is retained as
``mode='paper'`` for reference.

For the rover's low, oblique camera, ``mode='ground_corrected'`` is the default
and is intentionally stricter and more geometric:

1. expand the detector bbox to obtain local surrounding terrain,
2. back-project surrounding valid depth pixels to 3-D with camera intrinsics,
3. robustly fit a local 3-D ground plane with deterministic RANSAC + SVD,
4. intersect each bbox pixel ray with that plane to obtain expected ground Z,
5. compute measured-Z minus expected-ground-Z (positive = depression),
6. evaluate coherent central depression and a 10-strip residual profile,
7. compute profiles beyond the reliable rejection range too, but reject only
   when Depth coverage and the configured rejection policy permit it.
   Missing/long-range/ambiguous Depth never vetoes a detection by default.

This ground-corrected method is an engineering adaptation for the rover use
case; it is not claimed to be the exact algorithm from the cited paper.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Iterable

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float

    def validate(self) -> None:
        if not all(np.isfinite([self.fx, self.fy, self.cx, self.cy])):
            raise ValueError("camera intrinsics must be finite")
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError("fx and fy must be > 0")


@dataclass(frozen=True)
class DepthProfileConfig:
    # Paper settings.
    n_strips: int = 10
    tau_depth_m: float = 0.015
    tau_conf: float = 0.65

    # Operational policy.  In recall-first mode, Depth may reject only when
    # near-range data and the local ground model are both trustworthy.
    recall_first: bool = True
    reliable_depth_max_m: float = 1.0
    allow_long_range_rejection: bool = False
    profile_long_range: bool = True

    # Supplied RealSense error approximation: ~3 mm @0.5 m, ~12 mm @1 m.
    # sigma_z ~= coefficient * z^2.  Geometry thresholds are never below
    # tau_depth_m and are enlarged to >= depth_sigma_multiplier * sigma_z.
    depth_error_coefficient: float = 0.012
    depth_sigma_multiplier: float = 3.0

    # Sensor-valid depth and coverage gates.
    min_depth_m: float = 0.07
    max_depth_m: float | None = None
    min_valid_ratio: float = 0.35
    min_center_valid_ratio: float = 0.30

    # Low-view local-ground adaptation.
    mode: str = "ground_corrected"  # "paper" or "ground_corrected"
    context_scale: float = 1.60
    ground_margin_fraction: float = 0.08
    min_ground_points: int = 120
    min_strip_points: int = 5

    # Robust 3-D plane fit.
    ransac_iterations: int = 160
    plane_hypothesis_points: int = 2048
    plane_finalists: int = 8
    plane_inlier_sigma_multiplier: float = 3.0
    plane_min_inlier_threshold_m: float = 0.006
    plane_min_inlier_ratio: float = 0.55
    plane_max_rmse_multiplier: float = 1.5
    random_seed: int = 42

    # Depression geometry.  Values are deliberately conservative because the
    # detector is the recall-oriented candidate generator.
    min_depression_ratio: float = 0.12
    min_coherent_depression_ratio: float = 0.08
    min_center_depression_ratio_of_tau: float = 0.60
    min_bowl_amplitude_ratio_of_tau: float = 0.45
    central_component_x_margin: float = 0.15
    central_component_y_margin: float = 0.12

    # Flatness gates used for confident rejection in recall-first mode.
    flat_max_depression_ratio: float = 0.05
    flat_max_coherent_ratio: float = 0.03
    flat_p90_ratio_of_tau: float = 0.80

    # Metric depth: a depression is farther from the camera than the local
    # ground intersection along the same pixel ray.
    depression_polarity: str = "positive"

    def validate(self) -> None:
        if not np.isfinite(self.min_depth_m) or self.min_depth_m <= 0:
            raise ValueError('min_depth_m must be finite and positive')
        if self.max_depth_m is not None and (not np.isfinite(self.max_depth_m) or self.max_depth_m <= self.min_depth_m):
            raise ValueError('max_depth_m must exceed min_depth_m, or be None')
        if not np.isfinite(self.reliable_depth_max_m) or self.reliable_depth_max_m < self.min_depth_m:
            raise ValueError('reliable_depth_max_m must be finite and >= min_depth_m')
        if not np.isfinite(self.depth_error_coefficient) or self.depth_error_coefficient < 0:
            raise ValueError('depth_error_coefficient must be finite and nonnegative')
        if self.mode not in {"paper", "ground_corrected"}:
            raise ValueError("mode must be 'paper' or 'ground_corrected'")
        if self.n_strips < 2:
            raise ValueError("n_strips must be >= 2")
        if self.tau_depth_m <= 0:
            raise ValueError("tau_depth_m must be > 0")
        if not 0 <= self.tau_conf <= 1:
            raise ValueError("tau_conf must be in [0, 1]")
        if not 0 < self.min_valid_ratio <= 1:
            raise ValueError("min_valid_ratio must be in (0, 1]")
        if not 0 <= self.min_center_valid_ratio <= 1:
            raise ValueError("min_center_valid_ratio must be in [0, 1]")
        if self.context_scale <= 1.0:
            raise ValueError("context_scale must be > 1.0")
        if not 0 < self.plane_min_inlier_ratio <= 1:
            raise ValueError("plane_min_inlier_ratio must be in (0,1]")
        if self.depression_polarity not in {"positive", "negative"}:
            raise ValueError("depression_polarity must be 'positive' or 'negative'")


@dataclass(frozen=True)
class DepthValidationResult:
    is_crater: bool
    reason: str
    bbox: tuple[int, int, int, int]
    confidence: float
    valid_ratio: float
    strip_means_m: tuple[float | None, ...]
    adjacent_deltas_m: tuple[float | None, ...]
    max_positive_delta_m: float
    max_abs_delta_m: float
    depression_ratio: float | None = None
    center_depression_m: float | None = None
    representative_depth_m: float | None = None
    center_camera_xyz_m: tuple[float, float, float] | None = None
    effective_tau_depth_m: float | None = None
    reliable_valid_ratio: float | None = None
    filter_applied: bool = False
    decision_state: str = "unknown"
    ground_plane_inlier_ratio: float | None = None
    ground_plane_rmse_m: float | None = None
    coherent_depression_ratio: float | None = None
    bowl_amplitude_m: float | None = None
    center_valid_ratio: float | None = None
    profile_computed: bool = False
    rejection_allowed: bool = False
    reliable_range_max_m: float = 1.0

    def to_dict(self) -> dict:
        return asdict(self)


def _clip_bbox(bbox: Iterable[float], shape: tuple[int, int]) -> tuple[int, int, int, int]:
    h, w = shape
    x1, y1, x2, y2 = [int(round(v)) for v in bbox]
    x1 = int(np.clip(x1, 0, max(w - 1, 0)))
    x2 = int(np.clip(x2, 0, max(w - 1, 0)))
    y1 = int(np.clip(y1, 0, max(h - 1, 0)))
    y2 = int(np.clip(y2, 0, max(h - 1, 0)))
    if x2 <= x1 or y2 <= y1:
        raise ValueError(f"invalid bbox after clipping: {(x1, y1, x2, y2)}")
    return x1, y1, x2, y2


def _expand_bbox(bbox: tuple[int, int, int, int], shape: tuple[int, int], scale: float) -> tuple[int, int, int, int]:
    h, w = shape
    x1, y1, x2, y2 = bbox
    cx = 0.5 * (x1 + x2)
    cy = 0.5 * (y1 + y2)
    bw = (x2 - x1) * scale
    bh = (y2 - y1) * scale
    return (
        int(max(0, np.floor(cx - bw / 2))),
        int(max(0, np.floor(cy - bh / 2))),
        int(min(w - 1, np.ceil(cx + bw / 2))),
        int(min(h - 1, np.ceil(cy + bh / 2))),
    )


def _valid_depth(depth: np.ndarray, cfg: DepthProfileConfig) -> np.ndarray:
    valid = np.isfinite(depth) & (depth >= cfg.min_depth_m)
    if cfg.max_depth_m is not None:
        valid &= depth <= cfg.max_depth_m
    return valid


def _sensor_sigma(z_m: float | None, cfg: DepthProfileConfig) -> float:
    if z_m is None or not np.isfinite(z_m) or cfg.depth_error_coefficient <= 0:
        return 0.0
    return float(cfg.depth_error_coefficient * z_m * z_m)


def _effective_tau(z_m: float | None, cfg: DepthProfileConfig) -> float:
    return max(cfg.tau_depth_m, cfg.depth_sigma_multiplier * _sensor_sigma(z_m, cfg))


def _pixels_to_points(u: np.ndarray, v: np.ndarray, z: np.ndarray, intr: CameraIntrinsics) -> np.ndarray:
    x = (u - intr.cx) * z / intr.fx
    y = (v - intr.cy) * z / intr.fy
    return np.column_stack((x, y, z))


def _plane_from_three(points: np.ndarray) -> tuple[np.ndarray, float] | None:
    a, b, c = points
    n = np.cross(b - a, c - a)
    norm = float(np.linalg.norm(n))
    if not np.isfinite(norm) or norm < 1e-9:
        return None
    n = n / norm
    d = -float(np.dot(n, a))
    return n, d


def _refine_plane_svd(points: np.ndarray) -> tuple[np.ndarray, float] | None:
    if len(points) < 3:
        return None
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, vectors = np.linalg.eigh(centered.T @ centered)
    n = vectors[:, 0]
    norm = float(np.linalg.norm(n))
    if not np.isfinite(norm) or norm < 1e-9:
        return None
    n = n / norm
    d = -float(np.dot(n, centroid))
    # Plane sign does not affect ray intersection, but a stable orientation
    # makes diagnostics deterministic.
    if n[2] > 0:
        n = -n
        d = -d
    return n, d


def _fit_ground_plane_ransac(
    points: np.ndarray,
    rep_depth_m: float | None,
    cfg: DepthProfileConfig,
) -> tuple[np.ndarray, float, np.ndarray, float, float] | None:
    """Return normal, d, inlier mask, inlier ratio, RMSE."""
    n_points = len(points)
    if n_points < max(cfg.min_ground_points, 3):
        return None

    threshold = max(
        cfg.plane_min_inlier_threshold_m,
        cfg.plane_inlier_sigma_multiplier * _sensor_sigma(rep_depth_m, cfg),
    )
    rng = np.random.default_rng(cfg.random_seed)
    best_mask = None
    best_count = -1
    best_median = np.inf

    # Same random hypotheses; pre-score on a deterministic spatially spread
    # subset. Fully evaluate the eight finalists, then refit and check quality
    # on ALL ground points. This bounds the large-box hypothesis cost.
    indices=np.array([rng.choice(n_points,size=3,replace=False) for _ in range(cfg.ransac_iterations)])
    a,b,c=(points[indices[:,j]] for j in range(3))
    normal_batch=np.cross(b-a,c-a)
    lengths=np.linalg.norm(normal_batch,axis=1)
    valid=np.isfinite(lengths)&(lengths>=1e-9)
    normal_batch=normal_batch[valid]/lengths[valid,None]
    offset_batch=-np.sum(normal_batch*a[valid],axis=1)
    planes=list(zip(normal_batch,offset_batch))
    if not planes:
        return None
    sample=points[np.linspace(0,n_points-1,min(n_points,cfg.plane_hypothesis_points),dtype=int)]
    normals=np.stack([p[0] for p in planes],axis=1)
    offsets=np.array([p[1] for p in planes])
    support=(np.abs(sample@normals+offsets)<=threshold).sum(axis=0)
    finalists=np.argsort(-support,kind='stable')[:min(cfg.plane_finalists,len(planes))]
    for start in range(0, len(finalists), 4):
        batch = [planes[j] for j in finalists[start:start+4]]
        normals = np.stack([p[0] for p in batch], axis=1)
        offsets = np.array([p[1] for p in batch])
        residuals = np.abs(points @ normals + offsets)
        masks = residuals <= threshold
        counts = masks.sum(axis=0)
        highest = int(counts.max())
        if highest < max(3, best_count):
            continue
        for j in np.flatnonzero(counts == highest):
            med = float(np.median(residuals[masks[:, j], j]))
            if highest > best_count or (highest == best_count and med < best_median):
                best_mask = masks[:, j].copy()
                best_count, best_median = highest, med

    if best_mask is None or best_count < 3:
        return None

    refined = _refine_plane_svd(points[best_mask])
    if refined is None:
        return None
    n, d = refined
    residual = np.abs(points @ n + d)
    inliers = residual <= threshold
    if int(inliers.sum()) >= 3:
        refined = _refine_plane_svd(points[inliers])
        if refined is not None:
            n, d = refined
            residual = np.abs(points @ n + d)
            inliers = residual <= threshold

    ratio = float(inliers.mean())
    rmse = float(np.sqrt(np.mean(np.square(residual[inliers])))) if inliers.any() else np.inf
    return n, d, inliers, ratio, rmse


def _plane_expected_z(
    u: np.ndarray,
    v: np.ndarray,
    intr: CameraIntrinsics,
    normal: np.ndarray,
    d: float,
) -> np.ndarray:
    rx = (u - intr.cx) / intr.fx
    ry = (v - intr.cy) / intr.fy
    denom = normal[0] * rx + normal[1] * ry + normal[2]
    z = np.full(np.asarray(u).shape, np.nan, dtype=np.float64)
    ok = np.abs(denom) > 1e-9
    z[ok] = -d / denom[ok]
    z[(z <= 0) | (~np.isfinite(z))] = np.nan
    return z


def _strip_profile(
    values: np.ndarray,
    valid: np.ndarray,
    n_strips: int,
    min_strip_points: int,
    *,
    robust: bool,
) -> tuple[tuple[float | None, ...], tuple[float | None, ...]]:
    h, _ = values.shape
    means: list[float | None] = []
    for i in range(n_strips):
        y0 = int(np.floor(i * h / n_strips))
        y1 = h if i == n_strips - 1 else int(np.floor((i + 1) * h / n_strips))
        vals = values[y0:y1][valid[y0:y1]]
        if vals.size < min_strip_points:
            means.append(None)
        else:
            means.append(float(np.median(vals) if robust else np.mean(vals)))
    deltas = tuple(None if a is None or b is None else float(b - a) for a, b in zip(means[:-1], means[1:]))
    return tuple(means), deltas


def _bowl_amplitude(strip_values: tuple[float | None, ...]) -> float:
    finite = np.array([np.nan if v is None else v for v in strip_values], dtype=np.float64)
    if np.isfinite(finite).sum() < 4:
        return 0.0
    n = len(finite)
    edge_idx = list(range(max(1, n // 5))) + list(range(n - max(1, n // 5), n))
    inner_idx = list(range(max(1, n // 4), min(n, n - max(1, n // 4))))
    edge = finite[edge_idx]
    inner = finite[inner_idx]
    edge = edge[np.isfinite(edge)]
    inner = inner[np.isfinite(inner)]
    if edge.size == 0 or inner.size == 0:
        return 0.0
    return float(np.median(inner) - np.median(edge))


def _coherent_component_ratio(mask: np.ndarray, valid: np.ndarray, cfg: DepthProfileConfig) -> tuple[float, bool]:
    binary = (mask.astype(np.uint8) * 255)
    count, labels, stats, centroids = cv2.connectedComponentsWithStats(binary, connectivity=8)
    valid_count = max(int(valid.sum()), 1)
    if count <= 1:
        return 0.0, False

    h, w = mask.shape
    best_ratio = 0.0
    best_central = False
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area <= 0:
            continue
        ratio = area / valid_count
        cx, cy = centroids[label]
        central = (
            cfg.central_component_x_margin * w <= cx <= (1.0 - cfg.central_component_x_margin) * w
            and cfg.central_component_y_margin * h <= cy <= (1.0 - cfg.central_component_y_margin) * h
        )
        left = int(stats[label, cv2.CC_STAT_LEFT])
        top = int(stats[label, cv2.CC_STAT_TOP])
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        height = int(stats[label, cv2.CC_STAT_HEIGHT])
        touches_all_the_way = left == 0 and width >= w or top == 0 and height >= h
        central = bool(central and not touches_all_the_way)
        if ratio > best_ratio:
            best_ratio = float(ratio)
            best_central = central
    return best_ratio, best_central


def _representative_depth_and_xyz(
    roi: np.ndarray,
    valid: np.ndarray,
    bbox: tuple[int, int, int, int],
    intrinsics: CameraIntrinsics | None,
) -> tuple[float | None, tuple[float, float, float] | None]:
    vals = roi[valid]
    if vals.size == 0:
        return None, None
    z = float(np.median(vals))
    if intrinsics is None:
        return z, None
    x1, y1, x2, y2 = bbox
    u = 0.5 * (x1 + x2)
    v = 0.5 * (y1 + y2)
    return z, (float((u - intrinsics.cx) * z / intrinsics.fx), float((v - intrinsics.cy) * z / intrinsics.fy), z)


def _fallback_result(
    cfg: DepthProfileConfig,
    box: tuple[int, int, int, int],
    confidence: float,
    valid_ratio: float,
    reliable_valid_ratio: float,
    rep_depth: float | None,
    rep_xyz: tuple[float, float, float] | None,
    reason: str,
    *,
    strict_value: bool = False,
) -> DepthValidationResult:
    keep = True if cfg.recall_first else strict_value
    return DepthValidationResult(
        is_crater=keep,
        reason=reason,
        bbox=box,
        confidence=float(confidence),
        valid_ratio=valid_ratio,
        strip_means_m=tuple([None] * cfg.n_strips),
        adjacent_deltas_m=tuple([None] * (cfg.n_strips - 1)),
        max_positive_delta_m=0.0,
        max_abs_delta_m=0.0,
        representative_depth_m=rep_depth,
        center_camera_xyz_m=rep_xyz,
        reliable_valid_ratio=reliable_valid_ratio,
        filter_applied=False,
        decision_state="advisory_pass" if keep else "not_evaluated",
        reliable_range_max_m=cfg.reliable_depth_max_m,
    )


def validate_depth_candidate(
    depth_m: np.ndarray,
    bbox: Iterable[float],
    confidence: float,
    *,
    config: DepthProfileConfig | None = None,
    intrinsics: CameraIntrinsics | None = None,
) -> DepthValidationResult:
    cfg = config or DepthProfileConfig()
    cfg.validate()

    if depth_m.ndim != 2:
        raise ValueError("depth_m must be a 2-D aligned depth image")
    depth_m = depth_m.astype(np.float32, copy=False)
    box = _clip_bbox(bbox, depth_m.shape)
    x1, y1, x2, y2 = box
    roi = depth_m[y1:y2 + 1, x1:x2 + 1]
    valid = _valid_depth(roi, cfg)
    valid_ratio = float(valid.mean()) if valid.size else 0.0
    rep_depth, rep_xyz = _representative_depth_and_xyz(roi, valid, box, intrinsics)
    reliable_valid = valid & (roi <= cfg.reliable_depth_max_m)
    reliable_valid_ratio = float(reliable_valid.mean()) if reliable_valid.size else 0.0
    profile_valid = valid if cfg.profile_long_range or cfg.allow_long_range_rejection else reliable_valid
    rejection_allowed = bool(cfg.allow_long_range_rejection or (
        rep_depth is not None and rep_depth <= cfg.reliable_depth_max_m
        and reliable_valid_ratio >= cfg.min_valid_ratio))

    if valid_ratio <= 0:
        return _fallback_result(cfg, box, confidence, valid_ratio, reliable_valid_ratio, rep_depth, rep_xyz,
                                "depth_unavailable_pass_recall_first" if cfg.recall_first else "insufficient_valid_depth")

    if rep_depth is not None and rep_depth > cfg.reliable_depth_max_m and not (cfg.profile_long_range or cfg.allow_long_range_rejection):
        return _fallback_result(cfg, box, confidence, valid_ratio, reliable_valid_ratio, rep_depth, rep_xyz,
                                "long_range_depth_advisory_pass_recall_first" if cfg.recall_first else "long_range_depth_not_profiled")

    if float(profile_valid.mean()) < cfg.min_valid_ratio:
        return _fallback_result(cfg, box, confidence, valid_ratio, reliable_valid_ratio, rep_depth, rep_xyz,
                                "near_depth_insufficient_pass_recall_first" if cfg.recall_first else "insufficient_reliable_depth")

    # Paper reference mode: raw-depth strip mean and depth-jump OR confidence.
    if cfg.mode == "paper":
        means, deltas = _strip_profile(roi, profile_valid, cfg.n_strips, cfg.min_strip_points, robust=False)
        finite = [d for d in deltas if d is not None]
        max_pos = max(finite, default=0.0)
        max_abs = max((abs(d) for d in finite), default=0.0)
        is_crater = bool(max_pos > cfg.tau_depth_m or confidence > cfg.tau_conf)
        return DepthValidationResult(
            is_crater=is_crater or (cfg.recall_first and not rejection_allowed),
            reason=("long_range_paper_advisory" if not rejection_allowed else
                    "paper_depth_or_conf_pass" if is_crater else "paper_depth_and_conf_fail"),
            bbox=box,
            confidence=float(confidence),
            valid_ratio=valid_ratio,
            strip_means_m=means,
            adjacent_deltas_m=deltas,
            max_positive_delta_m=float(max_pos),
            max_abs_delta_m=float(max_abs),
            representative_depth_m=rep_depth,
            center_camera_xyz_m=rep_xyz,
            effective_tau_depth_m=cfg.tau_depth_m,
            reliable_valid_ratio=reliable_valid_ratio,
            filter_applied=rejection_allowed,
            decision_state=("advisory_pass" if not rejection_allowed and cfg.recall_first else
                            "paper_pass" if is_crater else "paper_reject"),
            profile_computed=True, rejection_allowed=rejection_allowed,
            reliable_range_max_m=cfg.reliable_depth_max_m,
        )

    # 3-D ground correction requires intrinsics; never silently fall back to an
    # inaccurate 2-D pixel/depth plane model.
    if intrinsics is None:
        return _fallback_result(cfg, box, confidence, valid_ratio, reliable_valid_ratio, rep_depth, rep_xyz,
                                "intrinsics_unavailable_pass_recall_first" if cfg.recall_first else "intrinsics_required")
    intrinsics.validate()

    ex1, ey1, ex2, ey2 = _expand_bbox(box, depth_m.shape, cfg.context_scale)
    context = depth_m[ey1:ey2 + 1, ex1:ex2 + 1]
    context_valid = _valid_depth(context, cfg)
    if not (cfg.profile_long_range or cfg.allow_long_range_rejection):
        context_valid &= context <= cfg.reliable_depth_max_m
    yy, xx = np.mgrid[ey1:ey2 + 1, ex1:ex2 + 1]

    mx = max(1, int(round((x2 - x1) * cfg.ground_margin_fraction)))
    my = max(1, int(round((y2 - y1) * cfg.ground_margin_fraction)))
    exclusion = (xx >= x1 - mx) & (xx <= x2 + mx) & (yy >= y1 - my) & (yy <= y2 + my)
    ground_mask = context_valid & (~exclusion)
    if int(ground_mask.sum()) < cfg.min_ground_points:
        return _fallback_result(cfg, box, confidence, valid_ratio, reliable_valid_ratio, rep_depth, rep_xyz,
                                "ground_context_insufficient_pass_recall_first" if cfg.recall_first else "insufficient_ground_context")

    gz = context[ground_mask].astype(np.float64)
    ground_points = _pixels_to_points(
        xx[ground_mask].astype(np.float64), yy[ground_mask].astype(np.float64), gz, intrinsics
    )
    ground_rep = float(np.median(gz)) if gz.size else rep_depth
    fit = _fit_ground_plane_ransac(ground_points, ground_rep, cfg)
    if fit is None:
        return _fallback_result(cfg, box, confidence, valid_ratio, reliable_valid_ratio, rep_depth, rep_xyz,
                                "ground_plane_fit_failed_pass_recall_first" if cfg.recall_first else "ground_plane_fit_failed")
    normal, plane_d, _inliers, inlier_ratio, plane_rmse = fit
    plane_threshold = max(
        cfg.plane_min_inlier_threshold_m,
        cfg.plane_inlier_sigma_multiplier * _sensor_sigma(ground_rep, cfg),
    )
    plane_good = (
        inlier_ratio >= cfg.plane_min_inlier_ratio
        and np.isfinite(plane_rmse)
        and plane_rmse <= cfg.plane_max_rmse_multiplier * plane_threshold
    )
    if not plane_good:
        result = _fallback_result(cfg, box, confidence, valid_ratio, reliable_valid_ratio, rep_depth, rep_xyz,
                                  "ground_plane_unreliable_pass_recall_first" if cfg.recall_first else "ground_plane_unreliable")
        return DepthValidationResult(**{**result.to_dict(), "ground_plane_inlier_ratio": inlier_ratio, "ground_plane_rmse_m": plane_rmse})

    ry, rx = np.mgrid[y1:y2 + 1, x1:x2 + 1]
    expected_z = _plane_expected_z(rx.astype(np.float64), ry.astype(np.float64), intrinsics, normal, plane_d)
    evaluable = profile_valid & np.isfinite(expected_z)
    if float(evaluable.mean()) < cfg.min_valid_ratio:
        return _fallback_result(cfg, box, confidence, valid_ratio, reliable_valid_ratio, rep_depth, rep_xyz,
                                "plane_projection_insufficient_pass_recall_first" if cfg.recall_first else "plane_projection_insufficient")

    residual = roi.astype(np.float64) - expected_z
    signed = residual if cfg.depression_polarity == "positive" else -residual
    effective_tau = _effective_tau(rep_depth, cfg)

    h, w = roi.shape
    cy0, cy1 = int(0.25 * h), max(int(0.75 * h), int(0.25 * h) + 1)
    cx0, cx1 = int(0.25 * w), max(int(0.75 * w), int(0.25 * w) + 1)
    center_valid = evaluable[cy0:cy1, cx0:cx1]
    center_valid_ratio = float(center_valid.mean()) if center_valid.size else 0.0
    if center_valid_ratio < cfg.min_center_valid_ratio:
        result = _fallback_result(cfg, box, confidence, valid_ratio, reliable_valid_ratio, rep_depth, rep_xyz,
                                  "center_depth_insufficient_pass_recall_first" if cfg.recall_first else "center_depth_insufficient")
        return DepthValidationResult(**{**result.to_dict(), "ground_plane_inlier_ratio": inlier_ratio,
                                        "ground_plane_rmse_m": plane_rmse, "center_valid_ratio": center_valid_ratio})

    depressed = evaluable & (signed > effective_tau)
    depression_ratio = float(depressed.sum() / max(int(evaluable.sum()), 1))
    center_values = signed[cy0:cy1, cx0:cx1][center_valid]
    center_dep = float(np.median(center_values)) if center_values.size else 0.0
    p90 = float(np.percentile(signed[evaluable], 90)) if evaluable.any() else 0.0

    coherent_ratio, coherent_central = _coherent_component_ratio(depressed, evaluable, cfg)
    means, deltas = _strip_profile(signed, evaluable, cfg.n_strips, cfg.min_strip_points, robust=True)
    finite = [d for d in deltas if d is not None]
    max_pos = max(finite, default=0.0)
    max_abs = max((abs(d) for d in finite), default=0.0)
    bowl_amp = _bowl_amplitude(means)

    center_gate = center_dep >= effective_tau * cfg.min_center_depression_ratio_of_tau
    coherent_gate = coherent_central and coherent_ratio >= cfg.min_coherent_depression_ratio
    ratio_gate = depression_ratio >= cfg.min_depression_ratio
    bowl_gate = bowl_amp >= effective_tau * cfg.min_bowl_amplitude_ratio_of_tau

    # A real depression should be spatially coherent and either centrally deep
    # or bowl-like in the vertical profile.  Detector confidence never bypasses
    # this geometry test once trustworthy near-range Depth is available.
    geometry_confirmed = bool(coherent_gate and ratio_gate and (center_gate or bowl_gate))

    flat_confident = bool(
        depression_ratio <= cfg.flat_max_depression_ratio
        and coherent_ratio <= cfg.flat_max_coherent_ratio
        and p90 <= effective_tau * cfg.flat_p90_ratio_of_tau
        and center_dep <= effective_tau * cfg.flat_p90_ratio_of_tau
    )

    if geometry_confirmed:
        is_crater = True
        reason = "ground_corrected_3d_depression_pass"
        state = "confirmed_crater"
    elif not rejection_allowed:
        is_crater = bool(cfg.recall_first)
        reason = "long_range_profile_advisory_pass" if is_crater else "long_range_geometry_not_confirmed"
        state = "advisory_pass" if is_crater else "not_confirmed"
    elif flat_confident:
        is_crater = False
        reason = "ground_corrected_flat_reject"
        state = "rejected_flat"
    elif cfg.recall_first:
        is_crater = True
        reason = "ground_corrected_ambiguous_pass_recall_first"
        state = "indeterminate_pass"
    else:
        is_crater = False
        reason = "ground_corrected_geometry_not_confirmed"
        state = "rejected_unconfirmed"

    return DepthValidationResult(
        is_crater=is_crater,
        reason=reason,
        bbox=box,
        confidence=float(confidence),
        valid_ratio=valid_ratio,
        strip_means_m=means,
        adjacent_deltas_m=deltas,
        max_positive_delta_m=float(max_pos),
        max_abs_delta_m=float(max_abs),
        depression_ratio=depression_ratio,
        center_depression_m=center_dep,
        representative_depth_m=rep_depth,
        center_camera_xyz_m=rep_xyz,
        effective_tau_depth_m=effective_tau,
        reliable_valid_ratio=reliable_valid_ratio,
        filter_applied=rejection_allowed,
        decision_state=state,
        ground_plane_inlier_ratio=inlier_ratio,
        ground_plane_rmse_m=plane_rmse,
        coherent_depression_ratio=coherent_ratio,
        bowl_amplitude_m=bowl_amp,
        center_valid_ratio=center_valid_ratio,
        profile_computed=True, rejection_allowed=rejection_allowed,
        reliable_range_max_m=cfg.reliable_depth_max_m,
    )
