"""Metric circular-rim reconstruction from four bounding-box tangent constraints.

All pixels here are UNDISTORTED and depth is optical-axis Z in metres in the
same camera frame. A surrounding-ground plane is explicitly an approximation
to the rim plane, not evidence that the physical rim is planar.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np
from .types import Box, Camera, Circle, Detection, Plane, InvalidMeasurement


@dataclass
class GeometryConfig:
    min_depth_m: float = .07
    # None: do not impose a scene-specific upper range. Sensor-invalid values
    # must be removed by the raw decoder / in-memory caller.
    max_depth_m: float | None = None
    min_box_height_px: float = 2.
    ring_padding_px: int = 16
    ring_scale: float = .3
    # Bound the reference terrain in 3-D relative to the candidate's apparent
    # diameter, rather than clipping all terrain at an absolute camera range.
    ground_local_radius_diameters: float = 2.
    max_points: int = 1800
    min_points: int = 60
    ransac_iterations: int = 100
    plane_threshold_m: float = .008
    min_inlier_ratio: float = .65
    max_plane_rms_m: float = .005
    min_plane_support_m: float = .025
    max_ground_slope_deg: float = 45.
    min_support_sectors: int = 3
    min_diameter_m: float = .03
    max_diameter_m: float = 3.
    max_reprojection_px: float = 2.
    max_relative_tangent_error: float = .08
    max_condition_number: float = 1000.
    min_view_cosine: float = .02
    box_sigma_px: float = .75
    plane_angle_sigma_deg: float = .5
    plane_distance_sigma_m: float = .005
    max_relative_diameter_sigma: float = .25
    max_center_sigma_m: float = .15
    border_margin_px: float = 1.
    seed: int = 42

    def __post_init__(self):
        if not all(math.isfinite(v) for k, v in vars(self).items()
                   if not (k == 'max_depth_m' and v is None)):
            raise ValueError('geometry configuration must be finite')
        positive = ('min_box_height_px', 'plane_threshold_m', 'max_plane_rms_m',
                    'min_plane_support_m', 'max_reprojection_px', 'max_relative_tangent_error',
                    'max_condition_number', 'box_sigma_px', 'plane_angle_sigma_deg',
                    'plane_distance_sigma_m', 'max_relative_diameter_sigma', 'max_center_sigma_m')
        if any(getattr(self, k) <= 0 for k in positive):
            raise ValueError('geometry thresholds/uncertainty floors must be positive')
        if not 0 < self.min_view_cosine <= 1 or not 0 <= self.max_ground_slope_deg < 90:
            raise ValueError('invalid viewing/ground angles')
        if min(self.ring_padding_px, self.ring_scale, self.border_margin_px, self.seed,
               self.ground_local_radius_diameters) < 0:
            raise ValueError('negative geometry padding/seed')
        if self.min_depth_m <= 0 or (self.max_depth_m is not None and self.max_depth_m <= self.min_depth_m):
            raise ValueError('invalid depth range')
        if self.max_points < self.min_points or self.min_points < 6:
            raise ValueError('invalid plane sample limits')
        if self.ransac_iterations < 1 or not 0 < self.min_inlier_ratio <= 1:
            raise ValueError('invalid RANSAC settings')
        if not 0 < self.min_diameter_m < self.max_diameter_m:
            raise ValueError('invalid diameter limits')
        if self.min_support_sectors not in (1, 2, 3, 4):
            raise ValueError('support sector count must be 1..4')

    def valid_depth(self, depth):
        depth = np.asarray(depth)
        valid = np.isfinite(depth) & (depth >= self.min_depth_m)
        if self.max_depth_m is not None:
            valid &= depth <= self.max_depth_m
        return valid


def plane_basis(normal):
    n = np.asarray(normal, dtype=float)
    n = n/np.linalg.norm(n)
    reference = np.eye(3)[np.argmin(np.abs(n))]
    e1 = reference - np.dot(reference, n)*n
    e1 /= np.linalg.norm(e1)
    return np.column_stack((e1, np.cross(n, e1)))


def project_circle(circle: Circle, camera: Camera) -> Box:
    """Exact pinhole extrema, avoiding a sampled polygon approximation."""
    basis = plane_basis(circle.normal)
    center = camera.K@circle.center
    axes = camera.K@(basis*circle.radius)
    d, e, f = center[2], axes[2, 0], axes[2, 1]
    if d <= math.hypot(e, f)+1e-8:
        raise InvalidMeasurement('circle_crosses_camera_plane')
    limits = []
    for k in (0, 1):
        a, b, c = center[k], axes[k, 0], axes[k, 1]
        coefficients = [d*d-e*e-f*f, -2*a*d+2*b*e+2*c*f, a*a-b*b-c*c]
        roots = np.roots(coefficients)
        if len(roots) != 2 or np.max(np.abs(roots.imag)) > 1e-5:
            raise InvalidMeasurement('degenerate_projection')
        limits.append(np.sort(roots.real))
    try:
        return Box(limits[0][0], limits[1][0], limits[0][1], limits[1][1])
    except ValueError as exc:
        raise InvalidMeasurement('degenerate_projection') from exc


def circle_from_box(box: Box, camera: Camera, plane: Plane, max_condition=1000.) -> Circle:
    origin = -plane.offset*plane.normal
    basis = plane_basis(plane.normal)
    l, t, r, b = box.xyxy
    image_lines = np.array([[1, 0, -l], [-1, 0, r], [0, 1, -t], [0, -1, b]])
    ray_planes = image_lines@camera.K
    ab = ray_planes@basis
    constants = ray_planes@origin
    lengths = np.linalg.norm(ab, axis=1)
    if np.min(lengths) < 1e-9:
        raise InvalidMeasurement('ray_plane_parallel_to_rim_plane')
    ab, constants = ab/lengths[:, None], constants/lengths
    system = np.column_stack((ab, -np.ones(4)))
    if np.linalg.cond(system) > max_condition:
        raise InvalidMeasurement('ill_conditioned_box_geometry')
    solution = np.linalg.lstsq(system, -constants, rcond=None)[0]
    if not np.isfinite(solution).all() or solution[2] <= 0:
        raise InvalidMeasurement('nonpositive_circle_radius')
    result = Circle(origin+basis@solution[:2], plane.normal.copy(), float(solution[2]))
    result.residual_m = float(np.sqrt(np.mean((system@solution+constants)**2)))
    projected = project_circle(result, camera)
    result.reprojection_px = float(np.max(np.abs(np.array(projected.xyxy)-box.xyxy)))
    return result


def ground_samples(depth, camera, box, all_boxes, cfg):
    """Sample outside ALL detected crater boxes, retaining sector coverage."""
    depth = np.asarray(depth)
    if depth.shape != (camera.height, camera.width):
        raise InvalidMeasurement('depth_image_size_mismatch')
    px = max(cfg.ring_padding_px, box.width*cfg.ring_scale)
    py = max(cfg.ring_padding_px, box.height*cfg.ring_scale)
    x0, x1 = max(0, math.floor(box.left-px)), min(camera.width, math.ceil(box.right+px))
    y0, y1 = max(0, math.floor(box.top-py)), min(camera.height, math.ceil(box.bottom+py))
    yy, xx = np.mgrid[y0:y1, x0:x1]
    z = depth[y0:y1, x0:x1]
    valid = cfg.valid_depth(z)
    for other in all_boxes:
        valid &= ~((xx >= other.left-1) & (xx <= other.right+1) &
                   (yy >= other.top-1) & (yy <= other.bottom+1))
    xx, yy, z = xx[valid], yy[valid], z[valid]
    if cfg.ground_local_radius_diameters:
        bx0, bx1 = max(0, math.floor(box.left)), min(camera.width, math.ceil(box.right)+1)
        by0, by1 = max(0, math.floor(box.top)), min(camera.height, math.ceil(box.bottom)+1)
        roi = depth[by0:by1, bx0:bx1]
        rz = roi[cfg.valid_depth(roi)]
        if not len(rz):
            raise InvalidMeasurement('insufficient_candidate_depth')
        target_z = float(np.median(rz))
        target = camera.rays([[(box.left+box.right)/2, (box.top+box.bottom)/2]])[0]*target_z
        apparent_diameter = box.width*target_z/camera.K[0,0]
        radius = max(2*cfg.min_plane_support_m, cfg.ground_local_radius_diameters*apparent_diameter)
        points = camera.rays(np.column_stack((xx, yy)))*z[:, None]
        local = np.linalg.norm(points-target, axis=1) <= radius
        xx, yy, z = xx[local], yy[local], z[local]
    if len(z) < cfg.min_points:
        raise InvalidMeasurement('insufficient_surrounding_depth')
    cx, cy = (box.left+box.right)/2, (box.top+box.bottom)/2
    dx, dy = (xx-cx)/max(box.width/2, 1), (yy-cy)/max(box.height/2, 1)
    sectors = np.where(np.abs(dx) > np.abs(dy), np.where(dx < 0, 0, 1), np.where(dy < 0, 2, 3))
    # Stratified sampling avoids losing the short sides of an elongated box.
    rng = np.random.default_rng(cfg.seed)
    indices = []
    for s in range(4):
        pool = np.flatnonzero(sectors == s)
        indices.extend(rng.choice(pool, min(len(pool), cfg.max_points//4), replace=False))
    indices = np.asarray(indices, dtype=int)
    uv = np.column_stack((xx[indices], yy[indices]))
    points = camera.rays(uv)*z[indices, None]
    return points, sectors[indices]


def fit_plane(points, sectors, expected_up, cfg):
    points = np.asarray(points, dtype=float)
    up = np.array(expected_up, dtype=float, copy=True)
    if not np.isfinite(up).all() or np.linalg.norm(up) < 1e-8:
        raise InvalidMeasurement('invalid_gravity_direction')
    up /= np.linalg.norm(up)
    if len(points) < cfg.min_points:
        raise InvalidMeasurement('insufficient_plane_points')
    rng = np.random.default_rng(cfg.seed)
    samples = np.array([rng.choice(len(points), 3, replace=False) for _ in range(cfg.ransac_iterations)])
    p0, p1, p2 = (points[samples[:, i]] for i in range(3))
    normals = np.cross(p1-p0, p2-p0)
    norms = np.linalg.norm(normals, axis=1)
    valid = norms > 1e-10
    normals, p0 = normals[valid]/norms[valid, None], p0[valid]
    normals *= np.where(normals@up >= 0, 1., -1.)[:, None]
    valid = normals@up >= math.cos(math.radians(cfg.max_ground_slope_deg))
    normals, p0 = normals[valid], p0[valid]
    if len(normals) == 0:
        raise InvalidMeasurement('no_ground_plane_hypothesis')
    offsets = -np.sum(normals*p0, axis=1)
    inliers = np.abs(points@normals.T+offsets) <= cfg.plane_threshold_m
    best = inliers[:, np.argmax(np.sum(inliers, axis=0))]
    # Refit to convergence, then use Huber weights to stop near-threshold
    # rocks/rim contamination from rotating the reference plane.
    for _ in range(8):
        if np.sum(best) < cfg.min_points or np.mean(best) < cfg.min_inlier_ratio:
            raise InvalidMeasurement('weak_plane_consensus')
        support = points[best]
        centroid = support.mean(axis=0)
        centered = support-centroid
        eigenvalues, vectors = np.linalg.eigh(centered.T@centered/len(support))
        normal = vectors[:, 0]
        if normal@up < 0:
            normal = -normal
        offset = -float(normal@centroid)
        updated = np.abs(points@normal+offset) <= cfg.plane_threshold_m
        if np.array_equal(best, updated):
            break
        best = updated
    support = points[best]
    for _ in range(8):
        residual = support@normal+offset
        scale = max(.00025, 1.4826*np.median(np.abs(residual-np.median(residual))))
        weights = np.minimum(1., 1.345*scale/np.maximum(np.abs(residual), 1e-12))
        centroid = np.average(support, axis=0, weights=weights)
        centered = support-centroid
        eigenvalues, vectors = np.linalg.eigh((centered*weights[:,None]).T@centered/weights.sum())
        normal = vectors[:,0]
        if normal@up < 0: normal = -normal
        offset = -float(normal@centroid)
    best = np.abs(points@normal+offset) <= cfg.plane_threshold_m
    if normal@up < math.cos(math.radians(cfg.max_ground_slope_deg)):
        raise InvalidMeasurement('plane_too_steep')
    if np.sum(best) < cfg.min_points or np.mean(best) < cfg.min_inlier_ratio:
        raise InvalidMeasurement('weak_plane_consensus')
    sector_counts = np.bincount(np.asarray(sectors)[best], minlength=4)
    if np.count_nonzero(sector_counts >= max(3, cfg.min_points//10)) < cfg.min_support_sectors:
        raise InvalidMeasurement('one_sided_plane_support')
    spread = float(np.sqrt(max(eigenvalues[1], 0.)))
    rms = float(np.sqrt(np.mean((points[best]@normal+offset)**2)))
    if spread < cfg.min_plane_support_m or rms > cfg.max_plane_rms_m:
        raise InvalidMeasurement('poor_plane_fit')
    return Plane(normal, offset, rms, float(np.mean(best)), spread, int(np.sum(best)))


def estimate_circle(detection: Detection, depth, camera, all_boxes, expected_up, cfg):
    box = detection.box
    if detection.label != 'crater':
        raise InvalidMeasurement('partial_crater_no_diameter')
    if box.touches_border(camera, cfg.border_margin_px):
        raise InvalidMeasurement('image_border_truncation')
    if box.height < cfg.min_box_height_px:
        raise InvalidMeasurement('box_too_thin_for_metric_geometry')
    points, sectors = ground_samples(depth, camera, box, all_boxes, cfg)
    plane = fit_plane(points, sectors, expected_up, cfg)
    circle = circle_from_box(box, camera, plane, cfg.max_condition_number)
    if not cfg.min_diameter_m <= circle.diameter <= cfg.max_diameter_m:
        raise InvalidMeasurement('diameter_out_of_bounds')
    if circle.reprojection_px > cfg.max_reprojection_px or circle.residual_m/circle.radius > cfg.max_relative_tangent_error:
        raise InvalidMeasurement('box_inconsistent_with_circular_rim')
    view = circle.center/np.linalg.norm(circle.center)
    if abs(plane.normal@view) < cfg.min_view_cosine:
        raise InvalidMeasurement('view_too_grazing')
    # Do not extrapolate a close-range depth plane into distant, unmeasured craters.
    basis = plane_basis(plane.normal)
    dz = circle.radius*np.linalg.norm(basis[2])
    if not cfg.valid_depth([circle.center[2]-dz, circle.center[2]+dz]).all():
        raise InvalidMeasurement('rim_outside_depth_operating_range')
    angle_sigma = max(math.radians(cfg.plane_angle_sigma_deg), plane.rms_m/max(plane.support_m, 1e-6))
    distance_sigma = max(cfg.plane_distance_sigma_m, plane.rms_m)
    sigmas = [cfg.box_sigma_px]*4 + [angle_sigma]*2 + [distance_sigma]
    jacobian = np.zeros((4, 7))
    for axis, sigma in enumerate(sigmas):
        if sigma <= 0:
            continue
        estimates = []
        for sign in (-1, 1):
            coords = np.array(box.xyxy, dtype=float)
            normal, offset = plane.normal.copy(), plane.offset
            if axis < 4:
                coords[axis] += sign*sigma
            elif axis < 6:
                normal += sign*sigma*basis[:, axis-4]
                normal /= np.linalg.norm(normal)
                # Keep the nominal circle centre on the perturbed plane.
                offset = -float(normal@circle.center)
            else:
                offset += sign*sigma
            try:
                estimate = circle_from_box(Box(*coords), camera, Plane(normal, offset), cfg.max_condition_number)
            except (ValueError, np.linalg.LinAlgError) as exc:
                raise InvalidMeasurement('unstable_uncertainty_estimate') from exc
            estimates.append(np.r_[estimate.center, estimate.diameter])
        jacobian[:, axis] = (estimates[1]-estimates[0])/2
    covariance = jacobian@jacobian.T
    circle.covariance = covariance[:3, :3]
    circle.diameter_variance = float(covariance[3, 3])
    if math.sqrt(circle.diameter_variance)/circle.diameter > cfg.max_relative_diameter_sigma:
        raise InvalidMeasurement('diameter_uncertainty_too_large')
    if math.sqrt(max(np.linalg.eigvalsh(circle.covariance))) > cfg.max_center_sigma_m:
        raise InvalidMeasurement('position_uncertainty_too_large')
    return circle, plane


def estimate_supported_geometry(box, depth, camera, all_boxes, expected_up, cfg, diameter_m=None):
    """A detector bbox is an ROI, not four measured rim tangencies.

    Retain the strict circle path when supported. With an external diameter,
    or a non-tangent bbox, intersect the centre ray and horizontal bbox chord
    with the fitted ground. This is explicitly a bbox geometry approximation;
    it still needs observed wall gradients downstream. Plane/range/border
    checks are not relaxed and the discrepancy is returned as diagnostics.
    """
    if box.touches_border(camera,cfg.border_margin_px):
        raise InvalidMeasurement('image_border_truncation')
    if box.height < cfg.min_box_height_px:
        raise InvalidMeasurement('box_too_thin_for_metric_geometry')
    if diameter_m is not None and (not np.isfinite(diameter_m) or diameter_m <= 0):
        raise InvalidMeasurement('invalid_external_diameter')
    strict_error=None
    if diameter_m is None:
        try:
            circle,plane=estimate_circle(Detection(box,'crater',1.),depth,camera,all_boxes,expected_up,cfg)
            return circle,plane,{'method':'bbox_tangent_circle','strict_circle_rejection':None}
        except InvalidMeasurement as e:
            strict_error=str(e)
            if strict_error not in ('box_inconsistent_with_circular_rim','diameter_uncertainty_too_large',
                                     'unstable_uncertainty_estimate','position_uncertainty_too_large'):
                raise
    points,sectors=ground_samples(depth,camera,box,all_boxes,cfg)
    plane=fit_plane(points,sectors,expected_up,cfg)
    cy=(box.top+box.bottom)/2;cx=(box.left+box.right)/2
    rays=camera.rays([[cx,cy],[box.left,cy],[box.right,cy]])
    denom=rays@plane.normal
    if np.min(np.abs(denom)/np.linalg.norm(rays,axis=1))<cfg.min_view_cosine:
        raise InvalidMeasurement('view_too_grazing')
    xyz=rays*(-plane.offset/denom)[:,None]
    if not cfg.valid_depth(xyz[:,2]).all():
        raise InvalidMeasurement('bbox_chord_outside_depth_operating_range')
    bbox_diameter=float(np.linalg.norm(xyz[2]-xyz[1]))
    diameter=bbox_diameter if diameter_m is None else float(diameter_m)
    if not cfg.min_diameter_m <= diameter <= cfg.max_diameter_m:
        raise InvalidMeasurement('diameter_out_of_bounds')
    circle=Circle(xyz[0],plane.normal.copy(),diameter/2)
    projection=project_circle(circle,camera)
    circle.reprojection_px=float(np.max(np.abs(np.array(projection.xyxy)-box.xyxy)))
    circle.diameter_variance=(bbox_diameter*2*cfg.box_sigma_px/max(box.width,1))**2
    circle.covariance=np.eye(3)*max(plane.rms_m,.0005)**2
    return circle,plane,{'method':'external_diameter_ground_center_ray' if diameter_m is not None else 'bbox_ground_chord',
        'strict_circle_rejection':strict_error,'bbox_ground_chord_diameter_m':bbox_diameter,
        'rim_tangency_assumed':False,'bbox_reprojection_discrepancy_px':circle.reprojection_px}
