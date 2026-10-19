"""Terrain-corrected angle -> paraboloid depth -> triangle mesh.

Zhou et al. 2023, Measurement 210:112550, Eq. 6 supplies the paraboloid
shape family, NOT this inverse estimator. We use the zero-raised-rim limit.
All quantities use metres in a local frame whose normal follows ground.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
import numpy as np
from scipy.spatial import ConvexHull, QhullError
from .acceleration import accelerate


@dataclass(frozen=True)
class Config:
    radial_bin_r: float = .1
    angular_sectors: int = 12
    minimum_points_per_cell: int = 3
    wall_min_r: float = .1
    wall_max_r: float = .9
    slope_window_r: float = .2
    ground_min_r: float = 1.25
    ground_max_r: float = 2.4
    mesh_extent_r: float = 1.4
    mesh_size: int = 65


def design(xy_r, degree):
    x, y = np.asarray(xy_r).T
    base = [np.ones_like(x), x, y]
    if degree == 2:
        base += [x*x, x*y, y*y]
    return np.column_stack(base)


def robust_fit(xy_r, height_m, degree):
    matrix = design(xy_r, degree)
    if len(matrix) < 3 * matrix.shape[1] or np.linalg.matrix_rank(matrix) < matrix.shape[1]:
        raise ValueError('insufficient_ground_surface_support')
    if np.linalg.cond(matrix) > 1e4:
        raise ValueError('ill_conditioned_ground_surface')
    weights = np.ones(len(matrix))
    coef = np.zeros(matrix.shape[1])
    for _ in range(12):
        coef = np.linalg.lstsq(matrix * np.sqrt(weights[:, None]),
                               height_m * np.sqrt(weights), rcond=None)[0]
        residual = height_m - matrix @ coef
        scale = max(1e-5, 1.4826 * np.median(np.abs(residual-np.median(residual))))
        weights = np.minimum(1., 1.345*scale/np.maximum(np.abs(residual), 1e-12))
    return coef


def fit_terrain(xy_r, height_m, mode='auto'):
    """Outside-only ground fit; angular held-out folds choose curvature.

The hull check and spatial holdouts limit unsupported extrapolation, but do
not certify the hidden ground under the crater. Residuals include roughness.
"""
    xy_r, height_m = np.asarray(xy_r), np.asarray(height_m)
    if len(xy_r) < 60:
        raise ValueError('insufficient_outside_ground_points')
    # Equal-area cells reduce the effect of perspective-dependent pixel density.
    keys = np.floor(xy_r/.12).astype(int)
    _, inv = np.unique(keys, axis=0, return_inverse=True)
    order=np.argsort(inv,kind='stable'); breaks=np.r_[0,np.flatnonzero(np.diff(inv[order]))+1,len(order)]
    med = accelerate('grouped_medians', np.column_stack((xy_r, height_m))[order], breaks)
    if med is not None:
        xy, z = med[:, :2], med[:, 2]
    else:
        xy = np.array([np.median(xy_r[order[a:b]],axis=0) for a,b in zip(breaks[:-1],breaks[1:])])
        z = np.array([np.median(height_m[order[a:b]]) for a,b in zip(breaks[:-1],breaks[1:])])
    angles = np.arctan2(xy[:, 1], xy[:, 0])
    folds = np.minimum(7, ((angles+np.pi)/(2*np.pi)*8).astype(int))
    try:
        hull = ConvexHull(xy)
    except QhullError as exc:
        raise ValueError('ground_points_are_collinear') from exc
    encloses_center = bool(np.all(hull.equations[:, -1] <= 0))
    coefficients, scores, centers, residuals = {}, {}, {}, {}
    for degree in (1, 2):
        try:
            coef = robust_fit(xy, z, degree)
            errors, predicted_centers = [], []
            for k in np.unique(folds):
                train, test = folds != k, folds == k
                if test.sum() < 3:
                    continue
                beta = robust_fit(xy[train], z[train], degree)
                errors.append(float(np.median(np.abs(z[test]-design(xy[test], degree)@beta))))
                predicted_centers.append(float(beta[0]))
            if len(errors) < 4:
                raise ValueError('insufficient_angular_ground_coverage')
            coefficients[degree] = coef
            scores[degree] = float(np.median(errors))
            centers[degree] = float(np.ptp(predicted_centers))
            residuals[degree] = float(np.sqrt(np.mean((z-design(xy, degree)@coef)**2)))
        except ValueError:
            continue
    if 1 not in coefficients:
        raise ValueError('ground_plane_cross_validation_failed')
    quad_supported = bool(2 in coefficients and encloses_center and scores[1] > 1e-5 and
                          scores[2] <= .85*scores[1] and
                          centers[2] <= max(.002, 2*scores[1]))
    if mode == 'quadratic' and (2 not in coefficients or not encloses_center):
        raise ValueError('quadratic_ground_extrapolation_unsupported')
    degree = 2 if (mode == 'quadratic' or mode == 'auto' and quad_supported) else 1
    return {'degree': degree, 'coefficients_m': coefficients[degree],
            'ground_cells': len(z), 'ground_points': len(height_m),
            'angular_folds': len(np.unique(folds)), 'hull_encloses_crater_center': encloses_center,
            'linear_holdout_mae_m': scores[1], 'quadratic_holdout_mae_m': scores.get(2),
            'quadratic_center_fold_range_m': centers.get(2),
            'quadratic_supported_by_holdout': quad_supported,
            'selected_ground_rms_m': residuals[degree],
            'forced_quadratic': bool(mode == 'quadratic' and not quad_supported)}


def surface_height(xy_r, terrain):
    return design(xy_r, terrain['degree']) @ np.asarray(terrain['coefficients_m'])


def profile_grid(xy_r, residual_m, cfg=Config(), *, return_positions=False, background_m=None):
    rho = np.linalg.norm(xy_r, axis=1)
    angle = np.arctan2(xy_r[:, 1], xy_r[:, 0])
    sector = np.minimum(cfg.angular_sectors-1,
                        ((angle+np.pi)/(2*np.pi)*cfg.angular_sectors).astype(int))
    count = int(round(1.2/cfg.radial_bin_r))
    centers = (np.arange(count)+.5)*cfg.radial_bin_r
    background = np.zeros_like(residual_m) if background_m is None else np.asarray(background_m)
    accelerated = accelerate('profile_cells', rho, sector, residual_m, background, centers,
                             cfg.radial_bin_r, cfg.angular_sectors,
                             cfg.minimum_points_per_cell, background_m is not None)
    if accelerated is not None:
        heights, counts, positions, backgrounds = accelerated
        if return_positions:
            return centers, heights, counts, positions, backgrounds
        return centers, heights, counts
    heights = np.full((cfg.angular_sectors, count), np.nan)
    counts = np.zeros_like(heights, dtype=int)
    positions = np.full_like(heights,np.nan)
    backgrounds = np.full_like(heights,np.nan)
    for s in range(cfg.angular_sectors):
        for j, r in enumerate(centers):
            take = ((sector == s) & (rho >= r-cfg.radial_bin_r/2) &
                    (rho < r+cfg.radial_bin_r/2))
            counts[s, j] = int(take.sum())
            if counts[s, j] >= cfg.minimum_points_per_cell:
                heights[s, j] = np.median(residual_m[take])
                positions[s,j] = np.median(rho[take])
                if background_m is not None: backgrounds[s,j] = np.median(np.asarray(background_m)[take])
    if return_positions:
        return centers,heights,counts,positions,backgrounds
    return centers, heights, counts


def depth_from_slope(radius_m, angle_deg, position_r=1.):
    """Paraboloid secant midpoint identity, not a diameter-only prior.

    z(r) = d*((r/R)^2-1); tan(theta_obs) = 2*d*r_mid/R^2.
    theta_max at r=R gives d=diameter*tan(theta_max)/4.
    """
    if not (np.isfinite([radius_m, angle_deg, position_r]).all() and
            radius_m > 0 and 0 < angle_deg < 85 and 0 < position_r <= 1):
        raise ValueError('invalid_radius_angle_or_slope_position')
    return radius_m*math.tan(math.radians(angle_deg))/(2*position_r)


def measure_slopes(centers_r, heights_m, radius_m, cfg=Config(), *, positions_r=None, background_heights_m=None):
    """Maximum positive inward-wall rise with observed support in each sector.

    NaN gaps are never bridged; each sector needs at least two eligible windows.
    Max, P95 and per-window inferred depths remain separate in output.
    """
    centers_r = np.asarray(centers_r)
    step = int(round(cfg.slope_window_r/cfg.radial_bin_r))
    if step < 2:
        raise ValueError('window_requires_at_least_three_bins')
    windows = []
    signed_windows = []
    for sector, values in enumerate(heights_m):
        candidate = []
        for start in range(len(centers_r)-step):
            end = start+step
            if centers_r[start] < cfg.wall_min_r or centers_r[end] > cfg.wall_max_r:
                continue
            segment = values[start:end+1]
            if not np.isfinite(segment).all():
                continue
            rr = centers_r[start:end+1] if positions_r is None else np.asarray(positions_r)[sector,start:end+1]
            if not np.isfinite(rr).all() or np.any(np.diff(rr)<=0): continue
            center=float((rr[0]+rr[-1])/2)
            width=float((rr[-1]-rr[0])*radius_m)
            if width < cfg.slope_window_r*radius_m*.5: continue
            # Use actual radial sample locations, not nominal bin centres.
            # A local fit uses the interior bin too, reducing endpoint noise.
            xx=rr*radius_m; dx=xx-xx.mean()
            gradient=float(dx@(segment-segment.mean())/(dx@dx))
            rise=gradient*width
            bg_gradient=0.
            if background_heights_m is not None:
                bg=np.asarray(background_heights_m)[sector,start:end+1]
                if not np.isfinite(bg).all(): continue
                bg_gradient=float(dx@(bg-bg.mean())/(dx@dx))
            # Angle between the observed radial section and LOCAL background
            # tangent. atan(residual gradient) is only exact on flat background.
            angle=math.degrees(math.atan(gradient+bg_gradient)-math.atan(bg_gradient))
            if abs(angle) >= 85:
                continue
            candidate.append({'sector': sector, 'center_r': center, 'angle_deg': angle,
                              'rise_m': rise, 'baseline_m': width,
                              'start_r': float(rr[0]), 'end_r': float(rr[-1]),
                              'background_gradient':bg_gradient,'observed_gradient':gradient+bg_gradient,
                              'fit_rms_m':float(np.sqrt(np.mean((segment-segment.mean()-gradient*dx)**2))),
                              'depth_from_angle_m': radius_m*(rise/width)/(2*center)})
        if len(candidate) >= 2:
            signed_windows.extend(candidate)
            windows.extend(w for w in candidate if w['rise_m'] > 0)
    if len(windows) < 3:
        raise ValueError('insufficient_supported_wall_windows')
    maximum = max(windows, key=lambda w:w['angle_deg'])
    d = maximum['depth_from_angle_m']
    # A paraboloid has radial gradient k*rho, k=2d/R. Fit every signed
    # supported gradient, including negative ones, so choosing positive/noisy
    # extrema cannot silently inflate the completed depth.
    x=np.array([w['center_r'] for w in signed_windows])
    g=np.array([w['rise_m']/w['baseline_m'] for w in signed_windows])
    k=float(np.median(g/x))
    for _ in range(15):
        e=g-k*x
        scale=max(1e-6,1.4826*np.median(np.abs(e-np.median(e))))
        weights=np.minimum(1.,1.345*scale/np.maximum(np.abs(e),1e-12))
        k=float(np.sum(weights*x*g)/np.sum(weights*x*x))
    if k<=0:
        raise ValueError('no_positive_paraboloid_gradient')
    model_angle=math.degrees(math.atan(k))
    robust_depth=depth_from_slope(radius_m,model_angle,1.)
    sectors = sorted({w['sector'] for w in windows})
    return {'observed_max_angle_deg': maximum['angle_deg'],
            'angle_formula':'degrees(atan(observed radial gradient)-atan(local ground radial gradient))',
            'max_background_gradient':maximum['background_gradient'],
            'max_observed_gradient':maximum['observed_gradient'],
            'observed_max_sector': maximum['sector'], 'observed_max_at_r': maximum['center_r'],
            'max_window_start_r': maximum['start_r'], 'max_window_end_r': maximum['end_r'],
            'max_window_rise_m': maximum['rise_m'], 'max_window_baseline_m': maximum['baseline_m'],
            'angle_p95_deg': float(np.percentile([w['angle_deg'] for w in windows],95)),
            'raw_max_depth_m':d, 'raw_max_extrapolated_rim_angle_deg':math.degrees(math.atan2(2*d,radius_m)),
            'inferred_depth_m':robust_depth, 'inferred_rim_max_angle_deg':model_angle,
            'gradient_fit_rms':float(np.sqrt(np.mean((g-k*x)**2))),
            'per_window_depth_p10_p90_m': np.percentile([w['depth_from_angle_m'] for w in windows],[10,90]).tolist(),
            'valid_sectors': sectors, 'valid_sector_fraction': len(sectors)/cfg.angular_sectors,
            'windows':signed_windows}


def make_mesh(radius_m, depth_m, terrain, circle_center, basis, normal, cfg=Config()):
    q = np.linspace(-cfg.mesh_extent_r,cfg.mesh_extent_r,cfg.mesh_size)
    x, y = np.meshgrid(q,q)
    xy_r = np.column_stack((x.ravel(),y.ravel()))
    rho = np.linalg.norm(xy_r,axis=1)
    background = surface_height(xy_r,terrain)
    depression = np.where(rho < 1, depth_m*(rho*rho-1),0.)
    local = np.column_stack((xy_r*radius_m, background+depression))
    camera = circle_center + local[:,:2]@basis.T + local[:,2,None]*normal
    faces=[]
    for j in range(cfg.mesh_size-1):
        for i in range(cfg.mesh_size-1):
            a=j*cfg.mesh_size+i; b=a+1; c=a+cfg.mesh_size; d=c+1
            faces.extend(((a,b,d),(a,d,c)))
    return local,camera,np.asarray(faces,dtype=int),depression


def write_obj(path, vertices, faces):
    with open(path,'w',encoding='ascii') as out:
        out.write('# Model-inferred crater completion. Units: metres. Not measured hidden geometry.\n')
        for x,y,z in vertices:
            out.write(f'v {x:.9f} {y:.9f} {z:.9f}\n')
        for a,b,c in faces:
            out.write(f'f {a+1} {b+1} {c+1}\n')


def write_points_ply(path, xyz):
    with open(path,'w',encoding='ascii') as out:
        out.write('ply\nformat ascii 1.0\ncomment observed aligned Depth points, metres\n')
        out.write(f'element vertex {len(xyz)}\nproperty float x\nproperty float y\nproperty float z\nend_header\n')
        np.savetxt(out,xyz,fmt='%.9f')
