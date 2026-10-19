"""Measurement pipeline, with no machine-specific paths or example inputs."""
from __future__ import annotations
from dataclasses import asdict
import json
from pathlib import Path
import cv2
import numpy as np
from .core import (Config,fit_terrain,surface_height,profile_grid,measure_slopes,
                   make_mesh,write_obj,write_points_ply)
from .geometry import GeometryConfig,ground_samples,estimate_circle,estimate_supported_geometry,plane_basis
from .types import Camera,Box,Detection

def json_ready(value):
    if isinstance(value,np.ndarray): return value.tolist()
    if isinstance(value,np.generic): return value.item()
    if isinstance(value,dict): return {k:json_ready(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)): return [json_ready(v) for v in value]
    return value


def save_geometry(report, arrays, out):
    (out/'metrics.json').write_text(json.dumps(report,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    np.savez_compressed(out/'reconstruction.npz',**arrays)
    for key,name in [('mesh_camera_m','crater_camera.obj'),('mesh_local_m','crater_local.obj')]:
        if key in arrays: write_obj(out/name,arrays[key],arrays['faces'])
    write_points_ply(out/'observed_camera.ply',arrays['observed_camera_m'])


def read_depth(path, scale, camera, invalid_raw_min=60000):
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('depth_scale_must_be_finite_positive')
    if not isinstance(invalid_raw_min, (int, np.integer)) or not 0 <= invalid_raw_min <= 65536:
        raise ValueError('invalid_raw_min_must_be_0_to_65536')
    raw=cv2.imdecode(np.fromfile(path,dtype=np.uint8),cv2.IMREAD_UNCHANGED)
    if raw is None or raw.ndim != 2 or raw.shape != (camera.height,camera.width):
        raise ValueError('aligned_depth_dimensions_do_not_match_intrinsics')
    if raw.dtype != np.uint16:
        raise ValueError('expected_16UC1_depth_PNG')
    z=raw.astype(float)*scale
    z[raw == 0]=np.nan
    if invalid_raw_min:
        z[raw >= invalid_raw_min]=np.nan
    return z


def reconstruct(depth,camera,boxes,box_index,cfg=Config(),terrain_mode='auto',
                diameter_m=None,expected_up=None,depth_estimator='robust-slope',geometry_mode='supported',
                geometry_config=None):
    geometry=geometry_config or GeometryConfig(max_ground_slope_deg=60.)
    geometry.__post_init__()
    box=boxes[box_index]
    if expected_up is None:
        p,_=ground_samples(depth,camera,box,boxes,geometry)
        _,_,vt=np.linalg.svd(p-p.mean(axis=0),full_matrices=False)
        up=vt[-1]
        if up[1]>0: up=-up
    else:
        up=np.asarray(expected_up,dtype=float)
    if geometry_mode=='strict':
        circle,plane=estimate_circle(Detection(box,'crater',1.),depth,camera,boxes,up,geometry)
        geometry_info={'method':'bbox_tangent_circle'}
    elif geometry_mode=='supported':
        circle,plane,geometry_info=estimate_supported_geometry(box,depth,camera,boxes,up,geometry,diameter_m)
    else: raise ValueError('unknown_geometry_mode')
    bbox_diameter=geometry_info.get('bbox_ground_chord_diameter_m',circle.diameter)
    if diameter_m is not None and geometry_mode=='strict':
        if not np.isfinite(diameter_m) or diameter_m <= 0:
            raise ValueError('invalid_external_diameter')
        circle.radius=diameter_m/2
    basis=plane_basis(plane.normal)
    yy,xx=np.mgrid[:camera.height,:camera.width]
    valid=geometry.valid_depth(depth)
    uv=np.column_stack((xx[valid],yy[valid]))
    xyz=camera.rays(uv)*depth[valid,None]
    local_xy=(xyz-circle.center)@basis
    xy_r=local_xy/circle.radius
    z=(xyz-circle.center)@plane.normal
    rho=np.linalg.norm(xy_r,axis=1)
    near=rho <= cfg.ground_max_r
    xyz,uv,xy_r,z,rho=xyz[near],uv[near],xy_r[near],z[near],rho[near]
    ground=(rho>=cfg.ground_min_r)
    crater_points=(rho<=1.2)
    for j,other in enumerate(boxes):
        in_box=((uv[:,0]>=other.left-2)&(uv[:,0]<=other.right+2)&
                (uv[:,1]>=other.top-2)&(uv[:,1]<=other.bottom+2))
        ground &= ~in_box
        if j != box_index:
            crater_points &= ~in_box
    terrain=fit_terrain(xy_r[ground],z[ground],terrain_mode)
    residual=z-surface_height(xy_r,terrain)
    observed_depression=residual[crater_points&(rho<=.9)]
    if len(observed_depression)<15 or np.percentile(observed_depression,20)>=-max(.0005,1.5*terrain['selected_ground_rms_m']):
        raise ValueError('insufficient_observed_depression_evidence')
    centers,heights,counts,positions,backgrounds=profile_grid(xy_r[crater_points],residual[crater_points],cfg,
        return_positions=True,background_m=surface_height(xy_r[crater_points],terrain))
    slopes=measure_slopes(centers,heights,circle.radius,cfg,positions_r=positions,background_heights_m=backgrounds)
    if depth_estimator=='observed-max':
        slopes['inferred_depth_m']=slopes['raw_max_depth_m']
        slopes['inferred_rim_max_angle_deg']=slopes['raw_max_extrapolated_rim_angle_deg']
    # Sensitivity to background choice is important because terrain curvature
    # below an occluding crater is extrapolated, never observed directly.
    variants={}
    for mode in ('plane','quadratic'):
        try:
            model=fit_terrain(xy_r[ground],z[ground],mode)
            h=z-surface_height(xy_r,model)
            rr,hh,_,rp,bg=profile_grid(xy_r[crater_points],h[crater_points],cfg,return_positions=True,
                background_m=surface_height(xy_r[crater_points],model))
            s=measure_slopes(rr,hh,circle.radius,cfg,positions_r=rp,background_heights_m=bg)
            variants[mode]={'angle_deg':s['observed_max_angle_deg'],
                            'depth_m':s['raw_max_depth_m'] if depth_estimator=='observed-max' else s['inferred_depth_m'],
                            'valid_sector_fraction':s['valid_sector_fraction']}
        except (ValueError,np.linalg.LinAlgError) as e:
            variants[mode]={'rejected':str(e)}
    mesh_local,mesh_camera,faces,depression=make_mesh(circle.radius,slopes['inferred_depth_m'],
                                      terrain,circle.center,basis,plane.normal,cfg)
    observed=crater_points&(rho<=1.)
    predicted=slopes['inferred_depth_m']*(rho[observed]**2-1)
    model_rms=float(np.sqrt(np.mean((residual[observed]-predicted)**2)))
    range_m=float(np.median(xyz[observed,2]))
    flags=['model_inferred_hidden_surface','no_absolute_geometry_ground_truth']
    if geometry_info['method']!='bbox_tangent_circle': flags.append('bbox_roi_geometry_not_measured_rim')
    if expected_up is None: flags.append('camera_frame_no_gravity_pose')
    if range_m>.5: flags.append('outside_d405_recommended_range')
    if slopes['valid_sector_fraction']<.75: flags.append('partial_direction_coverage')
    if slopes['max_window_rise_m']<2*terrain['selected_ground_rms_m']:
        flags.append('wall_rise_comparable_to_background_residual')
    if model_rms>max(2*terrain['selected_ground_rms_m'],.35*slopes['inferred_depth_m']):
        flags.append('paraboloid_does_not_fit_observed_surface_well')
    if all('depth_m' in variants[k] for k in variants):
        gap=abs(variants['plane']['depth_m']-variants['quadratic']['depth_m'])
        if gap>.25*slopes['inferred_depth_m']: flags.append('sensitive_to_background_model')
    # The fitted curved background can have a different tangent at the crater
    # centre from the initial RANSAC plane. Keep the mesh basis and this normal separate.
    ground_normal=plane.normal-basis@np.asarray(terrain['coefficients_m'][1:3])/circle.radius
    ground_normal/=np.linalg.norm(ground_normal)
    summary={'status':'exploratory_mesh','bbox_xyxy':box.xyxy,
       'geometry_method':geometry_info['method'],'geometry_diagnostics':geometry_info,
       'terrain_normal_at_crater_center_camera':ground_normal,
       'diameter_source':'external_override' if diameter_m is not None else geometry_info['method'],
       'bbox_diameter_m':bbox_diameter,'diameter_used_m':circle.diameter,
       'bbox_diameter_sigma_m':float(np.sqrt(circle.diameter_variance)),
       'bbox_reprojection_max_error_px':circle.reprojection_px,
       'circle_center_camera_m':circle.center,'ground_normal_camera':plane.normal,
       'local_to_camera_basis_columns':np.column_stack((basis,plane.normal)),
       'plane_rms_m':plane.rms_m,'terrain':terrain,'slopes':slopes,
       'depth_to_diameter_ratio':slopes['inferred_depth_m']/circle.diameter,
       'range_m':range_m,'observed_points':int(observed.sum()),
       'surface_model_rms_to_observed_m':model_rms,'background_sensitivity':variants,
       'quality_flags':flags,'use_for_preservation_ranking':False,
       'mesh_vertices':len(mesh_camera),'mesh_faces':len(faces),
       'model':'zero-raised-rim paraboloid limit of Zhou 2023 Eq.6 plus outside-only terrain fit',
       'depth_estimator':depth_estimator,
       'depth_formula':'fit signed residual radial gradients g_j=k*rho_j with Huber weights; d=R*k/2. For one window: g_residual=tan(theta_local+atan(g_background))-g_background; d=R*g_residual/(2*rho_mid). theta_local is the angle between the observed and local background tangents.',
       'not_paper_original_algorithm':True,'settings':asdict(cfg),
       'geometry_settings':asdict(geometry)}
    arrays={'mesh_local_m':mesh_local,'mesh_camera_m':mesh_camera,'faces':faces,
            'depression_m':depression,'observed_camera_m':xyz[observed],
            'points_local_m':np.column_stack((xy_r*circle.radius,z)),
            'points_uv':uv,'ground_mask':ground,'crater_mask':crater_points,
            'corrected_height_m':residual,'profile_r':centers,'sector_heights_m':heights,
            'sector_counts':counts,'sector_radius_positions_r':positions,'sector_background_heights_m':backgrounds}
    return json_ready(summary),arrays
