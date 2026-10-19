"""Optional raw-depth adapter to existing crater_analysis geometry code.

Uses the caller's installed NumPy and existing crater_analysis. No Numba or
mesh/depth-model fit is required. Ranking core remains standard-library-only.
All physical lengths are metres, final angles degrees. K must match the
aligned depth image; depth must already be converted with the sensor scale.
"""
import math
import numpy as np


def observed_windows(centers, heights, radius, cfg, positions, backgrounds):
    """Same observed-window geometry as the handoff, decoupled from depth fit."""
    if not np.isfinite(radius) or radius<=0:raise ValueError('invalid_radius')
    step=int(round(cfg.slope_window_r/cfg.radial_bin_r))
    if step<2:raise ValueError('window_requires_three_bins')
    windows=[]
    for sector,values in enumerate(heights):
        candidates=[]
        for start in range(len(centers)-step):
            end=start+step
            if centers[start]<cfg.wall_min_r or centers[end]>cfg.wall_max_r:continue
            yy=values[start:end+1];rr=positions[sector,start:end+1]
            bg=backgrounds[sector,start:end+1]
            if not all(np.isfinite(a).all() for a in (yy,rr,bg)) or np.any(np.diff(rr)<=0):continue
            width=float((rr[-1]-rr[0])*radius)
            if width<cfg.slope_window_r*radius*.5:continue
            x=rr*radius;dx=x-x.mean()
            residual_gradient=float(dx@(yy-yy.mean())/(dx@dx))
            ground_gradient=float(dx@(bg-bg.mean())/(dx@dx))
            observed_gradient=residual_gradient+ground_gradient
            angle=math.degrees(math.atan(observed_gradient)-math.atan(ground_gradient))
            if abs(angle)>=85:continue
            candidates.append(dict(sector=sector,angle_deg=angle,rise_m=residual_gradient*width,
                                   baseline_m=width,start_r=float(rr[0]),end_r=float(rr[-1]),
                                   center_r=float((rr[0]+rr[-1])/2),observed_gradient=observed_gradient,
                                   background_gradient=ground_gradient))
        if len(candidates)>=2:windows.extend(candidates)
    rising=[w for w in windows if w['rise_m']>0]
    if len(rising)<3:raise ValueError('insufficient_supported_wall_windows')
    maximum=max(rising,key=lambda w:w['angle_deg'])
    return dict(angle_p95_deg=float(np.percentile([w['angle_deg'] for w in rising],95)),
                observed_max_angle_deg=maximum['angle_deg'],
                valid_sectors=sorted({w['sector'] for w in rising}),
                valid_sector_fraction=len({w['sector'] for w in rising})/cfg.angular_sectors,
                max_window_rise_m=maximum['rise_m'],max_window_baseline_m=maximum['baseline_m'],
                windows=windows,positive_windows=len(rising))


def measure_observed(depth,camera,boxes,box_index,cfg=None,geometry_config=None,expected_up=None):
    from crater_analysis.core import Config,fit_terrain,surface_height,profile_grid
    from crater_analysis.geometry import GeometryConfig,ground_samples,estimate_supported_geometry,plane_basis
    cfg=cfg or Config(wall_max_r=.7)
    geometry=geometry_config or GeometryConfig(max_ground_slope_deg=60.)
    box=boxes[box_index]
    if expected_up is None:
        pts,_=ground_samples(depth,camera,box,boxes,geometry)
        _,_,vt=np.linalg.svd(pts-pts.mean(axis=0),full_matrices=False);up=vt[-1]
        if up[1]>0:up=-up
    else:up=np.asarray(expected_up,dtype=float)
    circle,plane,diagnostic=estimate_supported_geometry(box,depth,camera,boxes,up,geometry,None)
    basis=plane_basis(plane.normal);yy,xx=np.mgrid[:camera.height,:camera.width]
    valid=geometry.valid_depth(depth);uv=np.column_stack((xx[valid],yy[valid]))
    xyz=camera.rays(uv)*depth[valid,None];xy=(xyz-circle.center)@basis/circle.radius
    z=(xyz-circle.center)@plane.normal;rho=np.linalg.norm(xy,axis=1)
    near=rho<=cfg.ground_max_r;xyz,uv,xy,z,rho=xyz[near],uv[near],xy[near],z[near],rho[near]
    ground=rho>=cfg.ground_min_r;crater=rho<=1.2
    for j,other in enumerate(boxes):
        inside=(uv[:,0]>=other.left-2)&(uv[:,0]<=other.right+2)&(uv[:,1]>=other.top-2)&(uv[:,1]<=other.bottom+2)
        ground &= ~inside
        if j!=box_index:crater &= ~inside
    terrain=fit_terrain(xy[ground],z[ground],'auto');bg=surface_height(xy,terrain);residual=z-bg
    rr,hh,counts,rp,bp=profile_grid(xy[crater],residual[crater],cfg,return_positions=True,background_m=bg[crater])
    slopes=observed_windows(rr,hh,circle.radius,cfg,rp,bp)
    valid_angles=slopes['valid_sectors'];flags=[]
    if len(valid_angles)<.75*cfg.angular_sectors:flags.append('partial_direction_coverage')
    if slopes['max_window_rise_m']<2*terrain['selected_ground_rms_m']:flags.append('weak_height_signal_relative_to_ground_residual')
    # Preserve evidence of whether the observed ROI is actually depressed,
    # without making a completed-bowl model a prerequisite for observed slope.
    dep=residual[crater&(rho<=.9)]
    if len(dep)<15 or np.percentile(dep,20)>=-max(.0005,1.5*terrain['selected_ground_rms_m']):
        flags.append('weak_observed_depression_evidence')
    return (dict(slopes=slopes,terrain=terrain,geometry_method=diagnostic['method'],
                 diameter_used_m=circle.diameter,quality_flags=flags,
                 range_m=float(np.median(xyz[crater,2])),depth_model_used=False,
                 formula='degrees(atan(observed_radial_gradient)-atan(local_ground_radial_gradient))'),
            dict(points_local_m=np.column_stack((xy*circle.radius,z)),crater_mask=crater,ground_mask=ground,
                 profile_r=rr,sector_heights_m=hh,sector_radius_positions_r=rp,sector_background_heights_m=bp))
