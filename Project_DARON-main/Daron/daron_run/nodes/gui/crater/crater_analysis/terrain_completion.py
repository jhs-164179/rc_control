"""Observation-constrained, non-axisymmetric completion of a terrain height field.

The paraboloid is only a fallback for hidden regions. Spatial cell medians from
Depth constrain a smooth 2-D displacement field, including the rim and outside
terrain. No circular contour, zero rim displacement, or forced axial symmetry
is imposed on the resulting surface. This is our extension, not Zhou's method.
"""
from dataclasses import asdict, dataclass
import numpy as np
from scipy import sparse
from scipy.sparse.linalg import spsolve
from .core import surface_height
from .acceleration import accelerate


@dataclass(frozen=True)
class TerrainCompletionConfig:
    minimum_points: int = 3
    smoothness_candidates: tuple = (1., 4., 16., 64.)
    prior_weight: float = .001
    validation_block_cells: int = 6
    huber_iterations: int = 4
    solver_size: int = 33


def prior_height(xy, radius, depth, terrain):
    q=np.asarray(xy)/radius;rho=np.linalg.norm(q,axis=1)
    return surface_height(q,terrain)+np.where(rho<1,depth*(rho*rho-1),0.)


def bilinear_matrix(xy, axis):
    """Evaluate grid heights at the actual median XY of each observed cell."""
    n=len(axis);q=(np.asarray(xy)-axis[0])/(axis[1]-axis[0])
    q=np.clip(q,0,n-1);ij=np.minimum(np.floor(q).astype(int),n-2);f=q-ij
    x,y=ij.T;u,v=f.T
    cols=np.column_stack((y*n+x,y*n+x+1,(y+1)*n+x,(y+1)*n+x+1))
    vals=np.column_stack(((1-u)*(1-v),u*(1-v),(1-u)*v,u*v))
    return sparse.csr_matrix((vals.ravel(),(np.repeat(np.arange(len(xy)),4),cols.ravel())),shape=(len(xy),n*n))


def bending_matrix(n):
    """Discrete xx, yy, and xy second differences penalise correction curvature."""
    rows=[];cols=[];values=[];k=0
    for y in range(n):
        for x in range(1,n-1):
            rows.extend([k]*3);cols.extend([y*n+x-1,y*n+x,y*n+x+1]);values.extend([1,-2,1]);k+=1
    for y in range(1,n-1):
        for x in range(n):
            rows.extend([k]*3);cols.extend([(y-1)*n+x,y*n+x,(y+1)*n+x]);values.extend([1,-2,1]);k+=1
    for y in range(n-1):
        for x in range(n-1):
            rows.extend([k]*4);cols.extend([y*n+x,y*n+x+1,(y+1)*n+x,(y+1)*n+x+1])
            values.extend([2**.5,-2**.5,-2**.5,2**.5]);k+=1
    return sparse.csr_matrix((values,(rows,cols)),shape=(k,n*n))


def observed_cells(points, mask, axis, radius, depth, terrain, cfg):
    p=np.asarray(points)[mask];inside=(np.abs(p[:,:2])<=axis[-1]).all(axis=1)&np.isfinite(p).all(axis=1)
    p=p[inside];n=len(axis);pitch=axis[1]-axis[0]
    # A small crater may have fewer Depth pixels than output mesh vertices.
    # Increase aggregation pitch, not the minimum evidence requirement.
    candidates=[]
    for stride in (1,2,3,4,6,8):
        bins=int(np.ceil((n-1)/stride))+1
        ids=np.clip(np.rint((p[:,:2]-axis[0])/(pitch*stride)).astype(int),0,bins-1)
        keys=ids[:,1]*bins+ids[:,0];_,count=np.unique(keys,return_counts=True)
        supported=count>=cfg.minimum_points
        coverage=float(count[supported].sum()/max(1,len(p)))
        candidates.append((int(supported.sum()),stride,coverage,keys))
        if supported.sum()>=40 and coverage>=.8:break
    chosen=candidates[-1] if candidates[-1][0]>=40 and candidates[-1][2]>=.8 else max(candidates,key=lambda a:a[0])
    _,stride,coverage,keys=chosen
    order=np.argsort(keys);keys=keys[order];p=p[order]
    residual=p[:,2]-prior_height(p[:,:2],radius,depth,terrain)
    breaks=np.r_[0,np.flatnonzero(np.diff(keys))+1,len(keys)]
    accelerated = accelerate('cell_statistics', p, residual, breaks, cfg.minimum_points)
    if accelerated is not None:
        xy,delta,spread,counts = accelerated
        grid = np.rint((xy-axis[0])/pitch).astype(int)
    else:
        xy=[];delta=[];spread=[];counts=[];grid=[]
        for a,b in zip(breaks[:-1],breaks[1:]):
            if b-a<cfg.minimum_points:continue
            med=float(np.median(residual[a:b]));xy.append(np.median(p[a:b,:2],axis=0))
            delta.append(med);spread.append(1.4826*np.median(abs(residual[a:b]-med)));counts.append(b-a)
            grid.append(np.rint((xy[-1]-axis[0])/pitch).astype(int).tolist())
    if len(xy)<30:raise ValueError('too_few_spatial_cells_for_terrain_completion')
    xy=np.asarray(xy);spread=np.asarray(spread);delta=np.asarray(delta)
    noise=max(.0005,float(np.median(spread)))
    weights=np.clip((noise/np.maximum(spread,noise))**2,.05,1.)
    return {'xy':xy,'delta':delta,'spread':spread,'count':np.asarray(counts),'grid':np.asarray(grid),
        'weight':weights,'noise_m':noise,'height':prior_height(xy,radius,depth,terrain)+delta,
        'aggregation_stride':stride,'aggregation_pitch_m':pitch*stride,'pixel_coverage':coverage}


def solve_correction(A, delta, weights, penalty, smoothness, cfg):
    reg=smoothness*penalty+cfg.prior_weight*sparse.eye(A.shape[1],format='csr')
    w=weights.copy();u=np.zeros(A.shape[1])
    for _ in range(cfg.huber_iterations):
        u=spsolve((A.T@A.multiply(w[:,None])+reg).tocsc(),A.T@(w*delta))
        e=A@u-delta;scale=max(.0005,1.4826*np.median(abs(e-np.median(e))))
        w=weights*np.minimum(1,1.345*scale/np.maximum(abs(e),1e-12))
    if not np.isfinite(u).all():raise ValueError('nonfinite_terrain_completion')
    return u


def complete_terrain(report, arrays, prior_local, faces, cfg=TerrainCompletionConfig()):
    n=report['settings']['mesh_size'];R=report['diameter_used_m']/2;d=report['slopes']['inferred_depth_m']
    axis=np.asarray(prior_local)[:n,0]
    cells=observed_cells(arrays['points_local_m'],arrays['crater_mask']|arrays['ground_mask'],axis,R,d,report['terrain'],cfg)
    # Solve a smooth displacement on a coarser control grid, retaining the
    # original full-resolution triangulation and observation positions.
    m=min(n,cfg.solver_size)
    if m<5: raise ValueError('terrain_solver_grid_too_small')
    solve_axis=np.linspace(axis[0],axis[-1],m)
    A=bilinear_matrix(cells['xy'],solve_axis)
    output_A=bilinear_matrix(cells['xy'],axis)
    L=bending_matrix(m);penalty=(L.T@L)*((m-1)/(n-1))**2
    # Use the identical triangulation for surface export and validation baseline.
    # Small bilinear/analytic prior differences are accounted for in the target.
    target=cells['height']-output_A@prior_local[:,2]
    ix,iy=cells['grid'].T;block=cfg.validation_block_cells
    folds=(ix//block+2*(iy//block))%4
    cv=[]
    for lam in cfg.smoothness_candidates:
        err=np.zeros(len(target))
        for fold in range(4):
            train=folds!=fold;test=~train
            if train.sum()<30 or not test.any():raise ValueError('insufficient_spatial_holdout_cells')
            u=solve_correction(A[train],target[train],cells['weight'][train],penalty,lam,cfg)
            err[test]=A[test]@u-target[test]
        cv.append({'smoothness':lam,'holdout_mae_m':float(np.mean(abs(err))),
            'holdout_median_absolute_error_m':float(np.median(abs(err))),'holdout_rms_m':float(np.sqrt(np.mean(err**2)))})
    best=min(cv,key=lambda q:q['holdout_mae_m']);lam=best['smoothness']
    controls=solve_correction(A,target,cells['weight'],penalty,lam,cfg)
    u=bilinear_matrix(prior_local[:,:2],solve_axis)@controls
    top=np.array(prior_local,copy=True);top[:,2]+=u
    B=np.asarray(report['local_to_camera_basis_columns']);C=np.asarray(report['circle_center_camera_m'])
    camera=C+top@B.T
    fit=output_A@top[:,2]-cells['height']
    background=surface_height(top[:,:2]/R,report['terrain'])
    inside=np.linalg.norm(top[:,:2],axis=1)<=1.2*R
    deepest=int(np.argmax(np.where(inside,background-top[:,2],-np.inf)))
    baseline_mae=float(np.mean(abs(target)))
    metadata={'method':'observed_terrain_constrained_completion',
        'description':'Spatial median Depth constraints + smooth 2D displacement of hidden-surface paraboloid prior',
        'paper_original_algorithm':False,'rotational_symmetry_enforced':False,'circular_rim_enforced':False,
        'prior_center_depth_m':d,'inferred_depth_m':float(background[deepest]-top[deepest,2]),
        'depth_definition':'Maximum inferred depression relative to outside-ground trend within 1.2 bbox radii',
        'deepest_local_xyz_m':top[deepest].tolist(),'deepest_camera_xyz_m':camera[deepest].tolist(),
        'observed_cells':len(cells['xy']),'observed_cell_noise_m':cells['noise_m'],
        'aggregation_stride_mesh_cells':cells['aggregation_stride'],'aggregation_pitch_m':cells['aggregation_pitch_m'],
        'valid_depth_pixels_in_supported_cells_fraction':cells['pixel_coverage'],
        'observed_cell_fit_mae_m':float(np.mean(abs(fit))),'observed_cell_fit_rms_m':float(np.sqrt(np.mean(fit**2))),
        'prior_observed_cell_mae_m':baseline_mae,'prior_observed_cell_rms_m':float(np.sqrt(np.mean(target**2))),
        'selected_smoothness':lam,'spatial_validation':cv,'selected_holdout_mae_m':best['holdout_mae_m'],
        'solver_grid_size':m,'output_grid_size':n,
        'holdout_improved_over_prior':best['holdout_mae_m']<baseline_mae,
        'validation_definition':'4 spatial-block folds for displacement fit only; whole-frame geometry/prior kept fixed. Same-frame diagnostic, not independent depth ground truth',
        'correction_range_m':[float(u.min()),float(u.max())],'config':asdict(cfg),
        'physical_z_scale_applied':1.}
    diagnostics={'constraint_xy_m':cells['xy'],'constraint_height_m':cells['height'],
        'constraint_residual_m':fit,'constraint_count':cells['count'],'constraint_grid_xy':cells['grid'],
        'correction_m':u,'prior_local_m':prior_local,'validation_fold':folds}
    return top,camera,metadata,diagnostics
