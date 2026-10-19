"""Existing radial rim localization + local Tan2024-type Lorentzian width.

Hybrid project method, NOT a full reproduction of a single paper.
Measurement reference: Tan et al., Bioengineering 11:614 (2024), section 2.2.4,
https://doi.org/10.3390/bioengineering11060614 . Sigmoid fitting is optional
there; here the existing denoised radial ESF is differentiated directly.

The existing DP, candidate locations and connected arcs are unchanged. Fit
only the selected derivative lobe; do not fit a sigmoid to an entire ridge.
The measured two-crossing width remains the primary score input. A fitted
Lorentzian width is a separate model-based measurement, never silently mixed
with empirical widths. An experimental replacement score is diagnostic only.
No extrapolated width or new rim location is accepted.
Canonical resized/smoothed image units: not raw optical PSF or physical depth.
"""
from __future__ import annotations
from dataclasses import dataclass

import cv2
import numpy as np

from .radial_esf import (
    RadialESFConfig, RadialESFResult, _sampling_geometry,
    _contiguous_fwhm, _interpolated_fwhm, calculate_radial_esf_rim_sharpness,
)

ALGORITHM_VERSION='radial_esf_hybrid_v2_dual_20261007'
PAPER_DOI='10.3390/bioengineering11060614'


@dataclass(frozen=True)
class HybridFitConfig:
    # Project safeguards, frozen before real-data comparison; not paper values.
    min_samples: int = 5
    max_window_half_bins: int = 18
    max_center_shift_bins: float = 1.
    max_nrmse: float = .15
    min_width_ratio: float = .5
    max_width_ratio: float = 2.
    grid_widths: int = 25
    grid_centers: int = 9
    refinement_rounds: int = 3


@dataclass(frozen=True)
class HybridESFResult(RadialESFResult):
    lorentzian_fitted_sectors: int = 0
    observed_only_sectors: int = 0
    lorentzian_fit_fraction: float = 0.
    lorentzian_median_width_r: float | None = None
    experimental_replacement_score: float | None = None
    baseline_rim_sharpness_raw: float | None = None
    width_status: str = 'pending'
    paper_doi: str = PAPER_DOI
    method_scope: str = 'project_hybrid_not_full_paper_reproduction'


def _best_lorentzian(x,y,log_widths,centers):
    """Variable projection: amplitude and baseline solved exactly for each Γ,x0.

Finite coarse-to-fine search; no unbounded iterative curve_fit. This solver
is project code. It fits the same Lorentzian family as the paper's Eq.(6).
"""
    gg,cc=np.meshgrid(np.exp(log_widths),centers,indexing='ij')
    gamma=gg.ravel();center=cc.ravel()
    f=.25*gamma[:,None]**2/((x[None,:]-center[:,None])**2+.25*gamma[:,None]**2)
    mf=np.mean(f,axis=1);my=float(np.mean(y))
    # Sufficient statistics avoid materializing centered/residual matrices for
    # every model. This is the same ordinary least-squares objective.
    den=np.einsum('ij,ij->i',f,f)/len(y)-mf*mf
    cov=np.einsum('ij,j->i',f,y)/len(y)-mf*my
    amp=cov/np.maximum(den,1e-14)
    baseline=my-amp*mf
    var_y=float(np.mean((y-my)**2))
    error=np.maximum(0.,var_y-cov*cov/np.maximum(den,1e-14))
    # Reject degenerate constant/negative-peak solutions, not just bad residuals.
    peak=max(float(np.max(y)),1e-12)
    feasible=(den>1e-12)&(amp>0)&(amp<3*peak)&(baseline>-peak)&(baseline<peak)
    error=np.where(feasible,error,np.inf)
    i=int(np.argmin(error))
    exact_error=float(np.mean((amp[i]*f[i]+baseline[i]-y)**2))
    return exact_error if feasible[i] else np.inf,np.array([gamma[i],center[i],amp[i],baseline[i]])


def fit_local_lorentzian(derivative,k,noise_baseline=0.,config=None,diagnostics=False):
    """Fit only a measured lobe, in radial-bin coordinates.

Both empirical half-height crossings must exist before any fitting. The
caller may use that empirical width if the model fit is rejected.
"""
    cfg=config or HybridFitConfig()
    y=np.asarray(derivative,dtype=float)
    if y.ndim!=1 or len(y)<3 or not 0<=k<len(y) or not np.all(np.isfinite(y)):
        return {'fit_status':'rejected','fit_reason':'invalid_profile','width_bins':None}
    observed=_interpolated_fwhm(y,k,noise_baseline)
    result={'fit_status':'rejected','fit_reason':None,'width_bins':None,
            'observed_width_bins':observed}
    if observed is None:
        return {**result,'fit_reason':'unobserved_halfheight_crossing'}
    half_left,half_right=_contiguous_fwhm(y,k,noise_baseline)
    extent=min(cfg.max_window_half_bins,max(4,int(np.ceil(2*observed))))
    left=right=k
    while left>max(0,k-extent) and y[left-1]<=y[left]:left-=1
    while right<min(len(y)-1,k+extent) and y[right+1]<=y[right]:right+=1
    if left>=half_left or right<=half_right:
        return {**result,'fit_reason':'multiple_or_truncated_lobe'}
    if right-left+1<cfg.min_samples:
        return {**result,'fit_reason':'too_few_lobe_samples'}
    x=np.arange(left,right+1,dtype=float)-k;values=y[left:right+1]
    low=max(.5,observed*.25);high=max(low*1.01,2*(right-left))
    hard_low,hard_high=np.log(low),np.log(high)
    lwlo,lwhi=hard_low,hard_high
    clo,chi=-cfg.max_center_shift_bins,cfg.max_center_shift_bins
    error=np.inf;params=None
    for _ in range(cfg.refinement_rounds):
        widths=np.linspace(lwlo,lwhi,cfg.grid_widths)
        centers=np.linspace(clo,chi,cfg.grid_centers)
        error,params=_best_lorentzian(x,values,widths,centers)
        if not np.isfinite(error): return {**result,'fit_reason':'degenerate_fit'}
        wg,cg=widths[1]-widths[0],centers[1]-centers[0]
        lg=np.log(params[0]);cc=params[1]
        lwlo,lwhi=max(hard_low,lg-wg),min(hard_high,lg+wg)
        clo,chi=max(-cfg.max_center_shift_bins,cc-cg),min(cfg.max_center_shift_bins,cc+cg)
    gamma,center,amp,baseline=map(float,params)
    nrmse=float(np.sqrt(error)/max(float(np.ptp(values)),1e-12))
    ratio=gamma/observed
    result.update(fitted_width_bins=gamma,fit_center_shift_bins=center,
                  fit_amplitude=amp,fit_baseline=baseline,fit_nrmse=nrmse,
                  fitted_observed_width_ratio=ratio,fit_window_bins=[left,right])
    if nrmse>cfg.max_nrmse: reason='poor_lorentzian_fit'
    elif not cfg.min_width_ratio<=ratio<=cfg.max_width_ratio: reason='width_disagrees_with_observation'
    elif center-gamma/2<x[0] or center+gamma/2>x[-1]: reason='fitted_crossings_outside_observed_lobe'
    elif abs(center)>=cfg.max_center_shift_bins-.001: reason='fit_center_at_bound'
    else: reason=None
    result.update(fit_status='accepted' if reason is None else 'rejected',
                  fit_reason=reason,width_bins=gamma if reason is None else None)
    if diagnostics:
        dense=np.linspace(x[0],x[-1],150)
        curve=amp*.25*gamma**2/((dense-center)**2+.25*gamma**2)+baseline
        result['diagnostics']={'x_bins':x.tolist(),'derivative':values.tolist(),
            'fit_x_bins':dense.tolist(),'fit_derivative':curve.tolist()}
    return result


def _profiles(image,box,cfg):
    x1,y1,x2,y2=box;crop=image[y1:y2,x1:x2]
    if crop.ndim==3:crop=cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY)
    n=cfg.normalized_size
    resized=cv2.resize(crop,(n,n),interpolation=cv2.INTER_CUBIC)
    lo,hi=np.percentile(resized,[5,95])
    norm=np.clip((resized.astype(np.float32)-lo)/(hi-lo+1e-6),0,1)
    norm=cv2.GaussianBlur(norm,(0,0),1.)
    r,dr,mx,my=_sampling_geometry(n,cfg.sectors,cfg.radial_bins,cfg.profile_min_r,cfg.rim_max_r)
    samples=cv2.remap(norm,mx,my,cv2.INTER_LINEAR,borderMode=cv2.BORDER_REFLECT_101)
    profiles=np.median(samples.reshape(cfg.sectors,5,cfg.radial_bins),axis=1).astype(np.float32)
    return r,dr,cv2.GaussianBlur(profiles,(5,1),.8)


def calculate_hybrid_rim_sharpness(image,box_xyxy,config=None,fit_config=None,diagnostics=False):
    """Keep established localization/score and add a paper-based local width.

Score semantics intentionally remain project-relative image evidence:
legacy support × median(legacy local contrast / observed width).
Lorentzian widths and fit counts are separate; model mismatch must not change
the primary score. The experimental replacement score is not recommended.
"""
    cfg=config or RadialESFConfig();fcfg=fit_config or HybridFitConfig()
    base=calculate_radial_esf_rim_sharpness(image,box_xyxy,cfg)
    data=base.to_dict()
    data.update(algorithm_version=ALGORITHM_VERSION,baseline_rim_sharpness_raw=base.rim_sharpness_raw)
    if base.status!='valid' or base.accepted_sectors==0:
        return HybridESFResult(**data)
    r,dr,profiles=_profiles(image,base.bbox_xyxy,cfg)
    derivatives=np.abs(np.gradient(profiles,dr,axis=1))
    inner=(r>=.30)&(r<cfg.rim_min_r)
    records=[];widths=[];model_widths=[];contrasts=[];fitted=0;observed_only=0
    for rec in base.sector_measurements:
        updated=dict(rec)
        if rec.get('connected'):
            s=rec['sector'];k=int(np.argmin(np.abs(r-rec['rim_r'])))
            baseline=float(np.median(derivatives[s,inner]))
            fit=fit_local_lorentzian(derivatives[s],k,baseline,fcfg,diagnostics)
            if fit['fit_status']=='accepted':
                width=max(dr,fit['width_bins']*dr);source='lorentzian_fit';fitted+=1
                model_widths.append(width)
            else:
                # The legacy candidate already requires two observed crossings.
                # Recheck before fallback rather than manufacture a value.
                empirical=_interpolated_fwhm(derivatives[s],k,baseline)
                if empirical is None:raise RuntimeError('legacy observed-width invariant failed')
                width=rec['edge_width_r'];source='not_available';observed_only+=1
            updated.update(observed_edge_width_r=rec['edge_width_r'],
                           lorentzian_edge_width_r=float(width) if source=='lorentzian_fit' else None,
                           width_source='observed_fwhm',model_width_source=source,lorentzian_fit=fit)
            widths.append(width);contrasts.append(rec['contrast'])
        else:
            updated.update(width_source='not_measured')
        records.append(updated)
    local=float(np.median(np.asarray(contrasts)/np.maximum(widths,dr)))
    data.update(sector_measurements=tuple(records),
                lorentzian_median_width_r=float(np.median(model_widths)) if model_widths else None,
                experimental_replacement_score=float(base.evidence_support*local),
                lorentzian_fitted_sectors=fitted,observed_only_sectors=observed_only,
                lorentzian_fit_fraction=fitted/(fitted+observed_only),width_status='measured')
    return HybridESFResult(**data)
