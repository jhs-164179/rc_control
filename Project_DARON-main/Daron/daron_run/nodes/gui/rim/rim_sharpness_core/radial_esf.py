"""Relative image rim evidence from radial intensity / edge-spread profiles.

Edge width is motivated by Marziliano (2002); derivative-profile sampling by
Burns (2000); scale checks by Lindeberg (1998). The FWHM implementation,
candidate costs, thresholds and composite score are project adaptations, not
published crater calibration. Bright ridges need not be ideal step-edge ESFs.
This does not measure physical rim morphology, preservation grade or age.
"""
from __future__ import annotations
from dataclasses import asdict, dataclass
from functools import lru_cache
from typing import Sequence
import cv2
import numpy as np

ALGORITHM_VERSION = 'radial_esf_v2_20261007'


@lru_cache(maxsize=8)
def _sampling_geometry(n: int, sectors: int, radial_bins: int,
                       profile_min_r: float, rim_max_r: float):
    rb=np.linspace(profile_min_r,rim_max_r,radial_bins+1)
    centers=((rb[:-1]+rb[1:])/2).astype(np.float32); dr=float(rb[1]-rb[0])
    base=2*np.pi*(np.arange(sectors,dtype=np.float32)+.5)/sectors
    offsets=np.linspace(-.35,.35,5,dtype=np.float32)*(2*np.pi/sectors)
    angles=base[:,None,None]+offsets[None,:,None]
    radii=centers[None,None,:]*(n-1)/2
    map_x=((n-1)/2+radii*np.cos(angles)).astype(np.float32).reshape(sectors*5,radial_bins)
    map_y=((n-1)/2+radii*np.sin(angles)).astype(np.float32).reshape(sectors*5,radial_bins)
    return centers,dr,map_x,map_y


@dataclass(frozen=True)
class RadialESFConfig:
    normalized_size: int = 144
    sectors: int = 32
    # 72 bins are needed to resolve edge-spread width at the 144 px canonical
    # crop. 36 bins quantized mildly and sharply blurred synthetic rims to the
    # same FWHM and could reverse their ranking.
    radial_bins: int = 72
    profile_min_r: float = 0.28
    rim_min_r: float = 0.52
    rim_max_r: float = 0.96
    min_bbox_side_px: int = 20
    min_edge_snr: float = 2.5
    min_normalized_contrast: float = 0.06
    max_neighbour_radius_jump: float = 0.12
    max_multiscale_radius_shift: float = 0.08
    candidates_per_sector: int = 4


def _local_maxima(values: np.ndarray, indices: np.ndarray, limit: int) -> list[int]:
    peaks=indices[(values[indices]>=values[np.maximum(indices-1,0)]) &
                  (values[indices]>=values[np.minimum(indices+1,len(values)-1)])]
    return peaks[np.argsort(-values[peaks],kind='stable')[:limit]].tolist()


def _select_circular_path(candidate_sets: list[list[dict]], max_jump: float) -> list[dict|None]:
    """Exact cyclic DP for the specified unary + adjacent-pair objective.

All starting states are evaluated together, including the first unary cost.
Optimality refers to this discrete objective, not to anatomical rim identity.
"""
    if not candidate_sets: return []
    states=[c+[None] for c in candidate_sets]
    unaries=[np.asarray([-.85 if cur is None else float(cur['path_strength']) for cur in row]) for row in states]
    def transition(left,right):
        lr=np.asarray([r['rim_r'] for r in left[:-1]])
        rr=np.asarray([r['rim_r'] for r in right[:-1]])
        mat=np.full((len(left),len(right)),-.45,dtype=np.float64)
        mat[:-1,:-1]=-2.5*np.minimum(np.abs(lr[:,None]-rr[None,:])/max_jump,2.0)
        mat[-1,-1]=-.10
        return mat
    count=len(states[0]); scores=np.full((count,count),-np.inf)
    scores[np.arange(count),np.arange(count)]=unaries[0]
    backs=[]
    for s in range(1,len(states)):
        vals=scores[:,:,None]+transition(states[s-1],states[s])[None,:,:]
        back=np.argmax(vals,axis=1)
        scores=np.take_along_axis(vals,back[:,None,:],axis=1)[:,0,:]+unaries[s]
        backs.append(back)
    totals=scores+transition(states[-1],states[0]).T
    start_i,end_i=np.unravel_index(np.argmax(totals),totals.shape)
    ids=[int(end_i)]
    for back in reversed(backs): ids.append(int(back[start_i,ids[-1]]))
    ids.reverse()
    return [states[s][i] for s,i in enumerate(ids)]


@dataclass(frozen=True)
class RadialESFResult:
    status: str
    reason: str | None
    rim_sharpness_raw: float | None
    rim_completeness: float | None
    visible_arc_fraction: float | None
    rim_visibility: float | None
    median_edge_width_r: float | None
    median_edge_contrast: float | None
    median_edge_snr: float | None
    radius_consistency: float | None
    multiscale_stability: float | None
    accepted_sectors: int
    total_sectors: int
    observation_quality: float | None
    sector_measurements: tuple[dict, ...]
    bbox_xyxy: tuple[int, int, int, int]
    algorithm_version: str = ALGORITHM_VERSION
    local_edge_sharpness: float | None = None
    evidence_support: float | None = None
    width_censored_candidates: int = 0

    def to_dict(self): return asdict(self)


def _box(box, width, height):
    x1,y1,x2,y2=map(float,box)
    return max(0,int(np.floor(x1))),max(0,int(np.floor(y1))),min(width,int(np.ceil(x2))),min(height,int(np.ceil(y2)))


def _contiguous_fwhm(profile: np.ndarray, k: int, baseline: float) -> tuple[int,int]:
    half=baseline+0.5*max(0.0,float(profile[k])-baseline); left=right=k
    while left>0 and profile[left-1]>=half: left-=1
    while right+1<len(profile) and profile[right+1]>=half: right+=1
    return left,right


def _interpolated_fwhm(profile: np.ndarray, k: int, baseline: float) -> float | None:
    """Width in samples between linear half-height crossings.

Both crossings must be observed. Interpolation reduces bin quantization; it
does not recover spatial resolution lost in capture, resizing or smoothing.
"""
    if float(profile[k])<=baseline: return None
    left,right=_contiguous_fwhm(profile,k,baseline)
    if left==0 or right==len(profile)-1: return None
    half=baseline+.5*(float(profile[k])-baseline)
    xl=left-1+(half-float(profile[left-1]))/float(profile[left]-profile[left-1])
    xr=right+(float(profile[right])-half)/float(profile[right]-profile[right+1])
    return float(xr-xl)


def _connected_arc(accepted: list[dict], sectors: int, max_jump: float) -> tuple[list[dict],int]:
    """Remove isolated nodes and count only radius-compatible adjacent links."""
    by_sector={r['sector']:r for r in accepted}
    links=[s in by_sector and (s+1)%sectors in by_sector and
           abs(by_sector[s]['rim_r']-by_sector[(s+1)%sectors]['rim_r'])<=max_jump
           for s in range(sectors)]
    connected=[r for s,r in by_sector.items() if links[s] or links[(s-1)%sectors]]
    if not connected: return [],0
    longest=run=0
    for link in links+links:
        run=run+1 if link else 0
        longest=max(longest,run)
    return connected,min(sectors,longest+1)


def calculate_radial_esf_rim_sharpness(image: np.ndarray, box_xyxy: Sequence[float],
                                        config: RadialESFConfig|None=None) -> RadialESFResult:
    cfg=config or RadialESFConfig(); h,w=image.shape[:2]; box=_box(box_xyxy,w,h); x1,y1,x2,y2=box
    if x2-x1<cfg.min_bbox_side_px or y2-y1<cfg.min_bbox_side_px:
        return RadialESFResult('pending','bbox_too_small',None,None,None,None,None,None,None,None,None,0,cfg.sectors,None,(),box)
    crop=image[y1:y2,x1:x2]
    if crop.ndim==3: crop=cv2.cvtColor(crop,cv2.COLOR_BGR2GRAY)
    low,high=np.percentile(crop,[5,95]); span=float(high-low)
    if span<4:
        return RadialESFResult('pending','insufficient_image_contrast',None,None,None,None,None,None,None,None,None,0,cfg.sectors,0.0,(),box)
    n=cfg.normalized_size
    crop=cv2.resize(crop,(n,n),interpolation=cv2.INTER_CUBIC)
    p5,p95=np.percentile(crop,[5,95]); norm=np.clip((crop.astype(np.float32)-p5)/(p95-p5+1e-6),0,1)
    norm=cv2.GaussianBlur(norm,(0,0),1.0)
    quality=float(np.clip(np.percentile(cv2.magnitude(cv2.Sobel(norm,cv2.CV_32F,1,0),cv2.Sobel(norm,cv2.CV_32F,0,1)),90)/0.16,0,1))
    centers,dr,map_x,map_y=_sampling_geometry(n,cfg.sectors,cfg.radial_bins,cfg.profile_min_r,cfg.rim_max_r)
    # Five rays inside each angular sector are bilinearly sampled in one call.
    # Their median approximates the former wedge median while avoiding thousands
    # of full-image boolean masks per bbox.
    sampled=cv2.remap(norm,map_x,map_y,cv2.INTER_LINEAR,
                      borderMode=cv2.BORDER_REFLECT_101).reshape(cfg.sectors,5,cfg.radial_bins)
    profiles=np.median(sampled,axis=1).astype(np.float32)
    profiles=cv2.GaussianBlur(profiles,(5,1),.8)
    # Same float32 three-sample reductions as the per-candidate means, batched.
    three_means=np.mean(np.lib.stride_tricks.sliding_window_view(profiles,3,axis=1),axis=-1)
    derivatives=np.abs(np.gradient(profiles,dr,axis=1))
    coarse_profiles=cv2.GaussianBlur(profiles,(9,1),1.6)
    coarse_derivatives=np.abs(np.gradient(coarse_profiles,dr,axis=1))
    inner_mask=(centers>=.30)&(centers<cfg.rim_min_r)
    inner_all=derivatives[:,inner_mask]
    noise_medians=np.median(inner_all,axis=1)
    noise_mads=1.4826*np.median(np.abs(inner_all-noise_medians[:,None]),axis=1)+1e-4
    candidate_sets=[]; width_censored=0
    cand=np.flatnonzero((centers>=cfg.rim_min_r)&(centers<=cfg.rim_max_r))
    for s in range(cfg.sectors):
        prof=profiles[s]; deriv=derivatives[s]
        noise_med=float(noise_medians[s]); noise_mad=float(noise_mads[s])
        # Scale persistence is a heuristic: shadows and soil edges can also
        # persist. It is not proof that a candidate is the physical rim.
        coarse_deriv=coarse_derivatives[s]
        coarse_peaks=_local_maxima(coarse_deriv,cand,cfg.candidates_per_sector)
        choices=[]
        for k in _local_maxima(deriv,cand,cfg.candidates_per_sector):
            peak=float(deriv[k]); snr=(peak-noise_med)/noise_mad
            scale_shift=min((abs(float(centers[j]-centers[k])) for j in coarse_peaks),default=1.0)
            scale_stability=max(0.0,min(1.0,1.0-scale_shift/cfg.max_multiscale_radius_shift))
            l0=max(0,k-3); l1=min(len(prof),k+4)
            # Windows contain at most three samples after profile smoothing;
            # mean is equivalent in purpose and avoids hundreds of tiny-array
            # median allocations per frame.
            before=float(three_means[s,k-3]) if k>=3 else (float(np.mean(prof[l0:k])) if k>l0 else float(prof[k]))
            after=float(three_means[s,k+1]) if k+4<=len(prof) else (float(np.mean(prof[k+1:l1])) if l1>k+1 else float(prof[k]))
            contrast=abs(after-before)
            # Weak candidates remain available to the global path; final
            # acceptance still uses the stricter SNR/contrast/stability gates.
            if snr<1.25 or contrast<0.025 or scale_stability<0.20: continue
            width_samples=_interpolated_fwhm(deriv,k,noise_med)
            if width_samples is None:
                width_censored+=1
                continue
            width_r=max(dr,width_samples*dr)
            strength=.35*np.log1p(min(snr,30.0))+3.0*contrast+.5*scale_stability-1.5*width_r
            choices.append({'sector':s,'rim_r':float(centers[k]),'edge_width_r':float(width_r),
                            'contrast':float(contrast),'snr':float(snr),
                            'multiscale_stability':scale_stability,'path_strength':float(strength)})
        candidate_sets.append(choices)
    path=_select_circular_path(candidate_sets,cfg.max_neighbour_radius_jump)
    sectors=[]; accepted=[]
    for s,rec in enumerate(path):
        if rec is None: continue
        ok=bool(rec['snr']>=cfg.min_edge_snr and rec['contrast']>=cfg.min_normalized_contrast and rec['multiscale_stability']>=0.5)
        rec={**rec,'accepted':ok,'candidate_count':len(candidate_sets[s])}
        sectors.append(rec)
        if ok: accepted.append(rec)
    # A visible rim must form an arc. Remove isolated peaks and abrupt radial
    # jumps that are more likely soil texture or a bbox-border crossing.
    accepted,longest=_connected_arc(accepted,cfg.sectors,cfg.max_neighbour_radius_jump)
    completeness=len(accepted)/cfg.sectors
    arc_fraction=longest/cfg.sectors
    if not accepted:
        sectors=[{**r,'connected':False} for r in sectors]
        return RadialESFResult('valid',None,0.0,0.0,0.0,0.0,None,0.0,0.0,0.0,0.0,0,cfg.sectors,quality,tuple(sectors),box,
                               local_edge_sharpness=None,evidence_support=0.0,width_censored_candidates=width_censored)
    widths=np.asarray([r['edge_width_r'] for r in accepted]); contrasts=np.asarray([r['contrast'] for r in accepted]); snrs=np.asarray([r['snr'] for r in accepted])
    radii=np.asarray([r['rim_r'] for r in accepted])
    radius_mad=float(np.median(np.abs(radii-np.median(radii))))
    radius_consistency=float(np.exp(-radius_mad/0.08))
    scale_stability=float(np.median([r['multiscale_stability'] for r in accepted]))
    # Dimensionless relative evidence. Completeness and the longest connected
    # arc prevent isolated texture responses; radius and scale consistency
    # suppress irregular ground texture; contrast/width measures edge clarity.
    support=float(np.sqrt(completeness*arc_fraction*radius_consistency*scale_stability))
    visibility=float(support*np.median(contrasts))
    local_sharpness=float(np.median(contrasts/np.maximum(widths,dr)))
    raw=float(support*local_sharpness)
    connected_ids={r['sector'] for r in accepted}
    sectors=[{**r,'connected':r['sector'] in connected_ids} for r in sectors]
    return RadialESFResult('valid',None,raw,float(completeness),float(arc_fraction),visibility,float(np.median(widths)),
                           float(np.median(contrasts)),float(np.median(snrs)),radius_consistency,scale_stability,len(accepted),cfg.sectors,
                           quality,tuple(sectors),box,local_edge_sharpness=local_sharpness,
                           evidence_support=support,width_censored_candidates=width_censored)


def aggregate_pin_radial_esf(results: Sequence[RadialESFResult], min_observations: int = 3) -> dict:
    """Robustly combine repeated views of one mapped crater pin.

    The median limits single-frame shadow/texture failures.  Dispersion and
    observation quality are reported separately so downstream code can reject
    uncertain pins without changing the measured rim evidence.
    """
    valid=[r for r in results if r.status=='valid' and r.rim_sharpness_raw is not None]
    versions={getattr(r,'algorithm_version','radial_esf_v1') for r in valid}
    if len(versions)>1:
        return {'status':'pending','reason':'mixed_algorithm_versions','algorithm_versions':sorted(versions)}
    if len(valid)<min_observations:
        return {'status':'pending','reason':'insufficient_valid_observations',
                'valid_observations':len(valid),'required_observations':min_observations}
    raw=np.asarray([r.rim_sharpness_raw for r in valid],float)
    med=float(np.median(raw)); mad=float(1.4826*np.median(np.abs(raw-med)))
    relative_dispersion=float(mad/(abs(med)+1e-6))
    repeatability=float(np.exp(-relative_dispersion))
    quality=float(np.median([r.observation_quality or 0.0 for r in valid]))
    count_support=float(min(1.0,np.sqrt(len(valid)/5.0)))
    confidence=float(np.clip(repeatability*quality*count_support,0,1))
    return {
        'status':'valid','rim_sharpness_raw':med,
        'algorithm_version':next(iter(versions)),
        'rim_completeness':float(np.median([r.rim_completeness for r in valid])),
        'rim_visible_arc_fraction':float(np.median([r.visible_arc_fraction for r in valid])),
        'rim_visibility':float(np.median([r.rim_visibility for r in valid])),
        'rim_radius_consistency':float(np.median([r.radius_consistency for r in valid])),
        'rim_multiscale_stability':float(np.median([r.multiscale_stability for r in valid])),
        'valid_observations':len(valid),'raw_mad':mad,
        'repeatability':repeatability,'observation_quality':quality,
        'aggregation_confidence':confidence,
    }


def select_pin_representative_observation(observations: Sequence[dict]) -> dict:
    """Choose the CDA observation used for later rim calculation.

    Selection happens *before* rim analysis and therefore uses only detector
    metadata. Observations require ``confidence`` and may contain ``box``,
    ``stamp_ns`` and an image reference. A border-clipped detection can be
    marked with ``bbox_clipped=True`` and is excluded when any unclipped view
    exists. The highest detector confidence wins; bbox area breaks exact ties.
    """
    if not observations:
        return {'status':'pending','reason':'no_pin_observations'}
    indexed=[(i,o) for i,o in enumerate(observations)]
    unclipped=[item for item in indexed if not bool(item[1].get('bbox_clipped',False))]
    candidates=unclipped or indexed
    def area(o: dict) -> float:
        box=o.get('box') or o.get('bbox') or (0,0,0,0)
        if len(box)!=4: return 0.0
        return max(0.0,float(box[2])-float(box[0]))*max(0.0,float(box[3])-float(box[1]))
    i,o=max(candidates,key=lambda item:(float(item[1].get('confidence',item[1].get('score',0.0))),area(item[1])))
    return {'status':'valid','representative_index':i,
            'selection_rule':'highest_cda_confidence_then_bbox_area',
            'confidence':float(o.get('confidence',o.get('score',0.0))),
            'bbox_clipped':bool(o.get('bbox_clipped',False)),
            'observation':dict(o)}
