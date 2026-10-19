"""Standard-library bridge from GUI/SLAM measurements into crater_rank.

The caller supplies persistent physical IDs and acquisition/support metadata.
This module deliberately does not infer that two similar images are the same
crater. Nor does it declare lighting/physical wall support verified by itself.
"""
import math


def view_cell(incidence_deg,bbox_short_side_px,exposure_profile):
    """Initial 15-degree incidence / native-pixel bins; version in protocol_id.

    incidence_deg is the ray-to-local-ground-normal angle (not camera pitch).
    No hard maximum depth: image/depth quality determines usable distance.
    The bin widths are tunable acquisition controls, not physical thresholds.
    """
    if not isinstance(exposure_profile,str) or not exposure_profile.strip():
        raise ValueError('verified exposure/lighting profile required')
    if not math.isfinite(incidence_deg) or not 0<=incidence_deg<90:
        raise ValueError('incidence must be degrees from local normal, 0<=x<90')
    if not math.isfinite(bbox_short_side_px) or bbox_short_side_px<=0:
        raise ValueError('native bbox pixel size required')
    bounds=(32,48,64,96,128,192,256)
    low=max([b for b in bounds if bbox_short_side_px>=b],default=0)
    return 'inc%d_px%d_exp%s'%(int(incidence_deg//15)*15,low,exposure_profile)


def make_observation(*,mission_id,frame_id,crater_id,timestamp_s,protocol_id,
                     comparison_group,view_cell_id,support_id,geometry,rim,
                     quality,identity_confirmed=False,acquisition_verified=False,image=None):
    """geometry from measure_observed; rim from analyze_representative_crop.

    quality must contain native pixel size, valid-depth fraction, sync error,
    plane/quadratic P95 difference, saturation fraction and truncation state.
    These cannot be obtained from a completed mesh or an upscaled GUI preview.
    """
    slopes=geometry.get('slopes',{})
    q=dict(quality)
    q['wall_sectors']=len(slopes.get('valid_sectors',[]))
    windows=slopes.get('windows')
    q['wall_windows']=(sum(w.get('rise_m',0)>0 for w in windows)
                       if isinstance(windows,list) else None)
    q['wall_windows_definition']='positive_rising_windows'
    return dict(mission_id=str(mission_id),observation_id=str(frame_id)+':'+str(crater_id),
                crater_id=str(crater_id),timestamp_s=float(timestamp_s),protocol_id=protocol_id,
                comparison_group=comparison_group,view_cell=view_cell_id,support_id=support_id,
                slope_source='observed_depth',slope_deg=slopes.get('angle_p95_deg'),
                raw_max_slope_deg=slopes.get('observed_max_angle_deg'),
                rim_score=rim.get('rim_sharpness_raw') if rim.get('status')=='valid' else None,
                rim_status=rim.get('status'),quality=q,
                identity_confirmed=identity_confirmed,acquisition_verified=acquisition_verified,
                image=image,geometry_flags=geometry.get('quality_flags'))
