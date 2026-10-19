"""Choose among completed observations of ONE already-associated crater."""
import math

REVIEW_FLAGS=frozenset(('wall_rise_comparable_to_background_residual',
                       'sensitive_to_background_model','partial_direction_coverage'))

def measurement_priority(result):
    """Prefer supported slopes; retain a reviewed mesh if no stronger view exists."""
    supported=result.get('slope_measurement_quality')=='supported'
    flags=len(set(result.get('quality_flags',()))&REVIEW_FLAGS)
    rms=result.get('ground_reference_rms_m')
    rms=float(rms) if rms is not None and math.isfinite(float(rms)) else math.inf
    return (not supported,flags,rms,-float(result.get('representative_score',0)))

def best_completed(results):
    return min(results,key=measurement_priority) if results else None
