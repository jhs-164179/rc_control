"""GUI adapter for an already selected bbox crop; no ROS or pin selection.

Primary score uses observed width. Lorentzian width is a separate result.
Overlay: blue = model width available, amber = observed only, red = rejected.
"""
from __future__ import annotations
import argparse,json,sys
from pathlib import Path
from typing import Literal
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parent
if str(ROOT) not in sys.path:sys.path.insert(0,str(ROOT))
from rim_sharpness_core.hybrid import calculate_hybrid_rim_sharpness,ALGORITHM_VERSION
from rim_sharpness_core.radial_esf import RadialESFConfig

ColorOrder=Literal['bgr','rgb','gray']
COLORS_BGR={'model_width':(198,122,22),'observed_only':(78,192,255),'rejected':(85,77,255)}


def _validate(image,color_order):
    if not isinstance(image,np.ndarray) or image.dtype!=np.uint8:
        raise TypeError('representative_crop must be a uint8 NumPy image')
    if image.ndim not in (2,3) or not image.shape[0] or not image.shape[1]:
        raise ValueError('representative_crop must be a nonempty HxW or HxWx3 image')
    if image.ndim==2:
        if color_order!='gray':raise ValueError("2D images require color_order='gray'")
        return np.ascontiguousarray(image)
    if image.shape[2]!=3 or color_order not in ('bgr','rgb'):
        raise ValueError("HxWx3 images require color_order='bgr' or 'rgb'")
    return cv2.cvtColor(image,cv2.COLOR_BGR2GRAY if color_order=='bgr' else cv2.COLOR_RGB2GRAY)


def marker_kind(record):
    if not record.get('connected',False):return 'rejected'
    return 'model_width' if record.get('model_width_source')=='lorentzian_fit' else 'observed_only'


def _draw_round_dot(image,center,radius,color):
    """Blend a subpixel circular disc without the tiny cv2.circle cross shape."""
    x,y=center;h,w=image.shape[:2]
    x0=max(0,int(np.floor(x-radius-1)));x1=min(w,int(np.ceil(x+radius+1))+1)
    y0=max(0,int(np.floor(y-radius-1)));y1=min(h,int(np.ceil(y+radius+1))+1)
    if x0>=x1 or y0>=y1:return
    yy,xx=np.mgrid[y0:y1,x0:x1]
    # Analytic distance coverage gives a smooth, rotationally symmetric edge.
    alpha=np.clip(radius+.5-np.hypot(xx-x,yy-y),0.,1.)[...,None]
    roi=image[y0:y1,x0:x1]
    roi[:]=np.rint(roi*(1-alpha)+np.asarray(color)*alpha).astype(np.uint8)


def render_rim_overlay(original,data,*,color_order:ColorOrder='bgr',output_size=None):
    """Render at the final display size (width, height), then draw round dots.

Use this when a GUI resizes its preview: do not enlarge an already marked
low-resolution crop. With output_size=None the input size is preserved.
"""
    _validate(original,color_order)
    if original.ndim==2:image=cv2.cvtColor(original,cv2.COLOR_GRAY2BGR)
    elif color_order=='rgb':image=cv2.cvtColor(original,cv2.COLOR_RGB2BGR)
    else:image=np.ascontiguousarray(original).copy()
    src_h,src_w=image.shape[:2]
    if output_size is not None:
        if len(output_size)!=2 or any(not isinstance(v,(int,np.integer)) or v<=0 for v in output_size):
            raise ValueError('output_size must be positive integer (width, height)')
        w,h=map(int,output_size)
        interpolation=cv2.INTER_LINEAR if w>=src_w and h>=src_h else cv2.INTER_AREA
        image=cv2.resize(image,(w,h),interpolation=interpolation)
    h,w=image.shape[:2];n=RadialESFConfig().normalized_size;c=(n-1)/2
    radius=max(2.,2.*min(w/src_w,h/src_h))
    for rec in data['sector_measurements']:
        angle=2*np.pi*(rec['sector']+.5)/data['total_sectors']
        # Map canonical pixel centers back to the unresized crop.
        u=c+c*rec['rim_r']*np.cos(angle);v=c+c*rec['rim_r']*np.sin(angle)
        xy=((u+.5)*w/n-.5,(v+.5)*h/n-.5)
        _draw_round_dot(image,xy,radius,COLORS_BGR[marker_kind(rec)])
    return image


def analyze_representative_crop(representative_crop:np.ndarray,*,color_order:ColorOrder='bgr',
                                include_overlay:bool=False,include_diagnostics:bool=False,
                                overlay_size=None)->tuple[dict,np.ndarray|None]:
    """Preserves the prior GUI call signature; returns JSON-safe data and BGR overlay.

The caller owns representative selection, bbox clipping, timestamp and pin ID.
Width unit is canonical bbox radius, not physical length or native pixel width.
"""
    gray=_validate(representative_crop,color_order);h,w=gray.shape
    result=calculate_hybrid_rim_sharpness(gray,(0,0,w,h),diagnostics=include_diagnostics)
    data=result.to_dict()
    data['crop_bbox_xyxy']=data.pop('bbox_xyxy')
    # These two fields are unadopted experiment diagnostics, not GUI metrics.
    data.pop('experimental_replacement_score',None);data.pop('baseline_rim_sharpness_raw',None)
    data.update(algorithm=ALGORITHM_VERSION,input_width=w,input_height=h,
        width_unit='normalized_bbox_radius',rim_identity_verified=False,
        width_reason=(data['reason'] or 'no_connected_rim_candidates') if data['width_status']=='pending' else None)
    model_available=data['lorentzian_median_width_r'] is not None
    data['model_width_status']='measured' if model_available else 'pending'
    data['model_width_reason']=None if model_available else (data['width_reason'] or 'no_accepted_lorentzian_fit')
    data['rejected_selected_sectors']=sum(not q['connected'] for q in data['sector_measurements'])
    data['no_selected_candidate_sectors']=data['total_sectors']-len(data['sector_measurements'])
    overlay=render_rim_overlay(representative_crop,data,color_order=color_order,output_size=overlay_size) if include_overlay else None
    return data,overlay


def read_image(path):
    # Preserve the former read_image helper's BGR contract.
    image=cv2.imdecode(np.fromfile(path,np.uint8),cv2.IMREAD_COLOR)
    if image is None:raise FileNotFoundError(path)
    return image


def save_image(path,image):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    ok,encoded=cv2.imencode(path.suffix,image)
    if not ok:raise ValueError(f'Cannot encode {path}')
    encoded.tofile(path)


def main():
    parser=argparse.ArgumentParser(description='Paper-based hybrid rim measurement for a selected bbox crop')
    parser.add_argument('--image',type=Path,required=True,help='Already cropped representative image')
    parser.add_argument('--json',dest='json_path',type=Path)
    parser.add_argument('--overlay',type=Path)
    parser.add_argument('--overlay-scale',type=int,default=1,help='Resize background before drawing dots; e.g. 4 for a larger preview')
    parser.add_argument('--diagnostics',action='store_true')
    args=parser.parse_args()
    if args.overlay_scale<1:parser.error('--overlay-scale must be at least 1')
    source=read_image(args.image);h,w=source.shape[:2]
    data,overlay=analyze_representative_crop(source,include_overlay=bool(args.overlay),include_diagnostics=args.diagnostics,
                                           overlay_size=(w*args.overlay_scale,h*args.overlay_scale))
    payload=json.dumps(data,ensure_ascii=False,indent=2,allow_nan=False)
    if args.json_path:
        args.json_path.parent.mkdir(parents=True,exist_ok=True);args.json_path.write_text(payload,encoding='utf-8')
    if args.overlay:save_image(args.overlay,overlay)
    print(payload)


if __name__=='__main__':main()
