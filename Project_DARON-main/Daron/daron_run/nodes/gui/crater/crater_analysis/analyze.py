"""Portable entry point: observed maximum wall slope, estimated depth, full mesh."""
from __future__ import annotations
import argparse
import csv
import json
from pathlib import Path
import numpy as np

from .measurement import Camera,Box,read_depth,reconstruct,save_geometry
from .completion import save_completion,render_completion
from .complete_viewer import write_complete_viewer
from .core import surface_height
from .geometry import GeometryConfig
from .acceleration import configure_backend, backend_status, warmup


def result_summary(report,completion):
    """Do not substitute the old paraboloid rim angle for observed wall slope."""
    s=report['slopes'];depth=float(completion['inferred_depth_m']);diam=float(report['diameter_used_m'])
    flags=list(dict.fromkeys(completion.get('quality_flags',report['quality_flags'])))
    return {
        'status':'completed','frame_id':report.get('frame'),
        'observed_max_inner_wall_slope_deg':float(s['observed_max_angle_deg']),
        'observed_inner_wall_slope_p95_deg':float(s['angle_p95_deg']),
        'slope_angle_formula':s.get('angle_formula','atan(height change / radial distance)'),
        'slope_measurement_quality':'review' if any(f in flags for f in
            ('wall_rise_comparable_to_background_residual','sensitive_to_background_model','partial_direction_coverage')) else 'supported',
        'geometry_method':report.get('geometry_method',report['diameter_source']),
        'geometry_settings':report.get('geometry_settings'),
        'observed_range_m':report.get('range_m'),
        'terrain_normal_at_crater_center_camera':report.get('terrain_normal_at_crater_center_camera'),
        'slope_definition':'Maximum terrain-corrected radial slope among valid observed wall windows; excludes hidden wall',
        'slope_support':{'sector_index':s['observed_max_sector'],
            'window_start_radius_fraction':s['max_window_start_r'],
            'window_end_radius_fraction':s['max_window_end_r'],
            'window_rise_m':s['max_window_rise_m'],'window_baseline_m':s['max_window_baseline_m'],
            'observed_radial_gradient':s.get('max_observed_gradient'),
            'local_ground_radial_gradient':s.get('max_background_gradient'),
            'window_observed_rise_m':s.get('max_observed_gradient',s['max_window_rise_m']/s['max_window_baseline_m'])*s['max_window_baseline_m'],
            'window_ground_rise_m':s.get('max_background_gradient',0.)*s['max_window_baseline_m'],
            'supported_sector_count':len(s['valid_sectors']),
            'total_sector_count':report['settings']['angular_sectors']},
        'estimated_depth_m':depth,'estimated_depth_mm':depth*1000,
        'depth_definition':completion.get('depth_definition','Paraboloid center depth relative to outside-ground trend'),
        'depth_is_model_estimated':True,'diameter_m':diam,
        'diameter_source':report['diameter_source'],
        'depth_to_diameter_ratio':depth/diam,
        'diameter_to_depth_ratio':diam/depth if depth>0 else None,
        'physical_z_scale_applied':1.,
        'coordinate_frame':'Camera optical XYZ in metres; local ground frame also exported; no gravity/SLAM pose applied',
        'completion_method':completion.get('method'),
        'ground_surface_degree':report['terrain']['degree'],
        'ground_reference_rms_m':report['terrain']['selected_ground_rms_m'],
        'quality_flags':flags,
        'mesh':{'camera_obj':'completed_surface_camera.obj','local_obj':'completed_surface_local.obj',
            'closed_camera_obj':'completed_solid_camera.obj','closed_local_stl':'completed_solid_local.stl',
            'viewer':'complete_mesh_viewer.html','arrays':'completed_mesh.npz',
            'vertex_heights':'vertex_heights.csv',
            'top_vertices':completion['surface_topology']['vertices'],
            'top_triangles':completion['surface_topology']['triangles'],
            'solid_watertight':completion['solid_topology']['closed']},
        'initial_paraboloid_diagnostics':{'center_depth_m':s['inferred_depth_m'],
            'rim_angle_deg':s['inferred_rim_max_angle_deg'],
            'note':'Initial model parameters only; rim angle is not the observed maximum wall slope or completed-mesh maximum slope'}
    }


def save_result(result,report,out):
    out=Path(out)
    (out/'result.json').write_text(json.dumps(result,ensure_ascii=False,indent=2,allow_nan=False),encoding='utf-8')
    keys=['frame_id','observed_max_inner_wall_slope_deg','estimated_depth_m','estimated_depth_mm',
        'diameter_m','depth_to_diameter_ratio','diameter_to_depth_ratio','depth_is_model_estimated',
        'completion_method','physical_z_scale_applied']
    with (out/'result.csv').open('w',encoding='utf-8-sig',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=keys);writer.writeheader();writer.writerow({k:result[k] for k in keys})
    with np.load(out/'completed_mesh.npz') as data:
        local=data['top_local_m'];camera=data['top_camera_m'];near=data['near_observed_support'].astype(int)
    bg=surface_height(local[:,:2]/(result['diameter_m']/2),report['terrain'])
    table=np.c_[np.arange(len(local)),local,camera,bg,local[:,2]-bg,near]
    np.savetxt(out/'vertex_heights.csv',table,delimiter=',',comments='',
        header='vertex_id,local_x_m,local_y_m,local_z_m,camera_x_m,camera_y_m,camera_z_m,ground_reference_height_m,height_relative_to_ground_m,near_observed_support',
        fmt=['%d']+['%.9f']*8+['%d'])
    text=(f"관측 기반 최대 내벽 경사: {result['observed_max_inner_wall_slope_deg']:.3f} 도\n"
          f"추정 최대 함몰 깊이: {result['estimated_depth_mm']:.3f} mm\n"
          f"기준 직경: {result['diameter_m']*1000:.3f} mm\n"
          f"깊이/직경: {result['depth_to_diameter_ratio']:.6f}\n"
          f"직경/깊이: {result['diameter_to_depth_ratio']:.6f}\n\n"
          "경사는 주변 지면의 기울기·굴곡을 보정한 관측·분석 구간의 최댓값입니다.\n"
          "깊이는 가려진 영역을 포함한 완성 표면과 주변 기준 지면의 최대 차이로, 모델 추정값입니다.\n"
          "완성 표면의 전체 최대 경사와 초기 포물면의 림 각도는 관측 최대 경사와 다른 값입니다.\n"
          "물리적 Z 배율: 1 (시험장 높이에 추가 증폭·역보정 없음).\n"
          "3D 보기: complete_mesh_viewer.html\n")
    (out/'result_ko.txt').write_text(text,encoding='utf-8-sig')


def analyze_crater(depth_m,camera,boxes,box_index,output,*,frame_id=None,image=None,
                   terrain='auto',diameter_m=None,preview=True,input_depth=None,geometry_mode='supported',
                   geometry_config=None):
    """Use aligned optical-axis Z in metres; return the three requested outputs.

    boxes: all crater boxes in this image, so other craters are excluded from
    ground estimation. Returns a JSON-serialisable result and writes artifacts.
    No CDA network, ROS, GPU, or image-generation service is required.
    """
    out=Path(output);out.mkdir(parents=True,exist_ok=True)
    if not boxes or box_index<0 or box_index>=len(boxes):raise ValueError('box_index_out_of_range')
    depth_m=np.asarray(depth_m,dtype=float)
    if depth_m.shape!=(camera.height,camera.width):raise ValueError('aligned_depth_dimensions_do_not_match_intrinsics')
    report,arrays=reconstruct(depth_m,camera,boxes,box_index,terrain_mode=terrain,diameter_m=diameter_m,
        geometry_mode=geometry_mode,geometry_config=geometry_config)
    report.update(frame=frame_id,bbox_source='input_CDA_or_user_boxes',
        input_image=str(Path(image).resolve()) if image else None,
        input_depth=str(Path(input_depth).resolve()) if input_depth else 'in_memory_optical_Z_metres',
        camera={'width':camera.width,'height':camera.height,'K':camera.K.tolist()},
        physical_z_scale_applied=1.)
    save_geometry(report,arrays,out)
    *_,completion=save_completion(report,arrays,out,shape='observed-terrain')
    write_complete_viewer(report,arrays,out)
    result=result_summary(report,completion);save_result(result,report,out)
    if preview:render_completion(report,arrays,out)
    (out/'acceleration.json').write_text(json.dumps(backend_status(),indent=2),encoding='utf-8')
    return result


def main(argv=None):
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--depth',type=Path,required=True,help='Aligned uint16 raw Depth PNG; not a colour depth image')
    p.add_argument('--camera-json',type=Path,required=True,help='width, height, K for this aligned image')
    g=p.add_mutually_exclusive_group(required=True)
    g.add_argument('--boxes-json',type=Path,help='All bbox coordinates [[left,top,right,bottom],...]')
    g.add_argument('--bbox',type=float,nargs=4,help='One bbox in aligned image pixels')
    p.add_argument('--box-index',type=int,default=0)
    p.add_argument('--image',type=Path,help='Aligned IR/colour image for the comparison preview only')
    p.add_argument('--depth-scale',type=float,default=.001,help='Metres per raw depth count')
    p.add_argument('--invalid-raw-min',type=int,default=60000,help='Raw counts >= this value are invalid; 0 disables this sentinel rule')
    p.add_argument('--min-depth-m',type=float,default=.07)
    p.add_argument('--max-depth-m',type=float,default=None,help='Optional calibrated sensor limit in metres; default has no upper distance cutoff')
    p.add_argument('--geometry-config-json',type=Path,help='Optional GeometryConfig overrides, including sensor-specific plane noise and diameter limits')
    p.add_argument('--frame-id',type=int)
    p.add_argument('--diameter-m',type=float,help='Optional SLAM diameter override; bbox/depth still determine position')
    p.add_argument('--terrain',choices=['auto','plane','quadratic'],default='auto')
    p.add_argument('--geometry-mode',choices=['supported','strict'],default='supported')
    p.add_argument('--output',type=Path,default=Path('results'))
    p.add_argument('--no-preview',action='store_true',help='Skip PNG rendering; keep mesh, HTML, JSON and CSV')
    p.add_argument('--backend',choices=['auto','numpy','numba'],default=None,
                   help='Optional CPU backend; default follows CRATER_BACKEND or auto')
    p.add_argument('--warmup',action='store_true',help='Warm JIT kernels before loading input')
    args=p.parse_args(argv)
    if args.backend is not None:configure_backend(args.backend)
    if args.warmup:warmup()
    if not np.isfinite(args.depth_scale) or args.depth_scale<=0:p.error('--depth-scale must be finite and positive')
    try:
        geometry_values={'min_depth_m':args.min_depth_m,'max_depth_m':args.max_depth_m,'max_ground_slope_deg':60.}
        if args.geometry_config_json:
            geometry_values.update(json.loads(args.geometry_config_json.read_text(encoding='utf-8-sig')))
        geometry=GeometryConfig(**geometry_values)
        info=json.loads(args.camera_json.read_text(encoding='utf-8-sig'))
        camera=Camera(int(info['width']),int(info['height']),np.asarray(info['K'],dtype=float))
        coords=json.loads(args.boxes_json.read_text(encoding='utf-8-sig')) if args.boxes_json else [args.bbox]
        boxes=[Box(*map(float,b)) for b in coords]
        if not all(np.isfinite(b.xyxy).all() and b.right>b.left and b.bottom>b.top for b in boxes):
            raise ValueError('bbox_must_be_finite_left_top_right_bottom')
        result=analyze_crater(read_depth(args.depth,args.depth_scale,camera,args.invalid_raw_min),camera,boxes,args.box_index,args.output,
            frame_id=args.frame_id,image=args.image,terrain=args.terrain,diameter_m=args.diameter_m,
            preview=not args.no_preview,input_depth=args.depth,geometry_mode=args.geometry_mode,geometry_config=geometry)
    except (ValueError,OSError,KeyError,TypeError,np.linalg.LinAlgError) as error:
        args.output.mkdir(parents=True,exist_ok=True)
        failure={'status':'rejected','reason':str(error),'frame_id':args.frame_id}
        (args.output/'rejection.json').write_text(json.dumps(failure,ensure_ascii=False,indent=2),encoding='utf-8')
        # Overwrite result.json so an earlier success cannot be mistaken for this attempt.
        (args.output/'result.json').write_text(json.dumps(failure,ensure_ascii=False,indent=2),encoding='utf-8')
        with (args.output/'result.csv').open('w',encoding='utf-8-sig',newline='') as f:
            writer=csv.DictWriter(f,fieldnames=list(failure));writer.writeheader();writer.writerow(failure)
        (args.output/'result_ko.txt').write_text('계산 거부: '+str(error)+'\n이번 실행의 유효한 측정값은 없습니다. 기존 mesh가 남아 있어도 이번 실행 결과가 아닙니다.\n',encoding='utf-8-sig')
        print(json.dumps(failure,ensure_ascii=True));return 2
    print(json.dumps({k:result[k] for k in ['status','frame_id','observed_max_inner_wall_slope_deg',
        'estimated_depth_mm','diameter_m','depth_to_diameter_ratio']},ensure_ascii=True,indent=2))
    return 0


if __name__=='__main__':raise SystemExit(main())
