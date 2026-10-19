"""Complete crater surface and watertight export from measured model parameters.

The initial top is the Zhou Eq.6 zero-raised-rim limit used by reconstruct().
By default observed directional terrain constrains its final non-axisymmetric
shape. An artificial base closes the tile; it is NOT measured subsurface data.
"""
from __future__ import annotations
import argparse
import json
from pathlib import Path
import struct

import numpy as np

from .core import Config, make_mesh, surface_height, write_obj
from .acceleration import accelerate


def close_terrain_tile(top, top_faces, size, diameter_m):
    """Add outward-oriented side walls and a flat base beneath the whole top."""
    top=np.asarray(top,dtype=float);top_faces=np.asarray(top_faces,dtype=int)
    if top.shape!=(size*size,3) or not np.isfinite(top).all():
        raise ValueError('invalid_complete_surface_grid')
    ids=np.arange(size*size).reshape(size,size)
    boundary=np.r_[ids[0,:],ids[1:,-1],ids[-1,-2::-1],ids[-2:0:-1,0]]
    base_z=float(top[:,2].min()-max(.005,.08*diameter_m))
    base=top[boundary].copy();base[:,2]=base_z
    base_center=np.r_[base[:,:2].mean(axis=0),base_z]
    vertices=np.vstack((top,base,base_center));faces=top_faces.tolist()
    n=len(top);middle=len(vertices)-1
    for i,a in enumerate(boundary):
        j=(i+1)%len(boundary);b=int(boundary[j]);a=int(a)
        faces.extend([[b,a,n+i],[b,n+i,n+j],[middle,n+j,n+i]])
    return vertices,np.asarray(faces,dtype=int),base_z


def topology_check(vertices,faces,closed=False):
    v=np.asarray(vertices);f=np.asarray(faces)
    if not np.isfinite(v).all() or f.min()<0 or f.max()>=len(v):
        raise ValueError('invalid_mesh_numbers')
    cross=np.cross(v[f[:,1]]-v[f[:,0]],v[f[:,2]]-v[f[:,0]])
    if np.any(np.linalg.norm(cross,axis=1)<=1e-14):
        raise ValueError('degenerate_mesh_triangle')
    directed=np.concatenate((f[:,[0,1]],f[:,[1,2]],f[:,[2,0]]))
    edges,count=np.unique(np.sort(directed,axis=1),axis=0,return_counts=True)
    boundary=edges[count==1]
    if np.any(count>2):raise ValueError('nonmanifold_mesh_edge')
    components = accelerate('component_count', len(v), edges, np.unique(f))
    if components is None:
        parent=np.arange(len(v))
        def find(a):
            while parent[a]!=a:
                parent[a]=parent[parent[a]];a=parent[a]
            return a
        for a,b in edges:
            pa,pb=find(a),find(b)
            if pa!=pb:parent[pa]=pb
        components=len({find(int(i)) for i in np.unique(f)})
    volume=float(np.sum(v[f[:,0]]*np.cross(v[f[:,1]],v[f[:,2]]))/6)
    # Every shared edge must appear once in each direction.
    signed=np.where(directed[:,0]<directed[:,1],1,-1)
    _,inv=np.unique(np.sort(directed,axis=1),axis=0,return_inverse=True)
    orientation=np.bincount(inv,weights=signed)
    if np.any(abs(orientation[count==2])>0):raise ValueError('inconsistent_face_winding')
    if closed and (len(boundary) or components!=1 or volume<=0):
        raise ValueError('mesh_is_not_one_closed_outward_solid')
    return {'vertices':len(v),'triangles':len(f),'boundary_edges':len(boundary),
            'connected_components':components,'euler_characteristic':len(v)-len(edges)+len(f),
            'closed':bool(len(boundary)==0),'nondegenerate':True,'consistent_winding':True,
            'signed_volume_m3':volume if closed else None}


def write_stl(path,vertices,faces):
    tri=np.asarray(vertices)[faces];normal=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0])
    normal/=np.linalg.norm(normal,axis=1)[:,None]
    with open(path,'wb') as out:
        out.write(b'Crater completion, metres; artificial export base, not subsurface data'.ljust(80,b' '))
        out.write(struct.pack('<I',len(faces)))
        for n,t in zip(normal,tri):out.write(struct.pack('<12fH',*n,*t.ravel(),0))


def save_completion(report,arrays,out,shape='observed-terrain'):
    """Fill hidden surface while fitting observed terrain; never normalise Z."""
    out=Path(out);out.mkdir(parents=True,exist_ok=True)
    cfg=Config(**report['settings']);B=np.asarray(report['local_to_camera_basis_columns'])
    C=np.asarray(report['circle_center_camera_m']);R=report['diameter_used_m']/2
    d=report['slopes']['inferred_depth_m']
    top,camera,faces,depression=make_mesh(R,d,report['terrain'],C,B[:,:2],B[:,2],cfg)
    terrain_metadata=None;diagnostics={}
    if shape=='observed-terrain':
        from .terrain_completion import complete_terrain
        top,camera,terrain_metadata,diagnostics=complete_terrain(report,arrays,top,faces)
    elif shape!='paraboloid':raise ValueError('unknown_completion_shape')
    qa_top=topology_check(top,faces)
    if qa_top['boundary_edges']!=4*(cfg.mesh_size-1) or qa_top['euler_characteristic']!=1:
        raise ValueError('unexpected_hole_in_completed_top')
    solid,solid_faces,base_z=close_terrain_tile(top,faces,cfg.mesh_size,2*R)
    solid_camera=C+solid@B.T;qa_solid=topology_check(solid,solid_faces,closed=True)
    # Every mesh row/column covers the entire crater, including the unseen floor.
    mid=cfg.mesh_size**2//2
    if shape=='paraboloid':
        assert abs(top[mid,2]-(surface_height([[0,0]],report['terrain'])[0]-d))<1e-12
    write_obj(out/'completed_surface_local.obj',top,faces)
    write_obj(out/'completed_surface_camera.obj',camera,faces)
    write_obj(out/'completed_solid_local.obj',solid,solid_faces)
    write_obj(out/'completed_solid_camera.obj',solid_camera,solid_faces)
    write_stl(out/'completed_solid_local.stl',solid,solid_faces)
    xy=arrays['points_local_m'][:,:2]
    from scipy.spatial import cKDTree
    # Near a measured projected point is evidence of support, not exact copying.
    observed=arrays['crater_mask']|arrays['ground_mask']
    distance,_=cKDTree(xy[observed]).query(top[:,:2])
    support=distance<max(.002,2*R/cfg.mesh_size)
    np.savez_compressed(out/'completed_mesh.npz',top_local_m=top,top_camera_m=camera,
        top_faces=faces,solid_local_m=solid,solid_camera_m=solid_camera,solid_faces=solid_faces,
        near_observed_support=support,base_z_m=base_z,**diagnostics)
    provenance={'frame':report['frame'],'bbox_xyxy':report['bbox_xyxy'],
        'diameter_m':2*R,'observed_max_angle_deg':report['slopes']['observed_max_angle_deg'],
        'model_rim_angle_deg':report['slopes']['inferred_rim_max_angle_deg'],'inferred_depth_m':d,
        'height_reference':'local outside-ground trend; field geometry, not unamplified terrain',
        'ground_degree':report['terrain']['degree'],'ground_coefficients_m':report['terrain']['coefficients_m'],
        'full_top_surface':True,'unobserved_floor_and_walls_completed':True,
        'model':'Zhou 2023 Eq.6 zero-raised-rim paraboloid plus measured outside-ground trend',
        'derivation':'z=ground(x,y)+d*((r/R)^2-1) for r<R; d=D*tan(model_rim_angle)/4',
        'not_original_paper_inverse_algorithm':True,'physical_z_scale_applied':1.,
        'quality_flags':report['quality_flags'],'validation_against_measured_surface_rms_m':report['surface_model_rms_to_observed_m'],
        'artificial_export_base_z_m':base_z,'base_is_measured':False,
        'surface_topology':qa_top,'solid_topology':qa_solid}
    if terrain_metadata is not None:
        provenance.update(terrain_metadata)
        provenance['model']='Observed terrain constrained non-axisymmetric completion; paraboloid prior only for hidden surface'
        provenance['derivation']='z=prior(x,y)+u(x,y); robust observed spatial-cell heights constrain a smooth 2D displacement u'
        provenance['validation_against_measured_surface_rms_m']=terrain_metadata['observed_cell_fit_rms_m']
        if not terrain_metadata['holdout_improved_over_prior']:
            provenance['quality_flags']=provenance['quality_flags']+['completion_not_improved_in_spatial_holdout']
    else:provenance['method']='paraboloid'
    (out/'completion.json').write_text(json.dumps(provenance,ensure_ascii=False,indent=2),encoding='utf-8')
    return top,faces,solid,solid_faces,provenance


def render_completion(report,arrays,out):
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection
    data=np.load(Path(out)/'completed_mesh.npz');local=data['top_local_m']*1000;f=data['top_faces']
    meta=json.loads((Path(out)/'completion.json').read_text(encoding='utf-8'))
    terrain_mode=meta.get('method')=='observed_terrain_constrained_completion'
    # Camera X/right, optical Z/forward, -Y/up: retain the measured plane pose.
    # This display basis is NOT a gravity/SLAM world frame.
    B=np.asarray(report['local_to_camera_basis_columns'])
    camera_centered=local@B.T
    v=camera_centered[:,[0,2,1]]*np.array([1,1,-1]) if terrain_mode else local
    base=float(data['base_z_m'])*1000
    triangles=v[f];normal=np.cross(triangles[:,1]-triangles[:,0],triangles[:,2]-triangles[:,0]);normal/=np.linalg.norm(normal,axis=1)[:,None]
    light=np.array([-.3,-.5,1.]);light/=np.linalg.norm(light)
    grey=.44+.5*np.maximum(0,normal@light);colors=np.c_[grey,grey,grey,np.ones(len(grey))]
    has_image=bool(report.get('input_image') and Path(report['input_image']).is_file())
    for compare in ((False,True) if has_image else (False,)):
        fig=plt.figure(figsize=(14,7) if compare else (11,8),dpi=155,facecolor='#242424')
        if compare and report.get('input_image') and Path(report['input_image']).is_file():
            ax0=fig.add_axes([.025,.19,.37,.67]);ax0.set_facecolor('#242424')
            import cv2
            im=cv2.imdecode(np.fromfile(report['input_image'],dtype=np.uint8),cv2.IMREAD_GRAYSCALE)
            box=report['bbox_xyxy'];x0=max(0,int(box[0])-15);x1=min(im.shape[1],int(box[2])+16)
            y0=max(0,int(box[1])-15);y1=min(im.shape[0],int(box[3])+16)
            ax0.imshow(im[y0:y1,x0:x1],cmap='gray');ax0.axis('off')
            ax0.set_title(f"Observed IR | frame {report['frame']}",color='white',fontsize=15)
        ax=fig.add_axes([.42,.06,.57,.82] if compare else [.005,.045,.99,.9],projection='3d',facecolor='#242424')
        patch=Poly3DCollection(triangles,facecolors=colors,edgecolors='none',linewidths=0,antialiased=False)
        ax.add_collection3d(patch)
        # Brown plane is a display stand only, separated from the inferred floor.
        x0,y0=local[:,:2].min(axis=0);x1,y1=local[:,:2].max(axis=0)
        stand=np.array([(x0,y0,base),(x1,y0,base),(x1,y1,base),(x0,y1,base)])
        if terrain_mode:stand=(stand@B.T)[:,[0,2,1]]*np.array([1,1,-1])
        ax.add_collection3d(Poly3DCollection([stand],facecolors='#7e350d',edgecolors='none'))
        allv=np.vstack((v,stand));lo=allv.min(0);hi=allv.max(0)
        ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);ax.set_zlim(lo[2],hi[2])
        ax.set_box_aspect(hi-lo,zoom=1.05)
        ax.view_init(elev=38,azim=-57);ax.set_axis_off()
        if compare:
            fig.text(.5,.94,'Observed terrain constrains the completed crater surface' if terrain_mode else 'Complete crater shape inferred from measured diameter and wall slopes',ha='center',color='white',fontsize=18)
            text=f"BBox diameter {report['diameter_used_m']*100:.1f} cm   |   inferred depression {meta['inferred_depth_m']*100:.1f} cm   |   equal metric scale x1"
            fig.text(.5,.045,text,ha='center',color='#f1f1f1',fontsize=14)
            fig.text(.5,.012,'Full surface includes model-inferred floor and hidden walls. Brown base is for display.',ha='center',color='#b6b6b6',fontsize=10)
        else:
            fig.text(.04,.94,f"Frame {report['frame']} | "+('terrain-constrained completion' if terrain_mode else 'complete inferred crater'),color='white',fontsize=19)
            fig.text(.04,.06,f"D = {report['diameter_used_m']*100:.1f} cm   inferred depression = {meta['inferred_depth_m']*100:.1f} cm   |   scale x1",color='#e5e5e5',fontsize=14)
            fig.text(.04,.022,'Measured camera pose and directional relief retained; hidden regions inferred. Brown base: display only.' if terrain_mode else 'Unseen floor/walls inferred; test-field height retained. Brown base: display only.',color='#aaa',fontsize=10)
        fig.savefig(Path(out)/('complete_comparison.png' if compare else 'complete_crater.png'),facecolor=fig.get_facecolor());plt.close(fig)
    if terrain_mode:
        fig=plt.figure(figsize=(15,7.3),dpi=155,facecolor='#242424')
        variants=[data['prior_local_m']*1000,local]
        for j,vertex in enumerate(variants):
            mesh=(vertex@B.T)[:,[0,2,1]]*np.array([1,1,-1]);tri=mesh[f]
            normals=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]);normals/=np.linalg.norm(normals,axis=1)[:,None]
            grey=.44+.5*np.maximum(0,normals@light)
            ax=fig.add_axes([.01+j*.49,.07,.49,.81],projection='3d',facecolor='#242424')
            ax.add_collection3d(Poly3DCollection(tri,facecolors=np.c_[grey,grey,grey,np.ones(len(grey))],edgecolors='none',antialiased=False))
            ax.add_collection3d(Poly3DCollection([stand],facecolors='#7e350d',edgecolors='none'))
            ax.set_xlim(lo[0],hi[0]);ax.set_ylim(lo[1],hi[1]);ax.set_zlim(lo[2],hi[2]);ax.set_box_aspect(hi-lo,zoom=1.05)
            ax.view_init(elev=38,azim=-57);ax.set_axis_off()
            ax.set_title('Before: one radial profile for every direction' if j==0 else 'After: directional heights follow measured Depth',color='white',fontsize=14,pad=-2)
        fig.text(.5,.945,'Same camera orientation and metric scale | no extra height amplification',ha='center',color='white',fontsize=17)
        fig.text(.5,.035,f"Observed-cell height MAE: {meta['prior_observed_cell_mae_m']*1000:.2f} mm -> {meta['observed_cell_fit_mae_m']*1000:.2f} mm   |   hidden surfaces remain inferred",ha='center',color='#ccc',fontsize=13)
        fig.savefig(Path(out)/'terrain_before_after.png',facecolor=fig.get_facecolor());plt.close(fig)


def main():
    ap=argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--source',type=Path,required=True,help='Folder with metrics.json and reconstruction.npz')
    ap.add_argument('--output',type=Path)
    ap.add_argument('--no-preview',action='store_true')
    ap.add_argument('--shape',choices=['observed-terrain','paraboloid'],default='observed-terrain')
    args=ap.parse_args();out=args.output or args.source/'completion'
    report=json.loads((args.source/'metrics.json').read_text(encoding='utf-8'));arrays=dict(np.load(args.source/'reconstruction.npz'))
    *_,metadata=save_completion(report,arrays,out,shape=args.shape)
    from .complete_viewer import write_complete_viewer
    write_complete_viewer(report,arrays,out)
    if not args.no_preview:render_completion(report,arrays,out)
    print(json.dumps(metadata,ensure_ascii=True,indent=2))


if __name__=='__main__':main()
