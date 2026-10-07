"""Offline approximate grasp geometry from a fresh right-camera RGBD observation.

No hardware access. Lengths returned in mm. The CAD TCP and horizontal table
assumptions are explicit; this is not a verified hand-eye calibration.
"""
import hashlib
import inspect
import itertools
import json
from pathlib import Path


def _sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def _plane(points, np):
    rng = np.random.default_rng(17)
    sample = points[rng.choice(len(points), min(3500, len(points)), replace=False)]
    best = None
    for _ in range(250):
        a,b,c = sample[rng.choice(len(sample),3,replace=False)]
        n = np.cross(b-a,c-a); size = np.linalg.norm(n)
        if size < 1e-6: continue
        n /= size; d = -n@a; score = int((np.abs(sample@n+d)<5).sum())
        if best is None or score>best[0]: best = score,n,d
    if best is None: raise ValueError('No tabletop plane')
    n,d = best[1:]
    for _ in range(3):
        mask = np.abs(points@n+d)<5
        if mask.mean()<.65: raise ValueError('Insufficient tabletop plane inliers')
        inside=points[mask]; center=inside.mean(0)
        _,_,vh=np.linalg.svd((inside-center).T@(inside-center))
        n=vh[-1];d=float(-n@center)
        if d<0:n,d=-n,-d
    residual=points@n+d; inside=np.abs(residual)<5
    rms=float(np.sqrt(np.mean(residual[inside]**2)))
    if rms>3 or d<150: raise ValueError('Table plane too uncertain or camera too close')
    return n,d,rms,float(inside.mean())


def _fingers(rgb, xyz, z, cv2, np):
    z=np.where(np.isfinite(z),z,0)
    h,w=z.shape;v,u=np.indices(z.shape)
    value=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV)[:,:,2]
    reference=value[int(.55*h):int(.8*h),int(.15*w):int(.85*w)]
    threshold=max(6.,min(55.,float(np.median(reference))*.45))
    dark=((value<threshold)&(v>.60*h)).astype('uint8')
    count,labels,stats,_=cv2.connectedComponentsWithStats(dark,8)
    candidates=[];diagnostics=[]
    for index in range(1,count):
        x,y,bw,bh,area=stats[index]
        if not (area>60 and .07*w<x+bw/2<.96*w and bw<.24*w and y>.65*h and y+bh>.94*h):continue
        mask=labels==index
        inner=cv2.erode(mask.astype('uint8'),np.ones((3,3),'uint8'))>0
        near=inner&np.isfinite(z)&(z>120)&(z<300)&(v<y+max(14,int(.04*h)))
        points=xyz[near]
        tip_depth=z[inner&(z>0)&(v<y+max(14,int(.04*h)))]
        diagnostics.append(dict(bbox_xywh=[int(x),int(y),int(bw),int(bh)],
            near_tip_depth_pixels=int(len(points)),
            positive_tip_depth_median_mm=float(np.median(tip_depth)) if len(tip_depth) else None))
        if len(points)<12:continue
        median=np.median(points,axis=0)
        # Reject background-depth leakage and inconsistent near-depth surfaces.
        good=np.abs(points[:,2]-median[2])<6
        points=points[good]
        if len(points)<12:continue
        median=np.median(points,axis=0)
        rows=np.where(mask)[0];cols=np.where(mask)[1]
        distal=np.array([np.median(cols[rows<y+max(4,int(.01*h))]),float(y)])
        proximal=np.array([np.median(cols[rows>y+.75*bh]),np.median(rows[rows>y+.75*bh])])
        forward=distal-proximal;forward/=np.linalg.norm(forward)
        candidates.append(dict(point=median,area=int(area),bbox=[int(x),int(y),int(bw),int(bh)],
                               pixels=int(len(points)),forward=forward,
                               depth_mad_mm=float(np.median(np.abs(points[:,2]-median[2])))))
    if len(candidates)<2:
        raise ValueError('Need two separate visible fingers with near-depth support; background depth is not a finger. '+
                         json.dumps(diagnostics,allow_nan=False))
    pairs=[]
    for a,b in itertools.combinations(candidates,2):
        distance=np.linalg.norm(a['point']-b['point'])
        if 12<distance<110 and abs(a['bbox'][1]-b['bbox'][1])<.08*h:
            pairs.append((a['area']+b['area'],a,b))
    if not pairs:raise ValueError('No geometrically consistent finger pair')
    _,a,b=max(pairs,key=lambda x:x[0]);pair=sorted([a,b],key=lambda q:q['point'][0])
    return pair,float(threshold)


def estimate_scene(observation_path, joints_raw, pose_raw):
    """Return approximate scene/TCP geometry, or raise ValueError on poor evidence."""
    import cv2
    import numpy as np
    from scipy.spatial.transform import Rotation
    from piper_sdk.kinematics.piper_fk import C_PiperForwardKinematics
    path=Path(observation_path).resolve();obs=json.loads(path.read_text())
    frame=obs['cameras']['right_hand'];rgb_path=Path(frame['rgb_path']);depth_path=Path(frame['depth_m_path'])
    if not rgb_path.is_absolute():rgb_path=path.parent/rgb_path
    if not depth_path.is_absolute():depth_path=path.parent/depth_path
    rgb=cv2.imread(str(rgb_path));depth=np.load(depth_path,allow_pickle=False)*1000.
    if rgb is None or rgb.shape[:2]!=depth.shape:raise ValueError('RGB/depth dimensions mismatch')
    depth[~np.isfinite(depth)]=0
    if frame.get('depth_aligned_to')!='color':raise ValueError('Require depth aligned to color')
    if len(joints_raw)!=6 or len(pose_raw)!=6:raise ValueError('Need six raw joints and six raw pose fields')
    q=np.asarray(joints_raw,dtype=float)/1000.;pose=np.asarray(pose_raw,dtype=float)/1000.
    if not np.all(np.isfinite(q)) or not np.all(np.isfinite(pose)):raise ValueError('Nonfinite robot state')
    fk=np.array(C_PiperForwardKinematics(1).CalFK(np.deg2rad(q).tolist())[-1])
    r=Rotation.from_euler('xyz',pose[3:],degrees=True);rf=Rotation.from_euler('xyz',fk[3:],degrees=True)
    fk_mm=float(np.linalg.norm(fk[:3]-pose[:3]));fk_deg=float(np.rad2deg(np.linalg.norm((r.inv()*rf).as_rotvec())))
    if fk_mm>.5 or fk_deg>.1:raise ValueError('SDK FK and provided pose disagree')
    R=r.as_matrix() if hasattr(r,'as_matrix') else r.as_dcm()
    k=frame['intrinsics'];h,w=depth.shape;v,u=np.indices(depth.shape)
    if not all(np.isfinite(k[a]) for a in ('fx','fy','ppx','ppy')) or k['fx']<=0 or k['fy']<=0:
        raise ValueError('Invalid camera intrinsics')
    if any(abs(a)>1e-8 for a in k.get('coeffs',[])):raise ValueError('Nonzero distortion requires verified deprojection')
    xyz=np.stack(((u-k['ppx'])*depth/k['fx'],(v-k['ppy'])*depth/k['fy'],depth),-1)
    valid=np.isfinite(depth)&(depth>100)&(depth<2000)
    hsv=cv2.cvtColor(rgb,cv2.COLOR_BGR2HSV)
    red=(cv2.inRange(hsv,(0,85,5),(30,255,255))|cv2.inRange(hsv,(170,85,5),(179,255,255)))
    count,labels,stats,_=cv2.connectedComponentsWithStats(red,8)
    if count<2:raise ValueError('No red cube candidate')
    index=1+int(np.argmax(stats[1:,4]));mask=labels==index;x,y,bw,bh,area=stats[index]
    if sum(int(s[4])>.5*area for s in stats[1:])>1:raise ValueError('Multiple similarly sized red candidates')
    if area<60 or x<2 or y<2 or x+bw>w-2 or y+bh>h-2:raise ValueError('Red cube missing, too small, or cropped')
    inner=cv2.erode(mask.astype('uint8'),np.ones((3,3),'uint8'))>0
    if (inner&valid).sum()<30 or valid[inner].mean()<.75:raise ValueError('Insufficient red cube depth')
    table=(u>.08*w)&(u<.95*w)&(v>.23*h)&(v<.79*h)&valid
    table&=cv2.dilate(mask.astype('uint8'),np.ones((25,25),'uint8'))==0
    if table.sum()<1000:raise ValueError('Insufficient tabletop depth')
    n,d,rms,fraction=_plane(xyz[table],np)
    cube_points=xyz[inner&valid];heights=cube_points@n+d
    height=float(np.percentile(heights,95))
    if not 22<height<42:raise ValueError('Red candidate is inconsistent with a 30 mm cube')
    top_points=cube_points[(heights>=np.percentile(heights,65))&(heights<=np.percentile(heights,97))]
    if len(top_points)<8:raise ValueError('Insufficient cube top-surface points')
    top=np.median(top_points,axis=0);top_height=float(top@n+d)
    pair,threshold=_fingers(rgb,xyz,depth,cv2,np)
    left,right=pair;mid=(left['point']+right['point'])/2
    closing=right['point']-left['point'];spacing=float(np.linalg.norm(closing));closing/=spacing
    camera_horizontal=closing-n*(n@closing)
    if np.linalg.norm(camera_horizontal)<.25:raise ValueError('Observed closing axis too vertical to resolve yaw')
    camera_horizontal/=np.linalg.norm(camera_horizontal)
    observed_forward=left['forward']+right['forward'];observed_forward/=np.linalg.norm(observed_forward)
    candidates=[]
    for sign in (-1,1):
        base_axis=sign*R[:,1];base_horizontal=base_axis.copy();base_horizontal[2]=0
        if np.linalg.norm(base_horizontal)<.25:raise ValueError('Closing axis too vertical to resolve yaw')
        base_horizontal/=np.linalg.norm(base_horizontal)
        B=np.stack((base_horizontal,np.cross([0.,0.,1.],base_horizontal),[0.,0.,1.]),1)
        C=np.stack((camera_horizontal,np.cross(n,camera_horizontal),n),1);Rbc=B@C.T
        forward=Rbc.T@R[:,2]
        projection=np.array([k['fx']*(forward[0]*mid[2]-mid[0]*forward[2]),
                             k['fy']*(forward[1]*mid[2]-mid[1]*forward[2])])
        projection/=np.linalg.norm(projection)
        agreement=float(projection@observed_forward)
        mismatch=float(np.rad2deg(np.arccos(np.clip(base_axis@(Rbc@closing),-1,1))))
        candidates.append((agreement,mismatch,sign,Rbc))
    agreement,mismatch,sign,Rbc=max(candidates,key=lambda a:a[0])
    if agreement<.6 or mismatch>9:raise ValueError('Finger direction and SDK pose cannot resolve a consistent camera rotation')
    tcp=pose[:3]+R@np.array([0.,0.,135.8])
    height_tip=float(mid@n+d);table_z=float(tcp[2]-height_tip)
    grasp=top+n*(20.-top_height)
    to_base=lambda point:tcp+Rbc@(point-mid)
    places=[]
    for side in (1,-1):
        table_point=top-n*top_height+side*80*camera_horizontal
        uv=np.array([table_point[0]/table_point[2]*k['fx']+k['ppx'],table_point[1]/table_point[2]*k['fy']+k['ppy']])
        radius=25.;pixels=int(np.ceil(radius*max(k['fx'],k['fy'])/table_point[2]))
        if not(pixels<uv[0]<w-pixels and pixels<uv[1]<h-pixels):continue
        disk=(u-uv[0])**2+(v-uv[1])**2<pixels**2;good=disk&valid
        valid_fraction=float(good.sum()/disk.sum())
        if valid_fraction<.85:continue
        plane_error=np.abs(xyz[good]@n+d);p95=float(np.percentile(plane_error,95))
        if p95>7 or (mask&disk).any():continue
        places.append(dict(tcp_base_mm=to_base(table_point+n*20).tolist(),table_base_mm=to_base(table_point).tolist(),
                           empty_patch_radius_mm=radius,valid_fraction=valid_fraction,p95_plane_residual_mm=p95))
    if not places:raise ValueError('No visible empty tabletop patch for placement')
    return dict(schema=1,units='mm; rotations dimensionless; raw input positions micrometres and angles millidegrees',
        cube_top_camera_mm=top.tolist(),cube_grasp_camera_mm=grasp.tolist(),cube_height_mm=height,
        table_plane_camera=dict(normal=n.tolist(),offset_mm=d,rms_mm=rms,inlier_fraction=fraction),
        finger_tip_midpoint_camera_mm=mid.tolist(),finger_tip_height_mm=height_tip,finger_spacing_mm=spacing,
        fingers=[dict(point_camera_mm=a['point'].tolist(),bbox_xywh=a['bbox'],valid_near_depth_pixels=a['pixels']) for a in pair],
        R_base_camera_approx=Rbc.tolist(),closing_axis_sign=sign,axis_mismatch_deg=mismatch,
        forward_direction_agreement=agreement,tcp_offset_flange_mm=[0.,0.,135.8],tcp_base_mm=tcp.tolist(),
        cube_top_base_mm=to_base(top).tolist(),cube_grasp_base_mm=to_base(grasp).tolist(),table_base_z_mm=table_z,
        place_candidates=places,uncertainty=dict(rotation_deg=max(6.,mismatch+2),depth_mm=max(3.,2*rms),
             finger_feature_mm=12.,cube_centering_mm=8.,CAD_TCP_assumed=True,
             note='Visible finger surface midpoint can differ from central CAD TCP by about 10 mm; no verified absolute error bound. Upright base and horizontal table assumed.'),
        provenance=dict(observation=str(path),captured_at=obs.get('captured_at'),camera_serial=frame.get('serial'),
            sdk_fk_source_sha256=_sha(inspect.getfile(C_PiperForwardKinematics)),module_sha256=_sha(__file__),
            observation_sha256=_sha(path),rgb_sha256=_sha(rgb_path),depth_sha256=_sha(depth_path),
            fk_error_mm=fk_mm,fk_error_deg=fk_deg,dark_value_threshold=threshold,
            state_image_synchronization='Caller must pair fresh held-state with capture; offline function does not assert hardware synchronization'))
