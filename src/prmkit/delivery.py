"""Export or render a saved selection, independently of roadmap/search work."""
import csv
import json
from pathlib import Path

import numpy as np

from .artifacts import candidate_catalog, content_hash, read_path, read_selection
from .backends import make_backend
from .config import number
from .io import atomic_json, atomic_npz, digest
from .verification import PASS, attach_verification, current_verification


def selected_rows(directory, selection, only_verified=False):
    root=Path(directory).resolve()
    catalog=candidate_catalog(root)
    ids=set(read_selection(selection,catalog))
    catalog=attach_verification(catalog,current_verification(root,catalog))
    rows=[r for r in catalog["paths"] if r["path_id"] in ids]
    if not rows:
        raise ValueError("Selection is empty; choose one or more candidate paths")
    if only_verified and any(r["verification"]["status"] not in PASS for r in rows):
        raise ValueError("Selection includes unverified/invalid paths while only_verified=true")
    return root,catalog,rows


def _destination(root, kind, rows, catalog, parameters, output):
    from . import __version__
    job=dict(catalog_hash=catalog["catalog_hash"],paths=rows,parameters=parameters,kit_version=__version__)
    fingerprint=content_hash(job)
    folder=Path(output).resolve() if output is not None else root/kind/fingerprint[:20]
    if folder.exists() and any(folder.iterdir()):
        manifest=folder/"manifest.json"
        if not manifest.exists():
            raise ValueError(f"Output is not empty: {folder}; use a new directory")
        record=json.loads(manifest.read_text())
        if record["fingerprint"]!=fingerprint:
            raise ValueError("Export/render output exists with different inputs")
        for name,expected in record["files"].items():
            if digest(folder/name)!=expected:
                raise ValueError("Export/render artifact integrity failure")
        return folder,job,record
    folder.mkdir(parents=True,exist_ok=True)
    return folder,job,None


def _finish(folder, job, files, **extra):
    result=dict(complete=True,output=str(folder),fingerprint=content_hash(job),lineage=job,
                files={str(p.relative_to(folder)):digest(p) for p in files},**extra)
    atomic_json(folder/"manifest.json",result)
    return result


def export_paths(directory, selection, *, output=None, only_verified=False, usd=False):
    root,catalog,rows=selected_rows(directory,selection,only_verified)
    folder,job,existing=_destination(root,"exports",rows,catalog,dict(only_verified=only_verified,usd=usd),output)
    if existing:
        return existing
    files=[]
    for row in rows:
        q,config,record=read_path(root,row,check_geometry=True)
        backend=make_backend(config)
        ee=np.array([backend.state(state).ee for state in q])
        name=row["path_id"]
        path=folder/(name+".npz")
        atomic_npz(path,q=q,ee=ee,costs=np.asarray(row["costs"]),joint_names=np.asarray(backend.joint_names),metrics=np.asarray(row["metrics"]))
        files.append(path)
        path=folder/(name+".csv")
        with path.open("w",newline="") as file:
            writer=csv.writer(file);writer.writerow([*backend.joint_names,"ee_x_m","ee_y_m","ee_z_m"])
            writer.writerows(np.column_stack((q,ee)))
        files.append(path)
        path=folder/(name+".json")
        atomic_json(path,dict(row,source_inputs_sha256=digest(root/row["run"]/"inputs.json"),
                              joint_names=backend.joint_names,trajectory="Geometric joint polyline; no timing, torque or grasp controller"))
        files.append(path)
        if usd:
            from .usd import export_usd
            path=folder/(name+".usda")
            export_usd(config,root/row["run"]/"paths.npz",path,index=row["index"])
            files.append(path)
    return _finish(folder,job,files,paths=len(rows),selection=[r["path_id"] for r in rows])


def render_paths(directory, selection, *, output=None, only_verified=False, frames=48, fps=24, width=960, height=720):
    """CPU mesh replay MP4. Display time is not a robot execution trajectory."""
    for key,value,minimum in (("frames",frames,2),("fps",fps,1),("width",width,240),("height",height,180)):
        number(value,"render."+key,minimum,integer=True)
    if width%2 or height%2:
        raise ValueError("MP4 width/height must be even")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.animation import FFMpegWriter
        from mpl_toolkits.mplot3d.art3d import Poly3DCollection
        import imageio_ffmpeg
    except ImportError as error:
        raise ValueError("Install multiobjective-prm-kit[render] for CPU MP4 rendering") from error
    matplotlib.rcParams["animation.ffmpeg_path"]=imageio_ffmpeg.get_ffmpeg_exe()
    root,catalog,rows=selected_rows(directory,selection,only_verified)
    params=dict(frames=frames,fps=fps,width=width,height=height,only_verified=only_verified,backend="matplotlib_mesh_replay")
    folder,job,existing=_destination(root,"videos",rows,catalog,params,output)
    if existing:
        return existing
    files=[]
    for row in rows:
        q,config,_=read_path(root,row,check_geometry=True)
        backend=make_backend(config)
        lengths=np.r_[0,np.cumsum(np.linalg.norm(np.diff(q,axis=0)*backend.weights,axis=1))]
        if lengths[-1]>0:
            ticks=np.linspace(0,lengths[-1],frames)
            unique=np.r_[True,np.diff(lengths)>0]
            states=np.column_stack([np.interp(ticks,lengths[unique],q[unique,i]) for i in range(q.shape[1])])
        else:
            states=np.repeat(q[:1],frames,axis=0)
        fig=plt.figure(figsize=(width/100,height/100),dpi=100,facecolor="#f2f6f5")
        ax=fig.add_subplot(111,projection="3d",facecolor="#f2f6f5")
        ax.set_xlabel("X [m]");ax.set_ylabel("Y [m]");ax.set_zlabel("Z [m]")
        ax.view_init(elev=27,azim=42)
        meshes=[];artists=[]
        if hasattr(backend,"vis_geoms"):
            all_bounds=[]
            for state in states:
                backend.pin.forwardKinematics(backend.model,backend.data,state)
                for mesh,g in zip(backend.meshes,backend.vis_geoms):
                    tf=backend.data.oMi[g.parentJoint]*g.placement
                    low,high=np.asarray(mesh.bounds)
                    corners=np.array([[x,y,z] for x in (low[0],high[0]) for y in (low[1],high[1]) for z in (low[2],high[2])])
                    all_bounds.extend(corners@tf.rotation.T+tf.translation)
            all_bounds=np.asarray(all_bounds)
            low,high=all_bounds.min(0)-.1,all_bounds.max(0)+.1
            for i,mesh in enumerate(backend.meshes):
                color="#7aadb9" if i<7 else "#383f47" if i<backend.moving_count else "#ccbd9b"
                poly=Poly3DCollection([],facecolors=color,edgecolors="none",alpha=1 if i<backend.moving_count else .35)
                ax.add_collection3d(poly);artists.append(poly);meshes.append(np.asarray(mesh.triangles))
        else:
            low=np.r_[backend.lower,0][:3]-.05;high=np.r_[backend.upper,.2][:3]+.05
            for obj in backend.objects:
                if len(obj["position_m"])==2:
                    from matplotlib.patches import Rectangle
                    from mpl_toolkits.mplot3d import art3d
                    p=np.asarray(obj["position_m"]);d=np.asarray(obj["dimensions_m"])
                    patch=Rectangle(p-d/2,*d,facecolor="#b7a47a",alpha=.5);ax.add_patch(patch);art3d.pathpatch_2d_to_3d(patch,z=0)
        middle=(low+high)/2;radius=max(high-low)/2
        ax.set_xlim(middle[0]-radius,middle[0]+radius);ax.set_ylim(middle[1]-radius,middle[1]+radius);ax.set_zlim(middle[2]-radius,middle[2]+radius)
        ax.set_box_aspect((1,1,1))
        trace,=ax.plot([],[],[],color="#086b61",linewidth=2)
        marker,=ax.plot([],[],[],"o",color="#d18e0a",markersize=5)
        ee=[]
        path=folder/(row["path_id"]+".mp4")
        writer=FFMpegWriter(fps=fps,codec="libx264",extra_args=["-pix_fmt","yuv420p","-crf","23"],metadata={"title":row["path_id"],"comment":"Kinematic mesh preview; display timing only"})
        try:
            with writer.saving(fig,str(path),dpi=100):
                for index,state in enumerate(states):
                    if hasattr(backend,"vis_geoms"):
                        backend.pin.forwardKinematics(backend.model,backend.data,state)
                        for triangles,g,artist in zip(meshes,backend.vis_geoms,artists):
                            tf=backend.data.oMi[g.parentJoint]*g.placement
                            artist.set_verts(triangles@tf.rotation.T+tf.translation)
                        frame=backend.model.frames[backend.frame_id(backend.tool)]
                        tf=backend.data.oMi[frame.parentJoint]*frame.placement
                        position=tf.translation+tf.rotation@backend.tcp_offset
                    else:
                        position=np.pad(state,(0,3-len(state)))
                    ee.append(position);points=np.asarray(ee)
                    trace.set_data(points[:,0],points[:,1]);trace.set_3d_properties(points[:,2])
                    marker.set_data([position[0]],[position[1]]);marker.set_3d_properties([position[2]])
                    ax.set_title(f"{row['path_id']}  |  {row['verification']['status']}\nFrame {index+1}/{frames} · kinematic preview · no physical timing",fontsize=10)
                    writer.grab_frame()
        finally:
            plt.close(fig)
        files.append(path)
        path=folder/(row["path_id"]+".json")
        atomic_json(path,dict(path_id=row["path_id"],path_hash=row["path_hash"],verification=row["verification"],
                              frames=frames,fps=fps,render_scope="CPU triangle-mesh replay; no Isaac RTX, physics, grasp controller or sensor-noise simulation"))
        files.append(path)
    return _finish(folder,job,files,paths=len(rows),selection=[r["path_id"] for r in rows],render_scope="kinematic_mesh_preview")
