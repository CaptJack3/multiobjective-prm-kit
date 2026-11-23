"""Export geometry-only animated USDA for Isaac Sim or another USD viewer.

No simulator or USD Python module is required to export. Time is a display
clock; there is no dynamics simulation or velocity/acceleration qualification.
"""
import json
from pathlib import Path
import re

import numpy as np

from .backends import make_backend
from .geometry import valid_q


def _matrix(matrix):
    return "(" + ", ".join("(" + ", ".join(f"{x:.12g}" for x in row) + ")" for row in matrix) + ")"


def export_usd(config, paths, output, index=0, max_step=0.05, fps=24):
    backend = make_backend(config)
    if not hasattr(backend, "vis_geoms"):
        raise ValueError("USD export requires the URDF backend")
    if not np.isfinite(max_step) or max_step <= 0 or not np.isfinite(fps) or fps <= 0:
        raise ValueError("max_step and fps must be positive")
    output = Path(output)
    if output.exists():
        raise ValueError("USD output already exists; choose a new file")
    with np.load(paths, allow_pickle=False) as z:
        q = z["q"]
        if "offsets" in z:
            if not 0 <= index < len(z["offsets"])-1:
                raise ValueError("Path index out of range")
            q = q[z["offsets"][index]:z["offsets"][index+1]]
    if len(q) == 0 or not all(valid_q(backend, x) for x in q):
        raise ValueError("Path has invalid configurations")
    dense = [q[0]]
    for a,b in zip(q[:-1],q[1:]):
        count = max(1,int(np.ceil(np.max(np.abs(b-a))/max_step)))
        dense.extend(a+t*(b-a) for t in np.linspace(0,1,count+1)[1:])
    poses = [[] for _ in backend.vis_geoms]
    camera_poses = []
    camera = config.get("camera")
    for state in dense:
        backend.pin.forwardKinematics(backend.model,backend.data,state)
        for row,g in zip(poses,backend.vis_geoms):
            row.append((backend.data.oMi[g.parentJoint]*g.placement).homogeneous.T)
        if camera:
            frame = backend.model.frames[backend.frame_id(camera.get("frame",backend.tool))]
            mount = backend.data.oMi[frame.parentJoint]*frame.placement
            optical = mount*backend.pose({"position_m":camera.get("offset_m",[0,0,0]),"rotation":camera.get("rotation",np.eye(3).tolist())})
            transform = optical.homogeneous.copy()
            transform[:3,:3] = transform[:3,:3] @ np.diag([1,-1,-1])
            camera_poses.append(transform.T)
    lines = ['#usda 1.0', '(', '    defaultPrim = "World"', '    metersPerUnit = 1', '    upAxis = "Z"',
             f'    startTimeCode = 0',f'    endTimeCode = {len(dense)-1}',f'    timeCodesPerSecond = {fps}',
             '    customLayerData = { string replayScope = "Geometric replay; display timing only" }',')','def Xform "World" {']

    def add_mesh(name, mesh, transforms, color):
        name = re.sub(r"[^a-zA-Z0-9_]", "_", name)
        points = ", ".join("("+", ".join(f"{x:.9g}" for x in v)+")" for v in mesh.vertices)
        faces = ", ".join(str(int(x)) for x in mesh.faces.ravel())
        lines.extend([f'    def Xform "{name}" {{', '        matrix4d xformOp:transform.timeSamples = {'])
        for i,pose in enumerate(transforms):
            lines.append(f'            {i}: {_matrix(pose)},')
        lines.extend(['        }','        uniform token[] xformOpOrder = ["xformOp:transform"]','        def Mesh "Geometry" {',
                      f'            point3f[] points = [{points}]',f'            int[] faceVertexIndices = [{faces}]',
                      '            int[] faceVertexCounts = ['+', '.join(['3']*len(mesh.faces))+']',
                      '            uniform token subdivisionScheme = "none"','            bool doubleSided = true',
                      f'            color3f[] primvars:displayColor = [({color[0]}, {color[1]}, {color[2]})]',
                      '        }','    }'])
    for i,(mesh,transforms) in enumerate(zip(backend.meshes,poses)):
        color = [0.38,0.55,0.68] if i < backend.moving_count else [0.55,0.48,0.4]
        add_mesh(f"body_{i}_{backend.links[i]}",mesh,transforms if i < backend.moving_count else transforms[:1],color)
    lines.extend(['    def DomeLight "Light" {','        float inputs:intensity = 1000','    }'])
    if camera_poses:
        fov=camera.get("fov_deg",[90,90]);focal=10.
        lines.extend(['    def Camera "WristCamera" {',f'        float focalLength = {focal}',
                      f'        float horizontalAperture = {2*focal*np.tan(np.deg2rad(fov[0]/2))}',
                      f'        float verticalAperture = {2*focal*np.tan(np.deg2rad(fov[1]/2))}',
                      '        float2 clippingRange = (' + ', '.join(map(str,camera.get("clipping_range_m",[0.015,100]))) + ')', '        matrix4d xformOp:transform.timeSamples = {'])
        lines.extend(f'            {i}: {_matrix(pose)},' for i,pose in enumerate(camera_poses))
        lines.extend(['        }','        uniform token[] xformOpOrder = ["xformOp:transform"]','    }'])
    lines.append('}')
    output.parent.mkdir(parents=True,exist_ok=True)
    temporary=output.with_name(output.name+'.tmp')
    temporary.write_text('\n'.join(lines)+'\n',encoding='utf-8')
    temporary.replace(output)
    return dict(output=str(output),frames=len(dense),fps=fps,scope='kinematic_mesh_replay',
                camera='/World/WristCamera' if camera else None,
                note='Open the USDA in Isaac Sim and play the timeline; no controller dynamics or physical timing.')
