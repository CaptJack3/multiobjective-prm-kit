"""CPU ray visibility, using nominal robot collision meshes and attachments.

Ray placement and endpoint offsets follow dense_prm_pipeline/visibility.py.
This implementation uses double precision trimesh rays, not Warp float32.
"""
import math
import json
import numpy as np


def surface_points(center, dimensions, count):
    per_face = math.ceil(count / 6)
    grid = math.ceil(math.sqrt(per_face))
    values = (np.arange(grid) + 0.5) / grid - 0.5
    uv = np.asarray([(u, v) for u in values for v in values])[:per_face]
    points = []
    for axis in range(3):
        other = [v for v in range(3) if v != axis]
        for sign in (-1., 1.):
            face = np.tile(center, (len(uv), 1)).astype(float)
            face[:, axis] += sign * dimensions[axis] / 2
            face[:, other[0]] += uv[:, 0] * dimensions[other[0]]
            face[:, other[1]] += uv[:, 1] * dimensions[other[1]]
            points.append(face)
    return np.concatenate(points)[:count]


def visible_fraction(backend, q, camera):
    key = json.dumps(camera, sort_keys=True)
    if key not in backend._visibility_cache:
        count = camera.get("rays", 64)
        if not isinstance(count, int) or count < 1:
            raise ValueError("camera.rays must be a positive integer")
        if "target_object" in camera:
            target = next((o for o in backend.environment["objects"] if o["name"] == camera["target_object"]), None)
            if target is None:
                raise ValueError("camera.target_object not found")
            points = surface_points(np.zeros(3), np.array(target["dimensions_m"]), count)
            pose = backend.pose(target)
            points = points @ pose.rotation.T + pose.translation
        else:
            center = np.asarray(camera["target"], float)
            points = (surface_points(center, np.asarray(camera["target_dimensions_m"], float), count)
                      if "target_dimensions_m" in camera else center.reshape(1, 3))
        fov = np.asarray(camera.get("fov_deg", [90, 90]), float)
        if fov.shape != (2,) or not np.isfinite(fov).all() or np.any((fov <= 0) | (fov >= 180)):
            raise ValueError("camera.fov_deg must contain two angles in (0,180)")
        backend._visibility_cache[key] = points, np.tan(np.deg2rad(fov / 2))
    points, tangents = backend._visibility_cache[key]
    pin = backend.pin
    pin.forwardKinematics(backend.model, backend.data, q)
    frame = backend.model.frames[backend.frame_id(camera.get("frame", backend.tool))]
    mount = backend.data.oMi[frame.parentJoint] * frame.placement
    pose = mount * backend.pose({"position_m": camera.get("offset_m", [0, 0, 0]),
                                 "rotation": camera.get("rotation", np.eye(3).tolist())})
    delta = points - pose.translation
    lengths = np.linalg.norm(delta, axis=1)
    local = delta @ pose.rotation
    visible = (local[:, 2] > 0) & (np.abs(local[:, 0]) <= tangents[0] * local[:, 2]) & (np.abs(local[:, 1]) <= tangents[1] * local[:, 2]) & (lengths > 0.008)
    if "clipping_range_m" in camera:
        near, far = camera["clipping_range_m"]
        if not 0 <= near < far:
            raise ValueError("Camera clipping range needs 0 <= near < far")
        visible &= (local[:, 2] >= near) & (local[:, 2] <= far)
    directions = delta / np.maximum(lengths[:, None], 1e-15)
    origins = pose.translation + directions * 0.003
    for index, (mesh, geom) in enumerate(zip(backend.meshes, backend.vis_geoms)):
        if not getattr(backend, "occluders", [True] * len(backend.meshes))[index]:
            continue
        if "target_object" in camera and backend.links[index] == "world__" + camera["target_object"]:
            continue
        ids = np.flatnonzero(visible)
        if len(ids) == 0:
            break
        world = backend.data.oMi[geom.parentJoint] * geom.placement
        o = (origins[ids] - world.translation) @ world.rotation
        d = directions[ids] @ world.rotation
        loc, ray, _ = mesh.ray.intersects_location(o, d, multiple_hits=True)
        if len(ray):
            distance = np.einsum("ij,ij->i", loc - o[ray], d[ray])
            blocked = ray[(distance >= 0) & (distance <= lengths[ids[ray]] - 0.005)]
            visible[ids[blocked]] = False
    return float(visible.mean())
