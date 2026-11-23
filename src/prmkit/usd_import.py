"""Read a composed USD stage into a portable, static planning environment.

OpenUSD is optional and is never imported by the planning runtime. Collision
geometry is translated explicitly; unsupported physics is not silently dropped.
"""
from __future__ import annotations

import hashlib
from pathlib import Path
import tempfile

import numpy as np

from .io import atomic_json, digest


def _triangulate(points, counts, indices, holes=()):
    """Ear-clip planar simple polygons, including concave faces (not a fan)."""
    points = np.asarray(points, float)
    counts, indices = np.asarray(counts, int), np.asarray(indices, int)
    if (points.ndim != 2 or points.shape[1] != 3 or not np.isfinite(points).all()
            or counts.ndim != 1 or indices.ndim != 1 or np.any(counts < 3)
            or counts.sum() != len(indices) or np.any(indices < 0) or np.any(indices >= len(points))):
        raise ValueError("Invalid mesh points/face counts/indices")
    holes = set(map(int, holes))
    if not holes <= set(range(len(counts))):
        raise ValueError("Invalid mesh holeIndices")
    faces, start = [], 0
    for face_index, count in enumerate(counts):
        polygon = indices[start:start + count].tolist()
        start += count
        if face_index in holes:
            continue
        xyz = points[polygon] - points[polygon[0]]
        scale = float(np.max(np.linalg.norm(xyz, axis=1)))
        if scale <= 0:
            raise ValueError("Degenerate mesh face")
        xyz /= scale
        normal = np.cross(xyz, np.roll(xyz, -1, axis=0)).sum(axis=0)
        length = np.linalg.norm(normal)
        if length < 1e-12:
            raise ValueError("Degenerate or self-intersecting mesh face")
        normal /= length
        if np.max(np.abs(xyz @ normal)) > 1e-6:
            raise ValueError("Non-planar polygon: triangulate the source mesh before import")
        uv = np.delete(xyz, np.argmax(np.abs(normal)), axis=1)
        cross = lambda a, b: a[0] * b[1] - a[1] * b[0]
        area = sum(cross(a, b) for a, b in zip(uv, np.roll(uv, -1, axis=0)))
        sign = 1 if area > 0 else -1
        remaining = list(range(count))
        # Remove redundant collinear vertices before clipping.
        changed = True
        while changed and len(remaining) > 3:
            changed = False
            for i, b in enumerate(remaining):
                a, c = remaining[i-1], remaining[(i+1) % len(remaining)]
                if abs(cross(uv[b]-uv[a], uv[c]-uv[b])) < 1e-12:
                    remaining.pop(i)
                    changed = True
                    break
        while len(remaining) > 3:
            for i, b in enumerate(remaining):
                a, c = remaining[i-1], remaining[(i+1) % len(remaining)]
                if sign * cross(uv[b]-uv[a], uv[c]-uv[b]) <= 1e-12:
                    continue
                inside = any(all(sign * cross(uv[v]-uv[u], uv[j]-uv[u]) >= -1e-12
                                 for u, v in ((a, b), (b, c), (c, a)))
                             for j in remaining if j not in (a, b, c))
                if not inside:
                    faces.append([polygon[a], polygon[b], polygon[c]])
                    remaining.pop(i)
                    break
            else:
                raise ValueError("Cannot triangulate polygon; export a simple triangulated mesh")
        if len(remaining) == 3:
            faces.append([polygon[i] for i in remaining])
    if not faces:
        raise ValueError("Mesh has no usable faces")
    return np.asarray(faces, int)


def import_usd(source, output, *, root="/", exclude=(), geometry="collision",
               robot_frame=None, snapshot_time=None, meters_per_unit=None,
               up_axis=None, mesh_policy="authored", plane_size=None,
               plane_thickness=0.1):
    """Import a local USD into a new directory; return a JSON-safe report.

    The output frame is metres/Z-up, or metres in the explicitly selected robot
    frame. snapshot_time freezes authored USD poses; it does not simulate PhysX.
    """
    try:
        from pxr import Usd, UsdGeom, UsdPhysics, Sdf, Tf
        import trimesh
    except ImportError as e:
        raise ValueError('USD import dependencies unavailable; install "multiobjective-prm-kit[usd]"') from e
    source, output = Path(source).resolve(), Path(output).resolve()
    if not source.is_file():
        raise ValueError(f"Missing local USD file: {source}")
    if source.suffix.lower() not in (".usd", ".usda", ".usdc", ".usdz"):
        raise ValueError("Source must be a local .usd, .usda, .usdc or .usdz stage")
    if output.exists():
        raise ValueError(f"Import output already exists; choose a new directory: {output}")
    if geometry not in ("collision", "all") or mesh_policy not in ("authored", "triangles", "convex-hull"):
        raise ValueError("Invalid geometry selection or mesh policy")
    if snapshot_time is not None and not np.isfinite(snapshot_time):
        raise ValueError("snapshot_time must be finite")
    if plane_size is not None and (np.asarray(plane_size).shape != (2,) or
            not np.isfinite(plane_size).all() or np.any(np.asarray(plane_size) <= 0)):
        raise ValueError("plane_size requires two positive lengths in metres")
    if not np.isfinite(plane_thickness) or plane_thickness <= 0:
        raise ValueError("plane_thickness must be positive metres")
    try:
        stage = Usd.Stage.Open(str(source), load=Usd.Stage.LoadAll)
    except Tf.ErrorException as e:
        raise ValueError(f"Cannot open USD; save/copy its referenced assets locally: {e}") from e
    if not stage:
        raise ValueError(f"Cannot open USD: {source}")
    errors = stage.GetCompositionErrors()
    if errors:
        raise ValueError("USD has unresolved composition/assets; export a flattened stage or copy dependencies: "
                         + "; ".join(map(str, errors)))
    warnings = []
    units = float(meters_per_unit if meters_per_unit is not None else UsdGeom.GetStageMetersPerUnit(stage))
    if not np.isfinite(units) or units <= 0:
        raise ValueError("meters_per_unit must be positive")
    if meters_per_unit is None and not UsdGeom.StageHasAuthoredMetersPerUnit(stage):
        warnings.append("No authored metersPerUnit: USD default 0.01 m/unit used; verify size or override it.")
    axis = str(up_axis or UsdGeom.GetStageUpAxis(stage)).upper()
    if axis not in ("Y", "Z"):
        raise ValueError("Only Y-up or Z-up stages are supported")
    if up_axis is None and not stage.HasAuthoredMetadata("upAxis"):
        warnings.append(f"No authored upAxis: USD fallback {axis} used; verify orientation or override it.")
    time = Usd.TimeCode.Default() if snapshot_time is None else Usd.TimeCode(snapshot_time)
    cache = UsdGeom.XformCache(time)

    def get_prim(path):
        p = Sdf.Path(path)
        if not p.IsAbsolutePath() or (not p.IsPrimPath() and p != Sdf.Path.absoluteRootPath):
            raise ValueError(f"Expected absolute USD prim path: {path}")
        prim = stage.GetPrimAtPath(p)
        if not prim or not prim.IsActive():
            raise ValueError(f"USD prim does not exist or is inactive: {path}")
        return prim

    root_path = get_prim(root).GetPath()
    exclusions = [get_prim(p).GetPath() for p in exclude]
    if any(not p.HasPrefix(root_path) for p in exclusions):
        raise ValueError("Excluded prims must be inside --root")
    # Gf uses row-vector matrices; the rest of prmkit uses column vectors.
    world_to_output = np.eye(4)
    world_to_output[:3, :3] = units * (np.eye(3) if axis == "Z" else
                                     np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]))

    def ancestry(prim):
        while prim and not prim.IsPseudoRoot():
            yield prim
            prim = prim.GetParent()

    def sampled_transform(prim):
        xform = UsdGeom.Xformable(prim)
        # Even one time sample may differ from the default value. Require an
        # explicit snapshot instead of silently using Default() in that case.
        return bool(xform) and bool(xform.GetTimeSamples())

    def schema_names(prim):
        # Standalone usd-core does not register every Isaac/PhysX schema.
        authored = prim.GetMetadata("apiSchemas")
        return sorted(set(prim.GetAppliedSchemas()) | set(authored.GetAppliedItems() if authored else []))

    def unsupported_system(prim):
        return prim.GetTypeName() in ("PointInstancer", "Skeleton") or any(
            any(word in s.lower() for word in ("deformable", "particle", "articulation", "skelbinding"))
            for s in schema_names(prim))

    if robot_frame:
        frame = get_prim(robot_frame)
        transform = np.array(cache.GetLocalToWorldTransform(frame)).T
        if not np.allclose(transform[:3, :3].T @ transform[:3, :3], np.eye(3), atol=1e-8):
            raise ValueError("--robot-frame must have a rigid transform with no scale or shear")
        if np.linalg.det(transform[:3, :3]) < 0:
            raise ValueError("--robot-frame cannot be mirrored")
        if snapshot_time is None and any(sampled_transform(p) for p in ancestry(frame)):
            raise ValueError("Animated robot frame requires --snapshot-time")
        world_to_output = np.diag([units, units, units, 1.]) @ np.linalg.inv(transform)

    objects, entries, skipped, frozen = [], [], [], set()
    meshes = []
    supported = {"Cube", "Sphere", "Cylinder", "Capsule", "Mesh", "Plane"}

    def mesh_object(vertices, faces, position, name, convex=False):
        mesh = trimesh.Trimesh(vertices=vertices, faces=faces, process=False)
        if convex:
            try:
                mesh = mesh.convex_hull
            except Exception as e:
                raise ValueError("Convex hull needs a nondegenerate 3D mesh") from e
        if not np.isfinite(mesh.vertices).all() or not len(mesh.faces):
            raise ValueError("Invalid transformed mesh")
        filename = f"meshes/{len(meshes):05d}_{hashlib.sha256(name.encode()).hexdigest()[:12]}.obj"
        meshes.append((filename, mesh))
        return dict(name=name, shape="mesh", mesh=filename, position_m=position.tolist(), convex=convex)

    for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies()):
        path = prim.GetPath()
        if not path.HasPrefix(root_path):
            continue
        if any(path.HasPrefix(p) for p in exclusions):
            skipped.append(dict(prim=str(path), reason="explicitly excluded"))
            continue
        kind, name = prim.GetTypeName(), str(path)
        schemas = schema_names(prim)
        # Point instancers and deformables must not disappear from collision mode.
        if unsupported_system(prim):
            raise ValueError(f"{name}: {kind or schemas} is unsupported; exclude robots or bake static geometry")
        collision = prim.HasAPI(UsdPhysics.CollisionAPI)
        if collision and UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().GetNumTimeSamples():
            if snapshot_time is None:
                raise ValueError(f"{name}: animated collisionEnabled requires --snapshot-time")
            frozen.add(name)
        enabled = collision and bool(UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get(time))
        if geometry == "collision" and not enabled:
            if kind in supported or prim.IsA(UsdGeom.Gprim):
                skipped.append(dict(prim=name, reason="no enabled CollisionAPI"))
            continue
        if geometry == "all" and kind not in supported and not prim.IsA(UsdGeom.Gprim) and not collision:
            continue
        if kind not in supported:
            raise ValueError(f"{name}: unsupported collision geometry {kind!r}; export a triangle Mesh")
        if geometry == "all" and not enabled:
            warnings.append(f"{name}: imported as a collider by --geometry all, despite missing/disabled CollisionAPI.")
        try:
            for ancestor in ancestry(prim):
                if unsupported_system(ancestor):
                    raise ValueError(f"unsupported ancestor {ancestor.GetPath()}; bake static geometry or exclude the subtree")
                moving = ancestor.HasAPI(UsdPhysics.RigidBodyAPI) and bool(
                    UsdPhysics.RigidBodyAPI(ancestor).GetRigidBodyEnabledAttr().Get(time))
                varying_body = ancestor.HasAPI(UsdPhysics.RigidBodyAPI) and UsdPhysics.RigidBodyAPI(
                    ancestor).GetRigidBodyEnabledAttr().GetNumTimeSamples()
                animated = sampled_transform(ancestor)
                if moving or animated or varying_body:
                    if snapshot_time is None:
                        raise ValueError(f"moving/animated ancestor {ancestor.GetPath()}; use --snapshot-time to explicitly freeze authored poses")
                    frozen.add(str(ancestor.GetPath()))
            relevant = [a for a in prim.GetAttributes() if a.GetName() in
                        {"points", "faceVertexCounts", "faceVertexIndices", "holeIndices", "size", "radius", "height",
                         "axis", "physics:collisionEnabled", "physics:approximation"}]
            if any(a.GetNumTimeSamples() for a in relevant):
                if snapshot_time is None:
                    raise ValueError("animated geometry requires --snapshot-time")
                frozen.add(name)
            matrix = world_to_output @ np.array(cache.GetLocalToWorldTransform(prim)).T
            linear, position = matrix[:3, :3], matrix[:3, 3]
            if not np.isfinite(matrix).all() or abs(np.linalg.det(linear)) < 1e-24:
                raise ValueError("singular/nonfinite transform")
            scales = np.linalg.norm(linear, axis=0)
            rotation = linear / scales
            orthogonal = np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8)
            # Symmetric primitives are unchanged by flipping one local axis.
            if np.linalg.det(rotation) < 0:
                rotation[:, 0] *= -1
            obj = dict(name=name, collider=True, position_m=position.tolist(), rotation=rotation.tolist())
            approximation = None
            if kind == "Cube":
                size = float(UsdGeom.Cube(prim).GetSizeAttr().Get(time))
                if not np.isfinite(size) or size <= 0:
                    raise ValueError("Cube size must be positive")
                if orthogonal:
                    obj.update(shape="box", dimensions_m=(size * scales).tolist())
                else:
                    cube = trimesh.creation.box(extents=[size] * 3)
                    obj = mesh_object(cube.vertices @ linear.T, cube.faces, position, name, True)
            elif kind == "Mesh":
                schema = UsdGeom.Mesh(prim)
                points = np.asarray(schema.GetPointsAttr().Get(time), float)
                faces = _triangulate(points, schema.GetFaceVertexCountsAttr().Get(time),
                                     schema.GetFaceVertexIndicesAttr().Get(time), schema.GetHoleIndicesAttr().Get(time) or ())
                if (schema.GetOrientationAttr().Get(time) == "leftHanded") != (np.linalg.det(linear) < 0):
                    faces = faces[:, ::-1]
                approximation = str(UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get(time) or "none")
                unsupported_cooking = [s for s in schemas if any(w in s.lower() for w in
                                      ("sdfmesh", "convexdecomposition", "meshmerge"))]
                if unsupported_cooking and mesh_policy == "authored":
                    raise ValueError(f"unsupported PhysX cooking {unsupported_cooking}; bake geometry or explicitly choose --mesh-policy")
                if mesh_policy == "authored" and approximation not in ("none", "convexHull"):
                    raise ValueError(f"unsupported mesh approximation {approximation!r}; bake collision meshes, or explicitly choose --mesh-policy triangles/convex-hull")
                convex = mesh_policy == "convex-hull" or (mesh_policy == "authored" and approximation == "convexHull")
                obj = mesh_object(points @ linear.T, faces, position, name, convex)
                if mesh_policy != "authored":
                    warnings.append(f"{name}: authored collision approximation {approximation!r} / {unsupported_cooking} overridden by {mesh_policy}.")
                if str(schema.GetSubdivisionSchemeAttr().Get(time)) != "none":
                    warnings.append(f"{name}: uses the mesh control cage; rendered subdivision is not evaluated.")
            elif kind == "Plane":
                if plane_size is None:
                    raise ValueError("Plane requires --plane-size WIDTH LENGTH in metres to bound the planning region")
                if not orthogonal:
                    raise ValueError("Sheared plane unsupported; export a finite Mesh")
                normal_axis = str(prim.GetAttribute("axis").Get(time) or "Z")
                order = {"X": [1, 2, 0], "Y": [2, 0, 1], "Z": [0, 1, 2]}[normal_axis]
                rot = linear / scales
                rot = rot[:, order]
                if np.linalg.det(rot) < 0:
                    rot[:, 0] *= -1
                obj.update(shape="box", dimensions_m=[*map(float, plane_size), float(plane_thickness)],
                           rotation=rot.tolist(), position_m=(position - rot[:, 2] * plane_thickness / 2).tolist())
                warnings.append(f"{name}: plane replaced by finite {plane_size} m slab, thickness {plane_thickness} m; keep the entire robot inside its footprint.")
            else:
                schema = getattr(UsdGeom, kind)(prim)
                radius = float(schema.GetRadiusAttr().Get(time))
                if not np.isfinite(radius) or radius <= 0 or not orthogonal:
                    raise ValueError("Curved shapes require positive radius and a transform without shear")
                if kind == "Sphere":
                    if not np.allclose(scales, scales[0], rtol=1e-7, atol=1e-12):
                        raise ValueError("Nonuniformly scaled Sphere: export a triangle Mesh")
                    obj.update(shape="sphere", radius_m=float(radius * scales[0]))
                else:
                    axis_name = str(schema.GetAxisAttr().Get(time))
                    order = {"X": [1, 2, 0], "Y": [2, 0, 1], "Z": [0, 1, 2]}[axis_name]
                    sx, sy, sz = scales[order]
                    if not np.isclose(sx, sy, rtol=1e-7, atol=1e-12) or (kind == "Capsule" and not np.isclose(sx, sz, rtol=1e-7, atol=1e-12)):
                        raise ValueError(f"Nonuniformly scaled {kind}: export a triangle Mesh")
                    height = float(schema.GetHeightAttr().Get(time))
                    if not np.isfinite(height) or height <= 0:
                        raise ValueError("Cylinder/Capsule height must be positive")
                    obj.update(shape=kind.lower(), radius_m=float(radius * sx), length_m=float(height * sz),
                               rotation=rotation[:, order].tolist())
            objects.append(obj)
            entries.append(dict(prim=name, usd_type=kind, object=name, shape=obj["shape"],
                                authored_approximation=approximation, collision_enabled=bool(enabled)))
        except (ValueError, TypeError, KeyError, Tf.ErrorException) as e:
            raise ValueError(f"{name}: {e}") from e
    if not objects:
        raise ValueError("No collision objects selected. Check --root/--exclude, enable collisions in Isaac, or explicitly use --geometry all.")
    if skipped:
        warnings.append("Some prims were not imported; review skipped entries (disabled/non-collider geometry and explicit exclusions).")
    if frozen:
        warnings.append("Snapshot uses authored USD poses only; it does not run physics or reproduce transient GPU simulation state.")
    layer_hashes = {str(Path(l.realPath).resolve()): digest(l.realPath)
                    for l in stage.GetUsedLayers() if l.realPath and Path(l.realPath).is_file()}
    report = dict(schema_version=1, source=str(source), source_sha256=digest(source),
                  usd_version=".".join(map(str, Usd.GetVersion())), source_layers=layer_hashes,
                  settings=dict(root=root, exclude=list(exclude), geometry=geometry, robot_frame=robot_frame,
                                snapshot_time=snapshot_time, meters_per_unit=units, up_axis=axis,
                                mesh_policy=mesh_policy, plane_size=plane_size, plane_thickness=plane_thickness),
                  output_frame="robot_frame" if robot_frame else "Z-up world", world_to_output=world_to_output.tolist(),
                  objects=entries, skipped=skipped, frozen=sorted(frozen), warnings=warnings,
                  limitations=["Static geometric planning; no PhysX dynamics/contact materials/RTX rendering.",
                               "PhysX cooking settings, collision filtering, contact/rest offsets and materials are not reproduced; use planning margins and explicit pair policies.",
                               "Visibility uses imported collision geometry, not render-only meshes/materials.",
                               "Triangle meshes represent surfaces; convexHull produces a solid convex obstacle.",
                               "Robot, start/goals and camera are configured separately; a USD robot is not converted to URDF."])
    output.parent.mkdir(parents=True, exist_ok=True)
    with tempfile.TemporaryDirectory(prefix=".prmkit-usd-", dir=output.parent) as temp:
        bundle = Path(temp) / "bundle"
        bundle.mkdir()
        for filename, mesh in meshes:
            target = bundle / filename
            target.parent.mkdir(exist_ok=True)
            # OBJ preserves double precision and avoids binary STL float32 loss.
            target.write_text(trimesh.exchange.obj.export_obj(mesh, include_normals=False,
                                                            include_texture=False, digits=17), encoding="utf-8")
        atomic_json(bundle / "scene.json", dict(environment_id=source.stem, objects=objects,
                                               import_provenance=dict(source_sha256=report["source_sha256"],
                                                                      settings=report["settings"], output_frame=report["output_frame"])))
        report["files"] = {str(p.relative_to(bundle)): digest(p) for p in bundle.rglob("*") if p.is_file()}
        atomic_json(bundle / "import_report.json", report)
        if output.exists():
            raise ValueError(f"Import output was created concurrently: {output}")
        bundle.rename(output)
    return dict(status="imported", scene=str(output / "scene.json"), report=str(output / "import_report.json"),
                objects=len(objects), warnings=warnings, output_frame=report["output_frame"])
