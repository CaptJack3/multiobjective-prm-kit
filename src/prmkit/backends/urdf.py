"""Fixed-base URDF backend with explicit collision policy and attached bodies.

Supports bounded, one-coordinate revolute and prismatic joints. Free-flyers and
continuous/quaternion joints need a custom backend with its own interpolation.
"""
from __future__ import annotations

import json
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np

from . import State


class URDFBackend:
    def __init__(self, robot, environment):
        try:
            import coal
            import pinocchio as pin
            import trimesh
        except ImportError as e:
            raise ValueError('Robot dependencies unavailable; install "multiobjective-prm-kit[robot]". ' + str(e)) from e
        self.coal, self.pin, self.trimesh = coal, pin, trimesh
        self.robot, self.environment = robot, environment
        path = robot["urdf"]
        self.model, nominal, _ = pin.buildModelsFromUrdf(path, robot.get("package_dirs", [str(Path(path).parent)]))
        self.data = self.model.createData()
        joints = self.model.joints
        if self.model.nq != self.model.nv or any(j.nq != 1 or j.nv != 1 for j in list(joints)[1:]):
            raise ValueError("URDF backend supports fixed-base, bounded 1-DOF joints only")
        self.joint_names = list(self.model.names)[1:]
        if "joint_names" in robot and robot["joint_names"] != self.joint_names:
            raise ValueError(f"URDF joint order is {self.joint_names}")
        for key in ("lower", "upper", "weights"):
            if key in robot and np.asarray(robot[key]).shape != (self.model.nq,):
                raise ValueError(f"robot.{key} must have {self.model.nq} entries")
        self.lower = np.maximum(self.model.lowerPositionLimit, robot.get("lower", self.model.lowerPositionLimit))
        self.upper = np.minimum(self.model.upperPositionLimit, robot.get("upper", self.model.upperPositionLimit))
        self.weights = np.array(robot.get("weights", np.ones(self.model.nq)), float)
        self.tool = robot.get("tool_frame", "tool0")
        self.tcp_offset = np.asarray(robot.get("tcp_offset_m", [0, 0, 0]), float)
        mesh_mode = robot.get("collision_geometry", "planning") == "mesh"
        self.frame_id(self.tool)
        self.joint_types = {j.attrib["name"]: j.attrib["type"] for j in ET.parse(path).getroot().findall("joint")}
        if any(self.joint_types[n] not in ("revolute", "prismatic") for n in self.joint_names):
            raise ValueError("Only bounded revolute/prismatic joints supported")
        self.gm = pin.GeometryModel()
        self.links, self.meshes, self.vis_geoms = [], [], []
        hulls = np.load(robot["convex_file"], allow_pickle=False) if "convex_file" in robot and not mesh_mode else None
        seen = set()
        for g in nominal.geometryObjects:
            link = str(self.model.frames[g.parentFrame].name)
            self.meshes.append(self.to_mesh(g))
            self.vis_geoms.append(g.copy())
            if mesh_mode and g.meshPath and Path(g.meshPath).is_file():
                g = pin.GeometryObject(g.name, g.parentJoint, g.parentFrame, g.placement, self.triangle_model(self.meshes[-1]))
            if hulls is not None:
                if link in seen:
                    raise ValueError("Convex override requires one geometry per link")
                seen.add(link)
                frame = self.model.frames[g.parentFrame]
                geom = self.convex(hulls[link + "_v"], hulls[link + "_f"])
                g = pin.GeometryObject(link, frame.parentJoint, g.parentFrame, frame.placement, geom)
            self.gm.addGeometryObject(g)
            self.links.append(link)
        if hulls is not None:
            hulls.close()
        if not self.links:
            raise ValueError("URDF has no collision geometry")
        robot_count = len(self.links)
        self.roles = {link: "robot" for link in self.links}
        for part in robot.get("attachments", []):
            if part["name"] in self.links:
                raise ValueError("Attachment names must be unique and different from robot links")
            frame_id = self.frame_id(part["parent"])
            frame = self.model.frames[frame_id]
            shape, mesh = self.shape(part)
            if "visual_mesh" in part or "visual_mesh_keys" in part:
                mesh = self.read_mesh(part.get("visual_mesh", part["mesh"]),
                                      part.get("visual_mesh_keys"), part.get("scale", [1, 1, 1]))
            if mesh_mode and part.get("shape") == "mesh":
                shape = self.triangle_model(mesh)
            placement = frame.placement * self.pose(part)
            g = pin.GeometryObject(part["name"], frame.parentJoint, frame_id, placement, shape)
            self.gm.addGeometryObject(g)
            self.links.append(part["name"])
            self.meshes.append(mesh)
            self.vis_geoms.append(g.copy())
            self.roles[part["name"]] = part.get("role", "attachment")
        self.moving_count = len(self.links)
        self.world_objects = list(environment.get("objects", []))
        self.occluders = [True] * self.moving_count
        if len({o["name"] for o in self.world_objects}) != len(self.world_objects):
            raise ValueError("World object names must be unique")
        for obj in self.world_objects:
            shape, mesh = self.shape(obj)
            g = pin.GeometryObject("world__" + obj["name"], 0, 0, self.pose(obj), shape)
            self.gm.addGeometryObject(g)
            self.links.append("world__" + obj["name"])
            self.meshes.append(mesh)
            self.vis_geoms.append(g.copy())
            self.occluders.append(obj.get("occluder", obj.get("collider", True)))
        self.pair_kinds, self.pairs = [], []
        ignore = {frozenset(p) for p in robot.get("ignore_self_pairs", [])}
        active = None
        if "pair_policy" in robot:
            policy = json.loads(Path(robot["pair_policy"]).read_text())
            active = {frozenset(p) for p in policy["active_pairs"]}
            if "collision_links_ordered" in policy and set(policy["collision_links_ordered"]) != set(self.links[:robot_count]):
                raise ValueError("Collision policy links do not match URDF")
        for pair in ignore | (active or set()):
            if len(pair) != 2 or not pair <= set(self.links[:self.moving_count]):
                raise ValueError(f"Unknown or invalid self collision pair: {sorted(pair)}")
        for i in range(self.moving_count):
            for j in range(i+1, self.moving_count):
                a, b = self.gm.geometryObjects[i], self.gm.geometryObjects[j]
                pair = frozenset((self.links[i], self.links[j]))
                if a.parentJoint == b.parentJoint or pair in ignore:
                    continue
                if active is not None and i < robot_count and j < robot_count and pair not in active:
                    continue
                self.add_pair(i, j, "self")
        world_ignore = {tuple(p) for p in robot.get("ignore_world_pairs", [])}
        for pair in world_ignore:
            if len(pair) != 2 or pair[0] not in self.links[:self.moving_count] or pair[1] not in {o["name"] for o in self.world_objects}:
                raise ValueError(f"Unknown ignored world pair: {pair}")
        for i in range(self.moving_count):
            for j, obj in enumerate(self.world_objects, self.moving_count):
                if obj.get("collider", True) and (self.links[i], obj["name"]) not in world_ignore:
                    self.add_pair(i, j, "world")
        self.world_mask = np.array([k == "world" for k in self.pair_kinds], bool)
        self.held_mask = np.array([any(self.roles.get(link) == "held_object" for link in pair) for pair in self.pairs], bool)
        self.gd = pin.GeometryData(self.gm)
        for req in self.gd.distanceRequests:
            req.enable_signed_distance = False
            req.abs_err = req.rel_err = 0.
            req.gjk_tolerance = req.epa_tolerance = 1e-9
        self.bounds = self.make_bounds()
        self._visibility_cache = {}

    def frame_id(self, name):
        if not self.model.existFrame(name):
            raise ValueError(f"Unknown URDF frame: {name}")
        return self.model.getFrameId(name)

    def pose(self, obj):
        rotation = np.asarray(obj.get("rotation", self.pin.rpy.rpyToMatrix(*obj.get("rotation_rpy", [0, 0, 0]))), float)
        position = np.asarray(obj.get("position_m", [0, 0, 0]), float)
        if position.shape != (3,) or rotation.shape != (3, 3) or not np.isfinite(position).all() or not np.isfinite(rotation).all():
            raise ValueError("Pose requires finite xyz and 3x3 rotation")
        if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-8) or np.linalg.det(rotation) < 0.999999:
            raise ValueError("Rotation must be orthonormal and right-handed")
        return self.pin.SE3(rotation, position)

    def convex(self, vertices, faces):
        vv, ff = self.coal.StdVec_Vec3s(), self.coal.StdVec_Triangle()
        for v in vertices:
            vv.append(np.asarray(v, float))
        for f in faces:
            ff.append(self.coal.Triangle(*map(int, f)))
        return self.coal.Convex(vv, ff)

    def shape(self, obj):
        c, t = self.coal, self.trimesh
        kind = obj.get("shape", "box")
        if kind == "box":
            dims = np.array(obj["dimensions_m"], float)
            if dims.shape != (3,) or not np.isfinite(dims).all() or np.any(dims <= 0):
                raise ValueError("Box dimensions must be three positive values")
            return c.Box(*dims), t.creation.box(extents=dims)
        if kind == "mesh":
            mesh = self.read_mesh(obj["mesh"], obj.get("mesh_key"), obj.get("scale", [1, 1, 1]))
            if obj.get("convex", False):
                mesh = mesh.convex_hull
                return self.convex(mesh.vertices, mesh.faces), mesh
            return self.triangle_model(mesh), mesh
        radius, length = float(obj.get("radius_m", 0)), float(obj.get("length_m", 0))
        if not np.isfinite(radius) or radius <= 0 or (kind != "sphere" and (not np.isfinite(length) or length <= 0)):
            raise ValueError("Shape requires positive radius_m/length_m")
        if kind == "sphere":
            return c.Sphere(radius), t.creation.icosphere(subdivisions=2, radius=radius)
        if kind == "cylinder":
            return c.Cylinder(radius, length), t.creation.cylinder(radius=radius, height=length, sections=32)
        if kind == "capsule":
            # trimesh capsules are centered about the origin in current trimesh.
            return c.Capsule(radius, length), t.creation.capsule(radius=radius, height=length)
        raise ValueError(f"Unsupported geometry: {kind}")

    def read_mesh(self, path, keys=None, scale=(1, 1, 1)):
        if Path(path).suffix.lower() == ".npz":
            if isinstance(keys, str):
                keys = [keys]
            if not keys:
                raise ValueError("Packed NPZ mesh needs mesh_key or visual_mesh_keys")
            with np.load(path, allow_pickle=False) as data:
                meshes = [self.trimesh.Trimesh(data[k + "_v"], data[k + "_f"], process=False) for k in keys]
            mesh = self.trimesh.util.concatenate(meshes)
        else:
            mesh = self.trimesh.load_mesh(path, force="mesh")
        scale = np.asarray(scale, float)
        if scale.shape != (3,) or not np.isfinite(scale).all() or np.any(scale <= 0):
            raise ValueError("Mesh scale must be three positive values")
        mesh.apply_scale(scale)
        return mesh

    def triangle_model(self, mesh):
        model = self.coal.BVHModelOBBRSS()
        model.beginModel(len(mesh.faces), len(mesh.vertices))
        vv, ff = self.coal.StdVec_Vec3s(), self.coal.StdVec_Triangle()
        for v in mesh.vertices:
            vv.append(v)
        for f in mesh.faces:
            ff.append(self.coal.Triangle(*map(int, f)))
        model.addSubModel(vv, ff)
        model.endModel()
        return model

    def to_mesh(self, g):
        if g.meshPath and Path(g.meshPath).is_file():
            mesh = self.trimesh.load_mesh(g.meshPath, force="mesh")
            mesh.apply_scale(np.asarray(g.meshScale))
            return mesh
        geom, c = g.geometry, self.coal
        if isinstance(geom, c.Box):
            return self.trimesh.creation.box(extents=2 * np.asarray(geom.halfSide))
        if isinstance(geom, c.Sphere):
            return self.trimesh.creation.icosphere(radius=geom.radius)
        if isinstance(geom, c.Cylinder):
            return self.trimesh.creation.cylinder(radius=geom.radius, height=2*geom.halfLength)
        if isinstance(geom, c.Capsule):
            return self.trimesh.creation.capsule(radius=geom.radius, height=2*geom.halfLength)
        raise ValueError(f"Unsupported URDF collision shape: {type(geom).__name__}")

    def add_pair(self, i, j, kind):
        self.gm.addCollisionPair(self.pin.CollisionPair(i, j))
        self.pair_kinds.append(kind)
        self.pairs.append((self.links[i], self.links[j]))

    def make_bounds(self):
        """Sum each body's maximum point travel; shared ancestor motion cancels.

        Geometry AABB corners enclose all vertices. Prismatic travel is included
        in ancestor radii over the full declared joint limits, not just q0/q1.
        """
        radii, ancestors = [], []
        for geom in self.gm.geometryObjects:
            geom.geometry.computeLocalAABB()
            box = geom.geometry.aabb_local
            corners = np.array([[x, y, z] for x in (box.min_[0], box.max_[0])
                                for y in (box.min_[1], box.max_[1]) for z in (box.min_[2], box.max_[2])])
            radius = np.linalg.norm(corners @ geom.placement.rotation.T + geom.placement.translation, axis=1).max()
            row = np.zeros(self.model.nq)
            chain = set()
            j = geom.parentJoint
            while j:
                chain.add(j)
                k = self.model.joints[j].idx_q
                if self.joint_types[self.model.names[j]] == "prismatic":
                    row[k] = 1.
                    radius += max(abs(self.lower[k]), abs(self.upper[k]))
                else:
                    row[k] = radius
                radius += np.linalg.norm(self.model.jointPlacements[j].translation)
                j = self.model.parents[j]
            radii.append(row)
            ancestors.append(chain)
        bounds = []
        for pair in self.gm.collisionPairs:
            a, b = pair.first, pair.second
            row = radii[a] + radii[b]
            for j in ancestors[a] & ancestors[b]:
                row[self.model.joints[j].idx_q] = 0.
            bounds.append(row)
        return np.array(bounds).reshape(-1, self.model.nq)

    def motion_bound(self, dq):
        return self.bounds @ np.abs(dq)

    def state(self, q):
        p = self.pin
        p.forwardKinematics(self.model, self.data, np.asarray(q, float))
        p.updateGeometryPlacements(self.model, self.data, self.gm, self.gd)
        p.computeDistances(self.gm, self.gd)
        frame = self.model.frames[self.frame_id(self.tool)]
        tf = self.data.oMi[frame.parentJoint] * frame.placement
        ds = np.array([r.min_distance for r in self.gd.distanceResults])
        return State(ds, np.array(tf.translation + tf.rotation @ self.tcp_offset))

    def visibility(self, q, camera):
        from ..visibility import visible_fraction
        return visible_fraction(self, q, camera)
