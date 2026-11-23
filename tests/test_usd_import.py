"""USD integration: scene semantics, fail-closed import and real PRM consumers."""
import json
from pathlib import Path
import shutil

import numpy as np
import pytest

pytest.importorskip("pxr.Usd")
pytest.importorskip("trimesh")
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdUtils

from prmkit.usd_import import import_usd, _triangulate
from prmkit.cli import main
from prmkit.config import load


def stage_at(tmp_path, units=1, axis="Z", name="scene.usda"):
    stage = Usd.Stage.CreateNew(str(tmp_path / name))
    UsdGeom.SetStageMetersPerUnit(stage, units)
    UsdGeom.SetStageUpAxis(stage, axis)
    UsdGeom.Xform.Define(stage, "/World")
    return stage


def cube(stage, path="/World/Box", position=(0, 0, 0), dims=(1, 1, 1), collision=True):
    shape = UsdGeom.Cube.Define(stage, path)
    shape.CreateSizeAttr(1)
    shape.AddTranslateOp().Set(position)
    shape.AddScaleOp().Set(dims)
    if collision:
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    return shape


def convert(stage, tmp_path, **kwargs):
    stage.GetRootLayer().Save()
    result = import_usd(stage.GetRootLayer().realPath, tmp_path / "imported", **kwargs)
    scene = json.loads(Path(result["scene"]).read_text())
    report = json.loads(Path(result["report"]).read_text())
    return scene, report


def test_hierarchy_centimeters_y_up_and_hidden_colliders(tmp_path):
    stage = stage_at(tmp_path, units=.01, axis="Y")
    parent = UsdGeom.Xform.Define(stage, "/World/Group")
    parent.AddTranslateOp().Set((100, 200, 300))
    parent.AddRotateYOp().Set(30)
    shape = cube(stage, "/World/Group/Box", (20, 0, 0), (10, 20, -30))
    shape.CreateVisibilityAttr("invisible")  # physics still active
    cube(stage, "/World/Visual", collision=False)
    scene, report = convert(stage, tmp_path)
    assert len(scene["objects"]) == 1
    obj = scene["objects"][0]
    assert obj["shape"] == "box"
    np.testing.assert_allclose(obj["dimensions_m"], [.1, .2, .3])
    # An independently evaluated source corner set must match the output box.
    gf = UsdGeom.XformCache().GetLocalToWorldTransform(shape.GetPrim())
    corners = np.array([(x, y, z) for x in (-.5, .5) for y in (-.5, .5) for z in (-.5, .5)])
    expected = np.array([gf.Transform(Gf.Vec3d(*p)) for p in corners])[:, [0, 2, 1]] * [.01, -.01, .01]
    actual = (corners * obj["dimensions_m"]) @ np.array(obj["rotation"]).T + obj["position_m"]
    for point in expected:
        assert np.min(np.linalg.norm(actual-point, axis=1)) < 1e-10
    assert any(i["prim"] == "/World/Visual" for i in report["skipped"])


def test_robot_frame_and_exclusion(tmp_path):
    stage = stage_at(tmp_path)
    robot = UsdGeom.Xform.Define(stage, "/World/Robot")
    robot.AddTranslateOp().Set((3, 4, 5))
    robot.AddRotateZOp().Set(90)
    UsdPhysics.ArticulationRootAPI.Apply(robot.GetPrim())
    cube(stage, "/World/Robot/Link")
    cube(stage, position=(3, 6, 5))
    scene, report = convert(stage, tmp_path, exclude=["/World/Robot"], robot_frame="/World/Robot")
    np.testing.assert_allclose(scene["objects"][0]["position_m"], [2, 0, 0], atol=1e-12)
    assert report["output_frame"] == "robot_frame"


def test_local_reference_native_instances_and_portability(tmp_path):
    asset = stage_at(tmp_path, name="asset.usda")
    cube(asset, "/World/Shape")
    asset.SetDefaultPrim(asset.GetPrimAtPath("/World"))
    asset.GetRootLayer().Save()
    stage = stage_at(tmp_path)
    for i in range(2):
        instance = UsdGeom.Xform.Define(stage, f"/World/Instance{i}")
        instance.GetPrim().GetReferences().AddReference("asset.usda")
        instance.GetPrim().SetInstanceable(True)
        instance.AddTranslateOp().Set((i*3, 0, 0))
    scene, report = convert(stage, tmp_path)
    assert [o["position_m"][0] for o in scene["objects"]] == [0, 3]
    assert len(report["source_layers"]) == 2
    assert all("Instance" in o["name"] for o in scene["objects"])


def test_concave_polygon_holes_and_mesh_asset(tmp_path):
    stage = stage_at(tmp_path)
    mesh = UsdGeom.Mesh.Define(stage, "/World/Mesh")
    # L-shaped face: area 3, not the area of its convex hull (3.5).
    points = np.array([(0, 0, 0), (2, 0, 0), (2, 1, 0), (1, 1, 0), (1, 2, 0), (0, 2, 0)], float)
    mesh.CreatePointsAttr(points.tolist())
    mesh.CreateFaceVertexCountsAttr([6, 3])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3, 4, 5, 0, 1, 2])
    mesh.CreateHoleIndicesAttr([1])
    mesh.CreateSubdivisionSchemeAttr("none")
    mesh.AddScaleOp().Set((-2, 1, 1))
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    scene, report = convert(stage, tmp_path)
    import trimesh
    result = trimesh.load_mesh(tmp_path / "imported" / scene["objects"][0]["mesh"])
    assert len(result.faces) == 4
    assert result.area == pytest.approx(6)
    assert np.all(result.face_normals[:, 2] > .99)
    assert scene["objects"][0]["convex"] is False
    assert len(report["files"]) == 2
    moved = tmp_path / "moved"
    shutil.copytree(tmp_path / "imported", moved)
    Path(stage.GetRootLayer().realPath).unlink()
    assert trimesh.load_mesh(moved / scene["objects"][0]["mesh"]).area == pytest.approx(6)


@pytest.mark.parametrize("shape", ["Sphere", "Cylinder", "Capsule"])
def test_curved_primitives_preserve_analytic_collision_shape(tmp_path, shape):
    stage = stage_at(tmp_path)
    schema = getattr(UsdGeom, shape).Define(stage, "/World/Shape")
    schema.CreateRadiusAttr(.2)
    schema.AddScaleOp().Set((2, 2, 2))
    if shape != "Sphere":
        schema.CreateHeightAttr(.5)
        schema.CreateAxisAttr("X")
    UsdPhysics.CollisionAPI.Apply(schema.GetPrim())
    scene, _ = convert(stage, tmp_path)
    obj = scene["objects"][0]
    assert obj["radius_m"] == pytest.approx(.4)
    if shape != "Sphere":
        assert obj["length_m"] == pytest.approx(1)
        np.testing.assert_allclose(np.array(obj["rotation"])[:, 2], [1, 0, 0])


@pytest.mark.parametrize("failure", ["dynamic", "animated", "point_instancer", "articulation", "cone", "scaled_sphere", "bad_reference"])
def test_unsupported_scene_never_silently_produces_partial_environment(tmp_path, failure):
    stage = stage_at(tmp_path)
    shape = cube(stage)
    if failure == "dynamic":
        UsdPhysics.RigidBodyAPI.Apply(shape.GetPrim())
    elif failure == "animated":
        op = shape.GetOrderedXformOps()[0]
        op.Set((0, 0, 0), 0)
        op.Set((1, 0, 0), 1)
    elif failure == "point_instancer":
        UsdGeom.PointInstancer.Define(stage, "/World/Instances")
    elif failure == "articulation":
        UsdPhysics.ArticulationRootAPI.Apply(stage.GetPrimAtPath("/World"))
    elif failure == "cone":
        UsdPhysics.CollisionAPI.Apply(UsdGeom.Cone.Define(stage, "/World/Cone").GetPrim())
    elif failure == "scaled_sphere":
        sphere = UsdGeom.Sphere.Define(stage, "/World/Sphere")
        sphere.AddScaleOp().Set((1, 2, 1))
        UsdPhysics.CollisionAPI.Apply(sphere.GetPrim())
    elif failure == "bad_reference":
        stage.DefinePrim("/World/Missing").GetReferences().AddReference("missing.usda")
    stage.GetRootLayer().Save()
    with pytest.raises(ValueError):
        import_usd(stage.GetRootLayer().realPath, tmp_path / "imported")
    assert not (tmp_path / "imported").exists()


def test_explicit_snapshot_and_all_geometry(tmp_path):
    stage = stage_at(tmp_path)
    shape = cube(stage, collision=False)
    UsdPhysics.RigidBodyAPI.Apply(shape.GetPrim())
    op = shape.GetOrderedXformOps()[0]
    op.Set((0, 0, 0), 0)
    op.Set((2, 0, 0), 2)
    scene, report = convert(stage, tmp_path, geometry="all", snapshot_time=1)
    assert scene["objects"][0]["position_m"] == [1, 0, 0]
    assert report["frozen"] == ["/World/Box"]
    assert report["warnings"]


@pytest.mark.parametrize("schema", ["PhysxDeformableBodyAPI", "PhysxParticleClothAPI", "PhysxSDFMeshCollisionAPI"])
def test_unregistered_isaac_schemas_are_not_silently_ignored(tmp_path, schema):
    stage = stage_at(tmp_path)
    mesh = UsdGeom.Mesh.Define(stage, "/World/Mesh")
    mesh.CreatePointsAttr([(0, 0, 0), (1, 0, 0), (0, 1, 0)])
    mesh.CreateFaceVertexCountsAttr([3])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
    mesh.GetPrim().SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit(["PhysicsCollisionAPI", schema]))
    stage.GetRootLayer().Save()
    with pytest.raises(ValueError, match="unsupported"):
        import_usd(stage.GetRootLayer().realPath, tmp_path / "imported")
    assert not (tmp_path / "imported").exists()


def test_time_varying_disabled_collision_cannot_hide_obstacle(tmp_path):
    stage = stage_at(tmp_path)
    shape = cube(stage)
    attr = UsdPhysics.CollisionAPI(shape.GetPrim()).GetCollisionEnabledAttr()
    attr.Set(False)
    attr.Set(True, 2)
    stage.GetRootLayer().Save()
    with pytest.raises(ValueError, match="animated collisionEnabled"):
        import_usd(stage.GetRootLayer().realPath, tmp_path / "failed")
    scene, report = convert(stage, tmp_path, snapshot_time=2)
    assert len(scene["objects"]) == 1
    assert report["frozen"]


@pytest.mark.parametrize("schema", ["PhysxDeformableBodyAPI", "PhysicsArticulationRootAPI"])
def test_root_selection_cannot_hide_unsupported_ancestor(tmp_path, schema):
    stage = stage_at(tmp_path)
    stage.GetPrimAtPath("/World").SetMetadata("apiSchemas", Sdf.TokenListOp.CreateExplicit([schema]))
    cube(stage)
    stage.GetRootLayer().Save()
    with pytest.raises(ValueError, match="unsupported ancestor"):
        import_usd(stage.GetRootLayer().realPath, tmp_path / "imported", root="/World/Box")


def test_units_override_root_and_reset_transform(tmp_path):
    stage = stage_at(tmp_path, units=.01, axis="Y")
    group = UsdGeom.Xform.Define(stage, "/World/Group")
    group.AddTranslateOp().Set((50, 0, 0))
    shape = cube(stage, "/World/Group/Box", position=(2, 0, 0))
    shape.SetResetXformStack(True)
    cube(stage, "/World/Outside")
    scene, _ = convert(stage, tmp_path, root="/World/Group", meters_per_unit=1, up_axis="Z")
    assert len(scene["objects"]) == 1
    np.testing.assert_allclose(scene["objects"][0]["position_m"], [2, 0, 0])


def test_sheared_cube_preserves_affine_geometry_as_convex_mesh(tmp_path):
    stage = stage_at(tmp_path)
    shape = UsdGeom.Cube.Define(stage, "/World/Box")
    matrix = Gf.Matrix4d(1)
    matrix.SetRow(0, Gf.Vec4d(1, .3, 0, 0))
    shape.AddTransformOp().Set(matrix)
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    scene, _ = convert(stage, tmp_path)
    obj = scene["objects"][0]
    assert obj["shape"] == "mesh" and obj["convex"]
    import trimesh
    mesh = trimesh.load_mesh(tmp_path / "imported" / obj["mesh"])
    assert mesh.volume == pytest.approx(8)


def test_plane_requires_explicit_extent(tmp_path):
    stage = stage_at(tmp_path)
    plane = UsdGeom.Plane.Define(stage, "/World/Ground")
    plane.CreateAxisAttr("Z")
    UsdPhysics.CollisionAPI.Apply(plane.GetPrim())
    stage.GetRootLayer().Save()
    with pytest.raises(ValueError, match="plane-size"):
        import_usd(stage.GetRootLayer().realPath, tmp_path / "failed")
    scene, _ = convert(stage, tmp_path, plane_size=[10, 20], plane_thickness=.2)
    obj = scene["objects"][0]
    assert obj["dimensions_m"] == [10, 20, .2]
    assert obj["position_m"] == [0, 0, -.1]


def test_mesh_approximation_needs_explicit_override(tmp_path):
    stage = stage_at(tmp_path)
    mesh = UsdGeom.Mesh.Define(stage, "/World/Mesh")
    import trimesh
    data = trimesh.creation.box()
    mesh.CreatePointsAttr(data.vertices.tolist())
    mesh.CreateFaceVertexCountsAttr([3] * len(data.faces))
    mesh.CreateFaceVertexIndicesAttr(data.faces.flatten().tolist())
    UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())
    api = UsdPhysics.MeshCollisionAPI.Apply(mesh.GetPrim())
    api.CreateApproximationAttr("convexDecomposition")
    stage.GetRootLayer().Save()
    with pytest.raises(ValueError, match="approximation"):
        import_usd(stage.GetRootLayer().realPath, tmp_path / "failed")
    scene, report = convert(stage, tmp_path, mesh_policy="convex-hull")
    assert scene["objects"][0]["convex"]
    assert any("overridden" in w for w in report["warnings"])


def test_usdz_and_existing_output(tmp_path):
    stage = stage_at(tmp_path)
    cube(stage)
    stage.GetRootLayer().Save()
    package = tmp_path / "scene.usdz"
    assert UsdUtils.CreateNewUsdzPackage(Sdf.AssetPath(stage.GetRootLayer().realPath), str(package))
    result = import_usd(package, tmp_path / "imported")
    assert result["objects"] == 1
    before = (tmp_path / "imported/scene.json").read_bytes()
    with pytest.raises(ValueError, match="already exists"):
        import_usd(package, tmp_path / "imported")
    assert (tmp_path / "imported/scene.json").read_bytes() == before


def test_imported_wall_collision_and_costs(tmp_path):
    pytest.importorskip("pinocchio")
    from prmkit.backends import make_backend
    from prmkit.geometry import sample_edge, certify
    from prmkit.metrics import Metrics
    stage = stage_at(tmp_path)
    cube(stage, position=(.5, 0, 0), dims=(.001, .2, .2))
    scene, _ = convert(stage, tmp_path)
    urdf = tmp_path / "slider.urdf"
    urdf.write_text('''<robot name="slide"><link name="base"/><link name="tool"><collision><geometry><sphere radius="0.01"/></geometry></collision></link><joint name="x" type="prismatic"><parent link="base"/><child link="tool"/><axis xyz="1 0 0"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint></robot>''')
    config = dict(robot=dict(backend="urdf", urdf=str(urdf), tool_frame="tool"), environment=scene,
                  metrics=[dict(name="ee_length"), dict(name="clearance", threshold=.2)])
    backend = make_backend(config)
    crossing = sample_edge(backend, np.array([.1]), np.array([.9]), 1, {})
    assert np.all(crossing.distances > .3)
    assert certify(backend, crossing, dict(margin=.01, guard=2e-6, max_depth=18)) is None
    approaching = sample_edge(backend, np.array([.1]), np.array([.45]), 16, {})
    cost = Metrics(config).evaluate(approaching)
    assert cost[0] == pytest.approx(.35)
    assert cost[1] > 0
    camera = dict(frame="tool", offset_m=[.02, 0, 0], rotation=[[0, 0, 1], [1, 0, 0], [0, 1, 0]], target=[1, 0, 0])
    assert backend.visibility(np.array([.1]), camera) == 0
    assert backend.visibility(np.array([.8]), camera) == 1


def test_cli_import_then_real_ur5e_pipeline(tmp_path):
    pytest.importorskip("pinocchio")
    from prmkit.pipeline import run, validate_path
    project = tmp_path / "project"
    assert main(["init", str(project), "--template", "isaac"]) == 0
    assert main(["import-usd", str(project / "scene.usda"), "--output", str(project / "imported")]) == 0
    config = load(project / "experiment.yaml")
    result = run(config, project / "run", quiet=True)
    assert result["complete"] and result["paths"] > 0
    assert validate_path(config, project / "run/paths.npz")["valid"]
