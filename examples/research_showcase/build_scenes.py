"""Rebuild editable USD environments from bundled specifications and robot templates.

Requires numpy and usd-core, but neither prmkit nor a running Isaac installation.
The archived robot, cameras and lights are retained; the environment is authored
from JSON. Exported scenes are self-contained and can be opened in Isaac Sim.
Shape authoring is adapted from experiments/demo_extensions/dense/render_scenes.py.
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade

HERE = Path(__file__).resolve().parent


def identifier(name):
    # Match the original author's spelling, including negative/decimal names.
    return name.replace('-', 'm').replace('.', 'p')


def material(stage, name, color, metal=0., rough=.45):
    path = '/World/RebuiltMaterials/' + identifier(name)
    mat = UsdShade.Material.Define(stage, path)
    shader = UsdShade.Shader.Define(stage, path + '/surface')
    shader.CreateIdAttr('UsdPreviewSurface')
    shader.CreateInput('diffuseColor', Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*color))
    shader.CreateInput('metallic', Sdf.ValueTypeNames.Float).Set(float(metal))
    shader.CreateInput('roughness', Sdf.ValueTypeNames.Float).Set(float(rough))
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), 'surface')
    return mat


def mesh(stage, path, vertices, faces):
    obj = UsdGeom.Mesh.Define(stage, path)
    obj.CreatePointsAttr(vertices)
    obj.CreateFaceVertexCountsAttr([len(f) for f in faces])
    obj.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    obj.CreateSubdivisionSchemeAttr('none')
    obj.CreateDoubleSidedAttr(True)
    return obj


def lathe(profile, n=48):
    vertices = [[r*math.cos(2*math.pi*i/n), r*math.sin(2*math.pi*i/n), z]
                for r, z in profile for i in range(n)]
    faces = [[j*n+i, j*n+(i+1)%n, (j+1)*n+(i+1)%n, (j+1)*n+i]
             for j in range(len(profile)-1) for i in range(n)]
    return vertices, faces


def tube(points, radius, n=10):
    points = np.asarray(points, dtype=float)
    vertices = []
    for i, center in enumerate(points):
        z = points[min(i+1, len(points)-1)] - points[max(0, i-1)]
        z /= np.linalg.norm(z)
        x = np.cross(z, np.eye(3)[np.argmin(abs(z))])
        x /= np.linalg.norm(x)
        y = np.cross(z, x)
        vertices.extend([(center + radius*(math.cos(a)*x + math.sin(a)*y)).tolist()
                         for a in np.arange(n)*2*math.pi/n])
    faces = [[i*n+j, i*n+(j+1)%n, (i+1)*n+(j+1)%n, (i+1)*n+j]
             for i in range(len(points)-1) for j in range(n)]
    return vertices, faces


def cube(stage, path, position, dimensions):
    obj = UsdGeom.Cube.Define(stage, path)
    obj.CreateSizeAttr(1.)
    xf = UsdGeom.XformCommonAPI(obj)
    xf.SetTranslate(Gf.Vec3d(*position))
    xf.SetScale(Gf.Vec3f(*dimensions))
    return obj


def household(stage, spec):
    if spec['robot_base_world_m'] != [0, -.08, .86]:
        raise ValueError('Bundled robot templates use base [0, -0.08, 0.86]; changing the base also requires relocating the robot and cameras.')
    materials = {n: material(stage, n, *v) for n, v in spec['materials'].items()}
    for g in spec['geometry']:
        path = '/World/Scene/' + identifier(g['name'])
        shape, size = g['shape'], g['size']
        if shape == 'box':
            obj = cube(stage, path, g['position_m'], size)
        elif shape == 'cylinder':
            obj = UsdGeom.Cylinder.Define(stage, path)
            obj.CreateRadiusAttr(size[0]); obj.CreateHeightAttr(size[1])
            obj.CreateAxisAttr(g.get('axis', 'Z'))
        elif shape == 'sphere':
            obj = UsdGeom.Sphere.Define(stage, path)
            obj.CreateRadiusAttr(1.)
            UsdGeom.XformCommonAPI(obj).SetScale(Gf.Vec3f(*size))
        elif shape == 'lathe':
            obj = mesh(stage, path, *lathe(g['profile']))
        elif shape == 'tube':
            obj = mesh(stage, path, *tube(g['points'], size[0]))
        else:
            raise ValueError(f'Unsupported shape: {shape}')
        UsdGeom.XformCommonAPI(obj).SetTranslate(Gf.Vec3d(*g['position_m']))
        UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(materials[g['material']])
        obj.GetPrim().CreateAttribute('showcase:semanticName', Sdf.ValueTypeNames.String).Set(g['name'])
    for g in spec['colliders']:
        obj = cube(stage, '/World/CollisionProxies/' + identifier(g['name']),
                   g['position_m'], g['dimensions_m'])
        obj.CreateVisibilityAttr('invisible')
        UsdPhysics.CollisionAPI.Apply(obj.GetPrim())
    for target in spec['targets']:
        obj = UsdGeom.Xform.Define(stage, '/World/Targets/' + identifier(target['name']))
        obj.AddTranslateOp().Set(Gf.Vec3d(*target['tcp_world_m']))


def walls(stage, spec):
    for g in spec['objects']:
        obj = cube(stage, '/World/Scene/' + identifier(g['name']),
                   g['position_m'], g['dimensions_m'])
        mat = material(stage, g['name'], g.get('color_rgb', [.42, .47, .54]))
        UsdShade.MaterialBindingAPI.Apply(obj.GetPrim()).Bind(mat)
        if g.get('collider', True):
            UsdPhysics.CollisionAPI.Apply(obj.GetPrim())


def main():
    catalog = json.loads((HERE / 'catalog.json').read_text())
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--scene', choices=['all'] + [r['id'] for r in catalog], default='all')
    parser.add_argument('--output', type=Path, required=True, help='A new directory')
    parser.add_argument('--spec-root', type=Path, help='Optional folder containing SCENE/scene_spec.json (household) or SCENE/scene.json (walls)')
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    for row in catalog:
        if args.scene not in ('all', row['id']):
            continue
        source = HERE / row['static_scene']
        spec_file = HERE / row['geometry_spec']
        if args.spec_root:
            spec_file = args.spec_root / row['id'] / spec_file.name
        spec = json.loads(spec_file.read_text())
        stage = Usd.Stage.Open(str(source))
        for path in ['/World/Scene', '/World/CollisionProxies', '/World/Targets', '/World/TrajectoryGuides']:
            stage.RemovePrim(path)
        (household if 'geometry' in spec else walls)(stage, spec)
        stage.SetStartTimeCode(0); stage.SetEndTimeCode(0)
        stage.SetTimeCodesPerSecond(24); stage.SetFramesPerSecond(24)
        UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
        UsdGeom.SetStageMetersPerUnit(stage, 1.)
        output = args.output / (row['id'] + '.usdc')
        stage.GetRootLayer().Export(str(output))
        print(output)


if __name__ == '__main__':
    main()
