# Use an environment authored in Isaac Sim

The `import-usd` command converts a saved **static USD scene** into the ordinary
planning environment. Scene authors can work in Isaac; planning researchers
receive a portable folder and run PRM, collision checks, costs, Pareto search
and environment sweeps without an Isaac installation. The robot remains a
separate URDF. Import does not reproduce PhysX simulation or RTX measurements.

## First complete example

From the distribution directory, in an activated Python environment:

```bash
python -m pip install '.[robot,usd]'
prmkit init my_isaac_project --template isaac
prmkit import-usd my_isaac_project/scene.usda --output my_isaac_project/imported
prmkit check my_isaac_project/experiment.yaml
prmkit run my_isaac_project/experiment.yaml --output my_isaac_project/run
```

Open `my_isaac_project/run/report.html`. The bundled USD is a small illustrative
scene, not a scene captured from an Isaac runtime. The template uses a bare UR5e,
24 roadmap samples and length/clearance costs; it is an installation check.
For a new scene, replace the import command's source with your saved file.
Set `query.start`/`goals` for your task, and add camera/attachments if present.
Changing the environment alone does not define a meaningful robot task.

For an already installed wheel, install `multiobjective-prm-kit[robot,usd]` at
the same version. The `usd` extra is needed **only on the importing machine**.
The receiving planning machine needs the normal `robot` extra.

## Save the scene in Isaac

Save the stage locally as `.usd`, `.usda` or `.usdc`; `.usdz` is also accepted.
Include referenced USD files/payloads with their relative directory layout, or
export a flattened stage. Saving just the top-level file does not necessarily
include referenced geometry. Active composition errors stop import.

An optional one-time export in Isaac's Script Editor is:

```python
import omni.usd
stage = omni.usd.get_context().get_stage()
stage.Load()  # load payloads before exporting
stage.Export("/absolute/path/to/environment.usdc")
```

`Stage.Export` flattens composed scene data. It does not bake PhysX simulation
state or package external textures. Export after placing obstacles at their
intended authored positions. For simulated positions, first write those poses
back to USD. Use the stage units consistently when assembling referenced assets.
See [OpenUSD Stage.Export](https://openusd.org/release/api/class_usd_stage.html)
and [Isaac scene setup and saving](https://docs.isaacsim.omniverse.nvidia.com/latest/python_scripting/environment_setup.html).

By default, enable collision on the actual obstacle geometry in Isaac. Invisible
collision proxies are imported too. Render-only/disabled geometry is listed as
skipped in `import_report.json`. If the scene deliberately has no collision
annotations, `--geometry all` explicitly treats every supported geometry prim
in the selected subtree as a collider, including hidden/disabled geometry.
Lights, cameras, materials and ordinary Xform containers are not obstacles.

## Select the environment and align it with the robot

If the stage also contains the robot, exclude its subtree. The planner loads
the robot from URDF; importing it as an obstacle would create a duplicate.

```bash
prmkit import-usd saved_scene.usdc --output my_isaac_project/imported \
  --root /World --exclude /World/Robot --robot-frame /World/Robot/Base
```

Choose the prim whose pose matches the **URDF base frame**, not merely any
container named Robot. `--robot-frame` puts obstacles in that frame, in metres.
That frame must be rigid, with no scale/shear/reflection. It may be inside the
excluded robot subtree. Without this option, output is stage world coordinates
converted to metres and Z-up; the URDF base is assumed at that output origin.
For Y-up scenes, the world conversion is `(x, y, z) -> (x, -z, y)`.

Only the selected `--root` subtree is imported. Repeat `--exclude` for more
subtrees. Incorrect paths fail rather than silently doing nothing. Local USD
references, payloads and native instances are composed, and nested transforms,
nonuniform mesh scales, mirrored meshes and reset transform stacks are applied.
Mesh coordinates retain OBJ double precision. Source layers are read only.

`metersPerUnit` and `upAxis` come from the stage. Missing metadata uses USD's
defaults and produces a warning; override incorrect metadata with
`--meters-per-unit 1 --up-axis Z`. Referenced assets with different units still
need the correct scale in their parent scene: this is the USD assembler's
responsibility. See [USD units](https://openusd.org/release/api/group___usd_geom_linear_units__group.html)
and [world transforms](https://openusd.org/release/api/class_usd_geom_xform_cache.html).

## Supported geometry and explicit choices

| Content | Handling |
|---|---|
| Cube | Exact analytic box; a sheared cube becomes an exact convex polyhedron |
| Sphere | Exact analytic sphere with uniform scale |
| Cylinder | Exact analytic cylinder; equal radial scale, independent axial scale |
| Capsule | Exact analytic capsule with uniform scale; USD X/Y/Z axis handled |
| Mesh | Triangle surface, with triangulation of planar simple concave polygons and hole faces respected |
| Mesh `convexHull` | Single solid convex hull of the transformed geometry |
| Plane / typical ground collider | Requires `--plane-size WIDTH LENGTH` in metres; becomes a finite slab below the plane |
| Native instances / local references | Supported, including instance child geometry and individual transforms |
| Nonuniform sphere/capsule, sheared curved shapes | Rejected; export an explicit triangle mesh |
| Cone, curves, points, custom geometry | Rejected when selected as collision geometry; export supported meshes |
| PointInstancer, articulation, skeleton, deformable, particle content | Rejected in the selected subtree; exclude or bake supported static geometry |
| Enabled rigid body / sampled transforms or geometry | Requires explicit `--snapshot-time TIME_CODE`; frozen authored USD state only |

Example for a ground plane:

```bash
prmkit import-usd saved_scene.usdc --output my_isaac_project/imported \
  --exclude /World/Robot --plane-size 10 10 --plane-thickness 0.1
```

The full moving robot must stay inside the slab footprint and above its bottom;
outside those bounds it is **not** an infinite ground plane. Use a sufficiently
large footprint/thickness for your workspace or model a finite floor explicitly.

Default `--mesh-policy authored` accepts USD mesh approximations `none` and
`convexHull`. Unsupported approximations (e.g. convex decomposition or SDF) stop
import. You can bake their individual collision meshes first, or deliberately
use `--mesh-policy triangles` / `--mesh-policy convex-hull`. The override is
recorded as a warning; it changes the obstacle model. PhysX-specific cooking
settings are not reproduced. See [USD mesh collision semantics](https://openusd.org/release/api/class_usd_physics_mesh_collision_a_p_i.html).

Triangle meshes are **surfaces**, not general solid-volume occupancy tests.
Keep starts outside closed obstacles and choose convex hulls when solid convex
objects are intended. A convex hull fills concavities. Mesh subdivision is not
evaluated: import uses the control cage and reports this when applicable.

Contact/rest offsets, collision groups/filter pairs, masses, friction and physics
materials do not transfer. Use `validation.margin` and explicit robot collision
pair settings in the planning config. Collision geometry supports geometric
planning; equivalence to every PhysX collision result is not claimed.

## Outputs and handoff to another researcher

```text
imported/
  scene.json           environment, source hash, frame and import settings
  meshes/*.obj         local geometry assets (only when needed)
  import_report.json   prim-to-object map, skipped items, warnings, layer hashes
```

Review `import_report.json`, then copy **the entire folder**. `scene.json` uses
relative mesh paths; the original USD and Isaac installation are no longer
required. Source absolute filenames in the report are provenance, not runtime
dependencies. An existing destination is never overwritten. A rejected import
does not publish a partial environment.

The new `isaac` template already uses `environment: imported/scene.json`. In an
existing project, change that field, using a path relative to the experiment:

```yaml
environment: imported/scene.json
```

Imported objects keep their full USD prim path as a stable name. An old setting
such as `ignore_world_pairs: [[base_link_inertia, table]]` must be removed or
deliberately updated to the imported object's name. The `isaac` template avoids
E4-specific table exceptions and camera targets.

## Costs, camera and environment search

After import, the normal URDF/Pinocchio/Coal and CPU-ray pipeline runs unchanged.
No physics time stepping or rendered image is used to calculate the built-in
length, clearance or visibility costs. Add visibility only after defining its
camera frame/offset/FOV and target in the imported planning coordinate system:

```yaml
camera:
  frame: tool0
  offset_m: [0.1, 0, 0.03]
  rotation: [[0,0,1], [1,0,0], [0,1,0]]
  fov_deg: [90, 90]
  target: [0.6, 0, 0.4]
metrics:
  - name: ee_length
  - {name: clearance, threshold: 0.1, power: 2}
  - {name: visibility, intervals: 32}
```

These are example optics, not automatic calibration. `target` above is a single
point. For sampled target surfaces, `target_object` currently requires a box
with `dimensions_m`; use its full imported prim name or add a non-collider target
box. Render-only meshes, textures, lighting and transparency do not affect the
current visibility metric; it uses imported collision geometry.

Import several scenes into separate folders and run the same experiment with
different `--set environment=...` values and output directories. Keep robot,
query, seed, N/K/epsilon and metric definitions fixed for a controlled comparison.
For parametric changes within an imported scene, `prmkit sweep` can vary its
resolved object positions/dimensions just like a JSON-authored scene. Mesh scale
or rotation fields can be added explicitly if needed. Scene import does not
infer a family of environments or automatically search the Isaac stage graph.
See [environment spaces](ENVIRONMENT_SPACES.md).

## Validation scope

Tests use OpenUSD-authored stage fixtures, including USDZ, local references,
native instances, hidden colliders, mirrored and sheared transforms, malformed
imports, a thin obstacle collision test, costs/visibility and a complete UR5e PRM.
Isaac-specific schema names are checked even when the standalone OpenUSD build
does not register those schemas. No new Isaac GUI/RTX or PhysX session was run,
and no user-supplied production USD was available for acceptance testing.
