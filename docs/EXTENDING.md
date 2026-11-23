# Extend without editing the planner

## A new metric

Put a Python file beside `experiment.yaml`:

```python
# custom_metric.py
import numpy as np

def elevation_exposure(samples, parameters):
    floor = parameters.get("floor_m", 0.2)
    penalty = np.maximum(0.0, samples.ee[:, 2] - floor)
    return float(np.trapezoid(penalty, x=samples.t) * samples.joint_length)
```

Add it to the metrics list:

```yaml
- name: elevation
  plugin: custom_metric.py:elevation_exposure
  parameters: {floor_m: 0.25}
  symmetric: true
  intervals: 32
```

The callable receives `EdgeSamples`: `q`, normalized `t`, `ee` positions,
per-pair `distances`, boolean `world_mask`, `weights`, `joint_length`, `backend`
and `camera`. Return one finite nonnegative scalar. Costs must be additive along
the path and depend only on this edge/state, not on prior search history. All
objectives are minimized. Setting a negative scale to maximize a reward is not
supported; define a meaningful nonnegative loss instead.

Custom metrics default to **directional**. Declare `symmetric: true` only if
reversing the same edge always preserves cost. Otherwise both directions are
evaluated separately. Negative/NaN/infinite results stop the run. Files and
installed modules use `file.py:function` or `package.module:function`; plug-ins
are ordinary trusted Python code, not a sandboxed expression language.

Runnable source examples live in `examples/`. The same interface supports local
changes to a built-in formula, or replacing it entirely; no core edit is needed.

## A new environment

Point the experiment's `environment` field to a new JSON file. For a URDF robot:

```json
{
  "environment_id": "my_scene",
  "objects": [
    {"name": "obstacle", "shape": "box", "position_m": [0.5,0.2,0.3],
     "dimensions_m": [0.1,0.2,0.6], "rotation_rpy": [0,0,0.3]},
    {"name": "goal", "shape": "box", "position_m": [0.65,0,0.3],
     "dimensions_m": [0.05,0.05,0.05], "collider": false}
  ]
}
```

Supported shapes: `box` (`dimensions_m`), `sphere` (`radius_m`), `cylinder` and
`capsule` (`radius_m`, `length_m`, local Z axis), and `mesh` (`mesh`: relative
STL/OBJ path, optional positive xyz `scale`). All accept `position_m` and either
`rotation_rpy` in radians or a right-handed orthonormal `rotation` matrix.
Mesh geometry uses triangle collision; `convex: true` explicitly replaces it by
its convex hull. Mesh units must be metres after scaling. Closed, well-formed
meshes are needed for solid-obstacle interpretations; a triangle surface is not
an occupancy model of its interior.

The lightweight point backend accepts only axis-aligned boxes of its own 2D/3D
dimension. Use `prmkit import-usd scene.usd --output imported` for supported
static Isaac/USD collision scenes, then set `environment: imported/scene.json`.
Install the `usd` extra on the importing machine. Read the
[Isaac import guide](ISAAC_IMPORT.md) for units/frame alignment, robot exclusion,
supported geometry and explicit handling of ground planes/dynamic content.

## Another robot through URDF

```bash
prmkit init other_robot --template two_link
prmkit run other_robot/experiment.yaml --output other_robot/run
```

This ships an actual second URDF robot and exercises the same pipeline. To use
your robot, replace the URDF path, `tool_frame`, start/goals and weights. Include
collision geometry and finite joint limits in the URDF. `package_dirs` resolves
`package://` mesh references. The built-in backend supports fixed-base, bounded
one-coordinate revolute/prismatic joints and does not assume six joints.

By default, pairs on different moving joints are active, including adjacent
links. Explicitly declare intended mechanical contacts:

```yaml
robot:
  backend: urdf
  urdf: assets/my_robot.urdf
  tool_frame: my_tool
  weights: [1, 1, 0.8, 0.6]
  ignore_self_pairs: [[base_link, link1], [link1, link2]]
  ignore_world_pairs: [[base_link, table]]
```

Same-rigid-joint geometries are excluded automatically. Every other exclusion is
explicit and becomes part of provenance. `pair_policy` optionally supplies an
explicit active-link-pair list, as used by the shipped UR5e. Unknown pair names
fail rather than silently disabling checks. Optional `lower`/`upper` bounds may
narrow, never expand, URDF limits.

Mounted tools/cameras are listed under `robot.attachments`, each with unique
`name`, a URDF `parent` frame, primitive/mesh geometry, and local pose. They enter
collision checking, visibility and replay. The UR5e's explicit convex model is
specific to its links and must not be reused for another robot.

## A custom robot backend

Set `robot.backend: plugin` and `robot.plugin: my_backend.py:make_robot`.
The factory receives the robot mapping and resolved environment. Return an
object satisfying `prmkit.backends.Backend`:

- `lower`, `upper`, positive `weights`: arrays of length d; unique `joint_names`.
- `world_mask`: boolean array of length P defining each distance's role.
- `state(q)`: returns `State(distances[P], ee[3])`.
- `motion_bound(dq)`: P finite nonnegative bounds on pair-distance change per
  unit interpolation parameter over the entire edge.
- `visibility(q, camera)`: fraction in [0,1], needed only by visibility metrics.

The motion bound is a scientific contract. It must bound all geometry throughout
the edge, not just observed endpoint changes. The validator uses it to prove
continuous clearance; an incorrect plug-in bound invalidates that claim. The
shipped backends derive bounds from translation or enclosing body radii.

Floating bases, quaternion state spaces, continuous joint wrapping and
non-straight local motions require corresponding planner/interpolation support;
simply changing an array's dimension does not implement those semantics.

## Edit the kit itself

The distribution contains the complete new source. For core development, install
it in editable mode: `python -m pip install -e '.[robot,fast,dev]'`, then edit
`src/prmkit/` and run `python -m pytest`. This affects only your copy of the kit.
Ordinary installed wheels require reinstalling after source changes; local
metric/backend plug-ins are read directly and do not need reinstalling.

The recorded plug-in hash covers its entry source file. If your plug-in imports
other private helper files or data, version those inputs and change a version
parameter/new output directory when they change; arbitrary plug-in transitive
dependencies are not automatically discovered.

