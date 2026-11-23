# Experiment configuration

`schema_version: 1` uses YAML, or JSON (a YAML subset). Input paths are relative
to the file declaring them, not the shell's working directory. A robot YAML's
URDF/mesh paths are relative to that robot YAML. Values are SI units: metres,
radians for revolute joints, metres for prismatic joints. Joint order comes from
the URDF and is printed by `prmkit check`.

Unknown top-level, robot, planner, search and metric keys fail early. Use
`--set prm.n=500 --set search.epsilon=0` for a temporary scalar override. For
list entries, scene details and plug-ins, edit the project YAML/JSON.

```yaml
schema_version: 1
seed: 7
robot: assets/ur5e/robot.yaml
environment: assets/E4/scene.json
query:
  file: assets/E4/queries.npz  # arrays start [d], goals [g,d]
prm:
  n: 1000                   # accepted free samples, excluding query vertices
  k: 24                     # neighbors attempted for each sample
  query_k: 64                # neighbors attempted for start and each goal
  sampler: sobol             # or random
  max_draws: 1000000         # reject exhausted sampling explicitly
  workers: 1                # separate CPU processes for edge chunks
  edge_chunk: 256            # committed checkpoint size
validation:
  margin: 0.01              # strictly exceeded by continuous pair distances
  guard: 0.000002
  intervals: 16
  max_depth: 18              # unresolved intervals are rejected
metrics:
  - name: ee_length
    intervals: 16
  - name: clearance
    threshold: 0.1
    power: 2
    scope: world
    intervals: 16
  - name: visibility
    intervals: 32
camera:
  frame: tool0
  offset_m: [0.1, 0, 0.03]
  rotation: [[0,0,1], [1,0,0], [0,1,0]]
  fov_deg: [90, 90]
  rays: 64
  target_object: goal
search:
  epsilon: 0.05
  engine: auto
  timeout_s: 300
  max_labels: 500000
```

`N` is the number of accepted random/Sobol samples; the start and valid distinct
goals are additional vertices. The kNN distance is `norm(weights * (q1-q0))`.
Edges use straight interpolation in bounded joint coordinates. No angle wrapping
is performed. Neighbor proposals form an undirected union: final degree can
exceed K. Collision rejection can reduce it. K is clamped to available neighbors.
Both directions are exported as arcs; directional plug-ins are evaluated twice.

## Queries

Use either explicit `start` and `goals`, an NPZ `file`, or a `start` plus Cartesian
`target`. The latter uses seeded, bounded least-squares IK with free orientation:

```yaml
query:
  start: [0.2, 0.4]
  target: [0.35, 0.35, 0.15]
  ik_attempts: 100
  max_goals: 16
  ik_tolerance: 0.001
```

This example fits the two-link template, not UR5e. Invalid start states fail;
collision-invalid goal alternatives are filtered and counted. Incorrect dimensions
or joint limits are errors. IK exhaustion is not a proof of workspace infeasibility.
Multiple goals are joined to a virtual terminal vertex by zero-cost arcs, removed
from exported physical paths.

## Built-in metrics

For one edge, let `q(t)=q0+t(q1-q0)` and `Lq=norm(W*(q1-q0))`. All objectives are
minimized and summed along a path. `scale` multiplies one metric by a positive
constant. Each metric has its own quadrature `intervals` (default: validation
intervals); continuous collision certification is independent of those samples.

| Kind | Edge cost |
|---|---|
| `ee_length` | Sum of distances between sampled Cartesian tool positions |
| `joint_length` | `Lq` |
| `clearance` | `Lq * integral(max(0, 1-d(t)/threshold)**power, t=0..1)` |
| `visibility` | `Lq * integral(1-V(t), t=0..1)` |

Integration uses the trapezoidal rule, including both endpoints. Clearance
`scope: world` uses the minimum world-pair distance; `all` includes self pairs.
An empty pair set has infinite clearance and zero risk. The validity margin
never changes when a risk threshold or cost weight changes.

Optional metric `unit` is recorded in `summary.json`. Joint-integrated costs use
weighted joint-coordinate units; choose weights deliberately when mixing
revolute and prismatic coordinates. Unit metadata does not convert values.

For URDF robots, visibility uses camera axes right/down/forward, a rectangular
FOV, deterministic target-surface points, and CPU ray tests against nominal
URDF collision meshes, hardware and world colliders. Rays begin 3 mm beyond the
optical center and stop short of the target. The target object is normally
`collider: false`. `camera.target: [x,y,z]` instead uses one point. The point
demo's visibility is only a line-of-sight fraction; it has no wrist orientation.

A metric can be renamed with `name` while selecting its computation with `kind`:

```yaml
metrics:
  - {name: short_motion, kind: ee_length}
  - {name: proximity_linear, kind: clearance, threshold: 0.15, power: 1}
  - {name: proximity_quadratic, kind: clearance, threshold: 0.08, power: 2}
  - {name: joint_effort_proxy, kind: joint_length}
```

This is four objectives; `auto` uses the generic Python engine. A static scale
does not change componentwise relative-epsilon meaning. Nonadditive path scores
(for example minimum clearance as a bottleneck) cannot be substituted directly
into this additive solver; compute them after planning or implement an appropriate
search model.

## Search status

`epsilon: 0` selects Exact, retaining distinct equal-cost simple paths. Positive
epsilon selects A*pex. A completed approximate result covers the finite graph's
Pareto vectors componentwise within `(1+epsilon)` and numerical tolerance;
it is not a claim about the continuous configuration-space Pareto set.

`auto` uses the preserved compiled kernel for exactly three objectives if Numba
is installed, otherwise the Python port. `research` explicitly requests that
three-objective kernel; `python` supports any positive number of objectives.
The latter generalizes the corrected research algorithm and has exhaustive
small-graph coverage tests. Results may differ in tie-breaking between engines.

Timeout or label limits set `complete: false` and CLI exit code 3; any saved paths
are partial. The approximate-front guarantee applies only after completion.
Exit 4 means a completed search found no path in this roadmap. Exit 2 means an
input/runtime error. Exit 130 means interruption. Core/optional imports can be
inspected with `doctor`; missing robot extras do not prevent the point demo.

