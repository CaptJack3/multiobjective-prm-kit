# Run research cases from one file

For a standalone Hebrew walkthrough with selectable examples, copyable commands
and a run-count calculator, open [the HTML guide](RUN_CASE_GUIDE_HE.html).

Open **`run_experiment.py`** in the distribution root. Edit `CASE_ID` and the
single `SETTINGS` block. In an activated Python environment on Linux/WSL:

```bash
python -m pip install '.[robot]'
python run_experiment.py --plan
python run_experiment.py
```

There is no `init` step and no YAML editing. The supplied script explicitly
selects `profile="mounted_ur5e"` (Robotiq/D435/PickNik), three objectives and an
independent mesh postcheck. Public `run_case` keeps `bare_ur5e` as its default
for compatibility. The script has two walls, two positions per wall, N=24, K=6, one query and two
epsilons: **4 environments, 4 PRM builds, 8 searches**. Its short query checks
the workflow; it is not the large research experiment. No Isaac/GPU is required.

| `case_id` | What it does |
|---|---|
| `wall_grid` | Cartesian combinations of wall positions and dimensions |
| `wall_random` | Seeded sampling from ranges or discrete choices |
| `wall_sobol` | Seeded Sobol sampling from ranges or discrete choices |
| `fixed_environment` | Same scene, different N/K/epsilons/tasks/seeds |
| `environment_list` | Same experiment settings across supplied scenes |

The same entry point is available directly:

```python
from prmkit import run_case, list_cases

plan = run_case(
    "wall_grid", wall_count=2, grid_points=3,
    widths=[0.4, 0.8], heights=[0.6, 0.9],
    n=[100, 1000], k=[8, 16], epsilons=[0, 0.05, 0.1],
    num_queries=5, max_environments=20,
    output="runs/my_study", dry_run=True,
)
print(plan)  # Remove dry_run=True to execute.
```

`grid_points` means positions **per wall**, not the total environment count.
This full grid has `(3×2×2)² = 144` scenes; the budget takes the first 20, for
20×5×2×2×3 = 1,200 searches. N/K/epsilon and optional `seeds=[7,8]` form a
Cartesian product. A capped grid is a prefix in parameter order, not a random
sample. The plan reports counts and `grid_exhausted`. `max_environments` limits
scenes, not total searches. `dry_run` writes nothing and builds no graph.

## Re-run a selected scene more densely

```python
run_case(
    "fixed_environment",
    environment="runs/my_study/environments/env_00003/scene.json",
    queries="runs/my_study/queries.json",
    n=[1000, 5000], k=[16, 32], query_k=32, epsilons=[0, 0.05],
    output="runs/denser_env_3",
)
```

Passing the saved bank preserves the same tasks. To create one fixed scene
directly, replace `environment` with `wall_count=2, widths=0.6, heights=0.9`.
Fixed generated walls require one value per field. An existing experiment can
be used with `base_experiment="path/experiment.yaml"`; in `fixed_environment`
its scene is preserved unless explicit geometry is supplied. This also accepts
a generated candidate YAML from an older sweep.

## Heterogeneous walls and random sizes

Default walls surround the base on a circle of `radius=1.0` m, with outward
normals. `offset_range=(-0.2,0.2)` moves each along its normal. For a corridor,
unequal dimensions or custom orientations, provide `walls` instead of a count:

```python
walls = [
    dict(name="east", center_xy=[1, 0], move_axis=[1, 0],
         offsets=[-0.2, 0, 0.2], widths=[0.4, 0.8], heights=[0.6, 0.9],
         thicknesses=[0.08], yaw_degs=[0, 15]),
    dict(name="north", center_xy=[0, 1], move_axis=[0, 1],
         offsets=[-0.1, 0.1], widths=[0.5], heights=[1.0], yaw_degs=[90]),
]
run_case("wall_grid", walls=walls, max_environments=48,
         n=100, k=8, output="runs/unequal_walls")
```

Per-wall fields override shared settings. `move_axis` is normalized and stays
fixed while yaw changes. Width is local Y, thickness local X, height local Z;
lengths are metres and this interface's yaw angles are degrees. Centre Z is
`base_z + height/2`, so heights change while the bottom stays fixed (default
`base_z=0`). `yaw_offsets=[-15,0,15]` changes default yaw for every wall.
Use `offset_range=(0,0)` to vary dimensions without moving walls.

```python
run_case("wall_random", wall_count=3, max_environments=20,
         widths={"min": 0.3, "max": 0.9}, heights={"min": 0.5, "max": 1.2},
         offset_range=(-0.25, 0.25), seed=42, n=100, k=8,
         output="runs/random_walls")
```

Random ranges are uniform; lists are discrete choices; duplicates are possible.
`grid_points` does not discretize Random. This is not an adaptive optimizer;
wall–wall overlap constraints are not inferred.

## Two-wall XY grids without PRM

The standalone [XY example generator](../examples/generate_two_wall_xy_grid.py)
enumerates wall segments whose **endpoints** lie on a two-dimensional grid,
then forms unordered pairs of walls. This differs from `wall_grid` above,
where each wall moves along one fixed axis. It only generates geometry.

Edit [two_wall_xy_grid.yaml](../examples/two_wall_xy_grid.yaml) to set X/Y grid
coordinates, widths, heights, thickness, X/Y orientations and the bottom Z.
Width is the horizontal segment length; it must connect two grid points.
The example uses 20/60 cm widths and 25/90 cm heights on a 4-by-4 grid with
20 cm spacing. It has 64 individual wall variants and 2,016 unordered pairs;
excluding touching or overlapping boxes leaves 1,424 two-wall scenes.

```bash
python examples/generate_two_wall_xy_grid.py examples/two_wall_xy_grid.yaml --plan
python examples/generate_two_wall_xy_grid.py examples/two_wall_xy_grid.yaml \
  --output runs/my_two_wall_xy_space
```

Use a new output directory for each generation. Open its `index.html` gallery,
`candidate_grid.png`, or `examples_3d.png`. Each `environments/env_XXXXX/`
contains an ordinary `scene.json` and an SVG top view. PNG figures require the
`render` extra; `--no-plots` retains the HTML gallery and SVGs without it.
The generator checks wall-wall overlap only. It does not create a robot, table,
target, queries or a PRM, and makes no robot-feasibility claim. Generated scene
files can later be supplied to `fixed_environment` or `environment_list`.

## Queries, metrics and robots

A query is one independent planning task: a start plus one or more alternative
goals. It is not N, `query_k` or the number of returned Pareto paths.
Provide `queries=[{"start": start_vector, "goal": goal_vector}, ...]` with
numerical joint vectors, in URDF order (radians for revolute joints).
`goals` and IK `target` forms are also supported. Omit `num_queries` to use the
entire supplied list. JSON banks and NPZ `starts`/`goals` matrices (or a shared
`start` vector plus `goals`) are accepted through the same `queries` argument.

Without a bank, template goals become independent tasks first; extra queries
use seeded random collision-free endpoints in the **first scene**. The bank is
then fixed across all scenes and planner settings. Endpoints need not be
connected, and another scene can invalidate them. Such tasks are reported as
`invalid_query`, not resampled. `query_attempts=10000` bounds sampling trials;
exhaustion fails explicitly. Save `queries.json` for later comparisons.

`metrics` accepts built-in names or full metric dictionaries, including existing
Python plug-ins. For example:

```python
run_case("fixed_environment", wall_count=2, n=100, k=8,
         metrics=["ee_length", {"name": "risk", "kind": "clearance",
                   "threshold": 0.15, "power": 3, "scope": "world"}],
         rank_by="best.risk", rank_direction="min",
         validation={"margin": 0.01}, search={"timeout_s": 120},
         output="runs/changed_cost")
```

`metrics` defines path costs; `rank_by` selects environment scoring. Supply
`robot={...}` / a robot-definition filename plus matching queries for another
robot, or `base_experiment=...` to inherit its robot, camera, tasks and metrics.
N/K/epsilon and worker arguments override inherited settings. Relative explicit
inputs use `input_dir` (default working directory); referenced files resolve
relative to their owner. Use `environment_list` with `environments=[...]` to
compare supplied scenes. Imported Isaac `scene.json` is also accepted; its mesh
files must remain available. Saved scenes retain resolved mesh paths.

## Results and resume

Open `output/report.html`: it has N/K/epsilon/seed groups, query/status filters,
environment rankings and links to individual reports. Other outputs are
`results.csv`, `study_summary.json`, `plan.json`, `queries.json`, reusable
`environments/env_XXXXX/scene.json`, generated `configs/run_XXXXXXX.yaml`, and
ordinary pipeline outputs under `runs/run_XXXXXXX/`.

Rankings are separate per N/K/epsilon/seed. Eligible environments have all
queries evaluated to completion and at least one solved query. More solved
queries ranks first, then the mean selected score over solved queries. Partial
or failed runs remain visible. `path_count` depends on solver settings and is
not a geometric diversity measure. Completion does not mean every task has a path.

Only changing epsilon reuses the verified graph and costs (hard-linked on the
same filesystem, with a copy fallback). New N/K, query, seed or scene rebuilds the graph; there is no
incremental densification or shared multi-query roadmap here. Study jobs are
sequential; `workers` parallelizes edge work. Search timeouts apply per search,
not to graph construction or the entire study. Exact epsilon 0 is exact on the
constructed graph, not in continuous configuration space.

Resume unchanged inputs with `python run_experiment.py --resume` or
`run_case(..., resume=True)`. Changed inputs, code or dependencies require a new
output directory, e.g. `python run_experiment.py --output runs/revision_2`.
Generated inputs and completed run files are integrity-checked.

Alternatively, without editing Python:

```bash
prmkit case wall_grid --walls 2 --grid-points 2 --n 24 --k 6 \
  --epsilons 0 0.05 --num-queries 1 --max-environments 4 --output runs/case_cli
```

Add `--dry-run` to preview or `--resume` to continue identical inputs. Python
exposes the full option set. See [the architecture checkpoint](CASE_RUNNER_DESIGN.md).
