# Common experiment recipes

Activate `.venv` and run from the repository root. Give each changed experiment
its own output directory. Start with small graphs; the high-N example below is
configuration guidance, not a new performance benchmark.

## One wall, with a larger roadmap

```bash
prmkit run examples/01_single_wall.yaml --output runs/wall_n15000_k10 \
  --set roadmap.n_vertices=15000 --set roadmap.k_neighbors=10 --dry-run
```

Remove `--dry-run` to execute. N counts sampled vertices; start and goal vertices
are additional. K controls neighbor proposals, while `roadmap.query_k` controls
query connections. They need not have the same value. Cost and collision checking
can dominate runtime; a search can finish without a path or reach its budget.

## Generate a two-wall XY grid without PRM

The supplied geometry-only example combines two widths (0.20/0.60 m), two heights
(0.25/0.90 m) and wall endpoints on a 4×4 XY grid. Removing overlapping/touching
pairs leaves **1,424** environments.

```bash
python examples/generate_two_wall_xy_grid.py examples/two_wall_xy_grid.yaml --plan
python examples/generate_two_wall_xy_grid.py examples/two_wall_xy_grid.yaml \
  --output runs/two_wall_space
```

Open `runs/two_wall_space/index.html`. Every scene has `scene.json` and
`preview.svg` in `environments/env_XXXXX/`; the root also has
`candidate_grid.png` and `examples_3d.png`. Edit the YAML to change the grid and
sizes. Use `--no-plots` if you installed only core dependencies.

## Generate random scenes without planning

```bash
prmkit generate examples/03_multi_wall_random.yaml --output runs/random_scenes
```

This writes four scenes, each with three walls, plus XY previews. It does not
load the robot or build a graph. See [the distinction between generators](CASES.md).

## Compare several environments and planner settings

Edit `CASE_ID` and `SETTINGS` in `run_experiment.py`, then:

```bash
python run_experiment.py --plan
python run_experiment.py --output runs/wall_study
```

The default is four two-wall environments, one query and two epsilon values:
four PRM builds and eight searches. Open `runs/wall_study/report.html`.

For an explicit list of already-created scenes, the Python entry point is:

```python
from prmkit import run_case

plan = run_case(
    'environment_list',
    environments=['runs/two_wall_space/environments/env_00000/scene.json',
                  'runs/two_wall_space/environments/env_00001/scene.json'],
    profile='mounted_ur5e', n=100, k=8, query_k=12,
    epsilons=[0, 0.05], num_queries=1,
    output='runs/two_selected_scenes', dry_run=True,
)
print(plan)
```

Remove `dry_run=True` to execute. Supply explicit `queries` for your actual
research task; a generated scene alone does not define a meaningful task.
This low-level call does not request independent postchecks by default; add
`verification={'enabled': True, 'backend': 'coal_mesh', 'min_clearance_m': 0.005}`
when you want them. See [case options](CASES.md).

## Keep a task and change N/K

After the quickstart has produced `runs/first_wall`:

```bash
prmkit rerun runs/first_wall --output runs/density_study --n 100 500 --k 8 16
prmkit verify runs/density_study --backend coal_mesh --min-clearance-m 0.005
```

Saved scenes and queries are preserved; new graphs are built. `rerun` does not
implicitly provide the independent mesh verification requested in the second
command.

## View and export a planned path

```bash
prmkit explore runs/first_wall
prmkit select runs/first_wall --max-per-query 1 --output runs/first_wall_selection.json
prmkit render runs/first_wall --selection runs/first_wall_selection.json \
  --only-verified --frames 24 --fps 8 --output runs/first_wall_video
prmkit export runs/first_wall --selection runs/first_wall_selection.json \
  --only-verified --usd --output runs/first_wall_export
```

Rendering makes a CPU mesh preview and may take substantially longer than the
small planning example. Open the exported USD in Isaac Sim for geometric replay.
`select` defaults to verified candidates and may return an empty selection if no
candidate passed verification. The explorer also lets you download a manual
selection. See [result files and labels](RESULTS.md).

## Import a saved Isaac/USD scene

This complete example creates its own small USD fixture:

```bash
prmkit init runs/isaac_example --template isaac
prmkit import-usd runs/isaac_example/scene.usda --output runs/isaac_example/imported
prmkit run runs/isaac_example/experiment.yaml --output runs/isaac_example/result
```

For your stage, replace the source USD, align the robot base, exclude its robot
subtree if needed, and configure start/goal and camera. Read the
[static import contract](ISAAC_IMPORT.md) before importing a full simulator stage.

## Open or rebuild kitchen, workbench and shelves

Open `examples/research_showcase/index.html` locally, or rebuild all bundled
scene geometries without PRM:

```bash
python examples/research_showcase/build_scenes.py --output runs/rebuilt_scenes
```

The output USD files can be opened in Isaac. For editable procedural dimensions,
archived route plots and hardware revisions, see [research examples](RESEARCH_EXAMPLES.md).

## Continue an interrupted run

```bash
prmkit run examples/01_single_wall.yaml --output runs/first_wall --resume
```

Use unchanged inputs, code and dependencies. Completed graph work is retained;
an interrupted in-memory search may need to restart. Changing settings requires
a new output directory. `prmkit status runs/first_wall` reports saved progress.
