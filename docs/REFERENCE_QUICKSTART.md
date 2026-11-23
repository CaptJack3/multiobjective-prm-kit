# Extended reference and historical presets

**0.4: mounted UR5e → independent verification → Pareto selection → export/video.**
The new `mounted_ur5e` preset contains the exact frozen Robotiq 2F-85,
RealSense D435 and PickNik assembly used in the other research workflow.
The gripper is fixed open; camera transforms are nominal CAD calibration.

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[robot,render]'
prmkit run examples/01_single_wall.yaml --dry-run
prmkit run examples/01_single_wall.yaml
```

Open `runs/01_single_wall/explorer/explore.html`. The small example produces
saved scene/query/PRM candidates, a separate COAL mesh audit, linked Pareto
plots and a representative selection. Rendering is disabled by default.
`pareto` is an alias of `prmkit`. [Complete workflow](WORKFLOW.md) ·
[Verification semantics](VERIFICATION_AND_CLEARANCE.md) ·
[Hardware/objectives](OBJECTIVES.md) ·
[Hebrew HTML walkthrough](RESEARCH_WORKFLOW_HE.html).

A standalone distribution of the project's robot planning workflow. Build a
collision-checked PRM, compute an Exact or approximate Pareto frontier, export
joint paths, and search a configurable family of environments. Change ordinary
settings in YAML; add Python plug-ins only when you want a new computation.

This directory is independent of the research repository. Copy or unzip it
anywhere; no `results/`, historical roadmap, HPC account, GPU or Isaac installation
is needed for the basic example. All writes go into the project/output locations
you choose. The existing research code is not used or modified.

## One file for research cases

Open **`run_experiment.py`** and edit `CASE_ID` and `SETTINGS`. It generates all
working inputs, keeps the query bank fixed across comparisons, reuses graphs
across epsilons, and writes a study dashboard plus CSV/JSON results.

```bash
python -m pip install '.[robot]'
python run_experiment.py --plan
python run_experiment.py
```

The default is a small two-wall study: 4 environments and 8 searches. Choose
`wall_grid`, `wall_random`, `wall_sobol`, `fixed_environment` or `environment_list`.
The script explicitly selects the new mounted hardware, three objectives and
independent mesh postchecks. Public `run_case` retains its bare-UR5e default for
compatibility; pass `profile="mounted_ur5e"` for the new assembly.
The same entry point is available as `from prmkit import run_case`.
See [the cases guide](CASES.md) for wall dimensions, multiple queries,
metrics and rerunning one selected scene with denser N/K.

## First run

Use Python 3.10 or later. The current robot release is tested on Linux with Python 3.11.
From this distribution directory:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install .
.venv/bin/prmkit init my_project
.venv/bin/prmkit run my_project/experiment.yaml --output my_project/run
```

Open `my_project/run/report.html` in a browser. It works offline. The example is
a small translating disk navigating around a wall, with length/risk trade-offs.
It exercises the real planning pipeline, without robot dependencies.

On Windows, replace `.venv/bin/python` with `.venv\Scripts\python.exe` and
`.venv/bin/prmkit` with `.venv\Scripts\prmkit.exe`; create the environment with
`py -3.10 -m venv .venv`. The articulated-robot runtime is validated on Linux;
use Linux/WSL for the supplied robot workflow.

## Historical UR5e camera configuration

```bash
.venv/bin/python -m pip install '.[robot,fast]'
.venv/bin/prmkit doctor
.venv/bin/prmkit init ur5e_project --template ur5e
.venv/bin/prmkit check ur5e_project/experiment.yaml
.venv/bin/prmkit run ur5e_project/experiment.yaml --output ur5e_project/run
```

`ur5e` is a short, known-feasible query in E4, intended for installation checks.
It includes the robot convex envelopes, camera, flange, bracket and cable,
76 active collision pairs, and length, clearance and visibility costs. Its small
N and coarse cost quadrature are **not** the historical 5M experiment.

For the original scene/query banks, use `--template E4`, `E5`, or `E7`.
Their defaults preserve 10 mm hard clearance, a separate 100 mm risk threshold,
16 intervals for length/risk and 32 for visibility. They start at N=1000 so an
accidental command does not launch a 5M build. Sparse graphs may have no path.

```bash
prmkit init e4 --template E4
prmkit run e4/experiment.yaml --output e4/run_n10000 \
  --set prm.n=10000 --set prm.k=24 --set prm.workers=4 --set search.epsilon=0.05
```

Here and below `prmkit` means the executable inside your activated environment,
or its full `.venv/bin/prmkit` path. An interrupted run can be continued using
the **same command and settings** plus `--resume`.

## Import a scene built in Isaac Sim

```bash
python -m pip install '.[robot,usd]'
prmkit init isaac_project --template isaac
prmkit import-usd isaac_project/scene.usda --output isaac_project/imported
prmkit check isaac_project/experiment.yaml
prmkit run isaac_project/experiment.yaml --output isaac_project/run
```

The template includes a small USD example. Replace its input with your own saved
stage, and configure the robot task. Imported folders contain `scene.json`, local
meshes and an import report; they can travel without Isaac or the source USD.
For a stage containing the robot, use `--exclude` and, when needed, `--robot-frame`.
Ground planes require an explicit finite `--plane-size`. Unsupported geometry or
physics does not silently disappear. See the [complete import guide](ISAAC_IMPORT.md).

## What to change

| Need | Where |
|---|---|
| N, K and query connections | `prm.n`, `prm.k`, `prm.query_k` |
| Epsilon; exact frontier | `search.epsilon`; use `0` for Exact |
| Repeatable sampling | `seed`, `prm.sampler` (`sobol` or `random`) |
| New scene | `environment`: JSON file or inline object definitions |
| Scene built in Isaac Sim | `prmkit import-usd` converts supported static USD geometry into an environment folder |
| Another robot | `robot`: URDF, tool frame, weights and explicit collision policy |
| New start or goals | `query.start`, `query.goals`; or `query.target` for position-only IK |
| Different risk function parameters | Metric `threshold`, `power`, `scope`, `scale` |
| Different objective altogether | Add/replace metric entries; use a metric plug-in |
| Faster CPU build | `prm.workers`; complete edge chunks are checkpointed |
| Another number of objectives | Any positive count with `search.engine: python`; `auto` selects it as needed |
| Environment search | `space.yaml`: parameter fields, values/ranges, constraints and ranking |

## Search environments

The demo includes a six-scene wall-position/height grid:

```bash
prmkit sweep my_project/space.yaml --output my_project/sweep
prmkit sweep my_project/space.yaml --output my_project/sweep --resume
```

`space_summary.json` ranks completed feasible experiments and preserves each
candidate's settings and results. Grid and seeded random sampling are supported.
This searches the **space you define**; it does not infer a meaningful space
from an arbitrary scene. See [environment spaces](ENVIRONMENT_SPACES.md).

## Review, validate and replay

```bash
prmkit inspect ur5e_project/run
prmkit validate-path ur5e_project/experiment.yaml ur5e_project/run/paths.npz --index 0
prmkit export-usd ur5e_project/experiment.yaml ur5e_project/run/paths.npz \
  --index 0 --output ur5e_project/replay.usda
```

Open `replay.usda` in Isaac Sim and play its timeline. Select
`/World/WristCamera` to inspect the mounted camera view. The stage embeds meshes,
attached parts, world objects and sampled link transforms, so it travels without
the original repository. This is **kinematic mesh replay**; frames per second
set display timing, not a controller-executable trajectory. Isaac rendering is
optional and not required to produce the stage.

| Output | Contents |
|---|---|
| `summary.json`, `search.json` | Completion state, algorithm, epsilon, costs and paths |
| `front.csv` | One row per returned path, metrics and certified clearance |
| `paths/path_0000.csv` | Ordered joint coordinates, original vertex IDs and EE xyz |
| `paths.npz` | Concatenated joint/EE paths with offsets and objective vectors |
| `graph.npz` | Nodes, directed arcs, costs, per-arc continuous bounds and goals |
| `inputs.json` | Resolved settings, input/code hashes and dependency versions |
| `manifest.json` | Final artifact hashes; `inspect` verifies them |
| `progress.json`, `failure.json` | Latest stage and an error record if interrupted/failed |

## Guides and evidence

- [Research examples: kitchen, office, shelves, wall scenes, saved videos and smoothing](../examples/research_showcase/README.md) · [offline gallery](../examples/research_showcase/index.html). Includes portable scene-generation scripts and archived Isaac USD assets; see each example's hardware revision and reproduction scope.
- [Core toolkit capabilities — one-page Hebrew PDF](PRMKIT_CAPABILITIES_HE.pdf)
- [Short user guide by scenario, Hebrew HTML, with downloadable examples](USER_GUIDE_HE.html)
- [Interactive wall-search walkthrough, Hebrew HTML: create configurations and run end to end without Isaac](WALL_SEARCH_WIZARD_HE.html)
- [Architecture and milestones, Hebrew](../ARCHITECTURE.md)
- [Architecture, capabilities and limits — offline Hebrew HTML](ARCHITECTURE_HE.html)
- [Import an Isaac/USD environment](ISAAC_IMPORT.md)
- [Configuration and metric definitions](CONFIGURATION.md)
- [Add a metric, scene, robot or backend](EXTENDING.md)
- [Search environment spaces](ENVIRONMENT_SPACES.md)
- [Reproducibility and operating limits](REPRODUCIBILITY.md)
- [Validation results](../VALIDATION.md)
- [Copied sources and asset checksums](../PROVENANCE.json)
- [Third-party assets](../THIRD_PARTY_NOTICES.md)

Developers can install `.[robot,fast,dev]` and run `python -m pytest` from this
directory. No repository-root tests or historical scripts are required.
Use `pip install -r requirements-tested.txt` in Python 3.11 to reproduce the full
validation environment, including the optional USD and JavaScript checks.
To edit the kit's own source directly, install with `pip install -e '.[robot,fast,dev]'`.

