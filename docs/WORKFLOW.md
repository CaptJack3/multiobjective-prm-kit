# Run → verify → explore → select → export/render

The schema-2 experiment file and `run_case` call are two front ends to the same
planning implementation. Existing schema-1 configuration files still work.
`pareto` is an installed alias of `prmkit`; both expose the same real commands.

## First experiment

From the distribution directory, with Python 3.11 on Linux/WSL:

```bash
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[robot,render]'
prmkit run examples/01_single_wall.yaml --dry-run
prmkit run examples/01_single_wall.yaml
```

Open `runs/01_single_wall/explorer/explore.html`. This preset uses the exact
frozen Robotiq/D435/PickNik hardware, a small PRM, the three research objectives,
a 5 mm planning margin and a separate 5 mm COAL mesh postcheck. It is a short
installation task, not the full historical pregrasp/density experiment.

The one-Python-file workflow remains:

```bash
python run_experiment.py --plan
python run_experiment.py --output runs/my_walls
```

Edit `CASE_ID`, `SETTINGS`, `profile`, `constraints`, `verification` and `selection`.
Rendering is disabled by default. The public function's default remains
`bare_ur5e` for compatibility; the supplied script explicitly chooses
`mounted_ur5e`. Omitting postprocessing arguments retains the old raw-run behavior.

## Unified configuration

| Section | Responsibilities |
|---|---|
| `experiment` | Name, seed, optional base schema-1 experiment |
| `robot` | `preset` or explicit robot `definition` |
| `scene` | Generate grid/random/Sobol/manual walls, or use `file`/`files` |
| `queries` | Fixed pairs/file, seeded sampling, or explicit starts × goals |
| `roadmap` | `n_vertices`, `k_neighbors`, `query_k`, workers, planner seeds |
| `objectives` / `camera` | Cost definitions and optical target/model |
| `search` | Epsilon(s), timeout per search and max labels |
| `constraints` | Hard planning margin or postcheck-only threshold |
| `verification` | Backend, scope, interpolation, threshold, work limits |
| `selection` | Representatives or `file` pointing to a manual selection |
| `render` / `export` | Optional selected-path processing; disabled unless requested |
| `output.directory` | Results directory, relative to the experiment document |

All unknown fields fail clearly. The proposal's illustrative API is not accepted
silently: there is no `guided`, arbitrary XYZ workspace constraint generator,
variable wall-count range, scene acceptance/rejection optimizer, sampled
Cartesian-product endpoint bank or cross-scene base-roadmap filtering yet.
Explicit Cartesian-product joint vectors are supported. Shared query banks and
epsilon graph reuse are implemented. Grid budgets take a prefix; random/Sobol
sampling may repeat discrete combinations and does not impose wall non-overlap.

```bash
prmkit run examples/01_single_wall.yaml --output runs/changed \
  --set roadmap.n_vertices=100 --set search.epsilons='[0,0.05]'
```

Eight examples cover single-wall startup, grid, multiple random walls, imported
Isaac scenes, Cartesian queries, density rerun, strict postcheck and enforced
clearance. `04_import_isaac_scene` needs an imported `scene.json`; `06` needs the
saved output of `01`. Example `07` deliberately postchecks at 2 cm: the start's
self clearance can fail this stricter threshold. It does not claim to discover
new feasible alternatives by filtering old candidates.

## Independent stages

```bash
# Geometry only; writes scenes + XY previews, without loading a robot or solving.
prmkit generate examples/03_multi_wall_random.yaml --output runs/scene_bank
prmkit preview runs/scene_bank/environments/env_00000/scene.json --output preview.svg

# Keep concrete scenes and queries, change roadmap density.
prmkit rerun runs/01_single_wall --output runs/denser --n 100 500 --k 8 16

# Search exactly one saved graph again, without construction.
prmkit solve runs/01_single_wall/runs/run_0000000 --output runs/finer_search \
  --epsilon 0 --timeout-seconds 300

# Audit old results independently; update the explorer.
prmkit verify runs/01_single_wall --backend coal_mesh --scope all --min-clearance-m 0.02
prmkit explore runs/01_single_wall

# Select only passing candidates, or use the Explorer's downloaded selection.
prmkit select runs/01_single_wall --output selected_paths.json
prmkit export runs/01_single_wall --selection selected_paths.json --only-verified
prmkit render runs/01_single_wall --selection selected_paths.json --only-verified
```

The selection command defaults to verified candidates. Add
`--include-unverified` deliberately to select candidates before audit. Empty
selections are reported and are not rendered. Use `verify --scope selected
--selection selected_paths.json` to audit a manual choice first.

The standalone explorer provides linked objective-pair scatterplots, synchronized
point/checkbox selection, verification/clearance/cost filters, per-query/search
groups and `selected_paths.json` download. It currently has click selection, not
lasso or a robot 3D player. With more than three objectives it plots the first
three and retains all costs in the table and selection calculations.

## Outputs and reuse

Original `runs/run_XXXXXXX/` candidates remain immutable. New stages add:

```text
analysis/catalog.json                  # stable experiment/run/path IDs
analysis/selected_paths.json           # deterministic selection, if requested
verification/<job_id>/records.json     # all candidates, including unverified
verification/<job_id>/summary.csv
explorer/explore.html                  # self-contained viewer and selection UI
exports/<job_id>/                      # selected per-path NPZ, CSV, JSON, optional USD
videos/<job_id>/                       # selected MP4s and verification metadata
workflow/config_requested_*.yaml
workflow/config_resolved_*.json
```

`--resume` on a study reuses completed planning work only if inputs, code and
dependencies match. Postprocessing options are outside the planning fingerprint,
so change verification/selection/rendering independently. Full verification reruns
when requested; it does not reuse previous pass labels by assumption. Failure or
partial search candidates stay visible. `prmkit status RUN` summarizes progress.

Graphs shared across epsilon searches are hard-linked on the same filesystem;
cross-filesystem copies are the fallback. Hashes are checked either way. N/K,
scene, query, robot or enforced-margin changes construct/revalidate a new graph.
There is no incremental densification or automatic multi-query base graph yet.
Never edit a shared graph in place. Keep raw candidate/provenance files when
cleaning videos; there is no automatic source-data deletion. Slurm example:
`sbatch slurm/example.sbatch` from the distribution directory.

## Rendering and portability

`render` makes **CPU triangle-mesh preview MP4s**, with only the selected saved
polylines. It shows the verification status and uses display timing; no physics,
speed/acceleration qualification or grasp execution is implied. The renderer
does not invoke Isaac RTX or simulate a stereo sensor. Use `export --usd` to
create native animated USDA with the same mounted meshes and wrist camera for
opening/rendering in Isaac. Native RTX orchestration is a future optional adapter.

The toolkit source/presets are portable and include the mounted hardware assets.
Saved planning inputs resolve mesh filenames to absolute locations; moving an
entire study to a new machine for further verification requires restoring those
assets/paths or regenerating a configuration with explicitly relocated assets.
The explorer itself is a single standalone HTML file. CSV/NPZ trajectory exports
are portable data, but they are not complete VLA observation/action datasets.
