# Capabilities and command map

**M** means the number of environments. **N** means sampled roadmap vertices in
one planning run. **K** controls nearest-neighbor proposals; `query_k` separately
controls start/goal connection candidates. Rejected collision edges can reduce
the accepted degree, and incoming proposals can increase it.

Commands below are entry points, with abbreviated arguments. See
[copyable recipes](RECIPES.md) for complete examples.

| Task | Command / function | Result |
|---|---|---|
| Create an editable project | `prmkit init DIR --template ...` | Robot/scene/task configuration and assets |
| Generate one or M environments without planning | `prmkit generate CONFIG --output OUT` | Scene JSON and XY previews |
| Enumerate two walls on an XY grid, varying width/height | `python examples/generate_two_wall_xy_grid.py ...` | Accepted combinations, automatic HTML/SVG gallery and optional PNGs |
| Rebuild kitchen, workbench, shelves or wall USD | `python examples/research_showcase/build_scenes.py ...` | Editable self-contained USD scenes |
| Regenerate household dimensions/objects from code | `python examples/research_showcase/generate_household_specs.py ...` | Three procedural scene specifications |
| Import static geometry from Isaac/USD | `prmkit import-usd FILE --output OUT` | Portable scene, local meshes and conversion report |
| Preview a scene | `prmkit preview scene.json --output view.svg` | XY geometry footprint |
| Plan one robot task | `prmkit run CONFIG --output OUT` | PRM, paths, costs and reports |
| Plan across M supplied scenes | `run_case('environment_list', environments=[...])` | Study results across explicit scenes |
| Generate and plan across wall families | `run_case('wall_grid' / 'wall_random' / 'wall_sobol', ...)` | Generated scenes and per-task planning runs |
| Compare N, K, queries, seeds and epsilon | `python run_experiment.py --plan`, then run | Planned combinations, results CSV and study dashboard |
| Explore generic configuration parameters | `prmkit sweep space.yaml --output OUT` | Grid/random experiments with result ranking |
| Repeat a saved task with different N/K | `prmkit rerun RUN --n ... --k ...` | Same scene/query, newly built graphs |
| Search one saved graph again | `prmkit solve SINGLE_RUN --epsilon ...` | New search result without a new PRM |
| Independently check saved paths | `prmkit verify RUN --backend coal_mesh ...` | Sampled mesh-check records |
| Compare and select paths | `prmkit explore RUN`, `prmkit select RUN ...` | Linked cost plots, filters and a selection file |
| Export selected trajectories | `prmkit export RUN --selection FILE --usd` | CSV/NPZ/JSON and optional animated USD |
| Render selected trajectories | `prmkit render RUN --selection FILE ...` | CPU mesh preview MP4s |
| View historical research examples | Open `examples/research_showcase/index.html` locally | Archived Isaac videos, scenes, paths and comparisons |
| Inspect progress / installation | `prmkit status RUN`, `prmkit doctor` | Status or dependency diagnostics |
| Preview / continue an experiment | `prmkit run ... --dry-run` / `--resume` | Work estimate / reuse of unchanged completed work |

## Boundaries

- The XY helper generates geometry and checks wall–wall overlap. It does not
  check robot feasibility or solve a PRM. Its gallery shows every scene in 2D;
  the 3D overview contains a small selection.
- Generic `sweep` and the normal `run_case` workflow **run planning**. Choose
  `generate` or the XY helper when you only want environments.
- A shared query bank gives consistent comparisons; it does not imply one graph
  shared across scenes or queries. Epsilon searches reuse one graph; N/K changes
  build another graph. There is no incremental densification.
- Import covers supported static USD geometry. It does not import robot control,
  reproduce PhysX dynamics, or automatically infer the task and camera.
- Independent `coal_mesh` verification samples configurations. Its pass label
  is `discrete_verified`, not a continuous collision certificate.
- The built-in renderer is a CPU preview. Historical Isaac RTX recordings and
  smoothing demonstrations are supplied as research examples, with separate
  documented reproduction scope.
- A path is geometric. Grasp execution, payload dynamics, actuator feasibility,
  physical camera calibration and hardware control are outside this package.

See [verification details](VERIFICATION_AND_CLEARANCE.md) and
[the historical example inventory](../examples/research_showcase/README.md).
