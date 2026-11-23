# One research entry point

The public `prmkit.run_case(...)` function and the editable root
`run_experiment.py` script own experiment orchestration. Users set Python
arguments once; JSON/YAML inputs are generated for provenance and inspection.
All changes live in the user distribution, independently of the research tree.

## Cases

- `wall_grid`: a Cartesian product of wall positions, widths, heights,
  thicknesses and orientations, with an explicit environment budget.
- `wall_random`: seeded sampling of wall offsets and dimensions.
- `fixed_environment`: one supplied JSON environment, or one generated fixed
  wall layout; varies planner settings without changing that environment.
- `environment_list`: evaluate a supplied list of environments identically.

Wall heights and centres change together to preserve their specified base Z.
Per-wall dictionaries override shared settings for heterogeneous layouts.
All geometry is in metres; wall yaw is specified in degrees at this interface.

## Execution and comparisons

1. Validate inputs and compute the experiment size; `dry_run=True` writes
   nothing and performs no PRM construction.
2. Materialize environments and a reproducible query bank. A query is one
   independent planning task, not one roadmap node or one returned path.
   Supplied query banks are supported. Automatically generated extra queries
   sample collision-free endpoints in the first environment, then remain
   fixed across all environments and planner settings. This does not promise
   connectivity or validity in other environments.
3. Execute environment × query × seed × N × K × epsilon. N/K vectors form a
   Cartesian product. New N, K, queries or environments rebuild the roadmap.
4. Reuse a verified roadmap and its costs across epsilon values only when
   all graph-defining inputs, source files, code and dependency versions match.
5. Write per-run reports plus a study CSV, JSON summary and offline HTML index.
   Environment rankings are separate for each N/K/epsilon/seed; completed
   query coverage takes precedence over mean score among solved queries.
   Failed, partial and infeasible tasks remain visible.

## Reproducibility and extension

Input/code/dependency fingerprints protect resume. Generated inputs are checked
before reuse; edits require a new output directory. The existing collision,
cost, search and result-validation implementations remain the source of truth.
Metrics, robot descriptions and an existing experiment may be supplied to the
entry point; custom metric/backend plug-ins retain their existing interfaces.
Each saved environment and query bank can be passed into a later study.

This layer supplies finite grids and seeded random sampling, not adaptive
environment optimization, dynamic simulation or automatic wall-overlap rules.
The first implementation processes jobs sequentially; PRM edge workers remain
configurable. Reused graph files are copied into individual run outputs so each
run retains the existing inspect/validate/report contract.
