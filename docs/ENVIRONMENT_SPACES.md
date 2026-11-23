# Define and search a space of environments

The base experiment supplies the robot, query, planner and objectives. A space
file changes explicit fields in its **resolved** configuration. Consequently,
an environment referenced by a JSON filename can still be varied by its fields.
Arrays use zero-based numeric path segments.

```yaml
schema_version: 1
experiment: experiment.yaml
method: grid
max_evaluations: 6
parameters:
  environment.objects.0.position_m.1:
    values: [0.35, 0.50, 0.65]
  environment.objects.0.dimensions_m.1:
    values: [0.30, 0.50]
constraints:
  - {field: environment.objects.0.dimensions_m.1, op: lt, value: 0.8}
rank:
  by: path_count
  direction: max
```

Run `prmkit sweep space.yaml --output sweep_run`. It materializes one experiment
per candidate and writes `space_summary.json`, a ranking, errors, completion
states, and all individual run outputs. Add `--resume` after an interruption.
Each roadmap can use multiple CPU workers; candidate experiments are processed
sequentially to keep memory use bounded.

Grid enumeration follows YAML parameter order and value order. If
`max_evaluations` is smaller than the product size, only that initial prefix is
evaluated, and `grid_exhausted: false` is reported. Constraints consume candidate
slots and produce explicit `filtered` rows. They support `lt`, `le`, `gt`, `ge`,
`eq`, `ne`, with either `value` or `other_field`; no Python `eval` is used.

For random spaces:

```yaml
schema_version: 1
experiment: experiment.yaml
method: random
seed: 42
max_evaluations: 20
parameters:
  environment.objects.0.position_m.1: {min: 0.25, max: 0.75}
  environment.objects.0.dimensions_m.1: {values: [0.25, 0.4, 0.55]}
  prm.k: {min: 8, max: 24, integer: true}
rank: {by: best.risk, direction: min}
```

`integer: true` samples inclusive integer endpoints. Continuous ranges are
uniform. Repeated draws are allowed and receive separate candidate IDs. You can
vary N, K, epsilon, metric parameters, query coordinates, geometry or robot
settings using the same mechanism. Parameter fields must already exist in the
resolved base configuration; this catches spelling mistakes.

Ranking supports `path_count` or `best.<metric name>`. Only **completed feasible**
searches are ranked; partial/failed/no-path runs remain in the table. Counts are
solver- and epsilon-dependent and are not a direct measurement of geometric
diversity. Comparisons across different metric definitions or epsilons require
the intended experimental interpretation.

For a different ranking computation:

```yaml
rank:
  plugin: ranking.py:score
  direction: max
```

`score(summary, resolved_config, parameter_values)` returns a finite scalar.
This permits a user-defined environment objective without changing PRM metrics.
Examples include spread of the returned cost vectors or a task-specific score.
The plug-in source hash is recorded with the space signature. Invalid queries
are recorded as `invalid_query`; IK exhaustion carries its explicit qualification.
Other failures make the sweep incomplete.

This release provides finite grids and seeded random search. It does not infer
environment families, implement Bayesian optimization, or claim a global
optimum over a continuous environment space. Add a generator/optimizer around
the public `load`, `run` and output interfaces when that is your study.

