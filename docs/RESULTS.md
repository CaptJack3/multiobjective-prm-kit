# Reading experiment results

A successful command means the workflow completed. It does not mean that every
query has a solution or that a returned path passed independent verification.
Inspect both search status and verification status.

## Start with the reports

For the numbered examples and case studies:

| File / folder | Purpose |
|---|---|
| `report.html` | Study overview: environments, tasks, settings and completion |
| `explorer/explore.html` | Path cost plots, filters, verification and selection |
| `results.csv`, `study_summary.json` | Tabular/structured study results |
| `environments/env_XXXXX/scene.json` | Concrete geometry used in a scene |
| `queries.json` | Saved start/goal tasks |
| `runs/run_XXXXXXX/` | One raw planning run; stored graph, paths, costs and provenance |
| `analysis/catalog.json` | Stable path IDs across the study |
| `verification/` | Independent postcheck jobs and their results |
| `exports/`, `videos/` | Selected-path exports/rendering, when requested |

With `--output` on an export or render command, those artifacts go to the
explicit directory instead. A schema-1 `prmkit run` produces one run directly,
rather than the outer study layout above.

## Understand the costs

The supplied mounted-robot experiments minimize three objectives: end-effector
path length, integrated proximity penalty and integrated visibility loss.
Lower is better for each cost. The proximity integral is not minimum clearance;
visibility loss is not simply one minus mean visibility. A path can be preferable
for one objective and worse for another. See [exact definitions](OBJECTIVES.md).

An epsilon frontier describes the finite roadmap and the configured search.
The number of returned paths is not a count of distinct physical motion families.
Exact search (`epsilon=0`) can be expensive and still has configured work limits.

## Verification labels

`discrete_verified` means the independent mesh checker passed the sampled
configurations under its recorded settings. It does not certify every point
between samples. Planning validity, search completion and independent
verification are separate records. Historical example audits have their own
older model/method definitions; they are not new kit verification jobs.

`prmkit select` selects passing candidates by default. An empty selection is
meaningful: inspect the audit before changing thresholds or including unverified
paths. [Verification details](VERIFICATION_AND_CLEARANCE.md) explain the options.

## Keep enough to reproduce your work

Keep the editable configuration and all referenced assets with the run.
`inputs.json` records resolved paths and hashes; it is an audit record, not an
editable portable launcher. Preserve raw `graph.npz`, `paths.npz`, costs and
manifests when collecting selected outputs. Never edit a graph shared by epsilon
runs in place. See [reproducibility](REPRODUCIBILITY.md).
