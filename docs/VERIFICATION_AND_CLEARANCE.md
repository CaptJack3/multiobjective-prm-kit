# Verification and clearance

Planning and post-search verification are separate computations. Candidate path
files, their costs and the original manifests are never edited by verification.

## Hard planning margin versus postcheck

The Python entry point accepts:

```python
constraints=dict(min_clearance_m=0.005, clearance_mode="enforce_during_search")
verification=dict(enabled=True, backend="coal_mesh", scope="all",
                  min_clearance_m=0.02)
```

The planner requires the first margin (plus its numeric guard) for every active
collision pair using the existing motion-bound certificate. The second number
audits saved candidates. A candidate can satisfy planning and fail the stricter
postcheck. Neither threshold is the soft clearance objective's penalty radius.

With `clearance_mode="postcheck_only"`, `constraints.min_clearance_m` is routed
to verification and does not change the existing planner margin. Verification
must be enabled. A zero postcheck margin still rejects contact/collision.

The planning margin applies uniformly to active robot–world and self pairs,
including attached held bodies. Explicit robot pair exclusions remain effective.
Selective world/self planning margins are not implemented; unsupported switches
fail rather than silently disabling a check. Changing the enforced margin
changes the roadmap fingerprint; an old graph is not reused under a new margin.

## Independent saved-path audit

```bash
prmkit verify runs/01_single_wall --backend coal_mesh --scope all \
  --min-clearance-m 0.02 --max-joint-step-deg 0.5
prmkit verify runs/01_single_wall --scope representative
prmkit verify runs/01_single_wall --scope selected --selection selected_paths.json
```

`coal_mesh` builds robot triangle BVHs from the URDF's nominal collision meshes.
The mounted preset uses its original CAD triangles for the gripper, coupling,
adapter and D435, rather than their planning envelopes. Environment mesh objects
are checked as triangles; analytic box/sphere/etc. obstacles remain analytic.
Pair exclusions and fixed-rigid-body exclusions are preserved. This is an audit
of those models, not automatically a higher-fidelity physical model of every
possible URDF. Triangle surfaces are not volumetric reconstructions of arbitrary
open/invalid meshes; mesh quality remains part of the input contract.

Each segment is interpolated, not just its waypoints. Revolute joints use
`max_joint_step_deg` and prismatic/point coordinates use
`max_translation_step_m` (default 0.002 m). Limits are per coordinate. Work is
bounded by `max_samples` and `timeout_s` per path; exhaustion is a validation
error, never a pass. The analytic demonstration backend is available explicitly
through `--backend planning`; it is not labelled COAL mesh.

| Status | Meaning |
|---|---|
| `unverified` | This path was outside the audited subset or verification was disabled |
| `discrete_verified` | All sampled states met the requested checks and margin |
| `collision_found` | At least one checked sample was in contact/collision |
| `clearance_violation` | No sampled collision, but the margin was violated |
| `validation_error` | Invalid inputs, missing/changed assets, backend error or audit budget exhaustion |

No dense audit is labelled `continuous_verified`. The existing PRM certificate
is a separate statement about its planning geometry; it is not transferred to
the different mesh model used in this audit.

For every candidate, records include path/scene/robot hashes, validation options
and code/dependency fingerprint, UTC timestamp, checked groups, sample count,
observed minimum clearance and nearest pair. Failures identify segments and up
to 100 sampled violations. `None` clearance means no applicable active pairs,
not a measured infinite distance.

Records live under `verification/<job_id>/records.json` and `summary.csv`.
`verification/latest.json` points to the current job with a file hash. Original
candidate IDs are tied to experiment, run and path content. Changed geometry is
rejected before checking or replaying an old path; it cannot silently inherit a
pass status.

## Candidate fronts and selections

The Explorer shows the original candidate set and recomputes the nondominated
set within the verified subset for each search/query. This is
`verified_candidate_pf`, not a globally certified Pareto frontier. More
alternatives may require replanning; no automatic invalid-edge replan is claimed.

Representative verification checks a deterministic subset first. Other paths
remain unverified, and `only_verified=true` selection can use only the checked
passing subset. Manual selections bind to `catalog_hash` and scoped stable IDs;
selections from another run/catalog are rejected. Raw paths are never deleted.

## Attached held objects

The mounted Robotiq preset is fixed open and does not grasp an object. To model
an already rigidly held body, add a normal attachment with
`role: held_object`, geometry and a parent-frame pose. Its world and self pairs
are active by default. `verification.check_held_object=false` excludes those
pairs only from the postcheck and records that choice. Opening/closing fingers,
grasp transitions, contact dynamics and payload qualification are not provided.
