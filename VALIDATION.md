# Release validation

The standalone package was implemented and tested separately from the research
tree. [Machine-readable evidence](validation/evidence.json) and the
[test output](validation/tests.txt) accompany the source distribution.

## Historical automated checks (0.2.0)

**66 tests passed**, with all optional robot and USD tests enabled, on Linux /
Python 3.10.12. The measured suite runtime was 15.26 seconds after JIT warm-up.
The current release's dependency inventory is in [requirements-tested.txt](requirements-tested.txt).
OpenUSD is an optional dependency for scene import and USD export validation;
it is not needed for planning from an already imported scene. QuickJS is used
only for validation.

| Area | Evidence |
|---|---|
| Exact and A*pex | 120 generated graphs across 1–4 objectives, each checked at epsilon 0, 0.05, 0.2 and 0.7 against exhaustive simple-path enumeration: 480 comparisons |
| Historical A*pex regressions | Both preserved counterexamples pass with Python and the unmodified compiled kernel |
| Edge cases | Zero costs/cycles, parallel arcs, exact equal-cost paths, unreachable goals and explicit label-limit completion state |
| Continuous geometry | Thin obstacles between free endpoints are rejected; unresolved edges fail closed |
| Shapes | Box, sphere, cylinder, capsule, rotated geometry and triangle mesh crossing checks |
| Robot replacement | A second two-joint URDF runs the complete pipeline; prismatic motion is tested separately |
| Custom computation | Directional metric, rejection of negative costs, custom backend and seeded Cartesian IK |
| Restart/integrity | Interrupted worker build matches uninterrupted output; changed inputs and corrupt outputs are rejected; torn checkpoint publication is not trusted |
| Multiprocessing | One-worker and two-worker graph arrays and returned costs agree |
| Environment search | Six-candidate grid completes; identical resume preserves the result |
| Report | Embedded JavaScript executes with a browser-interface stub; no external web dependencies |
| USD | Animated stage parses with OpenUSD; final wrist pose matches independent FK |
| USD import (28 new cases) | Units/up axis, nested/reset/mirrored/sheared transforms, selected robot frame, references/native instances, USDZ, analytic shapes, concave polygon triangulation and holes |
| Import failures | Missing referenced layers, unsupported shapes, unknown PhysX schemas and unsupported ancestors fail without publishing partial output; moving geometry requires an explicit snapshot |
| Imported geometry consumers | A thin imported wall rejects a crossing with free endpoints; clearance/length costs and ray visibility use the imported scene |
| Imported-scene PRM | New `isaac` template imports a USD and completes UR5e planning plus independent path validation |
| Architecture HTML | All 13 local/anchor links validated; 16 capability rows filtered in all four modes; copy handler exercised using QuickJS with a DOM stub |

## Numerical comparisons with the research

For E4, E5 and E7, pair distances and Cartesian positions agree with the preserved
assembly implementation at sampled configurations. Selected edges' length and
clearance costs agree with the independent reference implementation.

The complete archived shortest path in each case was revalidated using the new
package with the original 16/32-interval metric settings:

| Case | Continuous margin revalidated | Absolute O1 error | Absolute O2 error | Absolute O3 error |
|---|---|---:|---:|---:|
| E4 | PASS | 0 | 5.82 × 10⁻¹⁰ | 2.22 × 10⁻¹⁶ |
| E5 | PASS | 0 | 1.10 × 10⁻⁹ | 0 |
| E7 | PASS | 0 | 1.29 × 10⁻¹⁰ | 0 |

CPU visibility also matches **144/144 archived Warp samples exactly**: 24 samples
from the shortest and lowest-visibility-cost paths in each of the three cases.
Fixture indices and source checksums are included. This is evidence for these
cases, not a universal assertion of identical ray behavior at all boundaries.

## Installation outside the repository (0.1.0 baseline)

A wheel was built and installed into a **fresh virtual environment**, using
downloaded dependencies. It was imported from that environment's `site-packages`.
All projects below were created and executed under `/tmp/prmkit-portability`,
outside the repository; no input dependency points into the research tree.

| Run | Result |
|---|---|
| Core-only disk demo, no robot libraries installed | N=160, K=12, epsilon=0.1; completed, 7 paths |
| Core-only environment grid | All 6 candidates completed |
| Modified parameters / Exact | N=40, K=6, epsilon=0; completed, 12 paths |
| Fresh installation of robot/fast extras | Native imports and dependency consistency check passed |
| Mounted UR5e short query | N=12, K=4, epsilon=0.05; completed, one path; independent path validation passed |
| Other robot | Two-link URDF, N=64, K=8, epsilon=0.05; completed, one path |
| USD export | Self-contained 10-frame UR5e stage with mounted camera |

The source ZIP was also extracted outside the repository and installed with the
ordinary `pip install <extracted-directory>` command, including build isolation.
Its demo completed with seven paths. Archive CRC and every source-manifest hash
were checked when packaging.

## Standalone USD handoff check (0.2.0)

The updated wheel was installed into `/tmp/prmkit-clean` (an independent
site-packages installation, not editable). The USD extra was installed there
and `pip check` reported no broken requirements. The supplied Isaac template
was extended with an OpenUSD-authored convex mesh and imported as three objects.
The source USD was moved out of its original location, and the entire project
was relocated to a second researcher's directory **before** checking/planning.

`check`, `run`, `validate-path`, `inspect` and `export-usd` all succeeded from
outside the repository. N=24, K=6, epsilon=0.05 produced one path with costs
`[0.213401114260497, 0.16032810226148878]`; independent path validation passed.
The 10-frame replay parsed with OpenUSD. No runtime input dependency points
into the research repository. See [the recorded commands and results](validation/usd_release.json)
and [the HTML checks](validation/architecture_html.json).

## Limits of this evidence

- The full historical 5M/10M PRMs were not rebuilt; large-scale CPU throughput
  and historical multi-GPU acceleration are not qualified by these tests.
- The exported USD stage was checked with OpenUSD. An Isaac Sim/RTX rendering
  session was not available for a new visual replay validation.
- The importer is tested using OpenUSD-authored scenes, not a new Isaac runtime
  session or a user-supplied production stage. PhysX cooking/contact equivalence
  is not claimed. The architecture HTML was checked structurally and with a JS
  stub; no browser screenshot/rendering test was performed.
- Linux/Python 3.10 was tested. Native Windows/macOS robot dependency installation
  was not tested; the documented robot route uses Linux/WSL.
- New sampling produces a new graph. Original scene/query compatibility does
  not imply bitwise reproduction of an old master-pool roadmap.
- No physical robot execution, trajectory timing or VLA training was performed.

## Original work preservation

All implementation, copied assets, documentation and distribution artifacts are
under `user_distribution/`. The original tracked working-tree diff retained the
same SHA-256 before and after the work:

`533e1ad28d687fb8727b9a3455765be93c0b70344faf101dcef945fae142bc68`

Existing modifications and deletions in the research checkout were left intact.

## Case runner, version 0.3.0 — 2026-11-22

The new one-function interface adds 27 tests; **93 tests passed** against an
installed wheel in a fresh Linux/Python 3.11 environment with the declared
robot/USD/fast extras. No tests were skipped in that environment; `pip check`
reported no broken requirements. This includes the original import, geometry,
robot, metric, search and pipeline tests. See `validation/case_tests.txt` and
`validation/case_runner.json` for exact environments and evidence.

The new tests exercise Cartesian products, no-write previews, deterministic
query banks, per-wall size/orientation choices, fixed-scene densification,
query-coverage ranking, infeasible tasks, interruption/resume, changed-input
rejection, corruption rejection and actual graph reuse across epsilon values.

A separate native UR5e validation ran 66 searches across a wall grid, random
walls, densification of a selected scene, an explicit environment list and an
inherited experiment with changed metrics. It recorded 58 completed searches
with paths and 8 completed searches with no path in their roadmaps. All 31
eligible additional-epsilon runs reused their source graphs; 10 selected paths
passed independent geometry/cost validation. Those native checks used an
existing Pinocchio 4.1.0 / Coal 3.0.4 environment. The fresh-wheel checks use the
declared Pinocchio 3.9.0 / Coal 3.0.2 dependencies instead.

The study dashboard was exercised in real Chromium: all eight comparison
groups, query/status filters, relative links, an individual report, and widths
1280/390/320 pixels passed, with no JavaScript errors or external requests.
No large-scale performance or physical robot claims are added by this work.
The research working-tree diff remained unchanged throughout this extension.

## Mounted hardware and independent workflow, version 0.4.0 — 2026-11-23

**106 tests passed, with no skips**, in the new isolated Linux/Python 3.11.15
environment using Pinocchio 3.9.0 and COAL 3.0.2. Robot, USD, fast and rendering
extras were installed; the optional QuickJS test dependency was also available.
`pip check` found no broken requirements. These tests include all prior tests
plus interior-segment collisions, sampled clearance, hard-margin invalidation,
raw-output preservation, validation errors, catalog/selection integrity,
verified-only selection, source-free Sobol generation, exact scene/query reruns,
corruption rejection, early option validation and decodable selected-path MP4.

The frozen Robotiq/D435/PickNik assembly NPZ matches its original SHA-256 byte
for byte. TCP and optical transforms are preserved. The new planning geometry
was compared against the original `ur5e_realsense_picknik_20261121` implementation:
**161 collision pairs at 6 source-trajectory poses**, maximum distance difference
`1.0080245665955445e-09 m`. This checks assembly reproduction, not physical
calibration or numerical identity of complete path-cost integrations.

The actual root `run_experiment.py` completed **8/8 searches in 4 two-wall scenes**
with the mounted robot, three objectives, N=24, K=6 and epsilon 0/0.05. All eight
candidate paths passed the separate 5 mm COAL mesh postcheck at 49 samples each.
All four eligible extra-epsilon runs shared their source graph by hardlink.
The on-machine example remains under `runs/release_0_4/`; generated runs
are excluded from the source distribution.

Real Chromium exercised linked plot/table selections, status/cost/clearance
filters, representatives, an empty catalog and a manual selection downloaded
from the actual native run. Python accepted the downloaded selection. The two
Hebrew guides were checked at widths 1280, 390 and 320, and all six new-guide
steps and copying were exercised. There were no JavaScript errors or external
requests. A separate six-point synthetic fixture was used only to test UI
interaction, not as research output. The guide's editable Python example was
parsed and its 288-run plan checked against `run_case`.

The actual downloaded selection produced a **12-frame 960×720 H.264 MP4** that
decoded completely, and a self-contained animated USDA that opened in OpenUSD
with 22 mesh prims and the mounted wrist camera. The multiple source CAD parts
are combined per rigid attachment. The video is CPU mesh replay with display
timing; neither native Isaac RTX nor physical execution was tested.

See [machine-readable evidence](validation/workflow_v04.json) and
[the test log](validation/workflow_tests.txt). All new implementation, assets,
docs and delivery artifacts are inside `user_distribution/`; the research
environment and original source/results were not edited. Known remaining
capabilities and limitations are listed in
[the specification alignment](docs/SPEC_ALIGNMENT_20261123.md).

A fresh wheel installation with robot/render/USD/fast extras also completed the
single-wall experiment and its independent audit from `/tmp`, outside the
checkout. No input dependency resolved into the research repository. The wheel
contained the frozen assembly and its licenses; dependency consistency passed.
Five legacy provenance metadata files now use repository-relative historical
source paths instead of a personal home directory. Geometry, queries and
numeric fixture data were not changed; `PROVENANCE.json` records both source
and distribution hashes where that metadata normalization was applied.

