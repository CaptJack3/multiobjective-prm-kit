# Research examples: scenes, saved routes and smoothing

Open **[index.html](index.html)** first. It is an offline gallery: no installation,
server, PRM build or Isaac Sim is needed to see the pictures and videos.
Copy the distribution together to retain the linked toolkit guides and PDF.

These are selected artifacts from experiments preceding PRM Kit. They show what
the research produced and supply editable environments for further work. The
commands below are portable additions; they do not run the old research pipeline.

## What is included

| Example | Ready to open | Ready to reproduce |
|---|---|---|
| Kitchen, office/workbench, multilevel shelves | Three static USD scenes, three animated USD scenes, multiview pictures | Original procedural geometry generator; USD rebuilding from editable JSON |
| E4, E5, E7, six walls | Four static USD scenes and scene pictures | USD rebuilding from scene JSON; E4/E5/E7 also have existing `prmkit init` templates |
| Three preferences in each of the seven scenes | 21 Isaac RTX videos, 21 route NPZ files, query/camera metadata, archived audits, costs | Plot the saved TCP paths and compare costs without planning |
| E4 shortest path before/after smoothing | Paired video, figures, original/smoothed replay data, cubic control points and archived audits | Inspect the exact saved arrays; the original optimization pipeline is not bundled here |

Each household scene contains three colored goal markers. The later preference
demonstrations use **one common start/goal per scene**, with three different paths.
Large PRM graphs, intermediate results and model-training weights are excluded.
The entire example directory is about 116 MB before compression.

## Open the original scenes in Isaac Sim

Use **File → Open** and choose, for example:

```text
examples/research_showcase/scenes/kitchen/scene.usdc
examples/research_showcase/scenes/office/scene.usdc
examples/research_showcase/scenes/shelves/scene.usdc
```

For the saved household motion, open `motion_scene.usdc` in the same folder and
play the timeline. Choose `/World/Cameras/Main` or `/World/Cameras/Wrist` in the
viewport camera selector. Wall scenes have overview cameras under `/World/Cameras`.
All ten archived USD files embed their assets; they need no historical server
paths or asset downloads. They were originally rendered with Isaac Sim 6.0.1.
This delivery checks their USD structure and dependencies; it does not claim a
new live Isaac/RTX run on the recipient's machine.

**There are two hardware revisions.** The archived USD scenes and household
animated stages use the earlier UR5e camera/bracket assembly with a bare tool
flange. `preference_routes/` contains the later 2026-11-21 Robotiq 2F-85,
PickNik mount and D435 demonstrations. The folder names, catalog and gallery
identify this difference. A saved route must be checked against its matching
hardware and query before reuse; it is not the animation in the older USD file.

## Recreate and edit the household geometry

Run from the distribution root. The geometry generator uses only the Python
standard library and preserves all original object dimensions and positions:

```bash
python examples/research_showcase/generate_household_specs.py \
  --output runs/household_specs
```

Edit `runs/household_specs/kitchen/scene_spec.json`, or edit the generator's
`kitchen()`, `office()` and `shelves()` functions to change the authored scene.
The specification contains visual geometry, conservative collision boxes,
materials and three goal markers. Rebuild the USD using the bundled robot,
lighting and camera template:

```bash
python -m pip install '.[usd]'
python examples/research_showcase/build_scenes.py \
  --scene kitchen --spec-root runs/household_specs \
  --output runs/rebuilt_kitchen
```

Open `runs/rebuilt_kitchen/kitchen.usdc` in Isaac Sim. To rebuild all seven
environments directly from the bundled specifications:

```bash
python examples/research_showcase/build_scenes.py --output runs/rebuilt_scenes
```

Both scripts require a **new output directory** and leave bundled inputs intact.
They do not invoke PRM. `build_scenes.py` uses NumPy and OpenUSD, and does not
import `prmkit`; it can also run with those two dependencies installed separately.
Its robot/camera templates are the archived hardware revision described above.
Changing geometry does not revalidate any saved trajectory. Collision proxies
are separate authored boxes; update them when editing visual objects.

## Coordinates and use with PRM Kit

`scene_spec.json` and household USD use world coordinates; the robot base is at
`[0, -0.08, 0.86]` metres. The bundled `scene.json` uses robot-base coordinates
for planning. Subtract that translation when converting world points into robot
coordinates. Wall examples use a zero base translation. The USD builder keeps
the robot base fixed and rejects changes to that field in a household spec.

For a quick XY view of the planning geometry:

```bash
prmkit preview examples/research_showcase/scenes/kitchen/scene.json \
  --output runs/kitchen_plan_view.svg
```

To plan on these scenes, supply the scene, matching robot profile and explicit
start/goal in a PRM Kit configuration; see [CASES.md](../../docs/CASES.md) and
[the USD import guide](../../docs/ISAAC_IMPORT.md). The historical query and
metric definitions are evidence, not a drop-in PRM Kit run configuration.
For E4/E5/E7, existing `prmkit init e4 --template E4` (or E5/E7) provides the
historical planning task. Those templates use the historical hardware profile.

## Inspect saved results without running PRM

```bash
python -m pip install '.[render]'
python examples/research_showcase/plot_routes.py \
  --scene kitchen --output runs/kitchen_routes.png
```

Choose any of the seven scene IDs. `metrics.csv` lists all 21 recorded objective
vectors. The plotted bars are normalized separately per objective; the CSV has
the raw values. A route NPZ contains `q` (six joint angles in radians), `ee`
(TCP positions in the robot-base frame), `time_s` (presentation timing),
`visibility`, `costs`, `source_q` and `bezier[segment, 4, 6]` cubic control points.
Load with `numpy.load(..., allow_pickle=False)`. Control points are **not** robot
waypoints; `q` contains sampled replay poses.

The later household study used task-biased 500K-node lazy PRMs. Its three routes
are extrema among eligible certified candidates, not a complete Pareto frontier
or an A*pex run. The later wall study reused old large-PRM paths with bounded
repair/smoothing. All videos are kinematic demonstrations; they show neither
physical grasp execution nor a validated controller trajectory. Audits are
copied historical evidence, not newly issued certificates.

The smoothing example is from a different, earlier E4 study (2026-10-18).
`smoothing/HISTORICAL_STUDY.txt` describes that original experiment and its old
repository commands; those commands require the original research repository.
This directory provides the selected results for inspection, not those runtimes.

## Provenance, licenses and checks

`SOURCE_MANIFEST.json` records source paths, SHA-256 hashes and transformations.
Binary scenes, images, videos and trajectory arrays are copied byte for byte.
Selected JSON metadata has historical absolute paths made non-absolute and some
scene JSON is projected to geometry-only fields. Historical source paths identify
provenance; they are not required runtime dependencies.

Preserve `licenses/` and the kit's [THIRD_PARTY_NOTICES.md](../../THIRD_PARTY_NOTICES.md).
The later robot assembly's three license files are already included under
`src/prmkit/resources/assets/mounted_ur5e/`. No new project-wide license is selected
by this example packaging work.

`PORTABILITY_CHECK.json` records the delivery checks and their limits. Repeat the
asset, geometry and portability checks without Isaac or a PRM build:

```bash
python examples/research_showcase/verify_examples.py --report runs/example_check.json
```

Regenerate the offline gallery after editing its catalog:

```bash
python examples/research_showcase/build_gallery.py
```
