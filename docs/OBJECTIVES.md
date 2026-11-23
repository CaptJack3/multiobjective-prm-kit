# Objective definitions and mounted UR5e

The `mounted_ur5e` preset is the frozen **standard** configuration from the
2026-11-21 RealSense/PickNik experiment. It contains the UR5e, a fixed-open
Robotiq 2F-85 and its actual 13.9 mm coupling, the PickNik adapter, and official
D435 CAD. It has 14 planning collision bodies and 34 source CAD parts. The same
CAD is used for visibility, USD export, CPU rendering and mesh post-verification.

The source NPZ is copied byte-for-byte. Geometry/optical provenance and source
hashes are in `resources/assets/mounted_ur5e/provenance.json`; the original asset
licenses accompany it. No temporary research directory is needed at runtime.

TCP offset from tool0: `[0, 0, 0.1659000001]` m. The nominal RGB optical centre
in wrist_3_link is `[-0.0325, -0.07832352198146629, 0.028948537881749364]` m.
The exact rotation matrix is retained, as are 640×360 resolution, 69.4° horizontal
FOV, 42.561177° aspect-consistent vertical FOV, and 0.005–5 m clipping range.
These are CAD/nominal sensor definitions, not laboratory calibration. No cables,
gripper actuation, physical depth noise or stereo reconstruction are included.

All objectives are minimized:

| Objective | Computation | Unit |
|---|---|---|
| `ee_length` (O1) | Sum of consecutive TCP displacements at edge quadrature samples | m |
| `clearance` (O2) | Integral of `max(0, 1 - d_world/threshold)^power` over weighted joint arc | weighted joint rad for this revolute robot |
| `visibility` (O3) | Integral of `1 - visible_target_fraction` over weighted joint arc | weighted joint rad |

The preset's O2 threshold is 0.1 m and power is 2. This is accumulated exposure,
not minimum clearance or a max–min objective. O3 uses 64 target-surface rays,
nominal pinhole FOV and robot/attachment/world occluders. The implementation uses
double-precision CPU trimesh rays, not the research Warp kernel; the geometric
definition is preserved, but floating-point boundary cases and cost quadrature
can differ. Do not compare new small-graph counts with historical million-node
fronts as if only one variable changed.

The preset's virtual target is a 4 cm box centred at `[0.65, 0, 0.3]` m. For a
new scene, explicitly provide `camera.target_object` or `camera.target` plus
`target_dimensions_m`, and matching queries. A target is not automatically a
collision object; define its physical geometry in the scene if required.
`occluder` and `collider` can be controlled separately on environment objects.
By default occlusion follows the collider flag. The visibility target itself is
excluded from occluders so its surface rays are not counted as obstruction.

Weighted joint arc uses weights `[1, 1, 0.85, 0.65, 0.45, 0.35]` for this UR5e.
It is the PRM distance/integration measure, not an automatic fourth objective.
Metric metadata, parameters, units, code and asset hashes are saved per run.

Historical `ur5e`, `E4`, `E5`, `E7` templates retain their historical camera
assembly; `bare_ur5e` retains the old case-runner behavior. Choose the new preset
explicitly with `profile="mounted_ur5e"`. The editable root `run_experiment.py`
and schema-2 examples now choose it by default.
