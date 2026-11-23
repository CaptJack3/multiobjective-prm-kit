# Included assets and reference code

The `mounted_ur5e` preset reuses the frozen standard wrist assembly from the
project's 2026-11-21 RealSense/PickNik work: Robotiq 2F-85, its coupling, the
PickNik adapter and the official D435 CAD. Geometry is byte-identical to that
assembly NPZ. Source URLs, hashes and nominal transforms are recorded in
`src/prmkit/resources/assets/mounted_ur5e/provenance.json`. Preserve the three
adjacent `picknik_LICENSE`, `robosuite_LICENSE`, and `realsense_LICENSE` files.
The gripper is fixed open; optics are nominal, not measured laboratory calibration.

The UR5e meshes and relocated URDF originate from the project's curated
`FINAL_ARTIFACTS/collision_models` collection. Preserve the accompanying
`UR_DESCRIPTION_LICENSE.txt` and `CUROBO_ASSET_LICENSE.txt` in
`src/prmkit/resources/assets/ur5e/` when redistributing those assets.

`PROVENANCE.json` identifies copied project sources, data and SHA-256 digests.
`tests/reference/assembly.py` is an unmodified project implementation retained
for regression comparison, not used by the runtime. The new two-link URDF is
an analytical example authored for this distribution.

Dependency packages retain their own licenses and are installed separately;
their binaries are not bundled in the source archive.

The optional `examples/research_showcase/` collection includes archived native
UR5e USD visuals and research media/trajectories. Preserve its `licenses/` folder
and `SOURCE_MANIFEST.json`. Its procedural household geometry is project-authored;
the later preference videos show the mounted assembly covered by the notices
above. Archived USD stages and later videos use different documented rig revisions.

