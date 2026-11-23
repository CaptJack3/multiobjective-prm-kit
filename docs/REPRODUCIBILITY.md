# Provenance, resuming and research compatibility

Each run stores resolved configuration, SHA-256 of referenced geometry and
plug-in source, kit source hashes, package versions, the random seed, graph,
paths and final artifact hashes. Copy the editable **project directory and its
assets** alongside results for reuse on another machine. `inputs.json` is an
audit record containing resolved absolute paths, not a relocatable experiment
launcher; the editable `experiment.yaml` is the portable entry point.

Only one process may own an output directory. A same-host dead process lock can
be recovered automatically. A lock owned by another host must be removed only
after verifying that owner has stopped. Do not launch two writers to one run.

Sampling checkpoints record accepted nodes and consumed random draws. Edge
chunks are committed atomically with integrity hashes. The final graph and
exports have independent hashes. Missing commit markers are recomputed; a
hash mismatch fails visibly. Resume rejects changed parameters, geometry,
plug-ins, code or dependency versions. Use another output directory for a new
experiment. A completed identical run is verified and returned unchanged.

The in-memory search queue is **not** checkpointed. Resume retains graph work
and restarts an interrupted/partial search. Thus search runtime may be repeated.
Increasing a search budget changes inputs; launch a separate output. To reuse
one completed graph for a different epsilon or search budget, use `prmkit solve
SINGLE_RUN --output NEW_OUTPUT`. Study runs also reuse graphs across epsilon
values. Changing N/K, scene or query builds another graph; there is no shared
cross-scene/query graph or incremental densification.

## Relationship to the original research

The three-objective `_research_kernel.py` is copied byte-for-byte from
`experiments/master_pareto/apex_kernel.py`. The generic Python engine ports its
corrected apex/path-pair logic, including terminal obligations and reopening;
the historical counterexamples are included in tests. The older two-objective
prototype in `src/moprm/apex.py` is deliberately not used.

UR5e model assets and E4/E5/E7 query banks are copied with hashes. The original
reference assembly source is retained **only in tests** for independent geometry
and cost comparison. Runtime has no import or path dependency on research code.
An original scene metadata copy retains historical paths for provenance; the
templates load separately generated portable `scene.json` files.

The mounted-model policy and separate 10 mm validity / 100 mm risk thresholds
are preserved. The CPU length and risk quadrature are compared against the
reference implementation. CPU visibility uses double precision trimesh ray
casting and the same target sampling convention; the historical engine used
Warp float32. Different ray implementations can differ at grazing boundaries.
Visibility geometry includes the nominal collision meshes and attached parts;
it is not a photometric camera or an RTX segmentation measurement.

New Sobol sequences are generated from the configured limits. The original
rebuild reused a historical master sample pool before falling back to new Sobol
blocks. Therefore identical N/K/epsilon **does not imply identical old graph
vertices or frontier vectors**. The user can rerun the methodology and original
scenes; this distribution does not contain the deleted/heavy 5M/10M roadmaps.

Exact ties are retained as distinct simple paths. On degenerate zero-cost
graphs this may generate many paths, so the label limit and completion status
are material parts of the result. Approximate coverage and collision certificates
apply to the declared finite graph, modeled geometry, active pairs and tolerances.

## Resource and platform scope

The portable implementation constructs geometry and rays on CPU, with optional
edge workers and a compiled search kernel. It does not package the historical
multi-GPU sphere-bound/native edge acceleration, master-roadmap masking or the
HPC scheduler. Large N/K is configurable, but this release's performance evidence
is for the sizes in `VALIDATION.md`, not a new 5M/10M throughput benchmark.

Memory scales with nodes, candidate edges, directed costs and search labels.
Increasing K increases edge work and storage. The research search preallocates
arrays from `max_labels`; one million labels require roughly 73 MB for those
arrays, in addition to graph, heaps and other process state. Python labels use
more overhead. Workers each instantiate geometry; start with one and measure.

The recommended full workflow and frozen dependency inventory use Linux/Python
3.11. A separate acceptance run also exercised Python 3.10 using compatible
resolved dependencies; the 3.11 frozen inventory must not be installed on 3.10. CPU planning
does not require CUDA, a display, Isaac Sim or a network after installation.
The optional robot dependencies pin compatible URDF/XML shared libraries to
avoid an observed import failure. A tested dependency inventory is distributed.

USDA export embeds geometry and time-sampled transforms. The stage is parsed
and checked using OpenUSD in tests. Opening/rendering it in an installed Isaac
Sim runtime remains a separate integration step; physical robot control,
calibration, trajectory timing and VLA training are outside this package.

USD **import** is available through the optional `usd` extra. It composes a local
stage into a portable static scene, retaining source hashes, units, frame and
explicit conversion choices. Unsupported geometry fails instead of publishing
a partial scene. OpenUSD is not required once that scene has been imported.
Import preserves supported geometric models; it does not replicate PhysX cooking,
collision filters, contact offsets, dynamics or photometric camera behavior.
See [the supported input contract and validation scope](ISAAC_IMPORT.md).

API references used when implementing the optional robot adapter:
[Pinocchio collision example](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/devel/doxygen-html/md_doc_2b-examples_2c-collisions.html)
and [Pinocchio geometry/distance API](https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/namespacepinocchio.html).

