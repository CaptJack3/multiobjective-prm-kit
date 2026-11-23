# Release validation

The first GitHub handoff passed an isolated-source, fresh-virtual-environment
check on **Linux x86_64 / Python 3.11.15** on 23 November 2026.

| Check | Result |
|---|---|
| Fresh dependency install and `pip check` | Passed |
| Existing automated test suite | **106 passed**, no failures or skips |
| Point demo and single-wall robot workflow | Completed |
| Independent verification and selected path export | One passing path; CSV/NPZ/JSON/USD produced |
| Selected robot video | Generated and decoded successfully |
| Geometry-only XY grid | 1,424 environments; zero PRM runs |
| Static USD import followed by planning | Completed |
| Research collection portability | Seven scenes rebuilt after relocation; ten USD stages with no external assets; 21 saved routes checked |
| Onboarding documentation | 116 local links checked |
| Installable wheel | Built successfully |

[Machine-readable command record](../validation/github_release.json) ·
[Test output](../validation/github_release_pytest.txt) ·
[Dependency inventory](../validation/github_release_dependencies.txt)

The checks ran from a copied source tree outside the research repository with
`PYTHONPATH` and `LD_LIBRARY_PATH` unset. They include the actual quickstart and
representative commands from the new guides. They are functional checks, not a
large-roadmap performance benchmark.

Earlier acceptance work also covered multi-query/epsilon studies, strict
verification failure, interrupted resume and reproducibility. Existing evidence
is listed in [VALIDATION.md](../VALIDATION.md). The research collection's
[separate check record](../examples/research_showcase/PORTABILITY_CHECK.json)
records comparison against original geometry and archived asset hashes.

The CI workflow runs core tests on Python 3.10/3.11 and the robot/USD workflow on
Python 3.11. A configured workflow is not a claim of a successful GitHub-hosted
run; consult the repository's Actions page for that result.

Live Isaac/RTX, browser interaction and hardware execution were not rerun in this
handoff. Archived videos/audits remain historical evidence.
