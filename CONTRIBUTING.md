# Contributing

Use a branch for changes and describe the user-visible behavior in a pull request.
Keep generated runs, virtual environments and caches out of Git. The bundled
research examples are selected source assets; preserve their provenance and
license files when changing them.

## Development environment

From the repository root on Linux/Python 3.11:

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[robot,render,usd,fast,dev]'
python -m pytest -q
python scripts/check_docs.py
```

Some optional tests skip when their dependencies are absent. The full test
setup can additionally install `quickjs==1.19.4` for JavaScript checks. Tests use
temporary directories and do not require a historical roadmap or Isaac runtime.
To verify the supplied research collection:

```bash
python examples/research_showcase/verify_examples.py --report runs/example_check.json
```

## Before requesting review

Explain the problem, the change and the checks you ran. Add meaningful regression
coverage when fixing planner behavior. For an added scenario, include a small
configuration and the expected outputs. Keep unit/frame conventions explicit:
metres, radians for joint values, and the declared robot/scene coordinate frame.

Do not silently change objective definitions, collision-pair exclusions or
verification labels. Treat saved input/output hashes as evidence. Use a new
output directory for changed experiments and document performance measurements
with the machine, dependencies and complete settings.

For a new metric/backend, follow [the extension contracts](docs/EXTENDING.md).
Documentation entry points are in [docs/README.md](docs/README.md).

## Build a portable source archive

```bash
python scripts/build_release.py
```

The ZIP includes guides, examples and assets, and excludes `.git`, virtual
environments, caches and run directories. It contains a SHA-256 manifest and is
verified after creation. `python -m build` separately builds the installable
wheel and source distribution. The wheel contains runtime resources; clone or
use the source ZIP to obtain the research gallery and editable example scripts.
