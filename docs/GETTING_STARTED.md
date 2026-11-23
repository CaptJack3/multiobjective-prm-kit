# Getting started

This guide takes a new checkout to a result you can inspect. Run commands from
the repository root. Use a new output directory whenever you change an experiment.

## 1. Get the repository

```bash
git clone https://github.com/CaptJack3/multiobjective-prm-kit.git
cd multiobjective-prm-kit
```

Alternatively choose **Code → Download ZIP** on GitHub, extract the archive and
open a terminal inside the extracted folder. Private repositories require a
GitHub account with access. The ZIP includes examples and assets just like a clone.

## 2. Install

The supported starting point for the full robot workflow is Linux/WSL and
**Python 3.11**. The package declares Python 3.10+ compatibility; the frozen
`requirements-tested.txt` inventory specifically targets Linux/Python 3.11.
Native Windows and macOS robot installations are not part of the validated setup.

```bash
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -e '.[robot,render,usd]'
prmkit doctor
```

The editable install makes changes to the local source immediately available.
Dependencies are installed from package repositories; the source download does
not include their binaries. An internet connection is needed for installation
unless your lab supplies a package mirror or wheel cache. Subsequent basic runs
work offline. Do not run this installation inside Isaac Sim's Python environment.

| Installation | Use |
|---|---|
| `python -m pip install -e .` | Point demo, configuration and scene generation |
| `python -m pip install -e '.[robot]'` | Articulated robots, collision checks and verification |
| `python -m pip install -e '.[robot,render,usd]'` | Recommended first installation; videos and USD import included |
| `python -m pip install -e '.[robot,render,usd,fast,dev]'` | Development/tests and optional compiled research search kernel |

To reproduce the recorded dependency inventory on **Python 3.11**, install
`python -m pip install -r requirements-tested.txt` and then the local package.
The inventory contains development tools as well as runtime dependencies.

## 3. Run a single-wall experiment

```bash
prmkit run examples/01_single_wall.yaml --output runs/first_wall --dry-run
prmkit run examples/01_single_wall.yaml --output runs/first_wall
```

The first command shows the plan; the second builds the PRM, searches, runs the
separate mesh postcheck and produces results. This is deliberately a short,
known-feasible installation task with N=24, K=6 and one query.

Open `runs/first_wall/explorer/explore.html` in a browser. Review the path costs
and verification label. The experiment also writes `report.html`, saved scenes,
query data, graphs and trajectory arrays. See [how to read them](RESULTS.md).

## 4. Change one setting

```bash
prmkit run examples/01_single_wall.yaml --output runs/wall_n100_k8 \
  --set roadmap.n_vertices=100 --set roadmap.k_neighbors=8
```

The supplied numbered examples use schema 2 and `roadmap.n_vertices` /
`roadmap.k_neighbors`. Older `prmkit init` templates use schema 1 and `prm.n` /
`prm.k`. Do not mix these field names. The [recipes](RECIPES.md) use the numbered
examples unless stated otherwise.

## Working on a server

Run Python and PRM on the server. To inspect results on your own computer, copy
the **whole run directory** and open its `report.html` or `explorer/explore.html`.
This preserves relative links to associated files. For the historical gallery,
copy the checkout or its source ZIP, then open
`examples/research_showcase/index.html` locally. No web server is required.

A GitHub Markdown guide can be read online. HTML guides and galleries should be
opened from a downloaded copy; GitHub's normal file view displays their source.

## Lightweight installation check

This translating-disk demo needs only the core package. It is useful before
installing articulated-robot dependencies:

```bash
python -m pip install -e .
prmkit init runs/point_demo --template demo
prmkit run runs/point_demo/experiment.yaml --output runs/point_demo/result
```

Open `runs/point_demo/result/report.html`. The point demo has its own geometric
model; it does not validate the UR5e runtime.

## Where next?

- [Recipes](RECIPES.md): one environment, many environments, XY grids, N/K and exported motion.
- [Research examples](RESEARCH_EXAMPLES.md): open or rebuild kitchen, workbench and shelves.
- [Troubleshooting](TROUBLESHOOTING.md): installation, paths, outputs and interrupted jobs.
