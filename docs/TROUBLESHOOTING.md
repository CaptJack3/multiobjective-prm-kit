# Troubleshooting

| Symptom | Check / next step |
|---|---|
| `prmkit: command not found` | Run `source .venv/bin/activate`, then `python -m pip install -e .` from the repository root. |
| Missing `pinocchio`, `coal` or robot geometry dependency | Run `python -m pip install -e '.[robot]'`, then `prmkit doctor`. |
| Missing `pxr` | Install `.[usd]`; OpenUSD is needed for importing/building USD, not for an already imported scene. |
| Missing plotting/video dependencies | Install `.[render]`. The geometry-only XY helper also supports `--no-plots`. |
| Frozen requirements will not install on Python 3.10 | Use Python **3.11** for `requirements-tested.txt`; it is a 3.11 inventory. |
| Shared-library/import errors in a lab environment | Start a clean Python 3.11 virtual environment. If your shell inherits Isaac/Conda library overrides, open a clean shell or unset `PYTHONPATH` and `LD_LIBRARY_PATH` for this environment. |
| Output directory already exists | Use a new name for changed inputs. For an interrupted identical run, use `--resume`. |
| `Unknown ... fields` | Check schema: numbered examples use `roadmap.n_vertices`; older schema-1 templates use `prm.n`. |
| No path / invalid query | Inspect search status and start/goal validity. Increasing N alone cannot fix an endpoint inside an obstacle. |
| No selected paths | Inspect independent verification; selection defaults to passing candidates. |
| Rendering takes longer than planning | The renderer is a CPU mesh preview. Select one path and start with a small frame count. |
| GitHub shows HTML code | Clone/download the directory and open the HTML locally in a browser. |
| Images missing in a copied report | Copy the whole output/gallery directory, including relative assets. |
| Different results from the old 5M experiment | The toolkit does not include the old roadmap/sampling pool. Check model, task, seed, sample budget and objective definitions. |

Before opening an issue, run `prmkit doctor` and save the command, configuration,
Python/platform information and relevant terminal error. Include a small
reproduction when possible. Remove credentials and unrelated private data from
anything you attach. See [contributing](../CONTRIBUTING.md).
