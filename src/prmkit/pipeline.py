"""One experiment, exclusive ownership, provenance, resumable construction."""
from __future__ import annotations

from contextlib import contextmanager
import csv
import hashlib
import importlib.metadata
import inspect
import json
import os
from pathlib import Path
import platform
import socket
import shutil
import sys
import time

import numpy as np

from . import __version__
from .backends import make_backend
from .geometry import sample_edge, certify, valid_q
from .io import atomic_json, atomic_npz, digest, jsonable, plugin
from .metrics import Metrics
from .roadmap import build, prepare_query, checkpoint_ready
from .search import solve


def signature(config, backend):
    files = {}
    def collect(obj):
        if isinstance(obj, dict):
            for k, v in obj.items():
                if not k.startswith("_"):
                    collect(v)
        elif isinstance(obj, list):
            for item in obj:
                collect(item)
        elif isinstance(obj, str) and len(obj) < 4096:
            p = Path(obj)
            if p.is_absolute() and p.is_file():
                files[str(p)] = digest(p)
    collect(config)
    for g in getattr(backend, "vis_geoms", []):
        if g.meshPath and Path(g.meshPath).is_file():
            files[str(Path(g.meshPath).resolve())] = digest(g.meshPath)
    for spec in [config["robot"], *config["metrics"]]:
        if "plugin" in spec:
            fn = plugin(spec["plugin"], spec.get("_base", config["_base"]))
            path = inspect.getsourcefile(fn)
            if path:
                files[str(Path(path).resolve())] = digest(path)
    code = {str(p.relative_to(Path(__file__).parent)): digest(p) for p in Path(__file__).parent.rglob("*.py")}
    versions = {}
    for name in ("numpy", "scipy", "pin", "coal", "trimesh", "numba"):
        try:
            versions[name] = importlib.metadata.version(name)
        except importlib.metadata.PackageNotFoundError:
            pass
    record = dict(config=config, files=files, code=code, versions=versions, kit_version=__version__)
    fingerprint = hashlib.sha256(json.dumps(jsonable(record), sort_keys=True, allow_nan=False).encode()).hexdigest()
    return dict(fingerprint=fingerprint, **record)


@contextmanager
def exclusive(directory):
    path = directory / ".run.lock"
    owner = dict(pid=os.getpid(), host=socket.gethostname(), created=time.time())
    if path.exists():
        previous = json.loads(path.read_text())
        if previous.get("host") == owner["host"]:
            try:
                os.kill(previous["pid"], 0)
            except ProcessLookupError:
                path.unlink()
            except PermissionError:
                pass
    try:
        fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    except FileExistsError as e:
        raise ValueError(f"Run already owned: {path}. If owner was on another host, verify it has stopped before removing this file.") from e
    with os.fdopen(fd, "w") as f:
        json.dump(owner, f)
    try:
        yield
    finally:
        path.unlink(missing_ok=True)


def verify_manifest(directory):
    directory = Path(directory)
    manifest = json.loads((directory / "manifest.json").read_text())
    for name, expected in manifest["files"].items():
        path = directory / name
        if not path.is_file() or digest(path) != expected:
            raise ValueError(f"Output integrity failure: {name}")
    return manifest


def check(config):
    backend = make_backend(config)
    metrics = Metrics(config)
    fixed, query = prepare_query(config, backend)
    # Evaluate actual functions early, including optional dependencies/camera.
    metrics.evaluate(sample_edge(backend, fixed[0], fixed[0], 1, config.get("camera", {})))
    return dict(status="ready", dimension=len(backend.lower), joint_names=backend.joint_names,
                query=query, metrics=[m["name"] for m in config["metrics"]],
                collision_pairs=len(backend.world_mask), continuous_bound=True)


def reuse_roadmap(record, source, directory):
    """Share a verified graph whose only configuration differences affect search.

    The input filename can differ, but resolution bases, external file hashes,
    code and dependency versions must match. No cache is shared across queries.
    """
    from copy import deepcopy
    source = Path(source).resolve()
    manifest = verify_manifest(source)
    old = json.loads((source / "inputs.json").read_text())
    def graph_inputs(value):
        value = deepcopy(value)
        value.pop("fingerprint", None)
        value["config"].pop("search", None)
        value["config"].pop("_file", None)
        return value
    if graph_inputs(old) != graph_inputs(record):
        raise ValueError("Roadmap reuse requires identical graph inputs, query, files, code and dependencies")
    if not checkpoint_ready(source / "graph.npz"):
        raise ValueError("Source roadmap has no committed graph checkpoint")
    # Publish query information before committing the graph; a crash cannot
    # leave a reusable graph without the query metadata needed by the manifest.
    atomic_json(directory / "query.json", json.loads((source / "query.json").read_text()))
    expected = manifest["files"]["graph.npz"]
    temporary = directory / "graph.npz.tmp"
    try:
        os.link(source / "graph.npz", temporary)
        storage = "hardlink"
    except OSError:
        shutil.copyfile(source / "graph.npz", temporary)
        storage = "copy"
    if digest(temporary) != expected:
        temporary.unlink()
        raise ValueError("Source roadmap changed while copying")
    temporary.replace(directory / "graph.npz")
    atomic_json(directory / "graph.npz.sha256.json", {"sha256": expected})
    atomic_json(directory / "roadmap_reuse.json", {"source": str(source), "graph_sha256": expected, "storage": storage})


def run(config, output, resume=False, quiet=False, *, roadmap_source=None):
    directory = Path(output).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    with exclusive(directory):
        backend = make_backend(config)
        metrics = Metrics(config)
        record = signature(config, backend)
        signature_file = directory / "inputs.json"
        if signature_file.exists():
            old = json.loads(signature_file.read_text())
            if old["fingerprint"] != record["fingerprint"]:
                raise ValueError("Inputs, plug-ins, dependencies or code changed. Choose a new output directory; stale checkpoints will not be reused.")
            if not resume:
                raise ValueError("Output already exists; use --resume or a new output directory")
            if (directory / "manifest.json").exists():
                manifest = verify_manifest(directory)
                if manifest["complete"]:
                    return json.loads((directory / "summary.json").read_text())
        else:
            if any(p.name != ".run.lock" for p in directory.iterdir()):
                raise ValueError("Output directory is not empty and is not a prmkit run")
            atomic_json(signature_file, record)
        began = time.monotonic()
        def progress(event):
            event["elapsed_s"] = time.monotonic() - began
            atomic_json(directory / "progress.json", event)
            if not quiet:
                print(json.dumps(event), flush=True)
        try:
            ready = check(config)
            atomic_json(directory / "check.json", ready)
            if roadmap_source is not None and not checkpoint_ready(directory / "graph.npz"):
                reuse_roadmap(record, roadmap_source, directory)
            graph = build(config, backend, metrics, directory, progress)
            progress(dict(stage="search", vertices=len(graph["nodes"]), directed_edges=len(graph["edges"])))
            result = solve(len(graph["nodes"]), graph["edges"], graph["costs"], 0, graph["goals"], **config["search"])
            atomic_json(directory / "search.json", result)
            write_paths(directory, config, backend, graph, result)
            summary = dict(schema_version=1, complete=result["complete"], termination=result["termination"],
                           status=("complete" if result["paths"] else "no_path_in_roadmap") if result["complete"] else "partial",
                           fingerprint=record["fingerprint"], n=config["prm"]["n"], k=config["prm"]["k"],
                           vertices=len(graph["nodes"]), directed_edges=len(graph["edges"]), paths=len(result["paths"]),
                           metrics=[m["name"] for m in config["metrics"]], costs=result["costs"],
                           metric_units=[m.get("unit", "m" if m.get("kind",m["name"]) == "ee_length" else "user_defined" if "plugin" in m else "weighted_joint_units") for m in config["metrics"]],
                           epsilon=result["epsilon"], algorithm=result["algorithm"], engine=result["engine"],
                           continuous_clearance=True, hard_margin=config["validation"]["margin"],
                           elapsed_s=time.monotonic()-began, python=sys.version, platform=platform.platform())
            atomic_json(directory / "summary.json", summary)
            from .report import write_report
            write_report(directory, config, graph, result, backend)
            names = ["graph.npz", "search.json", "summary.json", "inputs.json", "query.json", "paths.npz", "front.csv", "report.html"]
            if (directory / "roadmap_reuse.json").exists():
                names.append("roadmap_reuse.json")
            names += [str(p.relative_to(directory)) for p in (directory / "paths").glob("*.csv")]
            atomic_json(directory / "manifest.json", dict(complete=result["complete"], files={n: digest(directory/n) for n in names}))
            progress(dict(stage="done", status=summary["status"], paths=summary["paths"]))
            (directory / "failure.json").unlink(missing_ok=True)
            return summary
        except BaseException as error:
            atomic_json(directory / "failure.json", dict(type=type(error).__name__, message=str(error), complete=False))
            raise


def write_paths(directory, config, backend, graph, result):
    offsets = [0]
    vertices, qs, xyzs = [], [], []
    path_dir = directory / "paths"
    path_dir.mkdir(exist_ok=True)
    # Rerunning a partial search must not leave stale exported path files.
    for path in path_dir.glob("path_*.csv"):
        path.unlink()
    for i, path in enumerate(result["paths"]):
        vs = path["vertices"]
        q = graph["nodes"][vs]
        xyz = np.array([backend.state(x).ee for x in q])
        vertices.extend(vs); qs.extend(q); xyzs.extend(xyz); offsets.append(len(vertices))
        with (path_dir / f"path_{i:04d}.csv").open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["vertex", *backend.joint_names, "ee_x_m", "ee_y_m", "ee_z_m"])
            writer.writerows([[v, *j, *x] for v, j, x in zip(vs, q, xyz)])
    atomic_npz(directory / "paths.npz", offsets=np.array(offsets), vertices=np.array(vertices, dtype=np.int64),
               q=np.asarray(qs).reshape(-1, len(backend.lower)), ee=np.asarray(xyzs).reshape(-1, 3),
               costs=np.asarray(result["costs"]).reshape(-1, len(config["metrics"])))
    with (directory / "front.csv").open("w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["path_id", *[m["name"] for m in config["metrics"]], "waypoints", "minimum_certified_clearance", "search_complete"])
        for i, (path, costs) in enumerate(zip(result["paths"], result["costs"])):
            bound = np.min(graph["bounds"][path["edges"]], initial=np.inf)
            writer.writerow([i, *costs, len(path["vertices"]), bound, result["complete"]])


def validate_path(config, path, index=0):
    backend, metrics = make_backend(config), Metrics(config)
    with np.load(path, allow_pickle=False) as z:
        q = z["q"]
        if "offsets" in z:
            if not 0 <= index < len(z["offsets"])-1:
                raise ValueError("Path index out of range")
            q = q[z["offsets"][index]:z["offsets"][index+1]]
    if len(q) == 0 or not all(valid_q(backend, x) for x in q):
        raise ValueError("Path must contain valid joint configurations")
    costs = np.zeros(len(config["metrics"]))
    lower = np.inf
    if len(q) == 1:
        pairs = [(q[0], q[0])]
    else:
        pairs = zip(q[:-1], q[1:])
    for i, (a, b) in enumerate(pairs):
        samples = sample_edge(backend, a, b, config["validation"]["intervals"], config.get("camera", {}))
        bound = certify(backend, samples, config["validation"])
        if bound is None:
            return dict(valid=False, failed_edge=i, reason="collision_or_unproved_margin")
        costs += metrics.evaluate(samples)
        lower = min(lower, bound)
    return dict(valid=True, costs=costs.tolist(), certified_lower_bound=lower, waypoints=len(q),
                scope="geometry_and_costs_only; endpoints are not checked against the experiment query")
