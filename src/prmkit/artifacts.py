"""Saved-input and candidate-path catalog; no solver is invoked by this module."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path

import numpy as np

from .io import atomic_json, digest, jsonable


def content_hash(value):
    return hashlib.sha256(json.dumps(jsonable(value), sort_keys=True, separators=(",", ":"), allow_nan=False).encode()).hexdigest()


def path_hash(q):
    q = np.asarray(q, dtype="<f8", order="C")
    return hashlib.sha256(str(q.shape).encode() + q.tobytes()).hexdigest()


def run_inputs(directory, check_geometry=False):
    directory = Path(directory)
    manifest = json.loads((directory / "manifest.json").read_text())
    for name in ("inputs.json", "summary.json", "paths.npz"):
        if digest(directory / name) != manifest["files"].get(name):
            raise ValueError(f"Saved run integrity failure: {directory / name}")
    record = json.loads((directory / "inputs.json").read_text())
    if check_geometry:
        for name, expected in record["files"].items():
            if not Path(name).is_file() or digest(name) != expected:
                raise ValueError(f"Geometry/input changed since planning: {name}")
    return record


def candidate_catalog(directory, *, save=True):
    root = Path(directory).resolve()
    study_file = root / "study_summary.json"
    if study_file.exists():
        study = json.loads(study_file.read_text())
        source_id = json.loads((root / "study_inputs.json").read_text())["fingerprint"]
        runs = [(root / "runs" / row["run_id"], row) for row in study["rows"] if row.get("report")]
    else:
        source_id = run_inputs(root)["fingerprint"]
        runs = [(root, dict(run_id=root.name, environment_id="scene_00000", query_id="query_00000"))]
    paths, sources = [], []
    for run_dir, row in runs:
        record = run_inputs(run_dir)
        config = record["config"]
        summary = json.loads((run_dir / "summary.json").read_text())
        run_rel = str(run_dir.relative_to(root))
        source = dict(run=run_rel, inputs_sha256=digest(run_dir / "inputs.json"), paths_sha256=digest(run_dir / "paths.npz"))
        sources.append(source)
        scene_hash = content_hash(dict(scene=config["environment"], files={k:v for k,v in record["files"].items() if k in {o.get("mesh") for o in config["environment"]["objects"]}}))
        robot_hash = content_hash(dict(robot=config["robot"], files=record["files"]))
        with np.load(run_dir / "paths.npz", allow_pickle=False) as data:
            for index, cost in enumerate(data["costs"]):
                a, b = map(int, data["offsets"][index:index+2])
                q = data["q"][a:b]
                q_hash = path_hash(q)
                identity = dict(experiment=source_id, run=run_rel, index=index, path_hash=q_hash)
                paths.append(dict(path_id="P_" + content_hash(identity)[:24], index=index, run=run_rel,
                                  run_id=row["run_id"], scene_id=row["environment_id"], query_id=row["query_id"],
                                  experiment_id="E_"+source_id[:16], scene_geometry_hash=scene_hash,
                                  robot_geometry_hash=robot_hash, path_hash=q_hash, waypoints=len(q),
                                  costs=cost.tolist(), metrics=summary["metrics"], units=summary.get("metric_units", []),
                                  n=summary["n"], k=summary["k"], epsilon=summary["epsilon"], seed=config["seed"],
                                  search_complete=summary["complete"], search_status=summary["status"],
                                  verification={"status":"unverified"}))
    result = dict(schema_version=1, experiment_id="E_"+source_id[:16], sources=sources, paths=paths)
    result["catalog_hash"] = content_hash(result)
    if save:
        atomic_json(root / "analysis/catalog.json", result)
    return result


def read_path(root, row, *, check_geometry=False):
    run_dir = Path(root) / row["run"]
    record = run_inputs(run_dir, check_geometry)
    with np.load(run_dir / "paths.npz", allow_pickle=False) as data:
        index = row["index"]
        a, b = map(int, data["offsets"][index:index+2])
        q = data["q"][a:b].copy()
    if path_hash(q) != row["path_hash"]:
        raise ValueError("Path content no longer matches selection/catalog")
    return q, deepcopy(record["config"]), record


def read_selection(selection, catalog):
    if isinstance(selection, (str, Path)):
        selection = json.loads(Path(selection).read_text())
    if not isinstance(selection, dict) or selection.get("catalog_hash") != catalog["catalog_hash"]:
        raise ValueError("Selection must identify this exact catalog_hash")
    ids = selection.get("path_ids")
    if not isinstance(ids, list) or any(not isinstance(i, str) for i in ids) or len(set(ids)) != len(ids):
        raise ValueError("Selection path_ids must be unique strings")
    known = {r["path_id"] for r in catalog["paths"]}
    if not set(ids) <= known:
        raise ValueError("Selection contains unknown/stale path IDs")
    return ids


def scene_preview(scene, output):
    """Offline XY footprint preview; no simulator or graphics dependency."""
    import html
    objects = scene.get("objects", [])
    boxes = []
    for obj in objects:
        p = obj.get("position_m", [0,0,0])
        dims = obj.get("dimensions_m", [2*obj.get("radius_m", .04)]*3)
        yaw = obj.get("rotation_rpy", [0,0,0])[2]
        if "rotation" in obj:
            yaw = np.arctan2(obj["rotation"][1][0], obj["rotation"][0][0])
        boxes.append((obj, p, dims, yaw))
    extent = max([1.5] + [max(abs(float(v)) for v in p[:2])+float(max(d[:2])) for _,p,d,_ in boxes])
    scale = 220 / extent
    lines = ['<svg xmlns="http://www.w3.org/2000/svg" width="520" height="560" viewBox="0 0 520 560">',
             '<rect width="520" height="560" fill="#f2f6f5"/><text x="20" y="25" font-family="sans-serif" font-size="15">XY footprint; dimensions in metres</text>',
             '<path d="M20 270H500M260 30V510" stroke="#acbdb8"/>', '<circle cx="260" cy="270" r="7" fill="#185b6e"/>']
    for obj,p,d,yaw in boxes:
        x,y=260+scale*p[0],270-scale*p[1]
        w,h=scale*d[0],scale*d[1]
        lines.append(f'<rect x="{x-w/2:.4f}" y="{y-h/2:.4f}" width="{w:.4f}" height="{h:.4f}" transform="rotate({-np.degrees(yaw):.4f} {x:.4f} {y:.4f})" fill="#4c8b73" fill-opacity=".55" stroke="#24533f"><title>{html.escape(obj["name"])}</title></rect>')
    lines += ['<text x="20" y="540" font-family="sans-serif" font-size="12">Boxes shown by footprint; mesh/curved shapes are approximate markers.</text>', '</svg>']
    Path(output).parent.mkdir(parents=True, exist_ok=True)
    Path(output).write_text("\n".join(lines))
    return str(output)


def save_scenes(output, scenes, plan, base, queries, resume=False):
    from .pipeline import exclusive
    directory = Path(output).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    request = dict(scenes=scenes, plan=plan, base=base, queries=queries)
    fingerprint = content_hash(request)
    with exclusive(directory):
        marker = directory / "generation.json"
        if marker.exists():
            old = json.loads(marker.read_text())
            if not resume or old["fingerprint"] != fingerprint:
                raise ValueError("Generation exists or changed; choose a new output or resume identical inputs")
            for name, expected in old["files"].items():
                if digest(directory / name) != expected:
                    raise ValueError("Generated scene was modified")
            return old
        if any(p.name != ".run.lock" for p in directory.iterdir()):
            raise ValueError("Generation output must be empty")
        files = {}
        for i, item in enumerate(scenes):
            folder = directory / "environments" / f"env_{i:05d}"
            atomic_json(folder / "scene.json", item["scene"])
            scene_preview(item["scene"], folder / "preview.svg")
            for path in folder.iterdir():
                files[str(path.relative_to(directory))] = digest(path)
        result = dict(complete=True, stage="generated_only", fingerprint=fingerprint, plan=plan,
                      environments=len(scenes), output=str(directory), files=files,
                      note="Geometry generated without query feasibility, collision checks or planning")
        atomic_json(directory / "generation_inputs.json", request)
        atomic_json(marker, result)
        return result
