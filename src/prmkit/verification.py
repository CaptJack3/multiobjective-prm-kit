"""Independent sampled collision/clearance audit of saved candidate paths.

This module never calls the search engine and never labels dense sampling as a
continuous safety proof. A COAL BVH tests the supplied triangle geometry; it is
not a physical calibration model or an occupancy reconstruction of open meshes.
"""
from copy import deepcopy
import csv
from datetime import datetime, timezone
import importlib.metadata
import json
from pathlib import Path
import time

import numpy as np

from .artifacts import candidate_catalog, content_hash, read_path, read_selection, run_inputs
from .backends import make_backend
from .config import checked_keys, number
from .geometry import valid_q
from .io import atomic_json, digest


DEFAULT = dict(enabled=True, backend="coal_mesh", scope="all", min_clearance_m=0.0,
               check_robot_environment=True, check_self_collision=True, check_held_object=True,
               interpolation=dict(method="dense", max_joint_step_deg=0.5, max_translation_step_m=0.002),
               max_samples=2000000, timeout_s=120., on_invalid="mark")
PASS = {"discrete_verified", "continuous_verified"}


def options(value=None):
    if isinstance(value, bool):
        value = {"enabled": value}
    value = value or {}
    checked_keys(value, DEFAULT, "verification")
    out = deepcopy(DEFAULT)
    out.update(deepcopy(value))
    out["interpolation"] = dict(DEFAULT["interpolation"], **value.get("interpolation", {}))
    checked_keys(out["interpolation"], DEFAULT["interpolation"], "verification.interpolation")
    for key in ("enabled", "check_robot_environment", "check_self_collision", "check_held_object"):
        if not isinstance(out[key], bool):
            raise ValueError(f"verification.{key} must be boolean")
    if out["backend"] not in ("coal_mesh", "planning"):
        raise ValueError("verification.backend must be coal_mesh or planning")
    if out["scope"] not in ("all", "representative", "selected"):
        raise ValueError("verification.scope must be all, representative or selected")
    if out["on_invalid"] not in ("mark", "exclude_from_verified_view"):
        raise ValueError("verification.on_invalid never deletes source candidates")
    if out["interpolation"]["method"] != "dense":
        raise ValueError("This independent audit implements dense sampling, not continuous verification")
    if not out["check_self_collision"] and not out["check_robot_environment"]:
        raise ValueError("At least one collision-pair group must be audited")
    number(out["min_clearance_m"], "verification.min_clearance_m")
    for key in ("max_joint_step_deg", "max_translation_step_m"):
        number(out["interpolation"][key], key, strict=True)
    number(out["max_samples"], "verification.max_samples", 1, integer=True)
    number(out["timeout_s"], "verification.timeout_s", strict=True)
    return out


def apply_constraints(config, constraints, verification):
    """Keep the planner margin and an optional postcheck threshold separate."""
    checked_keys(constraints, ["min_clearance_m", "clearance_mode", "include_self_collision", "include_held_object"], "constraints")
    threshold = constraints.get("min_clearance_m", 0.)
    number(threshold, "constraints.min_clearance_m")
    mode = constraints.get("clearance_mode", "postcheck_only")
    if mode not in ("postcheck_only", "enforce_during_search"):
        raise ValueError("clearance_mode must be postcheck_only or enforce_during_search")
    for key in ("include_self_collision", "include_held_object"):
        if key in constraints and not isinstance(constraints[key], bool):
            raise ValueError(f"constraints.{key} must be boolean")
    if mode == "enforce_during_search":
        if constraints.get("include_self_collision", True) is False:
            raise ValueError("Planner margin applies to every active pair, including self pairs; use explicit robot pair exclusions")
        if constraints.get("include_held_object", True) is False and any(p.get("role") == "held_object" for p in config["robot"].get("attachments", [])):
            raise ValueError("Planner margin applies to held-object pairs too; cannot selectively disable their margin")
        config["validation"]["margin"] = threshold
        # margin/guard are already part of the roadmap fingerprint and certificate.
        return verification
    out = options(verification)
    if not out["enabled"]:
        raise ValueError("postcheck_only requires verification.enabled=true")
    out["min_clearance_m"] = threshold
    out["check_self_collision"] = constraints.get("include_self_collision", True)
    out["check_held_object"] = constraints.get("include_held_object", True)
    return out


def make_verifier(config, opts):
    config = deepcopy(config)
    if opts["backend"] == "coal_mesh":
        if config["robot"].get("backend", "urdf") != "urdf":
            raise ValueError("coal_mesh requires a URDF robot; use backend=planning for the analytic point example")
        config["robot"]["collision_geometry"] = "mesh"
        # Environment triangle meshes retain authored geometry; do not hull them.
        for obj in config["environment"]["objects"]:
            if obj.get("shape") == "mesh":
                obj["convex"] = False
    return make_backend(config)


def audit_path(q, backend, opts):
    """Evaluate endpoints and every interpolated segment, with bounded work."""
    q = np.asarray(q, float)
    if q.ndim != 2 or len(q) == 0 or not all(valid_q(backend, x) for x in q):
        raise ValueError("Invalid path dimension, nonfinite state or joint limit violation")
    interp = opts["interpolation"]
    limits = np.full(q.shape[1], interp["max_translation_step_m"])
    if hasattr(backend, "joint_types"):
        for i, name in enumerate(backend.joint_names):
            if backend.joint_types[name] == "revolute":
                limits[i] = np.deg2rad(interp["max_joint_step_deg"])
    elif hasattr(backend, "model"):
        raise ValueError("Verifier cannot infer joint units for this backend")
    counts = np.maximum(1, np.ceil(np.max(np.abs(np.diff(q, axis=0)) / limits, axis=1))).astype(np.int64)
    sample_budget = 1 + sum(map(int, counts)) + max(0, len(q)-2)
    if sample_budget > opts["max_samples"]:
        raise ValueError(f"Verification sample budget exceeded: {sample_budget} > {opts['max_samples']}")
    mask = np.where(backend.world_mask, opts["check_robot_environment"], opts["check_self_collision"])
    if not opts["check_held_object"]:
        held = getattr(backend, "held_mask", np.zeros(len(mask), bool))
        mask &= ~held
    pair_names = getattr(backend, "pairs", [("robot", o["name"]) for o in getattr(backend, "objects", [])])
    minimum = np.inf
    nearest = None
    samples = 0
    collisions, violations, evidence = set(), set(), []
    began = time.monotonic()
    def evaluate(state, segment):
        nonlocal minimum, nearest, samples
        if time.monotonic() - began > opts["timeout_s"]:
            raise ValueError("Verification timeout; no pass status is issued")
        distances = np.asarray(backend.state(state).distances)
        if distances.shape != mask.shape or not np.isfinite(distances[mask]).all():
            raise ValueError("Collision backend returned invalid distances")
        samples += 1
        if not mask.any():
            return
        indices = np.flatnonzero(mask)
        index = int(indices[np.argmin(distances[mask])])
        distance = float(distances[index])
        if distance < minimum:
            minimum, nearest = distance, list(pair_names[index])
        collision = distance <= 0.
        violation = distance < opts["min_clearance_m"]
        if collision:
            collisions.add(segment)
        if violation:
            violations.add(segment)
        if (collision or violation) and len(evidence) < 100:
            evidence.append(dict(segment=segment, sample=samples-1, pair=list(pair_names[index]), distance_m=distance))
    evaluate(q[0], 0)
    for segment, (a, b, count) in enumerate(zip(q[:-1], q[1:], counts)):
        # Recheck a segment's left endpoint so violation ownership is unambiguous.
        if segment:
            evaluate(a, segment)
        for i in range(1, int(count)+1):
            evaluate(a + (b-a)*(i/int(count)), segment)
    groups=dict(robot_environment=int(np.sum(mask & backend.world_mask)),
                self_collision=int(np.sum(mask & ~backend.world_mask)),
                held_object=int(np.sum(mask & getattr(backend,"held_mask",np.zeros(len(mask),bool)))))
    return dict(status="collision_found" if collisions else "clearance_violation" if violations else "discrete_verified",
                checks=[key for key,count in groups.items() if count], checked_pair_groups=groups,
                min_clearance_m_observed=None if not np.isfinite(minimum) else minimum,
                closest_pair=nearest, sample_count=samples, active_pair_count=int(mask.sum()),
                colliding_segment_ids=sorted(collisions), clearance_violation_segment_ids=sorted(violations),
                failure_samples=evidence, elapsed_s=time.monotonic()-began,
                assurance="Discrete samples of stated geometry and interpolation; not a continuous or physical safety guarantee")


def verify_paths(directory, verification=None, *, selection=None, catalog=None):
    root = Path(directory).resolve()
    catalog = catalog or candidate_catalog(root)
    opts = options(verification)
    chosen = {r["path_id"] for r in catalog["paths"]} if opts["enabled"] else set()
    if opts["enabled"] and opts["scope"] == "selected":
        chosen = set(read_selection(selection, catalog))
    elif opts["enabled"] and opts["scope"] == "representative":
        from .selection import representative
        chosen = set(representative(catalog, only_verified=False)["path_ids"])
    versions = {}
    for module in ("numpy", "pin", "coal", "trimesh"):
        try:
            versions[module] = importlib.metadata.version(module)
        except importlib.metadata.PackageNotFoundError:
            pass
    code = {str(p.relative_to(Path(__file__).parent)):digest(p) for p in Path(__file__).parent.rglob("*.py")}
    job = dict(catalog_hash=catalog["catalog_hash"], options=opts, path_ids=sorted(chosen), code=code, versions=versions)
    job_id = content_hash(job)
    folder = root / "verification" / job_id[:20]
    backends, errors, records = {}, {}, []
    # All selected source assets must still match the planning provenance.
    for row in catalog["paths"]:
        if row["path_id"] not in chosen or row["run"] in backends or row["run"] in errors:
            continue
        try:
            record = run_inputs(root / row["run"], check_geometry=True)
            backends[row["run"]] = make_verifier(record["config"], opts)
        except Exception as error:
            errors[row["run"]] = f"{type(error).__name__}: {error}"
    for row in catalog["paths"]:
        record = dict(path_id=row["path_id"], path_hash=row["path_hash"], status="unverified", backend=opts["backend"],
                      scene_geometry_hash=row["scene_geometry_hash"], robot_geometry_hash=row["robot_geometry_hash"],
                      validation_config_hash=job_id, required_min_clearance_m=opts["min_clearance_m"],
                      timestamp_utc=datetime.now(timezone.utc).isoformat(), sample_count=0,
                      checks=[], requested_checks=[name for name,key in [("robot_environment","check_robot_environment"),("self_collision","check_self_collision"),("held_object","check_held_object")] if opts[key]])
        if row["path_id"] in chosen:
            try:
                if row["run"] in errors:
                    raise ValueError(errors[row["run"]])
                q, _, _ = read_path(root, row)
                record.update(audit_path(q, backends[row["run"]], opts))
            except Exception as error:
                record.update(status="validation_error", sample_count=None, error=f"{type(error).__name__}: {error}")
        records.append(record)
    summary = {status:sum(r["status"] == status for r in records) for status in ("discrete_verified","collision_found","clearance_violation","validation_error","unverified")}
    result = dict(schema_version=1, catalog_hash=catalog["catalog_hash"], job_id=job_id, options=opts,
                  checked=len(chosen), candidates=len(records), summary=summary, records=records,
                  complete=not summary["validation_error"], lineage=job)
    atomic_json(folder / "records.json", result)
    csv_file = folder / "summary.csv"
    fields=["path_id","status","backend","required_min_clearance_m","min_clearance_m_observed","sample_count","validation_config_hash"]
    with csv_file.open("w",newline="") as file:
        writer=csv.DictWriter(file,fieldnames=fields,extrasaction="ignore");writer.writeheader();writer.writerows(records)
    atomic_json(root / "verification/latest.json", dict(file=str((folder / "records.json").relative_to(root)), sha256=digest(folder / "records.json"), catalog_hash=catalog["catalog_hash"]))
    return result


def current_verification(root, catalog):
    pointer = Path(root) / "verification/latest.json"
    if not pointer.exists():
        return None
    ref = json.loads(pointer.read_text())
    if ref["catalog_hash"] != catalog["catalog_hash"]:
        return None
    file = (Path(root) / ref["file"]).resolve()
    if not file.is_relative_to(Path(root).resolve()) or digest(file) != ref["sha256"]:
        raise ValueError("Verification record integrity failure")
    return json.loads(file.read_text())


def attach_verification(catalog, result):
    catalog = deepcopy(catalog)
    if result:
        if result["catalog_hash"] != catalog["catalog_hash"]:
            raise ValueError("Verification belongs to another candidate catalog")
        records = {r["path_id"]:r for r in result["records"]}
        for row in catalog["paths"]:
            record = records.get(row["path_id"])
            if record and record["path_hash"] == row["path_hash"]:
                row["verification"] = record
    return catalog
