"""Versioned experiment schema. Paths are relative to the file that owns them."""
from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import yaml

DEFAULTS = {
    "schema_version": 1,
    "seed": 7,
    "prm": {"n": 250, "k": 12, "query_k": 24, "sampler": "sobol",
            "max_draws": 100000, "workers": 1, "edge_chunk": 256},
    "validation": {"margin": 0.01, "guard": 0.000002, "intervals": 16,
                   "max_depth": 18},
    "search": {"epsilon": 0.1, "timeout_s": 120, "max_labels": 200000,
               "engine": "auto"},
    "metrics": [{"name": "ee_length"}, {"name": "clearance", "threshold": 0.1,
                "power": 2}],
}


def read(path):
    path = Path(path)
    with path.open(encoding="utf-8") as f:
        obj = yaml.safe_load(f)
    if not isinstance(obj, dict):
        raise ValueError(f"Expected an object in {path}")
    return obj


def checked_keys(obj, allowed, label):
    if not isinstance(obj, dict):
        raise ValueError(f"{label} must be a mapping")
    extra = set(obj) - set(allowed)
    if extra:
        raise ValueError(f"Unknown {label} fields: {sorted(extra)}")


def number(value, label, minimum=0, integer=False, strict=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not np.isfinite(value):
        raise ValueError(f"{label} must be finite numeric")
    if (value <= minimum if strict else value < minimum) or (integer and not isinstance(value, int)):
        raise ValueError(f"Invalid {label}: {value}")


def resolve(value, base):
    p = (Path(base) / value).resolve()
    if not p.is_file():
        raise ValueError(f"Missing input: {p}")
    return str(p)


def load(path, overrides=()):
    path = Path(path).resolve()
    return load_data(read(path), base=path.parent, source=path, overrides=overrides)


def load_data(data, base=".", source=None, overrides=()):
    """Validate an in-memory experiment using the same rules as a YAML file."""
    base = Path(base).resolve()
    path = Path(source).resolve() if source is not None else base / "experiment.yaml"
    raw = deepcopy(data)
    for item in overrides:
        key, sep, value = item.partition("=")
        if not sep:
            raise ValueError("--set requires field=value, e.g. prm.n=1000")
        parts = key.split(".")
        cursor = raw
        for part in parts[:-1]:
            cursor = cursor.setdefault(part, {})
        cursor[parts[-1]] = yaml.safe_load(value)
    checked_keys(raw, [*DEFAULTS, "robot", "environment", "query", "camera"], "experiment")
    cfg = deepcopy(DEFAULTS)
    for key, value in raw.items():
        if key in ("prm", "validation", "search"):
            checked_keys(value, DEFAULTS[key], key)
            cfg[key].update(value)
        else:
            cfg[key] = value
    if cfg["schema_version"] != 1:
        raise ValueError("Only schema_version: 1 is supported")
    for required in ("robot", "environment", "query"):
        if required not in cfg:
            raise ValueError(f"Missing {required}")
    robot = cfg["robot"]
    if isinstance(robot, str):
        robot_path = resolve(robot, base)
        robot = read(robot_path)
        robot_base = Path(robot_path).parent
    else:
        robot_base = base
    cfg["robot"] = deepcopy(robot)
    checked_keys(robot, ["backend", "urdf", "tool_frame", "joint_names", "lower", "upper", "weights", "radius",
                         "convex_file", "pair_policy", "package_dirs", "attachments", "ignore_self_pairs",
                         "ignore_world_pairs", "plugin", "parameters", "tcp_offset_m", "collision_geometry"], "robot")
    if robot.get("collision_geometry", "planning") not in ("planning", "mesh"):
        raise ValueError("robot.collision_geometry must be planning or mesh")
    tcp = np.asarray(robot.get("tcp_offset_m", [0, 0, 0]), float)
    if tcp.shape != (3,) or not np.isfinite(tcp).all():
        raise ValueError("robot.tcp_offset_m must be finite xyz in metres")
    cfg["robot"]["_base"] = str(robot_base)
    for key in ("urdf", "convex_file", "pair_policy"):
        if key in robot:
            cfg["robot"][key] = resolve(robot[key], robot_base)
    for key in ("package_dirs",):
        if key in robot:
            cfg["robot"][key] = [str((robot_base / p).resolve()) for p in robot[key]]
    for part in cfg["robot"].get("attachments", []):
        for key in ("mesh", "visual_mesh"):
            if key in part:
                part[key] = resolve(part[key], robot_base)
    env = cfg["environment"]
    if isinstance(env, str):
        env_path = resolve(env, base)
        env = read(env_path)
        env_base = Path(env_path).parent
    else:
        env_base = base
    cfg["environment"] = deepcopy(env)
    if not isinstance(env, dict) or not isinstance(env.get("objects"), list):
        raise ValueError("environment must contain an objects list (which may be empty)")
    names = []
    for obj in cfg["environment"].get("objects", []):
        if not isinstance(obj, dict) or not isinstance(obj.get("name"), str):
            raise ValueError("Each environment object needs a string name")
        names.append(obj["name"])
        for key in ("collider", "occluder"):
            if key in obj and not isinstance(obj[key], bool):
                raise ValueError(f"object.{key} must be true or false, not a string")
        if "mesh" in obj:
            obj["mesh"] = resolve(obj["mesh"], env_base)
    if len(set(names)) != len(names):
        raise ValueError("Environment object names must be unique")
    if "camera" in cfg:
        checked_keys(cfg["camera"], ["frame", "offset_m", "rotation", "fov_deg", "rays", "target_object", "target",
                                     "target_dimensions_m", "clipping_range_m", "resolution_wh"], "camera")
    query = cfg["query"]
    checked_keys(query, ["start", "goals", "file", "target", "ik_attempts", "ik_tolerance", "max_goals"], "query")
    if "file" in query and any(k in query for k in ("start", "goals", "target")):
        raise ValueError("query.file cannot be combined with start/goals/target")
    for key in ("ik_attempts", "max_goals"):
        if key in query:
            number(query[key], f"query.{key}", 1, integer=True)
    if "ik_tolerance" in query:
        number(query["ik_tolerance"], "query.ik_tolerance", strict=True)
    if "file" in query:
        with np.load(resolve(query["file"], base), allow_pickle=False) as z:
            query["start"] = z["start"].reshape(-1).tolist()
            query["goals"] = z["goals"].tolist()
        del query["file"]
    number(cfg["seed"], "seed", integer=True)
    for key in ("n", "k", "query_k", "max_draws", "workers", "edge_chunk"):
        number(cfg["prm"][key], f"prm.{key}", 0 if key == "n" else 1, integer=True)
    if cfg["prm"]["sampler"] not in ("sobol", "random"):
        raise ValueError("prm.sampler must be sobol or random")
    for key in ("margin", "guard"):
        number(cfg["validation"][key], f"validation.{key}")
    for key in ("intervals", "max_depth"):
        number(cfg["validation"][key], f"validation.{key}", 1, integer=True)
    number(cfg["search"]["epsilon"], "search.epsilon")
    number(cfg["search"]["timeout_s"], "search.timeout_s", strict=True)
    number(cfg["search"]["max_labels"], "search.max_labels", 2, integer=True)
    if cfg["search"]["engine"] not in ("auto", "python", "research"):
        raise ValueError("search.engine must be auto, python or research")
    if not isinstance(cfg["metrics"], list) or not cfg["metrics"]:
        raise ValueError("metrics must be a nonempty list")
    names = []
    for metric in cfg["metrics"]:
        checked_keys(metric, ["name", "kind", "unit", "threshold", "power", "scope", "scale", "intervals", "plugin", "parameters", "symmetric"], "metric")
        names.append(metric["name"])
        if not isinstance(metric["name"], str) or not metric["name"]:
            raise ValueError("Metric names must be nonempty strings")
        if "symmetric" in metric and not isinstance(metric["symmetric"], bool):
            raise ValueError("metric.symmetric must be true or false")
        number(metric.get("scale", 1), "metric.scale", strict=True)
        if "intervals" in metric:
            number(metric["intervals"], "metric.intervals", 1, integer=True)
        if metric.get("kind", metric["name"]) == "clearance":
            number(metric.get("threshold", 0.1), "metric.threshold", strict=True)
            number(metric.get("power", 2), "metric.power", strict=True)
            if metric.get("scope", "world") not in ("world", "all"):
                raise ValueError("clearance scope must be world or all")
    if len(set(names)) != len(names):
        raise ValueError("Metric names must be unique")
    cfg["_base"] = str(base)
    cfg["_file"] = str(path)
    return cfg
