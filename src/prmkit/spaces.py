"""Declarative finite grids and seeded random environment-space experiments."""
from copy import deepcopy
import hashlib
import itertools
import json
import math
from pathlib import Path

import numpy as np
import yaml

from .backends import make_backend
from .config import load, read, checked_keys
from .io import atomic_json, jsonable, plugin
from .pipeline import run, signature, exclusive
from .roadmap import InfeasibleQuery


def get_field(obj, path):
    for key in path.split("."):
        obj = obj[int(key)] if isinstance(obj, list) else obj[key]
    return obj


def set_field(obj, path, value):
    keys = path.split(".")
    parent = get_field(obj, ".".join(keys[:-1])) if len(keys) > 1 else obj
    key = int(keys[-1]) if isinstance(parent, list) else keys[-1]
    if isinstance(parent, dict) and key not in parent:
        raise ValueError(f"Parameter field does not exist: {path}")
    parent[key] = value


def portable_config(config):
    out = deepcopy(config)
    for spec in [out["robot"], *out["metrics"]]:
        if "plugin" in spec:
            module, name = spec["plugin"].rsplit(":", 1)
            if module.endswith(".py"):
                spec["plugin"] = str((Path(spec.get("_base", out["_base"])) / module).resolve()) + ":" + name
    def clean(value):
        if isinstance(value, dict):
            return {k: clean(v) for k, v in value.items() if not k.startswith("_")}
        if isinstance(value, list):
            return [clean(v) for v in value]
        return value
    return clean(out)


def constraint_ok(config, constraints):
    ops = {"lt": lambda a,b: a < b, "le": lambda a,b: a <= b, "gt": lambda a,b: a > b,
           "ge": lambda a,b: a >= b, "eq": lambda a,b: a == b, "ne": lambda a,b: a != b}
    for condition in constraints:
        checked_keys(condition, ["field", "op", "value", "other_field"], "constraint")
        if ("value" in condition) == ("other_field" in condition):
            raise ValueError("Constraint requires exactly one of value or other_field")
        if condition["op"] not in ops:
            raise ValueError("Constraint op must be lt/le/gt/ge/eq/ne")
        right = get_field(config, condition["other_field"]) if "other_field" in condition else condition["value"]
        if not ops[condition["op"]](get_field(config, condition["field"]), right):
            return False
    return True


def generate(space):
    params = space["parameters"]
    if not params:
        raise ValueError("Space parameters must not be empty")
    limit = space.get("max_evaluations", 100)
    if isinstance(limit, bool) or not isinstance(limit, int) or limit < 1:
        raise ValueError("max_evaluations must be a positive integer")
    method = space.get("method", "grid")
    if method == "grid":
        values = []
        for key, spec in params.items():
            if not isinstance(spec, dict) or set(spec) != {"values"} or not spec["values"]:
                raise ValueError(f"Grid parameter {key} needs a nonempty values list")
            values.append(spec["values"])
        total = math.prod(map(len, values))
        return [dict(zip(params, x)) for x in itertools.islice(itertools.product(*values), limit)], total
    if method != "random":
        raise ValueError("Space method must be grid or random")
    rng = np.random.default_rng(space.get("seed", 7))
    candidates = []
    for _ in range(limit):
        row = {}
        for key, spec in params.items():
            checked_keys(spec, ["values", "min", "max", "integer"], "random parameter")
            if "values" in spec:
                if not spec["values"]:
                    raise ValueError("Parameter values must not be empty")
                row[key] = spec["values"][int(rng.integers(len(spec["values"])))]
            else:
                a, b = spec["min"], spec["max"]
                if not np.isfinite([a,b]).all() or a > b:
                    raise ValueError("Random range must have finite min <= max")
                if spec.get("integer", False):
                    if int(a) != a or int(b) != b:
                        raise ValueError("Integer ranges require integer endpoints")
                    row[key] = int(rng.integers(a, b+1))
                else:
                    row[key] = float(rng.uniform(a,b))
        candidates.append(row)
    return candidates, None


def sweep(path, output, resume=False, quiet=False):
    path = Path(path).resolve()
    space = read(path)
    checked_keys(space, ["schema_version", "experiment", "method", "parameters", "constraints", "max_evaluations", "seed", "rank"], "space")
    if space.get("schema_version", 1) != 1:
        raise ValueError("Only space schema_version=1 is supported")
    base = load(path.parent / space["experiment"])
    base_record = signature(base, make_backend(base))
    candidates, total = generate(space)
    rank = space.get("rank", {"by": "path_count", "direction": "max"})
    checked_keys(rank, ["by", "direction", "plugin"], "rank")
    if rank.get("direction", "max") not in ("min", "max"):
        raise ValueError("rank.direction must be min or max")
    rank_fn = plugin(rank["plugin"], path.parent) if "plugin" in rank else None
    record = dict(space=space, base=base_record)
    if rank_fn:
        import inspect
        from .io import digest
        record["rank_source"] = digest(inspect.getsourcefile(rank_fn))
    fingerprint = hashlib.sha256(json.dumps(jsonable(record), sort_keys=True).encode()).hexdigest()
    directory = Path(output).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    with exclusive(directory):
        marker = directory / "space_inputs.json"
        if marker.exists():
            if not resume or json.loads(marker.read_text())["fingerprint"] != fingerprint:
                raise ValueError("Sweep exists or inputs changed; use --resume for identical inputs, otherwise a new directory")
        else:
            if any(p.name != ".run.lock" for p in directory.iterdir()):
                raise ValueError("Sweep output directory must be empty")
            atomic_json(marker, dict(fingerprint=fingerprint, **record))
        rows = []
        for i, parameters in enumerate(candidates):
            candidate = portable_config(base)
            for key, value in parameters.items():
                set_field(candidate, key, value)
            row = dict(id=f"candidate_{i:05d}", parameters=parameters, score=None)
            if not constraint_ok(candidate, space.get("constraints", [])):
                row.update(status="filtered", complete=True)
            else:
                candidate_file = directory / f"{row['id']}.yaml"
                candidate_file.write_text(yaml.safe_dump(candidate, sort_keys=False), encoding="utf-8")
                try:
                    config = load(candidate_file)
                    result = run(config, directory / row["id"], resume=resume, quiet=quiet)
                    row.update(status=result["status"], complete=result["complete"], paths=result["paths"], metrics=result["metrics"], costs=result["costs"])
                    # Only completed, feasible searches qualify for ranking.
                    if result["complete"] and result["paths"]:
                        if rank_fn:
                            score = float(rank_fn(result, config, parameters))
                        elif rank.get("by", "path_count") == "path_count":
                            score = float(result["paths"])
                        elif rank["by"].startswith("best."):
                            metric = rank["by"][5:]
                            score = min(c[result["metrics"].index(metric)] for c in result["costs"])
                        else:
                            raise ValueError("rank.by must be path_count or best.<metric name>")
                        if not np.isfinite(score):
                            raise ValueError("Ranking score must be finite")
                        row["score"] = score
                except InfeasibleQuery as e:
                    row.update(status="invalid_query", complete=True, error=str(e))
                except Exception as e:
                    row.update(status="failed", complete=False, error=f"{type(e).__name__}: {e}")
            rows.append(row)
            atomic_json(directory / "space_progress.json", dict(evaluated=len(rows), requested=len(candidates), rows=rows))
        ranked = sorted([r for r in rows if r["score"] is not None], key=lambda r: r["score"], reverse=rank.get("direction", "max") == "max")
        result = dict(complete=all(r["complete"] for r in rows), evaluated=len(rows), finite_grid_size=total,
                      grid_exhausted=total == len(rows) if total is not None else False,
                      ranking=[r["id"] for r in ranked], rows=rows,
                      ranking_scope="completed feasible candidates; path_count depends on epsilon and roadmap settings")
        atomic_json(directory / "space_summary.json", result)
        return result
