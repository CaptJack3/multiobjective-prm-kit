"""Research studies from one Python call, without hand-written YAML files."""
from __future__ import annotations

from copy import deepcopy
import csv
import hashlib
import itertools
import json
import math
from numbers import Real
from pathlib import Path

import numpy as np
import yaml

from .backends import make_backend
from .config import checked_keys, load, load_data, read
from .geometry import minimum, valid_q
from .io import atomic_json, digest, jsonable
from .metrics import Metrics
from .pipeline import exclusive, run, signature
from .roadmap import InfeasibleQuery
from .spaces import portable_config


CASES = {
    "wall_grid": "Cartesian grid of wall offsets, widths, heights, thicknesses and yaw angles",
    "wall_random": "Seeded random wall layouts; continuous ranges or discrete choices",
    "wall_sobol": "Seeded Sobol wall layouts; continuous ranges or discrete choices",
    "fixed_environment": "One supplied environment, or one fixed generated wall layout",
    "environment_list": "An explicit list of environments evaluated with the same settings",
}


def list_cases():
    return dict(CASES)


def _number(value, name, minimum=None, integer=False, positive=False):
    if isinstance(value, bool) or not isinstance(value, Real) or not math.isfinite(value):
        raise ValueError(f"{name} must be finite numeric")
    if integer and int(value) != value:
        raise ValueError(f"{name} must be an integer")
    if minimum is not None and (value <= minimum if positive else value < minimum):
        raise ValueError(f"Invalid {name}: {value}")
    return int(value) if integer else float(value)


def _values(value, name, minimum=None, integer=False, positive=False):
    if isinstance(value, (str, bytes, dict)):
        raise ValueError(f"{name} needs a number or a nonempty sequence")
    try:
        items = [value] if isinstance(value, Real) else list(value)
    except TypeError as error:
        raise ValueError(f"Invalid {name}") from error
    if not items:
        raise ValueError(f"{name} must not be empty")
    values = [_number(x, name, minimum, integer, positive) for x in items]
    if len(set(values)) != len(values):
        raise ValueError(f"Duplicate values in {name}; use different seeds for repeats")
    return values


def _domain(value, name, method, positive=False):
    if isinstance(value, dict):
        if method != "random" or set(value) != {"min", "max"}:
            raise ValueError(f"{name}: min/max ranges require wall_random")
        a, b = [_number(value[k], name, 0 if positive else None, positive=positive) for k in ("min", "max")]
        if a > b:
            raise ValueError(f"{name}: min must be <= max")
        return {"min": a, "max": b}
    return _values(value, name, 0 if positive else None, positive=positive)


def _environment(value, base):
    if isinstance(value, (str, Path)):
        path = (base / value).resolve()
        env, base = read(path), path.parent
    elif isinstance(value, dict):
        env = deepcopy(value)
    else:
        raise ValueError("An environment must be a scene JSON path or an objects dictionary")
    if not isinstance(env.get("objects"), list):
        raise ValueError("Environment needs an objects list")
    for obj in env["objects"]:
        if "mesh" in obj:
            path = (base / obj["mesh"]).resolve()
            if not path.is_file():
                raise ValueError(f"Missing environment mesh: {path}")
            obj["mesh"] = str(path)
    return env


def _wall_environments(case_id, wall_count, walls, grid_points, offset_range,
                       widths, heights, thicknesses, yaw_offsets, radius, budget, seed):
    if walls is not None and (not isinstance(walls, (list, tuple)) or any(not isinstance(w, dict) for w in walls)):
        raise ValueError("walls must be a list of wall dictionaries")
    count = (len(walls) if walls is not None else 3) if wall_count is None else _number(wall_count, "wall_count", 0, True)
    if walls is not None and len(walls) != count:
        raise ValueError("wall_count must equal len(walls)")
    points = _number(grid_points, "grid_points", 1, True)
    radius = _number(radius, "radius", 0)
    if not isinstance(offset_range, (tuple, list)) or len(offset_range) != 2:
        raise ValueError("offset_range needs (min, max) in metres")
    lo, hi = [_number(x, "offset_range") for x in offset_range]
    if lo > hi:
        raise ValueError("offset_range needs min <= max")
    method = "random" if case_id in ("wall_random", "wall_sobol") else "grid"
    offsets = ({"min": lo, "max": hi} if method == "random" else
               [float((lo + hi) / 2)] if points == 1 else np.linspace(lo, hi, points).tolist())
    if lo == hi:
        offsets = [lo]
    if case_id == "fixed_environment":
        offsets = [0.0]
    specs, domains = [], []
    allowed = ["name", "center_xy", "base_z", "move_axis", "offsets", "widths", "heights", "thicknesses", "yaw_degs"]
    for i in range(count):
        spec = deepcopy(walls[i]) if walls is not None else {}
        checked_keys(spec, allowed, "wall")
        angle = 2 * math.pi * i / max(count, 1)
        center = spec.get("center_xy", [radius * math.cos(angle), radius * math.sin(angle)])
        axis = spec.get("move_axis", [math.cos(angle), math.sin(angle)])
        center, axis = np.asarray(center, float), np.asarray(axis, float)
        if center.shape != (2,) or axis.shape != (2,) or not np.isfinite([center, axis]).all() or np.linalg.norm(axis) == 0:
            raise ValueError("Wall center_xy and nonzero move_axis must be finite XY vectors")
        axis = axis / np.linalg.norm(axis)
        name = spec.get("name", f"wall_{i:03d}")
        if not isinstance(name, str) or not name:
            raise ValueError("Wall name must be a nonempty string")
        shared_yaws = _domain(yaw_offsets, "yaw_offsets", method)
        if isinstance(shared_yaws, dict):
            yaws = {k: v + math.degrees(angle) for k, v in shared_yaws.items()}
        else:
            yaws = [v + math.degrees(angle) for v in shared_yaws]
        choices = [
            _domain(spec.get("offsets", offsets), "offsets", method),
            _domain(spec.get("widths", widths), "widths", method, True),
            _domain(spec.get("heights", heights), "heights", method, True),
            _domain(spec.get("thicknesses", thicknesses), "thicknesses", method, True),
            _domain(spec.get("yaw_degs", yaws), "yaw_degs", method),
        ]
        if case_id == "fixed_environment" and any(isinstance(d, dict) or len(d) != 1 for d in choices):
            raise ValueError("fixed_environment needs one value per wall field; use wall_grid to vary geometry")
        specs.append(dict(name=name, center=center.tolist(), axis=axis.tolist(), base_z=_number(spec.get("base_z", 0), "base_z")))
        domains.extend(choices)
    if len({s["name"] for s in specs}) != count:
        raise ValueError("Wall names must be unique")
    # Domain products are iterated lazily: no materialization of an entire grid.
    total = math.prod(len(d) for d in domains) if method == "grid" else None
    rng = np.random.default_rng(seed)
    if method == "grid":
        combinations = itertools.islice(itertools.product(*domains), budget)
    elif case_id == "wall_sobol" and domains:
        from scipy.stats import qmc
        unit = qmc.Sobol(d=len(domains), scramble=True, seed=seed).random_base2(math.ceil(math.log2(budget)))[:budget]
        combinations = ([float(d["min"] + u*(d["max"]-d["min"])) if isinstance(d,dict)
                         else d[min(int(u*len(d)),len(d)-1)] for d,u in zip(domains,row)] for row in unit)
    else:
        combinations = ([float(rng.uniform(d["min"], d["max"])) if isinstance(d, dict)
                         else d[int(rng.integers(len(d)))] for d in domains] for _ in range(budget))
    result = []
    for index, values in enumerate(combinations):
        objects, parameters = [], []
        for i, spec in enumerate(specs):
            offset, width, height, thickness, yaw = values[5*i:5*i+5]
            xy = np.array(spec["center"]) + offset * np.array(spec["axis"])
            obj = dict(name=spec["name"], shape="box", position_m=[*xy.tolist(), spec["base_z"] + height/2],
                       dimensions_m=[thickness, width, height], collider=True)
            if abs(yaw % 360) > 1e-12:
                obj["rotation_rpy"] = [0, 0, math.radians(yaw)]
            objects.append(obj)
            parameters.append(dict(offset=offset, width=width, height=height, thickness=thickness, yaw_deg=yaw))
        result.append(dict(scene={"environment_id": f"env_{index:05d}", "objects": objects}, parameters=parameters))
    return result, total, dict(walls=specs, domains=domains, method="sobol" if case_id == "wall_sobol" else method)


def _base_experiment(base_experiment, robot, metrics, input_dir, profile="bare_ur5e"):
    if base_experiment is None:
        resources = Path(__file__).parent / "resources"
        if profile not in ("bare_ur5e", "mounted_ur5e"):
            raise ValueError("profile must be bare_ur5e or mounted_ur5e")
        template = "isaac" if profile == "bare_ur5e" else "mounted_ur5e"
        raw = read(resources / "templates" / template / "experiment.yaml")
        raw["environment"] = {"objects": []}
        base = portable_config(load_data(raw, resources))
    elif isinstance(base_experiment, (str, Path)):
        base = portable_config(load(input_dir / base_experiment))
    elif isinstance(base_experiment, dict):
        mapping = portable_config(base_experiment) if "_base" in base_experiment else deepcopy(base_experiment)
        base = portable_config(load_data(mapping, base_experiment.get("_base", input_dir)))
    else:
        raise ValueError("base_experiment must be a filename or experiment dictionary")
    if robot is not None:
        base["robot"] = deepcopy(robot)
    if metrics is not None:
        if not isinstance(metrics, (list, tuple)):
            raise ValueError("metrics must be a list of names or metric dictionaries")
        base["metrics"] = [{"name": m} if isinstance(m, str) else deepcopy(m) for m in metrics]
    return portable_config(load_data(base, input_dir))


def _read_queries(queries, input_dir):
    if isinstance(queries, (str, Path)):
        path = (input_dir / queries).resolve()
        if path.suffix.lower() == ".npz":
            with np.load(path, allow_pickle=False) as data:
                starts = data["starts"] if "starts" in data else data["start"]
                goals = data["goals"]
            if starts.ndim == 1:
                starts = np.repeat(starts[None], len(goals), axis=0)
            if starts.ndim != 2 or goals.ndim != 2 or starts.shape != goals.shape:
                raise ValueError("Query NPZ needs matching starts/goals matrices or a shared start vector")
            queries = [dict(start=s.tolist(), goals=[g.tolist()]) for s, g in zip(starts, goals)]
        else:
            queries = json.loads(path.read_text(encoding="utf-8"))
            if isinstance(queries, dict):
                queries = queries["queries"]
    if queries is None:
        return None
    if not isinstance(queries, (list, tuple)) or not queries:
        raise ValueError("queries must be a nonempty list or a query-bank JSON/NPZ path")
    result = []
    for entry in queries:
        q = deepcopy(entry)
        if not isinstance(q, dict):
            raise ValueError("Each query needs start and goal/goals (or an IK target)")
        if "goal" in q:
            if "goals" in q or "target" in q:
                raise ValueError("Choose goal, goals or target per query")
            q["goals"] = [q.pop("goal")]
        checked_keys(q, ["start", "goals", "target", "ik_attempts", "ik_tolerance", "max_goals"], "query")
        result.append(q)
    return result


def _query_bank(base, supplied, count, backend, seed, attempts):
    if supplied is not None:
        bank = deepcopy(supplied)
    else:
        q = base["query"]
        bank = ([dict(start=q["start"], goals=[goal]) for goal in q["goals"][:count]]
                if "goals" in q else [deepcopy(q)])
    for q in bank:
        if not valid_q(backend, q.get("start", [])):
            raise ValueError("Query start must match robot dimensions and joint limits")
        if ("goals" in q) == ("target" in q):
            raise ValueError("Each query needs exactly one of goals or target")
        if "goals" in q and (not q["goals"] or any(not valid_q(backend, g) for g in q["goals"])):
            raise ValueError("Query goals must match robot dimensions and joint limits")
    if len(bank) >= count:
        return bank[:count]
    rng = np.random.default_rng(seed)
    threshold = base["validation"]["margin"] + base["validation"]["guard"]
    pair, used = [], {tuple(np.ravel([q["start"], q["goals"][0]])) for q in bank if "goals" in q}
    for _ in range(attempts):
        q = rng.uniform(backend.lower, backend.upper)
        if minimum(backend.state(q).distances) <= threshold:
            continue
        pair.append(q.tolist())
        if len(pair) == 2:
            key = tuple(np.ravel(pair))
            if key not in used and pair[0] != pair[1]:
                bank.append(dict(start=pair[0], goals=[pair[1]]))
                used.add(key)
            pair = []
        if len(bank) == count:
            return bank
    raise ValueError(f"Query sampling exhausted query_attempts={attempts}: obtained {len(bank)}/{count}. Supply queries or increase the budget.")


def _expected_file(path, text):
    """Regenerate missing derived inputs; never silently replace edited inputs."""
    if path.exists():
        if path.read_text(encoding="utf-8") != text:
            raise ValueError(f"Generated input was edited: {path}. Use a new output directory.")
        return
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(text, encoding="utf-8")
    temporary.replace(path)


def _json_text(value):
    return json.dumps(jsonable(value), indent=2, sort_keys=True, allow_nan=False) + "\n"


def _summary(plan, rows, rank_by, direction):
    groups = {}
    for row in rows:
        key = (row["n"], row["k"], row["epsilon"], row["seed"])
        groups.setdefault(key, {}).setdefault(row["environment_id"], []).append(row)
    rankings = []
    for key, environments in groups.items():
        ranked = []
        for env_id, tasks in environments.items():
            solved = [r for r in tasks if r.get("score") is not None]
            if len(tasks) != plan["num_queries"] or not all(t["complete"] for t in tasks) or not solved:
                continue
            score = float(np.mean([r["score"] for r in solved]))
            ranked.append(dict(environment_id=env_id, solved_queries=len(solved), total_queries=len(tasks), score=score,
                               environment=f"environments/{env_id}/scene.json"))
        ranked.sort(key=lambda x: (-x["solved_queries"], x["score"] if direction == "min" else -x["score"], x["environment_id"]))
        rankings.append(dict(n=key[0], k=key[1], epsilon=key[2], seed=key[3], environments=ranked))
    return dict(schema_version=1, complete=len(rows) == plan["runs"] and all(r["complete"] for r in rows),
                evaluated=len(rows), requested=plan["runs"], plan=plan, rows=rows, rankings=rankings,
                rank_by=rank_by, rank_direction=direction,
                ranking_scope="Separate N/K/epsilon/seed groups; all queries evaluated to completion; more solved queries first, then mean score over solved queries. No continuous-space optimality claim.")


def _write_results(directory, result):
    atomic_json(directory / "study_summary.json", result)
    fields = ["run_id", "environment_id", "query_id", "seed", "n", "k", "epsilon", "status", "complete", "paths", "score", "elapsed_s", "metrics", "costs", "config", "report", "error"]
    tmp = directory / "results.csv.tmp"
    with tmp.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(dict(row, metrics=json.dumps(row.get("metrics", [])), costs=json.dumps(row.get("costs", []))) for row in result["rows"])
    tmp.replace(directory / "results.csv")
    from .study_report import write_study_report
    write_study_report(directory / "report.html", result)


def run_case(case_id="wall_grid", *, output="runs/study", wall_count=None, walls=None,
             grid_points=3, offset_range=(-0.2, 0.2), widths=0.6, heights=0.9,
             thicknesses=0.08, yaw_offsets=0, radius=1.0, max_environments=100,
             environment=None, environments=None, n=24, k=6, epsilons=(0.05,),
             num_queries=None, queries=None, seed=7, seeds=None, query_attempts=10000,
             query_k=None, workers=1, metrics=None, robot=None, base_experiment=None,
             validation=None, search=None, rank_by=None, rank_direction=None,
             input_dir=None, resume=False, dry_run=False, quiet=False,
             profile="bare_ur5e", camera=None, constraints=None, verification=None,
             selection=None, render=None, export=None, generate_only=False):
    """Run environment × query × seed × N × K × epsilon from one call.

    Scalar or vector N/K/epsilons are accepted and crossed. Wall dimensions are
    metres, yaw angles degrees; wall_random also accepts {min, max} dimensions.
    `queries` accepts task dictionaries or a saved queries.json / NPZ bank.
    Otherwise template goals are used first, then seeded random free endpoints
    in the FIRST environment, held constant for all comparisons. See CASES.md.
    `dry_run=True` returns the plan without writing files or building roadmaps.
    """
    if case_id not in CASES:
        raise ValueError(f"Unknown case_id {case_id!r}; choose {', '.join(CASES)}")
    base_dir = Path(input_dir or Path.cwd()).resolve()
    if isinstance(selection,dict) and "file" in selection:
        selection=dict(selection,file=str((base_dir/selection["file"]).resolve()))
    budget = _number(max_environments, "max_environments", 1, True)
    seed = _number(seed, "seed", 0, True)
    ns, ks = _values(n, "n", 0, True), _values(k, "k", 1, True)
    eps = _values(epsilons, "epsilons", 0)
    repeats = _values(seed if seeds is None else seeds, "seeds", 0, True)
    attempts = _number(query_attempts, "query_attempts", 1, True)
    workers = _number(workers, "workers", 1, True)
    raw = _base_experiment(base_experiment, robot, metrics, base_dir, profile)
    if camera is not None:
        raw["camera"] = deepcopy(camera)
    if (case_id == "fixed_environment" and environment is None and environments is None
            and base_experiment is not None and walls is None and wall_count is None):
        environment = raw["environment"]
    if environment is not None and environments is not None:
        raise ValueError("Choose environment or environments")
    if case_id == "environment_list":
        if environment is not None or not isinstance(environments, (list, tuple)) or not environments:
            raise ValueError("environment_list requires environments=[scene1, scene2, ...]")
        if wall_count is not None or walls is not None:
            raise ValueError("Supplied environments already define their walls")
        scene_list = [dict(scene=_environment(e, base_dir), parameters={}) for e in environments[:budget]]
        total, env_spec = len(environments), {"method": "provided"}
    elif case_id == "fixed_environment" and environment is not None:
        if wall_count is not None or walls is not None:
            raise ValueError("The supplied environment already defines its walls")
        scene_list = [dict(scene=_environment(environment, base_dir), parameters={})]
        total, env_spec = 1, {"method": "provided"}
    else:
        if environment is not None or environments is not None:
            raise ValueError("Use fixed_environment or environment_list for supplied scenes")
        scene_list, total, env_spec = _wall_environments(case_id, wall_count, walls, grid_points, offset_range,
                                                       widths, heights, thicknesses, yaw_offsets, radius, budget, seed)
    supplied = _read_queries(queries, base_dir)
    query_count = (len(supplied) if supplied is not None else 1) if num_queries is None else _number(num_queries, "num_queries", 1, True)
    if supplied is not None and len(supplied) != query_count:
        raise ValueError("num_queries must match the supplied query bank; omit it to use all supplied queries")
    raw["environment"] = scene_list[0]["scene"]
    raw["prm"].update(n=ns[0], k=ks[0], workers=workers)
    if query_k is not None:
        raw["prm"]["query_k"] = _number(query_k, "query_k", 1, True)
    if validation is not None:
        raw["validation"].update(validation)
    if constraints is not None:
        from .verification import apply_constraints
        verification = apply_constraints(raw, constraints, verification)
    if search is not None:
        if "epsilon" in search:
            raise ValueError("Set epsilons=... instead of search.epsilon")
        raw["search"].update(search)
    raw["search"]["epsilon"] = eps[0]
    base = load_data(raw, base_dir)
    if verification is not None:
        from .verification import options as verification_options
        verification = verification_options(verification)
    if any(x is not None for x in (verification,selection,render,export)):
        from .workflow import validate_stages
        validate_stages(verification,selection,render,export)
    metric_names = [m["name"] for m in base["metrics"]]
    rank_by = rank_by or "best." + metric_names[0]
    if rank_by != "path_count" and (not rank_by.startswith("best.") or rank_by[5:] not in metric_names):
        raise ValueError("rank_by needs path_count or best.<configured metric name>")
    direction = rank_direction or ("max" if rank_by == "path_count" else "min")
    if direction not in ("min", "max"):
        raise ValueError("rank_direction must be min or max")
    plan = dict(case_id=case_id, environments=len(scene_list), finite_environment_count=total,
                grid_exhausted=total == len(scene_list) if total is not None else False,
                num_queries=query_count, n=ns, k=ks, epsilons=eps, seeds=repeats,
                query_k=base["prm"]["query_k"], metrics=metric_names,
                roadmaps=len(scene_list)*query_count*len(repeats)*len(ns)*len(ks),
                runs=len(scene_list)*query_count*len(repeats)*len(ns)*len(ks)*len(eps),
                query_source="supplied" if supplied is not None else "template goals, then seeded random free endpoints in first environment",
                rank_by=rank_by, rank_direction=direction)
    if dry_run:
        return dict(dry_run=True, **plan)
    if generate_only:
        from .artifacts import save_scenes
        return save_scenes(output, scene_list, plan, base, supplied, resume)
    backend = make_backend(base)
    Metrics(base)  # Fail unknown cost functions before creating a study.
    bank = _query_bank(base, supplied, query_count, backend, seed, attempts)
    external = {obj["mesh"]: digest(obj["mesh"]) for entry in scene_list for obj in entry["scene"]["objects"] if "mesh" in obj}
    request = dict(plan=plan, environment_spec=env_spec, environments=scene_list, queries=bank,
                   seed=seed, query_attempts=attempts, base=signature(base, backend), external_files=external)
    fingerprint = hashlib.sha256(_json_text(request).encode()).hexdigest()
    directory = Path(output).resolve()
    directory.mkdir(parents=True, exist_ok=True)
    rows = []
    with exclusive(directory):
        marker = directory / "study_inputs.json"
        if marker.exists():
            old = json.loads(marker.read_text())
            if not resume or old["fingerprint"] != fingerprint:
                raise ValueError("Study exists or inputs/code/dependencies changed; resume identical inputs or choose a new output directory")
        else:
            if any(p.name != ".run.lock" for p in directory.iterdir()):
                raise ValueError("Study output must be empty")
            atomic_json(marker, dict(fingerprint=fingerprint, **request))
        # Check all previously materialized inputs before updating any reports.
        # This also catches an edited config belonging to a later job on resume.
        inputs_manifest = directory / "generated_inputs.json"
        hashes = json.loads(inputs_manifest.read_text()) if inputs_manifest.exists() else {}
        for name, expected in hashes.items():
            path = directory / name
            if path.is_file() and digest(path) != expected:
                raise ValueError(f"Generated input was edited: {path}. Use a new output directory.")
        _expected_file(directory / "plan.json", _json_text(plan))
        _expected_file(directory / "queries.json", _json_text(dict(queries=bank, source=plan["query_source"], seed=seed)))
        for i, entry in enumerate(scene_list):
            _expected_file(directory / f"environments/env_{i:05d}/scene.json", _json_text(entry["scene"]))
        for path in [directory / "plan.json", directory / "queries.json", *sorted((directory / "environments").glob("*/scene.json"))]:
            hashes[str(path.relative_to(directory))] = digest(path)
        atomic_json(inputs_manifest, hashes)
        if not quiet:
            print(json.dumps(dict(stage="study_plan", **plan)), flush=True)
        try:
            for ei, entry in enumerate(scene_list):
                for qi, q in enumerate(bank):
                    for run_seed, count, neighbors in itertools.product(repeats, ns, ks):
                        graph_source = None
                        for epsilon in eps:
                            run_id = f"run_{len(rows):07d}"
                            config_file = directory / "configs" / f"{run_id}.yaml"
                            cfg = portable_config(base)
                            cfg.update(environment=str(directory / f"environments/env_{ei:05d}/scene.json"), query=q, seed=run_seed)
                            cfg["prm"].update(n=count, k=neighbors)
                            cfg["search"]["epsilon"] = epsilon
                            _expected_file(config_file, yaml.safe_dump(cfg, sort_keys=False))
                            hashes[str(config_file.relative_to(directory))] = digest(config_file)
                            atomic_json(inputs_manifest, hashes)
                            cfg = load(config_file)
                            run_dir = directory / "runs" / run_id
                            row = dict(run_id=run_id, environment_id=f"env_{ei:05d}", query_id=f"query_{qi:05d}",
                                       seed=run_seed, n=count, k=neighbors, epsilon=epsilon, score=None, paths=0,
                                       config=f"configs/{run_id}.yaml", report=None)
                            try:
                                result = run(cfg, run_dir, resume=resume, quiet=True, roadmap_source=graph_source)
                                row.update({key: result[key] for key in ("status", "complete", "paths", "metrics", "costs", "elapsed_s")})
                                row["report"] = f"runs/{run_id}/report.html"
                                if result["complete"] and result["paths"]:
                                    row["score"] = float(result["paths"]) if rank_by == "path_count" else min(c[result["metrics"].index(rank_by[5:])] for c in result["costs"])
                                if graph_source is None:
                                    graph_source = run_dir
                            except InfeasibleQuery as error:
                                row.update(status="invalid_query", complete=True, error=str(error))
                            except Exception as error:
                                row.update(status="failed", complete=False, error=f"{type(error).__name__}: {error}")
                            rows.append(row)
                            atomic_json(directory / "study_progress.json", dict(evaluated=len(rows), requested=plan["runs"], rows=rows))
                            if not quiet:
                                progress = {key: row[key] for key in ("run_id", "environment_id", "query_id", "n", "k", "epsilon", "seed", "status", "paths")}
                                if "error" in row:
                                    progress["error"] = row["error"]
                                print(json.dumps(dict(stage="study_run", evaluated=len(rows), requested=plan["runs"], **progress)), flush=True)
        finally:
            result = _summary(plan, rows, rank_by, direction)
            _write_results(directory, result)
    result = dict(result, output=str(directory), report=str(directory / "report.html"))
    if any(x is not None for x in (verification, selection, render, export)):
        from .workflow import postprocess
        result["analysis"] = postprocess(directory, verification=verification,
                                          selection=selection, render=render, export=export)
    return result
