"""A small experiment-document adapter onto run_case, not another planner."""
from copy import deepcopy
import itertools
import json
from pathlib import Path

import yaml

from .artifacts import content_hash, run_inputs
from .cases import run_case
from .config import checked_keys, read
from .io import atomic_json, digest


def resolve_experiment(path, overrides=()):
    path=Path(path).resolve(); raw=read(path)
    for item in overrides:
        field,separator,value=item.partition("=")
        if not separator:
            raise ValueError("--set requires section.field=value")
        target=raw
        for key in field.split(".")[:-1]:
            if key not in target or not isinstance(target[key],dict):
                raise ValueError(f"Unknown override section: {field}")
            target=target[key]
        target[field.split(".")[-1]]=yaml.safe_load(value)
    checked_keys(raw,["schema_version","experiment","robot","scene","queries","roadmap","objectives","camera","search","constraints","verification","selection","render","export","output"],"experiment document")
    if raw.get("schema_version") != 2:
        raise ValueError("Unified experiment document needs schema_version: 2")
    info=raw.get("experiment",{});checked_keys(info,["name","seed","base_experiment"],"experiment")
    scene=raw.get("scene",{});checked_keys(scene,["source","placement","count","walls_per_scene","grid_points","offset_range","wall_sizes","yaw_offsets","radius","walls","file","files","bounds_frame"],"scene")
    if scene.get("bounds_frame","robot_base") != "robot_base":
        raise ValueError("Generated walls use robot_base coordinates; transform external scenes before planning")
    source=scene.get("source","generated")
    if source=="generated":
        placement=scene.get("placement","grid")
        cases={"grid":"wall_grid","random":"wall_random","sobol":"wall_sobol","manual":"fixed_environment"}
        if placement not in cases:
            raise ValueError("scene.placement must be grid, random, sobol or manual; guided is not implemented")
        case=cases[placement]
    elif source in ("saved","isaac_export"):
        if bool(scene.get("file"))==bool(scene.get("files")):
            raise ValueError("Saved scenes need exactly one of scene.file or scene.files")
        case="environment_list" if "files" in scene else "fixed_environment"
    else:
        raise ValueError("scene.source must be generated, saved or isaac_export (already imported scene.json)")
    robot=raw.get("robot",{});checked_keys(robot,["preset","definition"],"robot selection")
    roadmap=raw.get("roadmap",{});checked_keys(roadmap,["n_vertices","k_neighbors","query_k","workers","seeds"],"roadmap")
    search=raw.get("search",{});checked_keys(search,["epsilon","epsilons","timeout_seconds","max_labels","engine"],"search")
    if "epsilon" in search and "epsilons" in search:
        raise ValueError("Choose search.epsilon or search.epsilons")
    output=raw.get("output",{});checked_keys(output,["directory"],"output")
    opts=dict(profile=robot.get("preset","mounted_ur5e"), seed=info.get("seed",7), input_dir=path.parent,
              n=roadmap.get("n_vertices",24),k=roadmap.get("k_neighbors",6),query_k=roadmap.get("query_k",12),
              workers=roadmap.get("workers",1),max_environments=scene.get("count",4),
              epsilons=search.get("epsilons",search.get("epsilon",[.05])),
              output=(path.parent/output.get("directory","../runs/"+info.get("name",path.stem))).resolve())
    if "definition" in robot:
        opts["robot"]=robot["definition"]
    if "base_experiment" in info:
        opts["base_experiment"]=info["base_experiment"]
    if "seeds" in roadmap:
        opts["seeds"]=roadmap["seeds"]
    if source=="generated":
        fields={"walls_per_scene":"wall_count","grid_points":"grid_points","offset_range":"offset_range","yaw_offsets":"yaw_offsets","radius":"radius","walls":"walls"}
        opts.update({dest:scene[key] for key,dest in fields.items() if key in scene})
        sizes=scene.get("wall_sizes",{});checked_keys(sizes,["widths","heights","thicknesses"],"scene.wall_sizes")
        opts.update(sizes)
    else:
        opts["environments" if "files" in scene else "environment"]=scene.get("files",scene.get("file"))
    queries=raw.get("queries",{});checked_keys(queries,["mode","count","file","pairs","starts","goals","max_attempts"],"queries")
    mode=queries.get("mode","sampled")
    if mode=="fixed":
        if "file" in queries and "pairs" in queries:
            raise ValueError("Choose query file or explicit pairs")
        if "file" in queries or "pairs" in queries:
            opts["queries"]=queries.get("file",queries.get("pairs"))
    elif mode=="cartesian_product":
        if not queries.get("starts") or not queries.get("goals"):
            raise ValueError("cartesian_product requires explicit starts and goals joint vectors")
        opts["queries"]=[dict(start=s,goal=g) for s,g in itertools.product(queries["starts"],queries["goals"])]
    elif mode!="sampled":
        raise ValueError("queries.mode must be fixed, sampled or cartesian_product")
    if "count" in queries:
        opts["num_queries"]=queries["count"]
    opts["query_attempts"]=queries.get("max_attempts",10000)
    opts["search"]={dest:search[key] for key,dest in [("timeout_seconds","timeout_s"),("max_labels","max_labels"),("engine","engine")] if key in search}
    for key in ("camera","constraints","verification","selection","render","export"):
        if key in raw:
            opts[key]=deepcopy(raw[key])
    for key in ("render","export"):
        if isinstance(opts.get(key),dict) and "output" in opts[key]:
            opts[key]["output"]=str((path.parent/opts[key]["output"]).resolve())
    if "objectives" in raw:
        metrics=raw["objectives"]
        if isinstance(metrics,dict):
            checked_keys(metrics,["enabled"],"objectives");metrics=metrics["enabled"]
        opts["metrics"]=metrics
    # Every real experiment gets the offline explorer; verification remains explicit.
    opts.setdefault("selection",dict(only_verified=bool(raw.get("verification",{}).get("enabled",False)) if isinstance(raw.get("verification",{}),dict) else bool(raw.get("verification"))))
    return raw,case,opts


def run_experiment(path, *, output=None, overrides=(), resume=False, dry_run=False, generate_only=False, quiet=False):
    raw,case,opts=resolve_experiment(path,overrides)
    if output is not None:
        opts["output"]=Path(output).resolve()
    result=run_case(case,**opts,resume=resume,dry_run=dry_run,generate_only=generate_only,quiet=quiet)
    if not dry_run:
        folder=Path(opts["output"])/"workflow";folder.mkdir(exist_ok=True)
        identifier=content_hash(raw)[:20]
        requested=folder/("config_requested_"+identifier+".yaml")
        requested.write_text(yaml.safe_dump(raw,sort_keys=False))
        atomic_json(folder/("config_resolved_"+identifier+".json"),dict(case_id=case,settings=opts))
    return result


def rerun(directory, output, *, n=None, k=None, epsilons=None, resume=False, dry_run=False, quiet=False):
    root=Path(directory).resolve()
    if (root/"study_summary.json").exists():
        study=json.loads((root/"study_summary.json").read_text());plan=study["plan"]
        generated=json.loads((root/"generated_inputs.json").read_text())
        for name,expected in generated.items():
            if not (root/name).is_file() or digest(root/name)!=expected:
                raise ValueError(f"Saved study input was modified: {name}")
        first=next((r for r in study["rows"] if r.get("report")),None)
        if first is None:
            raise ValueError("Study has no saved planning configuration to rerun")
        source=root/"runs"/first["run_id"]
        record=run_inputs(source,True)
        environments=sorted((root/"environments").glob("*/scene.json"))
        opts=dict(environments=environments,queries=root/"queries.json",max_environments=len(environments),
                  n=plan["n"] if n is None else n,k=plan["k"] if k is None else k,
                  epsilons=plan["epsilons"] if epsilons is None else epsilons,seeds=plan["seeds"],query_k=plan["query_k"])
        case="environment_list"
    else:
        record=run_inputs(root,True);config=record["config"]
        opts=dict(environment=config["environment"],queries=[config["query"]],
                  n=config["prm"]["n"] if n is None else n,k=config["prm"]["k"] if k is None else k,
                  epsilons=config["search"]["epsilon"] if epsilons is None else epsilons,
                  seed=config["seed"],query_k=config["prm"]["query_k"])
        case="fixed_environment"
    return run_case(case,base_experiment=record["config"],output=output,**opts,
                    resume=resume,dry_run=dry_run,quiet=quiet,selection={"only_verified":False})
