"""User entry point; all error messages include an actionable command context."""
import argparse
import importlib.metadata
import importlib.resources
import json
from pathlib import Path
import shutil
import sys

from . import __version__
from .config import load
from .io import jsonable


def main(argv=None):
    p = argparse.ArgumentParser(prog="prmkit", description="Portable multiobjective robot planning")
    p.add_argument("--version", action="version", version=__version__)
    sub = p.add_subparsers(dest="command", required=True)
    init = sub.add_parser("init", help="Create a runnable editable project")
    init.add_argument("directory", type=Path)
    init.add_argument("--template", choices=["demo", "ur5e", "mounted_ur5e", "isaac", "E4", "E5", "E7", "two_link"], default="demo")
    sub.add_parser("doctor", help="Inspect installed core and optional dependencies")
    from .cases import CASES
    cmd = sub.add_parser("case", help="Run a research case without editing YAML")
    cmd.add_argument("case_id", choices=CASES)
    cmd.add_argument("--output", type=Path, required=True)
    cmd.add_argument("--walls", type=int, dest="wall_count")
    cmd.add_argument("--grid-points", type=int, default=3)
    cmd.add_argument("--widths", type=float, nargs="+", default=[0.6])
    cmd.add_argument("--heights", type=float, nargs="+", default=[0.9])
    cmd.add_argument("--thicknesses", type=float, nargs="+", default=[0.08])
    cmd.add_argument("--n", type=int, nargs="+", default=[24])
    cmd.add_argument("--k", type=int, nargs="+", default=[6])
    cmd.add_argument("--epsilons", type=float, nargs="+", default=[0.05])
    cmd.add_argument("--num-queries", type=int)
    cmd.add_argument("--queries", type=Path, help="Saved query-bank JSON or NPZ")
    cmd.add_argument("--environment", type=Path)
    cmd.add_argument("--environments", type=Path, nargs="+")
    cmd.add_argument("--max-environments", type=int, default=100)
    cmd.add_argument("--seed", type=int, default=7)
    cmd.add_argument("--query-k", type=int)
    cmd.add_argument("--workers", type=int, default=1)
    cmd.add_argument("--resume", action="store_true")
    cmd.add_argument("--dry-run", action="store_true")
    cmd.add_argument("--quiet", action="store_true")
    cmd.add_argument("--profile", choices=["bare_ur5e","mounted_ur5e"], default="bare_ur5e")
    cmd.add_argument("--verify", action="store_true", help="Run an independent COAL mesh audit afterwards")
    cmd.add_argument("--generate-only", action="store_true")
    cmd = sub.add_parser("generate", help="Generate scenes and offline previews without building roadmaps")
    cmd.add_argument("config", type=Path)
    cmd.add_argument("--output", type=Path)
    cmd.add_argument("--resume", action="store_true")
    cmd.add_argument("--set", action="append", default=[])
    cmd = sub.add_parser("preview", help="Write a lightweight XY scene footprint SVG")
    cmd.add_argument("scene", type=Path)
    cmd.add_argument("--output", type=Path, required=True)
    cmd = sub.add_parser("rerun", help="Reuse saved scenes/queries with different N/K/epsilon")
    cmd.add_argument("directory", type=Path)
    cmd.add_argument("--output", type=Path, required=True)
    for flag in ("n","k"):
        cmd.add_argument("--"+flag, type=int, nargs="+")
    cmd.add_argument("--epsilons", type=float, nargs="+")
    cmd.add_argument("--resume", action="store_true")
    cmd.add_argument("--dry-run", action="store_true")
    cmd.add_argument("--quiet", action="store_true")
    cmd = sub.add_parser("solve", help="Search a saved single-run graph with a new epsilon/timeout")
    cmd.add_argument("directory", type=Path)
    cmd.add_argument("--output", type=Path, required=True)
    cmd.add_argument("--epsilon", type=float)
    cmd.add_argument("--timeout-seconds", type=float)
    cmd.add_argument("--max-labels", type=int)
    cmd.add_argument("--resume", action="store_true")
    cmd.add_argument("--quiet", action="store_true")
    cmd = sub.add_parser("verify", help="Audit saved paths independently; never reruns the solver")
    cmd.add_argument("directory", type=Path)
    cmd.add_argument("--backend", choices=["coal_mesh","planning"], default="coal_mesh")
    cmd.add_argument("--scope", choices=["all","representative","selected"], default="all")
    cmd.add_argument("--selection", type=Path)
    cmd.add_argument("--min-clearance-m", type=float, default=0.)
    cmd.add_argument("--max-joint-step-deg", type=float, default=.5)
    cmd.add_argument("--max-translation-step-m", type=float, default=.002)
    for name in ("explore","status"):
        sub.add_parser(name).add_argument("directory",type=Path)
    cmd = sub.add_parser("select", help="Save deterministic representative path IDs")
    cmd.add_argument("directory",type=Path)
    cmd.add_argument("--output",type=Path)
    cmd.add_argument("--include-unverified",action="store_true")
    cmd.add_argument("--max-per-query",type=int,default=7)
    for name in ("render","export"):
        cmd=sub.add_parser(name,help="Process only selected saved paths; no search")
        cmd.add_argument("directory",type=Path)
        cmd.add_argument("--selection",type=Path,required=True)
        cmd.add_argument("--output",type=Path)
        cmd.add_argument("--only-verified",action="store_true")
        if name=="render":
            cmd.add_argument("--frames",type=int,default=48)
            cmd.add_argument("--fps",type=int,default=24)
        else:
            cmd.add_argument("--usd",action="store_true")
    cmd = sub.add_parser("import-usd", help="Convert a local Isaac/USD stage to a portable static environment")
    cmd.add_argument("source", type=Path)
    cmd.add_argument("--output", type=Path, required=True, help="New output directory")
    cmd.add_argument("--root", default="/", help="Import only this absolute prim subtree")
    cmd.add_argument("--exclude", action="append", default=[], help="Exclude a prim subtree, e.g. /World/Robot; repeatable")
    cmd.add_argument("--geometry", choices=["collision", "all"], default="collision")
    cmd.add_argument("--robot-frame", help="Express obstacles in this rigid prim's frame (URDF base alignment)")
    cmd.add_argument("--snapshot-time", type=float, help="Explicitly freeze authored poses at this USD time code")
    cmd.add_argument("--meters-per-unit", type=float, help="Override source stage units")
    cmd.add_argument("--up-axis", choices=["Y", "Z"], help="Override source stage up axis")
    cmd.add_argument("--mesh-policy", choices=["authored", "triangles", "convex-hull"], default="authored")
    cmd.add_argument("--plane-size", type=float, nargs=2, metavar=("WIDTH", "LENGTH"), help="Finite plane footprint in metres")
    cmd.add_argument("--plane-thickness", type=float, default=0.1, help="Thickness below the plane in metres")
    for name in ("check", "run", "validate-path", "export-usd"):
        cmd = sub.add_parser(name)
        cmd.add_argument("config", type=Path)
        cmd.add_argument("--set", action="append", default=[], metavar="FIELD=VALUE")
        if name == "run":
            cmd.add_argument("--output", type=Path)
            cmd.add_argument("--resume", action="store_true")
            cmd.add_argument("--quiet", action="store_true")
            cmd.add_argument("--dry-run", action="store_true")
        if name in ("validate-path", "export-usd"):
            cmd.add_argument("path", type=Path)
            cmd.add_argument("--index", type=int, default=0)
        if name == "export-usd":
            cmd.add_argument("--output", type=Path, required=True)
            cmd.add_argument("--max-step", type=float, default=0.05)
            cmd.add_argument("--fps", type=float, default=24)
    cmd = sub.add_parser("sweep", help="Search a user-defined environment/parameter space")
    cmd.add_argument("space", type=Path)
    cmd.add_argument("--output", type=Path, required=True)
    cmd.add_argument("--resume", action="store_true")
    cmd.add_argument("--quiet", action="store_true")
    cmd = sub.add_parser("inspect", help="Verify saved outputs and print summary")
    cmd.add_argument("directory", type=Path)
    args = p.parse_args(argv)
    try:
        if args.command == "init":
            if args.directory.exists() and any(args.directory.iterdir()):
                raise ValueError("init destination must be absent or empty")
            resources = importlib.resources.files("prmkit") / "resources"
            with importlib.resources.as_file(resources) as root:
                args.directory.mkdir(parents=True, exist_ok=True)
                shutil.copytree(root / "templates" / args.template, args.directory, dirs_exist_ok=True)
                if args.template != "demo":
                    shutil.copytree(root / "assets", args.directory / "assets", dirs_exist_ok=True)
            if args.template == "isaac":
                print(f"Created {args.directory}. Next: prmkit import-usd {args.directory / 'scene.usda'} --output {args.directory / 'imported'}")
            else:
                print(f"Created {args.directory}. Next: prmkit check {args.directory / 'experiment.yaml'}")
            return 0
        if args.command == "doctor":
            result = dict(python=sys.version, kit=__version__, dependencies={})
            for name, module in [("numpy", "numpy"), ("scipy", "scipy"), ("PyYAML", "yaml"), ("pin", "pinocchio"), ("coal", "coal"), ("trimesh", "trimesh"), ("rtree", "rtree"), ("numba", "numba"), ("usd-core", "pxr.Usd")]:
                try:
                    __import__(module)
                    result["dependencies"][name] = dict(version=importlib.metadata.version(name), import_ok=True)
                except Exception as e:
                    result["dependencies"][name] = dict(import_ok=False, detail=str(e))
        elif args.command == "case":
            from .cases import run_case
            options = vars(args).copy()
            del options["command"]
            verify=options.pop("verify")
            if verify:
                options.update(verification={"enabled":True}, selection={"only_verified":True})
            result = run_case(**options)
        elif args.command == "generate":
            from .experiment import run_experiment
            result=run_experiment(args.config,output=args.output,overrides=args.set,resume=args.resume,generate_only=True)
        elif args.command == "preview":
            from .artifacts import scene_preview
            from .config import read
            result={"file":scene_preview(read(args.scene),args.output)}
        elif args.command == "rerun":
            from .experiment import rerun
            options=vars(args).copy();del options["command"]
            result=rerun(**options)
        elif args.command == "solve":
            from copy import deepcopy
            from .artifacts import run_inputs
            from .pipeline import run
            config=deepcopy(run_inputs(args.directory,True)["config"])
            for field,key in (("epsilon","epsilon"),("timeout_seconds","timeout_s"),("max_labels","max_labels")):
                if getattr(args,field) is not None:
                    config["search"][key]=getattr(args,field)
            # Validate scalar search settings before a potentially large graph is loaded.
            from .config import number
            number(config["search"]["epsilon"],"epsilon")
            number(config["search"]["timeout_s"],"timeout",strict=True)
            number(config["search"]["max_labels"],"max_labels",2,integer=True)
            result=run(config,args.output,args.resume,args.quiet,roadmap_source=args.directory)
        elif args.command == "verify":
            from .verification import verify_paths
            from .explorer import explore
            result=verify_paths(args.directory,dict(backend=args.backend,scope=args.scope,min_clearance_m=args.min_clearance_m,
                interpolation=dict(max_joint_step_deg=args.max_joint_step_deg,max_translation_step_m=args.max_translation_step_m)),selection=args.selection)
            result["explorer"]=explore(args.directory)
        elif args.command == "explore":
            from .explorer import explore
            result=explore(args.directory)
        elif args.command == "select":
            from .selection import select_paths
            result=select_paths(args.directory,dict(only_verified=not args.include_unverified,max_per_query=args.max_per_query),output=args.output)
        elif args.command in ("render","export"):
            from .delivery import render_paths,export_paths
            options=vars(args).copy();del options["command"]
            result=(render_paths if args.command=="render" else export_paths)(**options)
        elif args.command == "status":
            directory=args.directory
            names=["study_summary.json","summary.json","generation.json","study_progress.json","progress.json"]
            path=next((directory/name for name in names if (directory/name).exists()),None)
            if path is None:
                raise ValueError("No recognized run/study/generation status in this directory")
            record=json.loads(path.read_text())
            result={key:record[key] for key in ("complete","stage","status","evaluated","requested","paths","plan") if key in record}
        elif args.command == "import-usd":
            from .usd_import import import_usd
            options = vars(args).copy()
            del options["command"]
            result = import_usd(**options)
        elif args.command == "sweep":
            from .spaces import sweep
            result = sweep(args.space, args.output, args.resume, args.quiet)
        elif args.command == "inspect":
            from .pipeline import verify_manifest
            verify_manifest(args.directory)
            result = json.loads((args.directory / "summary.json").read_text())
        else:
            from .pipeline import check, run, validate_path
            from .config import read
            if args.command == "run" and read(args.config).get("schema_version") == 2:
                from .experiment import run_experiment
                result=run_experiment(args.config,output=args.output,overrides=args.set,resume=args.resume,dry_run=args.dry_run,quiet=args.quiet)
                print(json.dumps(jsonable(result),indent=2,allow_nan=False))
                return 0 if result.get("complete",True) and result.get("analysis",{}).get("complete",True) else 3
            config = load(args.config, args.set)
            if args.command == "check":
                result = check(config)
            elif args.command == "run":
                if args.output is None:
                    raise ValueError("Legacy schema_version:1 runs require --output")
                if args.dry_run:
                    result=dict(dry_run=True,runs=1,n=config["prm"]["n"],k=config["prm"]["k"],epsilon=config["search"]["epsilon"])
                else:
                    result = run(config, args.output, args.resume, args.quiet)
            elif args.command == "export-usd":
                from .usd import export_usd
                result = export_usd(config, args.path, args.output, args.index, args.max_step, args.fps)
            else:
                result = validate_path(config, args.path, args.index)
        print(json.dumps(jsonable(result), indent=2, allow_nan=False))
        if result.get("complete") is False or result.get("valid") is False or result.get("analysis",{}).get("complete") is False:
            return 3
        if result.get("status") == "no_path_in_roadmap":
            return 4
        return 0
    except (ValueError, KeyError, FileNotFoundError, ImportError, RuntimeError) as e:
        print(f"prmkit: {type(e).__name__}: {e}", file=sys.stderr)
        return 2
    except KeyboardInterrupt:
        print("Interrupted. Run the same command with --resume to reuse completed construction checkpoints.", file=sys.stderr)
        return 130
