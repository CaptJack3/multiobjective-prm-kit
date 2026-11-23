"""Edit SETTINGS below, then: python run_experiment.py

No YAML editing or init command is required. First install: pip install '.[robot]'
Use --plan for a preview, --resume for an interrupted unchanged study.
Results: runs/<case_id>/report.html, results.csv, study_summary.json.
"""
import argparse
import json
from pathlib import Path

from prmkit import run_case

ROOT = Path(__file__).resolve().parent

# Choose: wall_grid | wall_random | wall_sobol | fixed_environment | environment_list
CASE_ID = "wall_grid"

SETTINGS = dict(
    profile="mounted_ur5e",       # UR5e + fixed-open Robotiq 2F-85 + D435/PickNik
    wall_count=2,
    grid_points=2,                 # positions PER wall; 2 walls => 2**2 layouts
    offset_range=(-0.2, 0.2),      # displacement along each wall's move_axis [m]
    widths=[0.6],                 # full wall width [m]; e.g. [0.4, 0.6, 0.8]
    heights=[0.9],                # height [m]; the bottom remains at base_z
    thicknesses=[0.08],           # full thickness [m]
    yaw_offsets=[0],              # rotation relative to default wall yaw [deg]
    max_environments=4,           # grid prefix, or number of random samples
    n=[24],                      # e.g. [250, 1000, 5000]
    k=[6],                       # e.g. [8, 16, 32]
    epsilons=[0.0, 0.05],         # graph/costs reused across epsilons
    num_queries=1,                # independent planning tasks, held fixed
    seed=7,
    query_k=12,
    workers=1,
    # Planner margin is a hard constraint; independent mesh audit is a postcheck.
    constraints=dict(min_clearance_m=0.005, clearance_mode="enforce_during_search"),
    verification=dict(enabled=True, backend="coal_mesh", scope="all",
                      min_clearance_m=0.005),
    selection=dict(only_verified=True, max_per_query=7),
    render=dict(enabled=False),   # Render a saved selection later; no search rerun.
)

# To densify one SAVED environment, replace SETTINGS above with:
# SETTINGS = dict(
#     profile="mounted_ur5e",
#     environment=ROOT / "runs/wall_grid/environments/env_00000/scene.json",
#     queries=ROOT / "runs/wall_grid/queries.json",  # preserve the same tasks
#     n=[1000, 5000], k=[16, 32], epsilons=[0, 0.05], query_k=32,
# )
# CASE_ID = "fixed_environment"
#
# To create one fixed two-wall environment without a file:
# CASE_ID = "fixed_environment"
# SETTINGS = dict(profile="mounted_ur5e", wall_count=2, widths=0.6, heights=0.9,
#                 n=[100, 1000], k=[8, 16], epsilons=[0, 0.05])
#
# For continuous sizes, choose wall_random and set, for example:
# widths={"min": 0.3, "max": 0.9}, heights={"min": 0.5, "max": 1.2}
#
# For different walls, provide walls=[...] instead of wall_count. Example:
# walls=[dict(center_xy=[1,0], move_axis=[1,0], yaw_degs=[0,15],
#             widths=[0.4,0.8], heights=[0.6,0.9], offsets=[-0.2,0,0.2]),
#        dict(center_xy=[0,1], move_axis=[0,1], yaw_degs=[90],
#             widths=[0.5], heights=[1.0], thicknesses=[0.1])]
#
# For your own task, use queries=[{"start": [...], "goal": [...]}].
# For multiple saved scenes: CASE_ID="environment_list" and
# SETTINGS=dict(environments=["one.json","two.json"], queries="queries.json", ...).


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--plan", action="store_true", help="Print counts without running or writing")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--output", type=Path, help="New output folder for changed settings")
    args = parser.parse_args()
    result = run_case(CASE_ID, output=args.output or ROOT / "runs" / CASE_ID,
                      input_dir=ROOT, dry_run=args.plan, resume=args.resume, **SETTINGS)
    if args.plan:
        print(json.dumps(result, indent=2))
    else:
        print(f"Report: {result['report']}")
        if "analysis" in result:
            print(f"Explorer: {result['analysis']['explorer']['file']}")
        print(f"Completed {result['evaluated']}/{result['requested']}; complete={result['complete']}")
    return 0 if result.get("complete", True) and result.get("analysis", {}).get("complete", True) else 3


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        print("Interrupted. Rerun unchanged settings with --resume.")
        raise SystemExit(130)
