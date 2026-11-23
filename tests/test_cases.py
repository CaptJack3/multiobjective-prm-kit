from copy import deepcopy
import json
from pathlib import Path

import numpy as np
import pytest

from prmkit import run_case, list_cases
from prmkit.cases import _summary
from prmkit.config import load, load_data
from prmkit.io import digest
from prmkit.pipeline import run, verify_manifest, validate_path


@pytest.fixture
def point_experiment():
    return dict(schema_version=1, robot=dict(backend="point", lower=[-2]*3, upper=[2]*3,
                                            radius=.01, weights=[1]*3),
                environment={"objects": []}, query=dict(start=[-.4, 0, .3], goals=[[.4, 0, .3]]),
                metrics=[dict(name="ee_length"), dict(name="clearance", threshold=.1)],
                validation=dict(margin=.01, intervals=4), prm=dict(n=4, k=2, query_k=8),
                search=dict(epsilon=.05, timeout_s=5))


def test_plan_does_not_write_or_load_robot(tmp_path, monkeypatch):
    import prmkit.cases as cases
    monkeypatch.setattr(cases, "make_backend", lambda *_: pytest.fail("dry-run loaded native robot"))
    out = tmp_path / "plan"
    plan = run_case(wall_count=2, grid_points=3, widths=[.4, .8], heights=[.6, 1],
                    max_environments=10, n=[10, 20], k=[2, 4], epsilons=[0, .1], num_queries=3,
                    output=out, dry_run=True)
    assert plan["finite_environment_count"] == 144
    assert plan["environments"] == 10 and not plan["grid_exhausted"]
    assert plan["runs"] == 240 and plan["roadmaps"] == 120
    assert not out.exists()
    assert set(list_cases()) == {"wall_grid", "wall_random", "wall_sobol", "fixed_environment", "environment_list"}


def test_grid_queries_cross_product_graph_reuse_resume(point_experiment, tmp_path, monkeypatch):
    import prmkit.roadmap as roadmap
    original = roadmap.sample_nodes
    calls = []
    def sampled(*args, **kwargs):
        calls.append(1)
        return original(*args, **kwargs)
    monkeypatch.setattr(roadmap, "sample_nodes", sampled)
    kwargs = dict(base_experiment=point_experiment, wall_count=1, grid_points=2,
                  n=[2, 4], k=2, epsilons=[0, .1],
                  queries=[dict(start=[-.4, 0, .3], goal=[.4, 0, .3]),
                           dict(start=[-.4, .1, .3], goal=[.4, .1, .3])], quiet=True)
    out = tmp_path / "study"
    result = run_case(output=out, **kwargs)
    assert result["complete"] and result["evaluated"] == 16
    assert len(calls) == 8  # one construction, not two, for each epsilon pair
    assert len(result["rankings"]) == 4
    assert all(r["status"] == "complete" for r in result["rows"])
    for a, b in zip(result["rows"][::2], result["rows"][1::2]):
        one, two = [out / "runs" / row["run_id"] for row in [a, b]]
        assert digest(one / "graph.npz") == digest(two / "graph.npz")
        assert (two / "roadmap_reuse.json").exists()
        assert verify_manifest(two)["complete"]
        assert validate_path(load(out / b["config"]), two / "paths.npz")["valid"]
    assert run_case(output=out, resume=True, **kwargs) == result
    assert len(calls) == 8
    summary = (out / "study_summary.json").read_bytes()
    (out / result["rows"][-1]["config"]).write_text("edited")
    with pytest.raises(ValueError, match="edited"):
        run_case(output=out, resume=True, **kwargs)
    assert (out / "study_summary.json").read_bytes() == summary


def test_fixed_scene_and_queries_can_be_reused_for_densification(point_experiment, tmp_path):
    first = tmp_path / "first"
    run_case("fixed_environment", wall_count=1, base_experiment=point_experiment,
             output=first, n=2, k=2, quiet=True)
    scene = first / "environments/env_00000/scene.json"
    later = run_case("fixed_environment", environment=scene, queries=first / "queries.json",
                     base_experiment=point_experiment, n=[3, 5], k=[1, 3], epsilons=[0, .1],
                     output=tmp_path / "dense", quiet=True)
    assert later["complete"] and later["evaluated"] == 8
    assert json.loads(scene.read_text()) == json.loads((tmp_path / "dense/environments/env_00000/scene.json").read_text())
    assert all(row["environment_id"] == "env_00000" for row in later["rows"])


def test_random_ranges_queries_reproducible_and_floor_anchored(point_experiment, tmp_path):
    args = dict(case_id="wall_random", wall_count=1, max_environments=3,
                widths={"min": .2, "max": .5}, heights={"min": .3, "max": 1.2},
                n=2, k=2, num_queries=3, base_experiment=point_experiment, quiet=True)
    one = run_case(output=tmp_path / "one", **args)
    two = run_case(output=tmp_path / "two", **args)
    assert one["complete"] and two["complete"]
    assert json.loads((tmp_path / "one/queries.json").read_text()) == json.loads((tmp_path / "two/queries.json").read_text())
    for i in range(3):
        a = json.loads((tmp_path / f"one/environments/env_{i:05d}/scene.json").read_text())
        b = json.loads((tmp_path / f"two/environments/env_{i:05d}/scene.json").read_text())
        assert a == b
        wall = a["objects"][0]
        assert .2 <= wall["dimensions_m"][1] <= .5
        assert wall["position_m"][2] - wall["dimensions_m"][2]/2 == 0


def test_heterogeneous_walls_and_grid_order(tmp_path):
    from prmkit.cases import _wall_environments
    specs = [dict(center_xy=[1, 0], move_axis=[1, 0], base_z=.2, widths=[.4, .8],
                  heights=[.6, 1], yaw_degs=[0, 30], offsets=[0]),
             dict(name="second", center_xy=[0, 1], move_axis=[0, 1], widths=[.7],
                  heights=[.9], yaw_degs=[90], offsets=[-.1, .1])]
    scenes, total, _ = _wall_environments("wall_grid", 2, specs, 3, (-.2, .2), .6, .9, .08, 0, 1, 16, 7)
    assert total == 16 and len(scenes) == 16
    assert scenes[0]["scene"]["objects"][1]["position_m"][1] == .9
    assert scenes[1]["scene"]["objects"][1]["position_m"][1] == 1.1
    for entry in scenes:
        wall = entry["scene"]["objects"][0]
        assert wall["position_m"][2] - wall["dimensions_m"][2]/2 == pytest.approx(.2)
        assert entry["scene"]["objects"][1]["rotation_rpy"][2] == pytest.approx(np.pi/2)


def test_infeasible_query_is_visible_and_not_ranked(point_experiment, tmp_path):
    blocked = {"objects": [dict(name="block", shape="box", position_m=[0, 0, 0], dimensions_m=[10, 10, 10])]}
    r = run_case("environment_list", environments=[{"objects": []}, blocked],
                 base_experiment=point_experiment, output=tmp_path / "out", n=2, k=2, quiet=True)
    assert r["complete"]
    assert [row["status"] for row in r["rows"]] == ["complete", "invalid_query"]
    assert [e["environment_id"] for e in r["rankings"][0]["environments"]] == ["env_00000"]


def test_rankings_prioritize_query_coverage_and_exclude_partial():
    rows = []
    for env, scores, complete in [("all", [100, 100], True), ("one", [1, None], True), ("partial", [.1, .1], False)]:
        for score in scores:
            rows.append(dict(environment_id=env, n=10, k=2, epsilon=0, seed=7, complete=complete, score=score))
    r = _summary(dict(num_queries=2, runs=6), rows, "best.length", "min")
    assert not r["complete"]
    assert [e["environment_id"] for e in r["rankings"][0]["environments"]] == ["all", "one"]


def test_query_bank_npz_and_changed_inputs(point_experiment, tmp_path):
    bank = tmp_path / "queries.npz"
    np.savez(bank, start=[-.4, 0, .3], goals=[[.4, 0, .3], [.4, .1, .3]])
    args = dict(case_id="fixed_environment", environment={"objects": []}, base_experiment=point_experiment,
                queries=bank, output=tmp_path / "out", n=2, k=2, quiet=True)
    r = run_case(**args)
    assert r["complete"] and r["plan"]["num_queries"] == 2
    with pytest.raises(ValueError, match="changed"):
        run_case(**dict(args, n=3), resume=True)
    with pytest.raises(ValueError, match="match"):
        run_case(**args, num_queries=1)


@pytest.mark.parametrize("kwargs", [dict(n=-1), dict(k=0), dict(epsilons=[0, 0]),
    dict(epsilons=[float("nan")]), dict(num_queries=0), dict(grid_points=0),
    dict(widths=0), dict(heights=[]), dict(wall_count=-1), dict(max_environments=0),
    dict(widths={"min": .2, "max": .4}), dict(search={"epsilon": .1}),
    dict(rank_by="best.nonexistent"), dict(walls=[dict(center_xy=[1])]),
    dict(case_id="typo"), dict(offset_range=(1, -1))])
def test_bad_case_inputs_fail_before_writes(kwargs, tmp_path):
    out = tmp_path / "out"
    with pytest.raises((ValueError, TypeError)):
        run_case(output=out, dry_run=True, **kwargs)
    assert not out.exists()


def test_graph_reuse_rejects_different_geometry_or_corruption(point_experiment, tmp_path):
    cfg = load_data(point_experiment, tmp_path)
    first = tmp_path / "first"
    run(cfg, first, quiet=True)
    changed = deepcopy(cfg)
    changed["prm"]["n"] += 1
    with pytest.raises(ValueError, match="identical graph"):
        run(changed, tmp_path / "different", quiet=True, roadmap_source=first)
    (first / "graph.npz").write_bytes(b"corrupt")
    with pytest.raises(ValueError, match="integrity"):
        run(cfg, tmp_path / "corrupt", quiet=True, roadmap_source=first)


def test_case_interruption_resumes(point_experiment, tmp_path, monkeypatch):
    import prmkit.cases as cases
    original = cases.run
    calls = []
    def interrupted(*args, **kwargs):
        calls.append(1)
        if len(calls) == 2:
            raise KeyboardInterrupt()
        return original(*args, **kwargs)
    monkeypatch.setattr(cases, "run", interrupted)
    args = dict(case_id="fixed_environment", environment={"objects": []}, base_experiment=point_experiment,
                output=tmp_path / "out", n=2, k=2, epsilons=[0, .1], quiet=True)
    with pytest.raises(KeyboardInterrupt):
        run_case(**args)
    partial = json.loads((tmp_path / "out/study_summary.json").read_text())
    assert not partial["complete"] and partial["evaluated"] == 1
    monkeypatch.setattr(cases, "run", original)
    final = run_case(**args, resume=True)
    assert final["complete"] and final["evaluated"] == 2


def test_case_cli_plan(tmp_path, capsys):
    from prmkit.cli import main
    assert main(["case", "wall_grid", "--walls", "2", "--grid-points", "2", "--n", "10", "20",
                 "--k", "2", "--epsilons", "0", "0.1", "--dry-run", "--output", str(tmp_path / "out")]) == 0
    result = json.loads(capsys.readouterr().out)
    assert result["runs"] == 16 and not (tmp_path / "out").exists()
