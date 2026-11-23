from pathlib import Path
import importlib.util
import json
import sys
import types
from copy import deepcopy
import numpy as np
import pytest
from prmkit.cli import main
from prmkit.config import load
from prmkit.backends import make_backend
from prmkit.geometry import sample_edge,certify
from prmkit.metrics import Metrics
from prmkit.pipeline import run,validate_path

pin=pytest.importorskip('pinocchio')
pytest.importorskip('coal')


def robot_project(tmp_path,template):
    path=tmp_path/template
    assert main(['init',str(path),'--template',template])==0
    return load(path/'experiment.yaml')


def test_different_robot_runs_end_to_end(tmp_path):
    cfg=robot_project(tmp_path,'two_link');cfg['prm']['n']=16;cfg['search']['engine']='python'
    result=run(cfg,tmp_path/'out',quiet=True)
    assert result['complete'] and result['paths']>0
    b=make_backend(cfg);assert len(b.lower)==2
    assert validate_path(cfg,tmp_path/'out/paths.npz')['valid']


def test_mounted_ur5e_has_original_pairs_and_valid_short_query(tmp_path):
    cfg=robot_project(tmp_path,'ur5e');cfg['prm']['n']=0;cfg['search']['engine']='python'
    b=make_backend(cfg)
    assert len(b.pairs)==76 and int(b.world_mask.sum())==25
    assert not any('wrist_3_link' in p and any(x.startswith('cable_') for x in p) for p in b.pairs)
    result=run(cfg,tmp_path/'out',quiet=True)
    assert result['complete'] and result['paths']>0
    assert validate_path(cfg,tmp_path/'out/paths.npz')['valid']


def historical_checker(cfg,tmp_path,monkeypatch):
    # Execute a byte-for-byte preserved reference, with only its filesystem
    # wiring supplied by this independent adapter. No research modules imported.
    robot=cfg['robot'];env=deepcopy(cfg['environment'])
    env['robot']={'urdf':robot['urdf']}
    env['objectives']={'joint_path_weights':robot['weights']}
    path=tmp_path/'reference_env.json';path.write_text(json.dumps(env))
    links=['base_link_inertia','shoulder_link','upper_arm_link','forearm_link','wrist_1_link','wrist_2_link','wrist_3_link']
    class Base:
        def __init__(self,envpath):
            self.environment=json.loads(Path(envpath).read_text())
            self.model=pin.buildModelFromUrdf(robot['urdf'])
            self.lower=self.model.lowerPositionLimit;self.upper=self.model.upperPositionLimit
            self.self_pairs=json.loads(Path(robot['pair_policy']).read_text())['active_pairs']
    stub=types.ModuleType('core');stub.EnvironmentCoalChecker=Base;stub.LINKS=links
    monkeypatch.setitem(sys.modules,'core',stub)
    path=Path(__file__).parent/'reference/assembly.py'
    spec=importlib.util.spec_from_file_location('historical_assembly_test',path)
    module=importlib.util.module_from_spec(spec)
    before=sys.path[:]
    spec.loader.exec_module(module)
    sys.path[:]=before
    module.OUT=Path(robot['convex_file']).parent
    return module.AssemblyChecker(tmp_path/'reference_env.json')


@pytest.mark.parametrize('case',['E4','E5','E7'])
def test_geometry_and_cost_equivalence_to_preserved_research(case,tmp_path,monkeypatch):
    cfg=robot_project(tmp_path,case)
    b=make_backend(cfg);old=historical_checker(cfg,tmp_path,monkeypatch)
    with np.load(tmp_path/case/f'assets/{case}/reference_path.npz') as z:
        q=z['q']
    pairs={frozenset(pair):i for i,pair in enumerate(b.pairs)}
    order=[pairs[frozenset(pair)] for pair in old.pairs]
    for x in q[np.unique(np.linspace(0,len(q)-1,min(10,len(q)),dtype=int))]:
        ds,ee=old.state(x);state=b.state(x)
        np.testing.assert_allclose(state.distances[order],ds,atol=1e-9,rtol=1e-9)
        np.testing.assert_allclose(state.ee,ee,atol=1e-12)
    cfg['metrics']=cfg['metrics'][:2]
    for i in sorted(set([0,(len(q)-2)//2,len(q)-2])):
        expected=old.edge_reference(q[i],q[i+1],intervals=16)
        assert expected is not None
        s=sample_edge(b,q[i],q[i+1],16,{})
        assert certify(b,s,cfg['validation'])>.01
        np.testing.assert_allclose(Metrics(cfg).evaluate(s),expected[0],rtol=1e-8,atol=1e-10)


def test_prismatic_robot_motion_bound(tmp_path):
    urdf=tmp_path/'slide.urdf'
    urdf.write_text('''<robot name="slide"><link name="base"/><link name="tool"><collision><geometry><sphere radius="0.02"/></geometry></collision></link><joint name="slide" type="prismatic"><parent link="base"/><child link="tool"/><axis xyz="1 0 0"/><limit lower="0" upper="1" effort="10" velocity="1"/></joint></robot>''')
    cfg=dict(robot=dict(backend='urdf',urdf=str(urdf),tool_frame='tool'),environment=dict(objects=[dict(name='wall',position_m=[.5,0,0],dimensions_m=[.001,.2,.2])]))
    b=make_backend(cfg)
    s=sample_edge(b,np.array([.1]),np.array([.9]),1,{})
    assert certify(b,s,dict(margin=.01,guard=2e-6,max_depth=18)) is None


def test_animated_usd_opens_and_matches_fk(tmp_path):
    Usd=pytest.importorskip('pxr.Usd');UsdGeom=pytest.importorskip('pxr.UsdGeom')
    from prmkit.usd import export_usd
    cfg=robot_project(tmp_path,'ur5e');cfg['prm']['n']=0;cfg['search']['engine']='python'
    run(cfg,tmp_path/'out',quiet=True)
    target=tmp_path/'replay.usda'
    result=export_usd(cfg,tmp_path/'out/paths.npz',target)
    assert result['frames']>1
    stage=Usd.Stage.Open(str(target));assert stage
    assert stage.GetPrimAtPath('/World/WristCamera')
    assert stage.GetEndTimeCode()==result['frames']-1
    # Last wrist transform must equal independent FK of the exported goal.
    b=make_backend(cfg);b.state(np.array(cfg['query']['goals'][0]))
    g=b.vis_geoms[6];expected=(b.data.oMi[g.parentJoint]*g.placement).homogeneous.T
    prim=stage.GetPrimAtPath('/World/body_6_wrist_3_link')
    actual=np.array(UsdGeom.Xformable(prim).GetLocalTransformation(Usd.TimeCode(stage.GetEndTimeCode())))
    np.testing.assert_allclose(actual,expected,atol=1e-10)


@pytest.mark.parametrize('case',['E4','E5','E7'])
def test_cpu_visibility_matches_archived_warp_samples(case,tmp_path):
    cfg=robot_project(tmp_path,case);b=make_backend(cfg)
    with np.load(Path(__file__).parent/'fixtures/visibility_reference.npz') as z:
        q=z[case+'_q'];expected=z[case+'_visibility']
    actual=np.array([b.visibility(x,cfg['camera']) for x in q])
    assert np.any(expected>0)
    np.testing.assert_array_equal(actual,expected)
