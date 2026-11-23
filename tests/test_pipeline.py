from copy import deepcopy
import json
from pathlib import Path
import numpy as np
import pytest
from prmkit.cli import main
from prmkit.config import load
from prmkit.pipeline import run,verify_manifest,validate_path
from prmkit.spaces import sweep


def test_full_run_resume_and_integrity(demo,tmp_path):
    out=tmp_path/'run'
    result=run(demo,out,quiet=True)
    assert result['complete'] and result['paths']>0
    assert verify_manifest(out)['complete']
    assert validate_path(demo,out/'paths.npz')['valid']
    again=run(demo,out,resume=True,quiet=True)
    assert again==result
    changed=deepcopy(demo);changed['search']['epsilon']=.2
    with pytest.raises(ValueError,match='changed'): run(changed,out,resume=True,quiet=True)
    (out/'front.csv').write_text('corrupt')
    with pytest.raises(ValueError,match='integrity'): run(demo,out,resume=True,quiet=True)


def test_process_workers_reproduce_sequential(demo,tmp_path):
    demo['prm'].update(n=20,k=4,query_k=8,edge_chunk=8)
    one=run(demo,tmp_path/'one',quiet=True)
    demo['prm']['workers']=2
    two=run(demo,tmp_path/'two',quiet=True)
    with np.load(tmp_path/'one/graph.npz') as a,np.load(tmp_path/'two/graph.npz') as b:
        for key in a.files: np.testing.assert_array_equal(a[key],b[key])
    assert one['costs']==two['costs']


def test_asymmetric_plugin_is_scored_in_both_directions(demo,tmp_path):
    f=tmp_path/'direction.py'
    f.write_text('def cost(s, p):\n    return s.joint_length * (1 if s.q[-1,0]>=s.q[0,0] else 3)\n')
    demo['metrics']=[dict(name='directed',plugin=str(f)+':cost')]
    demo['environment']['objects']=[];demo['prm'].update(n=0,k=1,query_k=1)
    run(demo,tmp_path/'out',quiet=True)
    with np.load(tmp_path/'out/graph.npz') as z:
        np.testing.assert_allclose(z['costs'].ravel(),[.8,2.4])


def test_sweep_and_resume(demo,tmp_path):
    space=Path(demo['_file']).with_name('space.yaml')
    r=sweep(space,tmp_path/'sweep',quiet=True)
    assert r['complete'] and r['evaluated']==6 and r['ranking']
    assert sweep(space,tmp_path/'sweep',resume=True,quiet=True)==r


def test_invalid_config_and_cli_exit(tmp_path):
    main(['init',str(tmp_path/'project')])
    p=tmp_path/'project/experiment.yaml'
    for setting in ['prm.n=-1','prm.k=0','prm.N=10','search.epsilon=-.1','metrics=[]']:
        with pytest.raises(ValueError):load(p,[setting])
    assert main(['run',str(p),'--output',str(tmp_path/'bad'),'--set','prm.n=-1'])==2


def test_interrupted_edge_build_resumes_and_matches(demo,tmp_path,monkeypatch):
    import prmkit.roadmap as roadmap
    demo['prm'].update(n=24,k=4,edge_chunk=8)
    original=roadmap._chunk
    calls=[0]
    def interrupted(*args):
        calls[0]+=1
        if calls[0]==2:raise RuntimeError('injected worker interruption')
        return original(*args)
    monkeypatch.setattr(roadmap,'_chunk',interrupted)
    with pytest.raises(RuntimeError,match='injected'):run(demo,tmp_path/'interrupted',quiet=True)
    assert list((tmp_path/'interrupted/edge_checkpoints').glob('*.npz'))
    monkeypatch.setattr(roadmap,'_chunk',original)
    resumed=run(demo,tmp_path/'interrupted',resume=True,quiet=True)
    fresh=run(demo,tmp_path/'fresh',quiet=True)
    assert resumed['costs']==fresh['costs']
    with np.load(tmp_path/'interrupted/graph.npz') as a,np.load(tmp_path/'fresh/graph.npz') as b:
        for key in a.files:np.testing.assert_array_equal(a[key],b[key])


def test_offline_report_javascript_runs(demo,tmp_path):
    quickjs=pytest.importorskip('quickjs')
    run(demo,tmp_path/'report',quiet=True)
    source=(tmp_path/'report/report.html').read_text().split('<script>',1)[1].split('</script>',1)[0]
    ctx=quickjs.Context()
    ctx.eval('''const elements={}; const mockCanvas=new Proxy({},{get:()=>()=>{}});
    globalThis.document={getElementById:(id)=>elements[id]||(elements[id]={value:0,width:520,height:380,append:()=>{},getContext:()=>mockCanvas}),createElement:()=>({})};
    document.getElementById('projection').value='XY';''')
    ctx.eval(source)
    assert 'Completed' in ctx.eval("document.getElementById('status').textContent")


def test_custom_robot_factory_and_ik(demo,tmp_path):
    factory=tmp_path/'robot.py'
    factory.write_text('from prmkit.backends.point import PointBackend\ndef make(robot, environment):\n    return PointBackend(robot, environment)\n')
    demo['robot'].update(backend='plugin',plugin=str(factory)+':make',lower=[0,0,0],upper=[1,1,1],weights=[1,1,1])
    demo['environment']['objects']=[]
    demo['query']={'start':[.1,.1,.1],'target':[.8,.7,.6],'ik_attempts':2,'max_goals':1,'ik_tolerance':1e-5}
    demo['prm'].update(n=4,k=4,query_k=6)
    r=run(demo,tmp_path/'custom',quiet=True)
    assert r['complete'] and r['paths']>0
    with np.load(tmp_path/'custom/paths.npz') as z:np.testing.assert_allclose(z['ee'][-1],[.8,.7,.6],atol=1e-5)


def test_torn_checkpoint_publication_is_not_reused(tmp_path,monkeypatch):
    import prmkit.roadmap as r
    p=tmp_path/'checkpoint.npz'
    r.checkpoint(p,q=np.array([1]))
    original=r.atomic_json
    def fail(*args,**kwargs):raise RuntimeError('lost power before commit')
    monkeypatch.setattr(r,'atomic_json',fail)
    with pytest.raises(RuntimeError):r.checkpoint(p,q=np.array([2]))
    assert not r.checkpoint_ready(p)
    monkeypatch.setattr(r,'atomic_json',original)
    r.checkpoint(p,q=np.array([3]))
    assert r.checkpoint_ready(p)
