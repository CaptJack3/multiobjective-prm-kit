from copy import deepcopy
import json
from pathlib import Path
import subprocess

import numpy as np
import pytest
import yaml

from prmkit.artifacts import candidate_catalog, read_selection, content_hash
from prmkit.backends import make_backend
from prmkit.cases import run_case, _base_experiment, _wall_environments
from prmkit.cli import main
from prmkit.config import load_data
from prmkit.delivery import export_paths, render_paths
from prmkit.experiment import run_experiment, rerun
from prmkit.explorer import explore
from prmkit.io import digest
from prmkit.pipeline import run
from prmkit.selection import representative, select_paths, nondominated
from prmkit.verification import options, audit_path, apply_constraints, verify_paths, attach_verification, make_verifier


@pytest.fixture
def free_run(demo, tmp_path):
    demo['environment']['objects']=[]
    demo['prm'].update(n=0,k=1,query_k=1)
    directory=tmp_path/'free_run'
    assert run(demo,directory,quiet=True)['paths']==1
    return directory


def test_audit_checks_between_legal_waypoints_and_clearance():
    cfg=dict(robot=dict(backend='point',lower=[0,0],upper=[1,1],radius=.01),
             environment=dict(objects=[dict(name='wall',position_m=[.5,.5],dimensions_m=[.2,.2])]))
    backend=make_backend(cfg)
    settings=options(dict(backend='planning',min_clearance_m=.02,interpolation=dict(max_translation_step_m=.01)))
    q=np.array([[.2,.5],[.8,.5]])
    assert all(backend.state(v).distances.min()>.02 for v in q)
    result=audit_path(q,backend,settings)
    assert result['status']=='collision_found' and result['colliding_segment_ids']==[0]
    near=np.array([[.45,.62],[.55,.62]])
    result=audit_path(near,backend,settings)
    assert result['status']=='clearance_violation'
    assert result['min_clearance_m_observed']==pytest.approx(.01)
    result=audit_path(np.array([[.45,.65],[.55,.65]]),backend,settings)
    assert result['status']=='discrete_verified'
    assert result['sample_count']>2


def test_coal_mesh_catches_interior_segment_collision(tmp_path):
    pytest.importorskip('coal');pytest.importorskip('pinocchio')
    urdf=tmp_path/'slide.urdf'
    urdf.write_text('''<robot name="slide"><link name="base"/><link name="tool"><collision><geometry><sphere radius="0.02"/></geometry></collision></link><joint name="slide" type="prismatic"><parent link="base"/><child link="tool"/><axis xyz="1 0 0"/><limit lower="0" upper="1" effort="10" velocity="1"/></joint></robot>''')
    cfg=dict(robot=dict(backend='urdf',urdf=str(urdf),tool_frame='tool'),environment=dict(objects=[dict(name='wall',position_m=[.5,0,0],dimensions_m=[.02,.2,.2])]))
    settings=options(dict(interpolation=dict(max_translation_step_m=.005)))
    backend=make_verifier(cfg,settings)
    assert all(backend.state(q).distances.min()>0 for q in [[.1],[.9]])
    result=audit_path(np.array([[.1],[.9]]),backend,settings)
    assert result['status']=='collision_found'
    assert result['failure_samples'][0]['pair']==['tool','world__wall']


def test_postcheck_does_not_change_planning_margin_and_enforcement_does(demo):
    old=deepcopy(demo)
    verification=apply_constraints(demo,dict(min_clearance_m=.02,clearance_mode='postcheck_only'),None)
    assert demo==old and verification['min_clearance_m']==.02
    apply_constraints(demo,dict(min_clearance_m=.03,clearance_mode='enforce_during_search'),None)
    assert demo['validation']['margin']==.03
    with pytest.raises(ValueError,match='requires verification'):
        apply_constraints(old,dict(min_clearance_m=.02),False)


def test_verify_preserves_raw_and_errors_are_not_passes(free_run,monkeypatch):
    catalog=candidate_catalog(free_run)
    before={p.name:digest(p) for p in free_run.iterdir() if p.is_file()}
    result=verify_paths(free_run,dict(backend='planning',min_clearance_m=.02))
    assert result['summary']['discrete_verified']==1
    row=result['records'][0]
    assert all(row[k] for k in ['path_hash','scene_geometry_hash','robot_geometry_hash','validation_config_hash','timestamp_utc'])
    assert {p.name:digest(p) for p in free_run.iterdir() if p.is_file()}==before
    import prmkit.verification as module
    monkeypatch.setattr(module,'make_verifier',lambda *_: (_ for _ in ()).throw(RuntimeError('backend failed')))
    failed=verify_paths(free_run,dict(backend='planning'))
    assert failed['summary']['validation_error']==1 and not failed['complete']
    assert not select_paths(free_run)['path_ids']


def test_catalog_selection_namespace_integrity_and_no_solver_on_export(free_run,monkeypatch,tmp_path):
    catalog=candidate_catalog(free_run)
    assert candidate_catalog(free_run)==catalog
    verify_paths(free_run,dict(backend='planning'))
    selected=select_paths(free_run)
    assert len(selected['path_ids'])==1
    invalid=dict(selected,catalog_hash='another')
    with pytest.raises(ValueError,match='catalog_hash'):read_selection(invalid,catalog)
    invalid=dict(selected,path_ids=['unknown'])
    with pytest.raises(ValueError,match='unknown'):read_selection(invalid,catalog)
    import prmkit.pipeline as pipeline
    monkeypatch.setattr(pipeline,'solve',lambda *_a,**_k:pytest.fail('Postprocessing invoked solver'))
    result=export_paths(free_run,selected,only_verified=True)
    assert result['paths']==1
    out=Path(result['output'])
    with np.load(out/(selected['path_ids'][0]+'.npz'),allow_pickle=False) as data:
        assert data['q'].shape==(2,2) and data['joint_names'].tolist()==['position_0','position_1']
    assert export_paths(free_run,selected,only_verified=True)==result
    assert Path(explore(free_run)['file']).is_file()


def test_partial_verification_cannot_select_unverified(free_run):
    catalog=candidate_catalog(free_run)
    selection=dict(catalog_hash=catalog['catalog_hash'],path_ids=[])
    result=verify_paths(free_run,dict(backend='planning',scope='selected'),selection=selection)
    assert result['summary']['unverified']==1 and result['checked']==0
    assert not select_paths(free_run)['path_ids']
    assert select_paths(free_run,dict(only_verified=False))['path_ids']


def test_representatives_deterministic_verified_front():
    costs=[[0,4,4],[4,0,4],[4,4,0],[2,2,2],[3,3,3],[1,3,3]]
    rows=[dict(path_id=f'p{i}',run='r',scene_id='s',query_id='q',costs=c,metrics=['a','b','c'],verification={'status':'discrete_verified' if i!=0 else 'unverified'}) for i,c in enumerate(costs)]
    catalog=dict(catalog_hash='abc',paths=rows)
    result=representative(catalog)
    assert 'p0' not in result['path_ids'] and 'p4' not in result['path_ids']
    assert {'p1','p2','p3'}<=set(result['path_ids'])
    assert representative(dict(catalog,paths=list(reversed(rows))))==result


def test_saved_graph_reuse_and_margin_invalidation(free_run):
    from prmkit.artifacts import run_inputs
    config=deepcopy(run_inputs(free_run)['config'])
    config['search']['epsilon']=.1
    output=free_run.parent/'solved_again'
    result=run(config,output,roadmap_source=free_run,quiet=True)
    assert result['paths']==1
    assert (free_run/'graph.npz').stat().st_ino==(output/'graph.npz').stat().st_ino
    config['validation']['margin']+=.02
    with pytest.raises(ValueError,match='identical graph inputs'):
        run(config,free_run.parent/'wrong_margin',roadmap_source=free_run,quiet=True)


def test_sobol_generation_is_repeatable_and_does_not_load_robot(tmp_path,monkeypatch):
    import prmkit.cases as cases
    monkeypatch.setattr(cases,'make_backend',lambda *_:pytest.fail('Scene generation loaded robot'))
    opts=dict(profile='mounted_ur5e',wall_count=3,widths={'min':.3,'max':.9},max_environments=5,generate_only=True,quiet=True)
    one=run_case('wall_sobol',output=tmp_path/'one',**opts)
    two=run_case('wall_sobol',output=tmp_path/'two',**opts)
    for i in range(5):
        a=tmp_path/f'one/environments/env_{i:05d}/scene.json';b=tmp_path/f'two/environments/env_{i:05d}/scene.json'
        assert a.read_bytes()==b.read_bytes()
    assert one['environments']==5 and list((tmp_path/'one').rglob('preview.svg'))
    assert not list((tmp_path/'one').rglob('graph.npz'))


def test_unified_experiment_and_rerun_keep_inputs(demo,tmp_path):
    from prmkit.spaces import portable_config
    (tmp_path/'base.yaml').write_text(yaml.safe_dump(portable_config(demo)))
    config=dict(schema_version=2,experiment=dict(name='test',base_experiment='base.yaml'),
                scene=dict(source='saved',file=str(Path(demo['_file']).with_name('environment.json'))),
                queries=dict(mode='cartesian_product',starts=[demo['query']['start']],goals=demo['query']['goals']),
                roadmap=dict(n_vertices=24,k_neighbors=6),search=dict(epsilons=[0,.1]),
                verification=dict(backend='planning',enabled=True),selection=dict(only_verified=True),
                output=dict(directory='study'))
    path=tmp_path/'experiment.yaml';path.write_text(yaml.safe_dump(config))
    assert run_experiment(path,dry_run=True)['runs']==2 and not (tmp_path/'study').exists()
    result=run_experiment(path,quiet=True)
    assert result['complete'] and result['analysis']['verification']['summary']['discrete_verified']>0
    dense=rerun(tmp_path/'study',tmp_path/'denser',n=[30],k=[8],quiet=True)
    assert dense['complete']
    assert json.loads((tmp_path/'study/queries.json').read_text())['queries']==json.loads((tmp_path/'denser/queries.json').read_text())['queries']
    assert (tmp_path/'study/environments/env_00000/scene.json').read_bytes()==(tmp_path/'denser/environments/env_00000/scene.json').read_bytes()
    (tmp_path/'study/queries.json').write_text('{"queries": []}')
    with pytest.raises(ValueError,match='Saved study input was modified'):
        rerun(tmp_path/'study',tmp_path/'corrupted',n=[40],quiet=True)


def test_invalid_stage_options_fail_before_loading_robot(tmp_path,monkeypatch):
    import prmkit.cases as cases
    monkeypatch.setattr(cases,'make_backend',lambda *_:pytest.fail('Invalid settings reached robot setup'))
    with pytest.raises(ValueError,match='Native Isaac rendering is not implemented'):
        run_case(output=tmp_path/'bad',render={'backend':'isaac'})
    assert not (tmp_path/'bad').exists()


def test_mounted_preset_preserves_frozen_geometry_tcp_and_camera(resources):
    pytest.importorskip('pinocchio');pytest.importorskip('coal')
    provenance=json.loads((resources/'assets/mounted_ur5e/provenance.json').read_text())
    assert digest(resources/'assets/mounted_ur5e/assembly_meshes.npz')==provenance['source_sha256']
    config=load_data(_base_experiment(None,None,None,resources,'mounted_ur5e'))
    backend=make_backend(config)
    assert len(config['robot']['attachments'])==14
    q=np.asarray(config['query']['start']);state=backend.state(q)
    frame=backend.model.frames[backend.frame_id('tool0')]
    tf=backend.data.oMi[frame.parentJoint]*frame.placement
    np.testing.assert_allclose(state.ee,tf.translation+tf.rotation@np.array([0,0,.1659000001]),atol=1e-12)
    np.testing.assert_allclose(config['camera']['rotation'],np.array(provenance['T_flange_rgb_optical'])[:3,:3],atol=0)
    assert state.distances.min()>.005
    verifier=make_verifier(config,options())
    assert audit_path(np.array([q]),verifier,options())['status']=='discrete_verified'


def test_selected_cpu_video_can_be_decoded_without_solver(free_run,monkeypatch):
    ffmpeg=pytest.importorskip('imageio_ffmpeg')
    pytest.importorskip('matplotlib')
    import prmkit.pipeline as pipeline
    monkeypatch.setattr(pipeline,'solve',lambda *_a,**_k:pytest.fail('Rendering invoked solver'))
    selected=select_paths(free_run,dict(only_verified=False))
    result=render_paths(free_run,selected,frames=4,fps=2,width=320,height=240)
    movies=list(Path(result['output']).glob('*.mp4'))
    assert len(movies)==1
    subprocess.run([ffmpeg.get_ffmpeg_exe(),'-v','error','-i',str(movies[0]),'-f','null','-'],check=True,capture_output=True)
    assert json.loads(movies[0].with_suffix('.json').read_text())['verification']['status']=='unverified'
