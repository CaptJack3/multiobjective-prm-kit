import numpy as np
import pytest
from prmkit.backends import make_backend
from prmkit.geometry import sample_edge,certify
pytest.importorskip('pinocchio')
trimesh=pytest.importorskip('trimesh')

@pytest.mark.parametrize('shape',['box','sphere','cylinder','capsule','mesh'])
def test_world_shape_and_rotation_reject_crossing(shape,tmp_path):
    urdf=tmp_path/'robot.urdf'
    urdf.write_text('''<robot name="slide"><link name="base"/><link name="tool"><collision><geometry><sphere radius="0.01"/></geometry></collision></link><joint name="x" type="prismatic"><parent link="base"/><child link="tool"/><axis xyz="1 0 0"/><limit lower="0" upper="1" effort="1" velocity="1"/></joint></robot>''')
    obj=dict(name='obstacle',shape=shape,position_m=[.5,0,0],rotation_rpy=[0,0,.3],dimensions_m=[.05,.2,.2],radius_m=.05,length_m=.2)
    if shape=='mesh':
        mesh=tmp_path/'box.stl';trimesh.creation.box([.05,.2,.2]).export(mesh);obj['mesh']=str(mesh)
    b=make_backend(dict(robot=dict(backend='urdf',urdf=str(urdf),tool_frame='tool'),environment=dict(objects=[obj])))
    s=sample_edge(b,np.array([.1]),np.array([.9]),1,{})
    assert np.all(s.distances>0.1)
    assert certify(b,s,dict(margin=.01,guard=2e-6,max_depth=18)) is None
