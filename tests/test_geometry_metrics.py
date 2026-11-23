import numpy as np
import pytest
from prmkit.backends import make_backend
from prmkit.geometry import sample_edge,certify
from prmkit.metrics import Metrics


def test_thin_wall_is_rejected_between_free_endpoints(demo):
    demo['environment']['objects'][0].update(position_m=[.5,.5],dimensions_m=[.0001,.8])
    b=make_backend(demo)
    s=sample_edge(b,np.array([.1,.5]),np.array([.9,.5]),1,{})
    assert np.all(s.distances>demo['validation']['margin'])
    assert certify(b,s,demo['validation']) is None


def test_clearance_threshold_independent_of_validity(demo):
    demo['environment']['objects'][0].update(position_m=[.5,.5],dimensions_m=[.2,.2])
    b=make_backend(demo)
    s=sample_edge(b,np.array([.25,.3]),np.array([.25,.7]),32,{})
    bound=certify(b,s,demo['validation'])
    assert bound>demo['validation']['margin']
    metrics=Metrics(demo)
    costs=metrics.evaluate(s)
    np.testing.assert_allclose(costs[0],.4)
    assert costs[1]>0
    demo['metrics'][1]['threshold']=.05
    assert Metrics(demo).evaluate(s)[1]==0


def test_negative_custom_metric_rejected(demo,tmp_path):
    f=tmp_path/'metric.py';f.write_text('def cost(samples, parameters):\n    return -1\n')
    demo['metrics']=[dict(name='bad',plugin=str(f)+':cost')]
    b=make_backend(demo)
    with pytest.raises(ValueError,match='finite and nonnegative'):
        Metrics(demo).evaluate(sample_edge(b,np.array([.1,.1]),np.array([.2,.1]),2,{}))


def test_empty_world_has_no_risk(demo):
    demo['environment']['objects']=[]
    b=make_backend(demo);s=sample_edge(b,np.array([.1,.1]),np.array([.9,.9]),2,{})
    assert np.isinf(certify(b,s,demo['validation']))
    assert Metrics(demo).evaluate(s)[1]==0
