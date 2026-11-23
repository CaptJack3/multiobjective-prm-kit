import json
from pathlib import Path
import numpy as np
import pytest
from prmkit.search import solve


def oracle(n, edges, costs, start=0, goal=None):
    goal = n-1 if goal is None else goal
    values = []
    def visit(u, seen, cost):
        if u == goal:
            values.append(cost)
            return
        for i, (a,b) in enumerate(edges):
            if a == u and b not in seen:
                visit(b, seen | {int(b)}, cost + costs[i])
    visit(start, {start}, np.zeros(costs.shape[1]))
    return np.asarray([c for c in values if not any(np.all(x<=c) and np.any(x<c) for x in values)]).reshape(-1,costs.shape[1])


@pytest.mark.parametrize('dimensions',[1,2,3,4])
def test_exact_and_epsilon_against_exhaustive_paths(dimensions):
    for seed in range(30):
        rng=np.random.default_rng(seed)
        edges=np.array([(i,j) for i in range(6) for j in range(6) if i!=j and (rng.random()<.24 or j==i+1)])
        costs=rng.integers(0,8,(len(edges),dimensions)).astype(float)
        expected=oracle(6,edges,costs)
        for epsilon in [0,.05,.2,.7]:
            result=solve(6,edges,costs,0,[5],epsilon=epsilon,engine='python',max_labels=100000)
            assert result['complete']
            actual=np.asarray(result['costs'])
            assert all(np.any(np.all(actual<=(1+epsilon)*c+1e-9,axis=1)) for c in expected)
            if epsilon==0:
                assert {tuple(c) for c in actual}=={tuple(c) for c in expected}
            for path,c in zip(result['paths'],actual):
                np.testing.assert_allclose(costs[path['edges']].sum(axis=0),c)
                for i,eid in enumerate(path['edges']):
                    assert list(edges[eid])==path['vertices'][i:i+2]


@pytest.mark.parametrize('fixture',['terminal_obligation','real_ancestry_reopening'])
@pytest.mark.parametrize('engine',['python','research'])
def test_historical_counterexamples(fixture,engine):
    if engine=='research': pytest.importorskip('numba')
    f=json.loads((Path(__file__).parent/'fixtures'/f'{fixture}.json').read_text())
    edges=np.array([a[:2] for a in f['arcs']]);costs=np.array([a[2] for a in f['arcs']])
    exact=solve(f['n'],edges,costs,0,[f['n']-1],epsilon=0,engine=engine)
    approx=solve(f['n'],edges,costs,0,[f['n']-1],epsilon=f['epsilon'],engine=engine)
    assert exact['complete'] and approx['complete']
    expected=f.get('exact',exact['costs'])
    assert all(any(np.allclose(c,e) for c in exact['costs']) for e in expected)
    assert all(any(np.all(np.array(c)<=(1+f['epsilon'])*np.array(e)+1e-9) for c in approx['costs']) for e in expected)


def test_parallel_edges_zero_cycles_unreachable_and_limits():
    edges=np.array([[0,1],[0,1],[1,0],[1,2]])
    costs=np.array([[1,8],[8,1],[0,0],[0,0]],float)
    r=solve(3,edges,costs,0,[2],epsilon=0,engine='python')
    assert sorted(r['costs'])==[[1,8],[8,1]]
    r=solve(4,edges,costs,0,[3],engine='python')
    assert r['complete'] and not r['reachable'] and not r['paths']
    r=solve(3,edges,costs,0,[2],max_labels=2,engine='python')
    assert not r['complete'] and r['termination']=='label_limit'
    with pytest.raises(ValueError): solve(3,edges,-costs,0,[2])


def test_exact_preserves_equal_cost_simple_paths():
    edges=np.array([[0,1],[0,2],[1,3],[2,3]])
    costs=np.ones((4,2))
    r=solve(4,edges,costs,0,[3],epsilon=0,engine='python')
    assert len(r['paths'])==2 and r['costs']==[[2,2],[2,2]]
