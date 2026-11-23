"""Exact and A*pex with immutable path parents and terminal obligations.

Three objectives can use the unmodified research Numba kernel. The Python
engine is a dimension-independent port of that corrected kernel. Exact mode
retains distinct equal-cost simple paths, subject to the declared label budget.
"""
from __future__ import annotations

import heapq
import importlib.util
import time

import numpy as np
from scipy.sparse import csr_matrix
from scipy.sparse.csgraph import dijkstra


def pack(n, edges, costs, goals):
    d = costs.shape[1]
    goal = n
    arcs = np.vstack((edges, np.column_stack((goals, np.full(len(goals), goal)))))
    ec = np.vstack((costs, np.zeros((len(goals), d))))
    order = np.lexsort((arcs[:, 1], arcs[:, 0]))
    row, col = arcs[order].T
    indptr = np.r_[0, np.cumsum(np.bincount(row, minlength=n+1))].astype(np.int64)
    h = np.empty((n+1, d))
    # Parallel edges need a minimum, not sparse-matrix duplicate summation.
    keys, inv = np.unique(arcs[:, 1] * (n+1) + arcs[:, 0], return_inverse=True)
    for k in range(d):
        data = np.full(len(keys), np.inf)
        np.minimum.at(data, inv, ec[:, k])
        rev = csr_matrix((data, (keys // (n+1), keys % (n+1))), shape=(n+1, n+1))
        h[:, k] = dijkstra(rev, directed=True, indices=goal)
    return indptr, col.astype(np.int32), order.astype(np.int32), ec, h, goal


def _python(ip, nb, ei, ec, h, goal, start, eps, timeout, cap):
    began = time.monotonic()
    # g, node, parent, and edge never change after publication.
    nodes, parents, edges = [start], [-1], [-1]
    gs, apexes, status = [np.zeros(ec.shape[1])], [np.zeros(ec.shape[1])], [1]
    at = [[] for _ in range(goal+1)]
    at[start].append(0)
    heap = [(tuple(h[start]), 0)]
    solutions = []
    counters = dict(expanded=0, merges=0, generated=1)
    reason = "complete"

    def weak(a, b):
        return bool(np.all(a <= b))

    def strict(a, b):
        return weak(a, b) and bool(np.any(a < b))

    def covers(a, b):
        return bool(np.all(a <= (1+eps)*b + 1e-12))

    while heap:
        if time.monotonic() - began > timeout:
            reason = "timeout"
            break
        _, lid = heapq.heappop(heap)
        if status[lid] != 1:
            continue
        u = nodes[lid]
        if len(nodes) + (ip[u+1] - ip[u]) + 1 >= cap:
            reason = "label_limit"
            break
        status[lid] = 2
        counters["expanded"] += 1
        if u == goal:
            dominated = False
            for sol in solutions:
                if status[sol] == 3 and strict(gs[sol], gs[lid]):
                    if eps > 0:
                        apexes[sol] = np.minimum(apexes[sol], apexes[lid])
                    dominated = True
                    break
            if not dominated:
                for sol in solutions:
                    if status[sol] == 3 and strict(gs[lid], gs[sol]):
                        if eps > 0:
                            apexes[lid] = np.minimum(apexes[lid], apexes[sol])
                        status[sol] = 4
                status[lid] = 3
                solutions.append(lid)
            continue
        ancestors = set()
        if eps == 0:
            anc = lid
            while anc >= 0:
                ancestors.add(nodes[anc])
                anc = parents[anc]
        for ai in range(ip[u], ip[u+1]):
            v, eid = int(nb[ai]), int(ei[ai])
            if v in ancestors or not np.isfinite(h[v]).all():
                continue
            cg, ca, cp, ce = gs[lid] + ec[eid], apexes[lid] + ec[eid], lid, eid
            prune = False
            for sol in solutions:
                if status[sol] == 3 and (covers(gs[sol], ca+h[v]) if eps > 0 else strict(gs[sol], cg+h[v])):
                    if eps > 0:
                        apexes[sol] = np.minimum(apexes[sol], ca+h[v])
                    prune = True
                    break
            if prune:
                continue
            # Newest first, matching the research kernel's linked list.
            for other in reversed(at[v]):
                if status[other] not in (1, 2, 3):
                    continue
                if eps == 0:
                    if strict(gs[other], cg):
                        prune = True
                        break
                else:
                    if weak(apexes[other], ca) and weak(gs[other], cg):
                        prune = True
                        break
                    merged = np.minimum(ca, apexes[other])
                    c_ok, o_ok = covers(cg+h[v], merged+h[v]), covers(gs[other]+h[v], merged+h[v])
                    if c_ok or o_ok:
                        if o_ok and (not c_ok or gs[other].sum() < cg.sum()):
                            if weak(apexes[other], merged):
                                prune = True
                                break
                            cg, cp, ce = gs[other].copy(), parents[other], edges[other]
                        ca, status[other] = merged, 4
                        counters["merges"] += 1
            if prune:
                continue
            for other in at[v]:
                if status[other] in (1, 2) and strict(cg, gs[other]) and (eps == 0 or weak(ca, apexes[other])):
                    status[other] = 4
            new = len(nodes)
            nodes.append(v); parents.append(cp); edges.append(ce)
            gs.append(cg); apexes.append(ca); status.append(1)
            at[v].append(new)
            heapq.heappush(heap, (tuple(ca+h[v]), new))
    keep = [lid for lid in solutions if status[lid] == 3]
    counters["generated"] = len(nodes)
    return keep, nodes, parents, edges, gs, reason, counters


def _research(ip, nb, ei, ec, h, goal, start, eps, timeout, cap):
    from numba import types
    from numba.typed import List
    from ._research_kernel import advance
    node = np.empty(cap, np.int32); parent = np.empty(cap, np.int64); edge = np.empty(cap, np.int32)
    g = np.zeros((cap, 3)); apex = np.zeros_like(g)
    status = np.zeros(cap, np.uint8); nxt = np.full(cap, -1, np.int64); head = np.full(goal+1, -1, np.int64)
    node[0] = start; parent[0] = edge[0] = -1; status[0] = 1; head[start] = 0
    heap = List.empty_list(types.Tuple((types.float64, types.float64, types.float64, types.int64)))
    heap.append((float(h[start, 0]), float(h[start, 1]), float(h[start, 2]), 0))
    solutions = List.empty_list(types.int64)
    counters = np.zeros(7, np.int64)
    count, reason = 1, "complete"
    began = time.monotonic()
    while len(heap):
        if time.monotonic() - began > timeout:
            reason = "timeout"
            break
        count, limited = advance(ip, nb, ei, ec, h, goal, eps, heap, solutions, head,
                                 node, parent, edge, g, apex, nxt, status, count, counters, 100)
        if limited:
            reason = "label_limit"
            break
    return [int(i) for i in solutions if status[i] == 3], node, parent, edge, g, reason, dict(generated=count, expanded=int(counters[1]), merges=int(counters[5]))


def solve(n, edges, costs, start, goals, epsilon=0.1, timeout_s=120, max_labels=200000, engine="auto"):
    edges, costs = np.asarray(edges, dtype=np.int64), np.asarray(costs, dtype=float)
    goals = np.unique(np.asarray(goals, dtype=np.int64))
    if edges.shape != (len(costs), 2) or costs.ndim != 2 or costs.shape[1] < 1:
        raise ValueError("Expected directed edges [E,2] and costs [E,objectives]")
    if not np.isfinite(costs).all() or np.any(costs < 0) or not np.isfinite(epsilon) or epsilon < 0:
        raise ValueError("Costs and epsilon must be finite and nonnegative")
    if len(goals) == 0 or not 0 <= start < n or np.any(edges < 0) or np.any(edges >= n) or np.any(goals < 0) or np.any(goals >= n):
        raise ValueError("Invalid graph endpoints")
    began = time.monotonic()
    ip, nb, ei, ec, h, goal = pack(n, edges, costs, goals)
    if engine == "auto":
        engine = "research" if costs.shape[1] == 3 and importlib.util.find_spec("numba") else "python"
    if engine == "research" and costs.shape[1] != 3:
        raise ValueError("research engine needs 3 metrics; use engine=python for any dimension")
    if not np.isfinite(h[start]).all():
        return dict(paths=[], costs=[], termination="complete", complete=True, reachable=False,
                    algorithm="Exact" if epsilon == 0 else "A*pex", engine=engine, epsilon=epsilon,
                    runtime_s=time.monotonic()-began, counters={})
    worker = _research if engine == "research" else _python
    keep, nodes, parents, edge_ids, g, reason, counters = worker(ip, nb, ei, ec, h, goal, start, float(epsilon), timeout_s, max_labels)
    paths, vectors, seen = [], [], set()
    for lid in sorted(keep, key=lambda x: (tuple(g[x]), x)):
        if any(np.all(g[k] <= g[lid]) and np.any(g[k] < g[lid]) for k in keep):
            continue
        vs, es, current = [], [], lid
        while current >= 0:
            vs.append(int(nodes[current]))
            if edge_ids[current] >= 0:
                es.append(int(edge_ids[current]))
            current = int(parents[current])
        vs, es = vs[::-1], es[::-1]
        if vs[0] != start or vs[-1] != goal or vs[-2] not in goals:
            raise RuntimeError("Invalid parent chain endpoints")
        if not np.allclose(ec[es].sum(axis=0), g[lid], rtol=1e-10, atol=1e-11):
            raise RuntimeError("Reconstructed path cost mismatch")
        signature = (tuple(vs), tuple(es))
        if signature in seen:
            continue
        seen.add(signature)
        paths.append(dict(vertices=vs[:-1], edges=es[:-1]))
        vectors.append(np.asarray(g[lid]).tolist())
    return dict(paths=paths, costs=vectors, termination=reason, complete=reason == "complete",
                reachable=True, algorithm="Exact" if epsilon == 0 else "A*pex", engine=engine,
                epsilon=epsilon, runtime_s=time.monotonic()-began, counters=counters)
