"""Deterministic valid-state sampling, kNN connections and checkpointed edges."""
from __future__ import annotations

from concurrent.futures import ProcessPoolExecutor
import multiprocessing as mp
import json
from pathlib import Path
import warnings

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.stats import qmc

from .backends import make_backend
from .geometry import valid_q, minimum, sample_edge, certify
from .io import atomic_npz, atomic_json, digest
from .metrics import Metrics


class InfeasibleQuery(ValueError):
    pass


def checkpoint(path, **arrays):
    # Invalidate the old commit before replacing its payload. A crash between
    # data publication and hash publication then leaves an uncommitted file.
    Path(str(path) + ".sha256.json").unlink(missing_ok=True)
    atomic_npz(path, **arrays)
    atomic_json(Path(str(path) + ".sha256.json"), {"sha256": digest(path)})


def checkpoint_ready(path):
    """An interrupted two-file publication is recomputed, never trusted."""
    marker = Path(str(path) + ".sha256.json")
    if not path.exists() or not marker.exists():
        return False
    if digest(path) != json.loads(marker.read_text())["sha256"]:
        raise ValueError(f"Checkpoint integrity failure: {path}")
    return True


def prepare_query(config, backend):
    query = config["query"]
    threshold = config["validation"]["margin"] + config["validation"]["guard"]
    start = np.asarray(query.get("start", []), float)
    if not valid_q(backend, start):
        raise ValueError("query.start must match robot dimension and joint limits")
    if minimum(backend.state(start).distances) <= threshold:
        raise InfeasibleQuery("Start violates the hard clearance margin")
    if "goals" in query and "target" in query:
        raise ValueError("Choose query.goals or query.target, not both")
    if "target" in query:
        target = np.asarray(query["target"], float)
        if target.shape != (3,) or not np.isfinite(target).all():
            raise ValueError("IK target must be finite xyz in metres")
        goals = []
        rng = np.random.default_rng(config["seed"])
        for i in range(query.get("ik_attempts", 100)):
            x0 = start if i == 0 else rng.uniform(backend.lower, backend.upper)
            result = least_squares(lambda q: backend.state(q).ee-target, x0,
                                   bounds=(backend.lower, backend.upper), max_nfev=200)
            q = result.x
            if np.linalg.norm(backend.state(q).ee-target) <= query.get("ik_tolerance", 0.001) and minimum(backend.state(q).distances) > threshold:
                if not any(np.linalg.norm(q-g) < 1e-5 for g in goals):
                    goals.append(q)
            if len(goals) >= query.get("max_goals", 16):
                break
    else:
        goals = np.asarray(query.get("goals", []), float)
        if goals.ndim != 2 or goals.shape[1:] != start.shape or not len(goals):
            raise ValueError("query.goals must be a nonempty list of joint configurations")
        if not all(valid_q(backend, q) for q in goals):
            raise ValueError("Goal shape, values or limits do not match robot")
    accepted = []
    for q in goals:
        if minimum(backend.state(q).distances) > threshold and not any(np.array_equal(q, g) for g in accepted):
            accepted.append(q)
    if not accepted:
        raise InfeasibleQuery("No collision-free goal; IK failure is not proof of task infeasibility")
    return np.vstack((start, accepted)), dict(supplied_goals=len(goals), accepted_goals=len(accepted))


def sample_nodes(config, backend, fixed, directory, progress):
    p = config["prm"]
    path = directory / "nodes_checkpoint.npz"
    accepted, drawn = [], 0
    if checkpoint_ready(path):
        with np.load(path, allow_pickle=False) as z:
            accepted = list(z["accepted"])
            drawn = int(z["drawn"])
    if len(accepted) >= p["n"]:
        return np.vstack((fixed, np.asarray(accepted[:p["n"]]).reshape(-1, len(backend.lower))))
    if p["sampler"] == "sobol":
        sampler = qmc.Sobol(len(backend.lower), scramble=True, seed=config["seed"])
        if drawn:
            sampler.fast_forward(drawn)
        def draw(n):
            with warnings.catch_warnings():
                warnings.simplefilter("ignore", UserWarning)
                return sampler.random(n)
    else:
        # Replay RNG sequence in bounded chunks before resuming.
        rng = np.random.default_rng(config["seed"])
        for offset in range(0, drawn, 4096):
            rng.random((min(4096, drawn-offset), len(backend.lower)))
        def draw(n):
            return rng.random((n, len(backend.lower)))
    threshold = config["validation"]["margin"] + config["validation"]["guard"]
    while len(accepted) < p["n"] and drawn < p["max_draws"]:
        batch = min(256, p["max_draws"] - drawn)
        points = qmc.scale(draw(batch), backend.lower, backend.upper)
        for q in points:
            if minimum(backend.state(q).distances) > threshold:
                accepted.append(q)
        drawn += batch
        accepted = accepted[:p["n"]]
        checkpoint(path, accepted=np.asarray(accepted).reshape(-1, len(backend.lower)), drawn=np.array(drawn))
        progress(dict(stage="sampling", accepted=len(accepted), requested=p["n"], drawn=drawn))
    if len(accepted) != p["n"]:
        raise RuntimeError(f"Sampling exhausted prm.max_draws={p['max_draws']}: accepted {len(accepted)}/{p['n']}")
    return np.vstack((fixed, np.asarray(accepted).reshape(-1, len(backend.lower))))


def candidates(nodes, weights, fixed_count, k, query_k):
    if len(nodes) >= 2**31:
        raise ValueError("This graph format supports fewer than 2^31 vertices")
    scaled = nodes * weights
    tree = cKDTree(scaled)
    parts = []
    for start in range(0, len(nodes), 4096):
        end = min(start+4096, len(nodes))
        count = min(max(k, query_k)+1, len(nodes))
        nb = np.asarray(tree.query(scaled[start:end], k=count)[1]).reshape(end-start, count)
        batch = []
        for u, neighbors in zip(range(start, end), nb):
            limit = query_k if u < fixed_count else k
            for v in [int(v) for v in neighbors if v != u][:limit]:
                batch.append((min(u, v) << 32) | max(u, v))
        parts.append(np.unique(np.asarray(batch, dtype=np.int64)))
    keys = np.unique(np.concatenate(parts))
    return np.column_stack((keys >> 32, keys & ((1 << 32)-1)))


def evaluate_pair(backend, metrics, config, a, b):
    samples = sample_edge(backend, a, b, config["validation"]["intervals"], config.get("camera", {}))
    lower = certify(backend, samples, config["validation"])
    if lower is None:
        return None
    forward = metrics.evaluate(samples)
    reverse = forward if metrics.symmetric else metrics.evaluate(sample_edge(backend, b, a, config["validation"]["intervals"], config.get("camera", {})))
    return forward, reverse, lower


_WORKER = None


def _init_worker(config):
    global _WORKER
    _WORKER = make_backend(config), Metrics(config), config


def _evaluate_chunk(task):
    backend, metrics, config = _WORKER
    pairs, q0, q1 = task
    return _chunk(backend, metrics, config, pairs, q0, q1)


def _chunk(backend, metrics, config, pairs, q0, q1):
    edges, costs, bounds = [], [], []
    for pair, a, b in zip(pairs, q0, q1):
        result = evaluate_pair(backend, metrics, config, a, b)
        if result is not None:
            forward, reverse, bound = result
            edges.extend((pair, pair[::-1]))
            costs.extend((forward, reverse))
            bounds.extend((bound, bound))
    return dict(edges=np.array(edges, dtype=np.int64).reshape(-1, 2),
                costs=np.array(costs).reshape(-1, len(config["metrics"])), bounds=np.array(bounds))


def build(config, backend, metrics, directory, progress):
    directory = Path(directory)
    if checkpoint_ready(directory / "graph.npz"):
        with np.load(directory / "graph.npz", allow_pickle=False) as z:
            return {k: z[k] for k in z.files}
    fixed, query_info = prepare_query(config, backend)
    nodes = sample_nodes(config, backend, fixed, directory, progress)
    pairs = candidates(nodes, backend.weights, len(fixed), config["prm"]["k"], config["prm"]["query_k"])
    atomic_json(directory / "query.json", dict(**query_info, start=nodes[0], goals=nodes[1:len(fixed)], joint_names=backend.joint_names))
    shard_dir = directory / "edge_checkpoints"
    shard_dir.mkdir(exist_ok=True)
    chunk_size = config["prm"]["edge_chunk"]
    starts = list(range(0, len(pairs), chunk_size))
    missing = [s for s in starts if not checkpoint_ready(shard_dir / f"{s:012d}.npz")]
    workers = config["prm"]["workers"]
    pool = ProcessPoolExecutor(workers, initializer=_init_worker, initargs=(config,), mp_context=mp.get_context("spawn")) if workers > 1 else None
    try:
        for offset in range(0, len(missing), max(1, workers*2)):
            batch = missing[offset:offset+max(1, workers*2)]
            tasks = [(pairs[s:s+chunk_size], nodes[pairs[s:s+chunk_size, 0]], nodes[pairs[s:s+chunk_size, 1]]) for s in batch]
            results = pool.map(_evaluate_chunk, tasks) if pool else (_chunk(backend, metrics, config, *t) for t in tasks)
            for s, result in zip(batch, results):
                checkpoint(shard_dir / f"{s:012d}.npz", **result)
                progress(dict(stage="edges", checked_chunks=len(starts)-len(missing)+offset+batch.index(s)+1,
                              total_chunks=len(starts), candidate_edges=len(pairs)))
    finally:
        if pool:
            pool.shutdown(wait=True, cancel_futures=True)
    parts = []
    for s in starts:
        with np.load(shard_dir / f"{s:012d}.npz", allow_pickle=False) as z:
            parts.append({k: z[k] for k in z.files})
    graph = dict(nodes=nodes, goals=np.arange(1, len(fixed), dtype=np.int64))
    for key, shape in (("edges", (0, 2)), ("costs", (0, len(config["metrics"]))), ("bounds", (0,))):
        graph[key] = np.concatenate([p[key] for p in parts]) if parts else np.empty(shape, dtype=np.int64 if key == "edges" else float)
    checkpoint(directory / "graph.npz", **graph)
    return graph
