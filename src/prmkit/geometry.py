"""Endpoint-inclusive integration and conservative whole-edge validation."""
from dataclasses import dataclass
import numpy as np


def valid_q(backend, q):
    q = np.asarray(q, float)
    return (q.shape == backend.lower.shape and np.isfinite(q).all()
            and np.all(q >= backend.lower) and np.all(q <= backend.upper))


def minimum(values):
    return float(np.min(values, initial=np.inf))


@dataclass
class EdgeSamples:
    q: np.ndarray
    t: np.ndarray
    ee: np.ndarray
    distances: np.ndarray
    world_mask: np.ndarray
    weights: np.ndarray
    backend: object
    camera: dict

    @property
    def joint_length(self):
        return float(np.linalg.norm((self.q[-1] - self.q[0]) * self.weights))


def sample_edge(backend, a, b, intervals, camera):
    t = np.linspace(0, 1, intervals + 1)
    q = a + t[:, None] * (b - a)
    states = [backend.state(x) for x in q]
    for state in states:
        if np.asarray(state.ee).shape != (3,) or not np.isfinite(state.ee).all():
            raise ValueError("Backend state.ee must be finite xyz")
        if np.asarray(state.distances).shape != backend.world_mask.shape or np.isnan(state.distances).any():
            raise ValueError("Backend distances must match world_mask and contain no NaN")
    return EdgeSamples(q, t, np.array([s.ee for s in states]),
                       np.array([s.distances for s in states]), backend.world_mask,
                       backend.weights, backend, camera)


def certify(backend, samples, settings):
    """Return continuous clearance lower bound; None means rejected/unproved.

    For any t in [l,r], the nearest endpoint is at most half an interval away.
    Pair distance changes by at most motion_bound(dq) * |dt|. This bound is
    independent of metric quadrature; unresolved intervals are never accepted.
    """
    ds = samples.distances
    guard, margin = settings["guard"], settings["margin"]
    if np.isnan(ds).any() or minimum(ds) <= margin + guard:
        return None
    motion = np.asarray(backend.motion_bound(samples.q[-1] - samples.q[0]), float)
    if motion.shape != (ds.shape[1],) or not np.isfinite(motion).all() or np.any(motion < 0):
        raise ValueError("Backend motion_bound must return finite nonnegative per-pair bounds")
    stack = [(samples.t[i], samples.t[i+1], ds[i], ds[i+1], 0) for i in range(len(ds)-1)]
    lower = np.inf
    while stack:
        l, r, dl, dr, depth = stack.pop()
        bound = minimum(np.minimum(dl, dr) - motion * (r - l) / 2 - guard)
        if bound > margin:
            lower = min(lower, bound)
            continue
        if depth >= settings["max_depth"]:
            return None
        mid = (l + r) / 2
        dm = backend.state(samples.q[0] + mid * (samples.q[-1] - samples.q[0])).distances
        if np.isnan(dm).any() or minimum(dm) <= margin + guard:
            return None
        stack.extend([(l, mid, dl, dm, depth+1), (mid, r, dm, dr, depth+1)])
    return float(lower)

