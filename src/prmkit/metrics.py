"""Additive nonnegative edge objectives. Custom functions receive EdgeSamples."""
import numpy as np
from .geometry import sample_edge
from .io import plugin


def builtin(samples, metric):
    kind = metric.get("kind", metric["name"])
    if kind == "ee_length":
        return np.linalg.norm(np.diff(samples.ee, axis=0), axis=1).sum()
    if kind == "joint_length":
        return samples.joint_length
    if kind == "clearance":
        ds = samples.distances
        if metric.get("scope", "world") == "world":
            ds = ds[:, samples.world_mask]
        distance = np.min(ds, axis=1, initial=np.inf)
        penalty = np.maximum(0., 1 - distance / metric.get("threshold", 0.1)) ** metric.get("power", 2)
        return np.trapezoid(penalty, x=samples.t) * samples.joint_length
    if kind == "visibility":
        v = np.array([samples.backend.visibility(q, samples.camera) for q in samples.q])
        if not np.isfinite(v).all() or np.any((v < 0) | (v > 1)):
            raise ValueError("Backend visibility must be a fraction in [0, 1]")
        return np.trapezoid(1 - v, x=samples.t) * samples.joint_length
    raise ValueError(f"Unknown metric kind: {kind}")


class Metrics:
    def __init__(self, config):
        self.specs = config["metrics"]
        self.functions = []
        for m in self.specs:
            if "plugin" in m:
                self.functions.append(plugin(m["plugin"], config["_base"]))
            else:
                if m.get("kind", m["name"]) not in ("ee_length", "joint_length", "clearance", "visibility"):
                    raise ValueError(f"Unknown metric: {m['name']}")
                self.functions.append(None)
        self.symmetric = all(m.get("symmetric", "plugin" not in m) for m in self.specs)

    def evaluate(self, samples):
        cache = {len(samples.t) - 1: samples}
        result = []
        for m, fn in zip(self.specs, self.functions):
            intervals = m.get("intervals", len(samples.t) - 1)
            if intervals not in cache:
                cache[intervals] = sample_edge(samples.backend, samples.q[0], samples.q[-1], intervals, samples.camera)
            s = cache[intervals]
            value = float(fn(s, m.get("parameters", {})) if fn else builtin(s, m)) * m.get("scale", 1)
            if not np.isfinite(value) or value < 0:
                raise ValueError(f"Metric {m['name']} returned {value}; costs must be finite and nonnegative")
            result.append(value)
        return np.array(result)
