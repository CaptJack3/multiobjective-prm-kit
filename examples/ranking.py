"""Example environment-space objective; all objectives are assumed minimized."""
import numpy as np


def score(summary, config, parameters):
    """Larger relative spread of completed returned cost vectors ranks higher."""
    values = np.asarray(summary["costs"], float)
    if len(values) < 2:
        return 0.0
    span = values.max(axis=0) - values.min(axis=0)
    scale = np.maximum(np.abs(values).max(axis=0), 1e-12)
    return float(np.linalg.norm(span / scale))
