"""Copy beside experiment.yaml, then select custom_metric.py:elevation_exposure."""
import numpy as np


def elevation_exposure(samples, parameters):
    floor = parameters.get("floor_m", 0.2)
    penalty = np.maximum(0.0, samples.ee[:, 2] - floor)
    return float(np.trapezoid(penalty, x=samples.t) * samples.joint_length)


def uphill_motion(samples, parameters):
    """Directional example: use symmetric: false (the custom-metric default)."""
    return float(np.maximum(0, np.diff(samples.ee[:, 2])).sum())

