"""Robot extension boundary; the planner depends on this small protocol."""
from dataclasses import dataclass
from typing import Protocol

import numpy as np


@dataclass
class State:
    distances: np.ndarray
    ee: np.ndarray


class Backend(Protocol):
    lower: np.ndarray
    upper: np.ndarray
    weights: np.ndarray
    world_mask: np.ndarray
    joint_names: list[str]

    def state(self, q: np.ndarray) -> State: ...
    def motion_bound(self, dq: np.ndarray) -> np.ndarray: ...
    def visibility(self, q: np.ndarray, camera: dict) -> float: ...


def make_backend(config):
    robot = config["robot"]
    kind = robot.get("backend", "urdf")
    if kind == "point":
        from .point import PointBackend
        backend = PointBackend(robot, config["environment"])
    elif kind == "urdf":
        from .urdf import URDFBackend
        backend = URDFBackend(robot, config["environment"])
    elif kind == "plugin":
        from ..io import plugin
        backend = plugin(robot["plugin"], robot["_base"])(robot, config["environment"])
    else:
        raise ValueError(f"Unknown robot backend: {kind}")
    for name in ("lower", "upper", "weights"):
        setattr(backend, name, np.asarray(getattr(backend, name), dtype=float))
    backend.world_mask = np.asarray(backend.world_mask, dtype=bool)
    if backend.world_mask.ndim != 1:
        raise ValueError("Robot world_mask must be one-dimensional")
    d = len(backend.lower)
    if d == 0 or backend.upper.shape != (d,) or backend.weights.shape != (d,):
        raise ValueError("Robot limits and weights must have the same nonzero dimension")
    if not all(np.isfinite(getattr(backend, k)).all() for k in ("lower", "upper", "weights")):
        raise ValueError("Finite robot limits and weights are required")
    if np.any(backend.lower >= backend.upper) or np.any(backend.weights <= 0):
        raise ValueError("Robot needs lower < upper and strictly positive weights")
    if len(backend.joint_names) != d or len(set(backend.joint_names)) != d:
        raise ValueError("Unique joint_names required in configuration order")
    return backend

