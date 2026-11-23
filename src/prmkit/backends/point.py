"""A translating disk/sphere, for a dependency-light end-to-end example."""
import numpy as np

from . import State


class PointBackend:
    def __init__(self, robot, environment):
        self.lower = np.array(robot.get("lower", [0, 0]), float)
        self.upper = np.array(robot.get("upper", [1, 1]), float)
        self.weights = np.array(robot.get("weights", np.ones(len(self.lower))), float)
        self.joint_names = [f"position_{i}" for i in range(len(self.lower))]
        self.radius = float(robot.get("radius", 0.01))
        if len(self.lower) not in (2, 3) or not np.isfinite(self.radius) or self.radius < 0:
            raise ValueError("Point backend needs 2 or 3 dimensions and nonnegative radius")
        self.objects = [x for x in environment.get("objects", []) if x.get("collider", True)]
        for obj in self.objects:
            if obj.get("shape", "box") != "box" or "rotation_rpy" in obj:
                raise ValueError("Point demo supports axis-aligned boxes; use URDF for other geometry")
            for key in ("position_m", "dimensions_m"):
                a = np.asarray(obj[key], float)
                if a.shape != (len(self.lower),) or not np.isfinite(a).all():
                    raise ValueError(f"Invalid box {key}")
            if np.any(np.array(obj["dimensions_m"]) <= 0):
                raise ValueError("Box dimensions must be positive")
        self.world_mask = np.ones(len(self.objects), bool)

    def state(self, q):
        ds = []
        for obj in self.objects:
            delta = np.abs(q - np.asarray(obj["position_m"])) - np.asarray(obj["dimensions_m"]) / 2
            ds.append(np.linalg.norm(np.maximum(delta, 0)) + min(float(delta.max()), 0) - self.radius)
        return State(np.asarray(ds), np.pad(q, (0, 3 - len(q))))

    def motion_bound(self, dq):
        return np.full(len(self.objects), np.linalg.norm(dq))

    def visibility(self, q, camera):
        target = np.asarray(camera["target"], float)
        direction = target - q
        if target.shape != q.shape:
            raise ValueError("Point visibility target must match robot dimension")
        for obj in self.objects:
            low = np.array(obj["position_m"]) - np.array(obj["dimensions_m"]) / 2
            high = np.array(obj["position_m"]) + np.array(obj["dimensions_m"]) / 2
            near, far = 0., 1.
            for j, d in enumerate(direction):
                if abs(d) < 1e-15:
                    if q[j] < low[j] or q[j] > high[j]:
                        near, far = 1., 0.
                        break
                else:
                    t0, t1 = sorted(((low[j] - q[j]) / d, (high[j] - q[j]) / d))
                    near, far = max(near, t0), min(far, t1)
            if near <= far:
                return 0.
        return 1.

