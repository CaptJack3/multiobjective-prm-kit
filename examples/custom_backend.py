"""Working 3D translating sphere backend, using the published extension API.

robot:
  backend: plugin
  plugin: custom_backend.py:make_robot
  lower: [0, 0, 0]
  upper: [1, 1, 1]
  radius: 0.03

Supply 3D start/goals and 3D axis-aligned boxes in the environment.
"""
from prmkit.backends.point import PointBackend


def make_robot(robot, environment):
    return PointBackend(robot, environment)

