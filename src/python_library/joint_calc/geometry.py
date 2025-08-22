"""
a module for basic geometry dataclasses
"""

from dataclasses import dataclass

import math

import Rhino


@dataclass
class Vector:
    x: float
    y: float
    z: float

    def to_vector_3d(self):
        return Rhino.Geometry.Vector3d(self.x, self.y, self.z)

    @staticmethod
    def from_vector_3d(vector_3d: Rhino.Geometry.Vector3d):
        return Vector(vector_3d.X, vector_3d.Y, vector_3d.Z)

    def __mul__(self, other: float):
        if isinstance(other, (int, float)):
            return Vector(self.x * other, self.y * other, self.z * other)
        return NotImplemented

    def __rmul__(self, other: float):
        return self.__mul__(other)

    def __add__(self, other: "Vector"):
        if isinstance(other, Vector):
            return Vector(self.x + other.x, self.y + other.y, self.z + other.z)
        return NotImplemented

    def __sub__(self, other: "Vector"):
        if isinstance(other, Vector):
            return Vector(self.x - other.x, self.y - other.y, self.z - other.z)
        return NotImplemented

    def norm(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def compute_angle_with(self, other: "Vector"):
        if not isinstance(other, Vector):
            return NotImplemented
        dot_product = self.x * other.x + self.y * other.y + self.z * other.z
        norms = self.norm() * other.norm()

        if norms == 0:
            return 0.0

        value = max(-1.0, min(1.0, dot_product / norms))
        return math.acos(value)

    def is_parallel_to(self, other: "Vector", tolerance: float = 1e-6):
        if not isinstance(other, Vector):
            return NotImplemented
        angle = self.compute_angle_with(other)
        return abs(angle) < tolerance or abs(angle - math.pi) < tolerance


@dataclass
class Point:
    x: float
    y: float
    z: float

    def to_point_3d(self):
        return Rhino.Geometry.Point3d(self.x, self.y, self.z)

    def __sub__(self, other: "Point"):
        if isinstance(other, Point):
            return Vector(self.x - other.x, self.y - other.y, self.z - other.z)
        elif isinstance(other, Vector):
            return Point(self.x - other.x, self.y - other.y, self.z - other.z)
        return NotImplemented
