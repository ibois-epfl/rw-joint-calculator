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

    def __truediv__(self, other: float):
        if isinstance(other, (int, float)):
            return Vector(self.x / other, self.y / other, self.z / other)
        return NotImplemented

    def norm(self):
        return math.sqrt(self.x**2 + self.y**2 + self.z**2)

    def dot(self, other: "Vector"):
        if not isinstance(other, Vector):
            return NotImplemented
        return self.x * other.x + self.y * other.y + self.z * other.z

    def cross(self, other: "Vector"):
        if not isinstance(other, Vector):
            return NotImplemented
        return Vector(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

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

    def project_in_plane(self, normal: "Vector"):
        """Project the vector onto a plane defined by a normal vector."""
        if not isinstance(normal, Vector):
            return NotImplemented
        if normal.norm() == 0:
            raise ValueError("Normal vector cannot be null.")
        normal = normal / normal.norm()
        return self - normal * self.dot(normal)

    def project_along(self, other: "Vector"):
        if not isinstance(other, Vector):
            return NotImplemented
        if other.norm() == 0:
            raise ValueError("Other vector cannot be null.")
        other = other / other.norm()
        return other * self.dot(other)

    def normalized(self):
        norm = self.norm()
        if norm == 0:
            raise ValueError("Cannot normalize zero-length vector.")
        return self / norm


@dataclass
class Point:
    x: float
    y: float
    z: float

    @staticmethod
    def from_point_3d(point_3d: Rhino.Geometry.Point3d):
        return Point(point_3d.X, point_3d.Y, point_3d.Z)

    def to_point_3d(self):
        return Rhino.Geometry.Point3d(self.x, self.y, self.z)

    def __sub__(self, other: "Point"):
        if isinstance(other, Point):
            return Vector(self.x - other.x, self.y - other.y, self.z - other.z)
        elif isinstance(other, Vector):
            return Point(self.x - other.x, self.y - other.y, self.z - other.z)
        return NotImplemented

    def __add__(self, other: Vector):
        if not isinstance(other, Vector):
            return NotImplemented
        return Point(self.x + other.x, self.y + other.y, self.z + other.z)


@dataclass
class Plane:
    origin: Point
    x_axis: Vector
    y_axis: Vector

    def __post_init__(self):
        self.x_axis = self.x_axis / self.x_axis.norm()
        self.y_axis = self.y_axis / self.y_axis.norm()
        self.z_axis = self.x_axis.cross(self.y_axis)

    @staticmethod
    def from_rhino_plane(plane: Rhino.Geometry.Plane):
        origin = Point.from_point_3d(plane.Origin)
        x_axis = Vector.from_vector_3d(plane.XAxis)
        y_axis = Vector.from_vector_3d(plane.YAxis)
        return Plane(origin, x_axis, y_axis)

    def to_rhino_plane(self):
        return Rhino.Geometry.Plane(
            self.origin.to_point_3d(),
            self.x_axis.to_vector_3d(),
            self.y_axis.to_vector_3d(),
        )

    def translate(self, translation: Vector):
        if not isinstance(translation, Vector):
            return NotImplemented
        self.origin += translation
        return self
