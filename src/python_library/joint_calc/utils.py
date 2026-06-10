""" "This module collects utility functions, especially for Rhino-specific operations."""

import Rhino

import math

from . import geometry


def compute_area(brep_face: Rhino.Geometry.Brep) -> float:
    """Compute the area of a Brep face."""
    area_properties = Rhino.Geometry.AreaMassProperties.Compute(brep_face)
    if area_properties is None:
        raise ValueError("Could not compute area for the given Brep face.")
    return area_properties.Area


def compute_centroid(brep_face: Rhino.Geometry.Brep) -> geometry.Point:
    """Compute the centroid of a Brep face."""
    area_properties = Rhino.Geometry.AreaMassProperties.Compute(brep_face)
    if area_properties is None:
        raise ValueError("Could not compute centroid for the given Brep face.")
    centroid_3d = area_properties.Centroid
    return geometry.Point.from_Point3d(centroid_3d)


def compute_young_modulus(
    grain_orientation: geometry.Vector,
    face_normal: geometry.Vector,
    E0: float,
    E90: float,
) -> float:
    """Compute the Young's modulus based on the grain orientation using a simple interpolation."""
    # Following Hankinson's formula:
    theta = grain_orientation.compute_angle_with(face_normal)
    if theta == 0:
        return E0
    elif theta == math.pi / 2:
        return E90
    else:
        if theta > math.pi / 2:
            theta = math.pi - theta
        E = (E0 * E90) / (E0 * math.sin(theta) ** 2 + E90 * math.cos(theta) ** 2)
        print(
            f"Computing Young's modulus of {E / 1e9:.2f} GPa with theta: {math.degrees(theta):.2f} degrees"
        )
        return E
