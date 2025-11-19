""" "This module collects utility functions, especially for Rhino-specific operations."""

import Rhino

import joint_calc


def compute_area(brep_face: Rhino.Geometry.Brep) -> float:
    """Compute the area of a Brep face."""
    area_properties = Rhino.Geometry.AreaMassProperties.Compute(brep_face)
    if area_properties is None:
        raise ValueError("Could not compute area for the given Brep face.")
    return area_properties.Area


def compute_centroid(brep_face: Rhino.Geometry.Brep) -> joint_calc.geometry.Point:
    """Compute the centroid of a Brep face."""
    area_properties = Rhino.Geometry.AreaMassProperties.Compute(brep_face)
    if area_properties is None:
        raise ValueError("Could not compute centroid for the given Brep face.")
    centroid_3d = area_properties.Centroid
    return joint_calc.geometry.Point.from_Point3d(centroid_3d)
