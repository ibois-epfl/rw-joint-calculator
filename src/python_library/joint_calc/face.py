"""
This module contains the dataclass for joint faces
"""

from dataclasses import dataclass
import math

import Rhino

from . import geometry, utils


@dataclass
class JointSubFace:
    """
    A dataclass to store joint sub-face information.
    JointSubFaces are the result of subdividing JointFaces into smaller faces for more accurate stress calculations.
    """

    id: int
    parent_joint_face_id: int
    rh_joint_brep_face: Rhino.Geometry.BrepFace
    rh_normal: Rhino.Geometry.Vector3d
    mesh: Rhino.Geometry.Mesh = None
    Young_modulus: float = None
    max_bending_stress: float = 0.0
    axial_stress: float = 0.0
    location_of_max_stress: geometry.Point = None
    area: float = None
    centroid: geometry.Point = None
    effective_depth: float = None

    def __post_init__(self):
        self.area = utils.compute_area(self.rh_joint_brep_face)
        self.centroid = utils.compute_centroid(self.rh_joint_brep_face)
        self.effective_depth = math.sqrt(self.area)


@dataclass
class JointFace:
    """
    A dataclass to store joint face information.
    JointFaces are assumed to be flat
    """

    id: int
    parent_joint_id: int
    rh_joint_brep_face: Rhino.Geometry.BrepFace
    max_stress: float = -1.0
    effective_depth: float = None
    Young_modulus: float = None
    location_of_max_stress: geometry.Point = None
    bending_subface: JointSubFace = None
    axial_subface: JointSubFace = None

    def __post_init__(self):
        self.area = utils.compute_area(self.rh_joint_brep_face)
        self.centroid = utils.compute_centroid(self.rh_joint_brep_face)
        success, u, v = self.rh_joint_brep_face.ClosestPoint(
            self.centroid.to_point_3d()
        )
        self.rh_normal = self.rh_joint_brep_face.NormalAt(u, v)

    def compute_inertia(self, base_plane: Rhino.Geometry.Plane) -> float:
        """Compute the moment of inertia of the joint face around a given base plane."""
        transform = Rhino.Geometry.Transform.PlaneToPlane(
            base_plane, Rhino.Geometry.Plane.WorldXY
        )
        rh_face_copy = self.rh_joint_brep_face.Brep.Duplicate()
        rh_face_copy.Transform(transform)

        area_properties = Rhino.Geometry.AreaMassProperties.Compute(rh_face_copy)
        if area_properties is None:
            raise ValueError(
                "Could not compute area properties for the given Brep face."
            )
        return geometry.Vector.from_vector_3d(
            area_properties.WorldCoordinatesSecondMoments
        )

    @property
    def cumulated_stresses(self):
        sigma_bending = (
            self.bending_subface.max_bending_stress
            if self.bending_subface is not None
            else 0.0
        )
        sigma_axial = (
            self.axial_subface.axial_stress if self.axial_subface is not None else 0.0
        )
        return sigma_bending + sigma_axial
