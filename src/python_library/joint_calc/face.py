"""
This module contains the dataclass for joint faces
"""

from dataclasses import dataclass

import Rhino

from . import geometry, utils


@dataclass
class JointFace:
    """
    A dataclass to store joint face information.
    JointFaces are assumed to be flat
    """

    id: int
    parent_joint_id: int
    rh_joint_brep_face: Rhino.Geometry.BrepFace
    max_stress: float = 0.0
    effective_depth: float = None
    Young_modulus: float = None
    location_of_max_stress: geometry.Point = None
    mesh: Rhino.Geometry.Mesh = None

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
