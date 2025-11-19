"""
This module contains the dataclass for joint faces
"""

from dataclasses import dataclass

import Rhino

import joint_calc


@dataclass
class JointFace:
    """
    A dataclass to store joint face information.
    JointFaces are assumed to be flat
    """

    id: int
    parent_joint_id: int
    rh_joint_brep_face: Rhino.Geometry.BrepFace
    resultant_location: joint_calc.geometry.Point = None  # The location of the resultant force vector that will be applied on this face
    stress_distribution: Rhino.Geometry.Brep = None
    max_stress: float = 0.0

    def __post_init__(self):
        self.area = joint_calc.utils.compute_area(self.rh_joint_brep_face)
        self.normal = self.rh_joint_brep_face.NormalAt(0, 0)
