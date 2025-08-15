"""
This module contains the dataclass for joint faces
"""

from dataclasses import dataclass

import geometry

import Rhino

@dataclass
class JointFace:
    id: int
    parent_joint_id: int
    area: float
    normal: geometry.Vector
    brep_surface: Rhino.Geometry.Brep
    resultant_location: geometry.Vector # The location of the resultant force vector that will be applied on this face
