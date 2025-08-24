"""
This is a module to store joints with their faces
"""

from dataclasses import dataclass

from joint_calc import face


@dataclass
class Joint:
    name: str
    faces: list[face.JointFace]
    moment_weights: list[float]  # How much this face contributes to the overall moment
