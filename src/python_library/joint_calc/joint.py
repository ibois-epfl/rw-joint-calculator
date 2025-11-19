"""
This is a module to store joints with their faces
"""

from dataclasses import dataclass

import joint_calc


@dataclass
class Joint:
    id: int
    original_faces: list[joint_calc.face.JointFace]
    moment_vector: joint_calc.geometry.Vector
    working_faces: list[joint_calc.face.JointFace] = None

    def __post_init__(self):
        for joint_face in self.original_faces:
            rh_joint_brep_face = joint_face.rh_joint_brep_face
            rh_normal = rh_joint_brep_face.NormalAt(0, 0)
            if rh_normal * self.moment_vector.to_vector_3d() < 0:
                print("Inverting normal vector.")
                rh_normal *= -1
