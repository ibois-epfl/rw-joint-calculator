"""
This is a module to store joints with their faces
"""

from dataclasses import dataclass

import face

@dataclass
class Joint:
    name: str
    faces: list[face.JointFace]
    