"""
This is a module to store joints with their faces
"""

from dataclasses import dataclass

from joint_calc import face


def remove_duplicate_vectors(vectors):
    unique_vectors = []
    for vec in vectors:
        vec = vec / vec.norm()
        for other in unique_vectors:
            other = other / other.norm()
            if vec.dot(other) > 0.9:  # If the vectors are nearly equal
                break
        else:
            unique_vectors.append(vec)
    return unique_vectors


@dataclass
class Joint:
    name: str
    faces: list[face.JointFace]
    moment_weights: list[
        float
    ]  # How much the respective faces contribute to the overall moment
    main_axes: list = None

    def __post_init__(self):
        self.main_axes = self.__get_main_axes()

    def __get_main_axes(self):
        """
        Detect the main axes of the joint based on the face orientations.
        """
        main_axes = [joint_face.main_axis for joint_face in self.faces]
        main_axes = remove_duplicate_vectors(main_axes)
        return main_axes
