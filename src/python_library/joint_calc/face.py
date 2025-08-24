"""
This module contains the dataclass for joint faces
"""

from dataclasses import dataclass

from joint_calc import geometry

import Rhino


@dataclass
class JointFace:
    id: int
    parent_joint_id: int
    area: float
    normal: geometry.Vector
    brep_surface: Rhino.Geometry.Brep
    resultant_location: geometry.Point = None  # The location of the resultant force vector that will be applied on this face
    stress_distribution: Rhino.Geometry.Brep = None
    max_stress: float = 0.0
    main_axis: geometry.Vector = None

    def __post_init__(self):
        self.__detect_main_axis()
        self.normal = self.normal / self.normal.norm()  # Just to make sure

    def __detect_main_axis(self):
        """
        Detect the main axis of the face based on its normal vector.
        """
        edges = [
            geometry.Vector.from_vector_3d(edge.PointAtEnd - edge.PointAtStart)
            for edge in self.brep_surface.Edges
        ]
        for i, first_candidate_edge_vector in enumerate(edges):
            first_candidate_edge_vector = (
                first_candidate_edge_vector / first_candidate_edge_vector.norm()
            )
            if first_candidate_edge_vector.z > 0:
                first_candidate_edge_vector = -1 * first_candidate_edge_vector
            for j in range(i + 1, len(edges)):
                second_candidate_edge_vector = edges[j]
                second_candidate_edge_vector = (
                    second_candidate_edge_vector / second_candidate_edge_vector.norm()
                )
                if second_candidate_edge_vector.z > 0:
                    second_candidate_edge_vector = -1 * second_candidate_edge_vector
                # Check if the two candidate vectors are orthogonal
                if first_candidate_edge_vector.is_parallel_to(
                    second_candidate_edge_vector, 0.01
                ):
                    self.main_axis = (
                        first_candidate_edge_vector / first_candidate_edge_vector.norm()
                    )
                    return
