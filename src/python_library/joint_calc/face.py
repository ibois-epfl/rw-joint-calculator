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

    def __detect_main_axis(self):
        """
        Detect the main axis of the face based on its normal vector.
        """
        edges = [
            geometry.Vector.from_vector_3d(edge.PointAtEnd - edge.PointAtStart)
            for edge in self.brep_surface.Edges
        ]
        for first_candidate_edge_vector in edges:
            if first_candidate_edge_vector.z > 0:
                first_candidate_edge_vector = first_candidate_edge_vector * -1
            for second_candidate_edge_vector in edges:
                if second_candidate_edge_vector.z > 0:
                    second_candidate_edge_vector = second_candidate_edge_vector * -1
                # Check if the two candidate vectors are orthogonal
                if first_candidate_edge_vector.is_parallel_to(
                    second_candidate_edge_vector, 0.1
                ):
                    self.main_axis = (
                        first_candidate_edge_vector / first_candidate_edge_vector.norm()
                    )
                    return
