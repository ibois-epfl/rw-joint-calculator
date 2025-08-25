"""
This module contains the dataclass for joint faces
"""

from dataclasses import dataclass

from joint_calc import geometry
from joint_calc.stress_field import volume_lambda

import Rhino

TOL = Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance


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
    __stress_face_plane: geometry.Plane = None
    is_stressed: bool = False

    def __post_init__(self):
        self.__detect_main_axis()
        self.normal = self.normal / self.normal.norm()  # Just to make sure
        self.area = Rhino.Geometry.AreaMassProperties.Compute(self.brep_surface).Area

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

    def increase_stress_distribution(self, additional_stress: float):
        self.max_stress += additional_stress
        max_edge = geometry.Vector(0, 0, 0)
        for edge in self.stress_distribution.GetWireframe(0):
            edge_vector = geometry.Vector.from_vector_3d(
                Rhino.Geometry.Vector3d(edge.PointAtStart - edge.PointAtEnd)
            )
            if edge_vector.is_parallel_to(self.normal, 0.1) and (
                edge_vector.norm() > max_edge.norm()
            ):
                max_edge = edge_vector
        scale = max_edge.norm() / (self.max_stress)
        scaled_increased_stress = additional_stress * scale
        translation = -1 * scaled_increased_stress * self.normal.normalized()
        self.translate_stress_face_plane(translation)
        self.create_stress_distribution()
        # Rhino.RhinoDoc.ActiveDoc.Objects.AddBrep(self.stress_distribution)

    def set_stress_face_plane(self, plane: geometry.Plane):
        self.__stress_face_plane = plane

    def translate_stress_face_plane(self, translation: geometry.Vector):
        if self.__stress_face_plane is not None:
            self.__stress_face_plane = self.__stress_face_plane.translate(translation)

    def create_stress_distribution(self):
        """
        Create a stress distribution based on the private attribute __stress_face_plane.
        """
        if self.__stress_face_plane is None:
            raise ValueError(
                f"The face {self.id} of joint {self.parent_joint_id} has no stress face plane defined, and the stress distribution volume cannot be created."
            )
        lines = []
        for edge in self.brep_surface.Edges:
            line = Rhino.Geometry.Line(edge.PointAtStart, edge.PointAtEnd)
            lines.append(line)
        nurbs_curve = Rhino.Geometry.Polyline.CreateByJoiningLines(lines, TOL, False)[
            0
        ].ToPolylineCurve()
        lower_brep = Rhino.Geometry.Surface.CreateExtrusion(
            nurbs_curve, -10 * self.normal.to_vector_3d()
        ).ToBrep()
        upper_brep = Rhino.Geometry.Surface.CreateExtrusion(
            nurbs_curve, 5 * self.normal.to_vector_3d()
        ).ToBrep()
        joined_faces = Rhino.Geometry.Brep.JoinBreps([lower_brep, upper_brep], TOL)[0]
        total_brep = joined_faces.CapPlanarHoles(5 * TOL)
        total_brep.Faces.SplitKinkyFaces(TOL)
        total_brep.MergeCoplanarFaces(TOL)
        interval = Rhino.Geometry.Interval(-10, 10)
        first_splitting_brep = Rhino.Geometry.PlaneSurface(
            self.__stress_face_plane.to_rhino_plane(), interval, interval
        ).ToBrep()
        split_breps = total_brep.Split(first_splitting_brep, TOL)
        split_breps = [not_capped.CapPlanarHoles(5 * TOL) for not_capped in split_breps]
        sorted_split_brep = sorted(split_breps, key=volume_lambda, reverse=False)[0]
        second_splitting_brep = Rhino.Geometry.PlaneSurface(
            Rhino.Geometry.Plane(
                Rhino.Geometry.AreaMassProperties.Compute(self.brep_surface).Centroid,
                self.normal.to_vector_3d(),
            ),
            interval,
            interval,
        ).ToBrep()
        # Rhino.RhinoDoc.ActiveDoc.Objects.AddBrep(sorted_split_brep)
        # Rhino.RhinoDoc.ActiveDoc.Objects.AddBrep(second_splitting_brep)
        # Rhino.RhinoApp.WriteLine(f"len of sorted_split_brep: {len(sorted_split_brep.Split(second_splitting_brep, TOL))}")
        candidates_stress_distribution = [
            not_capped.CapPlanarHoles(5 * TOL)
            for not_capped in sorted_split_brep.Split(second_splitting_brep, TOL)
        ]
        if len(candidates_stress_distribution) == 0:
            self.is_stressed = False
            Rhino.RhinoApp.WriteLine(
                f"[INFO] face {self.id} of joint {self.parent_joint_id} has no stress on its faces."
            )
            self.stress_distribution = None
            self.resultant_location = None
        else:
            self.is_stressed = True
            self.stress_distribution = sorted(
                candidates_stress_distribution, key=volume_lambda, reverse=False
            )[0]

            # Compute the resultant location for this stress distribution
            centroid = Rhino.Geometry.AreaMassProperties.Compute(
                self.stress_distribution
            ).Centroid
            resultant_axis_line = Rhino.Geometry.Line(
                centroid, centroid + self.normal.to_vector_3d()
            )
            (
                intersection_result,
                curves,
                points,
            ) = Rhino.Geometry.Intersect.Intersection.CurveBrep(
                resultant_axis_line.ToNurbsCurve(), self.brep_surface, TOL
            )
            self.resultant_location = geometry.Point(
                points[0].X, points[0].Y, points[0].Z
            )
