"""
This module calculates the stress field in a joint based on a moment and a rotation axis.
It assumes a perfectly elastic material behavior (which is not completely accurate with wood)
"""

from joint_calc import geometry, joint, face

import Rhino

import math


def volume_lambda(brep):
    volume = abs(Rhino.Geometry.VolumeMassProperties.Compute(brep).Volume)
    return volume


def surface_lambda(brep):
    return Rhino.Geometry.AreaMassProperties.Compute(brep).Area


TOL = Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance


class StressFieldComputer:
    def __init__(
        self,
        moment: geometry.Vector,
        rotation_point: geometry.Point,
        joint: joint.Joint,
    ):
        self.moment = moment
        self.rotation_point = rotation_point
        self.joint = joint
        self.total_tensile_component = 0

    def compute_stress_field(self):
        """
        The stress field along the faces is modelled as a truncated volume extruded from the face
        """
        stress_volumes = []
        unit_stresses = []
        for joint_face in self.joint.faces:
            # Compute the stress distribution on each face
            stress_distribution, max_unit_stress = self.__compute_face_unit_stresses(
                joint_face
            )
            stress_volumes.append(volume_lambda(joint_face.stress_distribution))
            unit_stresses.append(max_unit_stress)
        total_volume = sum(stress_volumes)
        stress_volumes = [x / total_volume for x in stress_volumes]
        self.joint.moment_weights = stress_volumes

        for i, joint_face in enumerate(self.joint.faces):
            moment_arm = joint_face.resultant_location - self.rotation_point
            total_force_on_face = volume_lambda(joint_face.stress_distribution)
            force_angle = Rhino.Geometry.Vector3d.VectorAngle(
                self.moment.to_vector_3d(), joint_face.normal.to_vector_3d()
            )
            if force_angle > math.pi / 2:
                force_angle = math.pi - force_angle
            tensile_component = total_force_on_face * math.cos(force_angle)
            raw_unit_moment = Rhino.Geometry.Vector3d.CrossProduct(
                moment_arm.to_vector_3d(),
                joint_face.normal.to_vector_3d() * total_force_on_face,
            )
            raw_unit_moment_angle = Rhino.Geometry.Vector3d.VectorAngle(
                raw_unit_moment, self.moment.to_vector_3d()
            )
            moment_resisting_component = math.cos(
                raw_unit_moment_angle
            ) * geometry.Vector.from_vector_3d(raw_unit_moment)
            actual_moment = self.moment * self.joint.moment_weights[i]
            if actual_moment.norm() != 0:
                amplification_factor = (
                    actual_moment.norm() / moment_resisting_component.norm()
                )
                joint_face.max_stress = amplification_factor * unit_stresses[i]
                tensile_component *= amplification_factor
                self.total_tensile_component += tensile_component
            else:
                raise ValueError("Actual moment is zero, which is not allowed.")

        self.total_tensile_component /= 2  # because the tensile force is equally shared between the two beams (the screw only takes half on both ends)

    def __compute_face_unit_stresses(self, joint_face: face.JointFace):
        """
        Compute the stress distribution on a single face based on the applied moment and rotation axis.
        This is done by creating a Rhino.Geometry.Brep volume that represents the stress distribution resulting from a 1 degree rotation around the rotation axis.

        :param face: The face to compute the stress distribution for.
        :type face: face.JointFace

        :return stress_distribution: The stress distribution on the face, without scale.
        :rtype: Rhino.Geometry.Brep

        :return max_unit_stress: The maximum unit stress on the face.
        :rtype: float
        """
        interval = Rhino.Geometry.Interval(-10, 10)
        splitting_plane = Rhino.Geometry.Plane(
            self.rotation_point.to_point_3d(),
            joint_face.normal.to_vector_3d(),
            self.moment.to_vector_3d(),
        )
        splitting_brep = Rhino.Geometry.PlaneSurface(
            splitting_plane, interval, interval
        ).ToBrep()
        (
            success,
            intersection_curves,
            intersection_points,
        ) = Rhino.Geometry.Intersect.Intersection.BrepBrep(
            splitting_brep, joint_face.brep_surface, TOL
        )
        split_faces = joint_face.brep_surface.Split(splitting_brep, TOL)
        sorted_split_faces = sorted(split_faces, key=surface_lambda, reverse=True)
        split_face = sorted_split_faces[0]
        split_face_center = Rhino.Geometry.AreaMassProperties.Compute(
            split_face
        ).Centroid

        lines = []
        for edge in split_face.Edges:
            line = Rhino.Geometry.Line(edge.PointAtStart, edge.PointAtEnd)
            lines.append(line)
        nurbs_curve = Rhino.Geometry.Polyline.CreateByJoiningLines(lines, TOL, False)[
            0
        ].ToPolylineCurve()
        brep_volume = Rhino.Geometry.Surface.CreateExtrusion(
            nurbs_curve, -100 * joint_face.normal.to_vector_3d()
        ).ToBrep()
        brep_volume.Transform(
            Rhino.Geometry.Transform.Translation(TOL * joint_face.normal.to_vector_3d())
        )  # Because otherwise the split fails...
        splitting_plane_x_axis = Rhino.Geometry.Vector3d(
            intersection_curves[0].PointAtStart - intersection_curves[0].PointAtEnd
        )
        splitting_plane_y_axis = Rhino.Geometry.Vector3d(
            intersection_curves[0].PointAtStart - split_face_center
        )

        dot_product = (
            Rhino.Geometry.Vector3d.CrossProduct(
                splitting_plane_y_axis, splitting_plane_x_axis
            )
            * joint_face.normal.to_vector_3d()
        )
        splitting_plane = Rhino.Geometry.Plane(
            intersection_curves[0].PointAtStart,
            splitting_plane_x_axis,
            splitting_plane_y_axis,
        )
        if dot_product > 0:
            splitting_plane.Transform(
                Rhino.Geometry.Transform.Rotation(
                    -math.pi / 18,
                    splitting_plane_x_axis,
                    intersection_curves[0].PointAtStart,
                )
            )
        else:
            splitting_plane.Transform(
                Rhino.Geometry.Transform.Rotation(
                    math.pi / 18,
                    splitting_plane_x_axis,
                    intersection_curves[0].PointAtStart,
                )
            )
        splitting_brep = Rhino.Geometry.PlaneSurface(
            splitting_plane, interval, interval
        ).ToBrep()
        candidate_volumes = brep_volume.Split(splitting_brep, TOL)

        capped_volumes = []
        for candidate in candidate_volumes:
            candidate = candidate.CapPlanarHoles(10 * TOL)
            capped_volumes.append(candidate)

        stress_distribution = sorted(capped_volumes, key=volume_lambda, reverse=False)[
            0
        ]
        centroid = Rhino.Geometry.AreaMassProperties.Compute(
            stress_distribution
        ).Centroid
        resultant_axis_line = Rhino.Geometry.Line(
            centroid, centroid + joint_face.normal.to_vector_3d()
        )
        (
            intersection_result,
            curves,
            points,
        ) = Rhino.Geometry.Intersect.Intersection.CurveBrep(
            resultant_axis_line.ToNurbsCurve(), joint_face.brep_surface, TOL
        )

        max_unit_stress = 0
        for edge in stress_distribution.GetWireframe(0):
            edge_vector = geometry.Vector.from_vector_3d(
                Rhino.Geometry.Vector3d(edge.PointAtStart - edge.PointAtEnd)
            )
            if edge_vector.is_parallel_to(joint_face.normal, 0.1):
                stress = edge_vector.norm()
                if stress > max_unit_stress:
                    max_unit_stress = stress

        joint_face.resultant_location = geometry.Point(
            points[0].X, points[0].Y, points[0].Z
        )
        joint_face.stress_distribution = stress_distribution
        return stress_distribution, max_unit_stress
