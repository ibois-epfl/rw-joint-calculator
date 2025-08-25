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
        axial_force: geometry.Vector,
        screw_axis: geometry.Vector,
    ):
        self.moment = moment
        self.rotation_point = rotation_point
        self.joint = joint
        self.axial_force = axial_force
        self.screw_axis = screw_axis / screw_axis.norm()
        self.total_tensile_component = 0

    def compute_stress_field(self):
        """
        The stress field along the faces is modelled as a truncated volume extruded from the face
        """
        stress_volumes = []
        unit_stresses = []
        raw_unit_moment = geometry.Vector(0, 0, 0)
        for joint_face in self.joint.faces:
            # Compute the stress distribution on each face
            max_unit_stress = self.__compute_moment_unit_stresses(joint_face)
            stress_volumes.append(volume_lambda(joint_face.stress_distribution))
            unit_stresses.append(max_unit_stress)
        total_volume = sum(stress_volumes)
        self.joint.moment_weights = [x / total_volume for x in stress_volumes]

        for i, joint_face in enumerate(self.joint.faces):
            moment_arm = joint_face.resultant_location - self.rotation_point
            total_unit_force_on_face = volume_lambda(joint_face.stress_distribution)
            force_angle = self.moment.compute_angle_with(joint_face.normal)
            if force_angle > math.pi / 2:
                force_angle = math.pi - force_angle
            tensile_component = total_unit_force_on_face * math.cos(force_angle)
            cross_product = moment_arm.cross(
                joint_face.normal * total_unit_force_on_face
            )
            # all faces must participate in the same direction:
            if cross_product.dot(self.moment) < 0:
                cross_product = -1 * cross_product
            raw_unit_moment += cross_product

        unit_resisting_moment = raw_unit_moment.dot(self.moment / self.moment.norm())
        amplification_factor = self.moment.norm() / unit_resisting_moment
        for i, joint_face in enumerate(self.joint.faces):
            joint_face.max_stress = amplification_factor * unit_stresses[i]
            force_angle = self.moment.compute_angle_with(joint_face.normal)
            if force_angle > math.pi / 2:
                force_angle = math.pi - force_angle
            tensile_component = (
                volume_lambda(joint_face.stress_distribution)
                * math.cos(force_angle)
                * amplification_factor
            )
            self.total_tensile_component += tensile_component

        self.total_tensile_component /= 2  # because the tensile force is equally shared between the two beams (the screw only takes half on both ends)

        self.__compute_axial_stresses()

    def __compute_moment_unit_stresses(self, joint_face: face.JointFace):
        """
        Compute the stress distribution on a single face based on the applied moment and rotation axis.
        This is done by creating a Rhino.Geometry.Brep volume that represents the stress distribution resulting from a 10 degree rotation around the rotation axis.

        :param face: The face to compute the stress distribution for.
        :type face: face.JointFace

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
        scaled_brep_vertices = [pt.Location for pt in joint_face.brep_surface.Vertices]
        mean_pt = Rhino.Geometry.Point3d(0, 0, 0)
        for pt in scaled_brep_vertices:
            mean_pt += pt
        mean_pt /= len(scaled_brep_vertices)
        (is_on_face, crv, pts) = Rhino.Geometry.Intersect.Intersection.BrepBrep(
            splitting_brep, joint_face.brep_surface, TOL
        )
        scaled_brep_face = joint_face.brep_surface.DuplicateBrep()
        scaled_brep_face.Transform(Rhino.Geometry.Transform.Scale(mean_pt, 3.0))
        (
            success,
            intersection_curves,
            intersection_points,
        ) = Rhino.Geometry.Intersect.Intersection.BrepBrep(
            splitting_brep, scaled_brep_face, TOL
        )
        initial_face_center = Rhino.Geometry.AreaMassProperties.Compute(
            joint_face.brep_surface
        ).Centroid
        split_faces = (
            joint_face.brep_surface.Split(splitting_brep, TOL)
            if len(crv) > 0
            else [joint_face.brep_surface]
        )
        split_face = None
        if len(split_faces) > 1:
            for candidate in split_faces:
                candidate_center = Rhino.Geometry.AreaMassProperties.Compute(
                    candidate
                ).Centroid
                cross_product = Rhino.Geometry.Vector3d.CrossProduct(
                    candidate_center - initial_face_center,
                    initial_face_center - self.rotation_point.to_point_3d(),
                )
                dot = cross_product * joint_face.normal.to_vector_3d()
                if dot < 0:
                    split_face = candidate
                    split_face_center = candidate_center
                    break
        else:
            split_face = split_faces[0]
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
            Rhino.Geometry.Transform.Translation(
                2 * TOL * joint_face.normal.to_vector_3d()
            )
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
        joint_face.set_stress_face_plane(
            geometry.Plane.from_rhino_plane(splitting_plane)
        )
        joint_face.create_stress_distribution()

        max_unit_stress = 0
        for edge in joint_face.stress_distribution.GetWireframe(0):
            edge_vector = geometry.Vector.from_vector_3d(
                Rhino.Geometry.Vector3d(edge.PointAtStart - edge.PointAtEnd)
            )
            if edge_vector.is_parallel_to(joint_face.normal, 0.1):
                stress = edge_vector.norm()
                if stress > max_unit_stress:
                    max_unit_stress = stress

        return max_unit_stress

    def __compute_axial_stresses(self):
        """
        Compute the axial stresses on each face based on an axial force applied to the joint.
        To allow for force decomposition before

        :return: None
        """
        main_axes = self.joint.main_axes
        if len(main_axes) != 2:
            raise ValueError("Joint must have exactly two main axes")
        main_axes_angle = main_axes[0].compute_angle_with(main_axes[1])
        force_axis_angle = main_axes[0].compute_angle_with(self.axial_force)
        projected_force_on_axe_1 = (
            self.axial_force.norm()
            / math.sin(math.pi - main_axes_angle)
            * math.sin(main_axes_angle - force_axis_angle)
            * (main_axes[0] / main_axes[0].norm())
        )
        projected_force_on_axe_2 = self.axial_force - projected_force_on_axe_1
        assert (
            projected_force_on_axe_1 + projected_force_on_axe_2 - self.axial_force
        ).norm() < 1e-6, "Force decomposition failed"
        projected_forces = [projected_force_on_axe_1, projected_force_on_axe_2]
        if projected_force_on_axe_1.norm() / projected_force_on_axe_2.norm() > 50:
            print(
                "Warning: projected force on axis 2 is less than 2 % of the one on axis 1. It is therefore neglected."
            )
            projected_forces.pop(1)
        elif projected_force_on_axe_2.norm() / projected_force_on_axe_1.norm() > 50:
            print(
                "Warning: projected force on axis 1 is less than 2 % of the one on axis 2. It is therefore neglected."
            )
            projected_forces.pop(0)
        for i in range(len(projected_forces)):
            # Small trick that is reversed later in the function
            if i == 1:
                for face in self.joint.faces:
                    face.normal = -1 * face.normal
            facing_face_id = 0
            current_best_dot = 0.0
            for j in range(1, len(self.joint.faces)):
                joint_face = self.joint.faces[j]
                dot = joint_face.normal.dot(
                    projected_forces[i] / projected_forces[i].norm()
                )
                if dot < current_best_dot:
                    facing_face_id = j
                    current_best_dot = dot

            # Rhino.RhinoDoc.ActiveDoc.Objects.AddBrep(self.joint.faces[facing_face_id].brep_surface)
            helping_face_id = 0
            current_best_dot = 0
            for j in range(1, len(self.joint.faces)):
                if j == facing_face_id:
                    continue
                dot = self.joint.faces[facing_face_id].normal.dot(
                    self.joint.faces[j].normal
                )
                if dot > current_best_dot:
                    current_best_dot = dot
                    helping_face_id = j

            # Compute the equilibrium in the plane perpendicular to the force axis.
            projected_screw_axis = self.screw_axis.project_in_plane(projected_forces[i])
            projected_facing_normal = self.joint.faces[
                facing_face_id
            ].normal.project_in_plane(projected_forces[i])
            projected_helping_normal = self.joint.faces[
                helping_face_id
            ].normal.project_in_plane(projected_forces[i])
            screw_axis_projection_factor = (
                projected_screw_axis.norm() / self.screw_axis.norm()
            )
            facing_normal_projection_factor = (
                projected_facing_normal.norm()
                / self.joint.faces[facing_face_id].normal.norm()
            )
            helping_normal_projection_factor = (
                projected_helping_normal.norm()
                / self.joint.faces[helping_face_id].normal.norm()
            )
            screw_facing_face_angle = projected_screw_axis.compute_angle_with(
                projected_facing_normal
            )
            screw_helping_face_angle = projected_screw_axis.compute_angle_with(
                projected_helping_normal
            )
            if screw_facing_face_angle > math.pi / 2:
                screw_facing_face_angle = math.pi - screw_facing_face_angle
            if screw_helping_face_angle > math.pi / 2:
                screw_helping_face_angle = math.pi - screw_helping_face_angle

            # Assuming a unit force on the helping face:
            projected_screw_component = math.sin(
                math.pi - screw_helping_face_angle - screw_facing_face_angle
            ) / math.sin(screw_facing_face_angle)
            projected_facing_normal_component = math.sin(
                screw_helping_face_angle
            ) / math.sin(screw_facing_face_angle)

            # I call facing_unit_resultant the resultant vector on the facing face when the force on the helping face is 1 N
            facing_unit_resultant = (
                projected_facing_normal_component / facing_normal_projection_factor
            ) * self.joint.faces[facing_face_id].normal
            screw_unit_resultant = (
                projected_screw_component / screw_axis_projection_factor
            ) * self.screw_axis
            helping_unit_resultant = (
                -1 / helping_normal_projection_factor
            ) * self.joint.faces[helping_face_id].normal

            # Here we reverse the trick done at the beginning of this function
            if i == 1:
                for face in self.joint.faces:
                    face.normal = -1 * face.normal

            # Now I scale everything to make sure that they all counteract the exerted force
            resisting_unit_resultant = (
                facing_unit_resultant.project_along(projected_forces[i])
                + helping_unit_resultant.project_along(projected_forces[i])
                + screw_unit_resultant.project_along(projected_forces[i])
            )
            scaling_factor = (
                projected_forces[i].norm() / resisting_unit_resultant.norm()
            )
            facing_resultant = facing_unit_resultant * scaling_factor
            helping_resultant = helping_unit_resultant * scaling_factor
            screw_resultant = screw_unit_resultant * scaling_factor
            sigma_facing = facing_resultant.norm() / (
                self.joint.faces[facing_face_id].area
            )
            sigma_helping = helping_resultant.norm() / (
                self.joint.faces[helping_face_id].area
            )
            self.joint.faces[facing_face_id].increase_stress_distribution(sigma_facing)
            self.joint.faces[helping_face_id].increase_stress_distribution(
                sigma_helping
            )
            print(f"force in screw: {screw_resultant.norm()} N")
