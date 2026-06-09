"""
This is a module to store joints with their faces
"""

import Rhino

from dataclasses import dataclass
import math

from . import face, geometry, utils


@dataclass
class Joint:
    id: int
    original_faces: list[face.JointFace]
    moment_vector: geometry.Vector
    axial_force_vector: geometry.Vector
    rotation_point: geometry.Point
    wood_direction: geometry.Vector
    inertia_along_moment_axis: geometry.Vector = None
    k_value: float = None
    stress_resultant: geometry.Vector = None
    moment_resultant: geometry.Vector = None
    stress_display_meshes: list[Rhino.Geometry.Mesh] = None

    def detect_moment_working_faces(self):
        for joint_face in self.original_faces:
            plane = Rhino.Geometry.Plane(
                joint_face.centroid.to_point_3d(), joint_face.rh_normal
            )
            projection = Rhino.Geometry.Transform.ProjectAlong(plane, plane.Normal)
            rotation_axis = Rhino.Geometry.Line(
                self.rotation_point.to_point_3d(), self.moment_vector.to_vector_3d()
            )
            rotation_axis.Extend(1, 1)
            rotation_axis.Transform(projection)
            result = joint_face.rh_joint_brep_face.Brep.Split(
                [rotation_axis.ToNurbsCurve()],
                Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance,
            )
            if len(result) > 0:
                for r in result:
                    subface_centroid = utils.compute_centroid(r)
                    success, u, v = r.Faces[0].ClosestPoint(
                        subface_centroid.to_point_3d()
                    )
                    oriented_normal = r.Faces[0].NormalAt(u, v)
                    rot_point_to_centroid = Rhino.Geometry.Vector3d(
                        subface_centroid.x - self.rotation_point.x,
                        subface_centroid.y - self.rotation_point.y,
                        subface_centroid.z - self.rotation_point.z,
                    )
                    moment_participation = Rhino.Geometry.Vector3d.CrossProduct(
                        rot_point_to_centroid, oriented_normal
                    )
                    if moment_participation * self.moment_vector.to_vector_3d() < 0:
                        wf = face.JointSubFace(
                            id=0,
                            parent_joint_face_id=joint_face.id,
                            rh_normal=oriented_normal,
                            rh_joint_brep_face=r.Faces[0],
                            Young_modulus=joint_face.Young_modulus,
                        )
                        joint_face.bending_subface = wf
            else:
                centroid = joint_face.centroid
                oriented_normal = joint_face.rh_normal
                rot_point_to_centroid = Rhino.Geometry.Vector3d(
                    centroid.x - self.rotation_point.x,
                    centroid.y - self.rotation_point.y,
                    centroid.z - self.rotation_point.z,
                )
                moment_participation = Rhino.Geometry.Vector3d.CrossProduct(
                    rot_point_to_centroid, oriented_normal
                )
                if moment_participation * self.moment_vector.to_vector_3d() < 0:
                    wf = face.JointSubFace(
                        id=0,
                        parent_joint_face_id=joint_face.id,
                        rh_normal=oriented_normal,
                        rh_joint_brep_face=joint_face.rh_joint_brep_face,
                        Young_modulus=joint_face.Young_modulus,
                    )
                    joint_face.bending_subface = wf

    def detect_axial_force_working_faces(self):
        self.axial_force_working_faces = []
        for joint_face in self.original_faces:
            normal = joint_face.rh_normal
            if (
                normal
                * self.axial_force_vector.to_vector_3d()
                / self.axial_force_vector.norm()
                < -0.01
            ):
                wf = face.JointSubFace(
                    id=0,
                    parent_joint_face_id=joint_face.id,
                    rh_joint_brep_face=joint_face.rh_joint_brep_face,
                    rh_normal=joint_face.rh_normal,
                    Young_modulus=joint_face.Young_modulus,
                )
                joint_face.axial_subface = wf

    def compute_joint_rigidity(self):
        """
        Computes the K value of the joint based on the working faces and their inertia along the moment axis.
        """
        K = 0.0
        for joint_face in self.original_faces:
            if joint_face.bending_subface is None:
                continue
            else:
                working_face = joint_face.bending_subface
            if working_face.mesh is None:
                working_face.mesh = Rhino.Geometry.Mesh.CreateFromBrep(
                    working_face.rh_joint_brep_face.Brep,
                    Rhino.Geometry.MeshingParameters.Default,
                )[0]
                working_face.mesh.Subdivide()
            mesh = working_face.mesh

            normal = working_face.rh_normal
            Riemann_sum = 0.0
            for mesh_face in mesh.Faces:
                if mesh_face.IsQuad:
                    v0 = mesh.Vertices[mesh_face.A]
                    v1 = mesh.Vertices[mesh_face.B]
                    v2 = mesh.Vertices[mesh_face.C]
                    v3 = mesh.Vertices[mesh_face.D]
                    centroid = geometry.Point(
                        x=(v0.X + v1.X + v2.X + v3.X) / 4,
                        y=(v0.Y + v1.Y + v2.Y + v3.Y) / 4,
                        z=(v0.Z + v1.Z + v2.Z + v3.Z) / 4,
                    )
                    mesh_from_face = Rhino.Geometry.Mesh()
                    mesh_from_face.Vertices.Add(v0)
                    mesh_from_face.Vertices.Add(v1)
                    mesh_from_face.Vertices.Add(v2)
                    mesh_from_face.Vertices.Add(v3)
                    mesh_from_face.Faces.AddFace(0, 1, 2, 3)
                    area = Rhino.Geometry.AreaMassProperties.Compute(
                        mesh_from_face, True, False, False, False
                    ).Area

                else:
                    v0 = mesh.Vertices[mesh_face.A]
                    v1 = mesh.Vertices[mesh_face.B]
                    v2 = mesh.Vertices[mesh_face.C]
                    centroid = geometry.Point(
                        x=(v0.X + v1.X + v2.X) / 3,
                        y=(v0.Y + v1.Y + v2.Y) / 3,
                        z=(v0.Z + v1.Z + v2.Z) / 3,
                    )
                    mesh_from_face = Rhino.Geometry.Mesh()
                    mesh_from_face.Vertices.Add(v0)
                    mesh_from_face.Vertices.Add(v1)
                    mesh_from_face.Vertices.Add(v2)
                    mesh_from_face.Faces.AddFace(0, 1, 2)
                    area = Rhino.Geometry.AreaMassProperties.Compute(
                        mesh_from_face, True, False, False, False
                    ).Area

                d = Rhino.Geometry.Vector3d(
                    self.rotation_point.x - centroid.x,
                    self.rotation_point.y - centroid.y,
                    self.rotation_point.z - centroid.z,
                )
                d_perp = d - (d * normal) * normal
                theta = Rhino.Geometry.Vector3d.VectorAngle(
                    Rhino.Geometry.Vector3d.CrossProduct(d, normal),
                    self.moment_vector.to_vector_3d(),
                )
                Riemann_sum += area * d_perp.Length**2 * math.cos(theta)
            E = working_face.Young_modulus
            working_face.Young_modulus = E
            L = math.sqrt(working_face.area)
            working_face.effective_depth = L

            K += E * Riemann_sum / L
        self.k_value = K

        print(f"Computed K value for Joint {self.id}: {K:.2f} Nm/radians")

    def compute_moment_stress_distribution(self):
        """
        Computes the stress distribution on the working faces based on the applied moment and the computed K value.
        """
        if self.k_value is None:
            self.compute_joint_rigidity()

        psi = self.moment_vector.norm() / self.k_value
        print(
            f"Applied rotation (psi) for Joint {self.id}: {math.degrees(psi):.3f} degrees"
        )

        stress_resultant = geometry.Vector(0, 0, 0)
        moment_resultant = geometry.Vector(0, 0, 0)
        for joint_face in self.original_faces:
            if joint_face.bending_subface is None:
                continue
            else:
                working_face = joint_face.bending_subface
            if working_face.mesh is None:
                working_face.mesh = Rhino.Geometry.Mesh.CreateFromBrep(
                    working_face.rh_joint_brep_face.Brep,
                    Rhino.Geometry.MeshingParameters.Default,
                )[0]
                working_face.mesh.Subdivide()
            mesh = working_face.mesh
            normal = working_face.rh_normal
            max_stress = 0.0
            location_of_max_stress = None
            for mesh_face in mesh.Faces:
                if mesh_face.IsQuad:
                    v0 = mesh.Vertices[mesh_face.A]
                    v1 = mesh.Vertices[mesh_face.B]
                    v2 = mesh.Vertices[mesh_face.C]
                    v3 = mesh.Vertices[mesh_face.D]
                    centroid = geometry.Point(
                        x=(v0.X + v1.X + v2.X + v3.X) / 4,
                        y=(v0.Y + v1.Y + v2.Y + v3.Y) / 4,
                        z=(v0.Z + v1.Z + v2.Z + v3.Z) / 4,
                    )
                    mesh_from_face = Rhino.Geometry.Mesh()
                    mesh_from_face.Vertices.Add(v0)
                    mesh_from_face.Vertices.Add(v1)
                    mesh_from_face.Vertices.Add(v2)
                    mesh_from_face.Vertices.Add(v3)
                    mesh_from_face.Faces.AddFace(0, 1, 2, 3)
                    area = Rhino.Geometry.AreaMassProperties.Compute(
                        mesh_from_face, True, False, False, False
                    ).Area

                else:
                    v0 = mesh.Vertices[mesh_face.A]
                    v1 = mesh.Vertices[mesh_face.B]
                    v2 = mesh.Vertices[mesh_face.C]
                    centroid = geometry.Point(
                        x=(v0.X + v1.X + v2.X) / 3,
                        y=(v0.Y + v1.Y + v2.Y) / 3,
                        z=(v0.Z + v1.Z + v2.Z) / 3,
                    )
                    mesh_from_face = Rhino.Geometry.Mesh()
                    mesh_from_face.Vertices.Add(v0)
                    mesh_from_face.Vertices.Add(v1)
                    mesh_from_face.Vertices.Add(v2)
                    mesh_from_face.Faces.AddFace(0, 1, 2)
                    area = Rhino.Geometry.AreaMassProperties.Compute(
                        mesh_from_face, True, False, False, False
                    ).Area

                d = Rhino.Geometry.Vector3d(
                    self.rotation_point.x - centroid.x,
                    self.rotation_point.y - centroid.y,
                    self.rotation_point.z - centroid.z,
                )

                d_perp = d - (d * normal) * normal

                sigma = (
                    math.tan(psi)
                    * Rhino.Geometry.Vector3d.CrossProduct(d_perp, normal).Length
                    / working_face.effective_depth
                ) * working_face.Young_modulus
                if sigma > max_stress:
                    max_stress = sigma
                    location_of_max_stress = centroid
                stress_resultant += geometry.Vector.from_vector_3d(
                    normal * (sigma * area)
                )
                moment_resultant += geometry.Vector.from_vector_3d(
                    Rhino.Geometry.Vector3d.CrossProduct(
                        Rhino.Geometry.Vector3d(
                            centroid.x - self.rotation_point.x,
                            centroid.y - self.rotation_point.y,
                            centroid.z - self.rotation_point.z,
                        ),
                        normal * (sigma * area),
                    )
                )
            working_face.max_bending_stress = max_stress
            working_face.location_of_max_stress = location_of_max_stress
        self.stress_resultant = stress_resultant
        self.moment_resultant = moment_resultant

    def compute_axial_force_stress_distribution(self):
        """
        Computes the stress distribution on the working faces based on the applied axial force and the computed K value.
        """
        resultant_force = geometry.Vector(0, 0, 0)
        for joint_face in self.original_faces:
            if joint_face.axial_subface is None:
                continue
            else:
                working_face = joint_face.axial_subface
            normal = working_face.rh_normal
            area = working_face.area
            alpha = math.acos(
                normal
                * self.axial_force_vector.to_vector_3d()
                / self.axial_force_vector.norm()
            )
            effective_depth = math.sqrt(area)
            E = working_face.Young_modulus
            nomin = (
                -1
                * (self.axial_force_vector.norm() * math.cos(alpha))
                * E
                / effective_depth
            )
            denom = 0.0
            for other_face in self.original_faces:
                if other_face.axial_subface is None:
                    continue
                other_axial_subface = other_face.axial_subface
                other_normal = other_axial_subface.rh_normal
                other_area = other_axial_subface.area
                other_alpha = math.acos(
                    other_normal
                    * self.axial_force_vector.to_vector_3d()
                    / self.axial_force_vector.norm()
                )
                other_E = other_axial_subface.Young_modulus
                other_effective_depth = math.sqrt(other_area)
                if other_effective_depth == 0:
                    print(
                        f"Warning: Effective depth for face {other_face.id} is zero. Skipping contribution to axial stiffness."
                    )
                    continue
                denom += (
                    (math.cos(other_alpha) ** 2)
                    * other_E
                    * other_area
                    / other_effective_depth
                )
            if denom == 0:
                print(
                    f"Warning: Denominator for axial stress calculation for face {joint_face.id} is zero. Skipping axial stress calculation for this face."
                )
                continue
            sigma = nomin / denom
            working_face.axial_stress = sigma
            resultant_force += geometry.Vector.from_vector_3d(normal * (sigma * area))
        self.stress_resultant += resultant_force

    def colorise_mesh_by_stress(self):
        """
        Colors the mesh of each working face based on the computed stress distribution.
        """
        psi = self.moment_vector.norm() / self.k_value

        for joint_face in self.original_faces:
            base_is_axial = False
            if joint_face.axial_subface is not None:
                if joint_face.axial_subface.mesh is None:
                    joint_face.axial_subface.mesh = Rhino.Geometry.Mesh.CreateFromBrep(
                        joint_face.axial_subface.rh_joint_brep_face.Brep,
                        Rhino.Geometry.MeshingParameters.Default,
                    )[0]
                    joint_face.axial_subface.mesh.Subdivide()
                mesh = joint_face.axial_subface.mesh
                base_is_axial = True
            elif joint_face.bending_subface is not None:
                if joint_face.bending_subface.mesh is None:
                    joint_face.bending_subface.mesh = (
                        Rhino.Geometry.Mesh.CreateFromBrep(
                            joint_face.bending_subface.rh_joint_brep_face.Brep,
                            Rhino.Geometry.MeshingParameters.Default,
                        )[0]
                    )
                    joint_face.bending_subface.mesh.Subdivide()
                mesh = joint_face.bending_subface.mesh
            else:
                continue
            normal = joint_face.rh_normal
            mesh.VertexColors.CreateMonotoneMesh(
                Rhino.Display.ColorRGBA(255, 255, 255, 255)
            )
            sigma_max_moment = 0.0
            for mesh_face in mesh.Faces:
                if mesh_face.IsQuad:
                    v0 = mesh.Vertices[mesh_face.A]
                    v1 = mesh.Vertices[mesh_face.B]
                    v2 = mesh.Vertices[mesh_face.C]
                    v3 = mesh.Vertices[mesh_face.D]
                    centroid = geometry.Point(
                        x=(v0.X + v1.X + v2.X + v3.X) / 4,
                        y=(v0.Y + v1.Y + v2.Y + v3.Y) / 4,
                        z=(v0.Z + v1.Z + v2.Z + v3.Z) / 4,
                    )
                else:
                    v0 = mesh.Vertices[mesh_face.A]
                    v1 = mesh.Vertices[mesh_face.B]
                    v2 = mesh.Vertices[mesh_face.C]
                    centroid = geometry.Point(
                        x=(v0.X + v1.X + v2.X) / 3,
                        y=(v0.Y + v1.Y + v2.Y) / 3,
                        z=(v0.Z + v1.Z + v2.Z) / 3,
                    )
                sigma_axial = 0.0
                sigma_moment = 0.0
                if base_is_axial:
                    sigma_axial = joint_face.axial_subface.axial_stress
                if joint_face.bending_subface is not None:
                    closest_point = joint_face.bending_subface.mesh.ClosestPoint(
                        centroid.to_point_3d()
                    )
                    if (
                        closest_point.DistanceTo(centroid.to_point_3d())
                        < 10 * Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance
                    ):
                        d = Rhino.Geometry.Vector3d(
                            self.rotation_point.x - centroid.x,
                            self.rotation_point.y - centroid.y,
                            self.rotation_point.z - centroid.z,
                        )
                        d_perp = d - (d * normal) * normal
                        sigma_moment = (
                            math.tan(psi)
                            * Rhino.Geometry.Vector3d.CrossProduct(
                                d_perp, normal
                            ).Length
                            / joint_face.bending_subface.effective_depth
                        ) * joint_face.bending_subface.Young_modulus
                sigma = sigma_axial + sigma_moment
                if sigma_moment > sigma_max_moment:
                    sigma_max_moment = sigma_moment
                normalized_stress = (
                    sigma / self.overall_max_stress
                    if self.overall_max_stress > 0
                    else 0
                )
                color = Rhino.Display.ColorHSL(
                    1 - normalized_stress,  # Hue
                    1,  # Saturation
                    0.5,  # Lightness
                ).ToArgbColor()

                mesh.VertexColors.SetColor(mesh_face, color)
            self.stress_display_meshes = self.stress_display_meshes or []
            self.stress_display_meshes.append(mesh)

    @property
    def overall_max_stress(self):
        max_stress = 0.0
        for joint_face in self.original_faces:
            if joint_face.cumulated_stresses > max_stress:
                max_stress = joint_face.cumulated_stresses
        return max_stress

    def __post_init__(self):
        for joint_face in self.original_faces:
            joint_face.Young_modulus = utils.compute_young_modulus(
                grain_orientation=self.wood_direction,
                face_normal=geometry.Vector.from_vector_3d(joint_face.rh_normal),
                E0=10e9,  # Example value for E0 in Pascals
                E90=300e6,  # Example value for E90 in Pascals
            )
            text_dot = Rhino.Geometry.TextDot(
                f"E: {joint_face.Young_modulus / 1e9:.2f} GPa",
                joint_face.centroid.to_point_3d(),
            )
            Rhino.RhinoDoc.ActiveDoc.Objects.AddTextDot(text_dot)
        self.detect_moment_working_faces()
        self.detect_axial_force_working_faces()
        self.compute_joint_rigidity()
        self.compute_moment_stress_distribution()
        self.compute_axial_force_stress_distribution()
        self.colorise_mesh_by_stress()
