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
    rotation_point: geometry.Point
    wood_direction: geometry.Vector
    working_faces: list[face.JointFace] = None
    inertia_along_moment_axis: geometry.Vector = None
    k_value: float = None

    def detect_working_faces(self):
        self.working_faces = []
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
                        wf = face.JointFace(
                            id=joint_face.id,
                            parent_joint_id=self.id,
                            rh_joint_brep_face=r.Faces[0],
                        )
                        self.working_faces.append(wf)
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
                    self.working_faces.append(joint_face)

    def compute_joint_rigidity(self):
        """
        Computes the K value of the joint based on the working faces and their inertia along the moment axis.
        """
        K = 0.0
        for working_face in self.working_faces:
            mesh = Rhino.Geometry.Mesh.CreateFromBrep(
                working_face.rh_joint_brep_face.Brep,
                Rhino.Geometry.MeshingParameters.Default,
            )[0]
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
            E = utils.compute_young_modulus(
                grain_orientation=self.wood_direction,
                face_normal=geometry.Vector.from_vector_3d(working_face.rh_normal),
                E0=10e9,  # Example value for E0 in Pascals
                E90=300e6,  # Example value for E90 in Pascals
            )
            working_face.Young_modulus = E
            L = math.sqrt(working_face.area)
            working_face.effective_depth = L

            K += E * Riemann_sum / L
        self.k_value = K

        print(f"Computed K value for Joint {self.id}: {K:.2f} Nm/radians")
        # text_dot = Rhino.Geometry.TextDot(f"K value: {K:.2f}\nE:kNm / radians", self.rotation_point.to_point_3d())
        # Rhino.RhinoDoc.ActiveDoc.Objects.AddTextDot(text_dot)

    def __post_init__(self):
        self.detect_working_faces()
        self.compute_joint_rigidity()
