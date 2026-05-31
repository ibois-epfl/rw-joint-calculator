"""
This is a module to store joints with their faces
"""

import Rhino

from dataclasses import dataclass

from . import face, geometry, utils


@dataclass
class Joint:
    id: int
    original_faces: list[face.JointFace]
    moment_vector: geometry.Vector
    rotation_point: geometry.Point
    working_faces: list[face.JointFace] = None
    inertia_along_moment_axis: geometry.Vector = None

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
                    oriented_normal = r.Faces[0].NormalAt(0, 0)
                    Rhino.RhinoDoc.ActiveDoc.Objects.AddPoint(
                        subface_centroid.to_point_3d()
                    )
                    rot_point_to_centroid = Rhino.Geometry.Vector3d(
                        subface_centroid.x - self.rotation_point.x,
                        subface_centroid.y - self.rotation_point.y,
                        subface_centroid.z - self.rotation_point.z,
                    )
                    moment_participation = Rhino.Geometry.Vector3d.CrossProduct(
                        rot_point_to_centroid, oriented_normal
                    )
                    if moment_participation * self.moment_vector.to_vector_3d() < 0:
                        self.working_faces.append(r)
            else:
                centroid = joint_face.centroid
                Rhino.RhinoDoc.ActiveDoc.Objects.AddPoint(centroid.to_point_3d())
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
                    self.working_faces.append(joint_face.rh_joint_brep_face)

    def __post_init__(self):
        # for joint_face in self.original_faces:
        #     if joint_face.rh_normal * self.moment_vector.to_vector_3d() < 0:
        #         print("Inverting normal vector.")
        #         joint_face.rh_normal *= -1

        self.detect_working_faces()

        # Calculate total inertia along moment axis
        # for joint_face in self.working_faces:
        #     inertia = joint_face.compute_inertia(
        #         Rhino.Geometry.Plane(
        #             self.rotation_point.to_point_3d(),
        #             self.moment_vector.to_vector_3d(),
        #             joint_face.rh_normal,
        #         )
        #     )
        #     if self.inertia_along_moment_axis is None:
        #         self.inertia_along_moment_axis = inertia.z
        #     else:
        #         self.inertia_along_moment_axis += inertia.z
