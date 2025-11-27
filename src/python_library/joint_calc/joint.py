"""
This is a module to store joints with their faces
"""

import Rhino

from dataclasses import dataclass

import joint_calc


@dataclass
class Joint:
    id: int
    original_faces: list[joint_calc.face.JointFace]
    moment_vector: joint_calc.geometry.Vector
    rotation_point: joint_calc.geometry.Point
    working_faces: list[joint_calc.face.JointFace] = None
    inertia_along_moment_axis: joint_calc.geometry.Vector = None

    def detect_working_faces(self):
        self.working_faces = []
        for joint_face in self.original_faces:
            plane = Rhino.Geometry.Plane(
                self.rotation_point.to_point_3d(),
                self.moment_vector.to_vector_3d(),
                joint_face.rh_normal,
            )

            success, rh_crvs, rh_pts = Rhino.Geometry.Intersect.Intersection.BrepPlane(
                joint_face.rh_joint_brep_face.Brep,
                plane,
                Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance,
            )

            if success and rh_crvs:
                rh_curve = rh_crvs[0]
                rh_candidate_surfaces = joint_face.rh_joint_brep_face.Brep.Split(
                    [rh_curve],
                    joint_face.rh_normal,
                    False,
                    Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance,
                )
                candidate_surfaces_centroids = [
                    joint_calc.utils.compute_centroid(srf)
                    for srf in rh_candidate_surfaces
                ]
                for idx, centroid in enumerate(candidate_surfaces_centroids):
                    if (
                        Rhino.Geometry.Vector3d.CrossProduct(
                            joint_face.rh_normal,
                            Rhino.Geometry.Vector3d(
                                centroid.to_point_3d()
                                - self.rotation_point.to_point_3d()
                            ),
                        )
                        * self.moment_vector.to_vector_3d()
                        < 0
                    ):
                        self.working_faces.append(
                            joint_calc.face.JointFace(
                                id=joint_face.id,
                                parent_joint_id=joint_face.parent_joint_id,
                                rh_joint_brep_face=rh_candidate_surfaces[idx].Faces[0],
                            )
                        )

            else:
                if (
                    Rhino.Geometry.Vector3d.CrossProduct(
                        joint_face.rh_normal,
                        Rhino.Geometry.Vector3d(
                            joint_face.centroid.to_point_3d()
                            - self.rotation_point.to_point_3d()
                        ),
                    )
                    * self.moment_vector.to_vector_3d()
                    < 0
                ):
                    self.working_faces.append(
                        joint_calc.face.JointFace(
                            id=joint_face.id,
                            parent_joint_id=joint_face.parent_joint_id,
                            rh_joint_brep_face=joint_face.rh_joint_brep_face,
                        )
                    )

    def __post_init__(self):
        for joint_face in self.original_faces:
            if joint_face.rh_normal * self.moment_vector.to_vector_3d() < 0:
                print("Inverting normal vector.")
                joint_face.rh_normal *= -1

        self.detect_working_faces()

        # Calculate total inertia along moment axis
        for joint_face in self.working_faces:
            inertia = joint_face.compute_inertia(
                Rhino.Geometry.Plane(
                    self.rotation_point.to_point_3d(),
                    self.moment_vector.to_vector_3d(),
                    joint_face.rh_normal,
                )
            )
            if self.inertia_along_moment_axis is None:
                self.inertia_along_moment_axis = inertia.z
            else:
                self.inertia_along_moment_axis += inertia.z
