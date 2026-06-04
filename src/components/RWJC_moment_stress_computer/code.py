import System

import Rhino

from ghpythonlib.componentbase import executingcomponent as component

from joint_calc import face, joint, geometry


class RWJCMomentCalculator(component):
    def RunScript(
        self,
        brep_faces: System.Collections.Generic.List[Rhino.Geometry.BrepFace],
        moment_vector: Rhino.Geometry.Vector3d,
        anchor_point: Rhino.Geometry.Point3d,
        wood_direction: Rhino.Geometry.Vector3d,
    ):
        joint_faces = []
        for i, brep_face in enumerate(brep_faces):
            joint_face = face.JointFace(
                id=i,
                parent_joint_id=0,
                rh_joint_brep_face=brep_face.Faces[0],
            )
            joint_faces.append(joint_face)

        my_joint = joint.Joint(
            id=0,
            original_faces=joint_faces,
            moment_vector=geometry.Vector.from_vector_3d(moment_vector),
            rotation_point=geometry.Point.from_Point3d(anchor_point),
            wood_direction=geometry.Vector.from_vector_3d(wood_direction),
        )
        return my_joint


if __name__ == "__main__":
    c = RWJCMomentCalculator()
    catch = c.RunScript(brep_faces, moment_vector, anchor_point, wood_direction)  # noqa
    # catch = [working_face.rh_joint_brep_face for working_face in catch]
    max_stresses = [working_face.max_stress for working_face in catch.working_faces]
    max_stress_locations = [
        working_face.location_of_max_stress for working_face in catch.working_faces
    ]
    text_dots = [
        Rhino.Geometry.TextDot(
            f"Max Stress: {max_stress / 1e6:.2f} MPa", location.to_point_3d()
        )
        for max_stress, location in zip(max_stresses, max_stress_locations)
    ]
    working_face_breps = [
        working_face.rh_joint_brep_face for working_face in catch.working_faces
    ]
    resultant = catch.stress_resultant.to_vector_3d()
    meshes = [working_face.mesh for working_face in catch.working_faces]
