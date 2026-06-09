"""computes the stresses in a roundwood joint created by a moment"""

import System

import Rhino

from ghpythonlib.componentbase import executingcomponent as component

from joint_calc import face, joint, geometry


class RWJCMomentCalculator(component):
    def RunScript(
        self,
        brep_faces: System.Collections.Generic.List[Rhino.Geometry.Brep],
        moment_vector: Rhino.Geometry.Vector3d,
        anchor_point: Rhino.Geometry.Point3d,
        wood_direction: Rhino.Geometry.Vector3d,
        axial_force: Rhino.Geometry.Vector3d,
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
            axial_force_vector=geometry.Vector.from_vector_3d(axial_force),
            rotation_point=geometry.Point.from_Point3d(anchor_point),
            wood_direction=geometry.Vector.from_vector_3d(wood_direction),
        )

        working_face_breps = [
            joint_face.bending_subface.rh_joint_brep_face
            for joint_face in my_joint.original_faces
            if joint_face.bending_subface is not None
        ]
        max_stresses = []
        Es = []
        max_stress_locations = []
        for joint_face in my_joint.original_faces:
            max_stress = joint_face.cumulated_stresses
            if joint_face.bending_subface is not None:
                max_stress_location = joint_face.bending_subface.location_of_max_stress
                max_stress_locations.append(max_stress_location)
                Es.append(joint_face.Young_modulus)
                max_stresses.append(max_stress)
            elif joint_face.axial_subface is not None:
                max_stress_location = joint_face.axial_subface.centroid
                max_stress_locations.append(max_stress_location)
                Es.append(joint_face.Young_modulus)
                max_stresses.append(max_stress)
        text_dots = [
            Rhino.Geometry.TextDot(
                f"Max Stress: {max_stress / 1e6:.2f} MPa and E: {E / 1e9:.2f} GPa",
                location.to_point_3d(),
            )
            for max_stress, E, location in zip(max_stresses, Es, max_stress_locations)
        ]
        resultant_force = my_joint.stress_resultant.to_vector_3d()
        resultant_moment = my_joint.moment_resultant.to_vector_3d()
        meshes = my_joint.stress_display_meshes
        return [
            working_face_breps,
            text_dots,
            resultant_force,
            resultant_moment,
            meshes,
        ]


# if __name__ == "__main__":
#     component = RWJCMomentCalculator()
#     working_face_breps, text_dots, resultant_force, resultant_moment, meshes = component.RunScript(
#         brep_faces,
#         moment_vector,
#         anchor_point,
#         wood_direction,
#         axial_force,
#     )
