import System

import Rhino

from ghpythonlib.componentbase import executingcomponent as component

from joint_calc import face, joint, stress_field, geometry


class RWJCStressComputer(component):
    def RunScript(
        self,
        brep_faces: System.Collections.Generic.List[Rhino.Geometry.BrepFace],
        moment_vector: Rhino.Geometry.Vector3d,
        rotation_center: Rhino.Geometry.Point3d,
        axial_force: Rhino.Geometry.Vector3d,
        screw_axis: Rhino.Geometry.Vector3d,
    ):
        vectors = []
        anchors = []

        joint_faces = []
        for i, brep_face in enumerate(brep_faces):
            face_area = Rhino.Geometry.AreaMassProperties.Compute(brep_face).Area
            joint_face = face.JointFace(
                id=i,
                parent_joint_id=0,
                area=face_area,
                normal=geometry.Vector.from_vector_3d(
                    brep_face.Faces[0].NormalAt(0.5, 0.5)
                ),
                brep_surface=brep_face,
            )
            joint_faces.append(joint_face)

            vectors.append(
                Rhino.Geometry.Vector3d(
                    joint_face.normal.x / 10,
                    joint_face.normal.y / 10,
                    joint_face.normal.z / 10,
                )
            )

        my_joint = joint.Joint(name="Joint1", faces=joint_faces, moment_weights=None)

        stress_computer = stress_field.StressFieldComputer(
            moment=geometry.Vector(
                moment_vector.X,
                moment_vector.Y,
                moment_vector.Z,
            ),
            rotation_point=geometry.Point(
                rotation_center.X,
                rotation_center.Y,
                rotation_center.Z,
            ),
            joint=my_joint,
            axial_force=geometry.Vector(
                axial_force.X,
                axial_force.Y,
                axial_force.Z,
            ),
            screw_axis=geometry.Vector(
                screw_axis.X,
                screw_axis.Y,
                screw_axis.Z,
            ),
        )
        stress_computer.compute_stress_field()
        print(
            f"Total tensile component: {round(stress_computer.total_tensile_component) / 1000} kN"
        )

        for joint_face in my_joint.faces:
            if joint_face.is_stressed:
                anchors.append(
                    Rhino.Geometry.Point3d(
                        joint_face.resultant_location.x,
                        joint_face.resultant_location.y,
                        joint_face.resultant_location.z,
                    )
                )
            else:
                anchors.append(None)
        stress_distributions = [
            joint_face.stress_distribution for joint_face in my_joint.faces
        ]
        sigma_maxs = [
            f"sigma_max = {round(joint_face.max_stress / 1e6, 2)} MPa"
            for joint_face in my_joint.faces
        ]

        return [vectors, anchors, stress_distributions, sigma_maxs]


# if __name__ == "__main__":
#     component = RWJCStressComputer()
#     vectors, anchors, stress_distributions, sigma_maxs = component.RunScript(brep_faces, moment_vector, rotation_center, axial_force, screw_axis)
