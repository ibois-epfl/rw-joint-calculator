import System

import Rhino

from ghpythonlib.componentbase import executingcomponent as component

import joint_calc


class RWJCMomentCalculator(component):
    def RunScript(
        self,
        rh_base_face: Rhino.Geometry.Brep,
        rh_surfaces_elem: System.Collections.Generic.List[Rhino.Geometry.BrepFace],
        rh_rotation_axis: Rhino.Geometry.Line,
    ) -> System.Collections.Generic.List[Rhino.Geometry.Vector3d]:
        success, crv_param = Rhino.Geometry.Intersect.Intersection.CurveBrep(
            rh_rotation_axis,
            rh_base_face,
            Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance,
            Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance,
        )
        if success:
            print(
                f"{len(crv_param)} intersection points found between rotation axis and base face."
            )
            test_point = rh_rotation_axis.PointAt(crv_param[0])
        else:
            raise Exception(
                "No intersection found between rotation axis and base face."
            )

        rotation_axis_vector = joint_calc.geometry.Vector.from_vector_3d(
            Rhino.Geometry.Vector3d(
                rh_rotation_axis.PointAtStart - rh_rotation_axis.PointAtEnd
            )
        )

        jc_joint_faces = []
        for brep_surfaces in rh_surfaces_elem:
            jc_joint_faces.append(
                joint_calc.face.JointFace(
                    id=0, parent_joint_id=0, rh_joint_brep_face=brep_surfaces.Faces[0]
                )
            )

        jc_joint = joint_calc.joint.Joint(
            id=0,
            original_faces=jc_joint_faces,
            moment_vector=rotation_axis_vector,
            rotation_point=joint_calc.geometry.Point.from_Point3d(test_point),
        )

        return jc_joint.working_faces


if __name__ == "__main__":
    component = RWJCMomentCalculator()
    o_working_surfaces = component.RunScript(
        i_base_face,  # noqa: F821
        i_faces_elem,  # noqa: F821
        i_rotation_axis,  # noqa: F821
    )
