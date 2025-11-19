import System

import Rhino

from ghpythonlib.componentbase import executingcomponent as component


class RWJCMomentCalculator(component):
    def RunScript(
        self,
        rh_base_face: Rhino.Geometry.Brep,
        rh_faces_elem: System.Collections.Generic.List[Rhino.Geometry.BrepFace],
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

        planes = []
        anchors = []
        normals = []
        working_surfaces = []

        rotation_avis_vector = Rhino.Geometry.Vector3d(
            rh_rotation_axis.PointAtStart - rh_rotation_axis.PointAtEnd
        )

        for face in rh_faces_elem:
            brep_face = face.Faces[0]
            anchor = Rhino.Geometry.AreaMassProperties.Compute(face).Centroid
            normal = brep_face.NormalAt(0, 0)
            if normal * rotation_avis_vector < 0:
                print("Inverting normal vector.")
                normal *= -1
            normals.append(normal)
            plane = Rhino.Geometry.Plane(test_point, rotation_avis_vector, normal)
            success, crvs, pts = Rhino.Geometry.Intersect.Intersection.BrepPlane(
                face, plane, Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance
            )
            if success and crvs:
                curve = crvs[0]
                planes.append(plane)
                candidate_surfaces = face.Split(
                    [curve], Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance
                )
                candidate_surfaces_centroids = [
                    Rhino.Geometry.AreaMassProperties.Compute(srf).Centroid
                    for srf in candidate_surfaces
                ]
                for idx, centroid in enumerate(candidate_surfaces_centroids):
                    if (
                        Rhino.Geometry.Vector3d.CrossProduct(
                            normal, Rhino.Geometry.Vector3d(centroid - test_point)
                        )
                        * rotation_avis_vector
                        < 0
                    ):
                        working_surfaces.append(candidate_surfaces[idx])

            else:
                if (
                    Rhino.Geometry.Vector3d.CrossProduct(
                        normal, Rhino.Geometry.Vector3d(anchor - test_point)
                    )
                    * rotation_avis_vector
                    < 0
                ):
                    working_surfaces.append(face)
                planes.append(plane)
                print("No intersection found between face and plane.")

            anchors.append(anchor)
        return planes, anchors, normals, working_surfaces


if __name__ == "__main__":
    component = RWJCMomentCalculator()
    o_planes, o_anchors, o_normals, o_working_surfaces = component.RunScript(
        i_base_face,  # noqa: F821
        i_faces_elem,  # noqa: F821
        i_rotation_axis,  # noqa: F821
    )
