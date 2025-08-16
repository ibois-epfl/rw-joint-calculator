"""
This module calculates the stress field in a joint based on a moment and a rotation axis.
It assumes a perfectly elastic material behavior (which is not completely accurate with wood)
"""

import geometry, joint, face

import Rhino

import math

def volume_lambda(brep):
    volume = abs(Rhino.Geometry.VolumeMassProperties.Compute(brep).Volume)
    return volume
def surface_lambda(brep):
    return Rhino.Geometry.AreaMassProperties.Compute(brep).Area

TOL = Rhino.RhinoDoc.ActiveDoc.ModelAbsoluteTolerance
class StressFieldComputer:
    def __init__(self, 
                 moment: geometry.Vector, 
                 rotation_point: geometry.Vector,
                 joint: joint.Joint
                 ):
        self.moment = moment
        self.rotation_point = rotation_point
        self.joint = joint

    def compute_stress_field(self):
        """
        The stress field along the faces is modelled as a truncated volume extruded from the face
        """
        stress_volumes = []
        for face in self.joint.faces:
            # Compute the stress distribution on each face
            stress_distribution = self.compute_face_unit_stresses(face)
            stress_volume = volume_lambda(stress_distribution)
            stress_volumes.append(stress_volume)
        total_volume = sum(stress_volumes)
        stress_volumes = [x / total_volume for x in stress_volumes]
        self.joint.moment_weights = stress_volumes
    def compute_face_unit_stresses(self, face: face.JointFace):
        """
        Compute the stress distribution on a single face based on the applied moment and rotation axis.
        """
        interval = Rhino.Geometry.Interval(-10, 10)
        splitting_plane = Rhino.Geometry.Plane(Rhino.Geometry.Point3d(self.rotation_point.x, self.rotation_point.y, self.rotation_point.z), 
                                               Rhino.Geometry.Vector3d(face.normal.x, face.normal.y, face.normal.z),
                                               Rhino.Geometry.Vector3d(self.moment.x, self.moment.y, self.moment.z))
        splitting_brep = Rhino.Geometry.PlaneSurface(splitting_plane, interval, interval).ToBrep()
        success, intersection_curves, intersection_points = Rhino.Geometry.Intersect.Intersection.BrepBrep(splitting_brep, face.brep_surface, TOL)
        split_faces = face.brep_surface.Split(splitting_brep, TOL)
        sorted_split_faces = sorted(split_faces, key=surface_lambda_sorter, reverse=True)
        split_face = sorted_split_faces[0]
        split_face_center = Rhino.Geometry.AreaMassProperties.Compute(split_face).Centroid

        lines = []
        for edge in split_face.Edges:
            line = Rhino.Geometry.Line(edge.PointAtStart, edge.PointAtEnd)
            lines.append(line)
        nurbs_curve = Rhino.Geometry.Polyline.CreateByJoiningLines(lines, TOL, False)[0].ToNurbsCurve()
        brep_volume = Rhino.Geometry.Surface.CreateExtrusion(nurbs_curve, -100 * Rhino.Geometry.Vector3d(face.normal.x, face.normal.y, face.normal.z)).ToBrep()
        brep_volume.Transform(Rhino.Geometry.Transform.Translation(TOL * Rhino.Geometry.Vector3d(face.normal.x, face.normal.y, face.normal.z))) #Because otherwise the split fails...
        splitting_plane_x_axis = Rhino.Geometry.Vector3d(intersection_curves[0].PointAtStart - intersection_curves[0].PointAtEnd)
        splitting_plane_y_axis = Rhino.Geometry.Vector3d(intersection_curves[0].PointAtStart - split_face_center)
        dot_product = Rhino.Geometry.Vector3d.CrossProduct(splitting_plane_y_axis, splitting_plane_x_axis) * Rhino.Geometry.Vector3d(face.normal.x, face.normal.y, face.normal.z)
        print(dot_product)
        splitting_plane = Rhino.Geometry.Plane(intersection_curves[0].PointAtStart,splitting_plane_x_axis, splitting_plane_y_axis)
        if dot_product > 0:
            splitting_plane.Transform(Rhino.Geometry.Transform.Rotation(-math.pi/18, splitting_plane_x_axis, intersection_curves[0].PointAtStart))
        else:
            splitting_plane.Transform(Rhino.Geometry.Transform.Rotation(math.pi/18, splitting_plane_x_axis, intersection_curves[0].PointAtStart))
        splitting_brep = Rhino.Geometry.PlaneSurface(splitting_plane, interval, interval).ToBrep()
        candidate_volumes = brep_volume.Split(splitting_brep, TOL)
        capped_volumes = []
        for candidate in candidate_volumes:
            candidate = candidate.CapPlanarHoles(10*TOL)
            capped_volumes.append(candidate)
        sorted_volumes = sorted(capped_volumes, key=volume_lambda, reverse=False)
        centroid = Rhino.Geometry.AreaMassProperties.Compute(sorted_volumes[0]).Centroid
        resultant_axis_line = Rhino.Geometry.Line(centroid, centroid + Rhino.Geometry.Vector3d(face.normal.x, face.normal.y, face.normal.z))
        intersection_result, curves, points = Rhino.Geometry.Intersect.Intersection.CurveBrep(resultant_axis_line.ToNurbsCurve(), face.brep_surface, TOL)
        face.resultant_location = geometry.Vector(points[0].X, points[0].Y, points[0].Z)
        return sorted_volumes[0]
