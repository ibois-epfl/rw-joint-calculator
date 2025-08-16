import Rhino
import time

from joint_calc import face, joint, stress_field, geometry

vectors = []
anchors = []

joint_faces = []
for i, brep_face in enumerate(brep_faces):
    face_area = Rhino.Geometry.AreaMassProperties.Compute(brep_face).Area
    face_center = Rhino.Geometry.AreaMassProperties.Compute(brep_face).Centroid
    joint_face = face.JointFace(
        id=i,
        parent_joint_id=0,
        area=face_area,
        normal=geometry.Vector(brep_face.Faces[0].NormalAt(0.5, 0.5).X, brep_face.Faces[0].NormalAt(0.5, 0.5).Y, brep_face.Faces[0].NormalAt(0.5, 0.5).Z),
        brep_surface=brep_face,
        resultant_location=None
    )
    joint_faces.append(joint_face)

    vectors.append(Rhino.Geometry.Vector3d(joint_face.normal.x/10, joint_face.normal.y/10, joint_face.normal.z/10))

my_joint = joint.Joint(name="Joint1", faces=joint_faces, moment_weights=None)

stress_computer = stress_field.StressFieldComputer(
    moment=geometry.Vector(moment_vector.X, moment_vector.Y, moment_vector.Z),
    rotation_point=geometry.Vector(rotation_center.X, rotation_center.Y, rotation_center.Z),
    joint=my_joint
)
stress_computer.compute_stress_field()

for face in my_joint.faces:
    anchors.append(Rhino.Geometry.Point3d(face.resultant_location.x, face.resultant_location.y, face.resultant_location.z))

print(my_joint.moment_weights)