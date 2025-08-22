import Rhino

from joint_calc import face, joint, stress_field, geometry

vectors = []
anchors = []

joint_faces = []
for i, brep_face in enumerate(brep_faces):  # noqa F821
    face_area = Rhino.Geometry.AreaMassProperties.Compute(brep_face).Area
    face_center = Rhino.Geometry.AreaMassProperties.Compute(brep_face).Centroid
    joint_face = face.JointFace(
        id=i,
        parent_joint_id=0,
        area=face_area,
        normal=geometry.Vector.from_vector_3d(brep_face.Faces[0].NormalAt(0.5, 0.5)),
        brep_surface=brep_face,
    )
    joint_faces.append(joint_face)

    vectors.append(
        Rhino.Geometry.Vector3d(
            joint_face.normal.x / 10, joint_face.normal.y / 10, joint_face.normal.z / 10
        )
    )

my_joint = joint.Joint(name="Joint1", faces=joint_faces, moment_weights=None)

stress_computer = stress_field.StressFieldComputer(
    moment=geometry.Vector(
        moment_vector.X,  # noqa F821
        moment_vector.Y,  # noqa F821
        moment_vector.Z,  # noqa F821
    ),
    rotation_point=geometry.Point(
        rotation_center.X,  # noqa F821
        rotation_center.Y,  # noqa F821
        rotation_center.Z,  # noqa F821
    ),
    joint=my_joint,
)
stress_computer.compute_stress_field()
print(
    f"Total tensile component: {round(stress_computer.total_tensile_component) / 1000} kN"
)

for joint_face in my_joint.faces:
    anchors.append(
        Rhino.Geometry.Point3d(
            joint_face.resultant_location.x,
            joint_face.resultant_location.y,
            joint_face.resultant_location.z,
        )
    )
stress_distributions = [joint_face.stress_distribution for joint_face in my_joint.faces]
sigma_maxs = [
    f"sigma_max = {round(joint_face.max_stress / 1e6, 2)} MPa"
    for joint_face in my_joint.faces
]
