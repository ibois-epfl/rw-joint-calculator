# rw-joint-calculator
This repo contains (highly) experimental code to calculate the forces in a roundwood joint such as this one:

<center>
<img src="./assets/2025_08_22_joinery_detail_and_stresses.png" width="75%">


</center>

> [!WARNING]
> The current implementation does not take the screw into account (or rather assumes a central screw that takes all the tensile stresses perfectly in it axis). When that is fixed this warning will be removed :)

> [!WARNING]
> The current implementation only calculates the stresse resulting from moments, not axial stresses.

> [!WARNING]
> This repo has NOT been validated by mechanical tests, and the results might be very wrong, so do not use this for any other purpose than experimental research

## class diagram
This is just a small class diagram to explain easily the logic followed

```mermaid
classDiagram
class Face{
    +int id
    +int parent_joint_id
    +double area
    +Vector normal
    +Rhino.Geometry.Brep joint_surface
    +Point resultant_location
    +Rhino.Geometry.Brep stress_distribution
    +double max_stress
}

class Joint{
    +string name
    +[Face] joint_faces
    +[float] face_weights
}

class StressFieldComputer{
    +vector moment
    +vector rotation_center
    +Joint joint
    +void compute_stress_field()
}

class Vector{
    +double x
    +double y
    +double z
    +Rhino.Geometry.Vector3d to_vector_3d()
    +Vector from_vector_3d()
    +double norm()
    +double compute_angle_with()
    +bool is_parallel_to()
}

class Point{
    +double x
    +double y
    +double z
    +Rhino.Geometry.Point3d to_point_3d()
}

```
