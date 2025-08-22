# rw-joint-calculator
This repo contains (highly) experimental code to calculate the forces in a roundwood joint such as this one:

<div align="center">

<img src="./assets/2025_05_29_joinery_detail_01_cropped.png" width="55%">

*fig. 1: Roundwood joint base on the one developed by [@petrasvestartas](https://github.com/petrasvestartas)*

</div>


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
    +vector normal
    +double area
}

class Joint{
    +[Face] joint_faces
    +int id
}

class FieldComputer{
    +vector moment
    +vector rotation_center
    +SSCurve stress_strain_relationship
    +tuple get_max_stress()
    +Joint joint
}

class SSCurve{
    +string Material
    +double get_stress(strain)
}

```
