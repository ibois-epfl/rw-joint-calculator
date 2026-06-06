# analytical analysis of roundwood joint

## Moment-induced stresses

![illustration](./assets/2026_06_01_joint_drawing_with_annotations.png)

Under rotation around the m vector, the stresses $\sigma$ on a $dA$ element surface participate to the moment around $\vec{m}$ by:
$$M = \sum_i \int_{\Omega_i} \sigma * dA * (\vec{d} \times \vec{n}) \cdot \frac{\vec{m}}{||\vec{m}||} $$

Where:
- $\vec{n}$ is the unit normal vector to the area element $dA$.
- $\Omega_i$ is the joint face that can actively participate the the moment by contact forces only (no glue, so no tension is allowed on the joint faces). Tension forces are only taken by a screw passing by $\vec{m}$, and will be computed later.

Also, by a rotation of $\psi$ around the m axis, the stresses on a joint face can be expressed as:

$$ \sigma = \frac{\tan(\psi) || \vec{d}\times\vec{n} ||}{L} * E $$

Where L is the effective depth of wood considered compressed by the stresses on the face.

Due to the angle of the joint faces with respect to the wood fibres, the Young modulus will not be identical for all faces. If we inject the equation above in the first equation:
$$M = \sum_i \int_{\Omega_i} \tan(\psi) || \vec{d}\times\vec{n} || * dA * (\vec{d} \times \vec{n}) \cdot \frac{\vec{m}}{||\vec{m}||} * \frac{E_i}{L_i}$$

Assuming the Young modulus and the effective depth are constant over the surface:
$$M = \tan(\psi) \sum_i \frac{E_i}{L_i} \int_{\Omega_i} || \vec{d}\times\vec{n} || * dA * (\vec{d} \times \vec{n}) \cdot \frac{\vec{m}}{||\vec{m}||}$$

If we separate $\vec{d}$ into a perpendicular and parallel component to $\vec{n}$: $\vec{d} = \vec{d_{\perp}} + \vec{d_{\parallel}}$

$$M = \tan(\psi) \sum_i \frac{E_i}{L_i} \int_{\Omega_i} || (\vec{d_{\perp}} + \vec{d_{\parallel}}) \times \vec{n} || * dA *  ((\vec{d_{\perp}} + \vec{d_{\parallel}}) \times \vec{n} ) \cdot \frac{\vec{m}}{||\vec{m}||}$$

$$M = \tan(\psi) \sum_i \frac{E_i}{L_i} \int_{\Omega_i} || \vec{d_{\perp}}\times \vec{n} || * dA * (\vec{d_{\perp}}\times \vec{n} )\cdot \frac{\vec{m}}{||\vec{m}||}$$

$$M = \tan(\psi) \sum_i \frac{E_i}{L_i} \int_{\Omega_i} || \vec{d_{\perp}} || * ||\vec{d_{\perp}}|| * \cos(\widehat{(\vec{d}\times \vec{n}), \vec{m}}) * dA $$

Lastly, assuming the rotation is sufficiently small for $\psi \approx \tan(\psi)$ (with $\psi$ in radiants).
We also call $\theta$ the angle between $(\vec{d_{\perp}}\times \vec{n})$ and $\vec{m}$
$$M = \psi \sum_i \frac{E_i}{L_i} \int_{\Omega_i} || \vec{d_{\perp}} || ^2 * \cos(\theta) * dA $$

This expression depends on $L_i$, the effective depths taken into account for the deformation of the joint faces. It is not a value we can determine a-priori, and it determines linearly the value of $\psi$ for a given moment. A first-order estimate is to assume $L_i$ is proportional to the square root of the working area: $L_i = \alpha \sqrt{A_i}$, following a Saint-Venant assumption: since the joint faces are small compared to the timber piece, at a gicen depth the loads on that face will have diffused enough that we can start to  neglect deformations at deeper depths. In our case we need to calibrate $\alpha$ experimentally, but as a first estimation we will take $\alpha = 1$, meaning we assume constant stresses at a depth of up to $\sqrt{A_i}$, then 0 deeper than that.

The expression also depends on $\theta$, which is not constant on $\Omega_i$. One option is to use the value at the face centroid and thus be able to extract the $\cos(\theta)$ from the integral, assuming $\theta$ is sufficiently homogeneous over the face. This assumption is very dependent on joint geometry and should be systematically validated for a given geometry. Another is to discretize the surface and evaluate the integral as a Riemann sum.

The value of $E_i$ will depend on fibre orientation on the joint face and according to [1](https://doi.org/10.1007/978-3-030-81315-4) (p405) it can be axpressed from $E_{\perp}$ and $E_{\parallel}$ as:

$$ E_{\gamma} = \frac{E_{\parallel} * E_{\perp}}{E_{\parallel} * \sin(\gamma)^2 + E_{\perp} * \cos(\gamma)^2} $$

With $\gamma$ the angle between $\vec{n}$ and the wood fibre, which can be approximated with the beam axis.


## Axial-force-induced stresses

Assuming a single displacement vector $\vec{d}$ is applied to the faces, we can state that the stresses on face i are related to that global displacement $\vec{d}$ by the relation:

$$ \sigma_i = ||\vec{d}|| * \cos{\alpha}_i * \frac{E_i}{L_i} $$
Where $\alpha_i$ is the angle between the normal to face i and the displacement vector $\vec{d}$.
With the total force on the face i $\vec{F_i}$:
$$ \vec{F_i} = \sigma_i * A_i * \vec{n_i} $$

We thus have the relation between the applied force $\vec{F}$ and the individual face forces $\vec{F_i}$:

$$ ||\vec{F}|| = \sum_i{\sigma_i * A_i * \cos{\alpha_i}} $$
$$ ||\vec{F}|| = ||\vec{d}|| \sum_i{\frac{E_i}{L_i}* A_i * \cos^2{\alpha_i}} $$
$$ ||\vec{d}|| = \frac{||\vec{F}||}{\sum_i{\frac{E_i}{L_i}* A_i * \cos^2{\alpha_i}}} $$
And therefore:
$$ \sigma_i = \frac{||\vec{F}|| * \cos{\alpha_i} * \frac{E_i}{L_i}}{\sum_j{\frac{E_j}{L_j} * A_j * \cos^2{\alpha_j}}} $$
