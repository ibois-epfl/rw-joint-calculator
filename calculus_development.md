# analytical analysis of roundwood joint

![illustration](./assets/2026_06_01_joint_drawing_with_annotations.png)

Under rotation around the m vector, the stresses $\sigma$ on a $dA$ element surface participate to the moment around $\vec{m}$ by:
$$M = \sum_i \int_{\Omega_i} \sigma * dA * (\vec{n} \times \vec{d}) \cdot \frac{\vec{m}}{||\vec{m}||} $$

Also, by a rotation of $\psi$, the stresses can be expressed as:

$$ \sigma = \tan(\psi) || \vec{d}\times\vec{n} || * E $$
NOTE: no $\cdot \frac{\vec{m}}{||\vec{m}||}$ ?

So :
$$M = \sum_i \int_{\Omega_i} \tan(\psi) || \vec{d}\times\vec{n} || * dA * ||(\vec{n} \times \vec{d}) \cdot \frac{\vec{m}}{||\vec{m}||}|| * E_i$$

$$M = \tan(\psi) \sum_i E_i \int_{\Omega_i} || \vec{d}\times\vec{n} || * dA * ||(\vec{n} \times \vec{d}) \cdot \frac{\vec{m}}{||\vec{m}||}||$$

If we separate $\vec{d}$ into a perpendicular and parallel component to $\vec{n}$: $\vec{d} = \vec{d_{\perp}} + \vec{d_{\parallel}}$

$$M = \tan(\psi) \sum_i E_i \int_{\Omega_i} || \vec{n}\times(\vec{d_{\perp}} + \vec{d_{\parallel}}) || * dA * ||(\vec{n} \times (\vec{d_{\perp}} + \vec{d_{\parallel}})) \cdot \frac{\vec{m}}{||\vec{m}||}||$$

$$M = \tan(\psi) \sum_i E_i \int_{\Omega_i} || \vec{n}\times\vec{d_{\parallel}} || * dA * ||(\vec{n} \times \vec{d_{\parallel}})\cdot \frac{\vec{m}}{||\vec{m}||}||$$

$$M = \tan(\psi) \sum_i E_i \int_{\Omega_i} || \vec{d_{\parallel}} || * ||\vec{d_{\parallel}}|| * \cos(\widehat{(\vec{m}\times \vec{d}), \vec{m}}) * dA $$