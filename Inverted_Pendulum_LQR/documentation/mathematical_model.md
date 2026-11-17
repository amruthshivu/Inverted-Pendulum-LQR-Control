# Mathematical Modelling and Control Design Report
## Optimal Stabilization of an Inverted Pendulum on a Cart Using LQR Control

---

### 1. Physical System Description and Kinematic Configuration

The physical plant under investigation is a classical single-link inverted pendulum mounted on a motorized horizontal cart. The system consists of:
- A rigid cart of mass $M$ free to translate along a 1D horizontal rail under the influence of an external control force $u(t)$ and opposed by linear viscous damping $b \dot{p}$.
- A uniform rigid pendulum rod of mass $m$, total length $L$, center-of-mass (COM) distance $l = L/2$, and moment of inertia $I = \frac{1}{12}mL^2$ about its COM, attached to the cart at a frictionless rotational pivot pin.

#### Coordinate Definitions
- **Cart position $p(t)$:** Horizontal translation from origin, with $p > 0$ to the right.
- **Pendulum angle $\theta(t)$:** Angular displacement measured from the vertically upright unstable equilibrium ($\theta = 0$). Positive clockwise ($\theta > 0$ tilts right).
- **Actuator input $u(t)$:** Horizontal force applied to the cart, positive to the right.

#### Kinematics of Pendulum Center of Mass
The position coordinates of the pendulum center of mass $(x_m, y_m)$ in the inertial frame are:
$$x_m = p + l \sin\theta$$
$$y_m = l \cos\theta$$

Differentiating with respect to time:
$$\dot{x}_m = \dot{p} + l \cos\theta \, \dot{\theta}$$
$$\dot{y}_m = -l \sin\theta \, \dot{\theta}$$

---

### 2. Nonlinear Equations of Motion (Lagrangian Mechanics)

We formulate the system equations using the Euler-Lagrange equations:
$$\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{q}_k}\right) - \frac{\partial \mathcal{L}}{\partial q_k} = \mathcal{Q}_k, \quad q = [p, \theta]^T$$

#### Step 1: Kinetic Energy ($\mathcal{T}$)
$$\mathcal{T} = \mathcal{T}_{\text{cart}} + \mathcal{T}_{\text{pend, trans}} + \mathcal{T}_{\text{pend, rot}}$$
$$\mathcal{T}_{\text{cart}} = \frac{1}{2} M \dot{p}^2$$
$$\mathcal{T}_{\text{pend, trans}} = \frac{1}{2} m \left(\dot{x}_m^2 + \dot{y}_m^2\right) = \frac{1}{2} m \left[ (\dot{p} + l\cos\theta\,\dot{\theta})^2 + (-l\sin\theta\,\dot{\theta})^2 \right] = \frac{1}{2} m \left( \dot{p}^2 + 2 l \dot{p} \dot{\theta} \cos\theta + l^2 \dot{\theta}^2 \right)$$
$$\mathcal{T}_{\text{pend, rot}} = \frac{1}{2} I \dot{\theta}^2$$
$$\mathcal{T} = \frac{1}{2}(M + m)\dot{p}^2 + m l \cos\theta \, \dot{p}\dot{\theta} + \frac{1}{2}(I + m l^2)\dot{\theta}^2$$

#### Step 2: Potential Energy ($\mathcal{V}$)
Taking the pivot horizontal plane as the gravitational datum ($y = 0$):
$$\mathcal{V} = m g y_m = m g l \cos\theta$$

#### Step 3: Lagrangian ($\mathcal{L} = \mathcal{T} - \mathcal{V}$)
$$\mathcal{L} = \frac{1}{2}(M + m)\dot{p}^2 + m l \cos\theta \, \dot{p}\dot{\theta} + \frac{1}{2}(I + m l^2)\dot{\theta}^2 - m g l \cos\theta$$

#### Step 4: Generalized Forces ($\mathcal{Q}_k$)
$$\mathcal{Q}_p = u - b\dot{p}$$
$$\mathcal{Q}_\theta = 0$$

#### Step 5: Lagrange Equations
For coordinate $p$:
$$\frac{\partial \mathcal{L}}{\partial \dot{p}} = (M + m)\dot{p} + m l \cos\theta \, \dot{\theta}$$
$$\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{p}}\right) = (M + m)\ddot{p} + m l \cos\theta \, \ddot{\theta} - m l \sin\theta \, \dot{\theta}^2$$
$$\frac{\partial \mathcal{L}}{\partial p} = 0$$
$$(M + m)\ddot{p} + b\dot{p} + m l \cos\theta \, \ddot{\theta} - m l \sin\theta \, \dot{\theta}^2 = u \quad \text{--- [Eq. 1]}$$

For coordinate $\theta$:
$$\frac{\partial \mathcal{L}}{\partial \dot{\theta}} = (I + m l^2)\dot{\theta} + m l \cos\theta \, \dot{p}$$
$$\frac{d}{dt}\left(\frac{\partial \mathcal{L}}{\partial \dot{\theta}}\right) = (I + m l^2)\ddot{\theta} + m l \cos\theta \, \ddot{p} - m l \sin\theta \, \dot{p}\dot{\theta}$$
$$\frac{\partial \mathcal{L}}{\partial \theta} = -m l \sin\theta \, \dot{p}\dot{\theta} + m g l \sin\theta$$
$$(I + m l^2)\ddot{\theta} + m l \cos\theta \, \ddot{p} - m g l \sin\theta = 0 \quad \text{--- [Eq. 2]}$$

#### Step 6: Decoupled Explicit Form for Numerical Simulation
Writing Eqs. (1) and (2) as a matrix equation:
$$\begin{bmatrix} M + m & m l \cos\theta \\ m l \cos\theta & I + m l^2 \end{bmatrix} \begin{bmatrix} \ddot{p} \\ \ddot{\theta} \end{bmatrix} = \begin{bmatrix} u - b\dot{p} + m l \sin\theta \, \dot{\theta}^2 \\ m g l \sin\theta \end{bmatrix}$$

Let the mass-matrix determinant be:
$$\Delta(\theta) = (M + m)(I + m l^2) - (m l \cos\theta)^2$$
Since $\Delta(\theta) = I(M + m) + M m l^2 + m^2 l^2 \sin^2\theta > 0$, the mass matrix is strictly positive definite and invertible everywhere. Inverting yields the explicit accelerations:
$$\ddot{p} = \frac{(I + m l^2)(u - b\dot{p} + m l \sin\theta \, \dot{\theta}^2) - m^2 l^2 g \sin\theta \cos\theta}{\Delta(\theta)}$$
$$\ddot{\theta} = \frac{-m l \cos\theta (u - b\dot{p} + m l \sin\theta \, \dot{\theta}^2) + (M + m) m g l \sin\theta}{\Delta(\theta)}$$

---

### 3. State-Space Representation and Linearization

Defining state vector:
$$x = \begin{bmatrix} x_1 \\ x_2 \\ x_3 \\ x_4 \end{bmatrix} = \begin{bmatrix} p \\ \dot{p} \\ \theta \\ \dot{\theta} \end{bmatrix}$$

Linearizing about the upright equilibrium $x_e = [0, 0, 0, 0]^T, u_e = 0$, where $\cos\theta \approx 1$, $\sin\theta \approx \theta$, and $\dot{\theta}^2 \approx 0$:
$$\Delta_0 = (M + m)(I + m l^2) - (m l)^2$$

$$\dot{x} = A x + B u, \quad y = C x + D u$$

$$A = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -\frac{(I + m l^2)b}{\Delta_0} & -\frac{m^2 g l^2}{\Delta_0} & 0 \\
0 & 0 & 0 & 1 \\
0 & \frac{m l b}{\Delta_0} & \frac{(M + m)m g l}{\Delta_0} & 0
\end{bmatrix}, \quad
B = \begin{bmatrix}
0 \\
\frac{I + m l^2}{\Delta_0} \\
0 \\
-\frac{m l}{\Delta_0}
\end{bmatrix}$$

#### Controllability and Eigenvalue Properties
For the nominal laboratory parameters ($M=1.0\text{ kg}, m=0.1\text{ kg}, l=0.25\text{ m}, I=0.002083\text{ kg}\cdot\text{m}^2, b=0.1\text{ N}\cdot\text{s/m}, g=9.81\text{ m/s}^2$):
- Open-Loop Poles: $\lambda_1 = 0$, $\lambda_2 = -0.100$, $\lambda_3 = +5.617$, $\lambda_4 = -5.617\text{ rad/s}$.
- The pole at $+5.617\text{ rad/s}$ indicates open-loop saddle instability.
- The controllability matrix $\mathcal{C} = [B, AB, A^2 B, A^3 B]$ satisfies $\text{rank}(\mathcal{C}) = 4$, verifying complete state controllability.

---

### 4. Optimal LQR Controller Design

The Linear Quadratic Regulator minimizes the quadratic cost functional:
$$J = \int_0^\infty \left( x(t)^T Q x(t) + u(t)^T R u(t) \right) dt$$

Where $Q = \text{diag}(q_1, q_2, q_3, q_4) \ge 0$ and $R > 0$.

The optimal control law is given by:
$$u(t) = -K x(t)$$
Where $K$ is determined from the unique positive-definite solution $P$ to the Continuous-time Algebraic Riccati Equation (CARE):
$$A^T P + P A - P B R^{-1} B^T P + Q = 0$$
$$K = R^{-1} B^T P$$

#### Bryson's Rule Weight Formulation
Initial diagonal weights are computed using the inverse-square of maximum acceptable deviations:
$$q_1 = \frac{1}{p_{\max}^2} = \frac{1}{0.5^2} = 4.0\text{ m}^{-2}$$
$$q_2 = \frac{1}{\dot{p}_{\max}^2} = \frac{1}{1.0^2} = 1.0\text{ (m/s)}^{-2}$$
$$q_3 = \frac{1}{\theta_{\max}^2} = \frac{1}{0.2^2} = 25.0\text{ rad}^{-2}$$
$$q_4 = \frac{1}{\dot{\theta}_{\max}^2} = \frac{1}{1.0^2} = 1.0\text{ (rad/s)}^{-2}$$
$$R = \frac{1}{u_{\max}^2} = \frac{1}{10^2} = 0.01\text{ N}^{-2}$$

Solving the CARE produces:
$$K = \begin{bmatrix} -20.0000 & -22.8890 & -113.6308 & -21.1685 \end{bmatrix}$$
And closed-loop poles:
$$\lambda_{CL} = \{-1.660 \pm 0.950j, \ -4.998, \ -31.406\}$$
Confirming exponential asymptotic stability.

---

### 5. Comparative PID Controller Design

The baseline PID controller regulates the pendulum angle $\theta(t)$:
$$e_\theta(t) = \theta_{\text{ref}} - \theta(t) = -\theta(t)$$
$$u_{\text{PID}}(t) = K_p e_\theta(t) + K_i \int_0^t e_\theta(\tau) d\tau + K_d \frac{d e_\theta}{dt}$$

#### Actuator Saturation & Anti-Windup
Real actuators possess finite torque/force capability ($|u| \le F_{\max} = 15\text{ N}$). During large initial errors, actuator saturation occurs. To prevent integrator windup, conditional clamping stops integrator accumulation when saturated in the direction of error:
$$\dot{z}_{\text{int}} = \begin{cases} 0, & \text{if } |u| \ge F_{\max} \text{ and } \text{sign}(u) == \text{sign}(e_\theta) \\ e_\theta, & \text{otherwise} \end{cases}$$

#### SISO PID Limitation: Uncontrolled Cart Drift
A single-loop angle PID controller stabilizes $\theta \to 0$ by pushing the cart beneath the falling rod. However, because $p(t)$ is neither measured nor penalized in the single-loop error signal, the cart will experience indefinite positional drift ($p \to 1.5\text{ m}$, exceeding the rail limits). In contrast, the optimal LQR controller coordinates all 4 state variables simultaneously, stabilizing the pendulum upright while returning the cart to the track origin.
