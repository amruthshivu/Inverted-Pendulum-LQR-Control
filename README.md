# Optimal Stabilization of an Inverted Pendulum Using LQR Control – Interactive Simulator

An academic Control Systems Engineering project in MATLAB featuring mathematical modelling, nonlinear continuous-time dynamics, modern state-space optimal control (LQR), baseline PID benchmarking, a 9-point engineering verification suite, and an interactive animated simulator with dynamic parameter tuning.

---

## 📌 Project Overview

The inverted pendulum on a cart is a benchmark problem in modern control theory representing an underactuated, inherently unstable, nonlinear dynamic system with non-minimum phase characteristics.

This repository provides:
1. **Full Nonlinear Lagrangian Dynamics:** Coupled equations of motion without early linear approximations.
2. **State-Space Linearization:** Analytical matrices validated against numerical central-difference Jacobians down to machine precision ($7.58 \times 10^{-12}$).
3. **Continuous Algebraic Riccati Equation (CARE) Solver:** Optimal LQR synthesis based on Bryson's rule weightings.
4. **Comparative Baseline PID Controller:** SISO angle regulation with derivative filtering and actuator anti-windup clamping.
5. **Interactive Animated MATLAB Simulator:** Programmatic GUI (`uifigure`) featuring a 2D kinematic canvas, live $Q$ and $R$ tuning sliders, initial condition configuration, dynamic response plots, and performance telemetry.
6. **Engineering Validation Suite:** 9 automated tests verifying physical parameters, equilibrium derivatives, controllability, closed-loop stability, ODE convergence, and the nonlinear basin of attraction.

---

## 🔬 System Dynamics & State-Space Model

### State Vector
$$x = \begin{bmatrix} p \\ \dot{p} \\ \theta \\ \dot{\theta} \end{bmatrix} \begin{matrix} \text{(cart position [m])} \\ \text{(cart velocity [m/s])} \\ \text{(pendulum angle [rad])} \\ \text{(pendulum angular velocity [rad/s])} \end{matrix}$$

### Nonlinear Equations of Motion
$$(M + m)\ddot{p} + b\dot{p} + m l \cos\theta \, \ddot{\theta} - m l \sin\theta \, \dot{\theta}^2 = u$$
$$(I + m l^2)\ddot{\theta} + m l \cos\theta \, \ddot{p} - m g l \sin\theta = 0$$

Where:
- $M = 1.0\text{ kg}$ (cart mass)
- $m = 0.1\text{ kg}$ (pendulum rod mass)
- $L = 0.5\text{ m}, \ l = L/2 = 0.25\text{ m}$ (length and COM distance)
- $I = \frac{1}{12}mL^2 = 0.002083\text{ kg}\cdot\text{m}^2$ (rod inertia about COM)
- $b = 0.1\text{ N}\cdot\text{s/m}$ (viscous cart friction)
- $g = 9.81\text{ m/s}^2$ (gravitational acceleration)
- $F_{\max} = \pm 15.0\text{ N}$ (actuator saturation limit)

### Linearized State-Space Matrices ($x_e = 0, u_e = 0$)
$$\dot{x} = A x + B u, \quad y = C x + D u$$

$$A = \begin{bmatrix}
0 & 1 & 0 & 0 \\
0 & -0.0975 & -0.7178 & 0 \\
0 & 0 & 0 & 1 \\
0 & 0.2927 & 31.5835 & 0
\end{bmatrix}, \quad
B = \begin{bmatrix}
0 \\
0.9756 \\
0 \\
-2.9268
\end{bmatrix}$$

- **Open-Loop Poles:** $\lambda = \{0, \ -0.100, \ +5.617, \ -5.617\}\text{ rad/s}$ (Unstable Saddle).
- **Controllability:** $\text{rank}(\mathcal{C}) = 4$ of $4$ (Fully Controllable).

---

## 📊 Controller Performance Comparison (LQR vs PID)

Evaluated under an identical $10.0^\circ$ disturbance ($p_0 = 0\text{ m}, F_{\max} = \pm 15\text{ N}$):

| Metric | Optimal LQR | Baseline PID | Control-Theoretic Discussion |
| :--- | :---: | :---: | :--- |
| **Stabilization Status** | **STABILIZED** | **STABILIZED** | Both arrest fall and restore upright posture. |
| **2% Settling Time ($T_s$)** | **$0.930\text{ s}$** | **$0.330\text{ s}$** | PID angle transient is fast, but causes massive cart drift. |
| **Peak Angle Deviation** | $10.00^\circ$ | $10.00^\circ$ | Smooth monotonic recovery. |
| **Steady-State Error** | $< 10^{-3 \circ}$ | $0.0648^\circ$ | LQR achieves zero steady-state error. |
| **Max Cart Displacement** | **$0.143\text{ m}$** | **$1.496\text{ m}$** | **LQR confines cart to $15\text{ cm}$. PID drifts $1.5\text{ m}$!** |
| **Final Cart Position ($t_f$)** | **$-0.0000\text{ m}$** | **$+1.4964\text{ m}$** | **LQR returns cart to datum. PID leaves cart offset.** |
| **Track Limit Breached ($>1\text{m}$)**| **NO** | **YES** | **PID exceeds laboratory track length limit.** |
| **Integrated Control Effort** | $5.918\text{ N}^2\cdot\text{s}$ | $3.516\text{ N}^2\cdot\text{s}$ | Modest additional effort for dual position regulation. |

> **Key Takeaway:** A SISO PID controller regulates only angle $\theta(t)$, leaving cart position $p(t)$ as an uncontrolled drift mode. Optimal LQR solves this via full-state feedback by penalizing both cart displacement and pendulum angle in the quadratic cost functional.

---

## 🚀 Quick Start & MATLAB Execution

### 1. Launch the Master Menu
In MATLAB:
```matlab
cd('c:/path/to/CSD');
main
```

### 2. Direct Execution Shortcuts
- **Interactive Simulator GUI:**
  ```matlab
  main('app')
  ```
- **Benchmark Controller Comparison:**
  ```matlab
  main('compare')
  ```
- **LQR $Q$ & $R$ Sensitivity Sweep:**
  ```matlab
  main('tune')
  ```
- **Engineering Validation Audit:**
  ```matlab
  main('validate')
  ```

---

## 📁 Repository File Structure

```
.
├── main.m                                  # Root launcher
├── Inverted_Pendulum_LQR/
│   ├── main.m                              # Master menu launcher
│   ├── config/
│   │   ├── parameters.m                    # Plant parameters
│   │   └── controller_settings.m           # Nominal tuning settings
│   ├── models/
│   │   ├── nonlinear_dynamics.m            # Nonlinear equations of motion
│   │   ├── linear_state_space.m            # Analytical state-space matrices
│   │   └── linearize_model.m               # Numerical Jacobian verification
│   ├── controllers/
│   │   ├── design_lqr.m                    # CARE solver & LQR gain calculation
│   │   ├── pid_controller.m                # Angle PID with anti-windup
│   │   └── calculate_control_input.m       # Actuator dispatcher with saturation
│   ├── simulation/
│   │   ├── run_simulation.m                # ode45 simulation engine
│   │   ├── simulation_rhs.m                # Closed-loop ODE RHS
│   │   └── calculate_metrics.m             # Performance metric evaluator
│   ├── interface/
│   │   ├── create_simulator_app.m          # Interactive MATLAB App (uifigure)
│   │   ├── update_animation.m              # 2D kinematic visualizer
│   │   └── update_plots.m                  # Dynamic multi-channel response plots
│   ├── analysis/
│   │   ├── compare_controllers.m           # LQR vs PID benchmark script
│   │   ├── q_r_sensitivity.m               # Parametric Q & R sensitivity study
│   │   └── validation_tests.m              # 9-point quality audit suite
│   ├── results/
│   │   ├── plots/                          # Saved high-res PNG plots
│   │   ├── tables/                         # CSV / Text performance tables
│   │   └── simulation_data/                # .MAT data files
│   └── documentation/
│       ├── mathematical_model.md           # Theoretical derivation & proofs
│       ├── simulation_procedure.md         # Step-by-step user manual
│       └── validation_report.md            # Automated validation report
└── README.md
```

---

## 📜 License
This project is licensed under the MIT License.
