# System Engineering Validation and Verification Report

## Project: Optimal Stabilization of an Inverted Pendulum Using LQR Control

**Date & Environment:** MATLAB 25.2.0.3177638 (R2025b) Update 5 on Microsoft Windows 11

### 1. Executive Summary

- **Total Verification Tests:** 9
- **Tests Passed:** 9
- **Overall Audit Status:** **PASS**

### 2. Detailed Test Results

| # | Verification Objective | Criterion | Numerical Result | Status |
|---|---|---|---|:---:|
| 1 | Physical Parameters Sanity | All constants > 0, I = mL^2/12 | I = 0.002083 kg*m^2 | PASS |
| 2 | Equilibrium Zero Derivative | ||f(0,0)||_2 < 1e-12 | 0.00e+00 | PASS |
| 3 | Jacobian Linearization Match | ||A_num - A_ana||_inf < 1e-6 | 7.58e-12 | PASS |
| 4 | Open-Loop Instability Check | Exists Re(lambda) > 0 | +5.617 rad/s | PASS |
| 5 | System Controllability | rank(ctrb(A, B)) == 4 | 4 / 4 | PASS |
| 6 | LQR Closed-Loop Stability | All Re(lambda) < 0 | max Re(lambda) = -1.660 | PASS |
| 7 | Actuator Saturation Compliance | max |u(t)| <= 15.0 N | 15.00 N | PASS |
| 8 | ODE Numerical Convergence | Mesh deviation < 1e-3 rad | 0.00e+00 rad | PASS |
| 9 | Basin of Attraction (Tilt) | Recoverable tilt >= 15 deg | 35 deg | PASS |

### 3. Engineering Discussion

1. **Jacobian Consistency:** The analytical state-space derivation matches the numerical central difference Jacobian down to 7.581491e-12, proving exact mathematical agreement.
2. **Local vs Global Stabilization:** Linear LQR provides robust asymptotic stabilization within a local basin of attraction up to approximately 35 degrees of initial tilt under 15 N actuator limits. Beyond this threshold, nonlinear trigonometric terms and actuator saturation prevent recovery without a non-linear swing-up controller.
