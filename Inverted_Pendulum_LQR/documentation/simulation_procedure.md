# Step-by-Step MATLAB Execution Procedure & User Manual
## Project: Optimal Stabilization of an Inverted Pendulum Using LQR Control

This manual provides instructions for configuring, running, testing, and validating the Inverted Pendulum LQR Control Project in MATLAB.

---

### Step 1: MATLAB Environment Preparation

1. **Supported MATLAB Versions:**
   - MATLAB R2020a through R2025b (Verified and tested on MATLAB R2025b 64-bit on Windows 11).
2. **Toolbox Requirements:**
   - `Control System Toolbox` (for `lqr`, `ss`, `ctrb`, `eig`).
   - Base MATLAB graphics (`uifigure`, `uiaxes`, `uislider`, `uibutton`).
   - *(Optional)* `Symbolic Math Toolbox` (for symbolic Jacobians).
3. **Verify Installed Toolboxes in MATLAB:**
   ```matlab
   v = ver;
   any(strcmp({v.Name}, 'Control System Toolbox'))
   ```
4. **Set Working Directory & Path:**
   Open MATLAB and set your current working directory to the project folder:
   ```matlab
   cd('c:\Users\abhid\OneDrive\Desktop\CSD\Inverted_Pendulum_LQR');
   addpath(genpath(pwd));
   ```

---

### Step 2: Testing Physical Plant & Nonlinear ODEs

Verify that the nonlinear differential equations compute correct derivatives:
```matlab
params = parameters();
% Verify equilibrium derivative f(0, 0) == 0
xdot_eq = nonlinear_dynamics(0, zeros(4, 1), 0, params);
disp('Equilibrium state derivative (should be all zeros):');
disp(xdot_eq);
```
**Expected Output:**
```matlab
0
0
0
0
```

---

### Step 3: State-Space Linearization & Stability Audit

Compute the $A, B, C, D$ matrices, check open-loop poles, and verify controllability:
```matlab
[A, B, C, D] = linear_state_space(params);
[A_num, B_num, max_err_A, max_err_B] = linearize_model(params);

fprintf('Linearization Error: A = %.2e, B = %.2e\n', max_err_A, max_err_B);
fprintf('Open-Loop Poles:\n');
disp(eig(A));
fprintf('Controllability Matrix Rank: %d / 4\n', rank(ctrb(A, B)));
```
**Expected Output:**
- `Linearization Error`: $< 10^{-11}$
- `Open-Loop Poles`: One pole at $+5.617\text{ rad/s}$ (confirming open-loop instability)
- `Controllability Rank`: $4$ (full rank, system is completely controllable).

---

### Step 4: LQR Controller Synthesis

Solve the Continuous-time Algebraic Riccati Equation (CARE) and verify closed-loop poles:
```matlab
cfg = controller_settings();
[K, P, cl_poles, is_stable] = design_lqr(A, B, cfg.lqr.Q, cfg.lqr.R);

disp('Computed LQR Feedback Gain Matrix K:');
disp(K);
disp('Closed-Loop Poles:');
disp(cl_poles);
fprintf('Closed-Loop Stability Flag: %d\n', is_stable);
```
**Expected Gain Vector:**
```matlab
K = [-20.0000,  -22.8890,  -113.6308,  -21.1685]
```
All closed-loop poles have strictly negative real parts:
$$\lambda_{CL} = \{-1.660 \pm 0.950j, \ -4.998, \ -31.406\}$$

---

### Step 5: Benchmark Simulation (Open-Loop vs LQR vs PID)

Run the nonlinear simulations and calculate quantitative performance metrics:
```matlab
% Open Loop (Demonstrates falling pendulum)
res_ol = run_simulation('open_loop', params, cfg, [0; 0; deg2rad(2); 0], 0:0.01:2);
fprintf('Open-Loop Angle at t=2s: %.2f deg (Fallen)\n', res_ol.theta_deg(end));

% LQR Stabilization (10 deg initial disturbance)
res_lqr = run_simulation('lqr', params, cfg, [0; 0; deg2rad(10); 0], 0:0.01:6);
m_lqr = calculate_metrics(res_lqr);
fprintf('LQR Settling Time: %.2f s | Max Cart Disp: %.3f m\n', ...
    m_lqr.settling_time, m_lqr.max_cart_disp);

% PID Stabilization (10 deg initial disturbance)
res_pid = run_simulation('pid', params, cfg, [0; 0; deg2rad(10); 0], 0:0.01:6);
m_pid = calculate_metrics(res_pid);
fprintf('PID Settling Time: %.2f s | Max Cart Disp: %.3f m (Track Exceeded: %d)\n', ...
    m_pid.settling_time, m_pid.max_cart_disp, m_pid.track_exceeded);
```

---

### Step 6: Automated Controller Benchmark & Plot Generation

To generate overlaid comparative response figures and save comparison tables:
```matlab
compare_controllers(true);
```
**Saved Files:**
- `results/plots/lqr_vs_pid_comparison.png`
- `results/tables/controller_comparison.txt`
- `results/simulation_data/comparison_data.mat`

---

### Step 7: LQR Q and R Parameter Sensitivity Analysis

To run the systematic parameter sweep across 5 tuning configurations:
```matlab
q_r_sensitivity(true);
```
**Saved Files:**
- `results/plots/lqr_q_r_sensitivity.png`
- `results/tables/q_r_sensitivity_table.txt`

---

### Step 8: Complete Engineering Validation Suite

To run all 9 verification tests and generate the verification audit report:
```matlab
[test_results, pass_all] = validation_tests(true);
```
**Saved File:**
- `documentation/validation_report.md`

---

### Step 9: Launching the Interactive MATLAB Simulator GUI

To launch the full graphical simulator application:
```matlab
main('app');
```
Or simply:
```matlab
main
```
and select `[1]`.

#### How to Interact with the Simulator GUI:
1. **Controller Selection:** Select between "Optimal LQR", "Baseline PID", or "Open-Loop".
2. **LQR Tuning Sliders:** Drag sliders $q_1, q_2, q_3, q_4$ or $R$ to modify state and control weights. Click "Update LQR Gains" to recalculate $K$ live.
3. **Initial Conditions:** Adjust cart position $p_0$ or pendulum angle $\theta_0$ (e.g. $10^\circ$).
4. **Simulation Execution:**
   - Click `▶ START` to run the numerical simulation.
   - Click `❚❚ PAUSE` / `▶ RESUME` to pause or resume playback.
   - Click `↺ RESET` to return the system to the initial rest position.
   - Click `■ STOP` to abort playback.
5. **Speed Dropdown:** Choose between `0.5x`, `1.0x (Real-time)`, `2.0x`, or `Instant` (immediate graph display).
6. **Telemetry & Dashboard:** Observe live numerical indicators for 2% Settling Time, Peak Angle, Max Cart Drift, and Integrated Effort (ICE).

---

### Step 10: Troubleshooting FAQ

| Problem | Cause | Solution |
| :--- | :--- | :--- |
| `Undefined function 'lqr'` | Control System Toolbox not on path or uninstalled. | Verify installation via `ver('control')`. |
| `Actuator saturation exceeded` | Initial angle $\theta_0$ too large ($> 35^\circ$). | Linear LQR has a local basin of attraction. Use $|\theta_0| \le 25^\circ$. |
| `Cart exceeds rail limits (|p| > 1m)` | Occurs under PID control because angle PID does not penalize cart position. | Switch to LQR, or increase LQR cart position weight $q_1$. |
| `Figure window does not open` | MATLAB running in `-nodisplay` mode. | Run in standard desktop MATLAB session with display enabled. |
