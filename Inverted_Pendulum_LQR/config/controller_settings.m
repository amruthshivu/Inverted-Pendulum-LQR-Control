function cfg = controller_settings()
% CONTROLLER_SETTINGS Default configurations for LQR and PID controllers.
%
% Outputs:
%   cfg - Struct containing:
%       .lqr.q1     : Cart position penalty [1/m^2]
%       .lqr.q2     : Cart velocity penalty [1/(m/s)^2]
%       .lqr.q3     : Pendulum angle penalty [1/rad^2]
%       .lqr.q4     : Pendulum angular rate penalty [1/(rad/s)^2]
%       .lqr.R      : Control effort penalty [1/N^2]
%       .pid.Kp     : Pendulum angle proportional gain
%       .pid.Ki     : Pendulum angle integral gain
%       .pid.Kd     : Pendulum angle derivative gain
%       .pid.N      : Derivative low-pass filter coefficient
%       .pid.use_cascade : Flag to enable cart-centering outer loop
%       .pid.Kp_cart: Outer-loop cart position gain (cascade mode)
%       .pid.Kd_cart: Outer-loop cart velocity gain (cascade mode)
%       .sim.t_final: Total simulation duration [s]
%       .sim.dt     : Target time step for integration / animation [s]
%       .sim.x0     : Default initial state [p0, pdot0, theta0, thetadot0]^T

    % --- LQR Settings (Based on Bryson's Rule) ---
    cfg.lqr.q1 = 4.0;    % Max allowable cart pos error: 0.50 m  -> 1/(0.5)^2 = 4
    cfg.lqr.q2 = 1.0;    % Max allowable cart vel error: 1.00 m/s -> 1/(1.0)^2 = 1
    cfg.lqr.q3 = 25.0;   % Max allowable angle error: 0.20 rad (~11.5 deg) -> 1/(0.2)^2 = 25
    cfg.lqr.q4 = 1.0;    % Max allowable angular rate: 1.00 rad/s -> 1/(1.0)^2 = 1
    cfg.lqr.R  = 0.01;   % Max allowable force: 10 N -> 1/(10)^2 = 0.01

    % Assemble Q matrix
    cfg.lqr.Q = diag([cfg.lqr.q1, cfg.lqr.q2, cfg.lqr.q3, cfg.lqr.q4]);

    % --- PID Settings (Stabilizing angle feedback with derivative filtering) ---
    cfg.pid.Kp = 85.0;   % Proportional gain
    cfg.pid.Ki = 5.0;    % Integral gain (eliminates steady-state bias)
    cfg.pid.Kd = 12.0;   % Derivative gain (provides phase lead / damping)
    cfg.pid.N  = 100.0;  % Filter pole: s / (s/N + 1) to attenuate high-frequency noise
    cfg.pid.antiwindup = true; % Clamping anti-windup

    % Optional Cascade PID (Outer loop for cart position regulation)
    cfg.pid.use_cascade = false; % By default, pure benchmark angle PID
    cfg.pid.Kp_cart = 1.5;
    cfg.pid.Kd_cart = 2.0;

    % --- Simulation Runtime Settings ---
    cfg.sim.t_final = 8.0;      % 8 seconds simulation horizon
    cfg.sim.dt = 0.01;          % 100 Hz output sampling
    cfg.sim.reltol = 1e-6;      % ode45 relative tolerance
    cfg.sim.abstol = 1e-8;      % ode45 absolute tolerance

    % Default Initial Condition: 10 degrees (0.1745 rad) initial tilt
    cfg.sim.x0 = [0.0; 0.0; deg2rad(10.0); 0.0];
end
