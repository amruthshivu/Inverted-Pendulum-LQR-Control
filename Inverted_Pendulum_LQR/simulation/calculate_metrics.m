function metrics = calculate_metrics(res, tol_angle_deg)
% CALCULATE_METRICS Evaluates standard control performance metrics for
% simulation response of the inverted pendulum cart system.
%
% Inputs:
%   res            : Result struct from run_simulation.m
%   tol_angle_deg  : Angle tolerance for settling time (default: 1.0 deg or 2% of initial angle)
%
% Outputs:
%   metrics : Struct containing:
%       .is_stabilized : Boolean flag
%       .settling_time : 2% settling time [s] (or NaN if unstabilized)
%       .peak_angle_deg: Maximum absolute pendulum angle [deg]
%       .final_angle_deg: Final steady-state angle error [deg]
%       .max_cart_disp : Maximum absolute cart position [m]
%       .final_cart_pos: Final cart position [m]
%       .peak_force    : Maximum actuator force magnitude [N]
%       .ISE           : Integral of Squared Error (angle) [rad^2*s]
%       .ICE           : Integral of Control Effort [N^2*s]
%       .track_exceeded: Boolean flag (whether |p| > track_lim)

    t = res.t;
    theta_deg = res.theta_deg;
    theta_rad = res.theta;
    p = res.p;
    u = res.u;
    params = res.params;

    theta0_deg = abs(theta_deg(1));
    if nargin < 2 || isempty(tol_angle_deg)
        % Standard 2% of initial disturbance or 1.0 degree minimum resolution
        tol_angle_deg = max(0.02 * theta0_deg, 1.0);
    end

    % 1. Peak Angular Deviation
    metrics.peak_angle_deg = max(abs(theta_deg));
    metrics.final_angle_deg = abs(theta_deg(end));

    % 2. Cart Displacements
    metrics.max_cart_disp = max(abs(p));
    metrics.final_cart_pos = p(end);
    metrics.track_exceeded = metrics.max_cart_disp > params.track_lim;

    % 3. Peak Actuator Force
    metrics.peak_force = max(abs(u));

    % 4. Integral Criteria (Trapezoidal numerical quadrature)
    metrics.ISE = trapz(t, theta_rad.^2);
    metrics.ICE = trapz(t, u.^2);

    % 5. Stability & Settling Time Check
    % Check if response stays within tolerance band for the final 10% of simulation
    tail_idx = find(t >= 0.9 * t(end));
    stable_tail = all(abs(theta_deg(tail_idx)) <= tol_angle_deg) && (abs(res.theta_dot(end)) < 0.1);

    if stable_tail
        metrics.is_stabilized = true;
        % Find the last time instant the angle breached the tolerance band
        outside_band_idx = find(abs(theta_deg) > tol_angle_deg, 1, 'last');
        if isempty(outside_band_idx)
            metrics.settling_time = 0.0;
        elseif outside_band_idx < length(t)
            metrics.settling_time = t(outside_band_idx + 1);
        else
            metrics.settling_time = t(end);
        end
    else
        metrics.is_stabilized = false;
        metrics.settling_time = NaN;
    end
end
