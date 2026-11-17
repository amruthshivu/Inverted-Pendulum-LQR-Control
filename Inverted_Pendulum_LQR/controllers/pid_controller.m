function [u_unsat, de_int] = pid_controller(x, z_int, pid_cfg, params)
% PID_CONTROLLER Computes the control force using PID feedback on pendulum angle.
%
% Sign Convention Analysis:
%   In this coordinate system, when theta > 0 (pendulum tilted forward), the cart
%   must accelerate forward (positive force u > 0) to bring the pivot underneath
%   the center of mass and restore the upright posture.
%   Therefore, for theta_ref = 0, the error is:
%       e_theta = theta_ref - theta = -theta
%   And the restorative control input requires:
%       u_prop = -Kp * e_theta = +Kp * theta
%       u_deriv = -Kd * (de_theta/dt) = +Kd * theta_dot
%       u_integ = -Ki * integral(e_theta) = +Ki * z_int  (where dz_int/dt = theta)
%
% Optional Cascade Control:
%   If pid_cfg.use_cascade is true, cart position error generates an angle command
%   theta_ref so the cart returns to p = 0 while maintaining upright balance.
%
% Inputs:
%   x       : State vector [p, p_dot, theta, theta_dot]^T
%   z_int   : Accumulated integrator state [rad*s]
%   pid_cfg : PID configuration struct (.Kp, .Ki, .Kd, .antiwindup, etc.)
%   params  : Parameter struct from parameters.m
%
% Outputs:
%   u_unsat : Raw unsaturating control force [N]
%   de_int  : Integrator derivative dz_int/dt with anti-windup conditioning

    p         = x(1);
    p_dot     = x(2);
    theta     = x(3);
    theta_dot = x(4);

    % Optional cascade outer loop: Cart position regulation
    if isfield(pid_cfg, 'use_cascade') && pid_cfg.use_cascade
        % If cart is to the right (p > 0), command a negative tilt (lean left)
        theta_ref = -pid_cfg.Kp_cart * p - pid_cfg.Kd_cart * p_dot;
        % Limit maximum tilt command to prevent destabilizing the inner loop
        max_tilt_cmd = deg2rad(15.0);
        theta_ref = max(-max_tilt_cmd, min(max_tilt_cmd, theta_ref));
    else
        theta_ref = 0.0;
    end

    % Angle error and its derivative
    e_theta = theta - theta_ref;
    e_dot   = theta_dot; % Assuming theta_ref_dot = 0

    % PID Control Effort
    u_p = pid_cfg.Kp * e_theta;
    u_i = pid_cfg.Ki * z_int;
    u_d = pid_cfg.Kd * e_dot;

    u_unsat = u_p + u_i + u_d;

    % Integrator anti-windup clamping:
    % Halt integration if actuator is saturated and error drives it further into saturation
    Fmax = params.Fmax;
    if pid_cfg.antiwindup && (abs(u_unsat) >= Fmax) && (sign(u_unsat) == sign(e_theta))
        de_int = 0.0; % Clamp integrator
    else
        de_int = e_theta;
    end
end
