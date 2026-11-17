function xdot = nonlinear_dynamics(t, x, u, params)
% NONLINEAR_DYNAMICS State derivatives for the full nonlinear cart-pendulum system.
%
% Equations of Motion (Lagrangian derivation):
%   (M + m)*p_ddot + b*p_dot + m*l*cos(theta)*theta_ddot - m*l*sin(theta)*theta_dot^2 = u
%   (I + m*l^2)*theta_ddot + m*l*cos(theta)*p_ddot - m*g*l*sin(theta) = 0
%
% Inputs:
%   t      : Current time [s] (unused for autonomous system, required for ODE solvers)
%   x      : State vector [4x1]:
%            x(1) = p         (cart position, [m])
%            x(2) = p_dot     (cart velocity, [m/s])
%            x(3) = theta     (pendulum angle from upright, [rad])
%            x(4) = theta_dot (pendulum angular rate, [rad/s])
%   u      : Control force applied to cart [N] (scalar)
%   params : Parameter struct from parameters.m
%
% Outputs:
%   xdot   : Time derivatives dx/dt [4x1] = [p_dot; p_ddot; theta_dot; theta_ddot]

    % Unpack state variables
    p_dot     = x(2);
    theta     = x(3);
    theta_dot = x(4);

    % Unpack physical parameters
    M = params.M;
    m = params.m;
    l = params.l;
    I = params.I;
    b = params.b;
    g = params.g;

    % Trigonometric terms
    s = sin(theta);
    c = cos(theta);

    % Right-hand side generalized force terms
    % F1 = u - b*p_dot + m*l*sin(theta)*theta_dot^2
    % F2 = m*g*l*sin(theta)
    F1 = u - b * p_dot + m * l * s * (theta_dot^2);
    F2 = m * g * l * s;

    % Coupled mass matrix determinant:
    % Delta = (M + m)*(I + m*l^2) - (m*l*cos(theta))^2
    % Note: Delta > 0 for all theta since (M+m)(I+ml^2) - (ml)^2 > 0
    total_cart_mass = M + m;
    pend_rot_inertia = I + m * (l^2);
    coupling = m * l * c;

    delta = total_cart_mass * pend_rot_inertia - (coupling^2);

    % Solve for accelerations via 2x2 matrix inversion (Cramer's rule):
    % [ total_cart_mass   coupling       ] [ p_ddot     ] = [ F1 ]
    % [ coupling          pend_rot_inertia] [ theta_ddot ] = [ F2 ]
    p_ddot     = (pend_rot_inertia * F1 - coupling * F2) / delta;
    theta_ddot = (-coupling * F1 + total_cart_mass * F2) / delta;

    % Assemble state derivative vector
    xdot = [p_dot; p_ddot; theta_dot; theta_ddot];
end
