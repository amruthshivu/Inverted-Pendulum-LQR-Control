function [A, B, C, D, sys] = linear_state_space(params)
% LINEAR_STATE_SPACE Derives the analytical linearized state-space model
% around the upright unstable equilibrium x_e = [0, 0, 0, 0]^T, u_e = 0.
%
% States:
%   x = [p, p_dot, theta, theta_dot]^T
% Input:
%   u = horizontal force on cart [N]
% Output:
%   y = full state vector [4x1]
%
% Continuous-time system:
%   dx/dt = A*x + B*u
%       y = C*x + D*u
%
% Inputs:
%   params : Parameter struct from parameters.m
%
% Outputs:
%   A, B, C, D : State-space matrices
%   sys        : MATLAB ss object

    M = params.M;
    m = params.m;
    l = params.l;
    I = params.I;
    b = params.b;
    g = params.g;

    % Determinant at theta = 0:
    % Delta_0 = (M + m)*(I + m*l^2) - (m*l)^2 = I*(M + m) + M*m*l^2
    Delta_0 = (M + m) * (I + m * (l^2)) - (m * l)^2;

    % Matrix elements derived analytically from Taylor expansion at theta = 0:
    % cos(theta) -> 1, sin(theta) -> theta, theta_dot^2 -> 0
    A = zeros(4, 4);
    A(1, 2) = 1.0;
    A(2, 2) = -(I + m * (l^2)) * b / Delta_0;
    A(2, 3) = -(m^2 * g * (l^2)) / Delta_0;
    A(3, 4) = 1.0;
    A(4, 2) = (m * l * b) / Delta_0;
    A(4, 3) = ((M + m) * m * g * l) / Delta_0;

    B = zeros(4, 1);
    B(2) = (I + m * (l^2)) / Delta_0;
    B(4) = -(m * l) / Delta_0;

    C = eye(4);
    D = zeros(4, 1);

    % Create state-space object
    sys = ss(A, B, C, D);
    sys.StateName  = {'cart_pos', 'cart_vel', 'pend_angle', 'pend_rate'};
    sys.InputName  = {'cart_force'};
    sys.OutputName = sys.StateName;
end
