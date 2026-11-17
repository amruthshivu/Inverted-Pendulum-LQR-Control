function [A_num, B_num, max_err_A, max_err_B] = linearize_model(params)
% LINEARIZE_MODEL Numerically computes the Jacobian of the nonlinear model
% at the upright equilibrium (x = 0, u = 0) using central finite differences,
% and validates it against the analytical state-space matrices.
%
% Outputs:
%   A_num     : Numerically computed 4x4 state Jacobian (df/dx)
%   B_num     : Numerically computed 4x1 input Jacobian (df/du)
%   max_err_A : Maximum absolute difference ||A_num - A_analytical||_inf
%   max_err_B : Maximum absolute difference ||B_num - B_analytical||_inf

    if nargin < 1
        params = parameters();
    end

    x_eq = zeros(4, 1);
    u_eq = 0.0;
    delta = 1e-6;

    % Analytical matrices
    [A_ana, B_ana, ~, ~] = linear_state_space(params);

    % Numerical Jacobian A_num = df/dx at (x_eq, u_eq)
    A_num = zeros(4, 4);
    for j = 1:4
        x_plus  = x_eq; x_plus(j)  = x_plus(j) + delta;
        x_minus = x_eq; x_minus(j) = x_minus(j) - delta;

        f_plus  = nonlinear_dynamics(0, x_plus,  u_eq, params);
        f_minus = nonlinear_dynamics(0, x_minus, u_eq, params);

        A_num(:, j) = (f_plus - f_minus) / (2.0 * delta);
    end

    % Numerical Jacobian B_num = df/du at (x_eq, u_eq)
    u_plus  = u_eq + delta;
    u_minus = u_eq - delta;
    f_u_plus  = nonlinear_dynamics(0, x_eq, u_plus,  params);
    f_u_minus = nonlinear_dynamics(0, x_eq, u_minus, params);
    B_num = (f_u_plus - f_u_minus) / (2.0 * delta);

    % Verification errors
    max_err_A = max(max(abs(A_num - A_ana)));
    max_err_B = max(abs(B_num - B_ana));
end
