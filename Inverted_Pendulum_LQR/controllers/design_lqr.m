function [K, P, cl_poles, is_stable] = design_lqr(A, B, Q, R)
% DESIGN_LQR Synthesizes optimal state-feedback gain matrix K by solving
% the Continuous-time Algebraic Riccati Equation (CARE):
%   A'*P + P*A - P*B*inv(R)*B'*P + Q = 0
% Feedback law:
%   u = -K*x,  where K = inv(R)*B'*P
%
% Inputs:
%   A, B : Linear system matrices [4x4, 4x1]
%   Q    : State penalty matrix [4x4, positive semi-definite]
%   R    : Control penalty scalar [1x1, positive]
%
% Outputs:
%   K         : Optimal feedback gain vector [1x4] = [k_p, k_pdot, k_theta, k_thetadot]
%   P         : Symmetric positive definite Riccati solution matrix [4x4]
%   cl_poles  : Closed-loop eigenvalues eig(A - B*K) [4x1]
%   is_stable : Boolean flag (true if all real(cl_poles) < 0)

    % Validate inputs
    if size(Q, 1) ~= 4 || size(Q, 2) ~= 4
        error('design_lqr:InvalidQ', 'Q must be a 4x4 positive semi-definite matrix.');
    end
    if ~isscalar(R) || R <= 0
        error('design_lqr:InvalidR', 'R must be a positive scalar.');
    end

    % Solve CARE and compute gain matrix
    % MATLAB's lqr function uses Schur decomposition
    [K, P, cl_poles] = lqr(A, B, Q, R);

    % Check closed-loop asymptotic stability:
    % All closed-loop eigenvalues must reside strictly in open left half-plane
    is_stable = all(real(cl_poles) < -1e-5);
end
