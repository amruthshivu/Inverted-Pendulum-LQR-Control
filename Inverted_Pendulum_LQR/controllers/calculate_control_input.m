function [u, u_unsat] = calculate_control_input(ctrl_type, x, ctrl_data, params, enable_saturation)
% CALCULATE_CONTROL_INPUT Evaluates the control law for LQR, PID, or Open-Loop,
% applying actuator saturation constraints.
%
% Inputs:
%   ctrl_type         : 'lqr', 'pid', or 'open_loop'
%   x                 : Current state vector [p, p_dot, theta, theta_dot]^T
%   ctrl_data         : Struct or matrix depending on ctrl_type:
%                       - For 'lqr': gain matrix K [1x4]
%                       - For 'pid': struct with .cfg (pid settings) and .z_int (integrator state)
%                       - For 'open_loop': empty or ignored
%   params            : System parameter struct from parameters.m
%   enable_saturation : Boolean (default: true)
%
% Outputs:
%   u                 : Saturated actuator force [N]
%   u_unsat           : Unbounded control demand [N]

    if nargin < 5
        enable_saturation = true;
    end

    switch lower(ctrl_type)
        case 'open_loop'
            u_unsat = 0.0;

        case 'lqr'
            % ctrl_data is gain matrix K [1x4]
            K = ctrl_data;
            u_unsat = -K * x;

        case 'pid'
            % ctrl_data is struct with fields: .cfg, .z_int
            z_int = 0.0;
            if isfield(ctrl_data, 'z_int')
                z_int = ctrl_data.z_int;
            end
            [u_unsat, ~] = pid_controller(x, z_int, ctrl_data.cfg, params);

        otherwise
            error('calculate_control_input:UnknownType', 'Unknown controller type: %s', ctrl_type);
    end

    % Apply physical actuator saturation limits
    if enable_saturation && isfield(params, 'Fmax') && params.Fmax > 0
        u = max(-params.Fmax, min(params.Fmax, u_unsat));
    else
        u = u_unsat;
    end
end
