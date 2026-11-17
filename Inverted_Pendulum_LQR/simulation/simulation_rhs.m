function dxdt = simulation_rhs(t, x_state, ctrl_type, ctrl_data, params, enable_saturation)
% SIMULATION_RHS Continuous-time right-hand side differential equations
% for ode45 integration, coupling controller output with nonlinear plant.
%
% Inputs:
%   t                 : Time [s]
%   x_state           : State vector (4x1 for LQR/Open-Loop, 5x1 for PID with integrator)
%   ctrl_type         : 'open_loop', 'lqr', or 'pid'
%   ctrl_data         : Gain matrix K or PID config struct
%   params            : Parameters struct
%   enable_saturation : Boolean flag

    if nargin < 6
        enable_saturation = true;
    end

    x = x_state(1:4);

    switch lower(ctrl_type)
        case 'open_loop'
            u = 0.0;
            dxdt = nonlinear_dynamics(t, x, u, params);

        case 'lqr'
            K = ctrl_data;
            u_unsat = -K * x;
            if enable_saturation && isfield(params, 'Fmax') && params.Fmax > 0
                u = max(-params.Fmax, min(params.Fmax, u_unsat));
            else
                u = u_unsat;
            end
            dxdt = nonlinear_dynamics(t, x, u, params);

        case 'pid'
            z_int = 0.0;
            if length(x_state) >= 5
                z_int = x_state(5);
            end
            [u_unsat, de_int] = pid_controller(x, z_int, ctrl_data, params);
            if enable_saturation && isfield(params, 'Fmax') && params.Fmax > 0
                u = max(-params.Fmax, min(params.Fmax, u_unsat));
            else
                u = u_unsat;
            end
            xdot_plant = nonlinear_dynamics(t, x, u, params);
            dxdt = [xdot_plant; de_int];

        otherwise
            error('simulation_rhs:UnknownController', 'Unknown controller type: %s', ctrl_type);
    end
end
