function res = run_simulation(ctrl_type, params, cfg, x0, t_span, enable_saturation)
% RUN_SIMULATION Integrates the closed-loop nonlinear inverted pendulum system
% using MATLAB's variable-step Runge-Kutta solver ode45.
%
% Inputs:
%   ctrl_type         : 'open_loop', 'lqr', or 'pid'
%   params            : Parameters struct from parameters.m
%   cfg               : Configuration struct from controller_settings.m
%   x0                : Initial state vector [4x1] = [p0, pdot0, theta0, thetadot0]^T
%   t_span            : Time vector or [t_start, t_final] (default: 0:cfg.sim.dt:cfg.sim.t_final)
%   enable_saturation : Boolean flag (default: true)
%
% Outputs:
%   res : Struct containing:
%       .t         : Time vector [Nx1]
%       .x         : State trajectory [Nx4]
%       .p         : Cart position [Nx1]
%       .p_dot     : Cart velocity [Nx1]
%       .theta     : Pendulum angle [rad] [Nx1]
%       .theta_deg : Pendulum angle [deg] [Nx1]
%       .theta_dot : Pendulum angular rate [rad/s] [Nx1]
%       .u         : Applied actuator force [Nx1]
%       .u_unsat   : Unbounded control demand [Nx1]
%       .ctrl_type : Controller name string
%       .params    : Parameters struct
%       .cfg       : Configuration struct

    if nargin < 4 || isempty(x0)
        x0 = cfg.sim.x0;
    end
    if nargin < 5 || isempty(t_span)
        t_span = 0:cfg.sim.dt:cfg.sim.t_final;
    end
    if nargin < 6
        enable_saturation = true;
    end

    % Prepare controller data
    switch lower(ctrl_type)
        case 'open_loop'
            ctrl_data = [];
            x_init = x0(:);

        case 'lqr'
            [A, B, ~, ~] = linear_state_space(params);
            [K, ~, ~, ~] = design_lqr(A, B, cfg.lqr.Q, cfg.lqr.R);
            ctrl_data = K;
            x_init = x0(:);

        case 'pid'
            ctrl_data = cfg.pid;
            % 5th state is error integrator z_int, initialized to 0
            x_init = [x0(:); 0.0];

        otherwise
            error('run_simulation:UnknownType', 'Unknown controller type: %s', ctrl_type);
    end

    % Integration options
    opts = odeset('RelTol', cfg.sim.reltol, 'AbsTol', cfg.sim.abstol);

    % Run ode45 solver
    rhs_handle = @(t, x_s) simulation_rhs(t, x_s, ctrl_type, ctrl_data, params, enable_saturation);
    [t_out, x_out] = ode45(rhs_handle, t_span, x_init, opts);

    % Extract states and compute control signals along the trajectory
    N = length(t_out);
    u_applied = zeros(N, 1);
    u_demanded = zeros(N, 1);

    for i = 1:N
        xi = x_out(i, 1:4)';
        switch lower(ctrl_type)
            case 'open_loop'
                u_demanded(i) = 0.0;
                u_applied(i) = 0.0;

            case 'lqr'
                u_dem = -ctrl_data * xi;
                u_demanded(i) = u_dem;
                if enable_saturation
                    u_applied(i) = max(-params.Fmax, min(params.Fmax, u_dem));
                else
                    u_applied(i) = u_dem;
                end

            case 'pid'
                zi = x_out(i, 5);
                [u_dem, ~] = pid_controller(xi, zi, ctrl_data, params);
                u_demanded(i) = u_dem;
                if enable_saturation
                    u_applied(i) = max(-params.Fmax, min(params.Fmax, u_dem));
                else
                    u_applied(i) = u_dem;
                end
        end
    end

    % Pack output struct
    res.t         = t_out;
    res.x         = x_out(:, 1:4);
    res.p         = x_out(:, 1);
    res.p_dot     = x_out(:, 2);
    res.theta     = x_out(:, 3);
    res.theta_deg = rad2deg(x_out(:, 3));
    res.theta_dot = x_out(:, 4);
    res.u         = u_applied;
    res.u_unsat   = u_demanded;
    res.ctrl_type = ctrl_type;
    res.ctrl_data = ctrl_data;
    res.params    = params;
    res.cfg       = cfg;
    res.x0        = x0;
end
