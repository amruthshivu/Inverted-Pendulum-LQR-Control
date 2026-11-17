function update_plots(axes_struct, t_hist, x_hist, u_hist, params, cfg)
% UPDATE_PLOTS Updates the four dynamic response plots during or after simulation.
%
% Inputs:
%   axes_struct : Struct containing 4 axes handles:
%                 .ax_theta, .ax_p, .ax_u, .ax_omega
%   t_hist      : Time vector [kx1]
%   x_hist      : State history [kx4]
%   u_hist      : Force history [kx1]
%   params      : Physical parameters struct
%   cfg         : Controller settings struct

    if isempty(t_hist)
        return;
    end

    p_hist = x_hist(:, 1);
    theta_deg = rad2deg(x_hist(:, 3));
    omega_hist = x_hist(:, 4);

    % Plot 1: Pendulum Angle theta [deg]
    cla(axes_struct.ax_theta);
    plot(axes_struct.ax_theta, t_hist, theta_deg, 'b-', 'LineWidth', 1.8);
    hold(axes_struct.ax_theta, 'on');
    yline(axes_struct.ax_theta, 0, 'k--', 'LineWidth', 0.8);
    % Tolerance bounds (+/- 1 deg)
    yline(axes_struct.ax_theta, 1.0, 'r:', 'LineWidth', 1.0);
    yline(axes_struct.ax_theta, -1.0, 'r:', 'LineWidth', 1.0);
    hold(axes_struct.ax_theta, 'off');
    grid(axes_struct.ax_theta, 'on');
    xlabel(axes_struct.ax_theta, 'Time [s]');
    ylabel(axes_struct.ax_theta, '\theta [deg]');
    title(axes_struct.ax_theta, 'Pendulum Angle \theta(t)');
    xlim(axes_struct.ax_theta, [0, max(t_hist(end), cfg.sim.t_final)]);

    % Plot 2: Cart Position p [m]
    cla(axes_struct.ax_p);
    plot(axes_struct.ax_p, t_hist, p_hist, 'Color', [0.1, 0.6, 0.2], 'LineWidth', 1.8);
    hold(axes_struct.ax_p, 'on');
    yline(axes_struct.ax_p, 0, 'k--', 'LineWidth', 0.8);
    yline(axes_struct.ax_p, params.track_lim, 'r--', 'LineWidth', 1.0);
    yline(axes_struct.ax_p, -params.track_lim, 'r--', 'LineWidth', 1.0);
    hold(axes_struct.ax_p, 'off');
    grid(axes_struct.ax_p, 'on');
    xlabel(axes_struct.ax_p, 'Time [s]');
    ylabel(axes_struct.ax_p, 'Position p [m]');
    title(axes_struct.ax_p, 'Cart Displacement p(t)');
    xlim(axes_struct.ax_p, [0, max(t_hist(end), cfg.sim.t_final)]);

    % Plot 3: Actuator Force u [N]
    cla(axes_struct.ax_u);
    plot(axes_struct.ax_u, t_hist, u_hist, 'm-', 'LineWidth', 1.8);
    hold(axes_struct.ax_u, 'on');
    yline(axes_struct.ax_u, params.Fmax, 'r--', 'LineWidth', 1.0);
    yline(axes_struct.ax_u, -params.Fmax, 'r--', 'LineWidth', 1.0);
    hold(axes_struct.ax_u, 'off');
    grid(axes_struct.ax_u, 'on');
    xlabel(axes_struct.ax_u, 'Time [s]');
    ylabel(axes_struct.ax_u, 'Force u [N]');
    title(axes_struct.ax_u, 'Actuator Control Force u(t)');
    xlim(axes_struct.ax_u, [0, max(t_hist(end), cfg.sim.t_final)]);

    % Plot 4: Angular Velocity theta_dot [rad/s]
    cla(axes_struct.ax_omega);
    plot(axes_struct.ax_omega, t_hist, omega_hist, 'Color', [0.85, 0.45, 0.1], 'LineWidth', 1.8);
    hold(axes_struct.ax_omega, 'on');
    yline(axes_struct.ax_omega, 0, 'k--', 'LineWidth', 0.8);
    hold(axes_struct.ax_omega, 'off');
    grid(axes_struct.ax_omega, 'on');
    xlabel(axes_struct.ax_omega, 'Time [s]');
    ylabel(axes_struct.ax_omega, 'd\theta/dt [rad/s]');
    title(axes_struct.ax_omega, 'Angular Velocity d\theta/dt(t)');
    xlim(axes_struct.ax_omega, [0, max(t_hist(end), cfg.sim.t_final)]);
end
