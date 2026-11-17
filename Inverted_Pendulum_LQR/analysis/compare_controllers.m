function [res_lqr, res_pid, m_lqr, m_pid, tbl] = compare_controllers(save_results)
% COMPARE_CONTROLLERS Benchmarks the optimal LQR controller against the baseline
% PID controller under identical initial conditions, physical parameters, and
% actuator saturation constraints.
%
% Generates overlaid response plots and a structured performance comparison table.

    if nargin < 1
        save_results = true;
    end

    params = parameters();
    cfg    = controller_settings();

    % Identical initial disturbance: 10 degrees tilt, cart at rest at origin
    x0 = [0.0; 0.0; deg2rad(10.0); 0.0];
    t_span = 0:0.005:6.0; % High-resolution time vector

    % Run LQR Simulation
    fprintf('Running LQR simulation...\n');
    res_lqr = run_simulation('lqr', params, cfg, x0, t_span, true);
    m_lqr   = calculate_metrics(res_lqr);

    % Run PID Simulation
    fprintf('Running PID simulation...\n');
    res_pid = run_simulation('pid', params, cfg, x0, t_span, true);
    m_pid   = calculate_metrics(res_pid);

    % Display Comparison Table in Command Window
    fprintf('\n========================================================================\n');
    fprintf('            LQR vs PID BENCHMARK PERFORMANCE COMPARISON\n');
    fprintf('========================================================================\n');
    fprintf('Metric                          LQR Controller      PID Controller\n');
    fprintf('------------------------------------------------------------------------\n');
    fprintf('Stabilization Status:           %s        %s\n', ...
        status_str(m_lqr.is_stabilized), status_str(m_pid.is_stabilized));
    fprintf('2%% Settling Time [s]:           %-18.3f  %-18.3f\n', ...
        m_lqr.settling_time, m_pid.settling_time);
    fprintf('Peak Angle Deviation [deg]:     %-18.2f  %-18.2f\n', ...
        m_lqr.peak_angle_deg, m_pid.peak_angle_deg);
    fprintf('Final Steady-State Angle [deg]: %-18.4e  %-18.4f\n', ...
        m_lqr.final_angle_deg, m_pid.final_angle_deg);
    fprintf('Maximum Cart Displacement [m]:  %-18.3f  %-18.3f\n', ...
        m_lqr.max_cart_disp, m_pid.max_cart_disp);
    fprintf('Final Cart Position [m]:        %-18.4f  %-18.4f\n', ...
        m_lqr.final_cart_pos, m_pid.final_cart_pos);
    fprintf('Track Limit Exceeded (|p|>1m):  %-18s  %-18s\n', ...
        bool_str(m_lqr.track_exceeded), bool_str(m_pid.track_exceeded));
    fprintf('Peak Control Force [N]:         %-18.2f  %-18.2f\n', ...
        m_lqr.peak_force, m_pid.peak_force);
    fprintf('Integrated Squared Error (ISE): %-18.4e  %-18.4e\n', ...
        m_lqr.ISE, m_pid.ISE);
    fprintf('Integrated Control Effort (ICE):%-18.3f  %-18.3f\n', ...
        m_lqr.ICE, m_pid.ICE);
    fprintf('========================================================================\n\n');

    % Create Overlaid High-Resolution Figure
    fig = figure('Name', 'LQR vs PID Controller Comparison', ...
                 'Units', 'pixels', 'Position', [100, 80, 1100, 750], ...
                 'Color', 'w', 'Visible', 'on');

    % Subplot 1: Pendulum Angle theta(t)
    subplot(2, 2, 1);
    plot(res_lqr.t, res_lqr.theta_deg, 'b-', 'LineWidth', 2.0); hold on;
    plot(res_pid.t, res_pid.theta_deg, 'r--', 'LineWidth', 2.0);
    yline(0, 'k:');
    yline(1.0, 'k--', 'LineWidth', 0.8);
    yline(-1.0, 'k--', 'LineWidth', 0.8);
    grid on;
    xlabel('Time [s]', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel('\theta [deg]', 'FontSize', 10, 'FontWeight', 'bold');
    title('Pendulum Angular Deviation \theta(t)', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Optimal LQR', 'Baseline PID', 'Location', 'best');

    % Subplot 2: Cart Position p(t)
    subplot(2, 2, 2);
    plot(res_lqr.t, res_lqr.p, 'b-', 'LineWidth', 2.0); hold on;
    plot(res_pid.t, res_pid.p, 'r--', 'LineWidth', 2.0);
    yline(0, 'k:');
    yline(params.track_lim, 'r:', 'Track Limit (+1m)', 'LineWidth', 1.2);
    yline(-params.track_lim, 'r:', 'Track Limit (-1m)', 'LineWidth', 1.2);
    grid on;
    xlabel('Time [s]', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel('Position p [m]', 'FontSize', 10, 'FontWeight', 'bold');
    title('Cart Position p(t) (Drift Comparison)', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Optimal LQR (Centered)', 'Baseline PID (Drifting)', 'Location', 'best');

    % Subplot 3: Actuator Control Force u(t)
    subplot(2, 2, 3);
    plot(res_lqr.t, res_lqr.u, 'b-', 'LineWidth', 2.0); hold on;
    plot(res_pid.t, res_pid.u, 'r--', 'LineWidth', 2.0);
    yline(params.Fmax, 'k--', 'Saturation (+15N)', 'LineWidth', 1.0);
    yline(-params.Fmax, 'k--', 'Saturation (-15N)', 'LineWidth', 1.0);
    grid on;
    xlabel('Time [s]', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel('Force u [N]', 'FontSize', 10, 'FontWeight', 'bold');
    title('Actuator Control Force u(t)', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Optimal LQR', 'Baseline PID', 'Location', 'best');

    % Subplot 4: Pendulum Angular Velocity dtheta/dt(t)
    subplot(2, 2, 4);
    plot(res_lqr.t, res_lqr.theta_dot, 'b-', 'LineWidth', 2.0); hold on;
    plot(res_pid.t, res_pid.theta_dot, 'r--', 'LineWidth', 2.0);
    yline(0, 'k:');
    grid on;
    xlabel('Time [s]', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel('d\theta/dt [rad/s]', 'FontSize', 10, 'FontWeight', 'bold');
    title('Pendulum Angular Rate d\theta/dt(t)', 'FontSize', 11, 'FontWeight', 'bold');
    legend('Optimal LQR', 'Baseline PID', 'Location', 'best');

    sgtitle('Comparative Evaluation: Optimal LQR vs Baseline PID on Nonlinear Plant', ...
        'FontSize', 13, 'FontWeight', 'bold');

    % Save results if requested
    if save_results
        plot_path = fullfile('results', 'plots', 'lqr_vs_pid_comparison.png');
        saveas(fig, plot_path);
        fprintf('Saved comparison plot: %s\n', plot_path);

        tbl_path = fullfile('results', 'tables', 'controller_comparison.txt');
        fid = fopen(tbl_path, 'w');
        fprintf(fid, 'LQR vs PID BENCHMARK PERFORMANCE COMPARISON\n');
        fprintf(fid, 'Initial Condition: theta0 = 10 deg, p0 = 0 m\n\n');
        fprintf(fid, 'Metric,LQR,PID\n');
        fprintf(fid, 'Stabilized,%s,%s\n', bool_str(m_lqr.is_stabilized), bool_str(m_pid.is_stabilized));
        fprintf(fid, 'Settling_Time_s,%.3f,%.3f\n', m_lqr.settling_time, m_pid.settling_time);
        fprintf(fid, 'Peak_Angle_deg,%.2f,%.2f\n', m_lqr.peak_angle_deg, m_pid.peak_angle_deg);
        fprintf(fid, 'Final_Angle_deg,%.4e,%.4f\n', m_lqr.final_angle_deg, m_pid.final_angle_deg);
        fprintf(fid, 'Max_Cart_Disp_m,%.3f,%.3f\n', m_lqr.max_cart_disp, m_pid.max_cart_disp);
        fprintf(fid, 'Final_Cart_Pos_m,%.4f,%.4f\n', m_lqr.final_cart_pos, m_pid.final_cart_pos);
        fprintf(fid, 'Track_Exceeded,%s,%s\n', bool_str(m_lqr.track_exceeded), bool_str(m_pid.track_exceeded));
        fprintf(fid, 'Peak_Force_N,%.2f,%.2f\n', m_lqr.peak_force, m_pid.peak_force);
        fprintf(fid, 'ISE,%.4e,%.4e\n', m_lqr.ISE, m_pid.ISE);
        fprintf(fid, 'ICE,%.3f,%.3f\n', m_lqr.ICE, m_pid.ICE);
        fclose(fid);
        fprintf('Saved comparison table: %s\n', tbl_path);

        data_path = fullfile('results', 'simulation_data', 'comparison_data.mat');
        save(data_path, 'res_lqr', 'res_pid', 'm_lqr', 'm_pid');
        fprintf('Saved simulation data: %s\n', data_path);
    end

    tbl = struct('m_lqr', m_lqr, 'm_pid', m_pid);
end

function s = status_str(is_stab)
    if is_stab
        s = 'STABILIZED';
    else
        s = 'UNSTABLE';
    end
end

function s = bool_str(b)
    if b
        s = 'YES';
    else
        s = 'NO';
    end
end
