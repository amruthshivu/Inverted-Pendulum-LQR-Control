function results_table = q_r_sensitivity(save_results)
% Q_R_SENSITIVITY Performs systematic parametric study on LQR weighting matrices
% Q and R, evaluating stability, gain vectors, closed-loop poles, and time-domain
% performance on the nonlinear inverted pendulum system.

    if nargin < 1
        save_results = true;
    end

    params = parameters();
    cfg_base = controller_settings();
    [A, B, ~, ~] = linear_state_space(params);

    % Define 5 representative parametric configurations
    cases(1).name = 'Baseline (Bryson)';
    cases(1).q1 = 4.0;  cases(1).q2 = 1.0; cases(1).q3 = 25.0;  cases(1).q4 = 1.0; cases(1).R = 0.01;
    cases(1).color = [0, 0.4470, 0.7410]; % Blue
    cases(1).desc = 'Nominal balanced Bryson rule';

    cases(2).name = 'High Angle Penalty (q3 x 10)';
    cases(2).q1 = 4.0;  cases(2).q2 = 1.0; cases(2).q3 = 250.0; cases(2).q4 = 1.0; cases(2).R = 0.01;
    cases(2).color = [0.8500, 0.3250, 0.0980]; % Orange
    cases(2).desc = 'Aggressive pendulum stabilization';

    cases(3).name = 'High Cart Penalty (q1 x 10)';
    cases(3).q1 = 40.0; cases(3).q2 = 1.0; cases(3).q3 = 25.0;  cases(3).q4 = 1.0; cases(3).R = 0.01;
    cases(3).color = [0.4660, 0.6740, 0.1880]; % Green
    cases(3).desc = 'Strict cart rail centering';

    cases(4).name = 'High Control Cost (R x 10)';
    cases(4).q1 = 4.0;  cases(4).q2 = 1.0; cases(4).q3 = 25.0;  cases(4).q4 = 1.0; cases(4).R = 0.10;
    cases(4).color = [0.4940, 0.1840, 0.5560]; % Purple
    cases(4).desc = 'Conservative actuator effort / lower force';

    cases(5).name = 'Low Control Cost (R / 10)';
    cases(5).q1 = 4.0;  cases(5).q2 = 1.0; cases(5).q3 = 25.0;  cases(5).q4 = 1.0; cases(5).R = 0.001;
    cases(5).color = [0.6350, 0.0780, 0.1840]; % Dark Red
    cases(5).desc = 'Aggressive high-gain control';

    N_cases = length(cases);
    x0 = [0.0; 0.0; deg2rad(10.0); 0.0];
    t_span = 0:0.005:6.0;

    fprintf('\n========================================================================================\n');
    fprintf('                      LQR Q & R PARAMETRIC SENSITIVITY STUDY\n');
    fprintf('========================================================================================\n');
    fprintf('%-30s %-10s %-10s %-10s %-10s\n', 'Configuration', 'Ts [s]', 'Max |p| [m]', 'Peak u [N]', 'ICE [N^2*s]');
    fprintf('----------------------------------------------------------------------------------------\n');

    fig = figure('Name', 'LQR Q and R Parameter Sensitivity Analysis', ...
                 'Units', 'pixels', 'Position', [100, 100, 1200, 800], ...
                 'Color', 'w', 'Visible', 'on');

    results_table = cell(N_cases, 8);

    for k = 1:N_cases
        c = cases(k);
        Q_k = diag([c.q1, c.q2, c.q3, c.q4]);
        R_k = c.R;

        [K_k, ~, poles_k, ~] = design_lqr(A, B, Q_k, R_k);

        cfg_k = cfg_base;
        cfg_k.lqr.Q = Q_k;
        cfg_k.lqr.R = R_k;

        res_k = run_simulation('lqr', params, cfg_k, x0, t_span, true);
        m_k   = calculate_metrics(res_k);

        fprintf('%-30s %-10.3f %-10.3f %-10.2f %-10.2f\n', ...
            c.name, m_k.settling_time, m_k.max_cart_disp, m_k.peak_force, m_k.ICE);

        results_table{k, 1} = c.name;
        results_table{k, 2} = K_k;
        results_table{k, 3} = m_k.settling_time;
        results_table{k, 4} = m_k.peak_angle_deg;
        results_table{k, 5} = m_k.max_cart_disp;
        results_table{k, 6} = m_k.peak_force;
        results_table{k, 7} = m_k.ISE;
        results_table{k, 8} = m_k.ICE;

        % Subplot 1: Pendulum Angle
        subplot(3, 1, 1);
        plot(res_k.t, res_k.theta_deg, 'Color', c.color, 'LineWidth', 1.8); hold on;

        % Subplot 2: Cart Position
        subplot(3, 1, 2);
        plot(res_k.t, res_k.p, 'Color', c.color, 'LineWidth', 1.8); hold on;

        % Subplot 3: Actuator Force
        subplot(3, 1, 3);
        plot(res_k.t, res_k.u, 'Color', c.color, 'LineWidth', 1.8); hold on;
    end
    fprintf('========================================================================================\n\n');

    % Polish plots
    subplot(3, 1, 1);
    grid on; yline(0, 'k:');
    ylabel('\theta [deg]', 'FontSize', 10, 'FontWeight', 'bold');
    title('Pendulum Angle Trajectory \theta(t) vs LQR Weightings', 'FontSize', 11, 'FontWeight', 'bold');
    legend({cases.name}, 'Location', 'northeast', 'FontSize', 8);

    subplot(3, 1, 2);
    grid on; yline(0, 'k:');
    yline(params.track_lim, 'r--', 'Track Limit');
    yline(-params.track_lim, 'r--');
    ylabel('Position p [m]', 'FontSize', 10, 'FontWeight', 'bold');
    title('Cart Displacement p(t) vs LQR Weightings', 'FontSize', 11, 'FontWeight', 'bold');
    legend({cases.name}, 'Location', 'northeast', 'FontSize', 8);

    subplot(3, 1, 3);
    grid on; yline(0, 'k:');
    yline(params.Fmax, 'r--', 'F_{max} (+15N)');
    yline(-params.Fmax, 'r--', '-F_{max} (-15N)');
    xlabel('Time [s]', 'FontSize', 10, 'FontWeight', 'bold');
    ylabel('Force u [N]', 'FontSize', 10, 'FontWeight', 'bold');
    title('Actuator Control Force Demand u(t) vs LQR Weightings', 'FontSize', 11, 'FontWeight', 'bold');
    legend({cases.name}, 'Location', 'northeast', 'FontSize', 8);

    % Save results
    if save_results
        plot_path = fullfile('results', 'plots', 'lqr_q_r_sensitivity.png');
        saveas(fig, plot_path);
        fprintf('Saved sensitivity plot: %s\n', plot_path);

        tbl_path = fullfile('results', 'tables', 'q_r_sensitivity_table.txt');
        fid = fopen(tbl_path, 'w');
        fprintf(fid, 'LQR WEIGHTING SENSITIVITY STUDY RESULTS\n\n');
        fprintf(fid, 'Configuration,Settling_Time_s,Max_Cart_m,Peak_Force_N,ISE,ICE,K1,K2,K3,K4\n');
        for k = 1:N_cases
            K = results_table{k, 2};
            fprintf(fid, '%s,%.3f,%.3f,%.2f,%.4e,%.3f,%.2f,%.2f,%.2f,%.2f\n', ...
                results_table{k, 1}, results_table{k, 3}, results_table{k, 5}, ...
                results_table{k, 6}, results_table{k, 7}, results_table{k, 8}, ...
                K(1), K(2), K(3), K(4));
        end
        fclose(fid);
        fprintf('Saved sensitivity table: %s\n', tbl_path);
    end
end
