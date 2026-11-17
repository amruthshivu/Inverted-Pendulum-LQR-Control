function fig = create_simulator_app()
% CREATE_SIMULATOR_APP Programmatically builds the interactive MATLAB GUI
% for simulating, visualizing, and tuning the Inverted Pendulum LQR and PID controllers.
%
% Features:
%   - Real-time 2D animated kinematic canvas
%   - Live multi-channel dynamic response plots
%   - Interactive Q and R tuning sliders with real-time CARE gain calculation
%   - PID gain controls
%   - Initial condition configuration
%   - Start, Pause, Resume, Reset, and Stop controls
%   - Quantitative performance metrics dashboard

    % Load nominal parameters and configuration
    params = parameters();
    cfg    = controller_settings();

    % Compute initial LQR gains
    [A, B, ~, ~] = linear_state_space(params);
    [K_init, ~, poles_init, ~] = design_lqr(A, B, cfg.lqr.Q, cfg.lqr.R);

    % Application State Struct (stored in figure UserData)
    app.params      = params;
    app.cfg         = cfg;
    app.A           = A;
    app.B           = B;
    app.K           = K_init;
    app.sim_res     = [];
    app.anim_idx    = 1;
    app.is_running  = false;
    app.is_paused   = false;
    app.timer       = [];
    app.anim_handles = [];

    % Create Main Window
    fig = uifigure('Name', 'Optimal Stabilization of an Inverted Pendulum - LQR Simulator', ...
                   'Position', [50, 50, 1400, 850], ...
                   'Color', [0.95, 0.95, 0.96]);

    % Main 2-column Grid Layout (Left: Visualization & Plots, Right: Control Dashboard)
    main_grid = uigridlayout(fig, [1, 2]);
    main_grid.ColumnWidth = {'1.2x', '1x'};
    main_grid.Padding = [10, 10, 10, 10];
    main_grid.ColumnSpacing = 10;

    % =========================================================================
    % LEFT COLUMN: Kinematic Animation Viewport & Dynamic Time-Series Plots
    % =========================================================================
    left_panel = uipanel(main_grid, 'BackgroundColor', 'w', 'BorderType', 'none');
    left_grid = uigridlayout(left_panel, [2, 1]);
    left_grid.RowHeight = {'1x', '1.1x'};
    left_grid.Padding = [5, 5, 5, 5];
    left_grid.RowSpacing = 8;

    % Top: Kinematic Animation Viewport
    app.anim_ax = uiaxes(left_grid);
    app.anim_ax.BackgroundColor = [0.98, 0.98, 1.0];
    title(app.anim_ax, 'Inverted Pendulum Kinematic Animation', 'FontSize', 12, 'FontWeight', 'bold');

    % Bottom: 4 Dynamic Response Plots (2x2 grid)
    plot_panel = uipanel(left_grid, 'Title', 'Dynamic Closed-Loop System Responses', ...
        'FontWeight', 'bold', 'FontSize', 11, 'BackgroundColor', 'w');
    plot_grid = uigridlayout(plot_panel, [2, 2]);
    plot_grid.RowHeight = {'1x', '1x'};
    plot_grid.ColumnWidth = {'1x', '1x'};
    plot_grid.Padding = [5, 5, 5, 5];

    app.ax_theta = uiaxes(plot_grid);
    title(app.ax_theta, 'Pendulum Angle \theta(t) [deg]', 'FontSize', 9);
    grid(app.ax_theta, 'on');

    app.ax_p = uiaxes(plot_grid);
    title(app.ax_p, 'Cart Position p(t) [m]', 'FontSize', 9);
    grid(app.ax_p, 'on');

    app.ax_u = uiaxes(plot_grid);
    title(app.ax_u, 'Actuator Force u(t) [N]', 'FontSize', 9);
    grid(app.ax_u, 'on');

    app.ax_omega = uiaxes(plot_grid);
    title(app.ax_omega, 'Angular Velocity d\theta/dt [rad/s]', 'FontSize', 9);
    grid(app.ax_omega, 'on');

    app.plot_axes.ax_theta = app.ax_theta;
    app.plot_axes.ax_p     = app.ax_p;
    app.plot_axes.ax_u     = app.ax_u;
    app.plot_axes.ax_omega = app.ax_omega;

    % =========================================================================
    % RIGHT COLUMN: Interactive Control Dashboard
    % =========================================================================
    right_panel = uipanel(main_grid, 'Title', 'Control Dashboard & Telemetry', ...
        'FontWeight', 'bold', 'FontSize', 12, 'BackgroundColor', [0.97, 0.97, 0.98]);
    right_grid = uigridlayout(right_panel, [6, 1]);
    right_grid.RowHeight = {50, 160, 85, 65, 80, 220};
    right_grid.Padding = [8, 8, 8, 8];
    right_grid.RowSpacing = 8;

    % -------------------------------------------------------------------------
    % Section B: Controller Selection
    % -------------------------------------------------------------------------
    app.bg_ctrl = uibuttongroup(right_grid, 'Title', '1. Controller Selection', ...
        'FontWeight', 'bold', 'BackgroundColor', 'w');
    app.rb_lqr = uiradiobutton(app.bg_ctrl, 'Text', 'Optimal LQR', 'Position', [15, 8, 120, 22], 'Value', 1);
    app.rb_pid = uiradiobutton(app.bg_ctrl, 'Text', 'Baseline PID', 'Position', [155, 8, 120, 22], 'Value', 0);
    app.rb_ol  = uiradiobutton(app.bg_ctrl, 'Text', 'Open-Loop', 'Position', [295, 8, 120, 22], 'Value', 0);

    % -------------------------------------------------------------------------
    % Section C: LQR Weight Tuning (q1, q2, q3, q4, R)
    % -------------------------------------------------------------------------
    app.lqr_panel = uipanel(right_grid, 'Title', '2. LQR Tuning (Weights Q & R)', ...
        'FontWeight', 'bold', 'BackgroundColor', 'w');
    lqr_grid = uigridlayout(app.lqr_panel, [5, 3]);
    lqr_grid.RowHeight = {22, 22, 22, 22, 28};
    lqr_grid.ColumnWidth = {110, '1x', 65};
    lqr_grid.Padding = [5, 2, 5, 2];
    lqr_grid.RowSpacing = 3;

    % q1 (Cart position)
    uilabel(lqr_grid, 'Text', 'q1 (Cart Pos):', 'FontSize', 10);
    app.sld_q1 = uislider(lqr_grid, 'Limits', [0.1, 100], 'Value', cfg.lqr.q1);
    app.num_q1 = uieditfield(lqr_grid, 'numeric', 'Value', cfg.lqr.q1, 'Limits', [0, 1000]);

    % q2 (Cart velocity)
    uilabel(lqr_grid, 'Text', 'q2 (Cart Vel):', 'FontSize', 10);
    app.sld_q2 = uislider(lqr_grid, 'Limits', [0.1, 50], 'Value', cfg.lqr.q2);
    app.num_q2 = uieditfield(lqr_grid, 'numeric', 'Value', cfg.lqr.q2, 'Limits', [0, 1000]);

    % q3 (Pendulum angle)
    uilabel(lqr_grid, 'Text', 'q3 (Pend Angle):', 'FontSize', 10);
    app.sld_q3 = uislider(lqr_grid, 'Limits', [1, 500], 'Value', cfg.lqr.q3);
    app.num_q3 = uieditfield(lqr_grid, 'numeric', 'Value', cfg.lqr.q3, 'Limits', [0, 5000]);

    % q4 (Angular rate)
    uilabel(lqr_grid, 'Text', 'q4 (Angular Vel):', 'FontSize', 10);
    app.sld_q4 = uislider(lqr_grid, 'Limits', [0.1, 50], 'Value', cfg.lqr.q4);
    app.num_q4 = uieditfield(lqr_grid, 'numeric', 'Value', cfg.lqr.q4, 'Limits', [0, 1000]);

    % R (Control effort)
    uilabel(lqr_grid, 'Text', 'R (Effort Cost):', 'FontSize', 10);
    app.sld_R  = uislider(lqr_grid, 'Limits', [0.001, 1.0], 'Value', cfg.lqr.R);
    app.num_R  = uieditfield(lqr_grid, 'numeric', 'Value', cfg.lqr.R, 'Limits', [1e-5, 100]);

    % Link slider and edit fields
    app.sld_q1.ValueChangedFcn = @(s, e) sync_edit(app.num_q1, s.Value, fig);
    app.num_q1.ValueChangedFcn = @(e, ev) sync_slider(app.sld_q1, e.Value, fig);
    app.sld_q2.ValueChangedFcn = @(s, e) sync_edit(app.num_q2, s.Value, fig);
    app.num_q2.ValueChangedFcn = @(e, ev) sync_slider(app.sld_q2, e.Value, fig);
    app.sld_q3.ValueChangedFcn = @(s, e) sync_edit(app.num_q3, s.Value, fig);
    app.num_q3.ValueChangedFcn = @(e, ev) sync_slider(app.sld_q3, e.Value, fig);
    app.sld_q4.ValueChangedFcn = @(s, e) sync_edit(app.num_q4, s.Value, fig);
    app.num_q4.ValueChangedFcn = @(e, ev) sync_slider(app.sld_q4, e.Value, fig);
    app.sld_R.ValueChangedFcn  = @(s, e) sync_edit(app.num_R, s.Value, fig);
    app.num_R.ValueChangedFcn  = @(e, ev) sync_slider(app.sld_R, e.Value, fig);

    % -------------------------------------------------------------------------
    % Section D: PID Tuning
    % -------------------------------------------------------------------------
    app.pid_panel = uipanel(right_grid, 'Title', '3. PID Controller Tuning', ...
        'FontWeight', 'bold', 'BackgroundColor', 'w');
    pid_grid = uigridlayout(app.pid_panel, [2, 3]);
    pid_grid.RowHeight = {24, 24};
    pid_grid.Padding = [5, 2, 5, 2];

    uilabel(pid_grid, 'Text', 'Kp (Proportional):', 'FontSize', 10);
    uilabel(pid_grid, 'Text', 'Ki (Integral):', 'FontSize', 10);
    uilabel(pid_grid, 'Text', 'Kd (Derivative):', 'FontSize', 10);

    app.num_kp = uieditfield(pid_grid, 'numeric', 'Value', cfg.pid.Kp, 'Limits', [0, 500]);
    app.num_ki = uieditfield(pid_grid, 'numeric', 'Value', cfg.pid.Ki, 'Limits', [0, 100]);
    app.num_kd = uieditfield(pid_grid, 'numeric', 'Value', cfg.pid.Kd, 'Limits', [0, 100]);

    % -------------------------------------------------------------------------
    % Section E: Initial Conditions
    % -------------------------------------------------------------------------
    ic_panel = uipanel(right_grid, 'Title', '4. Initial Conditions & Actuator Constraints', ...
        'FontWeight', 'bold', 'BackgroundColor', 'w');
    ic_grid = uigridlayout(ic_panel, [2, 4]);
    ic_grid.RowHeight = {18, 22};
    ic_grid.Padding = [5, 2, 5, 2];

    uilabel(ic_grid, 'Text', 'p0 [m]:', 'FontSize', 10);
    uilabel(ic_grid, 'Text', 'p_dot0 [m/s]:', 'FontSize', 10);
    uilabel(ic_grid, 'Text', '\theta0 [deg]:', 'FontSize', 10);
    uilabel(ic_grid, 'Text', 'Sat [+/-15N]:', 'FontSize', 10);

    app.num_p0    = uieditfield(ic_grid, 'numeric', 'Value', 0.0, 'Limits', [-1.0, 1.0]);
    app.num_pdot0 = uieditfield(ic_grid, 'numeric', 'Value', 0.0);
    app.num_th0   = uieditfield(ic_grid, 'numeric', 'Value', 10.0, 'Limits', [-45.0, 45.0]);
    app.chk_sat   = uicheckbox(ic_grid, 'Text', 'Enable', 'Value', 1);

    % -------------------------------------------------------------------------
    % Section F: Simulation Execution Controls
    % -------------------------------------------------------------------------
    sim_ctrl_panel = uipanel(right_grid, 'Title', '5. Simulation Controls', ...
        'FontWeight', 'bold', 'BackgroundColor', 'w');
    sim_ctrl_grid = uigridlayout(sim_ctrl_panel, [2, 4]);
    sim_ctrl_grid.RowHeight = {28, 24};
    sim_ctrl_grid.Padding = [5, 2, 5, 2];

    app.btn_start = uibutton(sim_ctrl_grid, 'push', 'Text', '▶ START', ...
        'BackgroundColor', [0.15, 0.65, 0.2], 'FontColor', 'w', 'FontWeight', 'bold');
    app.btn_pause = uibutton(sim_ctrl_grid, 'push', 'Text', '❚❚ PAUSE', ...
        'BackgroundColor', [0.9, 0.6, 0.1], 'FontColor', 'w', 'FontWeight', 'bold');
    app.btn_reset = uibutton(sim_ctrl_grid, 'push', 'Text', '↺ RESET', ...
        'BackgroundColor', [0.3, 0.4, 0.5], 'FontColor', 'w', 'FontWeight', 'bold');
    app.btn_stop  = uibutton(sim_ctrl_grid, 'push', 'Text', '■ STOP', ...
        'BackgroundColor', [0.85, 0.2, 0.2], 'FontColor', 'w', 'FontWeight', 'bold');

    uilabel(sim_ctrl_grid, 'Text', 'Speed:', 'FontSize', 10, 'HorizontalAlignment', 'right');
    app.dd_speed = uidropdown(sim_ctrl_grid, 'Items', {'0.5x (Slow)', '1.0x (Real-time)', '2.0x (Fast)', 'Instant'}, ...
        'Value', '1.0x (Real-time)');
    app.btn_recalc_lqr = uibutton(sim_ctrl_grid, 'push', 'Text', 'Update LQR Gains', ...
        'FontSize', 10, 'FontWeight', 'bold');
    app.btn_recalc_lqr.Layout.Column = [3, 4];

    app.btn_start.ButtonPushedFcn      = @(b, e) on_start_pushed(fig);
    app.btn_pause.ButtonPushedFcn      = @(b, e) on_pause_pushed(fig);
    app.btn_reset.ButtonPushedFcn      = @(b, e) on_reset_pushed(fig);
    app.btn_stop.ButtonPushedFcn       = @(b, e) on_stop_pushed(fig);
    app.btn_recalc_lqr.ButtonPushedFcn = @(b, e) on_recalc_lqr(fig);

    % -------------------------------------------------------------------------
    % Section H: Quantitative Performance Results Dashboard
    % -------------------------------------------------------------------------
    app.metric_panel = uipanel(right_grid, 'Title', '6. Performance Metrics & Stability Status', ...
        'FontWeight', 'bold', 'BackgroundColor', 'w');
    metric_grid = uigridlayout(app.metric_panel, [6, 2]);
    metric_grid.RowHeight = {20, 20, 20, 20, 20, 26};
    metric_grid.ColumnWidth = {'1.2x', '1x'};
    metric_grid.Padding = [5, 2, 5, 2];

    uilabel(metric_grid, 'Text', 'Status:', 'FontWeight', 'bold');
    app.lbl_status = uilabel(metric_grid, 'Text', 'READY', 'FontWeight', 'bold', 'FontColor', [0 0.5 0]);

    uilabel(metric_grid, 'Text', '2% Settling Time:');
    app.lbl_ts = uilabel(metric_grid, 'Text', '-- s');

    uilabel(metric_grid, 'Text', 'Peak Angle Deviation:');
    app.lbl_peak_th = uilabel(metric_grid, 'Text', '-- deg');

    uilabel(metric_grid, 'Text', 'Max Cart Displacement:');
    app.lbl_max_p = uilabel(metric_grid, 'Text', '-- m');

    uilabel(metric_grid, 'Text', 'Integrated Effort (ICE):');
    app.lbl_ice = uilabel(metric_grid, 'Text', '-- N^2*s');

    uilabel(metric_grid, 'Text', 'Active LQR Gain Vector:');
    app.lbl_gain_k = uilabel(metric_grid, 'Text', sprintf('[%.1f, %.1f, %.1f, %.1f]', K_init), ...
        'FontSize', 9, 'FontWeight', 'bold');

    % Store handle structure into figure UserData
    fig.UserData = app;

    % Initial render of the inverted pendulum at rest / initial angle
    x_init = [app.num_p0.Value; app.num_pdot0.Value; deg2rad(app.num_th0.Value); 0.0];
    app.anim_handles = update_animation(app.anim_ax, 0.0, x_init, params, []);
    fig.UserData.anim_handles = app.anim_handles;

    % Ensure clean shutdown of timer on figure close
    fig.CloseRequestFcn = @(src, evt) on_close(fig);
end

% =============================================================================
% UI Callbacks & Logic Handlers
% =============================================================================

function on_ctrl_selection_changed(fig, ev)
    % ev.NewValue is the selected radio button
    % Controller type will be checked dynamically when simulation runs
end

function sync_edit(edit_h, val, fig)
    edit_h.Value = val;
    on_recalc_lqr(fig);
end

function sync_slider(sld_h, val, fig)
    if val >= sld_h.Limits(1) && val <= sld_h.Limits(2)
        sld_h.Value = val;
    end
    on_recalc_lqr(fig);
end

function on_recalc_lqr(fig)
    app = fig.UserData;
    q1 = app.num_q1.Value;
    q2 = app.num_q2.Value;
    q3 = app.num_q3.Value;
    q4 = app.num_q4.Value;
    R  = app.num_R.Value;

    Q = diag([q1, q2, q3, q4]);
    try
        [K, ~, poles, is_stable] = design_lqr(app.A, app.B, Q, R);
        app.K = K;
        app.cfg.lqr.Q = Q;
        app.cfg.lqr.R = R;
        app.lbl_gain_k.Text = sprintf('[%.1f, %.1f, %.1f, %.1f]', K);
        if is_stable
            app.lbl_gain_k.FontColor = [0, 0.5, 0];
        else
            app.lbl_gain_k.FontColor = 'r';
        end
    catch ME
        app.lbl_gain_k.Text = 'CARE Error!';
        app.lbl_gain_k.FontColor = 'r';
    end
    fig.UserData = app;
end

function on_start_pushed(fig)
    app = fig.UserData;

    % Stop any existing running animation
    if ~isempty(app.timer) && isvalid(app.timer)
        stop(app.timer);
        delete(app.timer);
    end

    % Determine controller
    if app.rb_lqr.Value
        ctrl_type = 'lqr';
    elseif app.rb_pid.Value
        ctrl_type = 'pid';
    else
        ctrl_type = 'open_loop';
    end

    % Update PID cfg from inputs
    app.cfg.pid.Kp = app.num_kp.Value;
    app.cfg.pid.Ki = app.num_ki.Value;
    app.cfg.pid.Kd = app.num_kd.Value;

    % Initial conditions
    p0     = app.num_p0.Value;
    pdot0  = app.num_pdot0.Value;
    th0    = deg2rad(app.num_th0.Value);
    thdot0 = 0.0;
    x0 = [p0; pdot0; th0; thdot0];

    enable_sat = logical(app.chk_sat.Value);

    % Update UI status
    app.lbl_status.Text = 'SOLVING ODE...';
    app.lbl_status.FontColor = [0.1, 0.4, 0.8];
    drawnow;

    % Run numerical solver
    res = run_simulation(ctrl_type, app.params, app.cfg, x0, [], enable_sat);
    metrics = calculate_metrics(res);

    % Update metric dashboard
    if metrics.is_stabilized
        app.lbl_status.Text = 'STABILIZED';
        app.lbl_status.FontColor = [0, 0.6, 0];
        app.lbl_ts.Text = sprintf('%.2f s', metrics.settling_time);
    else
        app.lbl_status.Text = 'UNSTABLE / FALLEN';
        app.lbl_status.FontColor = [0.85, 0.1, 0.1];
        app.lbl_ts.Text = 'Did not settle';
    end

    app.lbl_peak_th.Text = sprintf('%.2f deg', metrics.peak_angle_deg);
    app.lbl_max_p.Text   = sprintf('%.3f m', metrics.max_cart_disp);
    app.lbl_ice.Text     = sprintf('%.2f N^2*s', metrics.ICE);

    app.sim_res = res;
    app.anim_idx = 1;
    app.is_running = true;
    app.is_paused = false;

    % Check animation speed
    speed_str = app.dd_speed.Value;
    if strcmp(speed_str, 'Instant')
        % Immediately render end state and full plots
        app.anim_handles = update_animation(app.anim_ax, res.t(end), res.x(end, :)', app.params, app.anim_handles);
        update_plots(app.plot_axes, res.t, res.x, res.u, app.params, app.cfg);
        app.is_running = false;
        fig.UserData = app;
        return;
    end

    % Determine playback frame skip factor
    switch speed_str
        case '0.5x (Slow)'
            frame_skip = 1;
            timer_period = 0.02;
        case '1.0x (Real-time)'
            frame_skip = 2;
            timer_period = 0.02;
        case '2.0x (Fast)'
            frame_skip = 4;
            timer_period = 0.02;
    end

    % Create animated playback timer
    t_obj = timer('ExecutionMode', 'fixedRate', ...
                  'Period', timer_period, ...
                  'BusyMode', 'drop', ...
                  'TimerFcn', @(src, evt) timer_step(fig, frame_skip));
    app.timer = t_obj;
    fig.UserData = app;

    start(t_obj);
end

function timer_step(fig, frame_skip)
    if ~isvalid(fig)
        return;
    end
    app = fig.UserData;
    if ~app.is_running || app.is_paused
        return;
    end

    res = app.sim_res;
    idx = app.anim_idx;
    N = length(res.t);

    if idx > N
        stop(app.timer);
        app.is_running = false;
        fig.UserData = app;
        return;
    end

    % Update animation viewport
    t_curr = res.t(idx);
    x_curr = res.x(idx, :)';
    app.anim_handles = update_animation(app.anim_ax, t_curr, x_curr, app.params, app.anim_handles);

    % Update plots incrementally or every few frames
    if mod(idx, frame_skip * 4) == 0 || idx >= N
        update_plots(app.plot_axes, res.t(1:idx), res.x(1:idx, :), res.u(1:idx), app.params, app.cfg);
    end

    app.anim_idx = idx + frame_skip;
    fig.UserData = app;
end

function on_pause_pushed(fig)
    app = fig.UserData;
    if app.is_running
        app.is_paused = ~app.is_paused;
        if app.is_paused
            app.btn_pause.Text = '▶ RESUME';
            app.lbl_status.Text = 'PAUSED';
            app.lbl_status.FontColor = [0.8, 0.5, 0];
        else
            app.btn_pause.Text = '❚❚ PAUSE';
            app.lbl_status.Text = 'RUNNING';
            app.lbl_status.FontColor = [0, 0.5, 0.8];
        end
        fig.UserData = app;
    end
end

function on_stop_pushed(fig)
    app = fig.UserData;
    if ~isempty(app.timer) && isvalid(app.timer)
        stop(app.timer);
        delete(app.timer);
    end
    app.is_running = false;
    app.is_paused = false;
    app.lbl_status.Text = 'STOPPED';
    app.lbl_status.FontColor = [0.8, 0.2, 0.2];
    fig.UserData = app;
end

function on_reset_pushed(fig)
    app = fig.UserData;
    on_stop_pushed(fig);
    app = fig.UserData;

    % Reset to initial condition
    p0  = app.num_p0.Value;
    th0 = deg2rad(app.num_th0.Value);
    x_init = [p0; 0; th0; 0];

    app.anim_handles = update_animation(app.anim_ax, 0.0, x_init, app.params, app.anim_handles);

    % Clear dynamic plots
    cla(app.ax_theta);
    cla(app.ax_p);
    cla(app.ax_u);
    cla(app.ax_omega);

    app.lbl_status.Text = 'READY';
    app.lbl_status.FontColor = [0, 0.5, 0];
    app.lbl_ts.Text = '-- s';
    app.lbl_peak_th.Text = '-- deg';
    app.lbl_max_p.Text = '-- m';
    app.lbl_ice.Text = '-- N^2*s';

    fig.UserData = app;
end

function on_close(fig)
    app = fig.UserData;
    if isstruct(app) && isfield(app, 'timer') && ~isempty(app.timer) && isvalid(app.timer)
        stop(app.timer);
        delete(app.timer);
    end
    delete(fig);
end
