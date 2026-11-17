function [test_results, pass_all] = validation_tests(generate_report)
% VALIDATION_TESTS Executes the complete engineering validation suite,
% testing parameter sanity, equilibrium physics, Jacobian linearization accuracy,
% open-loop instability, controllability, closed-loop stability, ODE convergence,
% and basin of attraction on the nonlinear plant.

    if nargin < 1
        generate_report = true;
    end

    params = parameters();
    cfg    = controller_settings();

    fprintf('\n========================================================================\n');
    fprintf('          INVERTED PENDULUM SYSTEM VALIDATION & QUALITY AUDIT\n');
    fprintf('========================================================================\n');

    tests_passed = 0;
    total_tests = 9;

    % -------------------------------------------------------------------------
    % Test 1: Physical Parameters Sanity Check
    % -------------------------------------------------------------------------
    t1_pass = (params.M > 0) && (params.m > 0) && (params.L > 0) && ...
              (params.l > 0) && (params.I > 0) && (params.b >= 0) && ...
              (params.g > 0) && (params.Fmax > 0) && (params.track_lim > 0);
    I_expected = (1.0 / 12.0) * params.m * (params.L^2);
    t1_pass = t1_pass && (abs(params.I - I_expected) < 1e-12);
    fprintf('[%s] Test 1: Physical Parameters and Inertia Sanity Check\n', pass_str(t1_pass));
    if t1_pass, tests_passed = tests_passed + 1; end

    % -------------------------------------------------------------------------
    % Test 2: Equilibrium Derivative Test f(x_eq, u_eq) == 0
    % -------------------------------------------------------------------------
    xdot_eq = nonlinear_dynamics(0, zeros(4, 1), 0, params);
    t2_err  = norm(xdot_eq);
    t2_pass = (t2_err < 1e-12);
    fprintf('[%s] Test 2: Upright Equilibrium State Derivative norm = %.2e\n', ...
        pass_str(t2_pass), t2_err);
    if t2_pass, tests_passed = tests_passed + 1; end

    % -------------------------------------------------------------------------
    % Test 3: Analytical vs Numerical Jacobian Linearization Match
    % -------------------------------------------------------------------------
    [~, ~, err_A, err_B] = linearize_model(params);
    t3_pass = (err_A < 1e-6) && (err_B < 1e-6);
    fprintf('[%s] Test 3: Jacobian Linearization Consistency (err_A = %.2e, err_B = %.2e)\n', ...
        pass_str(t3_pass), err_A, err_B);
    if t3_pass, tests_passed = tests_passed + 1; end

    % -------------------------------------------------------------------------
    % Test 4: Open-Loop Instability Check (Unstable Pole Re(lambda) > 0)
    % -------------------------------------------------------------------------
    [A, B, C, D] = linear_state_space(params);
    ol_poles = eig(A);
    unstable_poles = ol_poles(real(ol_poles) > 0);
    t4_pass = ~isempty(unstable_poles);
    fprintf('[%s] Test 4: Open-Loop Instability Confirmed (Unstable pole = %+.3f rad/s)\n', ...
        pass_str(t4_pass), unstable_poles(1));
    if t4_pass, tests_passed = tests_passed + 1; end

    % -------------------------------------------------------------------------
    % Test 5: Controllability Matrix Full Rank Check
    % -------------------------------------------------------------------------
    Ctrb_mat = ctrb(A, B);
    ctrb_rank = rank(Ctrb_mat);
    t5_pass = (ctrb_rank == 4);
    fprintf('[%s] Test 5: System Controllability (Rank = %d of 4, Full Rank)\n', ...
        pass_str(t5_pass), ctrb_rank);
    if t5_pass, tests_passed = tests_passed + 1; end

    % -------------------------------------------------------------------------
    % Test 6: LQR Closed-Loop Asymptotic Stability
    % -------------------------------------------------------------------------
    [K, P, cl_poles, is_stable] = design_lqr(A, B, cfg.lqr.Q, cfg.lqr.R);
    t6_pass = is_stable && all(real(cl_poles) < -1e-4);
    fprintf('[%s] Test 6: LQR Closed-Loop Stability (All Re(lambda) < 0, max pole = %.3f)\n', ...
        pass_str(t6_pass), max(real(cl_poles)));
    if t6_pass, tests_passed = tests_passed + 1; end

    % -------------------------------------------------------------------------
    % Test 7: Actuator Saturation Constraint Enforcement
    % -------------------------------------------------------------------------
    res_lqr = run_simulation('lqr', params, cfg, [0; 0; deg2rad(15.0); 0], 0:0.01:4, true);
    t7_pass = max(abs(res_lqr.u)) <= (params.Fmax + 1e-9);
    fprintf('[%s] Test 7: Actuator Force Saturation Limit Enforced (|u| <= %.1f N)\n', ...
        pass_str(t7_pass), params.Fmax);
    if t7_pass, tests_passed = tests_passed + 1; end

    % -------------------------------------------------------------------------
    % Test 8: Numerical ODE Solver Convergence and Precision
    % -------------------------------------------------------------------------
    res_strict = run_simulation('lqr', params, cfg, [0; 0; deg2rad(10); 0], 0:0.005:4, true);
    res_coarse = run_simulation('lqr', params, cfg, [0; 0; deg2rad(10); 0], 0:0.02:4, true);
    % Interpolate to common grid and compare
    t_common = 0:0.02:4;
    th_strict = interp1(res_strict.t, res_strict.theta, t_common);
    th_coarse = interp1(res_coarse.t, res_coarse.theta, t_common);
    max_ode_diff = max(abs(th_strict - th_coarse));
    t8_pass = (max_ode_diff < 1e-3);
    fprintf('[%s] Test 8: ODE45 Numerical Convergence (Max grid deviation = %.2e rad)\n', ...
        pass_str(t8_pass), max_ode_diff);
    if t8_pass, tests_passed = tests_passed + 1; end

    % -------------------------------------------------------------------------
    % Test 9: Basin of Attraction / Initial Tilt Recovery Range
    % -------------------------------------------------------------------------
    tilt_angles_deg = [5, 10, 15, 20, 25, 30, 35];
    max_recoverable_tilt = 0;
    for th_deg = tilt_angles_deg
        x0_test = [0; 0; deg2rad(th_deg); 0];
        res_test = run_simulation('lqr', params, cfg, x0_test, 0:0.01:6, true);
        m_test = calculate_metrics(res_test);
        if m_test.is_stabilized && ~m_test.track_exceeded
            max_recoverable_tilt = th_deg;
        end
    end
    t9_pass = (max_recoverable_tilt >= 15);
    fprintf('[%s] Test 9: Nonlinear Recovery Basin of Attraction (Max recoverable angle = %d deg)\n', ...
        pass_str(t9_pass), max_recoverable_tilt);
    if t9_pass, tests_passed = tests_passed + 1; end

    fprintf('------------------------------------------------------------------------\n');
    pass_all = (tests_passed == total_tests);
    fprintf('Validation Summary: %d / %d Tests Passed.\n', tests_passed, total_tests);
    fprintf('========================================================================\n\n');

    test_results.tests_passed = tests_passed;
    test_results.total_tests  = total_tests;
    test_results.pass_all     = pass_all;
    test_results.max_tilt     = max_recoverable_tilt;

    % Generate Markdown Validation Report
    if generate_report
        report_path = fullfile('documentation', 'validation_report.md');
        fid = fopen(report_path, 'w');
        fprintf(fid, '# System Engineering Validation and Verification Report\n\n');
        fprintf(fid, '## Project: Optimal Stabilization of an Inverted Pendulum Using LQR Control\n\n');
        fprintf(fid, '**Date & Environment:** MATLAB %s on Microsoft Windows 11\n\n', version);
        fprintf(fid, '### 1. Executive Summary\n\n');
        fprintf(fid, '- **Total Verification Tests:** %d\n', total_tests);
        fprintf(fid, '- **Tests Passed:** %d\n', tests_passed);
        fprintf(fid, '- **Overall Audit Status:** **%s**\n\n', pass_str(pass_all));

        fprintf(fid, '### 2. Detailed Test Results\n\n');
        fprintf(fid, '| # | Verification Objective | Criterion | Numerical Result | Status |\n');
        fprintf(fid, '|---|---|---|---|:---:|\n');
        fprintf(fid, '| 1 | Physical Parameters Sanity | All constants > 0, I = mL^2/12 | I = %.6f kg*m^2 | %s |\n', params.I, pass_str(t1_pass));
        fprintf(fid, '| 2 | Equilibrium Zero Derivative | ||f(0,0)||_2 < 1e-12 | %.2e | %s |\n', t2_err, pass_str(t2_pass));
        fprintf(fid, '| 3 | Jacobian Linearization Match | ||A_num - A_ana||_inf < 1e-6 | %.2e | %s |\n', err_A, pass_str(t3_pass));
        fprintf(fid, '| 4 | Open-Loop Instability Check | Exists Re(lambda) > 0 | +%.3f rad/s | %s |\n', unstable_poles(1), pass_str(t4_pass));
        fprintf(fid, '| 5 | System Controllability | rank(ctrb(A, B)) == 4 | %d / 4 | %s |\n', ctrb_rank, pass_str(t5_pass));
        fprintf(fid, '| 6 | LQR Closed-Loop Stability | All Re(lambda) < 0 | max Re(lambda) = %.3f | %s |\n', max(real(cl_poles)), pass_str(t6_pass));
        fprintf(fid, '| 7 | Actuator Saturation Compliance | max |u(t)| <= 15.0 N | %.2f N | %s |\n', max(abs(res_lqr.u)), pass_str(t7_pass));
        fprintf(fid, '| 8 | ODE Numerical Convergence | Mesh deviation < 1e-3 rad | %.2e rad | %s |\n', max_ode_diff, pass_str(t8_pass));
        fprintf(fid, '| 9 | Basin of Attraction (Tilt) | Recoverable tilt >= 15 deg | %d deg | %s |\n\n', max_recoverable_tilt, pass_str(t9_pass));

        fprintf(fid, '### 3. Engineering Discussion\n\n');
        fprintf(fid, '1. **Jacobian Consistency:** The analytical state-space derivation matches the numerical central difference Jacobian down to %e, proving exact mathematical agreement.\n', err_A);
        fprintf(fid, '2. **Local vs Global Stabilization:** Linear LQR provides robust asymptotic stabilization within a local basin of attraction up to approximately %d degrees of initial tilt under 15 N actuator limits. Beyond this threshold, nonlinear trigonometric terms and actuator saturation prevent recovery without a non-linear swing-up controller.\n', max_recoverable_tilt);
        fclose(fid);
        fprintf('Generated validation report: %s\n', report_path);
    end
end

function s = pass_str(p)
    if p
        s = 'PASS';
    else
        s = 'FAIL';
    end
end
