% =========================================================================
% MASTER LAUNCHER: OPTIMAL STABILIZATION OF AN INVERTED PENDULUM (LQR)
% Academic Control Systems Engineering Project & Interactive Simulator
% =========================================================================
%
% Usage:
%   >> main              % Launches interactive mode / menu
%   >> main('app')       % Directly opens the Interactive GUI Simulator
%   >> main('compare')   % Runs LQR vs PID comparative analysis
%   >> main('tune')      % Runs Q & R sensitivity analysis
%   >> main('validate')  % Executes complete engineering verification audit
%   >> main('all')       % Runs all benchmarks and saves figures/tables
%
% =========================================================================

function main(mode)
    clc;
    close all;

    % Configure paths to include all project modules
    project_root = fileparts(mfilename('fullpath'));
    addpath(genpath(project_root));

    % Banner
    fprintf('========================================================================\n');
    fprintf('   OPTIMAL STABILIZATION OF AN INVERTED PENDULUM USING LQR CONTROL\n');
    fprintf('        State-Space Modelling, Optimal Control & Simulator\n');
    fprintf('========================================================================\n');
    fprintf(' MATLAB Version: %s\n', version);
    fprintf(' Project Path  : %s\n', project_root);
    fprintf('========================================================================\n\n');

    % Quick System Summary
    params = parameters();
    cfg    = controller_settings();
    [A, B, C, D] = linear_state_space(params);
    [K, ~, poles, ~] = design_lqr(A, B, cfg.lqr.Q, cfg.lqr.R);

    fprintf('Nominal Physical System:\n');
    fprintf('  Cart Mass (M): %.2f kg | Pendulum Mass (m): %.2f kg | Length (L): %.2f m\n', ...
        params.M, params.m, params.L);
    fprintf('  Cart Friction (b): %.2f N*s/m | Force Limit (Fmax): +/-%.1f N\n', ...
        params.b, params.Fmax);
    fprintf('  Unstable Open-Loop Pole: %+.3f rad/s\n', max(real(eig(A))));
    fprintf('  Computed LQR Gain Vector K: [%.2f, %.2f, %.2f, %.2f]\n\n', K);

    if nargin < 1 || isempty(mode)
        fprintf('Select an option to proceed:\n');
        fprintf('  [1] Launch Interactive GUI Simulator (Default)\n');
        fprintf('  [2] Run LQR vs PID Benchmark Comparison (Overlaid Plots & Table)\n');
        fprintf('  [3] Run LQR Q & R Weighting Sensitivity Study\n');
        fprintf('  [4] Run Full System Engineering Validation Suite (9/9 Tests)\n');
        fprintf('  [5] Run Complete Automated Suite & Save All Artifacts\n');
        fprintf('  [0] Exit\n\n');

        choice = input('Enter your choice [1-5, or 1 to launch]: ', 's');
        if isempty(choice)
            choice = '1';
        end
    else
        switch lower(mode)
            case {'app', 'gui', '1'}
                choice = '1';
            case {'compare', '2'}
                choice = '2';
            case {'tune', 'sensitivity', '3'}
                choice = '3';
            case {'validate', 'test', '4'}
                choice = '4';
            case {'all', '5'}
                choice = '5';
            otherwise
                choice = '1';
        end
    end

    switch choice
        case '1'
            fprintf('\nLaunching Interactive Animated Simulator GUI...\n');
            fig = create_simulator_app();
            fprintf('Simulator running. Use UI controls to start/pause/reset/tune.\n');

        case '2'
            fprintf('\nExecuting LQR vs PID Controller Benchmark...\n');
            compare_controllers(true);

        case '3'
            fprintf('\nExecuting LQR Q and R Parameter Sensitivity Analysis...\n');
            q_r_sensitivity(true);

        case '4'
            fprintf('\nExecuting Engineering Validation Suite...\n');
            validation_tests(true);

        case '5'
            fprintf('\nExecuting Complete Automated Project Suite...\n');
            validation_tests(true);
            compare_controllers(true);
            q_r_sensitivity(true);
            fprintf('Launching Interactive Simulator GUI...\n');
            create_simulator_app();

        case '0'
            fprintf('Exiting.\n');
            return;

        otherwise
            fprintf('Invalid selection. Launching GUI by default...\n');
            create_simulator_app();
    end
end
