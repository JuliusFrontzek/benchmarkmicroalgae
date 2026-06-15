%% Diagnostic: Simplest possible pH MPC — N=3, CO2-only, tracks benchmark pH SP
% MPC controls pH via CO2; PI controllers handle DO and temperature so that
% only pH is under MPC. Use this to validate the pH MPC in isolation.
clear; clc; rng(42,'twister'); close all
addpath('..');
addpath('sources');

% Point MATLAB at the project venv Python (the one with torch/casadi/l4casadi).
% MATLAB's default pyenv is the system python3, which lacks these packages and
% makes the MPC crash with "No module named 'torch'". Must run before any pyrun.
this_dir = fileparts(mfilename('fullpath'));
venv_py  = fullfile(this_dir, '..', '.venv', 'bin', 'python');
if isfile(venv_py)
    pe = pyenv;
    if ~strcmp(pe.Executable, venv_py)
        if pe.Status == "Loaded"
            warning(['Python is already loaded as %s. Restart MATLAB so the ' ...
                     'venv (%s) can be selected.'], pe.Executable, venv_py);
        else
            pyenv('Version', venv_py);
        end
    end
else
    warning('venv Python not found at %s; using default MATLAB pyenv.', venv_py);
end

S = load('Data_Benchmark_ext', 'Data');
Data = S.Data;

ctrl = struct();
ctrl.fn_pH_CO2  = @controller_simple_ph_mpc;
ctrl.fn_DO_air  = @controller_DO_PI;
ctrl.fn_Temp_HX = @controller_Temp_HX_PI;

fprintf('\nRunning simple pH MPC diagnostic (tracking benchmark pH setpoint)...\n');
[results] = simulate_benchmark_model(Data, ctrl);
showResults
save_result_plot_windows(results);
