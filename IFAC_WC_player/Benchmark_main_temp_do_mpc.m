%% Combined Temperature + Dissolved-Oxygen MPC
% Same temperature MPC as the simple-temp diagnostic, extended to also control
% the dissolved oxygen via air injection. One controller
% (controller_simple_temp_do_mpc) is bound to BOTH the DO and the Temp handles:
% it optimises Tin_hx and Qair together (Qhx fixed at max), tracking the
% benchmark temperature setpoint and an own day/night DO profile (150 % sat by
% day, 90 % sat at night). pH stays on the PI controller.
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
ctrl.fn_pH_CO2  = @controller_pH_PI;
ctrl.fn_DO_air  = @controller_simple_temp_do_mpc;
ctrl.fn_Temp_HX = @controller_simple_temp_do_mpc;

fprintf('\nRunning combined Temperature + DO MPC...\n');
[results] = simulate_benchmark_model(Data, ctrl);
showResults
save_result_plot_windows(results);
