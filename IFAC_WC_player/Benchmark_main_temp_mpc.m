%% ===== Diagnostic: Temperature-Only MPC =====
% Uses PI controllers for pH and DO, MPC only for temperature (target 27°C).
% Purpose: verify that the NN model predicts temperature dynamics correctly.
clear; clc; rng(42,'twister');
close all
addpath('sources');

%% Load disturbance data
try
    S = load('Data_Benchmark_ext', 'Data');
    Data = S.Data;
catch
    error('Data_Benchmark_ext.mat not found or missing variable "Data".');
end

%% Controller assignment
ctrl = struct();
ctrl.fn_pH_CO2  = @controller_pH_PI;
ctrl.fn_DO_air  = @controller_DO_PI;
ctrl.fn_Temp_HX = @controller_temp_mpc;   % temperature-only MPC at 27°C

%% Run simulation
fprintf('\nRunning diagnostic: Temperature-Only MPC (target 27 degC)...\n');
[results] = simulate_benchmark_model(Data, ctrl);

%% Results
showResults
