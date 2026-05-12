%% Diagnostic: Simplest possible MPC — 1-step, quadratic T tracking only
% PI controllers for pH and DO, minimal MPC for temperature.
% Use this to isolate whether L4CasADi+IPOPT integration works at all.
clear; clc; rng(42,'twister'); close all
addpath('..');
addpath('sources');

S = load('Data_Benchmark_ext', 'Data');
Data = S.Data;

ctrl = struct();
ctrl.fn_pH_CO2  = @controller_pH_PI;
ctrl.fn_DO_air  = @controller_DO_PI;
ctrl.fn_Temp_HX = @controller_simple_temp_mpc;

fprintf('\nRunning simple 1-step MPC diagnostic (target 27 degC)...\n');
[results] = simulate_benchmark_model(Data, ctrl);
showResults
save_result_plot_windows(results);
