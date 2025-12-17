%% ===== Benchmark Microalgae Raceway Reactor =====
clear; clc; rng(42,'twister');
addpath('..');

%% ---------------- Data load ----------------
load_data;

%% ---------------- Controllers (one per actuator) ----------------
ctrl = struct();
ctrl.fn_pH_CO2  = @controller_pH_OnOff;
ctrl.fn_DO_air  = @controller_DO_OnOff;
ctrl.fn_HD      = @controller_HD_fixed;
ctrl.fn_Temp_HX = @controller_Temp_HX_no_control;

%% ---------------- Simulation ----------------
fprintf('\nRunning Benchmark Microalgae Raceway simulation...\n');
[results] = simulate_benchmark_model(Data, ctrl);

%% ---------------- Results ----------------
show_results;