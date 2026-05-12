%% ===== Benchmark Microalgae Raceway Reactor (multi-controller, thermal-coupled) =====
clear; clc; rng(42,'twister');
close all
addpath('sources');

%% ---------------- Data load ----------------
% Expect: Data is an array of structs with fields:
%   Data(i).u(:,1)=Temp_ext[°C], u(:,2)=Rad_global[W/m^2], optionally u(:,3)=RH[%], u(:,4)=Wind[m/s]
%   Data(i).y(:,1)=Depth[m]   (only first sample used as IC if present)
try
    S = load('Data_Benchmark_ext','Data');
    Data = S.Data;
catch
    error('Data_Benchmark_ext.mat not found or missing variable "Data".');
end

%% ---------------- Controllers (one per actuator) ----------------
ctrl = struct();
ctrl.fn_pH_CO2  = @controller_pH;
ctrl.fn_DO_air  = @controller_DO;
ctrl.fn_Temp_HX = @controller_Temp_HX;

%% ---------------- Simulation ----------------
fprintf('\nRunning multi-controller thermal-coupled simulation...\n');
[results] = simulate_benchmark_model(Data, ctrl);

%% ---------------- Results ----------------

showResults
