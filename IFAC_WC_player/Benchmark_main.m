%% ===== Benchmark Microalgae Raceway Reactor (multi-controller, thermal-coupled) =====
clear; clc; rng(42,'twister');
close all

% 1. FIX PATHS: Add parent and sibling folders
addpath('..');           % For simulate_benchmark_model and plots
addpath('../player');    % For controller_HD_fixed
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
% ctrl.fn_pH_CO2  = @controller_pH;
% ctrl.fn_DO_air  = @controller_DO;
% ctrl.fn_Temp_HX = @controller_Temp_HX;
ctrl.fn_pH_CO2  = @controller_mpc_l4c;
ctrl.fn_DO_air  = @controller_mpc_l4c;
ctrl.fn_Temp_HX = @controller_mpc_l4c;
ctrl.fn_HD      = @controller_HD_fixed;

%% ---------------- Simulation ----------------
% The simulation loop will now call the same MPC function for each sub-system.
% Since the Python side uses a global 'instance', the solver is only loaded once.
fprintf('\nRunning Unified L4CasADi MPC simulation...\n');

%% ---------------- Simulation ----------------
fprintf('\nRunning multi-controller thermal-coupled simulation...\n');
[results] = simulate_benchmark_model(Data, ctrl);

%% ---------------- Results ----------------

% %% ---------------- Save Specific Fields to CSV ----------------
% % Extract the fields you need
% % Note: Using (:) ensures they are treated as column vectors

% % Measurements
% pH_data    = results.pH(:);
% DO_data = results.DO(:);
% depth_data = results.Depth(:);
% X_gl_data = results.X_gl(:);
% T_data = results.T(:);

% % Inputs
% Qco2_data = results.Qco2_del(:);
% Qair_data = results.Qair_del(:);
% Qd_data = results.Qd(:);
% Qh_data = results.Qh(:);
% Qw_data = results.HX.Qw_m3s(:);
% Thx_in_data = rresults.HX.Tin_C(:);


% all_u_data = vertcat(Data.u);

% % 2. Convert to a table with descriptive names
% % Based on your script comments, the columns are Temp, Rad, RH, Wind...
% % Adjust the names below to match your actual 5 columns.
% dataNames = {'Temp_ext', 'Rad_global', 'RelativeHumidity', 'WindSpeed', 'Column5'};
% dataTable = array2table(all_u_data, 'VariableNames', dataNames);


% % If you have a time vector, it's usually good to include it
% % time_data = results.t(:);

% % Create the table
% % 'VariableNames' defines the headers in the CSV
% outTable = table(pH_data, depth_data, ...
%     'VariableNames', {'pH', 'Depth'});

% % Write to file
% writetable(outTable, 'simulation_results.csv');
% fprintf('Successfully saved pH and Depth to simulation_results.csv\n');

showResults
save_result_plot_windows(results);