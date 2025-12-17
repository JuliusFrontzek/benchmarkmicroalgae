clc
fprintf('\nSimulation finished.\n');

%% Dynamic cost functions
fprintf('\nDynamic cost functions (J):\n');
fprintf('   %7s %20s %14s\n', 'pH', 'Disolved oxygen', 'Temperature');
fprintf('   %7.4g %20.4g %14.4g\n\n', ...
        results.J.pH, results.J.DO,results.J.T);

%% Key Performance Indicators (KPIs)
fprintf('Key Performance Indicators (KPIs):\n');
fprintf('   %-32s %14s %s\n', 'KPI', 'Value', 'Unit');
fprintf('   %-32s %14s %s\n', repmat('-',1,32), repmat('-',1,14), repmat('-',1,4));

fprintf('   %-32s %14.3f %s\n', 'Total air injected',              results.total_air_L,            'L');
fprintf('   %-32s %14.3f %s\n', 'Total CO2 injected',              results.total_CO2_L,            'L');

fprintf('   %-32s %14.3f %s\n', 'Total biomass produced',          results.prod_g,                 'g');
fprintf('   %-32s %14.4f %s\n', 'Productivity per unit area',      results.prod_areal_gm2_day,     'g/m^2/day');
fprintf('   %-32s %14.3f %s\n', 'Harvested amount',                results.harv_total_g,           'g');
fprintf('   %-32s %14.2f %s\n', 'Production yield ratio',       results.harv_frac,                 '%');
fprintf('   %-32s %14.4f %s\n', 'Harvested per unit area',         results.harv_prod_areal_gm2_day,'g/m^2/day');
fprintf('   %-32s %14.2f %s\n', 'Relative biomass accumulation',   results.acumm_rel,               '% of initial');

%% ---------------- Plots ----------------
t_h = results.t/3600;

figure('Color','w','Name','Outputs');
tiledlayout(5,1,'Padding','compact','TileSpacing','compact');

nexttile; plot(t_h, results.pH, 'LineWidth',1.4); hold on;
yline(results.refs.pH,':'); grid on; ylabel('pH'); title('pH');

nexttile; plot(t_h, results.DO, 'LineWidth',1.4); hold on;
yline(results.refs.DO,':'); grid on; ylabel('[% sat]'); title('DO');

nexttile; plot(t_h, results.X_gL, 'LineWidth',1.4);
grid on; ylabel('[g/L]'); title('Microalgae');

nexttile; plot(t_h, results.Depth, 'LineWidth',1.4);
grid on; ylabel('[m]'); title('Depth');

nexttile; plot(t_h, results.T, 'LineWidth',1.6);
grid on; ylabel('[^\circC]'); xlabel('Time [h]'); title('Raceway Temperature (simulated)');

figure('Color','w','Name','Gas injections');
tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
nexttile; plot(t_h, results.QCO2_del*60*1000, 'LineWidth',1.4);
grid on; ylabel('[L/min]'); title('CO2 (to reactor)');
nexttile; plot(t_h, results.Qair_del*60*1000, 'LineWidth',1.4);
grid on; ylabel('[L/min]'); title('Air (to reactor)');

figure('Color','w','Name','HX actuation');
tiledlayout(2,1,'Padding','compact','TileSpacing','compact');
nexttile; plot(t_h, results.HX.Qw_m3s*60*1000, 'LineWidth',1.4);
grid on; ylabel('[L/min]'); title('Coil-side flow (commanded)');
nexttile; plot(t_h, results.HX.Tin_C, 'LineWidth',1.4);
grid on; ylabel('[^\circC]'); xlabel('Time [h]'); title('HX inlet temperature (commanded)');

% ===== Growth Rates & Factors =====
figure('Color','w','Name','Rates & Factors');
tiledlayout(4,2,'Padding','compact','TileSpacing','compact');

nexttile; plot(t_h, results.mu_I, 'LineWidth',1.4); grid on;
ylabel('\mu_I [-]'); title('Light factor \mu_I');

nexttile; plot(t_h, results.mu_T, 'LineWidth',1.4); grid on;
ylabel('\mu_T [-]'); title('Temperature factor \mu_T');

nexttile; plot(t_h, results.mu_pH, 'LineWidth',1.4); grid on;
ylabel('\mu_{pH} [-]'); title('pH factor \mu_{pH}');

nexttile; plot(t_h, results.mu_DO, 'LineWidth',1.4); grid on;
ylabel('\mu_{DO} [-]'); title('DO factor \mu_{DO}');

nexttile; plot(t_h, results.P, 'LineWidth',1.4); grid on;
ylabel('P [s^{-1}]'); title('Photosynthesis rate P(t)');

nexttile; plot(t_h, results.mu, 'LineWidth',1.4); grid on;
ylabel('\mu [s^{-1}]'); xlabel('Time [h]'); title('Growth rate \mu(t)');

nexttile; plot(t_h, results.m, 'LineWidth',1.4); grid on;
ylabel('m [s^{-1}]'); xlabel('Time [h]'); title('Growth rate m(t)');

% ===== Environmental variables =====
figure('Color','w','Name','Environmental variables');
tiledlayout(4,1,'Padding','compact','TileSpacing','compact');

nexttile; yyaxis left
plot(t_h, results.Env.RadG, 'LineWidth',1.4); ylabel('Global [W m^{-2}]'); grid on
yyaxis right
plot(t_h, results.Env.RadPAR, 'LineWidth',1.4); ylabel('PAR [\mumol m^{-2} s^{-1}]');
title('Solar radiation');

nexttile; plot(t_h, results.Env.Temp_ext, 'LineWidth',1.4); grid on;
ylabel('T_{ext} [ºC]'); title('Ambient temperature');

nexttile; plot(t_h, results.Env.RH, 'LineWidth',1.4); grid on;
ylabel('RH [%]'); title('Relative humidity');

nexttile; plot(t_h, results.Env.Wind, 'LineWidth',1.4); grid on;
ylabel('U_{wind} [m s^{-1}]'); xlabel('Time [h]'); title('Wind speed');