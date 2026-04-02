function saved_paths = save_result_plot_windows(results, varargin)
%SAVE_RESULT_PLOT_WINDOWS Render benchmark result figures headlessly and save each one.
%
% This is a drop-in replacement for the current show_results/showResults usage
% when running without a GUI. By default it uses the existing showResults
% plotting routine when available so the saved images match MATLAB's live
% figures, then saves each figure window as its own timestamped PNG beside
% Benchmark_main.m and closes the generated figures afterward.
%
% Example:
%   save_result_plot_windows(results)
%   save_result_plot_windows(results, 'BaseName', 'Benchmark_main')
%   save_result_plot_windows(results, 'Plotter', @my_custom_plotter)
%   save_result_plot_windows(results, 'Plotter', '../show_results.m')

    parser = inputParser;
    parser.FunctionName = mfilename;
    addRequired(parser, 'results', @(x) isstruct(x) && ~isempty(x));
    addParameter(parser, 'OutputDir', fileparts(mfilename('fullpath')), @local_is_text_scalar);
    addParameter(parser, 'BaseName', 'Benchmark_main', @local_is_text_scalar);
    addParameter(parser, 'Plotter', [], @(x) isempty(x) || isa(x, 'function_handle') || local_is_text_scalar(x));
    addParameter(parser, 'PrintSummary', true, @(x) islogical(x) || isnumeric(x));
    addParameter(parser, 'FigureVisibility', 'off', @local_is_text_scalar);
    addParameter(parser, 'CloseFigures', true, @(x) islogical(x) || isnumeric(x));
    addParameter(parser, 'Resolution', 450, @(x) isnumeric(x) && isscalar(x) && x > 0);
    addParameter(parser, 'SaveAllIfNoNewFigures', true, @(x) islogical(x) || isnumeric(x));
    parse(parser, results, varargin{:});
    opts = parser.Results;

    output_dir = char(string(opts.OutputDir));
    if ~exist(output_dir, 'dir')
        mkdir(output_dir);
    end

    if opts.PrintSummary
        local_print_results_summary(results);
    end

    figures_before = findall(groot, 'Type', 'figure');
    previous_visibility = get(groot, 'defaultFigureVisible');
    visibility_cleanup = onCleanup(@() set(groot, 'defaultFigureVisible', previous_visibility));
    set(groot, 'defaultFigureVisible', char(string(opts.FigureVisibility)));

    local_run_plotter(results, opts.Plotter);
    drawnow;

    figures_after = findall(groot, 'Type', 'figure');
    figures_to_save = local_select_figures(figures_before, figures_after, logical(opts.SaveAllIfNoNewFigures));

    timestamp = char(datetime('now', 'Format', 'yyyyMMdd_HHmmss_SSS'));
    base_name = char(string(opts.BaseName));
    saved_paths = cell(numel(figures_to_save), 1);

    for idx = 1:numel(figures_to_save)
        fig = figures_to_save(idx);
        fig_label = local_figure_label(fig, idx);
        file_name = sprintf('%s_%02d_%s_%s.png', base_name, idx, fig_label, timestamp);
        saved_paths{idx} = fullfile(output_dir, file_name);

        try
            drawnow;
            exportgraphics(fig, saved_paths{idx}, 'Resolution', opts.Resolution);
        catch export_err
            warning('save_result_plot_windows:exportgraphicsFailed', ...
                'exportgraphics failed for "%s" (%s). Falling back to saveas.', ...
                fig_label, export_err.message);
            drawnow;
            saveas(fig, saved_paths{idx});
        end

        fprintf('Saved figure %d/%d: %s\n', idx, numel(figures_to_save), saved_paths{idx});
    end

    if logical(opts.CloseFigures) && ~isempty(figures_to_save)
        close(figures_to_save);
    end
end

function local_run_plotter(results, plotter)
    if isempty(plotter)
        if exist('showResults', 'file') == 6 || exist('showResults', 'file') == 2
            showResults;
        else
            local_default_show_results_plots(results);
        end
        return;
    end

    if isa(plotter, 'function_handle')
        plotter(results);
        return;
    end

    plotter_path = char(string(plotter));
    if exist(plotter_path, 'file') ~= 2
        error('save_result_plot_windows:PlotterNotFound', ...
            'Plotter script "%s" was not found.', plotter_path);
    end

    run(plotter_path);
end

function figures_to_save = local_select_figures(figures_before, figures_after, save_all_if_none_new)
    before_ids = double(figures_before(:));
    after_ids = double(figures_after(:));
    is_new = ~ismember(after_ids, before_ids);
    figures_to_save = figures_after(is_new);

    if isempty(figures_to_save) && save_all_if_none_new
        figures_to_save = figures_after;
    end

    if isempty(figures_to_save)
        return;
    end

    fig_numbers = arrayfun(@(h) get(h, 'Number'), figures_to_save);
    [~, order] = sort(fig_numbers);
    figures_to_save = figures_to_save(order);
end

function label = local_figure_label(fig, fallback_idx)
    label = strtrim(string(get(fig, 'Name')));
    if strlength(label) == 0
        label = strtrim(string(get(fig, 'Tag')));
    end
    if strlength(label) == 0
        label = sprintf('figure_%02d', fallback_idx);
    end

    label = lower(char(label));
    label = regexprep(label, '[^a-z0-9]+', '_');
    label = regexprep(label, '^_+|_+$', '');
    if isempty(label)
        label = sprintf('figure_%02d', fallback_idx);
    end
end

function local_print_results_summary(results)
    fprintf('\nSimulation finished.\n');

    fprintf('\nDynamic cost functions (J):\n');
    fprintf('   %7s %20s %14s\n', 'pH', 'Disolved oxygen', 'Temperature');
    fprintf('   %7.4g %20.4g %14.4g\n\n', ...
        results.J.pH, results.J.DO, results.J.T);

    fprintf('Key Performance Indicators (KPIs):\n');
    fprintf('   %-32s %14s %s\n', 'KPI', 'Value', 'Unit');
    fprintf('   %-32s %14s %s\n', repmat('-', 1, 32), repmat('-', 1, 14), repmat('-', 1, 4));

    fprintf('   %-32s %14.3f %s\n', 'Total air injected', results.total_air_L, 'L');
    fprintf('   %-32s %14.3f %s\n', 'Total CO2 injected', results.total_CO2_L, 'L');
    fprintf('   %-32s %14.3f %s\n', 'Total biomass produced', results.prod_g, 'g');
    fprintf('   %-32s %14.4f %s\n', 'Productivity per unit area', results.prod_areal_gm2_day, 'g/m^2/day');
    fprintf('   %-32s %14.3f %s\n', 'Harvested amount', results.harv_total_g, 'g');
    fprintf('   %-32s %14.2f %s\n', 'Production yield ratio', results.harv_frac, '%%');
    fprintf('   %-32s %14.4f %s\n', 'Harvested per unit area', results.harv_prod_areal_gm2_day, 'g/m^2/day');
    fprintf('   %-32s %14.2f %s\n', 'Relative biomass accumulation', results.acumm_rel, '%% of initial');
end

function local_default_show_results_plots(results)
    t_h = results.t / 3600;

    figure('Color', 'w', 'Name', 'Outputs');
    tiledlayout(5, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    plot(t_h, results.pH, 'LineWidth', 1.4);
    hold on;
    yline(results.refs.pH, ':');
    grid on;
    ylabel('pH');
    title('pH');

    nexttile;
    plot(t_h, results.DO, 'LineWidth', 1.4);
    hold on;
    yline(results.refs.DO, ':');
    grid on;
    ylabel('[% sat]');
    title('DO');

    nexttile;
    plot(t_h, results.X_gL, 'LineWidth', 1.4);
    grid on;
    ylabel('[g/L]');
    title('Microalgae');

    nexttile;
    plot(t_h, results.Depth, 'LineWidth', 1.4);
    grid on;
    ylabel('[m]');
    title('Depth');

    nexttile;
    plot(t_h, results.T, 'LineWidth', 1.6);
    grid on;
    ylabel('[^\circC]');
    xlabel('Time [h]');
    title('Raceway Temperature');

    figure('Color', 'w', 'Name', 'Gas injections');
    tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    plot(t_h, results.QCO2_del * 60 * 1000, 'LineWidth', 1.4);
    grid on;
    ylabel('[L/min]');
    title('CO2 (to reactor)');

    nexttile;
    plot(t_h, results.Qair_del * 60 * 1000, 'LineWidth', 1.4);
    grid on;
    ylabel('[L/min]');
    title('Air (to reactor)');

    figure('Color', 'w', 'Name', 'HX actuation');
    tiledlayout(2, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    plot(t_h, results.HX.Qw_m3s * 60 * 1000, 'LineWidth', 1.4);
    grid on;
    ylabel('[L/min]');
    title('Coil-side flow (commanded)');

    nexttile;
    plot(t_h, results.HX.Tin_C, 'LineWidth', 1.4);
    grid on;
    ylabel('[^\circC]');
    xlabel('Time [h]');
    title('HX inlet temperature (commanded)');

    figure('Color', 'w', 'Name', 'Rates & Factors');
    tiledlayout(3, 2, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    plot(t_h, results.mu_I, 'LineWidth', 1.4);
    grid on;
    ylabel('\mu_I [-]');
    title('Light factor \mu_I');

    nexttile;
    plot(t_h, results.mu_T, 'LineWidth', 1.4);
    grid on;
    ylabel('\mu_T [-]');
    title('Temperature factor \mu_T');

    nexttile;
    plot(t_h, results.mu_pH, 'LineWidth', 1.4);
    grid on;
    ylabel('\mu_{pH} [-]');
    title('pH factor \mu_{pH}');

    nexttile;
    plot(t_h, results.mu_DO, 'LineWidth', 1.4);
    grid on;
    ylabel('\mu_{DO} [-]');
    title('DO factor \mu_{DO}');

    nexttile;
    plot(t_h, results.mu, 'LineWidth', 1.4);
    grid on;
    ylabel('\mu [s^{-1}]');
    xlabel('Time [h]');
    title('Growth rate \mu(t)');

    nexttile;
    plot(t_h, results.m, 'LineWidth', 1.4);
    grid on;
    ylabel('m [s^{-1}]');
    xlabel('Time [h]');
    title('Respiration rate m(t)');

    figure('Color', 'w', 'Name', 'Environmental variables');
    tiledlayout(4, 1, 'Padding', 'compact', 'TileSpacing', 'compact');

    nexttile;
    yyaxis left;
    plot(t_h, results.Env.RadG, 'LineWidth', 1.4);
    ylabel('Global [W m^{-2}]');
    grid on;
    yyaxis right;
    plot(t_h, results.Env.RadPAR, 'LineWidth', 1.4);
    ylabel('PAR [\mumol m^{-2} s^{-1}]');
    title('Solar radiation');

    nexttile;
    plot(t_h, results.Env.Temp_ext, 'LineWidth', 1.4);
    grid on;
    ylabel('T_{ext} [ºC]');
    title('Ambient temperature');

    nexttile;
    plot(t_h, results.Env.RH, 'LineWidth', 1.4);
    grid on;
    ylabel('RH [%]');
    title('Relative humidity');

    nexttile;
    plot(t_h, results.Env.Wind, 'LineWidth', 1.4);
    grid on;
    ylabel('U_{wind} [m s^{-1}]');
    xlabel('Time [h]');
    title('Wind speed');
end

function tf = local_is_text_scalar(value)
    tf = (ischar(value) && isrow(value)) || (isstring(value) && isscalar(value));
end
