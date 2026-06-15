function [st_CtrlSignals, state] = controller_simple_temp_mpc(Timeline, obs, refs, env, future, st_CtrlSignals, state)
% Temperature-only MPC. Heat-exchanger flux Qhx held constant, optimises Tin_hx
% only with smooth quadratic tracking + Tin smoothness. CO2/air fixed at 0.
% The prediction horizon N is the single source of truth in mpc_handler.py
% (SIMPLE_TEMP_MPC_N), fetched once below so the forecast width always matches
% the solver. The scheduled harvest/dilution events are forecast and fed in so
% the controller can pre-heat ahead of the ~20 degC cold-water injections.

    if isempty(state) || ~isfield(state, 'last_solve_time')
        state.next_run_time   = -1;
        state.last_solve_time = -1;
        state.last_qhx = 0;
        state.last_tin = 30;
        state.sp_announced = false;
        % Horizon: single source of truth in mpc_handler.py.
        state.N = double(pyrun("from mpc_handler import SIMPLE_TEMP_MPC_N as n", "n"));
        fprintf('[SimpleTempMPC] Prediction horizon N = %d (%.0f min lookahead)\n', ...
                state.N, state.N * 5);
    end

    % Resolve stop-file paths relative to this controller so the cwd/path does
    % not matter (drop a file into IFAC_WC_player/ to trigger).
    ctrl_dir = fileparts(mfilename('fullpath'));

    % Graceful stop: hold last inputs constant, let the benchmark run to the
    % end (so the final plots are produced) without calling the MPC again.
    if exist(fullfile(ctrl_dir, 'STOP_but_finish.txt'), 'file')
        st_CtrlSignals.Qhx    = state.last_qhx;
        st_CtrlSignals.Tin_hx = state.last_tin;
        return;
    end

    if exist(fullfile(ctrl_dir, 'STOP.txt'), 'file')
        error('STOP.txt detected. Terminating simulation.');
    end

    time_step   = 5 * 60;
    currentTime = Timeline.time;

    % Track the benchmark's own temperature setpoint (fallback to 27 degC).
    if isfield(refs, 'T') && ~isempty(refs.T)
        T_setpoint = double(refs.T);
    else
        T_setpoint = 27.0;
    end
    if ~state.sp_announced
        fprintf('[SimpleTempMPC] Tracking benchmark T setpoint = %.2f degC\n', T_setpoint);
        state.sp_announced = true;
    end

    if (currentTime >= state.next_run_time) && (currentTime ~= state.last_solve_time)
        x_curr = [obs.pH, obs.DO, obs.Depth, obs.Xalg_gL, obs.T];

        % One-time sanity check: the forecast is indexed by integer step k and
        % assumed to be on the 5-min MPC grid (matching controller_temp_mpc.m).
        % Print the actual spacing so a mismatch is visible at runtime.
        if ~isfield(state, 'fc_checked')
            state.fc_checked = true;
            if isfield(future, 't_future') && numel(future.t_future) >= 2
                fprintf('[SimpleTempMPC] forecast spacing = %.0f s (MPC step = %d s)\n', ...
                        double(future.t_future(2) - future.t_future(1)), time_step);
            end
        end

        % Build (7, N) disturbance forecast matrix over the configured horizon.
        N_horiz = state.N;
        safeN = @(v, k) double(v(min(k, numel(v))));

        % Scheduled harvest/dilution forecast (rows 6,7): same schedule as the
        % full controller_temp_mpc.m -- 50 min op, dilution starts +20 min,
        % harvest at 09:00 on even days / 12:00 on odd days. Feeding this lets
        % the NN predict the cold-water dip so the MPC can pre-heat.
        op_duration     = 50 * 60;
        dilution_offset = 20 * 60;
        Qd_future = zeros(1, N_horiz);
        Qh_future = zeros(1, N_horiz);
        for k = 1:N_horiz
            t_lookahead = currentTime + (k-1)*time_step;
            sec_day = mod(t_lookahead, 86400);
            day_num = floor(t_lookahead / 86400);
            if mod(day_num, 2) == 0
                h_start = 9 * 3600;
            else
                h_start = 12 * 3600;
            end
            d_start = h_start + dilution_offset;
            if (sec_day >= h_start) && (sec_day < h_start + op_duration)
                Qh_future(k) = 1;
            end
            if (sec_day >= d_start) && (sec_day < d_start + op_duration)
                Qd_future(k) = 1;
            end
        end

        % Resample the weather forecast onto the MPC's 5-min grid. future.* is
        % sampled finer (60 s here) than the MPC step (300 s), so indexing it by
        % step k would only cover the first few minutes. Query at
        % t = forecast_start + (k-1)*time_step, clamped to the available range
        % (zero-order hold past the horizon, like the old safeN behaviour).
        use_interp = isfield(future, 't_future') && numel(future.t_future) >= 2;
        if use_interp
            tf = double(future.t_future(:));
            tq = tf(1) + (0:N_horiz-1) * time_step;
            tq = min(max(tq, tf(1)), tf(end));   % clamp to forecast range
            grid_interp = @(v) interp1(tf, double(v(:)), tq, 'linear');
            RadGlobal_q = grid_interp(future.RadGlobal);
            RadPAR_q    = grid_interp(future.RadPAR);
            Temp_ext_q  = grid_interp(future.Temp_ext);
            RH_q        = grid_interp(future.RH);
            Wind_q      = grid_interp(future.Wind);
        else
            RadGlobal_q = arrayfun(@(k) safeN(future.RadGlobal, k), 1:N_horiz);
            RadPAR_q    = arrayfun(@(k) safeN(future.RadPAR, k),    1:N_horiz);
            Temp_ext_q  = arrayfun(@(k) safeN(future.Temp_ext, k),  1:N_horiz);
            RH_q        = arrayfun(@(k) safeN(future.RH, k),        1:N_horiz);
            Wind_q      = arrayfun(@(k) safeN(future.Wind, k),      1:N_horiz);
        end

        d_matrix = [RadGlobal_q; RadPAR_q; Temp_ext_q; RH_q; Wind_q; ...
                    Qd_future; Qh_future];

        try
            u_python = pyrun( ...
                "from mpc_handler import get_simple_temp_mpc_action; " + ...
                "res = get_simple_temp_mpc_action(x, d, T_ref, u_prev_qhx, u_prev_tin)", ...
                "res", x=x_curr, d=d_matrix, T_ref=T_setpoint, ...
                u_prev_qhx=state.last_qhx, u_prev_tin=state.last_tin);

            result = double(u_python);
            state.last_qhx = result(1);
            state.last_tin = result(2);
            state.next_run_time   = currentTime + time_step;
            state.last_solve_time = currentTime;
        catch ME
            fprintf('[SimpleTempMPC] Python error: %s\n', ME.message);
        end
    end

    st_CtrlSignals.Qhx    = min(36/1000/60, max(0, state.last_qhx));
    st_CtrlSignals.Tin_hx = min(80,         max(0, state.last_tin));
end
