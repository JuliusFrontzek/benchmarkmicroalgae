function [st_CtrlSignals, state] = controller_simple_temp_do_mpc(Timeline, obs, refs, env, future, st_CtrlSignals, state)
% Combined temperature + dissolved-oxygen MPC. Optimises inlet temperature
% Tin_hx (temperature) and air injection Qair (DO) together; Qhx held at max,
% CO2 fixed (from the co-running pH controller). The horizon N is the single
% source of truth in mpc_handler.py (SIMPLE_TEMPDO_MPC_N), fetched once below.
%
% DO setpoint: own day/night profile (DO_DAY by day, DO_NIGHT by night), chosen
% by the forecast radiation so it switches only twice a day (dawn/dusk) -- the
% benchmark's own DO reference is a flat constant, but a day/night profile that
% lets DO ride low at night is cheaper overall. Weather is resampled onto the
% 5-min MPC grid and the scheduled harvest/dilution events are fed in (so the
% temperature half can pre-heat before cold-water injections).
%
% Bind this one controller to BOTH ctrl.fn_DO_air and ctrl.fn_Temp_HX: it is
% time-gated to solve once per step (the duplicate call hits a Python cache),
% and the benchmark extracts Qair from the DO handle and Qhx/Tin_hx from the
% temperature handle.

    % Radiation threshold separating day from night. Air injection only pulls DO
    % toward ~100 %% saturation, so injecting at night is wasted -- below this the
    % air is gated off inside the MPC (matches the baseline DO PI's rad_threshold).
    RAD_THRESHOLD = 10;     % [W/m^2] forecast RadGlobal above this => "day"

    if isempty(state) || ~isfield(state, 'last_solve_time')
        state.next_run_time   = -1;
        state.last_solve_time = -1;
        state.last_air = 0;
        state.last_qhx = 0;
        state.last_tin = 30;
        state.sp_announced = false;
        % Horizon: single source of truth in mpc_handler.py.
        state.N = double(pyrun("from mpc_handler import SIMPLE_TEMPDO_MPC_N as n", "n"));
        fprintf('[SimpleTempDOMPC] Prediction horizon N = %d (%.0f min lookahead)\n', ...
                state.N, state.N * 5);
    end

    % Resolve stop-file paths relative to this controller so the cwd/path does
    % not matter (drop a file into IFAC_WC_player/ to trigger).
    ctrl_dir = fileparts(mfilename('fullpath'));

    % Graceful stop: hold last inputs constant, let the benchmark run to the
    % end (so the final plots are produced) without calling the MPC again.
    if exist(fullfile(ctrl_dir, 'STOP_but_finish.txt'), 'file')
        st_CtrlSignals.Qair   = state.last_air;
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
    % Track the benchmark's own DO reference (what the benchmark scores against);
    % fall back to 150 %% sat if it is not supplied.
    if isfield(refs, 'DO') && ~isempty(refs.DO)
        DO_setpoint = double(refs.DO);
    else
        DO_setpoint = 150.0;
    end
    if ~state.sp_announced
        fprintf(['[SimpleTempDOMPC] Tracking benchmark refs: T = %.2f degC; ' ...
                 'DO = %.0f %% sat; air gated off below %d W/m^2\n'], ...
                 T_setpoint, DO_setpoint, RAD_THRESHOLD);
        state.sp_announced = true;
    end

    if (currentTime >= state.next_run_time) && (currentTime ~= state.last_solve_time)
        x_curr = [obs.pH, obs.DO, obs.Depth, obs.Xalg_gL, obs.T];

        N_horiz = state.N;
        safeN = @(v, k) double(v(min(k, numel(v))));

        % Scheduled harvest/dilution forecast (rows 6,7): same schedule as the
        % full controller_temp_mpc.m -- 50 min op, dilution starts +20 min,
        % harvest at 09:00 on even days / 12:00 on odd days.
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

        % Resample the weather forecast onto the MPC's 5-min grid (future.* is
        % sampled finer than the MPC step, so indexing by step k would only
        % cover the first few minutes). Clamp to the available range (ZOH).
        use_interp = isfield(future, 't_future') && numel(future.t_future) >= 2;
        if use_interp
            tf = double(future.t_future(:));
            tq = tf(1) + (0:N_horiz-1) * time_step;
            tq = min(max(tq, tf(1)), tf(end));
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

        % Track the benchmark's own DO reference, held flat over the horizon
        % (future.* carries no DO forecast). This is what the benchmark scores.
        DO_ref = DO_setpoint * ones(1, N_horiz);

        % Night air gate: 1 where the forecast says "day" (air allowed), 0 at
        % night so the MPC forces zero air injection (wasted otherwise).
        air_gate = double(RadGlobal_q > RAD_THRESHOLD);

        % CO2 actually applied by the co-running pH controller (threaded through
        % st_CtrlSignals). Missing -> NaN -> Python falls back to 0.
        u_co2 = NaN;
        if isfield(st_CtrlSignals, 'Qco2') && ~isempty(st_CtrlSignals.Qco2)
            u_co2 = double(st_CtrlSignals.Qco2);
        end
        if ~isfinite(u_co2), u_co2 = 0.0; end

        try
            u_python = pyrun( ...
                "from mpc_handler import get_simple_tempdo_mpc_action; " + ...
                "res = get_simple_tempdo_mpc_action(x, d, T_ref, DO_ref, u_co2, u_prev_air, u_prev_qhx, u_prev_tin, air_gate)", ...
                "res", x=x_curr, d=d_matrix, T_ref=T_setpoint, DO_ref=DO_ref, ...
                u_co2=u_co2, u_prev_air=state.last_air, u_prev_qhx=state.last_qhx, ...
                u_prev_tin=state.last_tin, air_gate=air_gate);

            result = double(u_python);
            state.last_air = result(1);
            state.last_qhx = result(2);
            state.last_tin = result(3);
            state.next_run_time   = currentTime + time_step;
            state.last_solve_time = currentTime;
        catch ME
            fprintf('[SimpleTempDOMPC] Python error: %s\n', ME.message);
        end

        % Belt-and-suspenders: guarantee exactly zero applied air at night,
        % regardless of any small solver residual (the MPC already gates it).
        if air_gate(1) <= 0
            state.last_air = 0;
        end
    end

    st_CtrlSignals.Qair   = min(500/1000/60, max(0, state.last_air));
    st_CtrlSignals.Qhx    = min(36/1000/60,  max(0, state.last_qhx));
    st_CtrlSignals.Tin_hx = min(80,          max(0, state.last_tin));
end
