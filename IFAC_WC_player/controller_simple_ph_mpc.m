function [st_CtrlSignals, state] = controller_simple_ph_mpc(Timeline, obs, refs, env, future, st_CtrlSignals, state)
% pH-only MPC: N=3 horizon. Optimises CO2 injection (Qco2) only, tracking the
% benchmark pH setpoint. Air/Qhx/Tin held fixed inside the NN rollout. Cost
% follows the main MicroalgaeMPC formulation (tracking + control smoothness +
% control consumption). IPOPT runs at print_level=5 (verbose).

    if isempty(state) || ~isfield(state, 'last_solve_time')
        state.next_run_time   = -1;
        state.last_solve_time = -1;
        state.last_co2 = 0;
        state.sp_announced = false;
    end

    % Resolve stop-file paths relative to this controller so the cwd/path does
    % not matter (drop a file into IFAC_WC_player/ to trigger).
    ctrl_dir = fileparts(mfilename('fullpath'));

    % Graceful stop: hold last input constant, let the benchmark run to the
    % end (so the final plots are produced) without calling the MPC again.
    if exist(fullfile(ctrl_dir, 'STOP_but_finish.txt'), 'file')
        st_CtrlSignals.Qco2 = state.last_co2;
        return;
    end

    if exist(fullfile(ctrl_dir, 'STOP.txt'), 'file')
        error('STOP.txt detected. Terminating simulation.');
    end

    time_step   = 5 * 60;
    currentTime = Timeline.time;

    % Track the benchmark's own pH setpoint (fallback to 8.0).
    if isfield(refs, 'pH') && ~isempty(refs.pH)
        pH_setpoint = double(refs.pH);
    else
        pH_setpoint = 8.0;
    end
    if ~state.sp_announced
        fprintf('[SimplePHMPC] Tracking benchmark pH setpoint = %.2f\n', pH_setpoint);
        state.sp_announced = true;
    end

    if (currentTime >= state.next_run_time) && (currentTime ~= state.last_solve_time)
        x_curr = [obs.pH, obs.DO, obs.Depth, obs.Xalg_gL, obs.T];

        % Build (7, N) disturbance forecast matrix. Fetch the horizon from the
        % Python single source of truth so the width always matches the NLP.
        N_horiz = double(pyrun("from mpc_handler import SIMPLE_PH_MPC_N as n", "n"));
        safeN = @(v, k) double(v(min(k, numel(v))));
        d_matrix = zeros(7, N_horiz);
        for k = 1:N_horiz
            d_matrix(:, k) = [safeN(future.RadGlobal, k); safeN(future.RadPAR, k); ...
                               safeN(future.Temp_ext, k);  safeN(future.RH, k); ...
                               safeN(future.Wind, k);       0; 0];
        end

        % Actually-applied secondary actuators from the co-running DO/Temp
        % controllers (threaded through st_CtrlSignals). Feeding these to the
        % pH prediction avoids the model mismatch of assuming training-mean
        % air/temperature. Missing fields -> NaN -> Python falls back to means.
        u_air = NaN; u_qhx = NaN; u_tin = NaN;
        if isfield(st_CtrlSignals, 'Qair')   && ~isempty(st_CtrlSignals.Qair)
            u_air = double(st_CtrlSignals.Qair);
        end
        if isfield(st_CtrlSignals, 'Qhx')    && ~isempty(st_CtrlSignals.Qhx)
            u_qhx = double(st_CtrlSignals.Qhx);
        end
        if isfield(st_CtrlSignals, 'Tin_hx') && ~isempty(st_CtrlSignals.Tin_hx)
            u_tin = double(st_CtrlSignals.Tin_hx);
        end

        try
            u_python = pyrun( ...
                "from mpc_handler import get_simple_ph_mpc_action; " + ...
                "res = get_simple_ph_mpc_action(x, d, pH_ref, u_prev_co2, u_air, u_qhx, u_tin)", ...
                "res", x=x_curr, d=d_matrix, pH_ref=pH_setpoint, ...
                u_prev_co2=state.last_co2, u_air=u_air, u_qhx=u_qhx, u_tin=u_tin);

            result = double(u_python);
            state.last_co2 = result(1);
            state.next_run_time   = currentTime + time_step;
            state.last_solve_time = currentTime;
        catch ME
            fprintf('[SimplePHMPC] Python error: %s\n', ME.message);
        end
    end

    st_CtrlSignals.Qco2 = min(20/1000/60, max(0, state.last_co2));
end
