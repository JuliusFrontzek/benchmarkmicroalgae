function [st_CtrlSignals, state] = controller_simple_temp_mpc(Timeline, obs, refs, env, future, st_CtrlSignals, state)
% Temperature-only MPC: N=3 horizon. Heat-exchanger flux Qhx held constant,
% optimises Tin_hx only with smooth quadratic tracking + Tin smoothness.
% CO2/air fixed at 0. IPOPT runs at print_level=5 (verbose).

    if isempty(state) || ~isfield(state, 'last_solve_time')
        state.next_run_time   = -1;
        state.last_solve_time = -1;
        state.last_qhx = 0;
        state.last_tin = 30;
        state.sp_announced = false;
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

        % Build (7, N) disturbance forecast matrix (N=3 steps)
        N_horiz = 3;
        safeN = @(v, k) double(v(min(k, numel(v))));
        d_matrix = zeros(7, N_horiz);
        for k = 1:N_horiz
            d_matrix(:, k) = [safeN(future.RadGlobal, k); safeN(future.RadPAR, k); ...
                               safeN(future.Temp_ext, k);  safeN(future.RH, k); ...
                               safeN(future.Wind, k);       0; 0];
        end

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
