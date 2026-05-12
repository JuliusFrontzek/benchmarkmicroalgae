function [st_CtrlSignals, state] = controller_simple_temp_mpc(Timeline, obs, refs, env, future, st_CtrlSignals, state)
% Minimal diagnostic MPC: 1-step horizon, Qhx+Tin_hx only, quadratic T tracking.
% No smoothness/consumption costs. IPOPT runs at print_level=5 (verbose).

    if exist('STOP.txt', 'file')
        error('STOP.txt detected. Terminating simulation.');
    end
    if isempty(state) || ~isfield(state, 'last_solve_time')
        state.next_run_time   = -1;
        state.last_solve_time = -1;
        state.last_qhx = 0;
        state.last_tin = 30;
    end

    time_step   = 5 * 60;
    currentTime = Timeline.time;
    T_setpoint  = 27.0;

    if (currentTime >= state.next_run_time) && (currentTime ~= state.last_solve_time)
        x_curr = [obs.pH, obs.DO, obs.Depth, obs.Xalg_gL, obs.T];

        % Use first step of forecast only (1-step MPC needs only d at step k=0)
        safe1 = @(v) double(v(1));
        d_curr = [safe1(future.RadGlobal); safe1(future.RadPAR); ...
                  safe1(future.Temp_ext);  safe1(future.RH); ...
                  safe1(future.Wind);      0; 0];

        try
            u_python = pyrun( ...
                "from mpc_handler import get_simple_temp_mpc_action; res = get_simple_temp_mpc_action(x, d, T_ref)", ...
                "res", x=x_curr, d=d_curr, T_ref=T_setpoint);

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
