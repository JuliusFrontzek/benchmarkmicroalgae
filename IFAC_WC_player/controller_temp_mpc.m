function [st_CtrlSignals, state] = controller_temp_mpc(Timeline, obs, refs, env, future, st_CtrlSignals, state)
% Temperature-only MPC controller (diagnostic).
% Optimises Qhx and Tin_hx to track a 27°C setpoint.
% CO2 and air are held at zero so this controller does not interact with pH/DO.

    if exist('STOP.txt', 'file')
        error('STOP.txt detected. Terminating simulation.');
    end

    if isempty(state) || ~isfield(state, 'last_solve_time')
        state.next_run_time  = -1;
        state.last_solve_time = -1;
        state.last_qhx = 0;
        state.last_tin = 30;  % neutral start
    end

    time_step   = 5 * 60;  % 5-minute MPC step [s]
    currentTime = Timeline.time;
    N           = 12;
    T_setpoint  = 27.0;  % [°C] diagnostic setpoint

    if (currentTime >= state.next_run_time) && (currentTime ~= state.last_solve_time)

        % State vector (same order as full MPC)
        x_curr = [obs.pH, obs.DO, obs.Depth, obs.Xalg_gL, obs.T];

        % Future disturbance matrix (7 rows × N cols)
        safe_pad = @(v, n) [v(1:min(end,n)); repmat(v(end), max(0, n-length(v)), 1)];

        Qd_future = zeros(1, N);
        Qh_future = zeros(1, N);

        op_duration      = 50 * 60;
        dilution_offset  = 20 * 60;

        for k = 1:N
            t_lookahead = currentTime + (k-1)*time_step;
            sec_day  = mod(t_lookahead, 86400);
            day_num  = floor(t_lookahead / 86400);

            if mod(day_num, 2) == 0
                h_start = 9  * 3600;
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

        d_future = [
            safe_pad(future.RadGlobal, N)'; ...
            safe_pad(future.RadPAR,    N)'; ...
            safe_pad(future.Temp_ext,  N)'; ...
            safe_pad(future.RH,        N)'; ...
            safe_pad(future.Wind,      N)'; ...
            Qd_future; ...
            Qh_future
            ];

        % CO2 and air from the currently applied signals (set to 0 for isolation)
        u_co2 = 0.0;
        u_air = 0.0;

        try
            u_python = pyrun( ...
                "from mpc_handler import get_temp_mpc_action; res = get_temp_mpc_action(x, d, T_ref, u_co2, u_air, u_prev_qhx, u_prev_tin)", ...
                "res", ...
                x=x_curr, d=d_future, T_ref=T_setpoint, ...
                u_co2=u_co2, u_air=u_air, ...
                u_prev_qhx=state.last_qhx, u_prev_tin=state.last_tin);

            result = double(u_python);
            state.last_qhx = result(1);
            state.last_tin = result(2);
            state.next_run_time  = currentTime + time_step;
            state.last_solve_time = currentTime;
        catch ME
            fprintf('[TempMPC] Python error: %s\n', ME.message);
        end
    end

    % Output with actuator saturation
    st_CtrlSignals.Qhx    = min(36/1000/60, max(0, state.last_qhx));
    st_CtrlSignals.Tin_hx = min(80,         max(0, state.last_tin));

end
