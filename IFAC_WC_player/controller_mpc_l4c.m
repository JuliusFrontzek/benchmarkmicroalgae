function [st_CtrlSignals, state] = controller_mpc_l4c(Timeline, obs, refs, env, future, st_CtrlSignals, state)
if isempty(state) || ~isfield(state, 'last_solve_time')
    state.next_run_time = -1;
    state.last_solve_time = -1;
    state.last_u = [0, 0, 0, 0];
end

time_step = 5 * 60;
currentTime = Timeline.time;
N = 12;

if (currentTime >= state.next_run_time) && (currentTime ~= state.last_solve_time)
    % 1. State Vector (Order: pH, DO, Depth, Xalg, T)
    x_curr = [obs.pH, obs.DO, obs.Depth, obs.Xalg_gL, obs.T];

    % 2. Future Disturbance Logic (7 rows)
    safe_pad = @(v, n) [v(1:min(end,n)); repmat(v(end), max(0, n-length(v)), 1)];

    Qd_future = zeros(1, N);
    Qh_future = zeros(1, N);

    op_duration = 50 * 60; % 3000s
    dilution_offset = 20 * 60; % Dilution starts 20m after Harvest (overlap)

    for k = 1:N
        t_lookahead = currentTime + (k-1)*time_step;
        sec_day = mod(t_lookahead, 86400);
        day_num = floor(t_lookahead / 86400);

        % 2-Day Cycle: Day A (0, 2, 4...) and Day B (1, 3, 5...)
        if mod(day_num, 2) == 0
            h_start = 9 * 3600;  % 9 AM
        else
            h_start = 12 * 3600; % 12 PM
        end

        d_start = h_start + dilution_offset;

        % Check if current step falls within the 50-min window
        if (sec_day >= h_start) && (sec_day < h_start + op_duration)
            Qh_future(k) = 1;
        end
        if (sec_day >= d_start) && (sec_day < d_start + op_duration)
            Qd_future(k) = 1;
        end
    end

    d_future = [
        safe_pad(future.RadGlobal, N)'; ...
        safe_pad(future.RadPAR, N)'; ...
        safe_pad(future.Temp_ext, N)'; ...
        safe_pad(future.RH, N)'; ...
        safe_pad(future.Wind, N)'; ...
        Qd_future; ... % Row 6: Scheduled Dilution
        Qh_future      % Row 7: Scheduled Harvest
        ];

    r_curr = [refs.pH, refs.DO, refs.T];

    try
        u_python = pyrun("from mpc_handler import get_mpc_action; res = get_mpc_action(x, d, r, u_prev)", ...
            "res", x=x_curr, d=d_future, r=r_curr, u_prev=state.last_u);

        state.last_u = double(u_python);
        state.next_run_time = currentTime + time_step;
        state.last_solve_time = currentTime;
    catch ME
        fprintf('Error in Python MPC: %s\n', ME.message);
    end
end

% Apply signals
u_opt = state.last_u;
st_CtrlSignals.Qco2   = max(0, u_opt(1));
st_CtrlSignals.Qair   = max(0, u_opt(2));
st_CtrlSignals.Qhx    = max(0, u_opt(3));
st_CtrlSignals.Tin_hx = max(0, u_opt(4));
end