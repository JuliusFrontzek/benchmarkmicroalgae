function [st_CtrlSignals, state] = controller_mpc_l4c(Timeline, obs, refs, env, future, st_CtrlSignals, state)
% Initialize state for ZOH
if isempty(state) || ~isfield(state, 'next_run_time')
    state.next_run_time = -1;
    state.last_u = [0, 0, 0, 0];
end

time_step = 30 * 60;

currentTime = Timeline.time;

if currentTime >= state.next_run_time
    % Prepare Vectors
    x_curr = [obs.pH, obs.DO, obs.Depth, obs.Xalg_gL, obs.T];

    % Keep d_curr as 7 elements as requested
    d_curr = [env.RadGlobal, env.RadPAR, env.Temp_ext, env.RH, env.Wind, st_CtrlSignals.Qd_bin, st_CtrlSignals.Qh_bin];

    r_curr = [refs.pH, refs.DO, refs.T];

    % Call Python
    try
        u_python = pyrun("from mpc_handler import get_mpc_action; res = get_mpc_action(x, d, r)", ...
            "res", x=x_curr, d=d_curr, r=r_curr);

        state.last_u = double(u_python);
        state.next_run_time = currentTime + time_step; % Schedule next run in 300s
    catch ME
        fprintf('Error in Python MPC at t=%.1f: %s\n', currentTime, ME.message);
    end
end

% Apply Zero Order Hold
u_opt = state.last_u;

% Map to Control Signals
st_CtrlSignals.Qco2   = max(0, u_opt(1));
st_CtrlSignals.Qair   = max(0, u_opt(2));
st_CtrlSignals.Qhx    = max(0, u_opt(3));
st_CtrlSignals.Tin_hx = max(0, u_opt(4));
end