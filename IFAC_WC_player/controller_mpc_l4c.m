function [st_CtrlSignals, state] = controller_mpc_l4c(Timeline, obs, refs, env, future, st_CtrlSignals, state)
% Initialize state if needed
if isempty(state), state = struct(); end

% 1. Prepare State Vector (nx=5)
% Order: [pH, DO, Depth, Xalg, T]
x_curr = [obs.pH, obs.DO, obs.Depth, obs.Xalg_gL, obs.T];

% 2. Prepare Disturbance Vector (nd=7)
% Map your env fields to the model's disturbance inputs
d_curr = [env.Temp_ext, env.RadGlobal, env.RH, env.Wind, env.RadPAR, 0, 0];

% 3. Prepare Reference Vector (3)
r_curr = [refs.pH, refs.DO, refs.T];

% 4. Call Python
% We use pyrun to call the helper function in our py script
try
    u_python = pyrun("from mpc_handler import get_mpc_action; res = get_mpc_action(x, d, r)", ...
        "res", x=x_curr, d=d_curr, r=r_curr);

    % Convert python list to matlab double
    u_opt = double(u_python);
catch ME
    fprintf('Error in Python MPC: %s\n', ME.message);
    u_opt = zeros(1, 4); % Fallback
end

% 5. Map Outputs to st_CtrlSignals
% Assumption: u_opt = [Qco2, Qair, Qd_bin, Qhx]
st_CtrlSignals.Qco2 = max(0, u_opt(1));
st_CtrlSignals.Qair = max(0, u_opt(2));
st_CtrlSignals.Qd_bin = (u_opt(3) > 0.5); % Binary conversion if needed
st_CtrlSignals.Qhx  = u_opt(4);

% Optional: set a default heating inlet temp if not part of MPC
st_CtrlSignals.Tin_hx = 40.0;
end