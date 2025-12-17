function [st_CtrlSignals, state] = controller_HD_fixed(Timeline, obs, refs, env, future, st_CtrlSignals, state)
% Daily fixed-time harvest/dilution sequence based on depth.
%
%   [st_CtrlSignals, state] = controller_HD_fixed(Timeline, obs, refs, env, future, st_CtrlSignals, state)
%
% Purpose:
%   Every simulation day, at a given clock time (default 09:00), start a harvest
%   phase that removes culture until the raceway depth has decreased by a fixed
%   dilution ratio (e.g. 20% of the depth at the beginning of harvest). Then, switch to
%   a dilution phase that adds fresh medium until the target depth is reached
%   again (e.g. Depth_target = 0.15 m).
%
%   Outputs Dil_cmd and Harv_cmd are boolean commands (0/1) that drive the
%   dilution and harvest actuators in the benchmark:
%       Qd = Qd_cmd  * Qd_r   (inflow / dilution)
%       Qh = Qh_cmd * Qh_r   (outflow / harvest)
%
% Inputs:
%   Timeline (struct) Timeline structure. Expected fields:
%                       - dt: Sample time [s]
%                       - index: Sample index [-]
%                       - time: Absolute time in seconds [s]
%                       - time_secday: Seconds of day [s]
%                       - hour: Hour of day [h]
%                       - min: Minutes of hour [min]
%
%   obs     (struct)   Measured/estimated variables at current step. Expected fields:
%                      - obs.pH: reactor pH [-]
%                      - obs.DO: dissolved oxygen [% sat]
%                      - obs.Depth: water depth [m]
%                      - obs.Xalg_gL: microalgae biomass concentration [g/L]
%                      - obs.T: reactor water temperature [°C]

%   refs    (struct)   Corresponding setpoints for controled variables. Expected fields:
%                      - refs.pH: pH reference [-]
%                      - refs.DO: Dissolved Oxygen reference [%]
%                      - refs.T: Temperature reference [°C]
%
%   env     (struct)   Environment/context for the controller. Expected fields:
%                      - env.RadGlobal: global radiation [W/m^2]
%                      - env.RadPAR: photosynthetically active radiation [µE/m^2/s]
%                      - env.Temp_ext: ambient temperature [°C]
%                      - env.RH: relative humidity [%]
%                      - env.Wind: wind speed [m/s]
%
%   future  (struct)   Optional look-ahead trajectories. Expected fields:
%                      - future.t_future: 
%                      - future.RadGlobal: global radiation [W/m^2]
%                      - future.RadPAR: photosynthetically active radiation [µE/m^2/s]
%                      - future.Temp_ext:ambient temperature [°C]
%                      - future.RH: relative humidity [%]
%                      - future.Wind: wind speed [m/s]
%
%   state   (struct)   Persistent controller state and tuning parameters.
%
% Outputs:
%   st_CtrlSignals  (struct) Structure for control signals. Expected fields:
%                      - Qco2  (double):  Commanded CO2 flow [m^3/s].
%                      - Qair  (double):  Commanded air flow [m^3/s].
%                      - Qd_bin  (double):  Commanded dilution signal [-].
%                      - Qh_bin  (double):  Commanded harvest signal [-].
%                      - Qhx  (double):  Commanded heat exchanger flow [m^3/s].
%                      - Tin_hx  (double):  Inlet temperature for heat exchanger [°C].
%
%   state (struct):  Updated state.

% ---------------- Initialization of state & parameters ----------------
if nargin < 6 || isempty(state)
    state = struct();
end

% Default schedule: start harvest each day at 09:00
if ~isfield(state,'harvest_hour') || isempty(state.harvest_hour)
    state.harvest_hour = 9;      % 9 AM
end
if ~isfield(state,'harvest_min') || isempty(state.harvest_min)
    state.harvest_min  = 0;      % 09:00
end

% Depth parameters
if ~isfield(state,'depth_target_m') || isempty(state.depth_target_m)
    state.depth_target_m = 0.15; % [m]
end
if ~isfield(state,'depth_drop_frac') || isempty(state.depth_drop_frac)
    state.depth_drop_frac = 0.20; % 20% drop during harvest
end
if ~isfield(state,'min_depth_m') || isempty(state.min_depth_m)
    state.min_depth_m = 0.05;    % [m] safety limit
end

% Mode / day bookkeeping
if ~isfield(state,'mode') || isempty(state.mode)
    state.mode = "idle";         % "idle", "harvest", "dilute"
end
if ~isfield(state,'current_day') || isempty(state.current_day)
    state.current_day = -1;      % force reset on first call
end
if ~isfield(state,'harvest_done_today') || isempty(state.harvest_done_today)
    state.harvest_done_today = false;
end
if ~isfield(state,'depth_start_harvest') || isempty(state.depth_start_harvest)
    state.depth_start_harvest = NaN;
end

% ---------------- Day reset logic ----------------
% Compute integer "day index" from absolute time
day_index = floor(Timeline.time / 86400);    % day 0,1,2,...
if day_index ~= state.current_day
    % New simulation day: reset daily flags and mode
    state.current_day        = day_index;
    state.harvest_done_today = false;
    state.mode               = "idle";
    state.depth_start_harvest = NaN;
end

% ---------------- Default commands ----------------
Dil_cmd  = 0;
Harv_cmd = 0;

% ---------------- Read current variables ----------------
Depth = obs.Depth;                         % [m]
Depth = max(Depth, 1e-6);                  % numerical safety
hour  = Timeline.hour;
minute= Timeline.min;

% ---------------- Mode transition logic ----------------
switch state.mode
    case "idle"
        % Check if it is time to start the daily harvest cycle.
        if ~state.harvest_done_today && ...
           hour == state.harvest_hour && minute >= state.harvest_min
            % Start harvest phase
            state.mode = "harvest";
            state.depth_start_harvest = Depth;  % reference depth for 20% drop
        end

    case "harvest"
        % Check if the required depth drop has been achieved or safety min reached
        target_depth_harvest = (1 - state.depth_drop_frac) * state.depth_start_harvest;
        target_depth_harvest = max(target_depth_harvest, state.min_depth_m);

        if Depth <= target_depth_harvest
            % Switch to dilution phase to refill to target depth
            state.mode = "dilute";
        end

    case "dilute"
        % Check if target depth has been recovered
        if Depth >= state.depth_target_m
            state.mode = "idle";
            state.harvest_done_today = true;  % one full cycle per day
        end
end

% ---------------- Command outputs based on mode ----------------
switch state.mode
    case "idle"
        Dil_cmd  = 0;
        Harv_cmd = 0;

    case "harvest"
        Dil_cmd  = 0;
        Harv_cmd = 1;   % only harvest: outflow ON

    case "dilute"
        Dil_cmd  = 1;   % only dilution: inflow ON
        Harv_cmd = 0;
end

%% Output
st_CtrlSignals.Qd_bin = Dil_cmd;
st_CtrlSignals.Qh_bin = Harv_cmd;
end
