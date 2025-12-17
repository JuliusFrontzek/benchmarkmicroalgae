function [st_CtrlSignals, state] = controller_pH_OnOff(Timeline, obs, refs, env, future, st_CtrlSignals, state)
% pH control - On/Off classical daytime control
%
%   [st_CtrlSignals, state] = controller_pH_OnOff(Timeline, obs, refs, env, future, st_CtrlSignals, state)
%
% Purpose:
%   pH control implementing an On/Off classical daytime controller in a valve.
%   When pH exceeds a given threshold (e.g. 8), the controller
%   turns the CO2 valve ON (as requested) and uses hysteresis plus minimum On/Off times to avoid chattering.
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

% ----------- Initialize state & parameters -----------
if nargin < 6 || isempty(state), state = struct(); end

% Setpoint from state or env
if ~isfield(state,'setpoint') || isempty(state.setpoint)
    if  isfield(refs,'pH') && ~isempty(refs.pH)
        state.setpoint = refs.pH;
    else
        state.setpoint = 8.0;
    end
end

% Hysteresis width
if ~isfield(state,'deadband') || isempty(state.deadband)
    state.deadband = 0.10;
end

% Dwell times to reduce chatter
if ~isfield(state,'min_on_s')  || isempty(state.min_on_s),  state.min_on_s  = 60; end
if ~isfield(state,'min_off_s') || isempty(state.min_off_s), state.min_off_s = 60; end

% Mode & last switch bookkeeping
if ~isfield(state,'mode') || isempty(state.mode), state.mode = "off"; end
if ~isfield(state,'last_switch') || isempty(state.last_switch), state.last_switch = -inf; end

% ----------- Hysteresis thresholds -----------
sp   = state.setpoint;
db   = max(state.deadband, 1e-6);
pHhi = sp + db/2;
pHlo = sp - db/2;

% ----------- Decision with dwell-time constraints -----------
pH  = obs.pH;
now = Timeline.time;

% Desired mode from hysteresis only
if     pH > pHhi &&  env.RadGlobal > 10
    desired = "on";
elseif pH < pHlo || env.RadGlobal < 10
    desired = "off";
else
    desired = state.mode; % stay as you are inside the band
end

% Enforce minimum dwell times
time_since = now - state.last_switch;
switch state.mode
    case "on"
        % Can turn off only if min_on_s elapsed
        if desired == "off" && time_since >= state.min_on_s
            state.mode = "off";
            state.last_switch = now;
        end
    case "off"
        % Can turn on only if min_off_s elapsed
        if desired == "on" && time_since >= state.min_off_s
            state.mode = "on";
            state.last_switch = now;
        end
end

% ----------- Command output (saturated) -----------
if state.mode == "on"
    qco2 = 20/1000/60;
else
    qco2 = 0;
end

%% Output
st_CtrlSignals.Qco2 = qco2;
end
