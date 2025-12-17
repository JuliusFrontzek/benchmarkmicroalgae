function [st_CtrlSignals, state] = controller_Temp_HX_no_control(Timeline, obs, refs, env, future, st_CtrlSignals, state)
% Temperature control - No control
%
%   [st_CtrlSignals, state] = controller_Temp_HX_no_control(Timeline, obs, refs, env, future, st_CtrlSignals, state)
%
% Purpose:
%   free evolution of the culture temperature without applying control in the heat exchanger.
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
%% Output
st_CtrlSignals.Qhx  = 0.0;
st_CtrlSignals.Tin_hx = obs.T;
end
