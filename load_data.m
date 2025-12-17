%% ---------------- Data load ----------------
% Expect: Data is an array of structs with fields:
%   Date (datetime) Date of the measured data.
%
%   t     (datetime)   Minute time array for every day.
%
%   u     (double)   Measured input variables. Expected fields:
%                      - Data.u(1): global solar radiation [W/m^2]
%                      - Data.u(2): photosynthetically active solar radiation [µE/m^2/s]
%                      - Data.u(3): ambient temperature [°C]
%                      - Data.u(4): relative humidity [%]
%                      - Data.u(5): wind speed [m/s]
try
    S = load('Data_Benchmark','Data');
    Data = S.Data;
catch
    error('Data_Benchmark_ext.mat not found or missing variable "Data".');
end
