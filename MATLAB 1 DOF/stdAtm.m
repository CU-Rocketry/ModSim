function [alt, T, P, rho] = stdAtm(alt)
% This function takes in the altitude of interest and returns the altitude,
% temperature, pressure, and density in SI units. Uses the Anderson model
% from AE212 in F21.
%
% NOTE: Only complete for the first gradient region of the atmosphere (only
% altitudes less than 11km)
%
% Ex.
% [alt, T, P, rho] = stdAtm(alt);
%
% Inputs:
% - altitude (m)
%
% Outputs:
% - altitude (m)
% - temperture (deg K)
% - pressure (Pa)
% - density (kg/m^3)

%% Define the Standard Sea-Level Conditions
t_0 = 288.16; % [K]
p_0 = 101325; % [Pa]
rho_0 = 1.2250; % [kg/m^3]

g_0 = 9.8; % [m/s^2]
R = 287; % [J/kg*K]

altitudes = [11, 25, 47, 53, 79, 90, 105] * 1e3; % [m]
temperatures = [288.16, 216.66, 282.66, 165.66, 225.66]; % [K]
pressures = [101325, 2.265016850997111e+04, 2.493582460578723e+03, 1.208796826557350e+02, 58.555450739915024, 1.015732573402018, 0.105215647482691, 0.007518917991855]; % [Pa]
densities = [1.2250, 0.364204971879773, 0.040095733925122, 0.045517672797983, 0.022049262446433, 6.526086350539201e-04, 6.760110080945176e-05, 3.546433798215636e-06]; % [kg/m^3]


%% Check the Altitude to Ensure Valid
if alt < 0
    % If altitude is less than 0
    disp('Altitude less than 0km, returning Sea-Level conditions.')

    T = t_0;
    P = p_0;
    rho = rho_0;
elseif alt > altitudes(7)
    % If altitude is above the top of the standard atm model
    disp('Altitude greater than 105km, returning conditions for 105km.')

    T = t_0;
    P = p_0;
    rho = rho_0;
end


%% Perform Calculations
if alt < altitudes(1)
    % First gradient region
    t_0 = temperatures(1); % [K]
    h_0 = 0; % [km]
    p_0 = pressures(1); % [Pa]
    rho_0 = densities(1); % [kg/m^3]

    a = -6.5e-3; % [K/m]

    T = t_0 + a * (alt - h_0); % [K]
    P = p_0 * (T / t_0) ^ (-g_0/(a*R)); % [Pa]
    rho = rho_0 * (T / t_0) ^ -((g_0/(a*R)) + 1); % [kg/m^3]

elseif alt < altitudes(2)
    % First isothermal region
    t_0 = temperatures(2); % [K]
    p_0 = pressures(2); % [Pa]
    rho_0 = densities(2); % [kg/m^3]
    h_0 = altitudes(1); % [m]

    T = t_0;
    P = p_0 * exp((-g_0/(R*T)*(alt-h_0))); % [Pa]
    rho = rho_0 * exp((-g_0/(R*T)*(alt-h_0))); % [kg/m^3]

elseif alt < altitudes(3)
    % Second gradient region
    t_0 = temperatures(2); %[K]
    h_0 = altitudes(2); %[km]
    p_0 = pressures(3); % [Pa]
    rho_0 = densities(3); % [kg/m^3]

    a = 3e-3; %[K/m]

    T = t_0 + a * (alt - h_0); % [K]
    P = p_0 * (T / t_0) ^ (-g_0/(a*R)); % [Pa]
    rho = rho_0 * (T / t_0) ^ -((g_0/(a*R)) + 1); % [kg/m^3]

elseif alt < altitudes(4)
    % Second isothermal region
    t_0 = temperatures(3); %[K]
    p_0 = pressures(4); % [Pa]
    rho_0 = densities(4); % [kg/m^3]
    h_0 = altitudes(3); % [m]
    
    T = t_0;
    P = p_0 * exp((-g_0/(R*T)*(alt-h_0))); % [Pa]
    rho = rho_0 * exp((-g_0/(R*T)*(alt-h_0))); % [kg/m^3]

elseif alt < altitudes(5)
    % Third gradient region
    t_0 = temperatures(3); %[K]
    h_0 = altitudes(4); %[km]
    p_0 = pressures(5); % [Pa]
    rho_0 = densities(5); % [kg/m^3]

    a = -4.5e-3; %[K/m]

    T = t_0 + a * (alt - h_0); % [K]
    P = p_0 * (T / t_0) ^ (-g_0/(a*R)); % [Pa]
    rho = rho_0 * (T / t_0) ^ -((g_0/(a*R)) + 1); % [kg/m^3]

elseif alt < altitudes(6)
    % Third isothermal region
    t_0 = temperatures(4); %[K]
    p_0 = pressures(6); % [Pa]
    rho_0 = densities(6); % [kg/m^3]
    h_0 = altitudes(5); % [m]

    T = t_0;
    P = p_0 * exp((-g_0/(R*T)*(alt-h_0))); % [Pa]
    rho = rho_0 * exp((-g_0/(R*T)*(alt-h_0))); % [kg/m^3]

elseif alt < altitudes(7)
    % Fourth gradient region
    t_0 = temperatures(4); %[K]
    h_0 = altitudes(6); %[km]
    p_0 = pressures(7); % [Pa]
    rho_0 = densities(7); % [kg/m^3]

    a = 4e-3; %[K/m]

    T = t_0 + a * (alt - h_0); % [K]
    P = p_0 * (T / t_0) ^ (-g_0/(a*R)); % [Pa]
    rho = rho_0 * (T / t_0) ^ -((g_0/(a*R)) + 1); % [kg/m^3]

elseif alt == altitudes(7)
    % End of the Standard Atmosphere Model
    T = temperatures(5); %[K]
    P = pressures(8); % [Pa]
    rho = densities(8); % [kg/m^3]
end

end