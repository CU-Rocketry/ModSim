%% 1-DOF Simulation Environment for 2024 SAC

clear

% TODO
% - load more parameters from the motor file

% Define Variables
M_dry = 21.975;          % [kg] Dry mass of rocket
Cd = 0.46;               % [unitless] Rocket total Cd
r_airfame = 0.0635;      % [m] Airfame Radius
h_fins = 0.1016;         % [m] Fin Height
t_fins = 0.0047625;      % [m] Fin Thickness
N_fins = 4;              % Number of Fins

motor_fname = 'thrust_curves/Cesaroni_9994M3400-P.rse';
motor_wet_mass = 8.108;   % [kg] Mass with no fuel
motor_prop_mass = 4.766;  % [kg] Mass of prop
motor_dry_mass = motor_wet_mass - motor_prop_mass;

g = 9.81;                % [m/s^2] Gravity

% Simulation Initial Conditions + Parameters
dT = 0.005; % [s]
z = 0;
z_dot = 0;
z_dot_dot = 0;

sim_end_time = 60;
t = 0;

% Calculate Parameters
A_fuselage = pi * r_airfame ^ 2;
A_fins = h_fins * t_fins * N_fins;

A = A_fuselage + A_fins; % [m^2] Rocket cross sectional area

motor = motor_generator(dT, motor_fname);
time_lookup = motor.time;
thrust_lookup = motor.thrust_lookup;
prop_mass_lookup = motor.prop_mass_lookup;

motor_burn_time = max(time_lookup); % [s] Motor burn time

% Recorder Setup
time = [];
r_z = [];
r_z_dot = [];
r_z_dot_dot = [];
r_T = [];
r_a = [];
r_P = [];
r_M = [];
r_motor_mass = [];
r_Th = [];
r_Cd = [];
r_Fd = [];
r_Mach = [];

% Run the simulation
iter = 0;
cont_bool = true;

%generate vector of motor masses
while cont_bool
    %% Calculate Properties at Current SimTime
    t = t + dT;
    iter = iter + 1;

    [T, a, P, rho] = atmosisa(z);


    %% Calculate forces and z_dot_dot
    % Atmospheric Drag
    q = 0.5 * rho * z_dot^2;
    Fd = q * A * Cd;

    if z_dot > 0
        % drag is opposite the velocity vector
        Fd = -1 * Fd;
    end

    % Motor Thrust and Mass
    if t >= 0 && t <= motor_burn_time
        Th = thrust_lookup(iter);
        motor_mass = prop_mass_lookup(iter) + motor_dry_mass;
    else
        Th = 0;
        motor_mass = motor_dry_mass; %[kg]
    end

    % Weight
    M = M_dry + motor_mass;
    W = M*g;

    % Solve governing eqn for z_dot_dot
    z_dot_dot = (Th + Fd - W)/M;

    % Calculate any other additional parameters
    mach = z_dot / a;


    %% Log Current Values to the Recorders
    r_z(iter) = z;
    r_z_dot(iter) = z_dot;
    r_z_dot_dot(iter) = z_dot_dot;
    time(iter) = t;
    r_T(iter) = T;
    r_a(iter) = a;
    r_P(iter) = P;
    r_M(iter) = M;
    r_motor_mass(iter) = motor_mass;
    r_Th(iter) = Th;
    r_Cd(iter) = Cd;
    r_Fd(iter) = Fd;
    r_Mach(iter) = mach;


    %% Calculate z and z_dot for the next timestep
    z_dot = z_dot + z_dot_dot * dT;
    z = z + z_dot * dT;

   
    %% Evaluate if sim continues
    if (t < sim_end_time && z > 0) || (iter < 5)
        % when sim is good to continue
        cont_bool = true;
    else
        % end sim
        cont_bool = false;
    end
end

%% Save Data
% not implemented yet


%% Load Reference Data and Plot to Compare
or_data = readtable(fullfile('or_sim_data', 'all_data_3.csv'));

or_time = table2array(or_data(:,"x_Time_s_"))';
or_z = table2array(or_data(:, "Altitude_m_"))';
or_z_dot = table2array(or_data(:, "TotalVelocity_m_s_"))';
or_z_dot_dot = table2array(or_data(:, "TotalAcceleration_m_s__"))';
or_mass = table2array(or_data(:, "Mass_g_"))';
or_thrust = table2array(or_data(:, "Thrust_N_"))';
or_drag_coefficient = table2array(or_data(:, "DragCoefficient___"))';
or_drag_force = table2array(or_data(:, "DragForce_N_"))';
or_motor_mass = table2array(or_data(:, "MotorMass_g_"))';
or_speed_of_sound = table2array(or_data(:,"SpeedOfSound_m_s_"))';
or_mach = table2array(or_data(:,"MachNumber___"))';
or_pressure = table2array(or_data(:,"AirPressure_mbar_"))';


%% Plot Our values and OR Values over each other
if true
    % position
    figure(1)
    plot(time, r_z, or_time, or_z)
    title('Position (m)')
    legend("1 DoF", "OpenRocket")

    % velocity
    figure(2)
    plot(time, r_z_dot, or_time, or_z_dot)
    title('Velocity (m/s)')
    legend("1 DoF", "OpenRocket")

    % acceleration
    figure(3)
    plot(time, abs(r_z_dot_dot), or_time, or_z_dot_dot)
    title('Acceleration (m/s^2)')
    legend("1 DoF", "OpenRocket")
end


%% Mass
if false
    figure(4)
    plot(time, r_M, or_time, or_mass/1000)
    title('Mass (kg)')
    legend("1 DoF", "OpenRocket")

    figure(5)
    plot(time, r_motor_mass, or_time, or_motor_mass/1000)
    title('Mass (kg)')
    legend("1 DoF", "OpenRocket")
end


%% Thrust
if true
    figure(6)
    plot(time, r_Th, or_time, or_thrust)
    title('Thrust (N)')
    legend("1 DoF", "OpenRocket")
end


%% Drag
if true
    figure(7)
    plot(time, r_Cd, or_time, or_drag_coefficient)
    title('Drag Coefficient')
    legend("1 DoF", "OpenRocket")

    figure(8)
    plot(time, abs(r_Fd), or_time, or_drag_force)
    title('Drag Force (N)')
    legend("1 DoF", "OpenRocket")
end


%% Air Properties
if false
    figure(9)
    plot(time, r_a, or_time, or_speed_of_sound)
    title('Speed of Sound (m/s)')
    legend("1 DoF", "OpenRocket")

    figure(10)
    plot(time, r_P, or_time, or_pressure * 100)
    title('Pressure (Pa)')
    legend("1 DoF", "OpenRocket")

    figure(11)
    plot(time, r_Mach, or_time, or_mach)
    title('Mach Number')
    legend("1 DoF", "OpenRocket")
end


%% Flight Analysis
or_apogee = max(or_z);
sim_apogee = max(r_z);
pct_diff_apogee = abs(or_apogee - sim_apogee) / or_apogee * 100;

disp(['OpenRocket Apogee: ' num2str(or_apogee)])
disp(['1 DOF Apogee: ' num2str(sim_apogee)])
disp(['Pct. Difference: ' num2str(pct_diff_apogee) ' %'])

