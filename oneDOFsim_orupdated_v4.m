%% 1-DOF Prewriting Example

clc
clear


% Define Variables
Cd = 0.46;              % [unitless] Rocket total Cd
M_dry = 21.975;         % [kg] Dry mass of rocket
A = 0.01266;            % [m^2] Rocket cross sectional area
motor_thrust = 3415;    % [N] Motor avg thrust
motor_mass = 8.108;     % [kg] Mass with full fuel
motor_dry_mass = 3.442; % [kg] Mass with no fuel
motor_burn_time = 2.92; % [s] Motor burn time
g = 9.81;               % [m/s^2] Gravity

sim_end_time = 60;

% Calculate Parameters/ Variable Setup/ Recorders
time = []; %zeros(1, steps);
r_z = []; %zeros(1, steps);
r_z_dot = [];
r_z_dot_dot = [];
r_T = [];
r_a = [];
r_P = [];


% Initial Conditions
dT = 0.01; % [s]
z = 0;
z_dot = 0;
z_dot_dot = 0;

t = 0;

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
         Th = motor_thrust;
         motor_mass = motor_dry_mass;
         % = (motor_mass - M_dry)/(motor_burn_time);
    else
        Th = 0;
        motor_mass = motor_dry_mass; %[kg]
    end

    % Weight
    M = M_dry + motor_mass;
    W = M*g;

    % Solve governing eqn for z_dot_dot
    z_dot_dot = (Th + Fd - W)/M;


    %% Log Current Values to the Recorders
    r_z(iter) = z;
    r_z_dot(iter) = z_dot;
    r_z_dot_dot(iter) = z_dot_dot;
    time(iter) = t;
    r_T(iter) = T;
    r_a(iter) = a;
    r_P(iter) = P;


    %% Calculate z and z_dot for the next timestep
    z_dot = z_dot + z_dot_dot * dT;
    z = z + z_dot * dT;

   
    %% Evaluate if sim continues
    if t < sim_end_time && z > 0
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
or_data = readtable("all_data_2.csv");

or_time = table2array(or_data(:,"x_Time_s_"));
or_z = table2array(or_data(:, "Altitude_m_"));
or_z_dot = table2array(or_data(:, "TotalVelocity_m_s_"));
or_z_dot_dot = table2array(or_data(:, "TotalAcceleration_m_s__"));
or_mass = table2array(or_data(:, "Mass_g_"));
or_thrust = table2array(or_data(:, "Thrust_N_"));
or_drag_coefficient = table2array(or_data(:, "DragCoefficient___"));
or_motor_mass = table2array(or_data(:, "MotorMass_g_"));


%% Plot Our values and OR Values over eachother
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
plot(time, r_z_dot_dot, or_time, or_z_dot_dot)
title('Acceleration (m/s^2)')
legend("1 DoF", "OpenRocket")

%% mass
figure(4)
plot(time, M, or_time, or_mass) %need recorder
title('Mass (kg)')
legend("1 DoF", "OpenRocket")

%% thrust
figure(5)
plot(time, Th, or_time, or_thrust)%need recorder
title('Thrust (N)')
legend("1 DoF", "OpenRocket")

%% drag
figure(6)
plot(time, Cd, or_time, or_drag_coefficient)%need recorder   
title('Drag(N)')
legend("1 DoF", "OpenRocket")



