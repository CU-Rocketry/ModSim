%% 1-DOF Prewriting Example

clc
clear


% Define Variables
Cd = 0.46;  % [unitless]
M_dry = 3.342; % [kg]
A = 0.01266;   % [m^2]
motor_thrust = 3415; % [N]
motor_mass = 8.108; %
% [kg]
motor_burn_time = 2.92; % [s]
g = 9.81;    % [m/s^2]
%rho = 1.225; % [kg/m^3]

sim_end_time = 60;

% Calculate Parameters/ Variable Setup
time = []; %zeros(1, steps);
r_z = []; %zeros(1, steps);
r_z_dot = [];
r_z_dot_dot = [];

% Set Simulation Parameters
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
    % Calculate Properties at Current SimTime
    t = t + dT;
    iter = iter + 1;

    [T, a, P, rho] = atmosisa(z);

    % Calculate forces and z_dot_dot
    % Atmospheric Drag
    q = 0.5 * rho * z_dot^2;
    Fd = q * A * Cd;

    % Motor Thrust and Mass
    if t >= 0 && t <= motor_burn_time 
         Th = motor_thrust;
         M = (motor_mass - M_dry)/(motor_burn_time);
    else
        Th = 0;
        motor_mass = 3.442; %[kg]
    end
%M= M_dry + motor_mass;
    % Weight
    W = M*g;

    % Solve governing eqn for z_dot_dot
    z_dot_dot = (Th - Fd - W)/M;


    %% Log Current Values to the Recorders
    r_z(iter) = z;
    r_z_dot(iter) = z_dot;
    r_z_dot_dot(iter) = z_dot_dot;
    time(iter) = t;


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


%% Load Reference Data and Plot to Compare
or_data_table = readtable("rocketv5.csv");
or_data = readmatrix("rocketv5.csv");

%% Plot Our values and OR Values over eachother
plot(time, r_z)
%%
plot(or_data(:,1), or_data(:,2))
%%
plot(time, r_z, or_data(:,1).', or_data(:,2).')
