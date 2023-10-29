%% 1-DOF Prewriting Example

clc
clear

% TODO
% - atm model
% - rocket mass change during burn



% Define Variables
Cd = 0.46;  % [unitless]
M = 21.805; % [kg]
A = 0.01266;   % [m^2]
motor_thrust = 3415; % [N]
motor_burn_time = 3; % [s]
g = 9.81;    % [m/s^2]
rho = 1.225; % [kg/m^3]

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

while cont_bool
    % Calculate Properties at Current SimTime
    t = t + dT;
    iter = iter + 1;

    % Calculate forces and z_dot_dot
    % Atmospheric Drag
    q = 0.5 * rho * z_dot^2;
    Fd = q * A * Cd;

    % Thrust
    if t > motor_burn_time
        T = 0;
    else
        T = motor_thrust;
    end

    % Weight
    W = M*g;

    % Solve governing eqn for z_dot_dot
    z_dot_dot = (T - Fd - W)/M;


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
or_data_table = readtable("ORtimeandalt.csv");
or_data = readmatrix("ORtimeandalt.csv");

%% Plot Our values and OR Values over eachother
plot(time, r_z)
%%
plot(or_data(:,1), or_data(:,2))
%%
plot(time, r_z, or_data(:,1).', or_data(:,2).')
