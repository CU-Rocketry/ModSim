%% 1-DOF Skeleton

clc
clear

% Define Variables
Cd = 0.1; % [unitless]
M = 1;    % [kg]
A = 0.025; % [m^2]
motor_thrust = 100; % [N]
motor_burn_time = 1; % [s]
g = 9.81; % [m/s^2]
rho = 1.225; % [kg/m^3]

steps = 145; %number of steps

% Calculate Parameters/ Variable Setup
time = zeros(1,steps);
zMat = zeros(1, steps);

% Set Simulation Parameters
dT = 0.1; % [s]
z = 0;
z_dot = 0;
z_dot_dot = 0;

t = 0; %[s]


% Run the simulation
iter = 0;

for i = 1:steps
    % Calculate Properties at Current SimTime
    t = t + dT;


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


    %% Calculate z and z_dot for the next timestep
    z_dot = z_dot + z_dot_dot * dT;
    z = z + z_dot * dT;
    zMat(i) = z; [matrix of altitude over time]
    time(i) = t; [matrix of time]
   
    %% Evaluate if sim continues
end

plot(time, zMat)
