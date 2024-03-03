%% 1-DOF Prewriting Example

%% Define Variables
Cd = 0.1; % [unitless]
M = 1;    % [kg]
A = 0.025; % [m^2]
motor_thrust = 100; % [N]
motor_burn_time = 1; % [s]
g = 9.81; % [m/s^2]
rho = 1.225; % [kg/m^3]


%% Calculate Parameters/ Variable Setup


%% Set Simulation Parameters
dT = 0.1; % [s]
z = 0;
z_dot = 0;
z_dot_dot = 0;


%% Run the simulation
cont_critereon = true;
iter = 0;

while cont_critereon
    %% Calculate Properties at Current SimTime
    t = iter * dT;


    %% Calculate forces and z_dot_dot
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
   
    %% Evaluate if sim continues
    iter = iter + 1;

    if iter > 1000
        cont_critereon = false;
    end
end

%% Save Data
