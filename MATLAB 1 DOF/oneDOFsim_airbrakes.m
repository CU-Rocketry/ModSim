%% 1-DOF Simulation Environment for 2024 SAC

clear

% TODO
% - load more parameters from the motor file
% - save recorder data to spreadsheet or CSV if needed
% - add drouge and main parachute deployment events


%% Define Variables
M_dry = 22.861;          % [kg] Dry mass of rocket
Cd = 0.458;              % [unitless] Rocket total Cd
r_airfame = 0.0635;      % [m] Airfame Radius
h_fins = 0.1016;         % [m] Fin Height
t_fins = 0.0047625;      % [m] Fin Thickness
N_fins = 4;              % Number of Fins

pad_altitude = 1400;     % [m] Spaceport America Pad Altitude MSL

motor_fname = 'thrust_curves/AeroTech_M2500T.rse';
motor_wet_mass = 8.108;  % [kg] Mass with no fuel
motor_prop_mass = 4.766; % [kg] Mass of prop
motor_dry_mass = motor_wet_mass - motor_prop_mass;

g = 9.81;                % [m/s^2] Gravity


%% Simulation Initial Conditions + Parameters
dT = 0.005;       % [s]
z = pad_altitude; % [m]
z_dot = 0;        % [m/s]
z_dot_dot = 0;    % [m/s^2]

sim_end_time = 60;
t = 0;


%% Calculate Sim Parameters
A_fuselage = pi * r_airfame ^ 2;
A_fins = h_fins * t_fins * N_fins;

A = A_fuselage + A_fins; % [m^2] Rocket cross sectional area

motor = motor_generator(dT, motor_fname);
time_lookup = motor.time;
thrust_lookup = motor.thrust_lookup;
prop_mass_lookup = motor.prop_mass_lookup;

motor_burn_time = max(time_lookup); % [s] Motor burn time


%% Airbrakes Aerodynamic Parameters
A_airbrakes = 0.00614;                    % [m^2]
Cd_airbrakes = 1.28;                      % Cd of a flat plate
k_airbrakes = Cd_airbrakes * A_airbrakes; % [m^2]
%k_airbrakes = 0.0; 

deploy_timestamp = motor_burn_time; % [s] The timestamp where the airbrakes deploy
retract_timestamp = inf;            % [s] The timestamp where the airbrakes retract


%% Recorder Setup
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
r_airbrakes_drag = [];


%% Run the simulation
iter = 0;
cont_bool = true;
apogee_reached = false;


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

    % Airbrakes Drag Force
    if (t > deploy_timestamp) && (t < retract_timestamp) && ~apogee_reached
        % Airbrakes are deployed
        Fd_airbrakes = q * k_airbrakes;
    else
        % Airbrakes are retracted
        Fd_airbrakes = 0;
    end

    if z_dot > 0
        % burn direction is opposite the velocity vector
        Fd_airbrakes = -1 * Fd_airbrakes;
    end

    % Solve governing eqn for z_dot_dot
    z_dot_dot = (Th + Fd + Fd_airbrakes - W)/M;

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
    r_airbrakes_drag(iter) = Fd_airbrakes;


    %% Calculate z and z_dot for the next timestep
    z_dot = z_dot + z_dot_dot * dT;
    z = z + z_dot * dT;


    %% Check for Sim Events
    if z_dot < -0.5
        apogee_reached = true;
    end

   
    %% Evaluate if sim continues
    if (t < sim_end_time && z > 0) || (iter < 5)
        % when sim is good to continue
        cont_bool = true;
    else
        % end sim
        cont_bool = false;
    end
end

% Create recorder for AGL altitude
r_z_agl = r_z - pad_altitude;


%% Save Data
% not implemented yet


%% Plot Our values
if true
    % position
    figure(1)
    plot(time, r_z_agl)
    title('Position AGL (m)')
    legend("1 DoF")

    % velocity
    figure(2)
    plot(time, r_z_dot)
    title('Velocity (m/s)')
    legend("1 DoF")

    % acceleration
    figure(3)
    plot(time, r_z_dot_dot)
    title('Acceleration (m/s^2)')
    legend("1 DoF")
end


%% Mass
if false
    figure(4)
    plot(time, r_M)
    title('Mass (kg)')
    legend("1 DoF")

    figure(5)
    plot(time, r_motor_mass)
    title('Mass (kg)')
    legend("1 DoF")
end


%% Thrust
if false
    figure(6)
    plot(time, r_Th)
    title('Thrust (N)')
    legend("1 DoF")
end


%% Drag
if true
    figure(7)
    plot(time, r_Cd)
    title('Drag Coefficient')
    legend("1 DoF", "OpenRocket")

    figure(8)
    plot(time, abs(r_Fd))
    title('Drag Force (N)')
    legend("1 DoF")
end


%% Airbrakes
if true
    figure(9)
    plot(time, r_airbrakes_drag);
    title('Airbrakes Drag (N)');
    legend("1 DOF");
end 


%% Air Properties
if false
    figure(10)
    plot(time, r_a)
    title('Speed of Sound (m/s)')
    legend("1 DoF")

    figure(11)
    plot(time, r_P)
    title('Pressure (Pa)')
    legend("1 DoF")

    figure(12)
    plot(time, r_Mach)
    title('Mach Number')
    legend("1 DoF")
end


%% Flight Analysis
sim_apogee = max(r_z_agl);

disp(['Apogee: ' num2str(sim_apogee) ' m (' num2str(3.281 * sim_apogee) ' ft)'])
