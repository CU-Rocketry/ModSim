%% 1-DOF Simulation Environment for 2024 SAC
% Simulates the control system for the airbrakes to evaluate their
% effectiveness.
% Specifically modified to model the pressure coupling effect on the
% airbrakes when the airbrakes are deployed
clear

% TODO (Control Systems Sim)
% - get actual k values from CFD
% - implement long. accel lockout for airbrakes activation


%% Define Variables
m_to_f = 3.281;

% Vehicle
M_dry = 22.861;          % [kg] Dry mass of rocket
Cd = 0.458;              % [unitless] Rocket total Cd
r_airfame = 0.0635;      % [m] Airfame Radius
h_fins = 0.1016;         % [m] Fin Height
t_fins = 0.0047625;      % [m] Fin Thickness
N_fins = 4;              % Number of Fins

% Env.
pad_altitude = 1400;     % [m] Spaceport America Pad Altitude MSL
g = 9.81;                % [m/s^2] Gravity
R = 287;                 % [J/kg*K] Gas Constant for Air

% Motor
motor_fname = 'thrust_curves/AeroTech_M2500T.rse';
motor_wet_mass = 8.108;  % [kg] Mass with no fuel
motor_prop_mass = 4.766; % [kg] Mass of prop
motor_dry_mass = motor_wet_mass - motor_prop_mass;

% Control System
target_alt_agl = 10000; % [ft]
refresh_time = 0.25; % [s] Refresh time of the airbrakes control system

time_lockout = 4.5; % [s] DTEG 7.4.1.1 (Boost phase ended, refine this detection in actual flight code, HAS ENOUGH CONTROL AUTHORITY)
Q_lockout = false;  %     DTEG 7.4.1.2 (Max. Q lockout Unimplemented)
alt_lockout = 2000; % [m] DTEG 7.4.1.3.2 (Altitude lockout for 10k flights passed, NOT ENOUGH CONTROL AUTHORITY)

accel_lockout = 0.0; % [m/s^2] When accel switches to negative then boost is over (NOT IMPLEMENTED)

target_alt_agl = target_alt_agl / m_to_f; % [m]
target_alt = target_alt_agl + pad_altitude; % [m]
apogee_prediction = -1; % initial value

c = -0.1; % Pressure coupling coefficient


%% Simulation Initial Conditions + Parameters
dT = 0.005;       % [s]
z = pad_altitude; % [m]
z_dot = 0;        % [m/s]
z_dot_dot = 0;    % [m/s^2]

sim_end_time = 60;
t = 0;


%% Calculate Motor Sim Parameters
motor = motor_generator(dT, motor_fname);
time_lookup = motor.time;
thrust_lookup = motor.thrust_lookup;
prop_mass_lookup = motor.prop_mass_lookup;

motor_burn_time = max(time_lookup); % [s] Motor burn time


%% Aerodynamic Parameters
% Vehicle
A_fuselage = pi * r_airfame ^ 2;
A_fins = h_fins * t_fins * N_fins;
A = A_fuselage + A_fins; % [m^2] Rocket cross sectional area

k_vehicle = A * Cd;

% Airbrakes
A_airbrakes = 0.00614;                    % [m^2]
Cd_airbrakes = 1.28;                      % Cd of a flat plate
k_airbrakes = Cd_airbrakes * A_airbrakes; % [m^2]
%k_airbrakes = 0.0;

% Control System
k_projectile = k_vehicle + k_airbrakes / 2;


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
r_airbrakes_state = [];
r_apogee_prediction = [];


%% Run the simulation
iter = 0;
cont_bool = true;
apogee_reached = false;

% Airbrakes Inital Conditions
AB_unlocked = false;
AB_deployed = false;
update_needed = false;
t_prev_update = -refresh_time;

while cont_bool
    %% Calculate Properties at Current SimTime
    t = t + dT;
    iter = iter + 1;

    %[T, a, P, rho] = atmosisa(z); % SLOW FUNCTION
    [T, a, P, rho] = stdAtm(z);

    % Airbrakes control logic here
    if (t - t_prev_update) >= refresh_time
        % Check and see if the airbrakes control system needs to be updated
        update_needed = true;
    else
        update_needed = false;
    end

    if AB_unlocked && update_needed
        % Lockout is passed, start active control to try to hit the desired apogee
        if AB_deployed == true
            % Add pressure coupling noise
            dh = c * (-R * T * z_dot^2)/(P * g);
            apogee_prediction = apogeePredict(z + dh, z_dot, k_projectile, M);
        else
            % Run a 'clean' apogee prediction
            apogee_prediction = apogeePredict(z, z_dot, k_projectile, M);
        end

        if apogee_prediction > target_alt % make sure AGL or ASL is consistent!!!
            AB_deployed = true;
        else
            AB_deployed = false;
        end

        t_prev_update = t; % record the update timestamp
    end


    %% Calculate forces and z_dot_dot
    % Atmospheric Drag
    q = 0.5 * rho * z_dot^2;

    if AB_deployed == true
        % Airbrakes are active
        Fd = q * (k_airbrakes + k_vehicle);
    else
        % Airbrakes are inactive
        Fd = q * k_vehicle;
    end

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
    r_airbrakes_state(iter) = AB_deployed;
    r_apogee_prediction(iter) = apogee_prediction;


    %% Calculate z and z_dot for the next timestep
    z_dot = z_dot + z_dot_dot * dT;
    z = z + z_dot * dT;


    %% Check for Sim Events
    if z_dot < -0.5
        apogee_reached = true;
    end

    %if t > time_lockout
    if t > time_lockout
        AB_unlocked = true;
    end
   
    %% Evaluate if sim continues
    if (t < sim_end_time && z_dot > 0) || (iter < 5)
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
    % save('simulated_data/retracted_flight.mat', 'r_z_agl_retracted', 'r_time_retracted')
    load('simulated_data/retracted_flight.mat')
    %save('simulated_data/deployed_flight.mat', 'r_z_agl_deployed', 'r_time_deployed')
    load('simulated_data/deployed_flight.mat')

    % position
    figure(1)
    plot(time, r_z_agl, r_time_retracted, r_z_agl_retracted, r_time_deployed, r_z_agl_deployed, time, r_apogee_prediction - pad_altitude)
    yline(target_alt_agl, 'r')
    title(['Position AGL (m) (c=' num2str(c) ')'])
    legend("Current Flight", "Retracted Flight", "Deployed Flight", "Apogee Prediction", "Target Alt.")
    xlabel("Time (s)")
    ylabel("Position (m)")
    ylim([0, inf])

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
    legend("1 DoF")

    figure(8)
    plot(time, abs(r_Fd))
    title('Drag Force (N)')
    legend("1 DoF")
end


%% Airbrakes
if true
    figure(9)
    plot(time, r_airbrakes_state);
    title('Airbrakes State (Bool)');
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

disp(['Apogee: ' num2str(sim_apogee) ' m (' num2str(m_to_f * sim_apogee) ' ft)'])
