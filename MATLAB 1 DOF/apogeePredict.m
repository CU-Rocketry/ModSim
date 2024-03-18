function [z] = apogeePredict(z, z_dot, k_projectile, M)
% Predicts the maximum vertical height achieved by an object in projectile
% motion with aerodynamic drag.
%
% Inputs:
% z_0          - ASL altitude [m]
% z_dot_0      - vertical velocity [m/s]
% k_projectile - effective aerodynamic cross-sectional area (Cd * A) [m^2]
% M            - projectile mass [kg] (REMEMBER: Vehicle dry mass + burnt motor mass)
% 
% Output:
% z - ASL altitude at apogee [m]

% NOTE: I think the atmosisa function is pretty slow...

% Setup Variables
g = 9.81; % [m/s^2] Gravity
dT = 0.1; % [s]
t = 0;    % [s]

max_iter = inf;
iter = 0;

% Weight
W = M * g;

while (z_dot > 0) && (iter < max_iter)
    %% Calculate Properties at Current SimTime
    %t = t + dT;
    iter = iter + 1;

    [~, ~, ~, rho] = atmosisa(z);


    %% Calculate forces and z_dot_dot
    % Atmospheric Drag
    q = 0.5 * rho * z_dot^2;
    Fd = q * k_projectile;

    if z_dot > 0
        % drag is opposite the velocity vector
        Fd = -1 * Fd;
    end

    % Solve governing eqn for z_dot_dot
    z_dot_dot = (Fd - W) / M;


    %% Calculate z and z_dot for the next timestep
    z_dot = z_dot + z_dot_dot * dT;
    z = z + z_dot * dT;
end

%disp(iter)

end