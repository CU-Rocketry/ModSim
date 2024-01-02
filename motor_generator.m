function motor = motor_generator(dT, fname)
% Thrust Curve Analysis Function for generating evenly spaced datapoints
% for lookup tables.
% More info: http://wiki.openrocket.info/RSE_File
%
% Inputs:
% dT - timestep (s)
% fname - filepath to a .rse file
%
% Outputs:
% motor - struct containing motor.thrust_lookup, motor.prop_mass_lookup
%
% fname = 'thrust_curves/Cesaroni_9994M3400-P.rse';
% dT = 0.01;
% motor = motor_generator(dT, fname)
% thrust_lookup = motor.thrust_lookup       % [N]
% prop_mass_lookup = motor.prop_mass_lookup % [kg]


%% Load + Parse The Inputted File
motor_string = readlines(fname);

data_text = [];
name_text = [];
specs_text = [];

for i = 1:length(motor_string)
    line = motor_string(i);

    if contains(line, "<eng-data ")
        % if the line contains thrust data
        data_text = [data_text; line];
    elseif contains(line, "<comments>")
        % if the line contains the motor name
        name_text = [name_text; line];
    elseif contains(line, "<engine ")
        % if the line is the first line in the block of specs
        ii = i + 1;

        while ~contains(motor_string(ii), "<")
            % assemble the whole string of data in this section
            line = strcat(line, " ", motor_string(ii));
            ii = ii + 1;
        end

        specs_text = [specs_text; line];
    end
end


%% Process Motor Name Data
name_text = erase(name_text, "<comments>");
name_text = erase(name_text, "</comments>");

motor.name = strtrim(name_text);


%% Process Motor Time Data
data_text = erase(data_text, "<eng-data ");
data_text = erase(data_text, "/>");
data_text = strtrim(data_text);

t_raw = [];
f_raw = [];
m_raw = [];
cg_raw = [];

for i = 1:length(data_text)
    line = data_text(i);
    line = strsplit(line);

    % Parse out T datapoint
    t_field = line(4);
    t_field = strsplit(t_field, "=");
    t_field = erase(t_field, '"');
    t_field = str2num(t_field(2));

    t_raw = [t_raw t_field];

    % Parse out F datapoint
    f_field = line(2);
    f_field = strsplit(f_field, "=");
    f_field = erase(f_field, '"');
    f_field = str2num(f_field(2));

    f_raw = [f_raw f_field];

    % Parse out M datapoint
    m_field = line(3);
    m_field = strsplit(m_field, "=");
    m_field = erase(m_field, '"');
    m_field = str2num(m_field(2));

    m_raw = [m_raw m_field];

    % Parse out CG datapoint
    cg_field = line(1);
    cg_field = strsplit(cg_field, "=");
    cg_field = erase(cg_field, '"');
    cg_field = str2num(cg_field(2));

    cg_raw = [cg_raw cg_field];
end

motor.t_raw = t_raw;
motor.f_raw = f_raw;
motor.m_raw = m_raw;
motor.cg_raw = cg_raw;


%% Process Motor Specs Data
%disp(specs_text)
specs_text = erase(specs_text, "<engine");
specs_text = erase(specs_text, ">");
specs_text = strtrim(specs_text);
specs_text = strsplit(specs_text);

for i = 1:length(specs_text)
    line = specs_text(i);
    spec_line = strsplit(line, "=");

    % Parse out the name and value for each spec.
    field_name = spec_line(1);
    field_value = spec_line(2);
    field_value = erase(field_value, '"');

    % Add the spec to the struct
    % TODO
end

motor.specs_text = specs_text;


%% Generate Time Lookup Table
t_max = max(t_raw);
t_min = min(t_raw);

N_pts = ceil((t_max - t_min) / dT);

time = linspace(t_min, t_max, N_pts);


%% Generate Thrust Lookup Table
thrust = zeros(1, N_pts);
prop_mass = zeros(1, N_pts);

for i = 1:N_pts
    % Interpolate the thrust value at each point
    t = time(i);

    % Find the lower point
    temp = t_raw';
    temp(temp > t) = inf;
    [k_1, dist_1] = dsearchn(temp,t);
    t_1 = t_raw(k_1);

    % Find the greater point
    temp = t_raw';
    %temp(k_1) = inf; % set the prev pt to inf so it can't be closest pt. again
    temp(temp <= t_1) = inf;
    [k_2, dist_2] = dsearchn(temp,t);
    t_2 = t_raw(k_2);
    
    % Calculate the thrust at the interpolated point
    f_1 = f_raw(k_1);
    f_2 = f_raw(k_2);

    f = (f_2 - f_1) / (t_2 - t_1) * (t - t_1) + f_1;

    thrust(i) = f;

    %plot(t_raw, f_raw, "-*", t_1, f_1, 'r*', t_2, f_2, 'k*', t, f, 'g*')
    %input(" ");

    % Calculate the thrust at the interpolated point
    m_1 = m_raw(k_1);
    m_2 = m_raw(k_2);

    m = (m_2 - m_1) / (t_2 - t_1) * (t - t_1) + m_1;

    prop_mass(i) = m;
end

motor.time = time;
motor.thrust_lookup = thrust;
motor.prop_mass_lookup = prop_mass / 1000;

%plot(t_raw, f_raw, "-*", time, thrust, "-*")
%plot(t_raw, m_raw, "-*", time, prop_mass, "-*")

end