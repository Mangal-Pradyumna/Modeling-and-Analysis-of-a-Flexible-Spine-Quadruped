%% Initializaion

clear all;
close all;
clc;

% Defining the parameters

g = -9.80665;      % Acceleration due to gravity (m/s^2)
l = 3;             % Length of the block (m)
b = l;             % Width of the block (m)
rho = 1000;        % density of the block (kg/m^3)
h = 0.5;           % Height/thickness of the blocks (m)

l_walk = 50;           % Length of walking path (m)
b_walk = 15;           % width of walking path (m)
h_walk = 0.1;          % Height/thickness of walking path (m)

l_thigh = 0.75;         % length of the upper part of leg (m)
b_thigh = 0.05;
t_thigh = b_thigh;      % thickness of the upper part of legs (m)

t_feet = t_thigh*1.1;           % length of square cross section of feet (m)
b_feet = t_feet*1.1;
l_feet = 0.3*t_feet;      % feet thickness, feet makes contact with the ground (m)
r_feet_sphere = l_feet/2;

contact_stiffness = abs(l*b*h*rho*2*g/0.1);    % approx Weight/ deflection (N/m)
contact_damping = contact_stiffness/10;     % Arbitrary Value (can be tuned) (Ns/m)

joint_stiffness = abs(l*b*h*rho*g*l/10);        % moment created by other block / deflection angle (Nm/deg)
joint_damping = joint_stiffness/10;             % Arbitrary Value (can be tuned) (Ns/m)

% center of leg endpoint rotation, measured from body-leg hinge point.
rotation_center = -0.95*l_thigh;
y_centre = 0.75*(l_thigh*2 + h_walk/2 + l_feet/2);
%% Getting the angular displacement for rear leg

% Link lengths
L1 = l_thigh;  % Length of link 1
L2 = l_thigh;  % Length of link 2



%%

%% Getting the angular displacement for front leg - theta_f_up

% Circle parameters
r = 0.5*l_thigh;        % Radius of the circular path
x0 = 0.9*l_thigh;       % Center of the circle (x-coordinate)
y0 = 0;       % Center of the circle (y-coordinate)
omega = -2*pi; % Angular velocity (rad/s)
tf = 10;        % Final time (s)

% Time vector
t = 0:0.01:10;  % 100 time steps

% Preallocate arrays for theta1, theta2, angular velocities, x, y
theta1 = zeros(1, length(t));
theta2 = zeros(1, length(t));
theta1_dot = zeros(1, length(t));
theta2_dot = zeros(1, length(t));
x_traj = zeros(1, length(t));
y_traj = zeros(1, length(t));

% Loop through time to compute angular displacements and velocities
for i = 1:length(t)
    % Desired end effector position (circular trajectory)
    x = x0 + r * cos(omega * t(i));
    y = y0 + r * sin(omega * t(i));
    x_traj(i) = x;
    y_traj(i) = y;

    % Inverse kinematics: compute theta2 first
    d = sqrt(x^2 + y^2);  % Distance to the end effector
    cos_theta2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
    theta2(i) = acos(cos_theta2);  % Second joint angle

    % Compute theta1
    theta1(i) = atan2(y, x) - atan2(L2 * sin(theta2(i)), L1 + L2 * cos(theta2(i)));

    % Compute angular velocities by differentiating w.r.t. time
    if i > 1
        theta1_dot(i) = (theta1(i) - theta1(i-1)) / (t(i) - t(i-1));
        theta2_dot(i) = (theta2(i) - theta2(i-1)) / (t(i) - t(i-1));
    end
end

% Plot the joint angles theta1 and theta2 over time
figure;
subplot(2, 1, 1);
plot(t, rad2deg(theta1), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('\theta_1 (degrees)');
title('Joint Angle \theta_1 over Time');
grid on;

subplot(2, 1, 2);
plot(t, rad2deg(theta2), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('\theta_2 (degrees)');
title('Joint Angle \theta_2 over Time');
grid on;


% Plot angular velocities
figure;
subplot(2, 1, 1);
plot(t, rad2deg(theta1_dot), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('{\omega_1} (degrees/s)');
title('Angular Velocity {\omega_1} over Time');
grid on;

subplot(2, 1, 2);
plot(t, rad2deg(theta2_dot), 'LineWidth', 2);
xlabel('Time (s)');
ylabel('{\omega_2} (degrees/s)');
title('Angular Velocity {\omega_2} over Time');
grid on;

theta_f_up = timeseries(theta1',t);
init_theta_f_up = theta1(1);
theta_f_down = timeseries(theta2',t);
init_theta_f_down = theta2(1); 



%%

%% Getting the angular displacement for front leg - theta_b_up

% Circle parameters
r = 0.5*l_thigh;        % Radius of the circular path
x0 = 0.9*l_thigh;       % Center of the circle (x-coordinate)
y0 = 0;       % Center of the circle (y-coordinate)
omega = 1.4*pi; % Angular velocity (rad/s)
tf = 10;        % Final time (s)

% Time vector
t = 0:0.01:10;  % 100 time steps

% Preallocate arrays for theta1, theta2, angular velocities, x, y
theta1 = zeros(1, length(t));
theta2 = zeros(1, length(t));
theta1_dot = zeros(1, length(t));
theta2_dot = zeros(1, length(t));
x_traj = zeros(1, length(t));
y_traj = zeros(1, length(t));

% Loop through time to compute angular displacements and velocities
for i = 1:length(t)
    % Desired end effector position (circular trajectory)
    x = x0 + r * cos(omega * t(i)+ 3*pi/4);
    y = y0 + r * sin(omega * t(i)+ 3*pi/4);
    x_traj(i) = x;
    y_traj(i) = y;

    % Inverse kinematics: compute theta2 first
    d = sqrt(x^2 + y^2);  % Distance to the end effector
    cos_theta2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
    theta2(i) = acos(cos_theta2);  % Second joint angle

    % Compute theta1
    theta1(i) = atan2(y, x) - atan2(L2 * sin(theta2(i)), L1 + L2 * cos(theta2(i)));

    % Compute angular velocities by differentiating w.r.t. time
    if i > 1
        theta1_dot(i) = (theta1(i) - theta1(i-1)) / (t(i) - t(i-1));
        theta2_dot(i) = (theta2(i) - theta2(i-1)) / (t(i) - t(i-1));
    end
end

theta_b_up = timeseries(theta1',t);
init_theta_b_up = theta1(1);
theta_b_down = timeseries(theta2',t);
init_theta_b_down = theta2(1);



%%

%% Getting the angular displacement for front leg - theta_f_up2

% Circle parameters
r = 0.5*l_thigh;        % Radius of the circular path
x0 = 0.9*l_thigh;       % Center of the circle (x-coordinate)
y0 = 0;       % Center of the circle (y-coordinate)
omega = 2*pi; % Angular velocity (rad/s)
tf = 10;        % Final time (s)

% Time vector
t = 0:0.01:10;  % 100 time steps

% Preallocate arrays for theta1, theta2, angular velocities, x, y
theta1 = zeros(1, length(t));
theta2 = zeros(1, length(t));
theta1_dot = zeros(1, length(t));
theta2_dot = zeros(1, length(t));
x_traj = zeros(1, length(t));
y_traj = zeros(1, length(t));

% Loop through time to compute angular displacements and velocities
for i = 1:length(t)
    % Desired end effector position (circular trajectory)
    x = x0 + r * cos(omega * t(i)+ pi/4);
    y = y0 + r * sin(omega * t(i)+ pi/4);
    x_traj(i) = x;
    y_traj(i) = y;

    % Inverse kinematics: compute theta2 first
    d = sqrt(x^2 + y^2);  % Distance to the end effector
    cos_theta2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
    theta2(i) = acos(cos_theta2);  % Second joint angle

    % Compute theta1
    theta1(i) = atan2(y, x) - atan2(L2 * sin(theta2(i)), L1 + L2 * cos(theta2(i)));

    % Compute angular velocities by differentiating w.r.t. time
    if i > 1
        theta1_dot(i) = (theta1(i) - theta1(i-1)) / (t(i) - t(i-1));
        theta2_dot(i) = (theta2(i) - theta2(i-1)) / (t(i) - t(i-1));
    end
end

theta_f_up2 = timeseries(theta1',t);
init_theta_f_up2 = theta1(1);
theta_f_down2 = timeseries(theta2',t);
init_theta_f_down2 = theta2(1);



%%

%% Getting the angular displacement for front leg - theta_b_up2

% Circle parameters
r = 0.5*l_thigh;        % Radius of the circular path
x0 = 0.9*l_thigh;       % Center of the circle (x-coordinate)
y0 = 0;       % Center of the circle (y-coordinate)
omega = 1.4*pi; % Angular velocity (rad/s)
tf = 10;        % Final time (s)

% Time vector
t = 0:0.01:10;  % 100 time steps

% Preallocate arrays for theta1, theta2, angular velocities, x, y
theta1 = zeros(1, length(t));
theta2 = zeros(1, length(t));
theta1_dot = zeros(1, length(t));
theta2_dot = zeros(1, length(t));
x_traj = zeros(1, length(t));
y_traj = zeros(1, length(t));

% Loop through time to compute angular displacements and velocities
for i = 1:length(t)
    % Desired end effector position (circular trajectory)
    x = x0 + r * cos(omega * t(i)+ pi);
    y = y0 + r * sin(omega * t(i)+ pi);
    x_traj(i) = x;
    y_traj(i) = y;

    % Inverse kinematics: compute theta2 first
    d = sqrt(x^2 + y^2);  % Distance to the end effector
    cos_theta2 = (x^2 + y^2 - L1^2 - L2^2) / (2 * L1 * L2);
    theta2(i) = acos(cos_theta2);  % Second joint angle

    % Compute theta1
    theta1(i) = atan2(y, x) - atan2(L2 * sin(theta2(i)), L1 + L2 * cos(theta2(i)));

    % Compute angular velocities by differentiating w.r.t. time
    if i > 1
        theta1_dot(i) = (theta1(i) - theta1(i-1)) / (t(i) - t(i-1));
        theta2_dot(i) = (theta2(i) - theta2(i-1)) / (t(i) - t(i-1));
    end
end

theta_b_up2 = timeseries(theta1',t);
init_theta_b_up2 = theta1(1);
theta_b_down2 = timeseries(theta2',t);
init_theta_b_down2 = theta2(1);

%% Running the simulink model

sim("phase_3.slx");
