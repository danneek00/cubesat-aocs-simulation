clear; clc; close all;
addpath(genpath(pwd));

%% Simulation parameters
tspan = [0 200];              % simulation time [s]

% Spacecraft inertia matrix [kg m^2]
J = diag([0.02, 0.025, 0.015]);

%% Initial conditions

% Initial attitude (quaternion: inertial -> body)
q0 = [1; 0; 0; 0];             % identity rotation

% Initial angular velocity [rad/s]
omega0 = deg2rad([0.5; -0.3; 0.2]);

% State vector: [q; omega]
x0 = [q0; omega0];

%% Integrate dynamics
opts = odeset('RelTol',1e-9,'AbsTol',1e-9);

[t, x] = ode45(@(t,x) attitude_dynamics(t, x, J), tspan, x0, opts);

%% Extract states
q = x(:,1:4);
omega = x(:,5:7);

%% Plot results
figure;
plot(t, omega);
grid on;
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');
legend('\omega_x','\omega_y','\omega_z');
title('Free rigid-body rotation (no control)');