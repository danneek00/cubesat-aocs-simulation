clear; clc; close all;
addpath(genpath(pwd));

MODE = "closed";   % "open" or "closed"

%% Simulation parameters
tspan = [0 120];              % simulation time [s]

% Spacecraft inertia matrix [kg m^2]
J = diag([0.02, 0.025, 0.015]);

%% Initial conditions

% Initial attitude (quaternion: inertial -> body)
q0 = [1; 0; 0; 0];             % identity rotation

% Initial angular velocity [rad/s]
omega0 = deg2rad([0.5; -0.3; 0.2]);

% State vector: [q; omega]
x0 = [q0; omega0];

%% Integrate dynamics open loop
%opts = odeset('RelTol',1e-9,'AbsTol',1e-9);
opts = odeset('RelTol',1e-9,'AbsTol',1e-9,'Stats','on');


switch MODE
    
    case "open"
        dyn = @(t,x) attitude_dynamics(t, x, J);

    case "closed"
        qd = [1;0;0;0];
        %Kp = 0.02*eye(3);
        %Kd = 0.08*eye(3);

        Kp = 0.01*eye(3);
        Kd = 0.08*eye(3);
        Kd = 3*Kd;
        dyn = @(t,x) attitude_dynamics_cl(t, x, J, qd, Kp, Kd);

    otherwise 
        error("Unknown MODE");

end 

[t, x] = ode45(dyn, tspan, x0, opts);

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

omega_norm = vecnorm(omega,2,2);
figure; plot(t, omega_norm); grid on;
xlabel('Time [s]'); ylabel('||\omega|| [rad/s]');
title('Rate magnitude');