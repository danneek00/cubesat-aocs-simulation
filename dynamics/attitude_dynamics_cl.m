function xdot = attitude_dynamics_cl(~, x, J, qd, Kp, Kd)

q = x(1:4);
omega = x(5:7);

% Normalize quaternion
q = q / norm(q);

% Attitude error
[~, e_vec] = attitude_error(q, qd);

% Control torque
tau = pd_controller(e_vec, omega, Kp, Kd);

% Quaternion kinematics
qdot = quaternion_kinematics(q, omega);

% Rotational dynamics
omega_dot = J \ (tau - cross(omega, J*omega));

xdot = [qdot; omega_dot];

end