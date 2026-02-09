function xdot = attitude_dynamics(~, x, J)

% State unpacking
q = x(1:4);        % quaternion
omega = x(5:7);    % angular velocity [rad/s]

% Normalize quaternion
q = q / norm(q);

% Quaternion kinematics
qdot = quaternion_kinematics(q, omega);

% Rigid-body rotational dynamics (no torque)
omega_dot = J \ ( -cross(omega, J*omega) );

% State derivative
xdot = [qdot; omega_dot];

end
