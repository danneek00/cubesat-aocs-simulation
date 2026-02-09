function [q_e, e_vec, theta] = attitude_error(q, qd)
% q, qd are [w x y z]' (inertial->body), unit quaternions

q  = q / norm(q);
qd = qd / norm(qd);

q_inv = [q(1); -q(2:4)];
q_e = quat_mult(qd, q_inv);

% avoid unwinding
if q_e(1) < 0
    q_e = -q_e;
end

e_vec = -q_e(2:4);

% absolute attitude error angle
theta = 2*atan2(norm(e_vec), q_e(1));
end