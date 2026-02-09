function tau = pd_controller(e_vec, omega, Kp, Kd)

tau = -Kp*e_vec - Kd*omega;

end
% TODO: Add saturation