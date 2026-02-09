function qdot = quaternion_kinematics(q, omega)

% Ensure column vectors
q = q(:);
omega = omega(:);

% Omega matrix
Omega = [  0       -omega';
           omega   -skew(omega) ];

% Quaternion derivative
qdot = 0.5 * Omega * q;

end

function S = skew(v)
S = [   0   -v(3)  v(2);
      v(3)    0   -v(1);
     -v(2)  v(1)    0  ];
end
