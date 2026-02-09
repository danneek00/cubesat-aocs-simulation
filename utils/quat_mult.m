function q = quat_mult(q1, q2)
% q = q1 ⊗ q2, with q = [w x y z]'
w1=q1(1); v1=q1(2:4);
w2=q2(1); v2=q2(2:4);
q = [w1*w2 - dot(v1,v2);
     w1*v2 + w2*v1 + cross(v1,v2)];
end
