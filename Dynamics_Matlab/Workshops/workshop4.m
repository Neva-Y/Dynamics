syms t a(t) b(t) c(t) d(t) e(t) f(t) L1 L2 L3
% The absolute linear velocity of point A, expressed in frame {0}.
% variable name: rOA_0_dot
rOA = [0;0;0]
rOA_0_dot = diff(rOA,t)
% The absolute linear velocity of point A, expressed in frame {2}.
% variable name: rOA_2_dot

R01 = [cos(a) -sin(a) 0 ; sin(a) cos(a) 0 ; 0 0 1];
R12 = [cos(b) 0 sin(b); 0 1 0; -sin(b) 0 cos(b)];
R20 = transpose(R01*R12);
rOA_2 = R20*roA;

w10_1 = [0;0;diff(a,t)];
w21_2 = [0;diff(b,t);0];
w2_2 = R01*R12*w21_2 + R01*w10_1


rOA_2_dot = diff(R20*roA) + cross(w2_2,rOA_2)