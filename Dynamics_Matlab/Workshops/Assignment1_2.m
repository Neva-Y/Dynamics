syms R d t alpha_(t)
% Q1: what is the position vector r_OG in frame 1? 
% Variable Name:  rOG_1

rOP_0 = [-R; 0; 0];
rGP_1 = [-d; 0; 0]

R01 = [cos(alpha_) sin(alpha_) 0; -sin(alpha_) cos(alpha_) 0; 0 0 1];
rOG_1 = transpose(R01)*rOP_0 - rGP_1

% Q2: what is the center of mass velocity in frame 1?
% Variable Name: rOG_1_dot
w10_1 = [0;0;-diff(alpha_, t)]
rOG_1_dot = diff(rOG_1, t) + cross(w10_1, rOG_1)


