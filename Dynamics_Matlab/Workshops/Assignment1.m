syms t theta_(t) beta_(t) phi_A(t) phi_B(t) phi_C(t) x(t) y(t) %Time dependent variables
syms H h R r a b d % dimensional variable indicated in the figure

% Q1: Rotation matrix R10 from frame {0} to frame {1} 
% Variable Name: R10
R01 = [cos(theta_) -sin(theta_) 0; sin(theta_) cos(theta_) 0; 0 0 1];
R10 = transpose(R01)

% Q2: Rotation matrix R12 from frame {2} to frame {1} 
% Variable Name: R12
R12 = [cos(beta_) -sin(beta_) 0; sin(beta_) cos(beta_) 0; 0 0 1]

% Q3: Ratation matrix Rf2 from frame {2} to frame {f} 
% Variable Name: Rf2
R2f = [cos(phi_A) 0 sin(phi_A); 0 1 0; -sin(phi_A) 0 cos(phi_A)];
Rf2 = transpose(R2f)

% Q4: Absolute velocity of the front wheel expressed in frame 1
% Variable Name: wf_1
w10_1 = [0;0;diff(theta_,t)];
w21_2 = [0;0;diff(beta_,t)];
wf2_f = [0;diff(phi_A,t);0];

wf_1 = R12*R2f*wf2_f + R12*w21_2 + w10_1


% Q5:1) Assign a coordinate system {v} that is fixed to the left rear wheel. 
% Variable Name: R1v
R1v = [cos(phi_B) 0 sin(phi_B); 0 1 0; -sin(phi_B) 0 cos(phi_B)];
Rv1 = transpose(R1v)

%    2) What is the absolute angular velocity of the left wheel in frame {v} 
% Variable Name: wv_v
wv1_v = [0; diff(phi_B,t); 0];
wv_v = wv1_v + Rv1*w10_1

% Q6 What is the vector from the origin of the inertial frame {0} (point O)  
% to a point J in the perimeter of the front wheel expressed in frame {1} if rOG_0=[x;y;H] and rAJ_f = [0;0;R]? 
% Variable Name: rOJ_1
rOG_0=[x;y;H];
rAJ_f=[0;0;R];
rGA_1 = [d; 0; -(H-R)];
rOJ_1 = R10*rOG_0 + rGA_1 + R12*R2f*rAJ_f


% Q7 What is the vector from the origin of the inertial frame {0} (point O)  
% to a point P in the perimeter of the right wheel expressed in frame {1} if rOG_0=[x;y;H] and rCP_r=[0;0;r]? 
% Variable Name: rOP_1
rCP_r = [0 ; 0; r];
R1r = [cos(phi_C) 0 sin(phi_C); 0 1 0; -sin(phi_C) 0 cos(phi_C)];
rGC_1 = [-a; -b; r-H];
rOP_1 = R10*rOG_0 + rGC_1 + R1r*rCP_r

% Q8 What is the vector from point O to the instantaneous contact point between the left wheel and the ground (point Mcontact) in frame {1}?
% variable name: rOM_1
rOG_0=[x;y;H];
rBM_v = [0;0;r];  
rGB_1 = [-a; b; r-H];
rOM_1 = R10*rOG_0 + rGB_1 + R1v*rBM_v
%rOM_1 = subs(rOM_1,phi_B,pi)


% Q9 What is the absolute linear velocity of a point J in the perimeter of the front wheel in frame{1}?
% Variable Name: rOJ_1_dot
rOJ_1_dot = diff(rOJ_1, t) + cross(w10_1, rOJ_1)


% Q10 What is the absolute linear velocity of a point N in the perimeter of the left wheel in frame {1}?
% Variable Name: rON_1_dot
rBN_v = [0 ; 0; r];
rON_1 = R10*rOG_0 + rGB_1 + R1v*rBN_v;
rON_1_dot = diff(rON_1, t) + cross(w10_1, rON_1)


