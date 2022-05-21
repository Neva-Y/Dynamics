syms t alpha_(t) beta_(t) theta_ R
% Q1:Assign a coordinate system that is rigidly attached to the disc {b}. What is the rotation that brings vectors from the inertial frame {0} to frame {b} 
% variable name: Rb0
R01 = [cos(alpha_) -sin(alpha_) 0; sin(alpha_) cos(alpha_) 0; 0 0 1]
R12 = [1 0 0; 0 cos(beta_) -sin(beta_); 0 sin(beta_) cos(beta_)]
R2b = [cos(theta_) 0 sin(theta_); 0 1 0; -sin(theta_) 0 cos(theta_)]

% Q2: What is the absolute angular velocity of the disk in frame {b}, that is attached to the disk? 
% variable name: wb_b
Rb2 = transpose(R2b)
R21 = transpose(R12)

w10_1 = [0; 0; diff(alpha_)]
w21_2 = [diff(beta_); 0; 0]
wb2_2 = [0; 0; 0]

wb_b = wb2_2 + Rb2*w21_2 + Rb2*R21*w10_1

% Q3: What is the absolute linear velocity of centre of mass of the disk in frame {1}? 
% variable name: rAB_1_dot
rAB_1 = [-4*R; 0; 0]
rAB_1_dot = diff(rAB_1, t) + cross(w10_1,rAB_1);

% Q4: Determine the absolute angular acceleration of the disk in frame {1} 
% variable name: wb_1_dot
wb_1 = R12*R2b*wb_b
wb_1_dot = diff(wb_1, t) + cross(w10_1, wb_1)
