clc
clear
close all

syms t alpha_(t) beta_(t) theta_ R m g

%% Assignment 1 pretests
% Coordinate system that is rigidly attached to the disc {b} (Pretest)
% variable name: Rb0
R01 = [cos(alpha_) -sin(alpha_) 0; sin(alpha_) cos(alpha_) 0; 0 0 1];
R12 = [1 0 0; 0 cos(beta_) -sin(beta_); 0 sin(beta_) cos(beta_)];
R2b = [cos(theta_) 0 sin(theta_); 0 1 0; -sin(theta_) 0 cos(theta_)];
Rb0 = R01 * R12 * R2b

Rb2 = inv(R2b);
R21 = inv(R12);
% Absolute angular velocity of the disk in frame {b}, that is attached to the disk (Pretest)
% Variable name: wb_b
w10_1 = [0; 0; diff(alpha_)];
w21_2 = [diff(beta_); 0; 0];
wb2_2 = [0; 0; 0];

wb_b = wb2_2 + Rb2*w21_2 + Rb2*R21*w10_1;

% Absolute linear velocity of centre of mass of the disk in frame {1} (pretest)
%% Variable name: rAB_1_dot
rAB_1=[-4*R;0;0];
rAB_1_dot = diff(rAB_1, t) + cross(w10_1,rAB_1);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%% New tests %%%
%% Determine the time derivative of the linear momentum of the disc in frame {1}(Test)
%% Varible name: p_dot_1
rAB_1_dotdot = diff(rAB_1_dot, t) + cross(w10_1,rAB_1_dot);

p_dot_1 = m*rAB_1_dotdot;


% Determine the time derivative of the angular momentum of the disc
% about its own centre of mass in frame {b} (test)
% Variable name: hG_b_dot

IGdisc_b = [1/2*m*R^2 0 0;
    0 1/4*m*R^2 0;
    0 0 1/4*m*R^2];

hGdisc_b = IGdisc_b * wb_b;

hG_b_dot = diff(hGdisc_b, t) + cross (wb_b, hGdisc_b);

%% Constraints
syms F_Bx F_By F_Bz M_Bx M_By M_Bz F_Ax F_Ay F_Az M_Ax M_Ay M_Az
% Reaction force to the disc at Point B (COM)
Fdisc_b = [F_Bx; F_By; F_Bz];
% Reaction moment to the rotor
Mdisc_b = [M_Bx; M_By; M_Bz];
% Reaction foce to the shaft at point A
Fshaft_2 = [F_Ax; F_Ay; F_Az];
% Reaction moment to the rotor
Mshaft_2 = [M_Ax; M_Ay; M_Az];
zero_reaction = M_Ax == 0;  

%% NE Equations
Fg_1 = [0;0;-m*g];
Rb1 = R01*Rb0;

lin_NE_disc = Fdisc_b  ==  Rb1*(p_dot_1 - Fg_1)
ang_NE_disc = Mdisc_b == hG_b_dot


