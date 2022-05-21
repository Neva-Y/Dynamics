syms F_hat vx1 vy1 vz1 wx1 wy1 wz1 m R v
%m = 1;
%R = 1;
%v = 1;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Important: Your solutions can only depend on the known parameters (m,R,v)
% The variables F_hat vx1 vy1 vz1 wx1 wy1 wz1 are the ones you have to solve for


%%%% BEFORE Collision %%%%
%Define position vectors and velocities
%Positions
rGA = [-4*R/(3*pi); -R; 0];


%Velocities
w_pre = [0;0;0];
v_pre = [0;0;-v];

%Inertia
I_semi = [1/4*m*R^2 0 0;
          0 (1/4 - 16/(9*pi^2))*m*R^2 0;
          0 0 1/4*m*R^2 + (1/4 - 16/(9*pi^2))*m*R^2];

%Impulse Force
F_impulse = [0; 0; F_hat];


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% AFTER collision %%%%%
w_post = [wx1; wy1; wz1];
rOG_dot_plus = [vx1; vy1; vz1];



%Find the angular part of the Impulse-Momentum equations
sumMG = cross(rGA, F_impulse);
deltaH = I_semi*(w_post - w_pre);
ang_IM = sumMG == deltaH;

%Find the linear part of the Impulse-Momentum equations
sumF = F_impulse;
deltaP = m*(rOG_dot_plus - v_pre);
lin_IM = sumF == deltaP;

% Coefficient of restitution formula
e = 1;
rGA_dot_plus = cross(w_post, rGA);
rOA_dot_plus = rOG_dot_plus + rGA_dot_plus;
COR_equation = e == - (rOA_dot_plus(3)/-v);

% Solve
S = solve([ang_IM; lin_IM; COR_equation],[F_hat vx1 vy1 vz1 wx1 wy1 wz1])

%Determine the angular velocity of the plate at the instant after collision. 
%Variable name: w1_1plus_sol
w1_1plus_sol = [S.wx1; S.wy1; S.wz1]

%Determine the absolute linear velocity of the centre of mass of the semicircular plate at the instant after collision.
%Variable name: rOG_dot_1_plus_sol
rOG_dot_1_plus_sol = [S.vx1; S.vy1; S.vz1] 


%%Determine the value of the impulsive force on point A of the semicircular plate. 
%Variable name: F_hat_sol
F_hat_sol = S.F_hat
