function [GeoJc_FR,GeoJc_FL,GeoJc_BR,GeoJc_BL] = contactJacobians(state)
dotgamma(:,1) = state(19:36);

r_BB_1_FR = [153.4;-53.31;50]./1000;
r_11_2_FR = [0;-65.25;26.25]./1000;
r_22_3_FR = [0;-224.68;0]./1000;
r_33_c_FR = [0;-279;0]./1000;

% Relative FL Leg Positions
r_BB_1_FL = [153.4;53.31;50]./1000;
r_11_2_FL = [0;65.25;26.25]./1000;
r_22_3_FL = [0;224.68;0]./1000;
r_33_c_FL = [0;279;0]./1000;

% Relative BR Leg Positions
r_BB_1_BR = [-153.4;-53.31;50]./1000;
r_11_2_BR = [0;-65.25;26.25]./1000;
r_22_3_BR = [0;-224.68;0]./1000;
r_33_c_BR = [0;-279;0]./1000;

% Relative BL Leg Positions
r_BB_1_BL = [-153.4;53.31;50]./1000;
r_11_2_BL = [0;65.25;26.25]./1000;
r_22_3_BL = [0;224.68;0]./1000;
r_33_c_BL = [0;279;0]./1000;

% Relative Orientations (T_(reference)_(new))
T_I_B = rotz(state(1))*roty(state(2))*rotx(state(3));

T_B_1_FR = rotz(state(7));
T_1_2_FR = rotx(state(11));
T_2_3_FR = rotx(state(15));
T_3_c_FR = T_2_3_FR;

T_B_1_FL = rotz(state(8));
T_1_2_FL = rotx(state(12));
T_2_3_FL = rotx(state(16));
T_3_c_FL = T_2_3_FL;

T_B_1_BR = rotz(state(9));
T_1_2_BR = rotx(state(13));
T_2_3_BR = rotx(state(17));
T_3_c_BR = T_2_3_BR;

T_B_1_BL = rotz(state(10));
T_1_2_BL = rotx(state(14));
T_2_3_BL = rotx(state(18));
T_3_c_BL = T_2_3_BL;

% phi = state(1);
theta = state(2);
psi = state(3);
% dotphi = state(19);
dottheta = state(20);
dotpsi = state(21);

GeoJB = [-sin(theta), 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                    cos(theta)*sin(psi),  cos(psi), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                    cos(psi)*cos(theta), -sin(psi), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
dotGeoJB = [-dottheta*cos(theta), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                         dotpsi*cos(psi)*cos(theta) - dottheta*sin(psi)*sin(theta), -dotpsi*sin(psi), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                         -dotpsi*cos(theta)*sin(psi) - dottheta*cos(psi)*sin(theta), -dotpsi*cos(psi), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];

I1_hat_FR = zeros(3,18);
I1_hat_FR(3,7) = 1; % z-axis, Theta1_FR
I1_tilde_FR = zeros(3,18);
[T_I_1_FR, ~, ~, GeoJ1_FR, dotGeoJ1_FR] = recursiveKin(dotgamma, T_I_B, T_B_1_FR, r_BB_1_FR, I1_hat_FR, I1_tilde_FR, GeoJB, dotGeoJB);

I2_hat_FR = zeros(3,18);
I2_hat_FR(1,11) = 1; % x-axis, Theta2_FR
I2_tilde_FR = zeros(3,18);
[T_I_2_FR, ~, ~, GeoJ2_FR, dotGeoJ2_FR] = recursiveKin(dotgamma, T_I_1_FR, T_1_2_FR, r_11_2_FR, I2_hat_FR, I2_tilde_FR, GeoJ1_FR, dotGeoJ1_FR);

I3_hat_FR = zeros(3,18);
I3_hat_FR(1,15) = 1; % x-axis, Theta3_FR
I3_tilde_FR = zeros(3,18);
[T_I_3_FR, ~, ~, GeoJ3_FR, dotGeoJ3_FR] = recursiveKin(dotgamma, T_I_2_FR, T_2_3_FR, r_22_3_FR, I3_hat_FR, I3_tilde_FR, GeoJ2_FR, dotGeoJ2_FR);

Ic_hat_FR = zeros(3,18);
Ic_tilde_FR = zeros(3,18);
[~, ~, ~, GeoJc_FR, ~] = recursiveKin(dotgamma, T_I_3_FR, T_3_c_FR, r_33_c_FR, Ic_hat_FR, Ic_tilde_FR, GeoJ3_FR, dotGeoJ3_FR);

I1_hat_FL = zeros(3,18);
I1_hat_FL(3,8) = 1; % z-axis, Theta1_FL
I1_tilde_FL = zeros(3,18);
[T_I_1_FL, ~, ~, GeoJ1_FL, dotGeoJ1_FL] = recursiveKin(dotgamma, T_I_B, T_B_1_FL, r_BB_1_FL, I1_hat_FL, I1_tilde_FL, GeoJB, dotGeoJB);

I2_hat_FL = zeros(3,18);
I2_hat_FL(1,12) = 1; % x-axis, Theta2_FL
I2_tilde_FL = zeros(3,18);
[T_I_2_FL, ~, ~, GeoJ2_FL, dotGeoJ2_FL] = recursiveKin(dotgamma, T_I_1_FL, T_1_2_FL, r_11_2_FL, I2_hat_FL, I2_tilde_FL, GeoJ1_FL, dotGeoJ1_FL);

I3_hat_FL = zeros(3,18);
I3_hat_FL(1,16) = 1; % x-axis, Theta3_FL
I3_tilde_FL = zeros(3,18);
[T_I_3_FL, ~, ~, GeoJ3_FL, dotGeoJ3_FL] = recursiveKin(dotgamma, T_I_2_FL, T_2_3_FL, r_22_3_FL, I3_hat_FL, I3_tilde_FL, GeoJ2_FL, dotGeoJ2_FL);

Ic_hat_FL = zeros(3,18);
Ic_tilde_FL = zeros(3,18);
[~, ~, ~, GeoJc_FL, ~] = recursiveKin(dotgamma, T_I_3_FL, T_3_c_FL, r_33_c_FL, Ic_hat_FL, Ic_tilde_FL, GeoJ3_FL, dotGeoJ3_FL);

I1_hat_BR = zeros(3,18);
I1_hat_BR(3,9) = 1; % z-axis, Theta1_BR
I1_tilde_BR = zeros(3,18);
[T_I_1_BR, ~, ~, GeoJ1_BR, dotGeoJ1_BR] = recursiveKin(dotgamma, T_I_B, T_B_1_BR, r_BB_1_BR, I1_hat_BR, I1_tilde_BR, GeoJB, dotGeoJB);

I2_hat_BR = zeros(3,18);
I2_hat_BR(1,13) = 1; % x-axis, Theta2_BR
I2_tilde_BR = zeros(3,18);
[T_I_2_BR, ~, ~, GeoJ2_BR, dotGeoJ2_BR] = recursiveKin(dotgamma, T_I_1_BR, T_1_2_BR, r_11_2_BR, I2_hat_BR, I2_tilde_BR, GeoJ1_BR, dotGeoJ1_BR);

I3_hat_BR = zeros(3,18);
I3_hat_BR(1,17) = 1; % x-axis, Theta3_BR
I3_tilde_BR = zeros(3,18);
[T_I_3_BR, ~, ~, GeoJ3_BR, dotGeoJ3_BR] = recursiveKin(dotgamma, T_I_2_BR, T_2_3_BR, r_22_3_BR, I3_hat_BR, I3_tilde_BR, GeoJ2_BR, dotGeoJ2_BR);

Ic_hat_BR = zeros(3,18);
Ic_tilde_BR = zeros(3,18);
[~, ~, ~, GeoJc_BR, ~] = recursiveKin(dotgamma, T_I_3_BR, T_3_c_BR, r_33_c_BR, Ic_hat_BR, Ic_tilde_BR, GeoJ3_BR, dotGeoJ3_BR);

I1_hat_BL = zeros(3,18);
I1_hat_BL(3,10) = 1; % z-axis, Theta1_BL
I1_tilde_BL = zeros(3,18);
[T_I_1_BL, ~, ~, GeoJ1_BL, dotGeoJ1_BL] = recursiveKin(dotgamma, T_I_B, T_B_1_BL, r_BB_1_BL, I1_hat_BL, I1_tilde_BL, GeoJB, dotGeoJB);

I2_hat_BL = zeros(3,18);
I2_hat_BL(1,14) = 1; % x-axis, Theta2_BL
I2_tilde_BL = zeros(3,18);
[T_I_2_BL, ~, ~, GeoJ2_BL, dotGeoJ2_BL] = recursiveKin(dotgamma, T_I_1_BL, T_1_2_BL, r_11_2_BL, I2_hat_BL, I2_tilde_BL, GeoJ1_BL, dotGeoJ1_BL);

I3_hat_BL = zeros(3,18);
I3_hat_BL(1,18) = 1; % x-axis, Theta3_BL
I3_tilde_BL = zeros(3,18);
[T_I_3_BL, ~, ~, GeoJ3_BL, dotGeoJ3_BL] = recursiveKin(dotgamma, T_I_2_BL, T_2_3_BL, r_22_3_BL, I3_hat_BL, I3_tilde_BL, GeoJ2_BL, dotGeoJ2_BL);

Ic_hat_BL = zeros(3,18);
Ic_tilde_BL = zeros(3,18);
[~, ~, ~, GeoJc_BL, ~] = recursiveKin(dotgamma, T_I_3_BL, T_3_c_BL, r_33_c_BL, Ic_hat_BL, Ic_tilde_BL, GeoJ3_BL, dotGeoJ3_BL);


end