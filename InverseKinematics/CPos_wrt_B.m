function [rc_FR, rc_FL, rc_BR, rc_BL] = CPos_wrt_B(Theta1,Theta2,Theta3)
% This function gives the positions of the contact points of the robot with
% respect to the body frame.
% jnt_var = [FR_vars,FL_vars,BR_vars,BL_vars]
% FR_vars = [Theta1(1),Theta2(1),Theta3(1)];
% FL_vars = [Theta1(2),Theta2(2),Theta3(2)];
% BR_vars = [Theta1(3),Theta2(3),Theta3(3)];
% BL_vars = [Theta1(4),Theta2(4),Theta3(4)];   

jnt_var = [Theta1(1),Theta2(1),Theta3(1),...
                 Theta1(2),Theta2(2),Theta3(2),...
                 Theta1(3),Theta2(3),Theta3(3),...
                 Theta1(4),Theta2(4),Theta3(4)];

% Relative Positions
rBfromI = [0;0;0]; % place inertial frame at base
% big plate: Short: 350, L: 700;
% Relative FR Leg Positions
r1fromB_FR = [153.4;-53.31;50];
r2from1_FR = [0;-65.25;26.25];
r3from2_FR = [0;-224.68;0];
rcfrom3_FR = [0;-279;0];

% Relative FL Leg Positions
r1fromB_FL = [153.4;53.31;50];
r2from1_FL = [0;65.25;26.25];
r3from2_FL = [0;224.68;0];
rcfrom3_FL = [0;279;0];

% Relative BR Leg Positions
r1fromB_BR = [-153.4;-53.31;50];
r2from1_BR = [0;-65.25;26.25];
r3from2_BR = [0;-224.68;0];
rcfrom3_BR = [0;-279;0];

% Relative BL Leg Positions
r1fromB_BL = [-153.4;53.31;50];
r2from1_BL = [0;65.25;26.25];
r3from2_BL = [0;224.68;0];
rcfrom3_BL = [0;279;0];


%% Orientations wrt I

% Orientation of Body
% TB = rotx(jnt_var(13))*roty(jnt_var(14))*rotz(jnt_var(15));
TB = eye(3);

% Orientation of FR leg
T1_FR = TB*rotz(jnt_var(1));
T2_FR = T1_FR * rotx(jnt_var(2));
T3_FR = T2_FR * rotx(jnt_var(3));

% Orientation of FR leg
T1_FL = TB*rotz(jnt_var(4));
T2_FL = T1_FL * rotx(jnt_var(5));
T3_FL = T2_FL * rotx(jnt_var(6));

% Orientation of BR leg
T1_BR = TB*rotz(jnt_var(7));
T2_BR = T1_BR * rotx(jnt_var(8));
T3_BR = T2_BR * rotx(jnt_var(9));

% Orientation of BL leg
T1_BL = TB*rotz(jnt_var(10));
T2_BL = T1_BL * rotx(jnt_var(11));
T3_BL = T2_BL * rotx(jnt_var(12));
               
%% Positions wrt I
rB = rBfromI;

% Positions of FR Leg
r1_FR = rB + TB*r1fromB_FR;
r2_FR = r1_FR + T1_FR * r2from1_FR;
r3_FR = r2_FR + T2_FR * r3from2_FR;
rc_FR = r3_FR + T3_FR * rcfrom3_FR;

% Positions of FL Leg
r1_FL = rB + TB*r1fromB_FL;
r2_FL = r1_FL + T1_FL * r2from1_FL;
r3_FL = r2_FL + T2_FL * r3from2_FL;
rc_FL = r3_FL + T3_FL * rcfrom3_FL;

% Position of BR Leg
r1_BR = rB + TB*r1fromB_BR;
r2_BR = r1_BR + T1_BR * r2from1_BR;
r3_BR = r2_BR + T2_BR * r3from2_BR;
rc_BR = r3_BR + T3_BR * rcfrom3_BR;

% Position of BL Leg
r1_BL = rB + TB*r1fromB_BL;
r2_BL = r1_BL + T1_BL * r2from1_BL;
r3_BL = r2_BL + T2_BL * r3from2_BL;
rc_BL = r3_BL + T3_BL * rcfrom3_BL;
% disp('r_BB_c_FR')
% disp(rc_FR)
% disp('r_BB_c_FL')
% disp(rc_FL)
% disp('r_BB_c_BR')
% disp(rc_BR)
% disp('r_BB_c_BL')
% disp(rc_BL)

end