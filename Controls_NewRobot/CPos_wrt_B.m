function [rc_FR, rc_FL, rc_BR, rc_BL] = CPos_wrt_B(Theta1,Theta2,Theta3)
% This function gives the positions of the contact points of the robot with
% respect to the body frame.
% Theta1 = [Theta1_FR, Theta1_FL, Theta1_BR, Theta1_BL]
% Theta2 = [Theta2_FR, Theta2_FL, Theta2_BR, Theta2_BL]
% Theta3 = [Theta3_FR, Theta3_FL, Theta3_BR, Theta3_BL]

jnt_var = [Theta1(1),Theta2(1),Theta3(1),...
                 Theta1(2),Theta2(2),Theta3(2),...
                 Theta1(3),Theta2(3),Theta3(3),...
                 Theta1(4),Theta2(4),Theta3(4)];

% Relative Positions
rBfromI = [0;0;0]; % place inertial frame at base
% big plate: Short: 350, L: 700;
% Relative FR Leg Positions
load('RobotConstants.mat')


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
r1_FR = rB + TB*r_BB_1_FR;
r2_FR = r1_FR + T1_FR * r_11_2_FR;
r3_FR = r2_FR + T2_FR * r_22_3_FR;
rc_FR = r3_FR + T3_FR * r_33_c_FR;

% Positions of FL Leg
r1_FL = rB + TB*r_BB_1_FL;
r2_FL = r1_FL + T1_FL * r_11_2_FL;
r3_FL = r2_FL + T2_FL * r_22_3_FL;
rc_FL = r3_FL + T3_FL * r_33_c_FL;

% Position of BR Leg
r1_BR = rB + TB*r_BB_1_BR;
r2_BR = r1_BR + T1_BR * r_11_2_BR;
r3_BR = r2_BR + T2_BR * r_22_3_BR;
rc_BR = r3_BR + T3_BR * r_33_c_BR;

% Position of BL Leg
r1_BL = rB + TB*r_BB_1_BL;
r2_BL = r1_BL + T1_BL * r_11_2_BL;
r3_BL = r2_BL + T2_BL * r_22_3_BL;
rc_BL = r3_BL + T3_BL * r_33_c_BL;
% disp('r_BB_c_FR')
% disp(rc_FR)
% disp('r_BB_c_FL')
% disp(rc_FL)
% disp('r_BB_c_BR')
% disp(rc_BR)
% disp('r_BB_c_BL')
% disp(rc_BL)

end