% Uses the draw_robot() function to draw the robot in a desired pose.
% FK_Solver(Theta1, Theta2, Theta3, T_I_B, r_II_B)
% RADIANS
% Theta1 = all legs' first joint angles (FR, FL, BR, BL)
% Theta2 = all legs' second joint angles (FR, FL, BR, BL)
% Theta3 = all legs' third joint angles (FR, FL, BR, BL)
% BodyRot = body zyx rotations (rotz(phi)roty(theta)rotx(psi))
% r_II_B = distance from inertial to body in inertial frame
% Returns r_II_c and r_BB_c, the vectors of the contact points in body and
% inertial frame of each leg.

clear
clc
close all

Theta1 = [0*pi/180,0*pi/180,0*pi/180,0*pi/180];
Theta2 = [0,0,0,0];
Theta3 = [0,0,0,0];

r_II_B = [0;0;200];

phi_degrees = 0;  % (z-rotation)
theta_degrees = 0;  % (y-rotation)
psi_degrees = 0;  % (x-rotation)
phi = phi_degrees*(pi/180);
theta = theta_degrees*(pi/180);
psi = psi_degrees*(pi/180);
BodyRot = [phi,theta,psi];

[r_II_c_FR,r_II_c_FL,r_II_c_BR,r_II_c_BL] = CPos_wrt_I(Theta1,Theta2,Theta3, BodyRot, r_II_B)
[r_BB_c_FR,r_BB_c_FL,r_BB_c_BR,r_BB_c_BL] = CPos_wrt_B(Theta1,Theta2,Theta3);

FK_Solver_Draw(Theta1,Theta2,Theta3,BodyRot,r_II_B,1);