% Manipulability Test

clear

val = 200;
% r_II_c_FR = [val;-val;0]./1000;
% r_II_c_FL = [val;val;0]./1000;
% r_II_c_BR = [-val;-val;0]./1000;
% r_II_c_BL = [-val;val;0]./1000;
r_II_c_FR = [250;-250;0]./1000;
r_II_c_FL = [100;100;0]./1000;
r_II_c_BR = [-100;-100;0]./1000;
r_II_c_BL = [-100;100;0]./1000;
r_II_c = [r_II_c_FR,r_II_c_FL, r_II_c_BR, r_II_c_BL];

% Body Rotation
phi_degrees = 0;  % (z-rotation)
theta_degrees = 0;  % (y-rotation)
psi_degrees = 0;  % (x-rotation)
phi = phi_degrees*(pi/180);
theta = theta_degrees*(pi/180);
psi = psi_degrees*(pi/180);
BodyRot = [phi,theta,psi];
T_I_B = rotz(phi)*roty(theta)*rotx(psi)

% Body Offset from Inertial
r_II_B = [0;0;250]./1000

% IK for legs
% Assuming we have some goal orientation and position known, IK lets us
% find the required or target joint angles for the robot to achieve.
% Forward kinematics is then applied to confirm that the robot is in a
% desired position

% check if at least 3 legs are on ground
legs_on_gnd = [r_II_c_FR(3) == 0, r_II_c_FL(3) == 0, r_II_c_BR(3) == 0, r_II_c_BL(3) == 0];

% Solve for joint angles of legs (IK) assuming a desired or known offset of
% the robot and orientation of the robot is known
[Theta1, Theta2, Theta2_2, Theta3, Theta3_2] = IK_Solver_Legs_Inertial(r_II_c, T_I_B, r_II_B, [1,1,1,1]);

% draw robot to confirm (FK)



%%
Theta1 = [0;0;0;pi/4];
Theta2_2 = [0;0;0;0];
Theta3_2 = [0;0;0;pi/2];
FK_Solver_Draw_CM(Theta1, Theta2_2, Theta3_2, T_I_B, r_II_B);

state(1) = phi;
state(2) = theta;
state(3) = psi;
state(4) = r_II_B(1);
state(5) = r_II_B(2);
state(6) = r_II_B(3);
state(7:10) = Theta1;
state(11:14) = Theta2_2;
state(15:18) = Theta3_2;

[muFR, muFL, muBR, muBL] = manipulability(state);
muFR
muFL
muBR
muBL