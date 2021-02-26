% Given: the position of the contact points with respect to the inertial
% frame, the rotation of the body with respect to the inertial frame, and
% the position of the body with respect to the inertial frame
% r_II_c, T_I_B, r_II_B
%
% Purpose: Confirms that the inverse kinematics for the legs, and the tilt
% of the body matches with the forward kinematics.
%
% Output: x/y-rotation of the body, z-height of the body, position of the
% contact points with respect to the inertial frame. This assumes the
% z-rotation of the body wrt to the inertial frame, and the x/y-position of
% the body wrt to the inertial frame have been kept track of in the robot's
% memory.
% r_II_c, T_I_B(x & y), r_II_B(z)
%
% "_sol" means solution or solved version of variable

clear
clc
close all

% Contact Point Positions
r_II_c_FR = [200;-200;0]./1000;
r_II_c_FL = [200;200;0]./1000;
r_II_c_BR = [-200;-200;0]./1000;
r_II_c_BL = [-200;200;0]./1000;
r_II_c = [r_II_c_FR,r_II_c_FL, r_II_c_BR, r_II_c_BL];

% Body Rotation
phi_degrees = 10;  % (z-rotation)
theta_degrees = 10;  % (y-rotation)
psi_degrees = 10;  % (x-rotation)
phi = phi_degrees*(pi/180);
theta = theta_degrees*(pi/180);
psi = psi_degrees*(pi/180);
BodyRot = [phi,theta,psi];
T_I_B = rotz(phi)*roty(theta)*rotx(psi)

% Body Offset from Inertial
r_II_B = [30;20;200]./1000

%% IK for legs
% Assuming we have some goal orientation and position known, IK lets us
% find the required or target joint angles for the robot to achieve.
% Forward kinematics is then applied to confirm that the robot is in a
% desired position

% check if at least 3 legs are on ground
legs_on_gnd = [r_II_c_FR(3) == 0, r_II_c_FL(3) == 0, r_II_c_BR(3) == 0, r_II_c_BL(3) == 0];

% Solve for joint angles of legs (IK) assuming a desired or known offset of
% the robot and orientation of the robot is known
[Theta1, Theta2, Theta2_2, Theta3, Theta3_2] = IK_Solver_Legs_Inertial(r_II_c, T_I_B, r_II_B, [1,1,1,1]);

% display joint angles in degrees
Theta1_Degrees = Theta1 .* 180/pi;
Theta2_Degrees = Theta2 .* 180/pi;
Theta2_2_Degrees = Theta2_2 .* 180/pi;
Theta3_Degrees = Theta3 .* 180/pi;
Theta3_2_Degrees = Theta3_2 .* 180/pi;

% draw robot to confirm (FK)
FK_Solver_Draw_CM(Theta1, Theta2_2, Theta3_2, T_I_B, r_II_B);

% solve for positions of contact points relative to B (FK) now that we know
% the joint angles
[r_BB_c_FR, r_BB_c_FL, r_BB_c_BR, r_BB_c_BL] = CPos_wrt_B(Theta1, Theta2_2, Theta3_2);
r_BB_c = [r_BB_c_FR,r_BB_c_FL,r_BB_c_BR,r_BB_c_BL];

%% Tilt and height
% assuming we have limited information (only joint angles), this function
% will allow us to find the tilt and z-heigh of the robot in order to apply
% controls to maintain balance
%
% this will be useful for computing tilt and height of body independent of
% world frame vectors. With this functions independence on world frame, the
% error or world frame measurements does not propagate, making this
% solution possibly more accurate and useful for balancing.

% confirm tilt and height (IK) assuming flat ground/all four legs touching,
% now that we know the relative contact point positions
X = [0,0,0]; % initial guess for solving for tilt/height
[r_II_B_z_sol, theta_sol, psi_sol] = IK_Solver_Tilt(X,r_BB_c, legs_on_gnd);
r_II_B_z_sol
theta_tilt_degrees_sol = theta_sol*180/pi
psi_tilt_degrees_sol = psi_sol*180/pi

% confirm r_II_c's assuming z-rotation is known (FK) now that we know the
% tilt and joint angles of the robot
ITB = rotz(phi)*roty(theta_sol)*rotx(psi_sol);
[r_II_c_FR_sol, r_II_c_FL_sol, r_II_c_BR_sol, r_II_c_BL_sol] = CPos_wrt_I(Theta1, Theta2_2, Theta3_2,ITB,r_II_B)

%% analytical solution for ITB and IIrB
% this function assumes we know the world frame position of the contact
% points. It gives us the final necessary information for inverse
% kinematics, though we need to keep track of IIrc sufficiently
%
% this function will allow us to reach a desired or goal rotation and
% position in world frame given the position of the legs.
%
% assuming weve kept track of r_II_c sufficiently and we know it, and we
% know r_BB_c which we always will with FK, then find the total rotation matrix
% T_I_B with Markley's Solution
[T_I_B_markley,r_II_B_markley] = IK_Solver_BodyRot_BodyPos(r_BB_c, r_II_c, legs_on_gnd);
T_I_B_markley
r_II_B_markley

