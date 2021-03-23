clear
clc

% first joint angles
% FR
Theta_d(1,1) = pi/4;
Theta_d(5,1) = -pi/4;
Theta_d(9,1) = 3*pi/4;

% FL
Theta_d(2,1) = -pi/4;
Theta_d(6,1) = pi/4;
Theta_d(10,1) = -3*pi/4;

% BR
Theta_d(3,1) = -pi/4;
Theta_d(7,1) = -pi/4;
Theta_d(11,1) = 3*pi/4;

% BL
Theta_d(4,1) = pi/4;
Theta_d(8,1) = pi/4;
Theta_d(12,1) = -3*pi/4;

Theta1(:,1) = [Theta_d(1);Theta_d(2);Theta_d(3);Theta_d(4)];
Theta2(:,1) = [Theta_d(5);Theta_d(6);Theta_d(7);Theta_d(8)];
Theta3(:,1) = [Theta_d(9);Theta_d(10);Theta_d(11);Theta_d(12)];

% [T_I_B_markley,r_II_B_markley] = IK_Solver_BodyRot_BodyPos(r_BB_c, r_II_c, legs_on_gnd);
T_I_B_markley = eye(3);
r_II_B_markley = [0;0;0.08];


[r_II_c_FR_sol, r_II_c_FL_sol, r_II_c_BR_sol, r_II_c_BL_sol] = CPos_wrt_I(Theta1, Theta2, Theta3,T_I_B_markley,r_II_B_markley)
r_II_c = [r_II_c_FR_sol, r_II_c_FL_sol, r_II_c_BR_sol, r_II_c_BL_sol];
[r_BB_c_FR, r_BB_c_FL, r_BB_c_BR, r_BB_c_BL] = CPos_wrt_B(Theta1, Theta2, Theta3);
r_BB_c = [r_BB_c_FR,r_BB_c_FL,r_BB_c_BR,r_BB_c_BL];
FK_Solver_Draw(Theta1, Theta2, Theta3, T_I_B_markley, r_II_B_markley);

%%
clc
[Theta1, ~, Theta2, ~, Theta3] = IK_Solver_Legs_Inertial(r_II_c, eye(3), [0;0;0.22], [1;1;1;1])
FK_Solver_Draw(Theta1, Theta2, Theta3, eye(3), [0;0;0.22])

%%
clear
clc
syms phi theta psi alpha beta r_II_B_x r_II_B_y r_II_B_z r_BB_cam_x r_BB_cam_y r_BB_cam_z

r_II_B = [r_II_B_x;r_II_B_y;r_II_B_z];
r_BB_cam = [r_BB_cam_x;r_BB_cam_y;r_BB_cam_z];
T_I_B = rotz(phi)*roty(theta)*rotx(psi);

r_II_cam = r_II_B + T_I_B*r_BB_cam
T_B_cam = rotz(alpha)*roty(beta)*rotx(0)


%%
clear
clc
close all

T_I_B = eye(3);
r_II_B = [0;0;0.22];
stage = uint8(1);
Theta = zeros(12,1);
FK_Solver_Draw(Theta(1:4),Theta(5:8),Theta(9:12),T_I_B,r_II_B);
while stage ~= 0 
    [Theta, stage] = getUp(Theta, stage);
    FK_Solver_Draw(Theta(1:4),Theta(5:8),Theta(9:12),T_I_B,r_II_B);
end