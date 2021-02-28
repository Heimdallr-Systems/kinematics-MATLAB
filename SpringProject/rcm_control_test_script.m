% cm controller test script

clear
clc
close all

r_II_B_0 = [0;0;0.17];
T_I_B_0 = eye(3);
Theta1_0 = [pi/4;-pi/4;-pi/4;pi/4];
Theta2_0 = [0;0;0;0];
Theta3_0 = [pi/2;-pi/2;pi/2;-pi/2];

[r_II_c_FR,r_II_c_FL,r_II_c_BR,r_II_c_BL] = CPos_wrt_I(Theta1_0, Theta2_0, Theta3_0, T_I_B_0, r_II_B_0);
r_II_c = [r_II_c_FR,r_II_c_FL,r_II_c_BR,r_II_c_BL];

Theta_0 = [Theta1_0;Theta2_0;Theta3_0];
rcm_0 = compute_rcm(Theta_0, r_II_B_0, T_I_B_0)
FK_Solver_Draw(Theta1_0,Theta2_0,Theta3_0,T_I_B_0,r_II_B_0,rcm_0)

rcm_d = [0;0];
[Theta1, Theta2, Theta3] = CM_Controller(Theta_0,rcm_0,rcm_d,r_II_B_0,T_I_B_0,[1,1,1,1]);

[r_BB_c_FR,r_BB_c_FL,r_BB_c_BR,r_BB_c_BL] = CPos_wrt_B(Theta1, Theta2, Theta3);
r_BB_c = [r_BB_c_FR,r_BB_c_FL,r_BB_c_BR,r_BB_c_BL];

legs_on_gnd = [1;1;1;1];

[T_I_B,r_II_B] = IK_Solver_BodyRot_BodyPos(r_BB_c, r_II_c, legs_on_gnd);

Theta = [Theta1;Theta2;Theta3];
rcm_1 = compute_rcm(Theta, r_II_B, T_I_B)
figure
FK_Solver_Draw(Theta1,Theta2,Theta3,T_I_B,r_II_B,rcm_1)