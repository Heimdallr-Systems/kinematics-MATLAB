clear
clc
%
 %load('G.mat')
%load('dGdX.mat')
load('dGdX_T.mat')
%disp(G)

diary dGdX_T
disp('function G_ans = dGdX_T(Theta1_0, Theta2_0, Theta3_0, r_II_B_d, r_II_c_leg1, r_II_c_leg2, r_II_c_leg3, r_II_c_leg4, T_I_B_d)')

disp('r_II_B_d_x = r_II_B_d(1);')
disp('r_II_B_d_y = r_II_B_d(2);')
disp('r_II_B_d_z = r_II_B_d(3);')

disp('Theta1_leg1 = Theta1_0(1);')
disp('Theta1_leg2 = Theta1_0(2);')
disp('Theta1_leg3 = Theta1_0(3);')
disp('Theta1_leg4 = Theta1_0(4);')

disp('Theta2_leg1 = Theta2_0(1);')
disp('Theta2_leg2 = Theta2_0(2);')
disp('Theta2_leg3 = Theta2_0(3);')
disp('Theta2_leg4 = Theta2_0(4);')

disp('Theta3_leg1 = Theta3_0(1);')
disp('Theta3_leg2 = Theta3_0(2);')
disp('Theta3_leg3 = Theta3_0(3);')
disp('Theta3_leg4 = Theta3_0(4);')

disp('r_II_c_0_x_leg1 = r_II_c_leg1(1);')
disp('r_II_c_0_y_leg1 = r_II_c_leg1(2);')
disp('r_II_c_0_z_leg1 = r_II_c_leg1(3);')
disp('r_II_c_0_x_leg2 = r_II_c_leg2(1);')
disp('r_II_c_0_y_leg2 = r_II_c_leg2(2);')
disp('r_II_c_0_z_leg2 = r_II_c_leg2(3);')
disp('r_II_c_0_x_leg3 = r_II_c_leg3(1);')
disp('r_II_c_0_y_leg3 = r_II_c_leg3(2);')
disp('r_II_c_0_z_leg3 = r_II_c_leg3(3);')
disp('r_II_c_0_x_leg4 = r_II_c_leg4(1);')
disp('r_II_c_0_y_leg4 = r_II_c_leg4(2);')
disp('r_II_c_0_z_leg4 = r_II_c_leg4(3);')

disp('T_I_B_d_1_1 = T_I_B_d(1,1);')
disp('T_I_B_d_1_2 = T_I_B_d(1,2);')
disp('T_I_B_d_1_3 = T_I_B_d(1,3);')
disp('T_I_B_d_2_1 = T_I_B_d(2,1);')
disp('T_I_B_d_2_2 = T_I_B_d(2,2);')
disp('T_I_B_d_2_3 = T_I_B_d(2,3);')
disp('T_I_B_d_3_1 = T_I_B_d(3,1);')
disp('T_I_B_d_3_2 = T_I_B_d(3,2);')
disp('T_I_B_d_3_3 = T_I_B_d(3,3);')
disp('G_ans = [')
disp(dGdX)
disp('];')
disp('end')
diary off
clc