clear
clc

syms Theta1_leg1 Theta1_leg2 Theta1_leg3 Theta1_leg4 real
syms Theta2_leg1 Theta2_leg2 Theta2_leg3 Theta2_leg4 real
syms Theta3_leg1 Theta3_leg2 Theta3_leg3 Theta3_leg4 real
syms r_II_B_d_x r_II_B_d_y r_II_B_d_z real
syms r_II_B_0_x r_II_B_0_y r_II_B_0_z real
syms r_II_c_0_x_leg1 r_II_c_0_y_leg1 r_II_c_0_z_leg1 real
syms r_II_c_0_x_leg2 r_II_c_0_y_leg2 r_II_c_0_z_leg2 real
syms r_II_c_0_x_leg3 r_II_c_0_y_leg3 r_II_c_0_z_leg3 real
syms r_II_c_0_x_leg4 r_II_c_0_y_leg4 r_II_c_0_z_leg4 real
% syms T_I_B_0_1_1 T_I_B_0_1_2 T_I_B_0_1_3 real
% syms T_I_B_0_2_1 T_I_B_0_2_2 T_I_B_0_2_3 real
% syms T_I_B_0_3_1 T_I_B_0_3_2 T_I_B_0_3_3 real

% T_I_B_0 = [T_I_B_0_1_1 T_I_B_0_1_2 T_I_B_0_1_3
%            T_I_B_0_2_1 T_I_B_0_2_2 T_I_B_0_2_3 
%            T_I_B_0_3_1 T_I_B_0_3_2 T_I_B_0_3_3];
r_II_B_0 = [r_II_B_0_x;r_II_B_0_y;r_II_B_0_z];

Theta1_0 = [Theta1_leg1, Theta1_leg2, Theta1_leg3, Theta1_leg4];
Theta2_0 = [Theta2_leg1, Theta2_leg2, Theta2_leg3, Theta2_leg4];
Theta3_0 = [Theta3_leg1, Theta3_leg2, Theta3_leg3, Theta3_leg4];

gamma = [Theta1_0(1);Theta2_0(1);Theta3_0(1);Theta1_0(2);Theta2_0(2);Theta3_0(2);...
         Theta1_0(3);Theta2_0(3);Theta3_0(3);Theta1_0(4);Theta2_0(4);Theta3_0(4)];

[r_BB_c_leg1, r_BB_c_leg2, r_BB_c_leg3, r_BB_c_leg4]= CPos_wrt_B(Theta1_0, Theta2_0, Theta3_0);

r_II_c_0_leg1 = [r_II_c_0_x_leg1; r_II_c_0_y_leg1; r_II_c_0_z_leg1];
r_II_c_0_leg2 = [r_II_c_0_x_leg2; r_II_c_0_y_leg2; r_II_c_0_z_leg2];
r_II_c_0_leg3 = [r_II_c_0_x_leg3; r_II_c_0_y_leg3; r_II_c_0_z_leg3];
r_II_c_0_leg4 = [r_II_c_0_x_leg4; r_II_c_0_y_leg4; r_II_c_0_z_leg4];

r_II_B_d = [r_II_B_d_x;r_II_B_d_y;r_II_B_d_z];

%% Orientation Solver
syms T_I_B_d_1_1 T_I_B_d_1_2 T_I_B_d_1_3 real
syms T_I_B_d_2_1 T_I_B_d_2_2 T_I_B_d_2_3 real
syms T_I_B_d_3_1 T_I_B_d_3_2 T_I_B_d_3_3 real

T_I_B_d = [T_I_B_d_1_1 T_I_B_d_1_2 T_I_B_d_1_3
           T_I_B_d_2_1 T_I_B_d_2_2 T_I_B_d_2_3
           T_I_B_d_3_1 T_I_B_d_3_2 T_I_B_d_3_3];

r_I_a = r_II_c_0_leg1 - r_II_c_0_leg2;
r_I_b = r_II_c_0_leg1 - r_II_c_0_leg3;

% NOTE, leg1, leg2, and leg3, are not necessarily the three legs on the ground,
% hence the subs coming after
r_B_a = r_BB_c_leg1 - r_BB_c_leg2;
r_B_b = r_BB_c_leg1 - r_BB_c_leg3;

cross_1 = simplify(cross(r_I_a,r_I_b),'Steps',10);

cross_2 = simplify(cross(r_B_a, r_B_b), 'Steps',10);

x_1 = simplify(r_I_a/sqrt(r_I_a(1)^2 + r_I_a(2)^2 + r_I_a(1)^2), 'Steps',10);
y_1 = simplify(cross_1/sqrt(cross_1(1)^2 + cross_1(2)^2 + cross_1(3)^2), 'Steps',10);

x_2 = simplify(r_B_a/sqrt(r_B_a(1)^2 + r_B_a(2)^2 + r_B_a(1)^2), 'Steps',10);
y_2 = cross_2/sqrt(cross_2(1)^2 + cross_2(2)^2 + cross_2(3)^2);

z_1 = cross(x_1,y_1);
z_2 = cross(x_2,y_2);

T_I_A = [x_1, y_1, z_1];
T_B_A = [x_2, y_2, z_2];

T_I_B = T_I_A * T_B_A';

TX_1 = T_I_B(:,1) - T_I_B_d(:,1);
TX_2 = T_I_B(:,2) - T_I_B_d(:,2);
TX_3 = T_I_B(:,3) - T_I_B_d(:,3);

FX_leg1 =  r_II_c_0_leg1 - T_I_B_d*r_BB_c_leg1 - r_II_B_d;
FX_leg2 =  r_II_c_0_leg2 - T_I_B_d*r_BB_c_leg2 - r_II_B_d;
FX_leg3 =  r_II_c_0_leg3 - T_I_B_d*r_BB_c_leg3 - r_II_B_d;
FX_leg4 =  r_II_c_0_leg4 - T_I_B_d*r_BB_c_leg4 - r_II_B_d;

G = [FX_leg1;FX_leg2;FX_leg3;FX_leg4]%[TX_1;TX_2;TX_3;FX_leg1;FX_leg2;FX_leg3;FX_leg4];
dGdX = jacobian(G,gamma)

% save("G.mat","G")
% save("dGdX.mat","dGdX")