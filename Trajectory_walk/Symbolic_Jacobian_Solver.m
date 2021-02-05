clear
clc

syms Theta1_FR Theta1_FL Theta1_BR Theta1_BL
syms Theta2_FR Theta2_FL Theta2_BR Theta2_BL
syms Theta3_FR Theta3_FL Theta3_BR Theta3_BL
syms phi theta psi
syms r_II_B_d_x r_II_B_d_y r_II_B_d_z
syms r_II_B_0_x r_II_B_0_y r_II_B_0_z
syms r_II_c_0_x_FR r_II_c_0_y_FR r_II_c_0_z_FR
syms r_II_c_0_x_FL r_II_c_0_y_FL r_II_c_0_z_FL
syms r_II_c_0_x_BR r_II_c_0_y_BR r_II_c_0_z_BR
syms r_II_c_0_x_BL r_II_c_0_y_BL r_II_c_0_z_BL

T_I_B_0 = rotz(phi)*roty(theta)*rotx(psi);
r_II_B_0 = [r_II_B_0_x;r_II_B_0_y;r_II_B_0_z];

Theta1_0 = [Theta1_FR, Theta1_FL, Theta1_BR, Theta1_BL];
Theta2_0 = [Theta2_FR, Theta2_FL, Theta2_BR, Theta2_BL];
Theta3_0 = [Theta3_FR, Theta3_FL, Theta3_BR, Theta3_BL];

X_FR = [Theta1_0(1);Theta2_0(1);Theta3_0(1)];
X_FL = [Theta1_0(2);Theta2_0(2);Theta3_0(2)];
X_BR = [Theta1_0(3);Theta2_0(3);Theta3_0(3)];
X_BL = [Theta1_0(4);Theta2_0(4);Theta3_0(4)];

[r_BB_c_FR, r_BB_c_FL, r_BB_c_BR, r_BB_c_BL]= CPos_wrt_B(Theta1_0, Theta2_0, Theta3_0);

r_II_c_0_FR = [r_II_c_0_x_FR; r_II_c_0_y_FR; r_II_c_0_z_FR];
r_II_c_0_FL = [r_II_c_0_x_FL; r_II_c_0_y_FL; r_II_c_0_z_FL];
r_II_c_0_BR = [r_II_c_0_x_BR; r_II_c_0_y_BR; r_II_c_0_z_BR];
r_II_c_0_BL = [r_II_c_0_x_BL; r_II_c_0_y_BL; r_II_c_0_z_BL];

r_II_B_d = [r_II_B_d_x;r_II_B_d_y;r_II_B_d_z];

FX_FR =   simplify(r_II_c_0_FR - T_I_B_0*r_BB_c_FR - r_II_B_d)
FX_FL =   simplify(r_II_c_0_FL - T_I_B_0*r_BB_c_FL - r_II_B_d)
FX_BR =   simplify(r_II_c_0_BR - T_I_B_0*r_BB_c_BR - r_II_B_d)
FX_BL =   simplify(r_II_c_0_BL - T_I_B_0*r_BB_c_BL - r_II_B_d)
      
dFdX_FR = simplify(jacobian(FX_FR,X_FR))
dFdX_FL = simplify(jacobian(FX_FL,X_FL))
dFdX_BR = simplify(jacobian(FX_BR,X_BR))
dFdX_BL = simplify(jacobian(FX_BL,X_BL))
