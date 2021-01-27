clear
clc

syms Theta1_FR Theta2_FR Theta3_FR real
syms Theta1_FL Theta2_FL Theta3_FL real
syms Theta1_BR Theta2_BR Theta3_BR real
syms Theta1_BL Theta2_BL Theta3_BL real

RobotConstants;
m1 = m1_FR;
m2 = m2_FR;
m3 = m3_FR;

% Orientation of FR leg
T_B_1_FR = rotz(Theta1_FR);
T_1_2_FR = T_B_1_FR * rotx(Theta2_FR);
T_2_3_FR = T_1_2_FR * rotx(Theta3_FR);

% Orientation of FR leg
T_B_1_FL = rotz(Theta1_FL);
T_1_2_FL = T_B_1_FL * rotx(Theta2_FL);
T_2_3_FL = T_1_2_FL * rotx(Theta3_FL);

% Orientation of BR leg
T_B_1_BR = rotz(Theta1_BR);
T_1_2_BR = T_B_1_BR * rotx(Theta2_BR);
T_2_3_BR = T_1_2_BR * rotx(Theta3_BR);

% Orientation of BL leg
T_B_1_BL = rotz(Theta1_BL);
T_1_2_BL = T_B_1_BL * rotx(Theta2_BL);
T_2_3_BL = T_1_2_BL * rotx(Theta3_BL);

% Link 1 CM
rcm_B_1_FR = r_BB_1_FR + T_B_1_FR*rcm_1_1_FR;
rcm_B_1_FL = r_BB_1_FL + T_B_1_FL*rcm_1_1_FL;
rcm_B_1_BR = r_BB_1_BR + T_B_1_BR*rcm_1_1_BR;
rcm_B_1_BL = r_BB_1_BL + T_B_1_BL*rcm_1_1_BL;

% Link 2 CM
rcm_B_2_FR = r_BB_1_FR + T_B_1_FR*r_11_2_FR + T_B_1_FR*T_1_2_FR*rcm_2_2_FR;
rcm_B_2_FL = r_BB_1_FL + T_B_1_FL*r_11_2_FL + T_B_1_FL*T_1_2_FL*rcm_2_2_FL;
rcm_B_2_BR = r_BB_1_BR + T_B_1_BR*r_11_2_BR + T_B_1_BR*T_1_2_BR*rcm_2_2_BR;
rcm_B_2_BL = r_BB_1_BL + T_B_1_BL*r_11_2_BL + T_B_1_BL*T_1_2_BL*rcm_2_2_BL;

% Link 3 CM
rcm_B_3_FR = r_BB_1_FR + T_B_1_FR*r_11_2_FR + T_B_1_FR*T_1_2_FR*r_22_3_FR + T_B_1_FR*T_1_2_FR*T_2_3_FR*rcm_3_3_FR;
rcm_B_3_FL = r_BB_1_FL + T_B_1_FL*r_11_2_FL + T_B_1_FL*T_1_2_FL*r_22_3_FL + T_B_1_FL*T_1_2_FL*T_2_3_FL*rcm_3_3_FL;
rcm_B_3_BR = r_BB_1_BR + T_B_1_BR*r_11_2_BR + T_B_1_BR*T_1_2_BR*r_22_3_BR + T_B_1_BR*T_1_2_BR*T_2_3_BR*rcm_3_3_BR;
rcm_B_3_BL = r_BB_1_BL + T_B_1_BL*r_11_2_BL + T_B_1_BL*T_1_2_BL*r_22_3_BL + T_B_1_BL*T_1_2_BL*T_2_3_BL*rcm_3_3_BL;

% CM
r_B_sys_cm = simplify((mB*rcm_B_B + m1*(rcm_B_1_FR+rcm_B_1_FL+rcm_B_1_BR+rcm_B_1_BL)...
                                                     + m2*(rcm_B_2_FR+rcm_B_2_FL+rcm_B_2_BR+rcm_B_2_BL)...
                                                     + m3*(rcm_B_3_FR+rcm_B_3_FL+rcm_B_3_BR+rcm_B_3_BL))...
                                                     /(mB + 4*m1 + 4*m2 + 4*m3))