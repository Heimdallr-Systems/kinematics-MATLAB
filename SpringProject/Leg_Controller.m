function [Theta1, Theta2, Theta2_2, Theta3, Theta3_2] = Leg_Controller(r_II_c_d, T_I_B, r_II_B, is_left_leg)
% This function operates similar to IK_Solver_Legs_Inertial, but instead is
% intended to use the current T_I_B and r_II_B with a desired r_BB_c_d.
% This should allow the leg desired to be moved act independent to the
% position/tilt controller.
% Works for one leg only!!!

% Constants
RobotConstants;
L2 = norm(r_22_3_BL);
L3 = norm(r_33_c_BL);

if is_left_leg == 1 % left leg
    r_II_c_BL = r_II_c_d;
    
    r_BB_c_BL = T_I_B\(r_II_c_BL - r_II_B);
    
    r_B1_c_BL = r_BB_c_BL - r_BB_1_BL;
    
    Theta1_BL = atan2(r_B1_c_BL(2),r_B1_c_BL(1))-pi/2;
    
    Theta1_BL = angle(exp(1j*Theta1_BL));
    
    r_11_c_BL = rotz(Theta1_BL)\r_B1_c_BL;
    
    r_BL = r_11_c_BL(2)-r_11_2_BL(2);
    
    s_BL = r_11_c_BL(3)-r_11_2_BL(3);
    
    D_BL = (r_BL^2 + s_BL^2 - L2^2 - L3^2)/(2*L2*L3);
    
    Theta3_BL = atan2(sqrt(1-D_BL^2),D_BL);
    Theta3_BL_2 = atan2(-sqrt(1-D_BL^2),D_BL);
    
    Theta2_BL = atan2(s_BL,r_BL) - atan2(L3*sin(Theta3_BL),L2 + L3*cos(Theta3_BL));
    Theta2_BL_2 = atan2(s_BL,r_BL) - atan2(L3*sin(Theta3_BL_2),L2 + L3*cos(Theta3_BL_2));
    if D_BL > 1
        D_BL = 1;
        disp('D was rounded to 1.')
    end
    
    Theta1 = Theta1_BL;
    Theta2 = Theta2_BL;
    Theta2_2 = Theta2_BL_2;
    Theta3 = Theta3_BL;
    Theta3_2 = Theta3_BL_2;
else % right leg
    r_II_c_R = r_II_c_d;
    
    r_BB_c_R = T_I_B\(r_II_c_R - r_II_B);
    
    r_B1_c_R = r_BB_c_R - r_BB_1_FR;
    
    Theta1 = atan2(r_B1_c_R(2),r_B1_c_R(1))+pi/2;
    
    Theta1 = angle(exp(1j*Theta1));
    
    r_1prime1_c_R  = rotz(pi)\(rotz(Theta1)\r_B1_c_R);
    
    r_1prime1_2_R = rotz(pi)\r_11_2_FR;
    
    r_R = r_1prime1_c_R(2)-r_1prime1_2_R(2);
    
    s_R = r_1prime1_c_R(3)-r_1prime1_2_R(3);
    
    D_R = (r_R^2 + s_R^2 - L2^2 - L3^2)/(2*L2*L3);
    
    Theta3_R_Temp = atan2(sqrt(1-D_R^2),D_R);
    Theta3_R_2_Temp = atan2(-sqrt(1-D_R^2),D_R);
    
    Theta3 = -atan2(sqrt(1-D_R^2),D_R);
    Theta3_2 = -atan2(-sqrt(1-D_R^2),D_R);
    
    Theta2 = -(atan2(s_R,r_R) - atan2(L3*sin(Theta3_R_Temp),L2 + L3*cos(Theta3_R_Temp)));
    Theta2_2 = -(atan2(s_R,r_R) - atan2(L3*sin(Theta3_R_2_Temp),L2 + L3*cos(Theta3_R_2_Temp)));
end
end