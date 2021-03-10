function [Theta1,Theta2,Theta3] = Leg_Controller_B(r_BB_c, leg_index)
% This function is used to "reset" the legs to some r_BB_c position
% Works for one leg only!!!

% Constants, known offsets
constants = RobotConstants();
% known lengths of last two links
L2 = norm(constants.r_22_3_BL);
L3 = norm(constants.r_33_c_BL);

switch leg_index
    case 1
        % Vector from Base to Contact
        r_BB_c_FR = r_BB_c;
        % Vector from Base to Contact minus first joint offset from base.
        % This is used to eliminate the offset so the frame can be centered on
        % the first joint, allowing for arctangent function to be used to solve
        % for theta1.
        r_B1_c_FR = r_BB_c_FR - constants.r_BB_1_FR;
        % Solve for Theta1
        Theta1_FR = atan2(r_B1_c_FR(2),r_B1_c_FR(1))+pi/2;
        % wrap theta1 between +-pi
        Theta1_FR = angle(exp(1j*Theta1_FR));
        % Solve for Theta3
        % Set up a positive frame centered on joint two, aligned with frame 1 in order to apply standard
        % two-link manipulator solution (refer to textbook)
        r_1prime1_c_FR = rotz(pi)\(rotz(Theta1_FR)\r_B1_c_FR);
        % vectors from joint 1 to joint 2
        % transform vectors to positive, no offset frame to allow for standard two link
        % problem
        r_1prime1_2_FR = rotz(pi)\constants.r_11_2_FR;
        % solves for y-axis component
        r_FR = r_1prime1_c_FR(2)-r_1prime1_2_FR(2);
        % solves for z-axis component
        s_FR = r_1prime1_c_FR(3)-r_1prime1_2_FR(3);
        % cos(theta3)
        D_FR = (r_FR^2 + s_FR^2 - L2^2 - L3^2)/(2*L2*L3);
        % Theta 3 values measured with respect to the frame (frame 1 prime) used to solve standard
        % 2-link problem frame . These will be used to solve for the theta 2 values
        Theta3_FR_Temp = atan2(sqrt(1-D_FR^2),D_FR);
        Theta3_FR_2_Temp = atan2(-sqrt(1-D_FR^2),D_FR);
        % Theta 3 values measured in defined way (from zeroed position)
        Theta3_FR = -atan2(sqrt(1-D_FR^2),D_FR);
        Theta3_FR_2 = -atan2(-sqrt(1-D_FR^2),D_FR);
        % Solve for Theta2
        Theta2_FR = -(atan2(s_FR,r_FR) - atan2(L3*sin(Theta3_FR_Temp),L2 + L3*cos(Theta3_FR_Temp)));
        Theta2_FR_2 = -(atan2(s_FR,r_FR) - atan2(L3*sin(Theta3_FR_2_Temp),L2 + L3*cos(Theta3_FR_2_Temp)));
        
        Theta1 = Theta1_FR;
        Theta2 = Theta2_FR_2;
        Theta3 = Theta3_FR_2;
    case 2
        r_BB_c_FL = r_BB_c;
        r_B1_c_FL = r_BB_c_FL - constants.r_BB_1_FL;
        Theta1_FL = atan2(r_B1_c_FL(2),r_B1_c_FL(1))-pi/2;
        Theta1_FL = angle(exp(1j*Theta1_FL));
        r_11_c_FL = rotz(Theta1_FL)\r_B1_c_FL;
        r_FL = r_11_c_FL(2)-constants.r_11_2_FL(2);
        s_FL = r_11_c_FL(3)-constants.r_11_2_FL(3);
        D_FL = (r_FL^2 + s_FL^2 - L2^2 - L3^2)/(2*L2*L3);
        Theta3_FL = atan2(sqrt(1-D_FL^2),D_FL);
        Theta3_FL_2 = atan2(-sqrt(1-D_FL^2),D_FL);
        Theta2_FL = atan2(s_FL,r_FL) - atan2(L3*sin(Theta3_FL),L2 + L3*cos(Theta3_FL));
        Theta2_FL_2 = atan2(s_FL,r_FL) - atan2(L3*sin(Theta3_FL_2),L2 + L3*cos(Theta3_FL_2));
        
        Theta1 = Theta1_FL;
        Theta2 = Theta2_FL_2;
        Theta3 = Theta3_FL_2;
    case 3
        r_BB_c_BR = r_BB_c;
        r_B1_c_BR = r_BB_c_BR - constants.r_BB_1_BR;
        Theta1_BR = atan2(r_B1_c_BR(2),r_B1_c_BR(1))+pi/2;
        Theta1_BR = angle(exp(1j*Theta1_BR));
        r_1prime1_c_BR  = rotz(pi)\(rotz(Theta1_BR)\r_B1_c_BR);
        r_1prime1_2_BR = rotz(pi)\constants.r_11_2_BR;
        r_BR = r_1prime1_c_BR(2)-r_1prime1_2_BR(2);
        s_BR = r_1prime1_c_BR(3)-r_1prime1_2_BR(3);
        D_BR = (r_BR^2 + s_BR^2 - L2^2 - L3^2)/(2*L2*L3);
        Theta3_BR_Temp = atan2(sqrt(1-D_BR^2),D_BR);
        Theta3_BR_2_Temp = atan2(-sqrt(1-D_BR^2),D_BR);
        Theta3_BR = -atan2(sqrt(1-D_BR^2),D_BR);
        Theta3_BR_2 = -atan2(-sqrt(1-D_BR^2),D_BR);
        Theta2_BR = -(atan2(s_BR,r_BR) - atan2(L3*sin(Theta3_BR_Temp),L2 + L3*cos(Theta3_BR_Temp)));
        Theta2_BR_2 = -(atan2(s_BR,r_BR) - atan2(L3*sin(Theta3_BR_2_Temp),L2 + L3*cos(Theta3_BR_2_Temp)));
        
        Theta1 = Theta1_BR;
        Theta2 = Theta2_BR_2;
        Theta3 = Theta3_BR_2;
        % If not 1, 2, or 3, it must be 4
    otherwise
        r_BB_c_BL = r_BB_c;
        r_B1_c_BL = r_BB_c_BL - constants.r_BB_1_BL;
        Theta1_BL = atan2(r_B1_c_BL(2),r_B1_c_BL(1))-pi/2;
        Theta1_BL = angle(exp(1j*Theta1_BL));
        r_11_c_BL = rotz(Theta1_BL)\r_B1_c_BL;
        r_BL = r_11_c_BL(2)-constants.r_11_2_BL(2);
        s_BL = r_11_c_BL(3)-constants.r_11_2_BL(3);
        D_BL = (r_BL^2 + s_BL^2 - L2^2 - L3^2)/(2*L2*L3);
        Theta3_BL = atan2(sqrt(1-D_BL^2),D_BL);
        Theta3_BL_2 = atan2(-sqrt(1-D_BL^2),D_BL);
        Theta2_BL = atan2(s_BL,r_BL) - atan2(L3*sin(Theta3_BL),L2 + L3*cos(Theta3_BL));
        Theta2_BL_2 = atan2(s_BL,r_BL) - atan2(L3*sin(Theta3_BL_2),L2 + L3*cos(Theta3_BL_2));
        
        Theta1 = Theta1_BL;
        Theta2 = Theta2_BL_2;
        Theta3 = Theta3_BL_2;
end

end