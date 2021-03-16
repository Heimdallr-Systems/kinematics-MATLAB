function [Theta1, Theta2, Theta3, r_II_B_d] = Body_Pose_Controller(r_II_c, T_I_B, r_II_B_d, r_II_B_0, legs_on_gnd)
% This function is used to solve for IK solutions of the robot's legs when
% measurements in terms of the inertial frame are known. Use this function
% when solving for tilt under the flat plane assumption.
% r_II_c = [r_II_c_FR,r_II_c_FL, r_II_c_BR, r_II_c_BL];
% r_II_c_?? = [x_II_c; y_II_c; z_II_c]
% BodyRot = zyx rotation of body
% r_II_B_d = vector from inertial to body
% Returns: Theta1 of each leg, both solutions for theta2 of each leg, both
% solutions for theta3 of each leg

% Constants, known offsets
constants = RobotConstants();
% known lengths of last two links
L2 = norm(constants.r_22_3_BL);
L3 = norm(constants.r_33_c_BL);

loop_toggle = false;

%initialize
Theta1 = zeros(4,1);
Theta1_2 = zeros(4,1);
Theta2 = zeros(4,1);
Theta2_2 = zeros(4,1);
Theta2_3 = zeros(4,1);
Theta2_4 = zeros(4,1);
Theta3 = zeros(4,1);
Theta3_2 = zeros(4,1);
Theta3_3 = zeros(4,1);
Theta3_4 = zeros(4,1);

% Reference for what the elements are for each variable
% Theta1(:,1) = [Theta1_FR, Theta1_FL, Theta1_BR, Theta1_BL]
% Theta1_2(:,1) = [Theta1_FR_2, Theta1_FL_2, Theta1_BR_2, Theta1_BL_2]
% Theta2(:,1) = [Theta2_FR, Theta2_FL, Theta2_BR, Theta2_BL]
% Theta2_2(:,1) =  [Theta2_FR_2, Theta2_FL_2, Theta2_BR_2, Theta2_BL_2]
% Theta2_3(:,1) = [Theta2_FR_3, Theta2_FL_3, Theta2_BR_3, Theta2_BL_3]
% Theta2_4(:,1) = [Theta2_FR_4, Theta2_FL_4, Theta2_BR_4, Theta2_BL_4]
% Theta3(:,1) = [Theta3_FR, Theta3_FL, Theta3_BR, Theta3_BL]
% Theta3_2(:,1) =  [Theta3_FR_2, Theta3_FL_2, Theta3_BR_2, Theta3_BL_2]
% Theta3_3(:,1) = [Theta3_FR_3, Theta3_FL_3, Theta3_BR_3, Theta3_BL_3]
% Theta3_4(:,1) = [Theta3_FR_4, Theta3_FL_4, Theta3_BR_4, Theta3_BL_4]

travel_dir = (r_II_B_d - r_II_B_0)./norm((r_II_B_d - r_II_B_0));
ii = uint16(0);
while (loop_toggle == false)
    ii = ii+1;
    if ii == 1000
        error('Limit Reached');
    end
    %% FR LEG
    loop_toggle = true;
    if legs_on_gnd(1) == 1
        r_II_c_FR = r_II_c(:,1);
        r_BB_c_FR = T_I_B\(r_II_c_FR - r_II_B_d);
        r_B1_c_FR = r_BB_c_FR - constants.r_BB_1_FR;
        
        Theta1(1,1) = atan2(r_B1_c_FR(2),r_B1_c_FR(1))+pi/2;
        Theta1(1,1) = angle(exp(1j*Theta1(1,1)));
        
        Theta1_2(1,1) = Theta1(1,1) + pi;
        
        r_1prime1_c_FR = rotz(pi)\(rotz(Theta1(1,1))\r_B1_c_FR);
        r_1prime1_c_FR_2 = rotz(pi)\(rotz(Theta1_2(1,1))\r_B1_c_FR);
        
        r_1prime1_2_FR = rotz(pi)\constants.r_11_2_FR;
        
        r_FR = r_1prime1_c_FR(2)-r_1prime1_2_FR(2);
        r_FR_2 = r_1prime1_c_FR_2(2)-r_1prime1_2_FR(2);
        
        s_FR = r_1prime1_c_FR(3)-r_1prime1_2_FR(3);
        s_FR_2 = r_1prime1_c_FR_2(3)-r_1prime1_2_FR(3);
        
        D_FR = (r_FR^2 + s_FR^2 - L2^2 - L3^2)/(2*L2*L3);
        D_FR_2 = (r_FR_2^2 + s_FR_2^2 - L2^2 - L3^2)/(2*L2*L3);
        
        if D_FR >0.999
            loop_toggle = false;
            r_II_B_d = r_II_B_d - travel_dir.*0.001;
%             r_II_B_d(3) = r_II_B_0(3);
        else
            Theta3_FR_Temp = atan2(sqrt(1-D_FR^2),D_FR);
            Theta3_FR_2_Temp = atan2(-sqrt(1-D_FR^2),D_FR);
            
            Theta3(1,1) = -atan2(sqrt(1-D_FR^2),D_FR);
            Theta3_2(1,1) = -atan2(-sqrt(1-D_FR^2),D_FR);
            
            Theta2(1,1) = -(atan2(s_FR,r_FR) - atan2(L3*sin(Theta3_FR_Temp),L2 + L3*cos(Theta3_FR_Temp)));
            Theta2_2(1,1) = -(atan2(s_FR,r_FR) - atan2(L3*sin(Theta3_FR_2_Temp),L2 + L3*cos(Theta3_FR_2_Temp)));
        end
        
        % see if second solution possible
        if D_FR_2 > 0.999
            Theta1_2(1,1) = 0;
            
            Theta3_3(1,1) = 0;
            Theta3_4(1,1) = 0;
            
            Theta2_3(1,1) = 0;
            Theta2_4(1,1) = 0;
        else
            Theta3_FR_Temp_2 = atan2(sqrt(1-D_FR_2^2),D_FR_2);
            Theta3_FR_2_Temp_2 = atan2(-sqrt(1-D_FR_2^2),D_FR_2);
            
            Theta3_3(1,1) = -atan2(sqrt(1-D_FR_2^2),D_FR_2);
            Theta3_4(1,1) = -atan2(-sqrt(1-D_FR_2^2),D_FR_2);
            
            Theta2_3(1,1) = -(atan2(s_FR_2,r_FR_2) - atan2(L3*sin(Theta3_FR_Temp_2),L2 + L3*cos(Theta3_FR_Temp_2)));
            Theta2_4(1,1) = -(atan2(s_FR_2,r_FR_2) - atan2(L3*sin(Theta3_FR_2_Temp_2),L2 + L3*cos(Theta3_FR_2_Temp_2)));
        end
    else
        Theta1(1,1) = 0;
        Theta1_2(1,1) = 0;
        Theta2(1,1) = 0;
        Theta2_2(1,1) = 0;
        Theta2_3(1,1) = 0;
        Theta2_4(1,1) = 0;
        Theta3(1,1) = 0;
        Theta3_2(1,1) = 0;
        Theta3_3(1,1) = 0;
        Theta3_4(1,1) = 0;
    end
    %% FL LEG
    if legs_on_gnd(2) == 1
        r_II_c_FL = r_II_c(:,2);
        r_BB_c_FL = T_I_B\(r_II_c_FL - r_II_B_d);
        r_B1_c_FL = r_BB_c_FL - constants.r_BB_1_FL;
        
        Theta1(2,1) = atan2(r_B1_c_FL(2),r_B1_c_FL(1))-pi/2;
        Theta1(2,1) = angle(exp(1j*Theta1(2,1)));
        
        Theta1_2(2,1) = Theta1(2,1) + pi;
        
        r_11_c_FL = rotz(Theta1(2,1))\r_B1_c_FL;
        
        r_11_c_FL_2 = rotz(Theta1_2(2,1))\r_B1_c_FL;
        
        r_FL = r_11_c_FL(2)-constants.r_11_2_FL(2);
        
        r_FL_2 = r_11_c_FL_2(2)-constants.r_11_2_FL(2);
        
        s_FL = r_11_c_FL(3)-constants.r_11_2_FL(3);
        
        s_FL_2 = r_11_c_FL_2(3)-constants.r_11_2_FL(3);
        
        D_FL = (r_FL^2 + s_FL^2 - L2^2 - L3^2)/(2*L2*L3);
        
        D_FL_2 = (r_FL_2^2 + s_FL_2^2 - L2^2 - L3^2)/(2*L2*L3);
        
        if D_FL_2 > 0.999
            Theta1_2(2,1) = 0;
            
            Theta3_3(2,1) = 0;
            Theta3_4(2,1) = 0;
            
            Theta2_3(2,1) = 0;
            Theta2_4(2,1) = 0;
        else
            Theta3_3(2,1) = atan2(sqrt(1-D_FL_2^2),D_FL_2);
            Theta3_4(2,1) = atan2(-sqrt(1-D_FL_2^2),D_FL_2);
            
            Theta2_3(2,1) = atan2(s_FL_2,r_FL_2) - atan2(L3*sin(Theta3_3(2,1)),L2 + L3*cos(Theta3_3(2,1)));
            Theta2_4(2,1) = atan2(s_FL_2,r_FL_2) - atan2(L3*sin(Theta3_4(2,1)),L2 + L3*cos(Theta3_4(2,1)));
        end
        
        if D_FL > 0.999
            loop_toggle = false;
            r_II_B_d = r_II_B_d - travel_dir.*0.001;
%             r_II_B_d(3) = r_II_B_0(3);
        else
            Theta3(2,1) = atan2(sqrt(1-D_FL^2),D_FL);
            Theta3_2(2,1) = atan2(-sqrt(1-D_FL^2),D_FL);
            
            Theta2(2,1) = atan2(s_FL,r_FL) - atan2(L3*sin(Theta3(2,1)),L2 + L3*cos(Theta3(2,1)));
            Theta2_2(2,1) = atan2(s_FL,r_FL) - atan2(L3*sin(Theta3_2(2,1)),L2 + L3*cos(Theta3_2(2,1)));
        end
        
    else
        Theta1(2,1) = 0;
        Theta1_2(2,1) = 0;
        Theta2(2,1) = 0;
        Theta2_2(2,1) = 0;
        Theta2_3(2,1) = 0;
        Theta2_4(2,1) = 0;
        Theta3(2,1) = 0;
        Theta3_2(2,1) = 0;
        Theta3_3(2,1) = 0;
        Theta3_4(2,1) = 0;
    end
    %% BR LEG
    if legs_on_gnd(3) == 1
        r_II_c_BR = r_II_c(:,3);
        r_BB_c_BR = T_I_B\(r_II_c_BR - r_II_B_d);
        r_B1_c_BR = r_BB_c_BR - constants.r_BB_1_BR;
        
        Theta1(3,1) = atan2(r_B1_c_BR(2),r_B1_c_BR(1))+pi/2;
        Theta1(3,1) = angle(exp(1j*Theta1(3,1)));
        
        Theta1_2(3,1) = Theta1(3,1) + pi;
        
        r_1prime1_c_BR  = rotz(pi)\(rotz(Theta1(3,1))\r_B1_c_BR);
        
        r_1prime1_c_BR_2  = rotz(pi)\(rotz(Theta1_2(3,1))\r_B1_c_BR);
        
        r_1prime1_2_BR = rotz(pi)\constants.r_11_2_BR;
        
        r_BR = r_1prime1_c_BR(2)-r_1prime1_2_BR(2);
        
        r_BR_2 = r_1prime1_c_BR_2(2)-r_1prime1_2_BR(2);
        
        s_BR = r_1prime1_c_BR(3)-r_1prime1_2_BR(3);
        
        s_BR_2 = r_1prime1_c_BR_2(3)-r_1prime1_2_BR(3);
        
        D_BR = (r_BR^2 + s_BR^2 - L2^2 - L3^2)/(2*L2*L3);
        
        D_BR_2 = (r_BR_2^2 + s_BR_2^2 - L2^2 - L3^2)/(2*L2*L3);
        
        if D_BR_2 > 0.999
            Theta1_2(3,1) = 0;
            
            Theta3_3(3,1) = 0;
            Theta3_4(3,1) = 0;
            
            Theta2_3(3,1) = 0;
            Theta2_4(3,1) = 0;
        else
            Theta3_BR_Temp_2 = atan2(sqrt(1-D_BR_2^2),D_BR_2);
            Theta3_BR_2_Temp_2 = atan2(-sqrt(1-D_BR_2^2),D_BR_2);
            
            Theta3_3(3,1) = -atan2(sqrt(1-D_BR_2^2),D_BR_2);
            Theta3_4(3,1) = -atan2(-sqrt(1-D_BR_2^2),D_BR_2);
            
            Theta2_3(3,1) = -(atan2(s_BR_2,r_BR_2) - atan2(L3*sin(Theta3_BR_Temp_2),L2 + L3*cos(Theta3_BR_Temp_2)));
            Theta2_4(3,1) = -(atan2(s_BR_2,r_BR_2) - atan2(L3*sin(Theta3_BR_2_Temp_2),L2 + L3*cos(Theta3_BR_2_Temp_2)));
            
        end
        
        if D_BR > 0.999
            loop_toggle = false;
            r_II_B_d = r_II_B_d - travel_dir.*0.001;
%             r_II_B_d(3) = r_II_B_0(3);
        else
            Theta3_BR_Temp = atan2(sqrt(1-D_BR^2),D_BR);
            Theta3_BR_2_Temp = atan2(-sqrt(1-D_BR^2),D_BR);
            
            Theta3(3,1) = -atan2(sqrt(1-D_BR^2),D_BR);
            Theta3_2(3,1) = -atan2(-sqrt(1-D_BR^2),D_BR);
            
            Theta2(3,1) = -(atan2(s_BR,r_BR) - atan2(L3*sin(Theta3_BR_Temp),L2 + L3*cos(Theta3_BR_Temp)));
            Theta2_2(3,1) = -(atan2(s_BR,r_BR) - atan2(L3*sin(Theta3_BR_2_Temp),L2 + L3*cos(Theta3_BR_2_Temp)));
        end
        
    else
        Theta1(3,1) = 0;
        Theta1_2(3,1) = 0;
        Theta2(3,1) = 0;
        Theta2_2(3,1) = 0;
        Theta2_3(3,1) = 0;
        Theta2_4(3,1) = 0;
        Theta3(3,1) = 0;
        Theta3_2(3,1) = 0;
        Theta3_3(3,1) = 0;
        Theta3_4(3,1) = 0;
    end
    %% BL LEG
    if legs_on_gnd(4) == 1
        r_II_c_BL = r_II_c(:,4);
        r_BB_c_BL = T_I_B\(r_II_c_BL - r_II_B_d);
        r_B1_c_BL = r_BB_c_BL - constants.r_BB_1_BL;
        
        Theta1(4,1) = atan2(r_B1_c_BL(2),r_B1_c_BL(1))-pi/2;
        Theta1(4,1) = angle(exp(1j*Theta1(4,1)));
        
        Theta1_2(4,1) = Theta1(4,1) + pi;
        
        r_11_c_BL = rotz(Theta1(4,1))\r_B1_c_BL;
        
        r_11_c_BL_2 = rotz(Theta1_2(4,1))\r_B1_c_BL;
        
        r_BL = r_11_c_BL(2)-constants.r_11_2_BL(2);
        
        r_BL_2 = r_11_c_BL_2(2)-constants.r_11_2_BL(2);
        
        s_BL = r_11_c_BL(3)-constants.r_11_2_BL(3);
        
        s_BL_2 = r_11_c_BL_2(3)-constants.r_11_2_BL(3);
        
        D_BL = (r_BL^2 + s_BL^2 - L2^2 - L3^2)/(2*L2*L3);
        
        D_BL_2 = (r_BL_2^2 + s_BL_2^2 - L2^2 - L3^2)/(2*L2*L3);
        
        if D_BL_2 > 0.999
            Theta1_2(4,1) = 0;
            
            Theta3_3(4,1) = 0;
            Theta3_4(4,1) = 0;
            
            Theta2_3(4,1) = 0;
            Theta2_4(4,1) = 0;
        else
            Theta3_3(4,1) = atan2(sqrt(1-D_BL_2^2),D_BL_2);
            Theta3_4(4,1) = atan2(-sqrt(1-D_BL_2^2),D_BL_2);
            
            Theta2_3(4,1) = atan2(s_BL_2,r_BL_2) - atan2(L3*sin(Theta3_3(4,1)),L2 + L3*cos(Theta3_3(4,1)));
            Theta2_4(4,1) = atan2(s_BL_2,r_BL_2) - atan2(L3*sin(Theta3_4(4,1)),L2 + L3*cos(Theta3_4(4,1)));
        end
        
        if D_BL > 0.999
            loop_toggle = false;
            r_II_B_d = r_II_B_d - travel_dir.*0.001;
%             r_II_B_d(3) = r_II_B_0(3);
        else
            Theta3(4,1) = atan2(sqrt(1-D_BL^2),D_BL);
            Theta3_2(4,1) = atan2(-sqrt(1-D_BL^2),D_BL);
            
            Theta2(4,1) = atan2(s_BL,r_BL) - atan2(L3*sin(Theta3(4,1)),L2 + L3*cos(Theta3(4,1)));
            Theta2_2(4,1) = atan2(s_BL,r_BL) - atan2(L3*sin(Theta3_2(4,1)),L2 + L3*cos(Theta3_2(4,1)));
        end
        
    else
        Theta1(4,1) = 0;
        Theta1_2(4,1) = 0;
        Theta2(4,1) = 0;
        Theta2_2(4,1) = 0;
        Theta2_3(4,1) = 0;
        Theta2_4(4,1) = 0;
        Theta3(4,1) = 0;
        Theta3_2(4,1) = 0;
        Theta3_3(4,1) = 0;
        Theta3_4(4,1) = 0;
    end
end

T1_cond = false(1,4);
% if theta1 wraps around into robot
T1_cond(1) = (Theta1(1) <= -pi/2) || (Theta1(1) >= pi); % FR
T1_cond(2) = (Theta1(2) <= -pi) || (Theta1(2) >= pi/2); %FL
T1_cond(3) = (Theta1(3) <= -pi) || (Theta1(3) >= pi/2); % BR
T1_cond(4) = (Theta1(4) <= -pi/2) || (Theta1(4) >= pi); % BL

for hh = uint8(1:1:4)
    if T1_cond(hh)
        Theta1(hh,1) = Theta1_2(hh);
        Theta2(hh,1) = Theta2_4(hh);
        Theta3(hh,1) = Theta3_4(hh);
    else
        Theta2(hh,1) = Theta2_2(hh);
        Theta3(hh,1) = Theta3_2(hh);
    end
end

end