function [Theta1,Theta2,Theta3,r_II_c_d] = Leg_Controller(r_II_c_d,r_II_c_0, T_I_B, r_II_B, leg_index)
% This function operates similar to IK_Solver_Legs_Inertial, but instead is
% intended to use the current T_I_B and r_II_B with a desired r_BB_c_d.
% This should allow the leg desired to be moved act independent to the
% position/tilt controller.
% Works for one leg only!!!

% Constants, known offsets
constants = RobotConstants();
% known lengths of last two links
L2 = norm(constants.r_22_3_BL);
L3 = norm(constants.r_33_c_BL);

%% Init these so that coder does not throw a hissy fit
% FIXME: This could actually cause problems. Check with Nick
Theta1 = 0;
Theta1_2 = 0;
Theta2_2 = 0;
Theta3_2 = 0;
Theta2_4 = 0;
Theta3_4 = 0;


%%

travel_dir = (r_II_c_d-r_II_c_0)/norm((r_II_c_d-r_II_c_0));
loop_toggle = 0;
ii = 0;
while loop_toggle == 0
    ii = ii+1;
    if ii == 1000
        error('Limit Reached');
    end
    loop_toggle = 1;
    %initialize
    switch leg_index
        %% FR LEG
        case 1
            r_II_c_FR = r_II_c_d;
            r_BB_c_FR = T_I_B\(r_II_c_FR - r_II_B);
            r_B1_c_FR = r_BB_c_FR - constants.r_BB_1_FR;
            
            Theta1 = atan2(r_B1_c_FR(2),r_B1_c_FR(1))+pi/2;
            Theta1 = angle(exp(1j*Theta1));
            
            Theta1_2 = Theta1 + pi;
            
            r_1prime1_c_FR = rotz(pi)\(rotz(Theta1)\r_B1_c_FR);
            r_1prime1_c_FR_2 = rotz(pi)\(rotz(Theta1_2)\r_B1_c_FR);
            
            r_1prime1_2_FR = rotz(pi)\constants.r_11_2_FR;
            
            r_FR = r_1prime1_c_FR(2)-r_1prime1_2_FR(2);
            r_FR_2 = r_1prime1_c_FR_2(2)-r_1prime1_2_FR(2);
            
            s_FR = r_1prime1_c_FR(3)-r_1prime1_2_FR(3);
            s_FR_2 = r_1prime1_c_FR_2(3)-r_1prime1_2_FR(3);
            
            D_FR = (r_FR^2 + s_FR^2 - L2^2 - L3^2)/(2*L2*L3);
            D_FR_2 = (r_FR_2^2 + s_FR_2^2 - L2^2 - L3^2)/(2*L2*L3);
            
            % see if second solution possible
            if D_FR_2 > 0.999
                Theta1_2 = 0;
                
                Theta3_3 = 0;
                Theta3_4 = 0;
                
                Theta2_3 = 0;
                Theta2_4 = 0;
            else
                Theta3_FR_Temp_2 = atan2(sqrt(1-D_FR_2^2),D_FR_2);
                Theta3_FR_2_Temp_2 = atan2(-sqrt(1-D_FR_2^2),D_FR_2);
                
                Theta3_3 = -atan2(sqrt(1-D_FR_2^2),D_FR_2);
                Theta3_4 = -atan2(-sqrt(1-D_FR_2^2),D_FR_2);
                
                Theta2_3 = -(atan2(s_FR_2,r_FR_2) - atan2(L3*sin(Theta3_FR_Temp_2),L2 + L3*cos(Theta3_FR_Temp_2)));
                Theta2_4 = -(atan2(s_FR_2,r_FR_2) - atan2(L3*sin(Theta3_FR_2_Temp_2),L2 + L3*cos(Theta3_FR_2_Temp_2)));
            end
            
            if D_FR > 0.999
                loop_toggle = 0;
                r_II_c_d = r_II_c_d - travel_dir.*0.01;
                r_II_c_d(3) = r_II_c_0(3);
            else
                Theta3_FR_Temp = atan2(sqrt(1-D_FR^2),D_FR);
                Theta3_FR_2_Temp = atan2(-sqrt(1-D_FR^2),D_FR);
                
                Theta3 = -atan2(sqrt(1-D_FR^2),D_FR);
                Theta3_2 = -atan2(-sqrt(1-D_FR^2),D_FR);
                
                Theta2 = -(atan2(s_FR,r_FR) - atan2(L3*sin(Theta3_FR_Temp),L2 + L3*cos(Theta3_FR_Temp)));
                Theta2_2 = -(atan2(s_FR,r_FR) - atan2(L3*sin(Theta3_FR_2_Temp),L2 + L3*cos(Theta3_FR_2_Temp)));
            end
            
            %% FL LEG
        case 2
            r_II_c_FL = r_II_c_d;
            r_BB_c_FL = T_I_B\(r_II_c_FL - r_II_B);
            r_B1_c_FL = r_BB_c_FL - constants.r_BB_1_FL;
            
            Theta1 = atan2(r_B1_c_FL(2),r_B1_c_FL(1))-pi/2;
            Theta1 = angle(exp(1j*Theta1));
            
            Theta1_2 = Theta1 + pi;
            
            r_11_c_FL = rotz(Theta1)\r_B1_c_FL;
            
            r_11_c_FL_2 = rotz(Theta1_2)\r_B1_c_FL;
            
            r_FL = r_11_c_FL(2)-constants.r_11_2_FL(2);
            
            r_FL_2 = r_11_c_FL_2(2)-constants.r_11_2_FL(2);
            
            s_FL = r_11_c_FL(3)-constants.r_11_2_FL(3);
            
            s_FL_2 = r_11_c_FL_2(3)-constants.r_11_2_FL(3);
            
            D_FL = (r_FL^2 + s_FL^2 - L2^2 - L3^2)/(2*L2*L3);
            
            D_FL_2 = (r_FL_2^2 + s_FL_2^2 - L2^2 - L3^2)/(2*L2*L3);
            
            if D_FL_2 > 0.999
                Theta1_2 = 0;
                
                Theta3_3 = 0;
                Theta3_4 = 0;
                
                Theta2_3 = 0;
                Theta2_4 = 0;
            else
                Theta3_3 = atan2(sqrt(1-D_FL_2^2),D_FL_2);
                Theta3_4 = atan2(-sqrt(1-D_FL_2^2),D_FL_2);
                
                Theta2_3 = atan2(s_FL_2,r_FL_2) - atan2(L3*sin(Theta3_3),L2 + L3*cos(Theta3_3));
                Theta2_4 = atan2(s_FL_2,r_FL_2) - atan2(L3*sin(Theta3_4),L2 + L3*cos(Theta3_4));
            end
            
            if D_FL > 0.999
                loop_toggle = 0;
                r_II_c_d = r_II_c_d - travel_dir.*0.01;
                r_II_c_d(3) = r_II_c_0(3);
            else
                Theta3 = atan2(sqrt(1-D_FL^2),D_FL);
                Theta3_2 = atan2(-sqrt(1-D_FL^2),D_FL);
                
                Theta2 = atan2(s_FL,r_FL) - atan2(L3*sin(Theta3),L2 + L3*cos(Theta3));
                Theta2_2 = atan2(s_FL,r_FL) - atan2(L3*sin(Theta3_2),L2 + L3*cos(Theta3_2));
            end
            
            
            %% BR LEG
        case 3
            r_II_c_BR = r_II_c_d;
            r_BB_c_BR = T_I_B\(r_II_c_BR - r_II_B);
            r_B1_c_BR = r_BB_c_BR - constants.r_BB_1_BR;
            
            Theta1 = atan2(r_B1_c_BR(2),r_B1_c_BR(1))+pi/2;
            Theta1 = angle(exp(1j*Theta1));
            
            Theta1_2 = Theta1 + pi;
            
            r_1prime1_c_BR  = rotz(pi)\(rotz(Theta1)\r_B1_c_BR);
            
            r_1prime1_c_BR_2  = rotz(pi)\(rotz(Theta1_2)\r_B1_c_BR);
            
            r_1prime1_2_BR = rotz(pi)\constants.r_11_2_BR;
            
            r_BR = r_1prime1_c_BR(2)-r_1prime1_2_BR(2);
            
            r_BR_2 = r_1prime1_c_BR_2(2)-r_1prime1_2_BR(2);
            
            s_BR = r_1prime1_c_BR(3)-r_1prime1_2_BR(3);
            
            s_BR_2 = r_1prime1_c_BR_2(3)-r_1prime1_2_BR(3);
            
            D_BR = (r_BR^2 + s_BR^2 - L2^2 - L3^2)/(2*L2*L3);
            
            D_BR_2 = (r_BR_2^2 + s_BR_2^2 - L2^2 - L3^2)/(2*L2*L3);
            
            if D_BR_2 > 0.999
                Theta1_2 = 0;
                
                Theta3_3 = 0;
                Theta3_4 = 0;
                
                Theta2_3 = 0;
                Theta2_4 = 0;
            else
                Theta3_BR_Temp_2 = atan2(sqrt(1-D_BR_2^2),D_BR_2);
                Theta3_BR_2_Temp_2 = atan2(-sqrt(1-D_BR_2^2),D_BR_2);
                
                Theta3_3 = -atan2(sqrt(1-D_BR_2^2),D_BR_2);
                Theta3_4 = -atan2(-sqrt(1-D_BR_2^2),D_BR_2);
                
                Theta2_3 = -(atan2(s_BR_2,r_BR_2) - atan2(L3*sin(Theta3_BR_Temp_2),L2 + L3*cos(Theta3_BR_Temp_2)));
                Theta2_4 = -(atan2(s_BR_2,r_BR_2) - atan2(L3*sin(Theta3_BR_2_Temp_2),L2 + L3*cos(Theta3_BR_2_Temp_2)));
            end
            
            if D_BR > 0.999
                loop_toggle = 0;
                r_II_c_d = r_II_c_d - travel_dir.*0.01;
                r_II_c_d(3) = r_II_c_0(3);
            else
                Theta3_BR_Temp = atan2(sqrt(1-D_BR^2),D_BR);
                Theta3_BR_2_Temp = atan2(-sqrt(1-D_BR^2),D_BR);
                
                Theta3 = -atan2(sqrt(1-D_BR^2),D_BR);
                Theta3_2 = -atan2(-sqrt(1-D_BR^2),D_BR);
                
                Theta2 = -(atan2(s_BR,r_BR) - atan2(L3*sin(Theta3_BR_Temp),L2 + L3*cos(Theta3_BR_Temp)));
                Theta2_2 = -(atan2(s_BR,r_BR) - atan2(L3*sin(Theta3_BR_2_Temp),L2 + L3*cos(Theta3_BR_2_Temp)));
            end
            
            
            %% BL LEG
        case 4
            r_II_c_BL = r_II_c_d;
            r_BB_c_BL = T_I_B\(r_II_c_BL - r_II_B);
            r_B1_c_BL = r_BB_c_BL - constants.r_BB_1_BL;
            
            Theta1 = atan2(r_B1_c_BL(2),r_B1_c_BL(1))-pi/2;
            Theta1 = angle(exp(1j*Theta1));
            
            Theta1_2 = Theta1 + pi;
            
            r_11_c_BL = rotz(Theta1)\r_B1_c_BL;
            
            r_11_c_BL_2 = rotz(Theta1_2)\r_B1_c_BL;
            
            r_BL = r_11_c_BL(2)-constants.r_11_2_BL(2);
            
            r_BL_2 = r_11_c_BL_2(2)-constants.r_11_2_BL(2);
            
            s_BL = r_11_c_BL(3)-constants.r_11_2_BL(3);
            
            s_BL_2 = r_11_c_BL_2(3)-constants.r_11_2_BL(3);
            
            D_BL = (r_BL^2 + s_BL^2 - L2^2 - L3^2)/(2*L2*L3);
            
            D_BL_2 = (r_BL_2^2 + s_BL_2^2 - L2^2 - L3^2)/(2*L2*L3);
            
            if D_BL_2 > 0.999
                Theta1_2 = 0;
                
                Theta3_3 = 0;
                Theta3_4 = 0;
                
                Theta2_3 = 0;
                Theta2_4 = 0;
            else
                Theta3_3 = atan2(sqrt(1-D_BL_2^2),D_BL_2);
                Theta3_4 = atan2(-sqrt(1-D_BL_2^2),D_BL_2);
                
                Theta2_3 = atan2(s_BL_2,r_BL_2) - atan2(L3*sin(Theta3_3),L2 + L3*cos(Theta3_3));
                Theta2_4 = atan2(s_BL_2,r_BL_2) - atan2(L3*sin(Theta3_4),L2 + L3*cos(Theta3_4));
                
                
            end
            
            if D_BL > 0.999
                loop_toggle = 0;
                r_II_c_d = r_II_c_d - travel_dir.*0.01;
                r_II_c_d(3) = r_II_c_0(3);
            else
                Theta3 = atan2(sqrt(1-D_BL^2),D_BL);
                Theta3_2 = atan2(-sqrt(1-D_BL^2),D_BL);
                
                Theta2 = atan2(s_BL,r_BL) - atan2(L3*sin(Theta3),L2 + L3*cos(Theta3));
                Theta2_2 = atan2(s_BL,r_BL) - atan2(L3*sin(Theta3_2),L2 + L3*cos(Theta3_2));
                
                
            end
            
        otherwise
            error("Leg_Index is set to an invalid value")
    end
    
    T1_tolerance = 0.2;
    
    % if theta1 wraps around into robot
    switch leg_index
        case 1
            T1_cond = (Theta1 <= -pi/2) || (Theta1 >= pi); % FR
            hardstop_cond = (Theta1 > pi/2-T1_tolerance) || (Theta1 < 0+T1_tolerance) ;
            hardstop_cond_2 = (Theta1_2 > pi/2-T1_tolerance) || (Theta1_2 < 0+T1_tolerance) ;
        case 2
            T1_cond = (Theta1 <= -pi) || (Theta1 >= pi/2); %FL
            hardstop_cond = (Theta1 > 0-T1_tolerance) || (Theta1 < -pi/2+T1_tolerance);
            hardstop_cond_2 = (Theta1_2 > 0-T1_tolerance) || (Theta1_2 < -pi/2+T1_tolerance);
        case 3
            T1_cond = (Theta1 <= -pi) || (Theta1 >= pi/2); % BR
            hardstop_cond = (Theta1 > 0-T1_tolerance) || (Theta1 < -pi/2+T1_tolerance);
            hardstop_cond_2 = (Theta1_2 > 0-T1_tolerance) || (Theta1_2 < -pi/2+T1_tolerance);
        case 4
            T1_cond = (Theta1 <= -pi/2) || (Theta1 >= pi); % BL
            hardstop_cond = (Theta1 > pi/2-T1_tolerance) || (Theta1 < 0+T1_tolerance) ;
            hardstop_cond_2 = (Theta1_2 > pi/2-T1_tolerance) || (Theta1_2 < 0+T1_tolerance) ;
        otherwise
            error("Leg Index is not a allowed value")
            
    end
    
    if T1_cond
        Theta1 = Theta1_2;
        Theta2 = Theta2_4;
        Theta3 = Theta3_4;
        if hardstop_cond_2
            loop_toggle = 0;
            r_II_c_d = r_II_c_d - travel_dir.*0.01;
            r_II_c_d(3) = r_II_c_0(3);
        end
    else
        Theta2 = Theta2_2;
        Theta3 = Theta3_2;
        if hardstop_cond
            loop_toggle = 0;
            r_II_c_d = r_II_c_d - travel_dir.*0.01;
            r_II_c_d(3) = r_II_c_0(3);
        end
    end
end

end