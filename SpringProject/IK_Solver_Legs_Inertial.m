function [Theta1, Theta2, Theta2_2, Theta3, Theta3_2] = IK_Solver_Legs_Inertial(r_II_c, T_I_B, r_II_B, legs_on_gnd)
% This function is used to solve for IK solutions of the robot's legs when
% measurements in terms of the inertial frame are known. Use this function
% when solving for tilt under the flat plane assumption.
% r_II_c = [r_II_c_FR,r_II_c_FL, r_II_c_BR, r_II_c_BL];
% r_II_c_?? = [x_II_c; y_II_c; z_II_c]
% BodyRot = zyx rotation of body
% r_II_B = vector from inertial to body
% Returns: Theta1 of each leg, both solutions for theta2 of each leg, both
% solutions for theta3 of each leg

% Constants, known offsets
constants=RobotConstants();
% known lengths of last two links
L2 = norm(constants.r_22_3_BL);
L3 = norm(constants.r_33_c_BL);

%initialize
Theta1 = zeros(4,1);
Theta2 = zeros(4,1);
Theta2_2 = zeros(4,1);
Theta3 = zeros(4,1);
Theta3_2 = zeros(4,1);

%% FR LEG
if legs_on_gnd(1) == 1
    % given I vectors
    r_II_c_FR = r_II_c(:,1);
    
    % convert to body vectors
    constants.r_BB_c_FR = T_I_B\(r_II_c_FR - r_II_B);
    
    % Vector from Base to Contact minus first joint offset from base.
    % This is used to eliminate the offset so the frame can be centered on
    % the first joint, allowing for arctangent function to be used to solve
    % for theta1.
    constants.r_B1_c_FR = constants.r_BB_c_FR - constants.r_BB_1_FR;
    
    % Solve for Theta1
    Theta1_FR = atan2(constants.r_B1_c_FR(2),constants.r_B1_c_FR(1))+pi/2;
    
    % wrap theta1 between +-pi
    Theta1_FR = angle(exp(1j*Theta1_FR));
    
    % Solve for Theta3
    % Set up a positive frame centered on joint two, aligned with frame 1 in order to apply standard
    % two-link manipulator solution (refer to textbook)
    r_1prime1_c_FR = rotz(pi)\(rotz(Theta1_FR)\constants.r_B1_c_FR);
    
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
    
    % if rounding causes the solution to be slightly greater than one, it
    % rounds it to one to allow for a solution to be computed
    % also notifies of error in workspace
    
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
    if D_FR > 1
        D_FR = 1;
        disp('D was rounded to 1.')
    end
    Theta1(1,1) = Theta1_FR;
    Theta2(1,1) = Theta2_FR;
    Theta2_2(1,1) = Theta2_FR_2;
    Theta3(1,1) = Theta3_FR;
    Theta3_2(1,1) = Theta3_FR_2;
end
%% FL LEG
if legs_on_gnd(2) == 1
    r_II_c_FL = r_II_c(:,2);
    
    constants.r_BB_c_FL = T_I_B\(r_II_c_FL - r_II_B);
    
    constants.r_B1_c_FL = constants.r_BB_c_FL - constants.r_BB_1_FL;
    
    Theta1_FL = atan2(constants.r_B1_c_FL(2),constants.r_B1_c_FL(1))-pi/2;
    
    Theta1_FL = angle(exp(1j*Theta1_FL));
    
    constants.r_11_c_FL = rotz(Theta1_FL)\constants.r_B1_c_FL;
    
    r_FL = constants.r_11_c_FL(2)-constants.r_11_2_FL(2);
    
    s_FL = constants.r_11_c_FL(3)-constants.r_11_2_FL(3);
    
    D_FL = (r_FL^2 + s_FL^2 - L2^2 - L3^2)/(2*L2*L3);
    
    Theta3_FL = atan2(sqrt(1-D_FL^2),D_FL);
    Theta3_FL_2 = atan2(-sqrt(1-D_FL^2),D_FL);
    
    Theta2_FL = atan2(s_FL,r_FL) - atan2(L3*sin(Theta3_FL),L2 + L3*cos(Theta3_FL));
    Theta2_FL_2 = atan2(s_FL,r_FL) - atan2(L3*sin(Theta3_FL_2),L2 + L3*cos(Theta3_FL_2));
    if D_FL > 1
        D_FL = 1;
        disp('D was rounded to 1.')
    end
    Theta1(2,1) = Theta1_FL;
    Theta2(2,1) = Theta2_FL;
    Theta2_2(2,1) = Theta2_FL_2;
    Theta3(2,1) = Theta3_FL;
    Theta3_2(2,1) = Theta3_FL_2;
end
%% BR LEG
if legs_on_gnd(3) == 1
    r_II_c_BR = r_II_c(:,3);
    
    constants.r_BB_c_BR = T_I_B\(r_II_c_BR - r_II_B);
    
    constants.r_B1_c_BR = constants.r_BB_c_BR - constants.r_BB_1_BR;
    
    Theta1_BR = atan2(constants.r_B1_c_BR(2),constants.r_B1_c_BR(1))+pi/2;
    
    Theta1_BR = angle(exp(1j*Theta1_BR));
    
    r_1prime1_c_BR  = rotz(pi)\(rotz(Theta1_BR)\constants.r_B1_c_BR);
    
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
    if D_BR > 1
        D_BR = 1;
        disp('D was rounded to 1.')
    end
    Theta1(3,1) = Theta1_BR;
    Theta2(3,1) = Theta2_BR;
    Theta2_2(3,1) = Theta2_BR_2;
    Theta3(3,1) = Theta3_BR;
    Theta3_2(3,1) = Theta3_BR_2;
end
%% BL LEG
if legs_on_gnd(4) == 1
    r_II_c_BL = r_II_c(:,4);
    
    constants.r_BB_c_BL = T_I_B\(r_II_c_BL - r_II_B);
    
    constants.r_B1_c_BL = constants.r_BB_c_BL - constants.r_BB_1_BL;
    
    Theta1_BL = atan2(constants.r_B1_c_BL(2),constants.r_B1_c_BL(1))-pi/2;
    
    Theta1_BL = angle(exp(1j*Theta1_BL));
    
    constants.r_11_c_BL = rotz(Theta1_BL)\constants.r_B1_c_BL;
    
    r_BL = constants.r_11_c_BL(2)-constants.r_11_2_BL(2);
    
    s_BL = constants.r_11_c_BL(3)-constants.r_11_2_BL(3);
    
    D_BL = (r_BL^2 + s_BL^2 - L2^2 - L3^2)/(2*L2*L3);
    
    Theta3_BL = atan2(sqrt(1-D_BL^2),D_BL);
    Theta3_BL_2 = atan2(-sqrt(1-D_BL^2),D_BL);
    
    Theta2_BL = atan2(s_BL,r_BL) - atan2(L3*sin(Theta3_BL),L2 + L3*cos(Theta3_BL));
    Theta2_BL_2 = atan2(s_BL,r_BL) - atan2(L3*sin(Theta3_BL_2),L2 + L3*cos(Theta3_BL_2));
    if D_BL > 1
        D_BL = 1;
        disp('D was rounded to 1.')
    end
    
    Theta1(4,1) = Theta1_BL;
    Theta2(4,1) = Theta2_BL;
    Theta2_2(4,1) = Theta2_BL_2;
    Theta3(4,1) = Theta3_BL;
    Theta3_2(4,1) = Theta3_BL_2;
end
end