function [Theta1, Theta2, Theta2_2, Theta3, Theta3_2] = IK_Solver_Legs(r_BB_c)
% This function is used to solve for IK solutions of the robot's legs when
% measurements in the inertial frame are unknown. 
% r_BB_c = [constants.r_BB_c_FR,constants.r_BB_c_FL, constants.r_BB_c_BR, constants.r_BB_c_BL];
% r_BB_c_?? = [x_BB_c; y_BB_c; z_BB_c]
% BodyRot = zyx rotation of body
% r_II_B = vector from inertial to body
% Returns: Theta1 of each leg, both solutions for theta2 of each leg, both
% solutions for theta3 of each leg

% Constants, known offsets
RobotConstants;

% Vector from Base to Contact
constants.r_BB_c_FR = r_BB_c(:,1);
constants.r_BB_c_FL = r_BB_c(:,2);
constants.r_BB_c_BR = r_BB_c(:,3);
constants.r_BB_c_BL = r_BB_c(:,4);

% Vector from Base to Contact minus first joint offset from base.
% This is used to eliminate the offset so the frame can be centered on
% the first joint, allowing for arctangent function to be used to solve
% for theta1.
constants.r_B1_c_FR = constants.r_BB_c_FR - constants.r_BB_1_FR;
constants.r_B1_c_FL = constants.r_BB_c_FL - constants.r_BB_1_FL;
constants.r_B1_c_BR = constants.r_BB_c_BR - constants.r_BB_1_BR;
constants.r_B1_c_BL = constants.r_BB_c_BL - constants.r_BB_1_BL;

% Solve for Theta1
Theta1_FR = atan2(constants.r_B1_c_FR(2),constants.r_B1_c_FR(1))+pi/2;
Theta1_FL = atan2(constants.r_B1_c_FL(2),constants.r_B1_c_FL(1))-pi/2;
Theta1_BR = atan2(constants.r_B1_c_BR(2),constants.r_B1_c_BR(1))+pi/2;
Theta1_BL = atan2(constants.r_B1_c_BL(2),constants.r_B1_c_BL(1))-pi/2;

% wrap theta1 between +-pi
Theta1_FR = angle(exp(1j*Theta1_FR));
Theta1_FL = angle(exp(1j*Theta1_FL));
Theta1_BR = angle(exp(1j*Theta1_BR));
Theta1_BL = angle(exp(1j*Theta1_BL));

Theta1 = [Theta1_FR, Theta1_FL, Theta1_BR, Theta1_BL];

% Solve for Theta3
% Set up a positive frame centered on joint two, aligned with frame 1 in order to apply standard
% two-link manipulator solution (refer to textbook)
r_1prime1_c_FR = rotz(pi)\(rotz(Theta1_FR)\constants.r_B1_c_FR);
constants.r_11_c_FL = rotz(Theta1_FL)\constants.r_B1_c_FL;
r_1prime1_c_BR  = rotz(pi)\(rotz(Theta1_BR)\constants.r_B1_c_BR);
constants.r_11_c_BL = rotz(Theta1_BL)\constants.r_B1_c_BL;

% vectors from joint 1 to joint 2
% transform vectors to positive, no offset frame to allow for standard two link
% problem
r_1prime1_2_FR = rotz(pi)\constants.r_11_2_FR;
r_1prime1_2_BR = rotz(pi)\constants.r_11_2_BR;
% solves for y-axis component
r_FR = r_1prime1_c_FR(2)-r_1prime1_2_FR(2);
r_FL = constants.r_11_c_FL(2)-constants.r_11_2_FL(2);
r_BR = r_1prime1_c_BR(2)-r_1prime1_2_BR(2);
r_BL = constants.r_11_c_BL(2)-constants.r_11_2_BL(2);
% solves for z-axis component
s_FR = r_1prime1_c_FR(3)-r_1prime1_2_FR(3);
s_FL = constants.r_11_c_FL(3)-constants.r_11_2_FL(3);
s_BR = r_1prime1_c_BR(3)-r_1prime1_2_BR(3);
s_BL = constants.r_11_c_BL(3)-constants.r_11_2_BL(3);
% known lengths of last two links
L2 = 224.68;
L3 = 279;
% cos(theta3)
D_FR = (r_FR^2 + s_FR^2 - L2^2 - L3^2)/(2*L2*L3);
D_FL = (r_FL^2 + s_FL^2 - L2^2 - L3^2)/(2*L2*L3);
D_BR = (r_BR^2 + s_BR^2 - L2^2 - L3^2)/(2*L2*L3);
D_BL = (r_BL^2 + s_BL^2 - L2^2 - L3^2)/(2*L2*L3);
% if rounding causes the solution to be slightly greater than one, it
% rounds it to one to allow for a solution to be computed
if D_FR > 1
    D_FR = 1;
    disp('D was rounded to 1.')
end
if D_FL > 1
    D_FL = 1;
    disp('D was rounded to 1.')
end
if D_BR > 1
    D_BR = 1;
    disp('D was rounded to 1.')
end
if D_BL > 1
    D_BL = 1;
    disp('D was rounded to 1.')
end

% Theta 3 values measured with respect to the frame (frame 1 prime) used to solve standard
% 2-link problem frame . These will be used to solve for the theta 2 values
Theta3_FR_Temp = atan2(sqrt(1-D_FR^2),D_FR);
Theta3_FR_2_Temp = atan2(-sqrt(1-D_FR^2),D_FR);
Theta3_BR_Temp = atan2(sqrt(1-D_BR^2),D_BR);
Theta3_BR_2_Temp = atan2(-sqrt(1-D_BR^2),D_BR);

% Theta 3 values measured in defined way (from zeroed position)
Theta3_FR = -atan2(sqrt(1-D_FR^2),D_FR);
Theta3_FR_2 = -atan2(-sqrt(1-D_FR^2),D_FR);
Theta3_FL = atan2(sqrt(1-D_FL^2),D_FL);
Theta3_FL_2 = atan2(-sqrt(1-D_FL^2),D_FL);
Theta3_BR = -atan2(sqrt(1-D_BR^2),D_BR);
Theta3_BR_2 = -atan2(-sqrt(1-D_BR^2),D_BR);
Theta3_BL = atan2(sqrt(1-D_BL^2),D_BL);
Theta3_BL_2 = atan2(-sqrt(1-D_BL^2),D_BL);

Theta3 = [Theta3_FR, Theta3_FL, Theta3_BR, Theta3_BL]; % solution 1
Theta3_2 = [Theta3_FR_2, Theta3_FL_2, Theta3_BR_2, Theta3_BL_2]; % solution 2

% Solve for Theta2
Theta2_FR = -(atan2(s_FR,r_FR) - atan2(L3*sin(Theta3_FR_Temp),L2 + L3*cos(Theta3_FR_Temp)));
Theta2_FR_2 = -(atan2(s_FR,r_FR) - atan2(L3*sin(Theta3_FR_2_Temp),L2 + L3*cos(Theta3_FR_2_Temp)));
Theta2_FL = atan2(s_FL,r_FL) - atan2(L3*sin(Theta3_FL),L2 + L3*cos(Theta3_FL));
Theta2_FL_2 = atan2(s_FL,r_FL) - atan2(L3*sin(Theta3_FL_2),L2 + L3*cos(Theta3_FL_2));
Theta2_BR = -(atan2(s_BR,r_BR) - atan2(L3*sin(Theta3_BR_Temp),L2 + L3*cos(Theta3_BR_Temp)));
Theta2_BR_2 = -(atan2(s_BR,r_BR) - atan2(L3*sin(Theta3_BR_2_Temp),L2 + L3*cos(Theta3_BR_2_Temp)));
Theta2_BL = atan2(s_BL,r_BL) - atan2(L3*sin(Theta3_BL),L2 + L3*cos(Theta3_BL));
Theta2_BL_2 = atan2(s_BL,r_BL) - atan2(L3*sin(Theta3_BL_2),L2 + L3*cos(Theta3_BL_2));

Theta2 = [Theta2_FR, Theta2_FL, Theta2_BR, Theta2_BL]; % solution 1
Theta2_2 = [Theta2_FR_2, Theta2_FL_2, Theta2_BR_2, Theta2_BL_2]; % solution 2


end