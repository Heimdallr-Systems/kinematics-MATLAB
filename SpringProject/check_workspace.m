function [legs_valid]=check_workspace(Theta)
%     This function is used to solve for IK solutions of the robot's legs when
% measurements in the inertial frame are unknown. 
% r_BB_c = [r_BB_c_FR,r_BB_c_FL, r_BB_c_BR, r_BB_c_BL];
% r_BB_c_?? = [x_BB_c; y_BB_c; z_BB_c]
% BodyRot = zyx rotation of body
% r_II_B = vector from inertial to body
% Returns: Logical array indicating that a position is not valid for each
% leg.
% Preset the valid outputs to 1, assume validity
FR_valid=1;
FL_valid=1;
BR_valid=1;
BL_valid=1;

Theta1_FR = Theta(1);
Theta1_FL = Theta(2);
Theta1_BR = Theta(3);
Theta1_BL = Theta(4);

Theta2_FR = Theta(5);
Theta2_FL = Theta(6);
Theta2_BR = Theta(7);
Theta2_BL = Theta(8);

Theta3_FR = Theta(9);
Theta3_FL = Theta(10);
Theta3_BR = Theta(11);
Theta3_BL = Theta(12);

% % Constants, known offsets
% RobotConstants;
% L2 = norm(r_22_3_FR);
% L3 = norm(r_33_c_FR);
% 
% % Vector from Base to Contact
% r_BB_c_FR = r_BB_c(:,1);
% r_BB_c_FL = r_BB_c(:,2);
% r_BB_c_BR = r_BB_c(:,3);
% r_BB_c_BL = r_BB_c(:,4);
% 
% % Vector from Base to Contact minus first joint offset from base.
% % This is used to eliminate the offset so the frame can be centered on
% % the first joint, allowing for arctangent function to be used to solve
% % for theta1.
% r_B1_c_FR = r_BB_c_FR - r_BB_1_FR;
% r_B1_c_FL = r_BB_c_FL - r_BB_1_FL;
% r_B1_c_BR = r_BB_c_BR - r_BB_1_BR;
% r_B1_c_BL = r_BB_c_BL - r_BB_1_BL;
% 
% % Solve for Theta1
% Theta1_FR = atan2(r_B1_c_FR(2),r_B1_c_FR(1))+pi/2;
% Theta1_FL = atan2(r_B1_c_FL(2),r_B1_c_FL(1))-pi/2;
% Theta1_BR = atan2(r_B1_c_BR(2),r_B1_c_BR(1))+pi/2;
% Theta1_BL = atan2(r_B1_c_BL(2),r_B1_c_BL(1))-pi/2;
% 
% % wrap theta1 between +-pi
% Theta1_FR = angle(exp(1j*Theta1_FR));
% Theta1_FL = angle(exp(1j*Theta1_FL));
% Theta1_BR = angle(exp(1j*Theta1_BR));
% Theta1_BL = angle(exp(1j*Theta1_BL));

Theta1 = [Theta1_FR, Theta1_FL, Theta1_BR, Theta1_BL];
if(abs(Theta1(1) - pi/4)>(pi/3))
    FR_valid=0;
end
if(abs(Theta1(2) - (-pi/4))> (pi/2.5))
    FL_valid=0;
end
if(abs(Theta1(3) - (-pi/4))>(pi/2.5))
    BR_valid=0;
end
if(abs(Theta1(4) - pi/4)> (pi/2.5))  
    BL_valid=0;
end
% Solve for Theta3
% Set up a positive frame centered on joint two, aligned with frame 1 in order to apply standard
% two-link manipulator solution (refer to textbook)
% r_1prime1_c_FR = rotz(pi)\(rotz(Theta1_FR)\r_B1_c_FR);
% r_11_c_FL = rotz(Theta1_FL)\r_B1_c_FL;
% r_1prime1_c_BR  = rotz(pi)\(rotz(Theta1_BR)\r_B1_c_BR);
% r_11_c_BL = rotz(Theta1_BL)\r_B1_c_BL;
% 
% % vectors from joint 1 to joint 2
% % transform vectors to positive, no offset frame to allow for standard two link
% % problem
% r_1prime1_2_FR = rotz(pi)\r_11_2_FR;
% r_1prime1_2_BR = rotz(pi)\r_11_2_BR;
% % solves for y-axis component
% r_FR = r_1prime1_c_FR(2)-r_1prime1_2_FR(2);
% r_FL = r_11_c_FL(2)-r_11_2_FL(2);
% r_BR = r_1prime1_c_BR(2)-r_1prime1_2_BR(2);
% r_BL = r_11_c_BL(2)-r_11_2_BL(2);
% % solves for z-axis component
% s_FR = r_1prime1_c_FR(3)-r_1prime1_2_FR(3);
% s_FL = r_11_c_FL(3)-r_11_2_FL(3);
% s_BR = r_1prime1_c_BR(3)-r_1prime1_2_BR(3);
% s_BL = r_11_c_BL(3)-r_11_2_BL(3);
% % known lengths of last two links
% 
% % cos(theta3)
% D_FR = (r_FR^2 + s_FR^2 - L2^2 - L3^2)/(2*L2*L3);
% D_FL = (r_FL^2 + s_FL^2 - L2^2 - L3^2)/(2*L2*L3);
% D_BR = (r_BR^2 + s_BR^2 - L2^2 - L3^2)/(2*L2*L3);
% D_BL = (r_BL^2 + s_BL^2 - L2^2 - L3^2)/(2*L2*L3);
% % if rounding causes the solution to be slightly greater than one, it
% % rounds it to one to allow for a solution to be computed
% if D_FR > 1
%     FR_valid=0;
%     D_FR=1;
% end
% if D_FL > 1
%     FL_valid=0;
%     D_FL=1;
% end
% if D_BR > 1
%     D_BR=1;
%     BR_valid=0;
% end
% if D_BL > 1
%     BL_valid=0;
%     D_BL=1;
% end
% % Theta 3 values measured with respect to the frame (frame 1 prime) used to solve standard
% % 2-link problem frame . These will be used to solve for the theta 2 values
% 
% Theta3_FR_Temp = atan2(sqrt(1-D_FR^2),D_FR);
% Theta3_FR_2_Temp = atan2(-sqrt(1-D_FR^2),D_FR);
% Theta3_BR_Temp = atan2(sqrt(1-D_BR^2),D_BR);
% Theta3_BR_2_Temp = atan2(-sqrt(1-D_BR^2),D_BR);
% 
% % Theta 3 values measured in defined way (from zeroed position)
% Theta3_FR = -atan2(sqrt(1-D_FR^2),D_FR);
% Theta3_FR_2 = -atan2(-sqrt(1-D_FR^2),D_FR);
% Theta3_FL = atan2(sqrt(1-D_FL^2),D_FL);
% Theta3_FL_2 = atan2(-sqrt(1-D_FL^2),D_FL);
% Theta3_BR = -atan2(sqrt(1-D_BR^2),D_BR);
% Theta3_BR_2 = -atan2(-sqrt(1-D_BR^2),D_BR);
% Theta3_BL = atan2(sqrt(1-D_BL^2),D_BL);
% Theta3_BL_2 = atan2(-sqrt(1-D_BL^2),D_BL);

% check for the extension of the last join
if(abs(Theta3_FL*180/pi)<35)
    FL_valid=0;
end
if(abs(Theta3_BR*180/pi)<35)
    BR_valid=0;
end
if(abs(Theta3_FR*180/pi)<35)
    FR_valid=0;
end
if(abs(Theta3_BL*180/pi)<35)
    
    BL_valid=0;
end


if(abs(Theta3_FL*180/pi)>110) && (abs(Theta2_FL*180/pi)<30)
    FL_valid=0;
end
if(abs(Theta3_BR*180/pi)>110)&& (abs(Theta2_BR*180/pi)<30)
    BR_valid=0;
end
if(abs(Theta3_FR*180/pi)>110)&& (abs(Theta2_FR*180/pi)<30)
    FR_valid=0;
end
if(abs(Theta3_BL*180/pi)>110)&& (abs(Theta2_BL*180/pi)<30)
   BL_valid = 0; 
   
end
legs_valid=[FR_valid, FL_valid,BR_valid,BL_valid];
end