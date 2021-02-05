function [r_II_B_z_plus, theta_plus, psi_plus] = IK_Solver_Tilt(X,r_BB_c, legs_on_gnd)
% This function finds the z-height of the body and the x/y-rotation, or
% tilt, of the body. Must provide an initial guess of these values. then the position of the
% contact points with respect to the body. This should only be used when
% the positions of the contact points with respect to the inertial frame are
% unknown, and only the positions of the contact points with respect to the
% base are known. This function also only uses contact point positions with
% respect to the body which will always be more accurate than the position
% of the contacts with respect to the inertial. So for a more accurate tilt
% measurement, this function may be more desireable for tilt stability than
% IK_Solver_BodyRot_BodyPos.m
%
% Due to the naming of the symbolic variables, the legs will referenced by
% FL,FR, and BL when there are three legs touching the ground, even if
% those aren't the correct legs that are touching the ground.
%
% THIS FUNCTION ONLY WORKS ASSUMING ALL CONTACT POINTS ARE TOUCHING THE
% GROUND, WHERE THE GROUND IS A FLAT PLANE WHERE THE Z-HEIGHT IN INERTIAL
% FRAME IS ZERO.
% X = [r_II_B_z;theta;psi];
% r_BB_c = [r_BB_c_FR_x, r_BB_c_FL_x, r_BB_c_BR_x, r_BB_c_BL_x
%                    r_BB_c_FR_y, r_BB_c_FL_y, r_BB_c_BR_y, r_BB_c_BL_y
%                    r_BB_c_FR_z, r_BB_c_FL_z, r_BB_c_BR_z, r_BB_c_BL_z]
% legs_on_gnd = [1 or 0,1 or 0,1 or 0,1 or 0] either 3 or 4 legs are touching the ground

legs = find(legs_on_gnd == 1);

if length(legs) == 4
    r_BB_c_FR_x = r_BB_c(1,1);
    r_BB_c_FR_y = r_BB_c(2,1);
    r_BB_c_FR_z = r_BB_c(3,1);
    r_BB_c_FL_x = r_BB_c(1,2);
    r_BB_c_FL_y = r_BB_c(2,2);
    r_BB_c_FL_z = r_BB_c(3,2);
    r_BB_c_BR_x = r_BB_c(1,3);
    r_BB_c_BR_y = r_BB_c(2,3);
    r_BB_c_BR_z = r_BB_c(3,3);
    r_BB_c_BL_x = r_BB_c(1,4);
    r_BB_c_BL_y = r_BB_c(2,4);
    r_BB_c_BL_z = r_BB_c(3,4);
elseif length(legs) == 3
    r_BB_c_FR_x = r_BB_c(1,legs(1));
    r_BB_c_FR_y = r_BB_c(2,legs(1));
    r_BB_c_FR_z = r_BB_c(3,legs(1));
    r_BB_c_FL_x = r_BB_c(1,legs(2));
    r_BB_c_FL_y = r_BB_c(2,legs(2));
    r_BB_c_FL_z = r_BB_c(3,legs(2));
    r_BB_c_BR_x = r_BB_c(1,legs(3));
    r_BB_c_BR_y = r_BB_c(2,legs(3));
    r_BB_c_BR_z = r_BB_c(3,legs(3));
else
    error('Number of legs touching ground needs to be either 3 or 4')
end

% initial guess
r_II_B_z = X(1);
theta = X(2);
psi = X(3);

error = 1;
jj = 0;

% compute with Newton-Raphson Method until error is small, or 300
% iterations have passed
while (error > 0.001) && (jj < 300)
    jj = jj+1;
    
    if length(legs) == 4
        FX = [r_II_B_z - sin(theta)*r_BB_c_FR_x + cos(theta)*sin(psi)*r_BB_c_FR_y + cos(psi)*cos(theta)*r_BB_c_FR_z;...
            r_II_B_z - sin(theta)*r_BB_c_FL_x + cos(theta)*sin(psi)*r_BB_c_FL_y + cos(psi)*cos(theta)*r_BB_c_FL_z;...
            r_II_B_z - sin(theta)*r_BB_c_BR_x + cos(theta)*sin(psi)*r_BB_c_BR_y + cos(psi)*cos(theta)*r_BB_c_BR_z;...
            r_II_B_z - sin(theta)*r_BB_c_BL_x + cos(theta)*sin(psi)*r_BB_c_BL_y + cos(psi)*cos(theta)*r_BB_c_BL_z];
        
        dFdX = [ 1, - r_BB_c_FR_x*cos(theta) - r_BB_c_FR_z*cos(psi)*sin(theta) - r_BB_c_FR_y*sin(psi)*sin(theta), r_BB_c_FR_y*cos(psi)*cos(theta) - r_BB_c_FR_z*cos(theta)*sin(psi)
            1, - r_BB_c_FL_x*cos(theta) - r_BB_c_FL_z*cos(psi)*sin(theta) - r_BB_c_FL_y*sin(psi)*sin(theta), r_BB_c_FL_y*cos(psi)*cos(theta) - r_BB_c_FL_z*cos(theta)*sin(psi)
            1, - r_BB_c_BR_x*cos(theta) - r_BB_c_BR_z*cos(psi)*sin(theta) - r_BB_c_BR_y*sin(psi)*sin(theta), r_BB_c_BR_y*cos(psi)*cos(theta) - r_BB_c_BR_z*cos(theta)*sin(psi)
            1, - r_BB_c_BL_x*cos(theta) - r_BB_c_BL_z*cos(psi)*sin(theta) - r_BB_c_BL_y*sin(psi)*sin(theta), r_BB_c_BL_y*cos(psi)*cos(theta) - r_BB_c_BL_z*cos(theta)*sin(psi)];
        
    elseif length(legs) == 3
        FX = [r_II_B_z - sin(theta)*r_BB_c_FR_x + cos(theta)*sin(psi)*r_BB_c_FR_y + cos(psi)*cos(theta)*r_BB_c_FR_z;...
            r_II_B_z - sin(theta)*r_BB_c_FL_x + cos(theta)*sin(psi)*r_BB_c_FL_y + cos(psi)*cos(theta)*r_BB_c_FL_z;...
            r_II_B_z - sin(theta)*r_BB_c_BR_x + cos(theta)*sin(psi)*r_BB_c_BR_y + cos(psi)*cos(theta)*r_BB_c_BR_z];
        
        dFdX = [ 1, - r_BB_c_FR_x*cos(theta) - r_BB_c_FR_z*cos(psi)*sin(theta) - r_BB_c_FR_y*sin(psi)*sin(theta), r_BB_c_FR_y*cos(psi)*cos(theta) - r_BB_c_FR_z*cos(theta)*sin(psi)
            1, - r_BB_c_FL_x*cos(theta) - r_BB_c_FL_z*cos(psi)*sin(theta) - r_BB_c_FL_y*sin(psi)*sin(theta), r_BB_c_FL_y*cos(psi)*cos(theta) - r_BB_c_FL_z*cos(theta)*sin(psi)
            1, - r_BB_c_BR_x*cos(theta) - r_BB_c_BR_z*cos(psi)*sin(theta) - r_BB_c_BR_y*sin(psi)*sin(theta), r_BB_c_BR_y*cos(psi)*cos(theta) - r_BB_c_BR_z*cos(theta)*sin(psi)];
    end
    
    Xplus = X - dFdX\FX;
    
    r_II_B_z_plus = Xplus(1);
    theta_plus = Xplus(2);
    psi_plus = Xplus(3);
    
    if length(legs) == 4
        FXplus = [r_II_B_z_plus - sin(theta_plus)*r_BB_c_FR_x + cos(theta_plus)*sin(psi_plus)*r_BB_c_FR_y + cos(psi_plus)*cos(theta_plus)*r_BB_c_FR_z;...
            r_II_B_z_plus - sin(theta_plus)*r_BB_c_FL_x + cos(theta_plus)*sin(psi_plus)*r_BB_c_FL_y + cos(psi_plus)*cos(theta_plus)*r_BB_c_FL_z;...
            r_II_B_z_plus - sin(theta_plus)*r_BB_c_BR_x + cos(theta_plus)*sin(psi_plus)*r_BB_c_BR_y + cos(psi_plus)*cos(theta_plus)*r_BB_c_BR_z;...
            r_II_B_z_plus - sin(theta_plus)*r_BB_c_BL_x + cos(theta_plus)*sin(psi_plus)*r_BB_c_BL_y + cos(psi_plus)*cos(theta_plus)*r_BB_c_BL_z];
    elseif length(legs) == 3
        FXplus = [r_II_B_z_plus - sin(theta_plus)*r_BB_c_FR_x + cos(theta_plus)*sin(psi_plus)*r_BB_c_FR_y + cos(psi_plus)*cos(theta_plus)*r_BB_c_FR_z;...
            r_II_B_z_plus - sin(theta_plus)*r_BB_c_FL_x + cos(theta_plus)*sin(psi_plus)*r_BB_c_FL_y + cos(psi_plus)*cos(theta_plus)*r_BB_c_FL_z;...
            r_II_B_z_plus - sin(theta_plus)*r_BB_c_BR_x + cos(theta_plus)*sin(psi_plus)*r_BB_c_BR_y + cos(psi_plus)*cos(theta_plus)*r_BB_c_BR_z];
    end
    
    error = norm(FX - FXplus);
    
    r_II_B_z = r_II_B_z_plus;
    theta = theta_plus;
    psi = psi_plus;
    
    X = [r_II_B_z;theta;psi];
end
end