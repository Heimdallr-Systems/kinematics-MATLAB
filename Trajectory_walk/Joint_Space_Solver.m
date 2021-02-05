function [Theta1_d,Theta2_d,Theta3_d] = Joint_Space_Solver(Theta1_0, Theta2_0, Theta3_0, Euler_0, r_II_B_0, r_II_B_d, legs_on_gnd)
% This function will solve for desired joint space positions given a
% desired body position.
% r_II_B_d = [Bx;By;Bz];
%
% Initial Body Orientation wrt I
% T_I_B_0  = rotz(phi)roty(theta)rotx(psi)
%
% Initial Body Position
% r_II_B_0 = [x;y;z];
%
% initial joint positions
% Theta1_0 = [Theta1_FR, Theta1_FL, Theta1_BR, Theta1_BL]
% Theta2_0 = [Theta2_FR, Theta2_FL, Theta2_BR, Theta2_BL]
% Theta3_0 = [Theta3_FR, Theta3_FL, Theta3_BR, Theta3_BL]
%
% desired/initial body orientation wrt zyx Euler sequence
% Euler_d = [phi,theta,psi]
%
% legs_on_gnd - logical array for telling if at least 3 or max four legs on
% gnd
% legs_on_gnd = [1/0,1/0,1/0,1/0];

legs = find(legs_on_gnd == 1);

if ~((length(legs) == 3) || (length(legs) == 4))
   error('Minimum of 3 legs on ground not satisfied');
end

% Desired Orientation
phi = Euler_0(1);
theta = Euler_0(2);
psi = Euler_0(3);
T_I_B_0 = rotz(phi)*roty(theta)*rotx(psi);

% Desired Position
r_II_B_d_x = r_II_B_d(1);
r_II_B_d_y = r_II_B_d(2);
r_II_B_d_z = r_II_B_d(3);

% Initial Joint Angles
Theta1_FR = Theta1_0(1);
Theta1_FL = Theta1_0(2);
Theta1_BR = Theta1_0(3);
Theta1_BL = Theta1_0(4);

Theta2_FR = Theta2_0(1);
Theta2_FL = Theta2_0(2);
Theta2_BR = Theta2_0(3);
Theta2_BL = Theta2_0(4);

Theta3_FR = Theta3_0(1);
Theta3_FL = Theta3_0(2);
Theta3_BR = Theta3_0(3);
Theta3_BL = Theta3_0(4);


% Intial position of contact points with respect to inertial
[r_II_c_FR, r_II_c_FL, r_II_c_BR, r_II_c_BL] = CPos_wrt_I(Theta1_0,Theta2_0,Theta3_0,T_I_B_0,r_II_B_0);

r_II_c_0_x_FR = r_II_c_FR(1);
r_II_c_0_y_FR = r_II_c_FR(2);
r_II_c_0_z_FR = r_II_c_FR(3);
r_II_c_0_x_FL = r_II_c_FL(1);
r_II_c_0_y_FL = r_II_c_FL(2);
r_II_c_0_z_FL = r_II_c_FL(3);
r_II_c_0_x_BR = r_II_c_BR(1);
r_II_c_0_y_BR = r_II_c_BR(2);
r_II_c_0_z_BR = r_II_c_BR(3);
r_II_c_0_x_BL = r_II_c_BL(1);
r_II_c_0_y_BL = r_II_c_BL(2);
r_II_c_0_z_BL = r_II_c_BL(3);

X_FR = [Theta1_FR
    Theta2_FR
    Theta3_FR];
X_FL = [Theta1_FL
    Theta2_FL
    Theta3_FL];
X_BR = [Theta1_BR
    Theta2_BR
    Theta3_BR];
X_BL = [Theta1_BL
    Theta2_BL
    Theta3_BL];

error = 1;
jj = 0;

% compute with Newton-Raphson Method until error is small, or 300
% iterations have passed
while (error > 0.001) && (jj < 300)
    jj = jj+1;
    
    FX_FR = [r_II_c_0_x_FR - r_II_B_d_x + (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(279*sin(Theta2_FR + Theta3_FR) + (5617*sin(Theta2_FR))/25 - 305/4) - (cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*((261*cos(Theta1_FR))/4 + (5617*cos(Theta1_FR)*cos(Theta2_FR))/25 + 279*cos(Theta1_FR)*cos(Theta2_FR)*cos(Theta3_FR) - 279*cos(Theta1_FR)*sin(Theta2_FR)*sin(Theta3_FR) + 5331/100) - cos(phi)*cos(theta)*((261*sin(Theta1_FR))/4 + (5617*cos(Theta2_FR)*sin(Theta1_FR))/25 + 279*cos(Theta2_FR)*cos(Theta3_FR)*sin(Theta1_FR) - 279*sin(Theta1_FR)*sin(Theta2_FR)*sin(Theta3_FR) + 767/5)
        r_II_c_0_y_FR - r_II_B_d_y - (cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*(279*sin(Theta2_FR + Theta3_FR) + (5617*sin(Theta2_FR))/25 - 305/4) + (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*((261*cos(Theta1_FR))/4 + (5617*cos(Theta1_FR)*cos(Theta2_FR))/25 + 279*cos(Theta1_FR)*cos(Theta2_FR)*cos(Theta3_FR) - 279*cos(Theta1_FR)*sin(Theta2_FR)*sin(Theta3_FR) + 5331/100) - cos(theta)*sin(phi)*((261*sin(Theta1_FR))/4 + (5617*cos(Theta2_FR)*sin(Theta1_FR))/25 + 279*cos(Theta2_FR)*cos(Theta3_FR)*sin(Theta1_FR) - 279*sin(Theta1_FR)*sin(Theta2_FR)*sin(Theta3_FR) + 767/5)
        r_II_c_0_z_FR - r_II_B_d_z + sin(theta)*((261*sin(Theta1_FR))/4 + (5617*cos(Theta2_FR)*sin(Theta1_FR))/25 + 279*cos(Theta2_FR)*cos(Theta3_FR)*sin(Theta1_FR) - 279*sin(Theta1_FR)*sin(Theta2_FR)*sin(Theta3_FR) + 767/5) + cos(psi)*cos(theta)*(279*sin(Theta2_FR + Theta3_FR) + (5617*sin(Theta2_FR))/25 - 305/4) + cos(theta)*sin(psi)*((261*cos(Theta1_FR))/4 + (5617*cos(Theta1_FR)*cos(Theta2_FR))/25 + 279*cos(Theta1_FR)*cos(Theta2_FR)*cos(Theta3_FR) - 279*cos(Theta1_FR)*sin(Theta2_FR)*sin(Theta3_FR) + 5331/100)];
    
    FX_FL = [r_II_c_0_x_FL - r_II_B_d_x - (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(279*sin(Theta2_FL + Theta3_FL) + (5617*sin(Theta2_FL))/25 + 305/4) + (cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*((261*cos(Theta1_FL))/4 + (5617*cos(Theta1_FL)*cos(Theta2_FL))/25 + 279*cos(Theta1_FL)*cos(Theta2_FL)*cos(Theta3_FL) - 279*cos(Theta1_FL)*sin(Theta2_FL)*sin(Theta3_FL) + 5331/100) + cos(phi)*cos(theta)*((261*sin(Theta1_FL))/4 + (5617*cos(Theta2_FL)*sin(Theta1_FL))/25 + 279*cos(Theta2_FL)*cos(Theta3_FL)*sin(Theta1_FL) - 279*sin(Theta1_FL)*sin(Theta2_FL)*sin(Theta3_FL) - 767/5)
        r_II_c_0_y_FL - r_II_B_d_y + (cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*(279*sin(Theta2_FL + Theta3_FL) + (5617*sin(Theta2_FL))/25 + 305/4) - (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*((261*cos(Theta1_FL))/4 + (5617*cos(Theta1_FL)*cos(Theta2_FL))/25 + 279*cos(Theta1_FL)*cos(Theta2_FL)*cos(Theta3_FL) - 279*cos(Theta1_FL)*sin(Theta2_FL)*sin(Theta3_FL) + 5331/100) + cos(theta)*sin(phi)*((261*sin(Theta1_FL))/4 + (5617*cos(Theta2_FL)*sin(Theta1_FL))/25 + 279*cos(Theta2_FL)*cos(Theta3_FL)*sin(Theta1_FL) - 279*sin(Theta1_FL)*sin(Theta2_FL)*sin(Theta3_FL) - 767/5)
        r_II_c_0_z_FL - r_II_B_d_z - sin(theta)*((261*sin(Theta1_FL))/4 + (5617*cos(Theta2_FL)*sin(Theta1_FL))/25 + 279*cos(Theta2_FL)*cos(Theta3_FL)*sin(Theta1_FL) - 279*sin(Theta1_FL)*sin(Theta2_FL)*sin(Theta3_FL) - 767/5) - cos(psi)*cos(theta)*(279*sin(Theta2_FL + Theta3_FL) + (5617*sin(Theta2_FL))/25 + 305/4) - cos(theta)*sin(psi)*((261*cos(Theta1_FL))/4 + (5617*cos(Theta1_FL)*cos(Theta2_FL))/25 + 279*cos(Theta1_FL)*cos(Theta2_FL)*cos(Theta3_FL) - 279*cos(Theta1_FL)*sin(Theta2_FL)*sin(Theta3_FL) + 5331/100)];
    
    FX_BR = [r_II_c_0_x_BR - r_II_B_d_x + (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(279*sin(Theta2_BR + Theta3_BR) + (5617*sin(Theta2_BR))/25 - 305/4) - (cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*((261*cos(Theta1_BR))/4 + (5617*cos(Theta1_BR)*cos(Theta2_BR))/25 + 279*cos(Theta1_BR)*cos(Theta2_BR)*cos(Theta3_BR) - 279*cos(Theta1_BR)*sin(Theta2_BR)*sin(Theta3_BR) + 5331/100) - cos(phi)*cos(theta)*((261*sin(Theta1_BR))/4 + (5617*cos(Theta2_BR)*sin(Theta1_BR))/25 + 279*cos(Theta2_BR)*cos(Theta3_BR)*sin(Theta1_BR) - 279*sin(Theta1_BR)*sin(Theta2_BR)*sin(Theta3_BR) - 767/5)
        r_II_c_0_y_BR - r_II_B_d_y - (cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*(279*sin(Theta2_BR + Theta3_BR) + (5617*sin(Theta2_BR))/25 - 305/4) + (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*((261*cos(Theta1_BR))/4 + (5617*cos(Theta1_BR)*cos(Theta2_BR))/25 + 279*cos(Theta1_BR)*cos(Theta2_BR)*cos(Theta3_BR) - 279*cos(Theta1_BR)*sin(Theta2_BR)*sin(Theta3_BR) + 5331/100) - cos(theta)*sin(phi)*((261*sin(Theta1_BR))/4 + (5617*cos(Theta2_BR)*sin(Theta1_BR))/25 + 279*cos(Theta2_BR)*cos(Theta3_BR)*sin(Theta1_BR) - 279*sin(Theta1_BR)*sin(Theta2_BR)*sin(Theta3_BR) - 767/5)
        r_II_c_0_z_BR - r_II_B_d_z + sin(theta)*((261*sin(Theta1_BR))/4 + (5617*cos(Theta2_BR)*sin(Theta1_BR))/25 + 279*cos(Theta2_BR)*cos(Theta3_BR)*sin(Theta1_BR) - 279*sin(Theta1_BR)*sin(Theta2_BR)*sin(Theta3_BR) - 767/5) + cos(psi)*cos(theta)*(279*sin(Theta2_BR + Theta3_BR) + (5617*sin(Theta2_BR))/25 - 305/4) + cos(theta)*sin(psi)*((261*cos(Theta1_BR))/4 + (5617*cos(Theta1_BR)*cos(Theta2_BR))/25 + 279*cos(Theta1_BR)*cos(Theta2_BR)*cos(Theta3_BR) - 279*cos(Theta1_BR)*sin(Theta2_BR)*sin(Theta3_BR) + 5331/100)];
    
    FX_BL = [r_II_c_0_x_BL - r_II_B_d_x - (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(279*sin(Theta2_BL + Theta3_BL) + (5617*sin(Theta2_BL))/25 + 305/4) + (cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*((261*cos(Theta1_BL))/4 + (5617*cos(Theta1_BL)*cos(Theta2_BL))/25 + 279*cos(Theta1_BL)*cos(Theta2_BL)*cos(Theta3_BL) - 279*cos(Theta1_BL)*sin(Theta2_BL)*sin(Theta3_BL) + 5331/100) + cos(phi)*cos(theta)*((261*sin(Theta1_BL))/4 + (5617*cos(Theta2_BL)*sin(Theta1_BL))/25 + 279*cos(Theta2_BL)*cos(Theta3_BL)*sin(Theta1_BL) - 279*sin(Theta1_BL)*sin(Theta2_BL)*sin(Theta3_BL) + 767/5)
        r_II_c_0_y_BL - r_II_B_d_y + (cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*(279*sin(Theta2_BL + Theta3_BL) + (5617*sin(Theta2_BL))/25 + 305/4) - (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*((261*cos(Theta1_BL))/4 + (5617*cos(Theta1_BL)*cos(Theta2_BL))/25 + 279*cos(Theta1_BL)*cos(Theta2_BL)*cos(Theta3_BL) - 279*cos(Theta1_BL)*sin(Theta2_BL)*sin(Theta3_BL) + 5331/100) + cos(theta)*sin(phi)*((261*sin(Theta1_BL))/4 + (5617*cos(Theta2_BL)*sin(Theta1_BL))/25 + 279*cos(Theta2_BL)*cos(Theta3_BL)*sin(Theta1_BL) - 279*sin(Theta1_BL)*sin(Theta2_BL)*sin(Theta3_BL) + 767/5)
        r_II_c_0_z_BL - r_II_B_d_z - sin(theta)*((261*sin(Theta1_BL))/4 + (5617*cos(Theta2_BL)*sin(Theta1_BL))/25 + 279*cos(Theta2_BL)*cos(Theta3_BL)*sin(Theta1_BL) - 279*sin(Theta1_BL)*sin(Theta2_BL)*sin(Theta3_BL) + 767/5) - cos(psi)*cos(theta)*(279*sin(Theta2_BL + Theta3_BL) + (5617*sin(Theta2_BL))/25 + 305/4) - cos(theta)*sin(psi)*((261*cos(Theta1_BL))/4 + (5617*cos(Theta1_BL)*cos(Theta2_BL))/25 + 279*cos(Theta1_BL)*cos(Theta2_BL)*cos(Theta3_BL) - 279*cos(Theta1_BL)*sin(Theta2_BL)*sin(Theta3_BL) + 5331/100)];
    
    dFdX_FR = [ -((27900*cos(Theta2_FR + Theta3_FR) + 22468*cos(Theta2_FR) + 6525)*(cos(Theta1_FR)*cos(phi)*cos(theta) - sin(Theta1_FR)*cos(psi)*sin(phi) + sin(Theta1_FR)*cos(phi)*sin(psi)*sin(theta)))/100, (279*cos(Theta2_FR + Theta3_FR) + (5617*cos(Theta2_FR))/25)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + (cos(Theta1_FR)*(6975*sin(Theta2_FR + Theta3_FR) + 5617*sin(Theta2_FR))*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)))/25 + (sin(Theta1_FR)*cos(phi)*cos(theta)*(6975*sin(Theta2_FR + Theta3_FR) + 5617*sin(Theta2_FR)))/25, 279*cos(Theta2_FR + Theta3_FR)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + 279*sin(Theta2_FR + Theta3_FR)*cos(Theta1_FR)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + 279*sin(Theta2_FR + Theta3_FR)*sin(Theta1_FR)*cos(phi)*cos(theta)
        -((27900*cos(Theta2_FR + Theta3_FR) + 22468*cos(Theta2_FR) + 6525)*(sin(Theta1_FR)*cos(phi)*cos(psi) + cos(Theta1_FR)*cos(theta)*sin(phi) + sin(Theta1_FR)*sin(phi)*sin(psi)*sin(theta)))/100, (sin(Theta1_FR)*cos(theta)*sin(phi)*(6975*sin(Theta2_FR + Theta3_FR) + 5617*sin(Theta2_FR)))/25 - (cos(Theta1_FR)*(6975*sin(Theta2_FR + Theta3_FR) + 5617*sin(Theta2_FR))*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)))/25 - (279*cos(Theta2_FR + Theta3_FR) + (5617*cos(Theta2_FR))/25)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)), 279*sin(Theta2_FR + Theta3_FR)*sin(Theta1_FR)*cos(theta)*sin(phi) - 279*sin(Theta2_FR + Theta3_FR)*cos(Theta1_FR)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - 279*cos(Theta2_FR + Theta3_FR)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))
        ((cos(Theta1_FR)*sin(theta) - sin(Theta1_FR)*cos(theta)*sin(psi))*(27900*cos(Theta2_FR + Theta3_FR) + 22468*cos(Theta2_FR) + 6525))/100,                                                                        cos(psi)*cos(theta)*(279*cos(Theta2_FR + Theta3_FR) + (5617*cos(Theta2_FR))/25) - (sin(Theta1_FR)*sin(theta)*(6975*sin(Theta2_FR + Theta3_FR) + 5617*sin(Theta2_FR)))/25 - (cos(Theta1_FR)*cos(theta)*sin(psi)*(6975*sin(Theta2_FR + Theta3_FR) + 5617*sin(Theta2_FR)))/25,                                                                        279*cos(Theta2_FR + Theta3_FR)*cos(psi)*cos(theta) - 279*sin(Theta2_FR + Theta3_FR)*sin(Theta1_FR)*sin(theta) - 279*sin(Theta2_FR + Theta3_FR)*cos(Theta1_FR)*cos(theta)*sin(psi)];
    
    dFdX_FL = [ ((27900*cos(Theta2_FL + Theta3_FL) + 22468*cos(Theta2_FL) + 6525)*(cos(Theta1_FL)*cos(phi)*cos(theta) - sin(Theta1_FL)*cos(psi)*sin(phi) + sin(Theta1_FL)*cos(phi)*sin(psi)*sin(theta)))/100, - (279*cos(Theta2_FL + Theta3_FL) + (5617*cos(Theta2_FL))/25)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - (cos(Theta1_FL)*(6975*sin(Theta2_FL + Theta3_FL) + 5617*sin(Theta2_FL))*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)))/25 - (sin(Theta1_FL)*cos(phi)*cos(theta)*(6975*sin(Theta2_FL + Theta3_FL) + 5617*sin(Theta2_FL)))/25, - 279*cos(Theta2_FL + Theta3_FL)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - 279*sin(Theta2_FL + Theta3_FL)*cos(Theta1_FL)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - 279*sin(Theta2_FL + Theta3_FL)*sin(Theta1_FL)*cos(phi)*cos(theta)
        ((27900*cos(Theta2_FL + Theta3_FL) + 22468*cos(Theta2_FL) + 6525)*(sin(Theta1_FL)*cos(phi)*cos(psi) + cos(Theta1_FL)*cos(theta)*sin(phi) + sin(Theta1_FL)*sin(phi)*sin(psi)*sin(theta)))/100,   (279*cos(Theta2_FL + Theta3_FL) + (5617*cos(Theta2_FL))/25)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + (cos(Theta1_FL)*(6975*sin(Theta2_FL + Theta3_FL) + 5617*sin(Theta2_FL))*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)))/25 - (sin(Theta1_FL)*cos(theta)*sin(phi)*(6975*sin(Theta2_FL + Theta3_FL) + 5617*sin(Theta2_FL)))/25,   279*cos(Theta2_FL + Theta3_FL)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + 279*sin(Theta2_FL + Theta3_FL)*cos(Theta1_FL)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - 279*sin(Theta2_FL + Theta3_FL)*sin(Theta1_FL)*cos(theta)*sin(phi)
        -((cos(Theta1_FL)*sin(theta) - sin(Theta1_FL)*cos(theta)*sin(psi))*(27900*cos(Theta2_FL + Theta3_FL) + 22468*cos(Theta2_FL) + 6525))/100,                                                                          (sin(Theta1_FL)*sin(theta)*(6975*sin(Theta2_FL + Theta3_FL) + 5617*sin(Theta2_FL)))/25 - cos(psi)*cos(theta)*(279*cos(Theta2_FL + Theta3_FL) + (5617*cos(Theta2_FL))/25) + (cos(Theta1_FL)*cos(theta)*sin(psi)*(6975*sin(Theta2_FL + Theta3_FL) + 5617*sin(Theta2_FL)))/25,                                                                          279*sin(Theta2_FL + Theta3_FL)*sin(Theta1_FL)*sin(theta) - 279*cos(Theta2_FL + Theta3_FL)*cos(psi)*cos(theta) + 279*sin(Theta2_FL + Theta3_FL)*cos(Theta1_FL)*cos(theta)*sin(psi)];
    
    dFdX_BR = [ -((27900*cos(Theta2_BR + Theta3_BR) + 22468*cos(Theta2_BR) + 6525)*(cos(Theta1_BR)*cos(phi)*cos(theta) - sin(Theta1_BR)*cos(psi)*sin(phi) + sin(Theta1_BR)*cos(phi)*sin(psi)*sin(theta)))/100, (279*cos(Theta2_BR + Theta3_BR) + (5617*cos(Theta2_BR))/25)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + (cos(Theta1_BR)*(6975*sin(Theta2_BR + Theta3_BR) + 5617*sin(Theta2_BR))*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)))/25 + (sin(Theta1_BR)*cos(phi)*cos(theta)*(6975*sin(Theta2_BR + Theta3_BR) + 5617*sin(Theta2_BR)))/25, 279*cos(Theta2_BR + Theta3_BR)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + 279*sin(Theta2_BR + Theta3_BR)*cos(Theta1_BR)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + 279*sin(Theta2_BR + Theta3_BR)*sin(Theta1_BR)*cos(phi)*cos(theta)
        -((27900*cos(Theta2_BR + Theta3_BR) + 22468*cos(Theta2_BR) + 6525)*(sin(Theta1_BR)*cos(phi)*cos(psi) + cos(Theta1_BR)*cos(theta)*sin(phi) + sin(Theta1_BR)*sin(phi)*sin(psi)*sin(theta)))/100, (sin(Theta1_BR)*cos(theta)*sin(phi)*(6975*sin(Theta2_BR + Theta3_BR) + 5617*sin(Theta2_BR)))/25 - (cos(Theta1_BR)*(6975*sin(Theta2_BR + Theta3_BR) + 5617*sin(Theta2_BR))*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)))/25 - (279*cos(Theta2_BR + Theta3_BR) + (5617*cos(Theta2_BR))/25)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)), 279*sin(Theta2_BR + Theta3_BR)*sin(Theta1_BR)*cos(theta)*sin(phi) - 279*sin(Theta2_BR + Theta3_BR)*cos(Theta1_BR)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - 279*cos(Theta2_BR + Theta3_BR)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))
        ((cos(Theta1_BR)*sin(theta) - sin(Theta1_BR)*cos(theta)*sin(psi))*(27900*cos(Theta2_BR + Theta3_BR) + 22468*cos(Theta2_BR) + 6525))/100,                                                                        cos(psi)*cos(theta)*(279*cos(Theta2_BR + Theta3_BR) + (5617*cos(Theta2_BR))/25) - (sin(Theta1_BR)*sin(theta)*(6975*sin(Theta2_BR + Theta3_BR) + 5617*sin(Theta2_BR)))/25 - (cos(Theta1_BR)*cos(theta)*sin(psi)*(6975*sin(Theta2_BR + Theta3_BR) + 5617*sin(Theta2_BR)))/25,                                                                        279*cos(Theta2_BR + Theta3_BR)*cos(psi)*cos(theta) - 279*sin(Theta2_BR + Theta3_BR)*sin(Theta1_BR)*sin(theta) - 279*sin(Theta2_BR + Theta3_BR)*cos(Theta1_BR)*cos(theta)*sin(psi)];
    
    dFdX_BL = [ ((27900*cos(Theta2_BL + Theta3_BL) + 22468*cos(Theta2_BL) + 6525)*(cos(Theta1_BL)*cos(phi)*cos(theta) - sin(Theta1_BL)*cos(psi)*sin(phi) + sin(Theta1_BL)*cos(phi)*sin(psi)*sin(theta)))/100, - (279*cos(Theta2_BL + Theta3_BL) + (5617*cos(Theta2_BL))/25)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - (cos(Theta1_BL)*(6975*sin(Theta2_BL + Theta3_BL) + 5617*sin(Theta2_BL))*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)))/25 - (sin(Theta1_BL)*cos(phi)*cos(theta)*(6975*sin(Theta2_BL + Theta3_BL) + 5617*sin(Theta2_BL)))/25, - 279*cos(Theta2_BL + Theta3_BL)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - 279*sin(Theta2_BL + Theta3_BL)*cos(Theta1_BL)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - 279*sin(Theta2_BL + Theta3_BL)*sin(Theta1_BL)*cos(phi)*cos(theta)
        ((27900*cos(Theta2_BL + Theta3_BL) + 22468*cos(Theta2_BL) + 6525)*(sin(Theta1_BL)*cos(phi)*cos(psi) + cos(Theta1_BL)*cos(theta)*sin(phi) + sin(Theta1_BL)*sin(phi)*sin(psi)*sin(theta)))/100,   (279*cos(Theta2_BL + Theta3_BL) + (5617*cos(Theta2_BL))/25)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + (cos(Theta1_BL)*(6975*sin(Theta2_BL + Theta3_BL) + 5617*sin(Theta2_BL))*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)))/25 - (sin(Theta1_BL)*cos(theta)*sin(phi)*(6975*sin(Theta2_BL + Theta3_BL) + 5617*sin(Theta2_BL)))/25,   279*cos(Theta2_BL + Theta3_BL)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + 279*sin(Theta2_BL + Theta3_BL)*cos(Theta1_BL)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - 279*sin(Theta2_BL + Theta3_BL)*sin(Theta1_BL)*cos(theta)*sin(phi)
        -((cos(Theta1_BL)*sin(theta) - sin(Theta1_BL)*cos(theta)*sin(psi))*(27900*cos(Theta2_BL + Theta3_BL) + 22468*cos(Theta2_BL) + 6525))/100,                                                                          (sin(Theta1_BL)*sin(theta)*(6975*sin(Theta2_BL + Theta3_BL) + 5617*sin(Theta2_BL)))/25 - cos(psi)*cos(theta)*(279*cos(Theta2_BL + Theta3_BL) + (5617*cos(Theta2_BL))/25) + (cos(Theta1_BL)*cos(theta)*sin(psi)*(6975*sin(Theta2_BL + Theta3_BL) + 5617*sin(Theta2_BL)))/25,                                                                          279*sin(Theta2_BL + Theta3_BL)*sin(Theta1_BL)*sin(theta) - 279*cos(Theta2_BL + Theta3_BL)*cos(psi)*cos(theta) + 279*sin(Theta2_BL + Theta3_BL)*cos(Theta1_BL)*cos(theta)*sin(psi)];
    
    
    Xplus_FR = X_FR - dFdX_FR\FX_FR;
    Xplus_FL = X_FL - dFdX_FL\FX_FL;
    Xplus_BR = X_BR - dFdX_BR\FX_BR;
    Xplus_BL = X_BL - dFdX_BL\FX_BL;
    
    
    Theta1_FR = Xplus_FR(1);
    Theta2_FR = Xplus_FR(2);
    Theta3_FR = Xplus_FR(3);
    
    Theta1_FL = Xplus_FL(1);
    Theta2_FL = Xplus_FL(2);
    Theta3_FL = Xplus_FL(3);
    
    Theta1_BR = Xplus_BR(1);
    Theta2_BR = Xplus_BR(2);
    Theta3_BR = Xplus_BR(3);
    
    Theta1_BL = Xplus_BL(1);
    Theta2_BL = Xplus_BL(2);
    Theta3_BL = Xplus_BL(3);
    
    FX_FR_Plus = [(767*cos(phi)*cos(theta))/5 - r_II_B_d_x + (5331*cos(psi)*sin(phi))/100 + (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*((5617*sin(Theta2_FR))/25 + 279*cos(Theta2_FR)*sin(Theta3_FR) + 279*cos(Theta3_FR)*sin(Theta2_FR) - 305/4) + (305*sin(phi)*sin(psi))/4 - (cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*((261*cos(Theta1_FR))/4 + (5617*cos(Theta1_FR)*cos(Theta2_FR))/25 + 279*cos(Theta1_FR)*cos(Theta2_FR)*cos(Theta3_FR) - 279*cos(Theta1_FR)*sin(Theta2_FR)*sin(Theta3_FR) + 5331/100) + (261*cos(Theta1_FR)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)))/4 - (5617*sin(Theta2_FR)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)))/25 + (5617*cos(Theta2_FR)*(cos(Theta1_FR)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + sin(Theta1_FR)*cos(phi)*cos(theta)))/25 - 279*cos(Theta3_FR)*(sin(Theta2_FR)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - cos(Theta2_FR)*(cos(Theta1_FR)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + sin(Theta1_FR)*cos(phi)*cos(theta))) - 279*sin(Theta3_FR)*(cos(Theta2_FR)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + sin(Theta2_FR)*(cos(Theta1_FR)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + sin(Theta1_FR)*cos(phi)*cos(theta))) - cos(phi)*cos(theta)*((261*sin(Theta1_FR))/4 + (5617*cos(Theta2_FR)*sin(Theta1_FR))/25 + 279*cos(Theta2_FR)*cos(Theta3_FR)*sin(Theta1_FR) - 279*sin(Theta1_FR)*sin(Theta2_FR)*sin(Theta3_FR) + 767/5) + (261*sin(Theta1_FR)*cos(phi)*cos(theta))/4 + (305*cos(phi)*cos(psi)*sin(theta))/4 - (5331*cos(phi)*sin(psi)*sin(theta))/100
        279*sin(Theta3_FR)*(cos(Theta2_FR)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + sin(Theta2_FR)*(cos(Theta1_FR)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - sin(Theta1_FR)*cos(theta)*sin(phi))) - (5331*cos(phi)*cos(psi))/100 - r_II_B_d_y - (305*cos(phi)*sin(psi))/4 - (cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*((5617*sin(Theta2_FR))/25 + 279*cos(Theta2_FR)*sin(Theta3_FR) + 279*cos(Theta3_FR)*sin(Theta2_FR) - 305/4) + (767*cos(theta)*sin(phi))/5 + (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*((261*cos(Theta1_FR))/4 + (5617*cos(Theta1_FR)*cos(Theta2_FR))/25 + 279*cos(Theta1_FR)*cos(Theta2_FR)*cos(Theta3_FR) - 279*cos(Theta1_FR)*sin(Theta2_FR)*sin(Theta3_FR) + 5331/100) - (261*cos(Theta1_FR)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)))/4 + (5617*sin(Theta2_FR)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)))/25 - (5617*cos(Theta2_FR)*(cos(Theta1_FR)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - sin(Theta1_FR)*cos(theta)*sin(phi)))/25 + 279*cos(Theta3_FR)*(sin(Theta2_FR)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) - cos(Theta2_FR)*(cos(Theta1_FR)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - sin(Theta1_FR)*cos(theta)*sin(phi))) - (5331*sin(phi)*sin(psi)*sin(theta))/100 - cos(theta)*sin(phi)*((261*sin(Theta1_FR))/4 + (5617*cos(Theta2_FR)*sin(Theta1_FR))/25 + 279*cos(Theta2_FR)*cos(Theta3_FR)*sin(Theta1_FR) - 279*sin(Theta1_FR)*sin(Theta2_FR)*sin(Theta3_FR) + 767/5) + (261*sin(Theta1_FR)*cos(theta)*sin(phi))/4 + (305*cos(psi)*sin(phi)*sin(theta))/4
        (305*cos(psi)*cos(theta))/4 - (767*sin(theta))/5 - r_II_B_d_z - (261*sin(Theta1_FR)*sin(theta))/4 - (5331*cos(theta)*sin(psi))/100 - (5617*cos(Theta2_FR)*(sin(Theta1_FR)*sin(theta) + cos(Theta1_FR)*cos(theta)*sin(psi)))/25 - 279*cos(Theta3_FR)*(cos(Theta2_FR)*(sin(Theta1_FR)*sin(theta) + cos(Theta1_FR)*cos(theta)*sin(psi)) + sin(Theta2_FR)*cos(psi)*cos(theta)) + 279*sin(Theta3_FR)*(sin(Theta2_FR)*(sin(Theta1_FR)*sin(theta) + cos(Theta1_FR)*cos(theta)*sin(psi)) - cos(Theta2_FR)*cos(psi)*cos(theta)) + sin(theta)*((261*sin(Theta1_FR))/4 + (5617*cos(Theta2_FR)*sin(Theta1_FR))/25 + 279*cos(Theta2_FR)*cos(Theta3_FR)*sin(Theta1_FR) - 279*sin(Theta1_FR)*sin(Theta2_FR)*sin(Theta3_FR) + 767/5) + cos(psi)*cos(theta)*((5617*sin(Theta2_FR))/25 + 279*cos(Theta2_FR)*sin(Theta3_FR) + 279*cos(Theta3_FR)*sin(Theta2_FR) - 305/4) + cos(theta)*sin(psi)*((261*cos(Theta1_FR))/4 + (5617*cos(Theta1_FR)*cos(Theta2_FR))/25 + 279*cos(Theta1_FR)*cos(Theta2_FR)*cos(Theta3_FR) - 279*cos(Theta1_FR)*sin(Theta2_FR)*sin(Theta3_FR) + 5331/100) - (261*cos(Theta1_FR)*cos(theta)*sin(psi))/4 - (5617*sin(Theta2_FR)*cos(psi)*cos(theta))/25];
    
    FX_FL_Plus = [(767*cos(phi)*cos(theta))/5 - r_II_B_d_x - (5331*cos(psi)*sin(phi))/100 - (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*((5617*sin(Theta2_FL))/25 + 279*cos(Theta2_FL)*sin(Theta3_FL) + 279*cos(Theta3_FL)*sin(Theta2_FL) + 305/4) + (305*sin(phi)*sin(psi))/4 + (cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*((261*cos(Theta1_FL))/4 + (5617*cos(Theta1_FL)*cos(Theta2_FL))/25 + 279*cos(Theta1_FL)*cos(Theta2_FL)*cos(Theta3_FL) - 279*cos(Theta1_FL)*sin(Theta2_FL)*sin(Theta3_FL) + 5331/100) - (261*cos(Theta1_FL)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)))/4 + (5617*sin(Theta2_FL)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)))/25 - (5617*cos(Theta2_FL)*(cos(Theta1_FL)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + sin(Theta1_FL)*cos(phi)*cos(theta)))/25 + 279*cos(Theta3_FL)*(sin(Theta2_FL)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - cos(Theta2_FL)*(cos(Theta1_FL)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + sin(Theta1_FL)*cos(phi)*cos(theta))) + 279*sin(Theta3_FL)*(cos(Theta2_FL)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + sin(Theta2_FL)*(cos(Theta1_FL)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + sin(Theta1_FL)*cos(phi)*cos(theta))) + cos(phi)*cos(theta)*((261*sin(Theta1_FL))/4 + (5617*cos(Theta2_FL)*sin(Theta1_FL))/25 + 279*cos(Theta2_FL)*cos(Theta3_FL)*sin(Theta1_FL) - 279*sin(Theta1_FL)*sin(Theta2_FL)*sin(Theta3_FL) - 767/5) - (261*sin(Theta1_FL)*cos(phi)*cos(theta))/4 + (305*cos(phi)*cos(psi)*sin(theta))/4 + (5331*cos(phi)*sin(psi)*sin(theta))/100
        (5331*cos(phi)*cos(psi))/100 - r_II_B_d_y - 279*sin(Theta3_FL)*(cos(Theta2_FL)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + sin(Theta2_FL)*(cos(Theta1_FL)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - sin(Theta1_FL)*cos(theta)*sin(phi))) - (305*cos(phi)*sin(psi))/4 + (cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*((5617*sin(Theta2_FL))/25 + 279*cos(Theta2_FL)*sin(Theta3_FL) + 279*cos(Theta3_FL)*sin(Theta2_FL) + 305/4) + (767*cos(theta)*sin(phi))/5 - (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*((261*cos(Theta1_FL))/4 + (5617*cos(Theta1_FL)*cos(Theta2_FL))/25 + 279*cos(Theta1_FL)*cos(Theta2_FL)*cos(Theta3_FL) - 279*cos(Theta1_FL)*sin(Theta2_FL)*sin(Theta3_FL) + 5331/100) + (261*cos(Theta1_FL)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)))/4 - (5617*sin(Theta2_FL)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)))/25 + (5617*cos(Theta2_FL)*(cos(Theta1_FL)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - sin(Theta1_FL)*cos(theta)*sin(phi)))/25 - 279*cos(Theta3_FL)*(sin(Theta2_FL)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) - cos(Theta2_FL)*(cos(Theta1_FL)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - sin(Theta1_FL)*cos(theta)*sin(phi))) + (5331*sin(phi)*sin(psi)*sin(theta))/100 + cos(theta)*sin(phi)*((261*sin(Theta1_FL))/4 + (5617*cos(Theta2_FL)*sin(Theta1_FL))/25 + 279*cos(Theta2_FL)*cos(Theta3_FL)*sin(Theta1_FL) - 279*sin(Theta1_FL)*sin(Theta2_FL)*sin(Theta3_FL) - 767/5) - (261*sin(Theta1_FL)*cos(theta)*sin(phi))/4 + (305*cos(psi)*sin(phi)*sin(theta))/4
        (305*cos(psi)*cos(theta))/4 - (767*sin(theta))/5 - r_II_B_d_z + (261*sin(Theta1_FL)*sin(theta))/4 + (5331*cos(theta)*sin(psi))/100 + (5617*cos(Theta2_FL)*(sin(Theta1_FL)*sin(theta) + cos(Theta1_FL)*cos(theta)*sin(psi)))/25 + 279*cos(Theta3_FL)*(cos(Theta2_FL)*(sin(Theta1_FL)*sin(theta) + cos(Theta1_FL)*cos(theta)*sin(psi)) + sin(Theta2_FL)*cos(psi)*cos(theta)) - 279*sin(Theta3_FL)*(sin(Theta2_FL)*(sin(Theta1_FL)*sin(theta) + cos(Theta1_FL)*cos(theta)*sin(psi)) - cos(Theta2_FL)*cos(psi)*cos(theta)) - sin(theta)*((261*sin(Theta1_FL))/4 + (5617*cos(Theta2_FL)*sin(Theta1_FL))/25 + 279*cos(Theta2_FL)*cos(Theta3_FL)*sin(Theta1_FL) - 279*sin(Theta1_FL)*sin(Theta2_FL)*sin(Theta3_FL) - 767/5) - cos(psi)*cos(theta)*((5617*sin(Theta2_FL))/25 + 279*cos(Theta2_FL)*sin(Theta3_FL) + 279*cos(Theta3_FL)*sin(Theta2_FL) + 305/4) - cos(theta)*sin(psi)*((261*cos(Theta1_FL))/4 + (5617*cos(Theta1_FL)*cos(Theta2_FL))/25 + 279*cos(Theta1_FL)*cos(Theta2_FL)*cos(Theta3_FL) - 279*cos(Theta1_FL)*sin(Theta2_FL)*sin(Theta3_FL) + 5331/100) + (261*cos(Theta1_FL)*cos(theta)*sin(psi))/4 + (5617*sin(Theta2_FL)*cos(psi)*cos(theta))/25];
    
    FX_BR_Plus = [(5331*cos(psi)*sin(phi))/100 - (767*cos(phi)*cos(theta))/5 - r_II_B_d_x + (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*((5617*sin(Theta2_BR))/25 + 279*cos(Theta2_BR)*sin(Theta3_BR) + 279*cos(Theta3_BR)*sin(Theta2_BR) - 305/4) + (305*sin(phi)*sin(psi))/4 - (cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*((261*cos(Theta1_BR))/4 + (5617*cos(Theta1_BR)*cos(Theta2_BR))/25 + 279*cos(Theta1_BR)*cos(Theta2_BR)*cos(Theta3_BR) - 279*cos(Theta1_BR)*sin(Theta2_BR)*sin(Theta3_BR) + 5331/100) + (261*cos(Theta1_BR)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)))/4 - (5617*sin(Theta2_BR)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)))/25 + (5617*cos(Theta2_BR)*(cos(Theta1_BR)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + sin(Theta1_BR)*cos(phi)*cos(theta)))/25 - 279*cos(Theta3_BR)*(sin(Theta2_BR)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - cos(Theta2_BR)*(cos(Theta1_BR)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + sin(Theta1_BR)*cos(phi)*cos(theta))) - 279*sin(Theta3_BR)*(cos(Theta2_BR)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + sin(Theta2_BR)*(cos(Theta1_BR)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + sin(Theta1_BR)*cos(phi)*cos(theta))) - cos(phi)*cos(theta)*((261*sin(Theta1_BR))/4 + (5617*cos(Theta2_BR)*sin(Theta1_BR))/25 + 279*cos(Theta2_BR)*cos(Theta3_BR)*sin(Theta1_BR) - 279*sin(Theta1_BR)*sin(Theta2_BR)*sin(Theta3_BR) - 767/5) + (261*sin(Theta1_BR)*cos(phi)*cos(theta))/4 + (305*cos(phi)*cos(psi)*sin(theta))/4 - (5331*cos(phi)*sin(psi)*sin(theta))/100
        279*sin(Theta3_BR)*(cos(Theta2_BR)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + sin(Theta2_BR)*(cos(Theta1_BR)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - sin(Theta1_BR)*cos(theta)*sin(phi))) - (5331*cos(phi)*cos(psi))/100 - r_II_B_d_y - (305*cos(phi)*sin(psi))/4 - (cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*((5617*sin(Theta2_BR))/25 + 279*cos(Theta2_BR)*sin(Theta3_BR) + 279*cos(Theta3_BR)*sin(Theta2_BR) - 305/4) - (767*cos(theta)*sin(phi))/5 + (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*((261*cos(Theta1_BR))/4 + (5617*cos(Theta1_BR)*cos(Theta2_BR))/25 + 279*cos(Theta1_BR)*cos(Theta2_BR)*cos(Theta3_BR) - 279*cos(Theta1_BR)*sin(Theta2_BR)*sin(Theta3_BR) + 5331/100) - (261*cos(Theta1_BR)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)))/4 + (5617*sin(Theta2_BR)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)))/25 - (5617*cos(Theta2_BR)*(cos(Theta1_BR)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - sin(Theta1_BR)*cos(theta)*sin(phi)))/25 + 279*cos(Theta3_BR)*(sin(Theta2_BR)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) - cos(Theta2_BR)*(cos(Theta1_BR)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - sin(Theta1_BR)*cos(theta)*sin(phi))) - (5331*sin(phi)*sin(psi)*sin(theta))/100 - cos(theta)*sin(phi)*((261*sin(Theta1_BR))/4 + (5617*cos(Theta2_BR)*sin(Theta1_BR))/25 + 279*cos(Theta2_BR)*cos(Theta3_BR)*sin(Theta1_BR) - 279*sin(Theta1_BR)*sin(Theta2_BR)*sin(Theta3_BR) - 767/5) + (261*sin(Theta1_BR)*cos(theta)*sin(phi))/4 + (305*cos(psi)*sin(phi)*sin(theta))/4
        (767*sin(theta))/5 - r_II_B_d_z + (305*cos(psi)*cos(theta))/4 - (261*sin(Theta1_BR)*sin(theta))/4 - (5331*cos(theta)*sin(psi))/100 - (5617*cos(Theta2_BR)*(sin(Theta1_BR)*sin(theta) + cos(Theta1_BR)*cos(theta)*sin(psi)))/25 - 279*cos(Theta3_BR)*(cos(Theta2_BR)*(sin(Theta1_BR)*sin(theta) + cos(Theta1_BR)*cos(theta)*sin(psi)) + sin(Theta2_BR)*cos(psi)*cos(theta)) + 279*sin(Theta3_BR)*(sin(Theta2_BR)*(sin(Theta1_BR)*sin(theta) + cos(Theta1_BR)*cos(theta)*sin(psi)) - cos(Theta2_BR)*cos(psi)*cos(theta)) + sin(theta)*((261*sin(Theta1_BR))/4 + (5617*cos(Theta2_BR)*sin(Theta1_BR))/25 + 279*cos(Theta2_BR)*cos(Theta3_BR)*sin(Theta1_BR) - 279*sin(Theta1_BR)*sin(Theta2_BR)*sin(Theta3_BR) - 767/5) + cos(psi)*cos(theta)*((5617*sin(Theta2_BR))/25 + 279*cos(Theta2_BR)*sin(Theta3_BR) + 279*cos(Theta3_BR)*sin(Theta2_BR) - 305/4) + cos(theta)*sin(psi)*((261*cos(Theta1_BR))/4 + (5617*cos(Theta1_BR)*cos(Theta2_BR))/25 + 279*cos(Theta1_BR)*cos(Theta2_BR)*cos(Theta3_BR) - 279*cos(Theta1_BR)*sin(Theta2_BR)*sin(Theta3_BR) + 5331/100) - (261*cos(Theta1_BR)*cos(theta)*sin(psi))/4 - (5617*sin(Theta2_BR)*cos(psi)*cos(theta))/25];
    
    FX_BL_Plus = [(305*sin(phi)*sin(psi))/4 - (767*cos(phi)*cos(theta))/5 - (5331*cos(psi)*sin(phi))/100 - (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*((5617*sin(Theta2_BL))/25 + 279*cos(Theta2_BL)*sin(Theta3_BL) + 279*cos(Theta3_BL)*sin(Theta2_BL) + 305/4) - r_II_B_d_x + (cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*((261*cos(Theta1_BL))/4 + (5617*cos(Theta1_BL)*cos(Theta2_BL))/25 + 279*cos(Theta1_BL)*cos(Theta2_BL)*cos(Theta3_BL) - 279*cos(Theta1_BL)*sin(Theta2_BL)*sin(Theta3_BL) + 5331/100) - (261*cos(Theta1_BL)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)))/4 + (5617*sin(Theta2_BL)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)))/25 - (5617*cos(Theta2_BL)*(cos(Theta1_BL)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + sin(Theta1_BL)*cos(phi)*cos(theta)))/25 + 279*cos(Theta3_BL)*(sin(Theta2_BL)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - cos(Theta2_BL)*(cos(Theta1_BL)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + sin(Theta1_BL)*cos(phi)*cos(theta))) + 279*sin(Theta3_BL)*(cos(Theta2_BL)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + sin(Theta2_BL)*(cos(Theta1_BL)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + sin(Theta1_BL)*cos(phi)*cos(theta))) + cos(phi)*cos(theta)*((261*sin(Theta1_BL))/4 + (5617*cos(Theta2_BL)*sin(Theta1_BL))/25 + 279*cos(Theta2_BL)*cos(Theta3_BL)*sin(Theta1_BL) - 279*sin(Theta1_BL)*sin(Theta2_BL)*sin(Theta3_BL) + 767/5) - (261*sin(Theta1_BL)*cos(phi)*cos(theta))/4 + (305*cos(phi)*cos(psi)*sin(theta))/4 + (5331*cos(phi)*sin(psi)*sin(theta))/100
        (5331*cos(phi)*cos(psi))/100 - r_II_B_d_y - 279*sin(Theta3_BL)*(cos(Theta2_BL)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + sin(Theta2_BL)*(cos(Theta1_BL)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - sin(Theta1_BL)*cos(theta)*sin(phi))) - (305*cos(phi)*sin(psi))/4 + (cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*((5617*sin(Theta2_BL))/25 + 279*cos(Theta2_BL)*sin(Theta3_BL) + 279*cos(Theta3_BL)*sin(Theta2_BL) + 305/4) - (767*cos(theta)*sin(phi))/5 - (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*((261*cos(Theta1_BL))/4 + (5617*cos(Theta1_BL)*cos(Theta2_BL))/25 + 279*cos(Theta1_BL)*cos(Theta2_BL)*cos(Theta3_BL) - 279*cos(Theta1_BL)*sin(Theta2_BL)*sin(Theta3_BL) + 5331/100) + (261*cos(Theta1_BL)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)))/4 - (5617*sin(Theta2_BL)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)))/25 + (5617*cos(Theta2_BL)*(cos(Theta1_BL)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - sin(Theta1_BL)*cos(theta)*sin(phi)))/25 - 279*cos(Theta3_BL)*(sin(Theta2_BL)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) - cos(Theta2_BL)*(cos(Theta1_BL)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - sin(Theta1_BL)*cos(theta)*sin(phi))) + (5331*sin(phi)*sin(psi)*sin(theta))/100 + cos(theta)*sin(phi)*((261*sin(Theta1_BL))/4 + (5617*cos(Theta2_BL)*sin(Theta1_BL))/25 + 279*cos(Theta2_BL)*cos(Theta3_BL)*sin(Theta1_BL) - 279*sin(Theta1_BL)*sin(Theta2_BL)*sin(Theta3_BL) + 767/5) - (261*sin(Theta1_BL)*cos(theta)*sin(phi))/4 + (305*cos(psi)*sin(phi)*sin(theta))/4
        (767*sin(theta))/5 - r_II_B_d_z + (305*cos(psi)*cos(theta))/4 + (261*sin(Theta1_BL)*sin(theta))/4 + (5331*cos(theta)*sin(psi))/100 + (5617*cos(Theta2_BL)*(sin(Theta1_BL)*sin(theta) + cos(Theta1_BL)*cos(theta)*sin(psi)))/25 + 279*cos(Theta3_BL)*(cos(Theta2_BL)*(sin(Theta1_BL)*sin(theta) + cos(Theta1_BL)*cos(theta)*sin(psi)) + sin(Theta2_BL)*cos(psi)*cos(theta)) - 279*sin(Theta3_BL)*(sin(Theta2_BL)*(sin(Theta1_BL)*sin(theta) + cos(Theta1_BL)*cos(theta)*sin(psi)) - cos(Theta2_BL)*cos(psi)*cos(theta)) - sin(theta)*((261*sin(Theta1_BL))/4 + (5617*cos(Theta2_BL)*sin(Theta1_BL))/25 + 279*cos(Theta2_BL)*cos(Theta3_BL)*sin(Theta1_BL) - 279*sin(Theta1_BL)*sin(Theta2_BL)*sin(Theta3_BL) + 767/5) - cos(psi)*cos(theta)*((5617*sin(Theta2_BL))/25 + 279*cos(Theta2_BL)*sin(Theta3_BL) + 279*cos(Theta3_BL)*sin(Theta2_BL) + 305/4) - cos(theta)*sin(psi)*((261*cos(Theta1_BL))/4 + (5617*cos(Theta1_BL)*cos(Theta2_BL))/25 + 279*cos(Theta1_BL)*cos(Theta2_BL)*cos(Theta3_BL) - 279*cos(Theta1_BL)*sin(Theta2_BL)*sin(Theta3_BL) + 5331/100) + (261*cos(Theta1_BL)*cos(theta)*sin(psi))/4 + (5617*sin(Theta2_BL)*cos(psi)*cos(theta))/25];
    
    
    if length(legs) == 4
    FX = [FX_FR, FX_FL, FX_BR, FX_BL];
    FXplus = [FX_FR_Plus, FX_FL_Plus, FX_BR_Plus, FX_BL_Plus];
    elseif legs_on_gnd(1) == 0
    FX = [FX_FL, FX_BR, FX_BL];
    FXplus = [FX_FL_Plus, FX_BR_Plus, FX_BL_Plus];
    elseif legs_on_gnd(2) == 0
    FX = [FX_FR, FX_BR, FX_BL];
    FXplus = [FX_FR_Plus, FX_BR_Plus, FX_BL_Plus];
    elseif legs_on_gnd(3) == 0
    FX = [FX_FL, FX_FR, FX_BL];
    FXplus = [FX_FL_Plus, FX_FR_Plus, FX_BL_Plus];
    elseif legs_on_gnd(4) == 0
    FX = [FX_FL, FX_FR, FX_BR];
    FXplus = [FX_FL_Plus, FX_FR_Plus, FX_BR_Plus];
    end
    
    error = norm(FX - FXplus);
    
    X_FR = Xplus_FR;
    X_FL = Xplus_FL;
    X_BR = Xplus_BR;
    X_BL = Xplus_BL;
    
    Theta1_d = [Theta1_FR, Theta1_FL, Theta1_BR, Theta1_BL];
    Theta2_d = [Theta2_FR, Theta2_FL, Theta2_BR, Theta2_BL];
    Theta3_d = [Theta3_FR, Theta3_FL, Theta3_BR, Theta3_BL];
end