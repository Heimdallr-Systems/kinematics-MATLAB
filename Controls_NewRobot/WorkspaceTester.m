clear
clc

% x_d_val = [0,0,0.03,-0.03,0,0,0,0,0,0,0,0];
% y_d_val = [0,0,0,0,0.03,-0.03,0,0,0,0,0,0];
% z_d_val = [0.2,0.31,0.2,0.2,0.22,0.22,0.22,0.22,0.22,0.22,0.22,0.22];
% phi_d_val = [0,0,0,0,0,0,pi/14,-pi/14,0,0,0,0];
% theta_d_val =  [0,0,0,0,0,0,0,0,pi/14,-pi/14,0,0];
% psi_d_val =  [0,0,0,0,0,0,0,0,0,0,pi/14,-pi/14];
% 
% x_d = [];
% y_d = [];
% z_d = [];
% phi_d = [];
% theta_d = [];
% psi_d = [];
% 
Theta1(1) = pi/4;
Theta1(2) = -pi/4;
Theta1(3) = -pi/4;
Theta1(4) = pi/4;
% second joint angles
Theta2(1) = 0;
Theta2(2) = 0;
Theta2(3) = 0;
Theta2(4) = 0;
% third joint angles
Theta3(1) = pi/2;
Theta3(2) = -pi/2;
Theta3(3) = pi/2;
Theta3(4) = -pi/2;
% 
r_II_B = [0;0;0.242];
T_I_B = eye(3);
% 
t = 1:1:1;
% 
% for ii = 1:1:t(end)
%     x_d = cat(2,x_d,x_d_val(ii).*ones(1,ceil(length(t)/t(end))));
%     y_d = cat(2,y_d,y_d_val(ii).*ones(1,ceil(length(t)/t(end))));
%     z_d = cat(2,z_d,z_d_val(ii).*ones(1,ceil(length(t)/t(end))));
%     phi_d = cat(2,phi_d,phi_d_val(ii).*ones(1,ceil(length(t)/t(end))));
%     theta_d = cat(2,theta_d,theta_d_val(ii).*ones(1,ceil(length(t)/t(end))));
%     psi_d = cat(2,psi_d,psi_d_val(ii).*ones(1,ceil(length(t)/t(end))));
% end

% FK_Solver_Draw(Theta1,Theta2,Theta3,T_I_B,r_II_B);

x_d = 0.00;
y_d = 0.00;
z_d = 0.21;
phi_d = 0;
theta_d = -pi/16;
psi_d = 0;

for ii = 1:1:length(t)
    T_I_B_d = rotz(phi_d(ii))*roty(theta_d(ii))*rotx(psi_d(ii));
    r_II_B_d(:,1) = [x_d(ii);y_d(ii);z_d(ii)];
    
    [Theta1_d,Theta2_d,Theta3_d] = Joint_Space_Solver(Theta1, Theta2, Theta3, r_II_B, r_II_B_d, T_I_B, T_I_B_d);
    FK_Solver_Draw(Theta1_d,Theta2_d,Theta3_d,T_I_B_d,r_II_B_d);
    pause(0.1)
end