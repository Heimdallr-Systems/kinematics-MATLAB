%RC_Func_Sample  A faster version of RC_Func_Test for codegen testing. 
clear
clc
close all

h = 0.01; % time step
t = 0:h:10; % time vector

%%% Initial Conditions %%%
b=zeros(36,length(t)); % state matrix
b(4,1) = 0;
b(5,1) = 0;
b(6,1) = 0.25; % body height % default 0.18 % 0.3
% first joint angles
b(7,1) = pi/4;
b(8,1) = -pi/4;
b(9,1) = -pi/4;
b(10,1) = pi/4;
% second joint angles
b(11,1) = pi/6+0.1; %pi/4 %0
b(12,1) = -pi/6;
b(13,1) = pi/6;
b(14,1) = -pi/6;
% third joint angles
b(15,1) = pi/3; %pi/8 %pi/2
b(16,1) = -pi/3;
b(17,1) = pi/3;
b(18,1) = -pi/3;

%%% Desired Body Pose Trajectory %%%
% x_d = 1*ones(1,length(t));
% y_d = 0.*ones(1,length(x_d));
% z_d = 0.245*ones(1,length(x_d));
% phi_d = pi/2.*ones(1,length(x_d));
% theta_d =  zeros(1,length(x_d));
% psi_d =  zeros(1,length(x_d));
r_II_B_d = [1;0;0.2];
Euler_d = [pi/2,0,0];
phi_d = pi/2;
theta_d = 0;
psi_d = 0;

% Control Constants
Kd=6;
Kp=30;

%%% Floor Definition %%%
Kp_floor = -5000; % floor spring constant
Kd_floor = -800; % floor damping coefficient
b_fric_floor = -2000; % floor coefficient of lateral, viscous friction

% Numerically Integrate for Position of Manipulator
init_toggle = true;
for ii = 1:length(t)
    
    [Theta1_d,Theta2_d,Theta3_d,phi_d_temp,r_II_B_d_temp,floor_toggle,legs_valid] = Robot_Control(r_II_B_d, Euler_d, b(:,ii), init_toggle);
    init_toggle = false;
    
    %%% Solve for Current State in order to Plot %%%
    phi = b(1,ii);
    theta = b(2,ii);
    psi = b(3,ii);
    T_I_B = rotz(phi)*roty(theta)*rotx(psi);
    r_II_B = [b(4,ii);b(5,ii);b(6,ii)];
    Theta1 = [b(7,ii);b(8,ii);b(9,ii);b(10,ii)];
    Theta2 = [b(11,ii);b(12,ii);b(13,ii);b(14,ii)];
    Theta3 = [b(15,ii);b(16,ii);b(17,ii);b(18,ii)];
    
    %%%KINEMATIC%%%
    b(1,ii+1) = phi_d_temp;
    b(2,ii+1) = theta_d;
    b(3,ii+1) = psi_d;
    
    b(4,ii+1) = r_II_B_d_temp(1);
    b(5,ii+1) = r_II_B_d_temp(2);
    b(6,ii+1) = r_II_B_d_temp(3);

    b(7,ii+1) = Theta1_d(1);
    b(8,ii+1) = Theta1_d(2);
    b(9,ii+1) = Theta1_d(3);
    b(10,ii+1) = Theta1_d(4);
    b(11,ii+1) = Theta2_d(1);
    b(12,ii+1) = Theta2_d(2);
    b(13,ii+1) = Theta2_d(3);
    b(14,ii+1) = Theta2_d(4);
    b(15,ii+1) = Theta3_d(1);
    b(16,ii+1) = Theta3_d(2);
    b(17,ii+1) = Theta3_d(3);
    b(18,ii+1) = Theta3_d(4); 
    disp(ii)
end
