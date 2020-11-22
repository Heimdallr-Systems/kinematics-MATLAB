function [jnt_accel]=motor_dynamics(gamma, d, H,G, theta_d, Fgamma)
% Computes the joint acceleration of a SER0044 using experimentally
% measured values.
% Inputs:
%   jnt_pos: position of the motor, in radians
%   jnt_vel: angular velocity of the motor in radians/second
%   theta_d: Target position of the motor, in radians
%   tau    : torque on the motor, in newton-meters
% Outputs:
%   jnt_accel: Acceleration of each joint, in radians per second squared.
% Convert inputs into degrees
theta_d=theta_d.*180/pi;
gamma=gamma.*180/pi;

% Calculated constants
Hm=0.0491*eye(18);
Kd=2.0781;
Kp=34.1862;
% Screw with these values if the motor is behaving oddly
Rm=48.922*eye(18);
Bm=0.7641*eye(18);

% assuming no joint friction
B = zeros(18,18);
C = zeros(18,18);

jnt_pos(:,1) = gamma(7:18);
jnt_vel(:,1) = gamma(25:36);

% perform the calculation
%jnt_accel=(Kp.*theta_d-damping.*jnt_vel-Kp.*jnt_pos-J_mult.*tau)./Hm.*pi./180;
V(1:6,1) = [0;0;0;0;0;0];
V(7:18,1)= Kp*(theta_d-jnt_pos)-Kd*jnt_vel;

% theta_d_size = size(theta_d)
% jnt_pos_size = size(gamma)
% Fgamma_size = size(Fgamma)
% Hm_size = size(Hm)
% Rm_size = size(Rm)
% H_size = size(H)
% V_size = size(V)
% Bm_size = size(Bm)
% C_size = size(C)
% B_size = size(B)
% d_size = size(d)
% G_size = size(G)
% jnt_pos_size = size(jnt_pos)
% jnt_vel_size = size(jnt_vel)

% a = (Hm+Rm*H);
% gamma(25:36,1)
% size(gamma(25:36,1))
% b = (Bm+Rm*B)*gamma(25:36,1);
% c = (Rm*C)*sign(gamma(25:36,1));
% d = Rm*d-Rm*G+Rm*Fgamma;
jnt_accel=(Hm+Rm*H)\(V-(Bm+Rm*B)*gamma(19:36,1)-(Rm*C)*sign(gamma(19:36,1))-Rm*d+Rm*G+Rm*Fgamma);
Fgamma_net = (Hm+Rm*H)*jnt_accel+(Bm+Rm*B)*gamma(19:36,1)+(Rm*C)*sign(gamma(19:36,1))+Rm*d+Rm*G;
end