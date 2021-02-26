clear
clc

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

Theta1(:,1) = [b(7,ii);b(8,ii);b(9,ii);b(10,ii)];
Theta2(:,1) = [b(11,ii);b(12,ii);b(13,ii);b(14,ii)];
Theta3(:,1) = [b(15,ii);b(16,ii);b(17,ii);b(18,ii)];