clear
clc
close all

%% Find Joint angles for a desired starting pose
r_II_c_FR = [300;-300;0];
r_II_c_FL = [300;300;0];
r_II_c_BR = [-300;-300;0];
r_II_c_BL = [-300;300;0];
r_II_c = [r_II_c_FR,r_II_c_FL, r_II_c_BR, r_II_c_BL];

phi_degrees = 0;  % (z-rotation)
theta_degrees = 0;  % (y-rotation)
psi_degrees = 0;  % (x-rotation)
phi = phi_degrees*(pi/180);
theta = theta_degrees*(pi/180);
psi = psi_degrees*(pi/180);
Euler_0 = [phi,theta,psi];
T_I_B_0 = rotz(phi)*roty(theta)*rotx(psi);
T_I_B_d = T_I_B_0;

% Body Offset from Inertial
r_II_B_0 = [0;0;250];

% IK for legs

% check if at least 3 legs are on ground
legs_on_gnd = [r_II_c_FR(3) == 0, r_II_c_FL(3) == 0, r_II_c_BR(3) == 0, r_II_c_BL(3) == 0];

% Solve for joint angles of legs (IK) assuming a desired or known offset of
% the robot and orientation of the robot is known
[Theta1_0, ~, Theta2_0, ~, Theta3_0] = IK_Solver_Legs_Inertial(r_II_c, T_I_B_0, r_II_B_0);

% draw robot to confirm (FK)
% FK_Solver_Draw_CM(Theta1_0, Theta2_0, Theta3_0, T_I_B_0, r_II_B_0);

%% Find joint angles for a desired body position
Theta1_0 = [0;0;0;0];
Theta2_0 = [0;0;0;0];
Theta3_0 = [pi/2;-pi/2;pi/2;-pi/2];
r_II_B_0 = [0;0;250];
r_II_B_d = [0;0;100];
T_I_B_d = rotz(0)*roty(0)*rotx(0);
[Theta1_d,Theta2_d,Theta3_d] = Joint_Space_Solver(Theta1_0, Theta2_0, Theta3_0, r_II_B_0, r_II_B_d, T_I_B_0, T_I_B_d, legs_on_gnd);
%FK_Solver_Draw_CM(Theta1_d, Theta2_d, Theta3_d, T_I_B_d, r_II_B_d);

%% Dance!
% % Generate body trajectory

phase = 0:0.01:4*pi;

writerObj = VideoWriter('Trajectory Demo','MPEG-4');
writerObj.FrameRate = 60;
open(writerObj);

ax = gca;
ax.NextPlot = 'replaceChildren';
%Preallocate a 40-element array M to store the movie frames.

loops = 1:1:length(phase);
M(loops) = struct('cdata',[],'colormap',[]);
phase = 0:0.01:4*pi;
phase_2 = [0:0.005:2*pi];%0:0.1:pi/2;
for ii = 1:1:length(phase)
    r_II_B_d = [100.*cos(phase(ii));-100.*sin(phase(ii));200];
    %r_II_B_d = [0;0;250];
    T_I_B_d = rotz(0.4*sin(phase_2(ii)))*roty(0.2*sin(phase_2(ii)))*rotx(0.2*sin(phase_2(ii)));
    [Theta1_d,Theta2_d,Theta3_d] = Joint_Space_Solver(Theta1_0, Theta2_0, Theta3_0, r_II_B_0, r_II_B_d, T_I_B_0, T_I_B_d, legs_on_gnd);
    FK_Solver_Draw_CM(Theta1_d, Theta2_d, Theta3_d, T_I_B_d, r_II_B_d);
    r_II_B_0 = r_II_B_d;
    Theta1_0 = Theta1_d;
    Theta2_0 = Theta2_d;
    Theta3_0 = Theta3_d;
    T_I_B_0 = T_I_B_d;
    
    M(ii) = getframe(gcf);
    writeVideo(writerObj,M(ii));
end

close(writerObj);