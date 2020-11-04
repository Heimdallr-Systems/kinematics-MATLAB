clear
clc
close all

%% Find Joint angles for a desired starting pose
r_II_c_FR = [300;-300;100];
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
BodyRot = [phi,theta,psi];
Euler_0 = BodyRot;
T_I_B_0 = rotz(phi)*roty(theta)*rotx(psi);

% Body Offset from Inertial
r_II_B_0 = [0;0;250];

% IK for legs

% check if at least 3 legs are on ground
legs_on_gnd = [r_II_c_FR(3) == 0, r_II_c_FL(3) == 0, r_II_c_BR(3) == 0, r_II_c_BL(3) == 0];

% Solve for joint angles of legs (IK) assuming a desired or known offset of
% the robot and orientation of the robot is known
[Theta1_0, ~, Theta2_0, ~, Theta3_0] = IK_Solver_Legs_Inertial(r_II_c, T_I_B_0, r_II_B_0);

% draw robot to confirm (FK)
FK_Solver_Draw(Theta1_0, Theta2_0, Theta3_0, T_I_B_0, r_II_B_0);

%% Find joint angles for a desired body position
r_II_B_d = [120;120;250];
[Theta1_d,Theta2_d,Theta3_d] = Joint_Space_Solver(Theta1_0, Theta2_0, Theta3_0, Euler_0, r_II_B_0, r_II_B_d, legs_on_gnd);

FK_Solver_Draw(Theta1_d, Theta2_d, Theta3_d, T_I_B_0, r_II_B_d);
%% WALK!
% Generate body trajectory
dt=0.05;
u=@(t) double(t>=0);

t = 0:dt:2*pi;
r_II_c_start=[300,-300,0;
              300,300,0;
              -300,-300,0;
              -300,300,0]';
r_II_B_d = [200.*sin(t)+120
            (200.*t+120)
            250.*ones(1,length(t))];
% Generate direction vector

z_FR = [0,200,0,0,0];
z_FL = [0,0,200,0,0];
z_BR = [0,0,0,200,0];
z_BL = [0,0,0,0,200];
%for jj = 1:1:1
r_II_c_FR = [300,-300,0;
             500,-300,0;
             700,-300,0;
             900,-300,0;
             1100,-300,0;
             1400,-300,0;
             1600,-300,0;
             1800,-300,0;
             ]';
r_II_c_FL = [300,300,0;
             530,300,0;
             720,300,0;
             950,300,0;
             1110,300,0;
             1420,300,0;
             1656,300,0;
             1880,300,0;
             ]';
r_II_c_BR = [-300,-300,0;
             -100,-300,0;
             100,-300,0;
             300,-300,0;
             500,-300,0;
             800,-300,0;
             1100,-300,0;
             1400,-300,0;
             ]';
r_II_c_BL = [-300,300,0;
             -150,300,0;
             170,300,0;
             320,300,0;
             560,300,0;
             820,300,0;
             1170,300,0;
             1410,300,0;
             ]';
% Current index of r_II_c for each leg
FR_pos_index=1;
FL_pos_index=1;
BR_pos_index=1;
BL_pos_index=1;

dist=150;
%r_II_c = [r_II_c_FR(:,FR_pos_index),r_II_c_FL(:,FL_pos_index), r_II_c_BR(:,BR_pos_index), r_II_c_BL(:,BL_pos_index)];
r_II_c=r_II_c_start;
[Theta1_0, ~, Theta2_0, ~, Theta3_0] = IK_Solver_Legs_Inertial(r_II_c, T_I_B_0, r_II_B_0);

% check if at least 3 legs are on ground
legs_on_gnd = [r_II_c_FR(3,FR_pos_index) == 0, r_II_c_FL(3,FR_pos_index) == 0, r_II_c_BR(3,FR_pos_index) == 0, r_II_c_BL(3,FR_pos_index) == 0];

Theta1 = Theta1_0;
Theta2 = Theta2_0;
Theta3 = Theta3_0;


for ii = 2:1:length(t)
%leg_control_override

[Theta1_d,Theta2_d,Theta3_d] = Joint_Space_Solver(Theta1, Theta2, Theta3, Euler_0, r_II_B_0, r_II_B_d(:,ii), legs_on_gnd);
    [rc_FR, rc_FL, rc_BR, rc_BL]=CPos_wrt_B(Theta1_d,Theta2_d,Theta3_d);
    r_BB_c=[rc_FR,rc_FL,rc_BR,rc_BL];
    legs_valid=check_workspace(r_BB_c);
    if(legs_valid(1)==0) % FR
        %Theta1_d(1)=0;
        FR_pos_index=FR_pos_index+1;
        r_II_c_FR=step_planner(r_II_B_d(:,ii-1),r_II_B_d(:,ii),r_II_c,dist);
        r_II_c = [r_II_c_FR(:,1),r_II_c(:,2), r_II_c(:,3), r_II_c(:,4)];
        [Theta1_0, ~, Theta2_0, ~, Theta3_0] = IK_Solver_Legs_Inertial(r_II_c, T_I_B_0, r_II_B_0);
        Theta3_d(1)=Theta3_0(1);
        Theta2_d(1)=Theta2_0(1);
        Theta1_d(1)=Theta1_0(1);
    end
    if(legs_valid(2)==0) % FL
       % Theta1_d(2)=0;
        FL_pos_index=FL_pos_index+1;
        r_II_c_FL=step_planner(r_II_B_d(:,ii-1),r_II_B_d(:,ii),r_II_c,dist);
        r_II_c = [r_II_c(:,1),r_II_c_FL(:,2), r_II_c(:,3), r_II_c(:,4)];
        %r_II_c = [r_II_c_FR(:,FR_pos_index),r_II_c_FL(:,FL_pos_index), r_II_c_BR(:,BR_pos_index), r_II_c_BL(:,BL_pos_index)];
        [Theta1_0, ~, Theta2_0, ~, Theta3_0] = IK_Solver_Legs_Inertial(r_II_c, T_I_B_0, r_II_B_0);
        Theta3_d(2)=Theta3_0(2);
        Theta2_d(2)=Theta2_0(2);
        Theta1_d(2)=Theta1_0(2);
    end
    if(legs_valid(3)==0) % BR
        %Theta1_d(3)=0;
        %Theta2_d(3)=0;
            BR_pos_index=BR_pos_index+1;
            r_II_c_BL=step_planner(r_II_B_d(:,ii-1),r_II_B_d(:,ii),r_II_c,dist);
            r_II_c = [r_II_c(:,1),r_II_c(:,2), r_II_c_BL(:,3), r_II_c(:,4)];
            %r_II_c = [r_II_c_FR(:,FR_pos_index),r_II_c_FL(:,FL_pos_index), r_II_c_BR(:,BR_pos_index), r_II_c_BL(:,BL_pos_index)];
            [Theta1_0, ~, Theta2_0, ~, Theta3_0] = IK_Solver_Legs_Inertial(r_II_c, T_I_B_0, r_II_B_0);
            Theta3_d(3)=Theta3_0(3);
            Theta2_d(3)=Theta2_0(3);
            Theta1_d(3)=Theta1_0(3);
    end
    if(legs_valid(4)==0) % BL
        %Theta1_d(4)=0;
        BL_pos_index=BL_pos_index+1;
        r_II_c_BL=step_planner(r_II_B_d(:,ii-1),r_II_B_d(:,ii),r_II_c,dist);
        r_II_c = [r_II_c(:,1),r_II_c(:,2), r_II_c(:,3), r_II_c_BL(:,4)];
        %r_II_c = [r_II_c_FR(:,FR_pos_index),r_II_c_FL(:,FL_pos_index), r_II_c_BR(:,BR_pos_index), r_II_c_BL(:,BL_pos_index)];
        [Theta1_0, ~, Theta2_0, ~, Theta3_0] = IK_Solver_Legs_Inertial(r_II_c, T_I_B_0, r_II_B_0);
        Theta3_d(4)=Theta3_0(4);
        Theta2_d(4)=Theta2_0(4);
        Theta1_d(4)=Theta1_0(4);
    end
FK_Solver_Draw(Theta1_d, Theta2_d, Theta3_d, T_I_B_0, r_II_B_d(:,ii));
%pause(.2);
r_II_B_0 = r_II_B_d(:,ii);
Theta1 = Theta1_d;
Theta2 = Theta2_d;
Theta3 = Theta3_d;
fprintf("FR steps: %d. FL steps: %d. BR steps: %d. BL steps: %d.\n",FR_pos_index,FL_pos_index,BR_pos_index,BL_pos_index);
end

pause(1);

%end