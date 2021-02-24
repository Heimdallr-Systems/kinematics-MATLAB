function FK_Solver_Draw_CM(Theta1,Theta2,Theta3,T_I_B,r_II_B,rcm,legs_valid,view_angle, view_type)
% This program draws the robot based on given joint angles. This function
% is used to test the forward kinematics of this robotic system.

% NOTE: The "Left" side is the side that is in the positive y-direction
% FL = Front Left Leg
% FR = Front Right Leg
% BL = Back Left Leg
% BR = Back Right Leg

% jnt_var is a vector of joint variables arranged in the following manner
% [FR Leg Joint Angles 1-3, FL Leg Joint Angles 1-3
%  BR Leg Joint Angles 1-3, BL Leg Joint Angles 1-3
%  Body Quaternions q1,q2,q3,q0]

jnt_var = [Theta1(1),Theta2(1),Theta3(1),...
    Theta1(2),Theta2(2),Theta3(2),...
    Theta1(3),Theta2(3),Theta3(3),...
    Theta1(4),Theta2(4),Theta3(4)];

set(gcf, 'Position', [600 80 900 900] );

h = clf(gcf);

Floor_v = [-600 600 0
    600 600 0
    -600 -600 0
    600 -600 0];

hold on

%% Relative Positions
%r_II_B = [0;0;0]; % place inertial frame at base
% big plate: Short: 350, L: 700;
% Relative Offsets

RobotConstants;
%% Orientations wrt I

% Orientation of Body
TB = T_I_B;

% Orientation of FR leg
T1_FR = TB*rotz(jnt_var(1));
T2_FR = T1_FR * rotx(jnt_var(2));
T3_FR = T2_FR * rotx(jnt_var(3));

% Orientation of FL leg
T1_FL = TB*rotz(jnt_var(4));
T2_FL = T1_FL * rotx(jnt_var(5));
T3_FL = T2_FL * rotx(jnt_var(6));

% Orientation of BR leg
T1_BR = TB*rotz(jnt_var(7));
T2_BR = T1_BR * rotx(jnt_var(8));
T3_BR = T2_BR * rotx(jnt_var(9));

% Orientation of BL leg
T1_BL = TB*rotz(jnt_var(10));
T2_BL = T1_BL * rotx(jnt_var(11));
T3_BL = T2_BL * rotx(jnt_var(12));

%% Positions wrt I
rB = r_II_B;

% Positions of FR Leg
r1_FR = rB + TB*r_BB_1_FR;
r2_FR = r1_FR + T1_FR * r_11_2_FR;
r3_FR = r2_FR + T2_FR * r_22_3_FR;
rc_FR = r3_FR + T3_FR * r_33_c_FR;

% Positions of FL Leg
r1_FL = rB + TB*r_BB_1_FL;
r2_FL = r1_FL + T1_FL * r_11_2_FL;
r3_FL = r2_FL + T2_FL * r_22_3_FL;
rc_FL = r3_FL + T3_FL * r_33_c_FL;

% Position of BR Leg
r1_BR = rB + TB*r_BB_1_BR;
r2_BR = r1_BR + T1_BR * r_11_2_BR;
r3_BR = r2_BR + T2_BR * r_22_3_BR;
rc_BR = r3_BR + T3_BR * r_33_c_BR;

% Position of BL Leg
r1_BL = rB + TB*r_BB_1_BL;
r2_BL = r1_BL + T1_BL * r_11_2_BL;
r3_BL = r2_BL + T2_BL * r_22_3_BL;
rc_BL = r3_BL + T3_BL * r_33_c_BL;
%% Calculate leg heights
legs=[rc_BR,rc_BL,rc_FL,rc_FR];
index = find(legs(3,:) <= 5/1000)';

if length(index) >= 3
    points(1:3,1)=legs(:,index(1));
    points(1:3,2)=legs(:,index(2));
    points(1:3,3)=legs(:,index(3));
    if length(index) == 4
        points(1:3,4)=legs(:,index(4));
    end
end
%% Draw floor
Floor_f = [1 2 4 3];
patch('Faces', Floor_f, 'Vertices', Floor_v, 'EdgeColor', 'None',...
    'FaceColor', [0 0 0.8], 'FaceAlpha', 0.5);
hold on
%% Transform the stl coordinates based upon FK
[Body, Body_f, n, c, stltitle] = stlread('Body.stl');
[FLLink1, FLLink1_f, n, c, stltitle] = stlread('Link_1_FL.stl');
[FLLink2, FLLink2_f, n, c, stltitle] = stlread('Link_2_FL.stl');
[FLLink3, FLLink3_f, n, c, stltitle] = stlread('Link_3_FL.stl');
[FRLink1, FRLink1_f, n, c, stltitle] = stlread('Link_1_FR.stl');
[FRLink2, FRLink2_f, n, c, stltitle] = stlread('Link_2_FR.stl');
[FRLink3, FRLink3_f, n, c, stltitle] = stlread('Link_3_FR.stl');
[BRLink1, BRLink1_f, n, c, stltitle] = stlread('Link_1_BR.stl');
[BRLink2, BRLink2_f, n, c, stltitle] = stlread('Link_2_BR.stl');
[BRLink3, BRLink3_f, n, c, stltitle] = stlread('Link_3_BR.stl');
[BLLink1, BLLink1_f, n, c, stltitle] = stlread('Link_1_BL.stl');
[BLLink2, BLLink2_f, n, c, stltitle] = stlread('Link_2_BL.stl');
[BLLink3, BLLink3_f, n, c, stltitle] = stlread('Link_3_BL.stl');
Body_v = (repmat(rB,1,length(Body)) + TB*Body');
FLLink1_v=(repmat(r1_FL,1,length(FLLink1))+T1_FL*FLLink1');
FLLink2_v=(repmat(r2_FL,1,length(FLLink2))+T2_FL*FLLink2');
FLLink3_v=(repmat(r3_FL,1,length(FLLink3))+T3_FL*FLLink3');

FRLink1_v=(repmat(r1_FR,1,length(FRLink1))+T1_FR*FRLink1');
FRLink2_v=(repmat(r2_FR,1,length(FRLink2))+T2_FR*FRLink2');
FRLink3_v=(repmat(r3_FR,1,length(FRLink3))+T3_FR*FRLink3');

BLLink1_v=(repmat(r1_BL,1,length(BLLink1))+T1_BL*BLLink1');
BLLink2_v=(repmat(r2_BL,1,length(BLLink2))+T2_BL*BLLink2');
BLLink3_v=(repmat(r3_BL,1,length(BLLink3))+T3_BL*BLLink3');

BRLink1_v=(repmat(r1_BR,1,length(BRLink1))+T1_BR*BRLink1');
BRLink2_v=(repmat(r2_BR,1,length(BRLink2))+T2_BR*BRLink2');
BRLink3_v=(repmat(r3_BR,1,length(BRLink3))+T3_BR*BRLink3');
%% draw robot
%set(gcf, 'Position', [50 50 950 900])
patch('Faces', Body_f, 'Vertices', Body_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);

if legs_valid(2)
patch('Faces', FLLink1_f, 'Vertices', FLLink1_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
patch('Faces', FLLink2_f, 'Vertices', FLLink2_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
patch('Faces', FLLink3_f, 'Vertices', FLLink3_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
else
patch('Faces', FLLink1_f, 'Vertices', FLLink1_v', 'EdgeColor', 'None', 'FaceColor', [1 0 0]);
patch('Faces', FLLink2_f, 'Vertices', FLLink2_v', 'EdgeColor', 'None', 'FaceColor', [1 0 0]);
patch('Faces', FLLink3_f, 'Vertices', FLLink3_v', 'EdgeColor', 'None', 'FaceColor', [1 0 0]);
    
end

if legs_valid(1)
patch('Faces', FRLink1_f, 'Vertices', FRLink1_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
patch('Faces', FRLink2_f, 'Vertices', FRLink2_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
patch('Faces', FRLink3_f, 'Vertices', FRLink3_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
else
patch('Faces', FRLink1_f, 'Vertices', FRLink1_v', 'EdgeColor', 'None', 'FaceColor', [1 0 0]);
patch('Faces', FRLink2_f, 'Vertices', FRLink2_v', 'EdgeColor', 'None', 'FaceColor', [1 0 0]);
patch('Faces', FRLink3_f, 'Vertices', FRLink3_v', 'EdgeColor', 'None', 'FaceColor', [1 0 0]);
    
end

if legs_valid(4)
patch('Faces', BLLink1_f, 'Vertices', BLLink1_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
patch('Faces', BLLink2_f, 'Vertices', BLLink2_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
patch('Faces', BLLink3_f, 'Vertices', BLLink3_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
else
patch('Faces', BLLink1_f, 'Vertices', BLLink1_v', 'EdgeColor', 'None', 'FaceColor', [1 0 0]);
patch('Faces', BLLink2_f, 'Vertices', BLLink2_v', 'EdgeColor', 'None', 'FaceColor', [1 0 0]);
patch('Faces', BLLink3_f, 'Vertices', BLLink3_v', 'EdgeColor', 'None', 'FaceColor', [1 0 0]); 
    
end

if legs_valid(3)
patch('Faces', BRLink1_f, 'Vertices', BRLink1_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
patch('Faces', BRLink2_f, 'Vertices', BRLink2_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
patch('Faces', BRLink3_f, 'Vertices', BRLink3_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
else
patch('Faces', BRLink1_f, 'Vertices', BRLink1_v', 'EdgeColor', 'None', 'FaceColor', [1 0 0]);
patch('Faces', BRLink2_f, 'Vertices', BRLink2_v', 'EdgeColor', 'None', 'FaceColor', [1 0 0]);
patch('Faces', BRLink3_f, 'Vertices', BRLink3_v', 'EdgeColor', 'None', 'FaceColor', [1 0 0]);
    
end

% draw support points
if length(index) >= 3
    scatter3(points(1,:),points(2,:),points(3,:),'rx')
    patch(points(1,:),points(2,:),points(3,:),[1,.5,.5])
end

hold on
plot3([rcm(1),rcm(1)-0.4,rcm(1)+0.4],[rcm(2),rcm(2),rcm(2)],[rcm(3),rcm(3),rcm(3)],'-r');
plot3([rcm(1),rcm(1),rcm(1)],[rcm(2),rcm(2)-0.4,rcm(2)+0.4],[rcm(3),rcm(3),rcm(3)],'-g');
plot3([rcm(1),rcm(1),rcm(1)],[rcm(2),rcm(2),rcm(2)],[rcm(3),rcm(3)-0.4,rcm(3)+0.4],'-b');
plot3(rcm(1),rcm(2),rcm(3),'*r');


axis equal
axis tight
camlight left
set(gca,'projection', 'perspective')
if view_angle == 'iso'
view([1;1;0.5])
elseif view_angle == 'top'
view([-5 2 5])
end

if view_type == 'fixed'
axis([-.6 1.4 -.6 0.6 -.2 .6])

elseif view_type == 'follw'
    axis([r_II_B(1)-0.6 r_II_B(1)+0.6 r_II_B(2)-0.6 r_II_B(2)+0.6 r_II_B(3)-0.45 r_II_B(3)+0.4])
end
grid on
hold off
xlabel('x')
ylabel('y')
zlabel('z')
title('Heimdallr Robot')
drawnow
hold off
end