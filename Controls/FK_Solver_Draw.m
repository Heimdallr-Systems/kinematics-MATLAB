function FK_Solver_Draw(Theta1,Theta2,Theta3,T_I_B,rBfromI,setgndplane)
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

rBfromI = rBfromI*1000;

jnt_var = [Theta1(1),Theta2(1),Theta3(1),...
                 Theta1(2),Theta2(2),Theta3(2),...
                 Theta1(3),Theta2(3),Theta3(3),...
                 Theta1(4),Theta2(4),Theta3(4)];

h = clf(gcf);

Floor_v = [-600 600 0
                 600 600 0
                 -600 -600 0
                 600 -600 0];
             
Floor_f = [1 2 4 3];

% patch('Faces', Floor_f, 'Vertices', Floor_v, 'EdgeColor', 'None',...
%       'FaceColor', [0 0 0.8], 'FaceAlpha', 0.5);
hold on
%set(gcf, 'Position', [50 50 950 900])

%% Relative Positions
%rBfromI = [0;0;0]; % place inertial frame at base
% big plate: Short: 350, L: 700;
% Relative FR Leg Positions
r1fromB_FR = [153.4;-53.31;50];
r2from1_FR = [0;-65.25;26.25];
r3from2_FR = [0;-224.68;0];
rcfrom3_FR = [0;-279;0];

% Relative FL Leg Positions
r1fromB_FL = [153.4;53.31;50];
r2from1_FL = [0;65.25;26.25];
r3from2_FL = [0;224.68;0];
rcfrom3_FL = [0;279;0];

% Relative BR Leg Positions
r1fromB_BR = [-153.4;-53.31;50];
r2from1_BR = [0;-65.25;26.25];
r3from2_BR = [0;-224.68;0];
rcfrom3_BR = [0;-279;0];

% Relative BL Leg Positions
r1fromB_BL = [-153.4;53.31;50];
r2from1_BL = [0;65.25;26.25];
r3from2_BL = [0;224.68;0];
rcfrom3_BL = [0;279;0];


%% Orientations wrt I

% Orientation of Body
 TB = T_I_B;

% Orientation of FR leg
T1_FR = TB*rotz(jnt_var(1));
T2_FR = T1_FR * rotx(jnt_var(2));
T3_FR = T2_FR * rotx(jnt_var(3));

% Orientation of FR leg
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
rB = rBfromI;

% Positions of FR Leg
r1_FR = rB + TB*r1fromB_FR;
r2_FR = r1_FR + T1_FR * r2from1_FR;
r3_FR = r2_FR + T2_FR * r3from2_FR;
rc_FR = r3_FR + T3_FR * rcfrom3_FR;

% Positions of FL Leg
r1_FL = rB + TB*r1fromB_FL;
r2_FL = r1_FL + T1_FL * r2from1_FL;
r3_FL = r2_FL + T2_FL * r3from2_FL;
rc_FL = r3_FL + T3_FL * rcfrom3_FL;

% Position of BR Leg
r1_BR = rB + TB*r1fromB_BR;
r2_BR = r1_BR + T1_BR * r2from1_BR;
r3_BR = r2_BR + T2_BR * r3from2_BR;
rc_BR = r3_BR + T3_BR * rcfrom3_BR;

% Position of BL Leg
r1_BL = rB + TB*r1fromB_BL;
r2_BL = r1_BL + T1_BL * r2from1_BL;
r3_BL = r2_BL + T2_BL * r3from2_BL;
rc_BL = r3_BL + T3_BL * rcfrom3_BL;
%% Calculate leg heights
legs=[rc_BR,rc_BL,rc_FL,rc_FR];
[vals,index]=sort([rc_BR(3),rc_BL(3),rc_FL(3),rc_FR(3)]);
points(1:3,1)=legs(:,index(1));
points(1:3,2)=legs(:,index(2));
maxchange=10;
pindex=3;
if(abs(legs(3,index(1))-legs(3,index(3)))<maxchange)
    points(1:3,pindex)=legs(:,index(3));
    pindex=4;
end
if(abs(legs(3,index(1))-legs(3,index(4)))<maxchange)
    points(1:3,pindex)=legs(:,index(4));
end
%% Draw floor
Floor_f = [1 2 4 3];

patch('Faces', Floor_f, 'Vertices', Floor_v, 'EdgeColor', 'None',...
       'FaceColor', [0 0 0.8], 'FaceAlpha', 0.5);
hold on
%% Transform the stl coordinates based upon FK
[Body, Body_f, n, c, stltitle] = stlread('Body.stl');
[FLLink1, FLLink1_f, n, c, stltitle] = stlread('Left_Shoulder_Link.stl');
[FLLink2, FLLink2_f, n, c, stltitle] = stlread('Left_Leg_Link_One.stl');
[FLLink3, FLLink3_f, n, c, stltitle] = stlread('Left_Leg_Link_Two.stl');
[FRLink1, FRLink1_f, n, c, stltitle] = stlread('Left_Shoulder_Link.stl');
[FRLink2, FRLink2_f, n, c, stltitle] = stlread('Left_Leg_Link_One.stl');
[FRLink3, FRLink3_f, n, c, stltitle] = stlread('Left_Leg_Link_Two.stl');
[BRLink1, BRLink1_f, n, c, stltitle] = stlread('Right_Shoulder_Link.stl');
[BRLink2, BRLink2_f, n, c, stltitle] = stlread('Right_Leg_Link_One.stl');
[BRLink3, BRLink3_f, n, c, stltitle] = stlread('Right_Leg_Link_Two.stl');
[BLLink1, BLLink1_f, n, c, stltitle] = stlread('Right_Shoulder_Link.stl');
[BLLink2, BLLink2_f, n, c, stltitle] = stlread('Right_Leg_Link_One.stl');
[BLLink3, BLLink3_f, n, c, stltitle] = stlread('Right_Leg_Link_Two.stl');
Body_v = (repmat(rB,1,length(Body)) + TB*Body')./1000;
FLLink1_v=(repmat(r1_FL,1,length(FLLink1))+T1_FL*FLLink1')./1000;
FLLink2_v=(repmat(r2_FL,1,length(FLLink2))+T2_FL*FLLink2')./1000;
FLLink3_v=(repmat(r3_FL,1,length(FLLink3))+T3_FL*FLLink3')./1000;

FRLink1_v=(repmat(r1_FR,1,length(FRLink1))+T1_FR*rotz(pi)*FRLink1')./1000;
FRLink2_v=(repmat(r2_FR,1,length(FRLink2))+T2_FR*rotx(pi)*FRLink2')./1000;
FRLink3_v=(repmat(r3_FR,1,length(FRLink3))+T3_FR*rotx(pi)*FRLink3')./1000;

BLLink1_v=(repmat(r1_BL,1,length(BLLink1))+T1_BL*rotz(pi)*BLLink1')./1000;
BLLink2_v=(repmat(r2_BL,1,length(BLLink2))+T2_BL*rotx(pi)*BLLink2')./1000;
BLLink3_v=(repmat(r3_BL,1,length(BLLink3))+T3_BL*rotx(pi)*BLLink3')./1000;

BRLink1_v=(repmat(r1_BR,1,length(BRLink1))+T1_BR*BRLink1')./1000;
BRLink2_v=(repmat(r2_BR,1,length(BRLink2))+T2_BR*BRLink2')./1000;
BRLink3_v=(repmat(r3_BR,1,length(BRLink3))+T3_BR*BRLink3')./1000;
%% draw robot
%set(gcf, 'Position', [50 50 950 900])
patch('Faces', Body_f, 'Vertices', Body_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
patch('Faces', FLLink1_f, 'Vertices', FLLink1_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
patch('Faces', FLLink2_f, 'Vertices', FLLink2_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
patch('Faces', FLLink3_f, 'Vertices', FLLink3_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);

 patch('Faces', FRLink1_f, 'Vertices', FRLink1_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
 patch('Faces', FRLink2_f, 'Vertices', FRLink2_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
 patch('Faces', FRLink3_f, 'Vertices', FRLink3_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
% 
 patch('Faces', BLLink1_f, 'Vertices', BLLink1_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
 patch('Faces', BLLink2_f, 'Vertices', BLLink2_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
 patch('Faces', BLLink3_f, 'Vertices', BLLink3_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
% 
 patch('Faces', BRLink1_f, 'Vertices', BRLink1_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
 patch('Faces', BRLink2_f, 'Vertices', BRLink2_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
 patch('Faces', BRLink3_f, 'Vertices', BRLink3_v', 'EdgeColor', 'None', 'FaceColor', [0.792157 0.819608 0.933333]);
%draw support points
scatter3(points(1,:)./1000,points(2,:)./1000,points(3,:)./1000,'rx')
patch(points(1,:)./1000,points(2,:)./1000,points(3,:)./1000,[1,.5,.5])
axis equal
camlight left
set(gca,'projection', 'perspective')
view([1;1;0.5])
axis([-.6 .6 -.6 .6 -.2 .6])
grid on
hold off
xlabel('x') 
ylabel('y')
zlabel('z')
title('Heimdallr Robot - Walking animation')
drawnow
end