% Robot Constants

clear
clc

% Relative Offsets
% Relative FR Leg Positions
r_BB_1_FR = [153.4;-53.31;50]./1000;
r_11_2_FR = [0;-65.25;26.25]./1000;
r_22_3_FR = [0;-224.68;0]./1000;
r_33_c_FR = [0;-279;0]./1000;

% Relative FL Leg Positions
r_BB_1_FL = [153.4;53.31;50]./1000;
r_11_2_FL = [0;65.25;26.25]./1000;
r_22_3_FL = [0;224.68;0]./1000;
r_33_c_FL = [0;279;0]./1000;

% Relative BR Leg Positions
r_BB_1_BR = [-153.4;-53.31;50]./1000;
r_11_2_BR = [0;-65.25;26.25]./1000;
r_22_3_BR = [0;-224.68;0]./1000;
r_33_c_BR = [0;-279;0]./1000;

% Relative BL Leg Positions
r_BB_1_BL = [-153.4;53.31;50]./1000;
r_11_2_BL = [0;65.25;26.25]./1000;
r_22_3_BL = [0;224.68;0]./1000;
r_33_c_BL = [0;279;0]./1000;

% Masses
mB = 1;

m1_FR = 1;
m2_FR = 1;
m3_FR = 1;

m1_FL = 1;
m2_FL = 1;
m3_FL = 1;

m1_BR = 1;
m2_BR = 1;
m3_BR = 1;

m1_BL = 1;
m2_BL = 1;
m3_BL = 1;

% CM vectors (rcm_(reference)_(body)_(leg))
rcm_B_B = [0;0;0];
rcm_1_1_FR = r_11_2_FR./2;
rcm_2_2_FR = r_22_3_FR./2;
rcm_3_3_FR = r_33_c_FR./2;

rcm_1_1_FL = r_11_2_FL./2;
rcm_2_2_FL = r_22_3_FL./2;
rcm_3_3_FL = r_33_c_FL./2;

rcm_1_1_BR = r_11_2_BR./2;
rcm_2_2_BR = r_22_3_BR./2;
rcm_3_3_BR = r_33_c_BR./2;

rcm_1_1_BL = r_11_2_BL./2;
rcm_2_2_BL = r_22_3_BL./2;
rcm_3_3_BL = r_33_c_BL./2;

% Inertia Matricies (J_(reference)_(body))
JBB =eye(3);

J11_FR =eye(3);
J22_FR = eye(3);
J33_FR = eye(3);

J11_FL =eye(3);
J22_FL =eye(3);
J33_FL = eye(3);

J11_BR = eye(3);
J22_BR = eye(3);
J33_BR =eye(3);

J11_BL = eye(3);
J22_BL =eye(3);
J33_BL = eye(3);

% Mass Moment Vectors
GAMMABB = rcm_B_B*mB;

GAMMA11_FR = rcm_1_1_FR*m1_FR;
GAMMA22_FR = rcm_2_2_FR*m2_FR;
GAMMA33_FR = rcm_3_3_FR*m3_FR;

GAMMA11_FL = rcm_1_1_FL*m1_FL;
GAMMA22_FL = rcm_2_2_FL*m2_FL;
GAMMA33_FL = rcm_3_3_FL*m3_FL;

GAMMA11_BR = rcm_1_1_BR*m1_BR;
GAMMA22_BR = rcm_2_2_BR*m2_BR;
GAMMA33_BR = rcm_3_3_BR*m3_BR;

GAMMA11_BL = rcm_1_1_BL*m1_BL;
GAMMA22_BL = rcm_2_2_BL*m2_BL;
GAMMA33_BL = rcm_3_3_BL*m3_BL;

save('robotConstants.mat');