r_BB_1_FR = [125.78; -125.78; 25.4]./1000;
r_11_2_FR = [0; -39; 0]./1000;
r_22_3_FR = [ 0; -150; 0]./1000;
r_33_c_FR = [0; -204; 0]./1000;

% Relative FL Leg Positions
r_BB_1_FL = [ 125.78; 125.78; 25.4]./1000;
r_11_2_FL = [0; 39; 0]./1000;
r_22_3_FL = [ 0; 150; 0]./1000;
r_33_c_FL = [0; 204; 0]./1000;

% Relative BR Leg Positions
r_BB_1_BR = [-125.78; -125.78; 25.4]./1000;
r_11_2_BR = [0; -39; 0]./1000;
r_22_3_BR = [ 0; -150; 0]./1000;
r_33_c_BR = [0; -204; 0]./1000;

% Relative BL Leg Positions
r_BB_1_BL = [ -125.78; 125.78; 25.4]./1000;
r_11_2_BL = [0; 39; 0]./1000;
r_22_3_BL = [ 0; 150; 0]./1000;
r_33_c_BL = [0; 204; 0]./1000;

% Masses
mB = 2.72497050;

m1_FR = 0.07941012;
m2_FR = 0.51394001;
m3_FR = 0.38648899;

m1_FL = 0.07941022;
m2_FL = 0.51394001;
m3_FL = 0.38648899;

m1_BR = 0.07941003;
m2_BR = 0.51394001;
m3_BR = 0.38648899;

m1_BL = 0.07941012;
m2_BL = 0.51394001;
m3_BL = 0.38648899;

% CM vectors (rcm_(reference)_(body)_(leg))
rcm_B_B = [-0.00058841;-0.00160130;0.03870501];

rcm_1_1_FR = [0;-0.01950002;0];
rcm_2_2_FR = [-0.01562645;-0.07500000;-0.00007567];
rcm_3_3_FR = [0.00000286;-0.10411606;0.00002025 ];

rcm_1_1_FL = [0;0.01950000;0];
rcm_2_2_FL = [-0.01562645;0.07500000;-0.00007567];
rcm_3_3_FL = [-0.00000286;0.10411606;0.00002025];

rcm_1_1_BR = [0; -0.01950000 ;0];
rcm_2_2_BR = [0.01562644;-0.07500000;-0.00007567];
rcm_3_3_BR = [0.00000286;-0.10411606;0.00002025];

rcm_1_1_BL = [0;0.01950002;0];
rcm_2_2_BL = [0.01562644;0.07500000;-0.00007567];
rcm_3_3_BL = [-0.00000286;0.10411606;0.00002025];

% Inertia Matricies (J_(reference)_(body))
JBB =  [0.03324076,0.00007952,-0.00058194 
    0.00007952,0.03133513,0.00094644 
    -0.00058194,0.00094644,0.03825413];

J11_FR = [0.00006962,0.00000000,0.00000000 
0.00000000,0.00001651,0.00000000 
0.00000000,0.00000000,0.00006962];
J22_FR = [0.00487136,-0.00060233,-0.00000005 
-0.00060233,0.00075364,-0.00000287 
-0.00000005,-0.00000287,0.00550480];
J33_FR = [0.00592285,0.00000002,0.00000000 
0.00000002,0.00012963,0.00000911 
0.00000000,0.00000911,0.00591442];


J11_FL = [0.00006962,0.00000000,0.00000000 
0.00000000,0.00001651,0.00000000 
0.00000000,0.00000000,0.00006962];
J22_FL = [0.00487136,0.00060233,-0.00000005 
0.00060233,0.00075364,-0.00000296 
-0.00000005,0.00000296,0.00550480];
J33_FL = [0.00592285,0.00000002,0.00000000 
0.00000002,0.00012963,-0.00000911 
0.00000000,-0.00000911,0.00591442];


J11_BR = [0.00006962,0.00000000,0.00000000 
0.00000000,0.00001651,0.00000000 
0.00000000,0.00000000,0.00006962];
J22_BR = [0.00487136,0.00060233,0.00000005 
         0.00060233,0.00075364,-0.00000296 
        0.00000005,-0.00000296,0.00550480];
J33_BR = [0.00592285,0.00000002,0.00000000 
0.00000002,0.00012963,0.00000911 
0.00000000,0.00000911,0.00591442];



J11_BL = [0.00006962,0.00000000,0.00000000 
0.00000000,0.00001651,0.00000000 
0.00000000,0.00000000,0.00006962];
J22_BL = [0.00487136,-0.00060233,0.00000005 
        -0.00060233,0.00075364,0.00000287 
        0.00000005,0.00000287,0.00550480];
J33_BL = [0.00592285,0.00000002,0.00000000
          0.00000002,0.00012963,-0.00000911 
          0.00000000,-0.00000911,0.00591442];

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

B = zeros(18,18);
C = zeros(18,18);