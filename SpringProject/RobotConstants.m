function constants = RobotConstants()
constants.r_BB_1_FR = [125.78; -125.78; 25.4]./1000;
constants.r_11_2_FR = [0; -54; 0]./1000;
constants.r_22_3_FR = [ 0; -150; 0]./1000;
constants.r_33_c_FR = [0; -204; 0]./1000;

% Relative FL Leg Positions
constants.r_BB_1_FL = [ 125.78; 125.78; 25.4]./1000;
constants.r_11_2_FL = [0; 54; 0]./1000;
constants.r_22_3_FL = [ 0; 150; 0]./1000;
constants.r_33_c_FL = [0; 204; 0]./1000;

% Relative BR Leg Positions
constants.r_BB_1_BR = [-125.78; -125.78; 25.4]./1000;
constants.r_11_2_BR = [0; -54; 0]./1000;
constants.r_22_3_BR = [ 0; -150; 0]./1000;
constants.r_33_c_BR = [0; -204; 0]./1000;

% Relative BL Leg Positions
constants.r_BB_1_BL = [ -125.78; 125.78; 25.4]./1000;
constants.r_11_2_BL = [0; 54; 0]./1000;
constants.r_22_3_BL = [ 0; 150; 0]./1000;
constants.r_33_c_BL = [0; 204; 0]./1000;

% Masses
constants.mB = 2.72497050;

constants.m1_FR = 0.07941012;
constants.m2_FR = 0.51394001;
constants.m3_FR = 0.38648899;

constants.m1_FL = 0.07941022;
constants.m2_FL = 0.51394001;
constants.m3_FL = 0.38648899;

constants.m1_BR = 0.07941003;
constants.m2_BR = 0.51394001;
constants.m3_BR = 0.38648899;

constants.m1_BL = 0.07941012;
constants.m2_BL = 0.51394001;
constants.m3_BL = 0.38648899;

% CM vectors (rcm_(reference)_(body)_(leg))
constants.rcm_B_B = [-0.00058841;-0.00160130;0.03870501];

constants.rcm_1_1_FR = [0;-0.01950002;0];
constants.rcm_2_2_FR = [-0.01562645;-0.07500000;-0.00007567];
constants.rcm_3_3_FR = [0.00000286;-0.10411606;0.00002025 ];

constants.rcm_1_1_FL = [0;0.01950000;0];
constants.rcm_2_2_FL = [-0.01562645;0.07500000;-0.00007567];
constants.rcm_3_3_FL = [-0.00000286;0.10411606;0.00002025];

constants.rcm_1_1_BR = [0; -0.01950000 ;0];
constants.rcm_2_2_BR = [0.01562644;-0.07500000;-0.00007567];
constants.rcm_3_3_BR = [0.00000286;-0.10411606;0.00002025];

constants.rcm_1_1_BL = [0;0.01950002;0];
constants.rcm_2_2_BL = [0.01562644;0.07500000;-0.00007567];
constants.rcm_3_3_BL = [-0.00000286;0.10411606;0.00002025];

% Inertia Matricies (J_(reference)_(body))
constants.JBB =  [0.03324076,0.00007952,-0.00058194
    0.00007952,0.03133513,0.00094644
    -0.00058194,0.00094644,0.03825413];

constants.J11_FR = [0.00006962,0.00000000,0.00000000
    0.00000000,0.00001651,0.00000000
    0.00000000,0.00000000,0.00006962];
constants.J22_FR = [0.00487136,-0.00060233,-0.00000005
    -0.00060233,0.00075364,-0.00000287
    -0.00000005,-0.00000287,0.00550480];
constants.J33_FR = [0.00592285,0.00000002,0.00000000
    0.00000002,0.00012963,0.00000911
    0.00000000,0.00000911,0.00591442];


constants.J11_FL = [0.00006962,0.00000000,0.00000000
    0.00000000,0.00001651,0.00000000
    0.00000000,0.00000000,0.00006962];
constants.J22_FL = [0.00487136,0.00060233,-0.00000005
    0.00060233,0.00075364,-0.00000296
    -0.00000005,0.00000296,0.00550480];
constants.J33_FL = [0.00592285,0.00000002,0.00000000
    0.00000002,0.00012963,-0.00000911
    0.00000000,-0.00000911,0.00591442];


constants.J11_BR = [0.00006962,0.00000000,0.00000000
    0.00000000,0.00001651,0.00000000
    0.00000000,0.00000000,0.00006962];
constants.J22_BR = [0.00487136,0.00060233,0.00000005
    0.00060233,0.00075364,-0.00000296
    0.00000005,-0.00000296,0.00550480];
constants.J33_BR = [0.00592285,0.00000002,0.00000000
    0.00000002,0.00012963,0.00000911
    0.00000000,0.00000911,0.00591442];



constants.J11_BL = [0.00006962,0.00000000,0.00000000
    0.00000000,0.00001651,0.00000000
    0.00000000,0.00000000,0.00006962];
constants.J22_BL = [0.00487136,-0.00060233,0.00000005
    -0.00060233,0.00075364,0.00000287
    0.00000005,0.00000287,0.00550480];
constants.J33_BL = [0.00592285,0.00000002,0.00000000
    0.00000002,0.00012963,-0.00000911
    0.00000000,-0.00000911,0.00591442];

constants.B = zeros(18,18);
constants.C = zeros(18,18);

% Codegen will not work if we try to add to the struct after reading from
% it. so we need to initialize some constants.
constants.GAMMABB = zeros(3,1);

constants.GAMMA11_FR = zeros(3,1);
constants.GAMMA22_FR = zeros(3,1);
constants.GAMMA33_FR = zeros(3,1);

constants.GAMMA11_FL = zeros(3,1);
constants.GAMMA22_FL = zeros(3,1);
constants.GAMMA33_FL = zeros(3,1);

constants.GAMMA11_BR = zeros(3,1);
constants.GAMMA22_BR = zeros(3,1);
constants.GAMMA33_BR = zeros(3,1);

constants.GAMMA11_BL = zeros(3,1);
constants.GAMMA22_BL = zeros(3,1);
constants.GAMMA33_BL = zeros(3,1);

% Mass Moment Vectors
constants.GAMMABB = constants.rcm_B_B*constants.mB;

constants.GAMMA11_FR = constants.rcm_1_1_FR*constants.m1_FR;
constants.GAMMA22_FR = constants.rcm_2_2_FR*constants.m2_FR;
constants.GAMMA33_FR = constants.rcm_3_3_FR*constants.m3_FR;

constants.GAMMA11_FL = constants.rcm_1_1_FL*constants.m1_FL;
constants.GAMMA22_FL = constants.rcm_2_2_FL*constants.m2_FL;
constants.GAMMA33_FL = constants.rcm_3_3_FL*constants.m3_FL;

constants.GAMMA11_BR = constants.rcm_1_1_BR*constants.m1_BR;
constants.GAMMA22_BR = constants.rcm_2_2_BR*constants.m2_BR;
constants.GAMMA33_BR = constants.rcm_3_3_BR*constants.m3_BR;

constants.GAMMA11_BL = constants.rcm_1_1_BL*constants.m1_BL;
constants.GAMMA22_BL = constants.rcm_2_2_BL*constants.m2_BL;
constants.GAMMA33_BL = constants.rcm_3_3_BL*constants.m3_BL;


end