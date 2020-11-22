function [H,d,G] = robot_terms(state)
% gamma = [phi; theta; psi; x; y; z; ...
%                  Theta1_FR; Theta1_FL; Theta1_BR; Theta1_BL; ...
%                  Theta2_FR; Theta2_FL; Theta2_BR; Theta2_BL; ...
%                  Theta3_FR; Theta3_FL; Theta3_BR; Theta3_BL];
% dotgamma = [dotphi; dottheta; dotpsi; dotx; doty; dotz; ...
%                        dotTheta1_FR; dotTheta1_FL; dotTheta1_BR; dotTheta1_BL; ...
%                        dotTheta2_FR; dotTheta2_FL; dotTheta2_BR; dotTheta2_BL; ...
%                        dotTheta3_FR; dotTheta3_FR; dotTheta3_BR; dotTheta3_BL];
%
% state = [gamma;dotgamma];


dotgamma(:,1) = state(19:36);

dotstate = zeros(36,1);
dotstate(1:18,1) = dotgamma;

% Physical Constants %
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

% Relative Orientations (T_(reference)_(new))
T_I_B = rotz(state(1))*roty(state(2))*rotx(state(3));

T_B_1_FR = rotz(state(7));
T_1_2_FR = rotx(state(11));
T_2_3_FR = rotx(state(15));

T_B_1_FL = rotz(state(8));
T_1_2_FL = rotx(state(12));
T_2_3_FL = rotx(state(16));

T_B_1_BR = rotz(state(9));
T_1_2_BR = rotx(state(13));
T_2_3_BR = rotx(state(17));

T_B_1_BL = rotz(state(10));
T_1_2_BL = rotx(state(14));
T_2_3_BL = rotx(state(18));

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

g = -9.8;

phi = state(1);
theta = state(2);
psi = state(3);
dotphi = state(19);
dottheta = state(20);
dotpsi = state(21);

% Body
GeoJB = [-sin(theta), 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                    cos(theta)*sin(psi),  cos(psi), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                    cos(psi)*cos(theta), -sin(psi), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                    0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                    0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                    0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
dotGeoJB = [-dottheta*cos(theta), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                         dotpsi*cos(psi)*cos(theta) - dottheta*sin(psi)*sin(theta), -dotpsi*sin(psi), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                         -dotpsi*cos(theta)*sin(psi) - dottheta*cos(psi)*sin(theta), -dotpsi*cos(psi), 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
                         0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0];
w_BB_I = [dotphi;dottheta;dotpsi];
MB = [JBB,skew(GAMMABB)*T_I_B.';T_I_B*skew(GAMMABB).',mB*eye(3)];
HB = GeoJB.'*MB*GeoJB; % System Mass Matric
GB = (GeoJB.'*[skew(GAMMABB)*T_I_B.'*[0;0;g];mB*[0;0;g]]); % Generalize Gravitational Forces
dB = GeoJB.'*MB*dotGeoJB*dotgamma + GeoJB.'*[cross(w_BB_I, JBB*w_BB_I);T_I_B*cross(w_BB_I,cross(w_BB_I,GAMMABB))]; % vector of coriolis and centripital terms
                    

% Link 1 FR
I1_hat_FR = zeros(3,18);
I1_hat_FR(3,7) = 1; % z-axis, Theta1_FR
I1_tilde_FR = zeros(3,18);
[T_I_1_FR, w_11_I_FR, ~, GeoJ1_FR, dotGeoJ1_FR] = recursiveKin(dotgamma, T_I_B, T_B_1_FR, r_BB_1_FR, I1_hat_FR, I1_tilde_FR, GeoJB, dotGeoJB);
M1_FR = [J11_FR,skew(GAMMA11_FR)*T_I_1_FR.';T_I_1_FR*skew(GAMMA11_FR).',m1_FR*eye(3)];
H1_FR = GeoJ1_FR.'*M1_FR*GeoJ1_FR;
G1_FR = (GeoJ1_FR.'*[skew(GAMMA11_FR)*T_I_1_FR.'*[0;0;g];m1_FR*[0;0;g]]);
d1_FR = GeoJ1_FR.'*M1_FR*dotGeoJ1_FR*dotgamma + GeoJ1_FR.'*[cross(w_11_I_FR, J11_FR*w_11_I_FR);T_I_1_FR*cross(w_11_I_FR,cross(w_11_I_FR,GAMMA11_FR))];

% Link 2 FR
I2_hat_FR = zeros(3,18);
I2_hat_FR(1,11) = 1; % x-axis, Theta2_FR
I2_tilde_FR = zeros(3,18);
[T_I_2_FR, w_22_I_FR, ~, GeoJ2_FR, dotGeoJ2_FR] = recursiveKin(dotgamma, T_I_1_FR, T_1_2_FR, r_11_2_FR, I2_hat_FR, I2_tilde_FR, GeoJ1_FR, dotGeoJ1_FR);
M2_FR = [J22_FR,skew(GAMMA22_FR)*T_I_2_FR.';T_I_2_FR*skew(GAMMA22_FR).',m2_FR*eye(3)];
H2_FR = GeoJ2_FR.'*M2_FR*GeoJ2_FR;
G2_FR = (GeoJ2_FR.'*[skew(GAMMA22_FR)*T_I_2_FR.'*[0;0;g];m2_FR*[0;0;g]]);
d2_FR = GeoJ2_FR.'*M2_FR*dotGeoJ2_FR*dotgamma + GeoJ2_FR.'*[cross(w_22_I_FR, J22_FR*w_22_I_FR);T_I_2_FR*cross(w_22_I_FR,cross(w_22_I_FR,GAMMA22_FR))];

% Link 3 FR
I3_hat_FR = zeros(3,18);
I3_hat_FR(1,15) = 1; % x-axis, Theta3_FR
I3_tilde_FR = zeros(3,18);
[T_I_3_FR, w_33_I_FR, ~, GeoJ3_FR, dotGeoJ3_FR] = recursiveKin(dotgamma, T_I_2_FR, T_2_3_FR, r_22_3_FR, I3_hat_FR, I3_tilde_FR, GeoJ2_FR, dotGeoJ2_FR);
M3_FR = [J33_FR,skew(GAMMA33_FR)*T_I_3_FR.';T_I_3_FR*skew(GAMMA33_FR).',m3_FR*eye(3)];
H3_FR = GeoJ3_FR.'*M3_FR*GeoJ3_FR;
G3_FR = (GeoJ3_FR.'*[skew(GAMMA33_FR)*T_I_3_FR.'*[0;0;g];m3_FR*[0;0;g]]);
d3_FR = GeoJ3_FR.'*M3_FR*dotGeoJ3_FR*dotgamma + GeoJ3_FR.'*[cross(w_33_I_FR, J33_FR*w_33_I_FR);T_I_3_FR*cross(w_33_I_FR,cross(w_33_I_FR,GAMMA33_FR))];


% Link 1 FL
I1_hat_FL = zeros(3,18);
I1_hat_FL(3,8) = 1; % z-axis, Theta1_FL
I1_tilde_FL = zeros(3,18);
[T_I_1_FL, w_11_I_FL, ~, GeoJ1_FL, dotGeoJ1_FL] = recursiveKin(dotgamma, T_I_B, T_B_1_FL, r_BB_1_FL, I1_hat_FL, I1_tilde_FL, GeoJB, dotGeoJB);
M1_FL = [J11_FL,skew(GAMMA11_FL)*T_I_1_FL.';T_I_1_FL*skew(GAMMA11_FL).',m1_FL*eye(3)];
H1_FL = GeoJ1_FL.'*M1_FL*GeoJ1_FL;
G1_FL = (GeoJ1_FL.'*[skew(GAMMA11_FL)*T_I_1_FL.'*[0;0;g];m1_FL*[0;0;g]]);
d1_FL = GeoJ1_FL.'*M1_FL*dotGeoJ1_FL*dotgamma + GeoJ1_FL.'*[cross(w_11_I_FL, J11_FL*w_11_I_FL);T_I_1_FL*cross(w_11_I_FL,cross(w_11_I_FL,GAMMA11_FL))];

% Link 2 FL
I2_hat_FL = zeros(3,18);
I2_hat_FL(1,12) = 1; % x-axis, Theta2_FL
I2_tilde_FL = zeros(3,18);
[T_I_2_FL, w_22_I_FL, ~, GeoJ2_FL, dotGeoJ2_FL] = recursiveKin(dotgamma, T_I_1_FL, T_1_2_FL, r_11_2_FL, I2_hat_FL, I2_tilde_FL, GeoJ1_FL, dotGeoJ1_FL);
M2_FL = [J22_FL,skew(GAMMA22_FL)*T_I_2_FL.';T_I_2_FL*skew(GAMMA22_FL).',m2_FL*eye(3)];
H2_FL = GeoJ2_FL.'*M2_FL*GeoJ2_FL;
G2_FL = (GeoJ2_FL.'*[skew(GAMMA22_FL)*T_I_2_FL.'*[0;0;g];m2_FL*[0;0;g]]);
d2_FL = GeoJ2_FL.'*M2_FL*dotGeoJ2_FL*dotgamma + GeoJ2_FL.'*[cross(w_22_I_FL, J22_FL*w_22_I_FL);T_I_2_FL*cross(w_22_I_FL,cross(w_22_I_FL,GAMMA22_FL))];

% Link 3 FL
I3_hat_FL = zeros(3,18);
I3_hat_FL(1,16) = 1; % x-axis, Theta3_FL
I3_tilde_FL = zeros(3,18);
[T_I_3_FL, w_33_I_FL, ~, GeoJ3_FL, dotGeoJ3_FL] = recursiveKin(dotgamma, T_I_2_FL, T_2_3_FL, r_22_3_FL, I3_hat_FL, I3_tilde_FL, GeoJ2_FL, dotGeoJ2_FL);
M3_FL = [J33_FL,skew(GAMMA33_FL)*T_I_3_FL.';T_I_3_FL*skew(GAMMA33_FL).',m3_FL*eye(3)];
H3_FL = GeoJ3_FL.'*M3_FL*GeoJ3_FL;
G3_FL = (GeoJ3_FL.'*[skew(GAMMA33_FL)*T_I_3_FL.'*[0;0;g];m3_FL*[0;0;g]]);
d3_FL = GeoJ3_FL.'*M3_FL*dotGeoJ3_FL*dotgamma + GeoJ3_FL.'*[cross(w_33_I_FL, J33_FL*w_33_I_FL);T_I_3_FL*cross(w_33_I_FL,cross(w_33_I_FL,GAMMA33_FL))];


% Link 1 BR
I1_hat_BR = zeros(3,18);
I1_hat_BR(3,9) = 1; % z-axis, Theta1_BR
I1_tilde_BR = zeros(3,18);
[T_I_1_BR, w_11_I_BR, ~, GeoJ1_BR, dotGeoJ1_BR] = recursiveKin(dotgamma, T_I_B, T_B_1_BR, r_BB_1_BR, I1_hat_BR, I1_tilde_BR, GeoJB, dotGeoJB);
M1_BR = [J11_BR,skew(GAMMA11_BR)*T_I_1_BR.';T_I_1_BR*skew(GAMMA11_BR).',m1_BR*eye(3)];
H1_BR = GeoJ1_BR.'*M1_BR*GeoJ1_BR;
G1_BR = (GeoJ1_BR.'*[skew(GAMMA11_BR)*T_I_1_BR.'*[0;0;g];m1_BR*[0;0;g]]);
d1_BR = GeoJ1_BR.'*M1_BR*dotGeoJ1_BR*dotgamma + GeoJ1_BR.'*[cross(w_11_I_BR, J11_BR*w_11_I_BR);T_I_1_BR*cross(w_11_I_BR,cross(w_11_I_BR,GAMMA11_BR))];

% Link 2 BR
I2_hat_BR = zeros(3,18);
I2_hat_BR(1,13) = 1; % x-axis, Theta2_BR
I2_tilde_BR = zeros(3,18);
[T_I_2_BR, w_22_I_BR, ~, GeoJ2_BR, dotGeoJ2_BR] = recursiveKin(dotgamma, T_I_1_BR, T_1_2_BR, r_11_2_BR, I2_hat_BR, I2_tilde_BR, GeoJ1_BR, dotGeoJ1_BR);
M2_BR = [J22_BR,skew(GAMMA22_BR)*T_I_2_BR.';T_I_2_BR*skew(GAMMA22_BR).',m2_BR*eye(3)];
H2_BR = GeoJ2_BR.'*M2_BR*GeoJ2_BR;
G2_BR = (GeoJ2_BR.'*[skew(GAMMA22_BR)*T_I_2_BR.'*[0;0;g];m2_BR*[0;0;g]]);
d2_BR = GeoJ2_BR.'*M2_BR*dotGeoJ2_BR*dotgamma + GeoJ2_BR.'*[cross(w_22_I_BR, J22_BR*w_22_I_BR);T_I_2_BR*cross(w_22_I_BR,cross(w_22_I_BR,GAMMA22_BR))];

% Link 3 BR
I3_hat_BR = zeros(3,18);
I3_hat_BR(1,17) = 1; % x-axis, Theta3_BR
I3_tilde_BR = zeros(3,18);
[T_I_3_BR, w_33_I_BR, ~, GeoJ3_BR, dotGeoJ3_BR] = recursiveKin(dotgamma, T_I_2_BR, T_2_3_BR, r_22_3_BR, I3_hat_BR, I3_tilde_BR, GeoJ2_BR, dotGeoJ2_BR);
M3_BR = [J33_BR,skew(GAMMA33_BR)*T_I_3_BR.';T_I_3_BR*skew(GAMMA33_BR).',m3_BR*eye(3)];
H3_BR = GeoJ3_BR.'*M3_BR*GeoJ3_BR;
G3_BR = (GeoJ3_BR.'*[skew(GAMMA33_BR)*T_I_3_BR.'*[0;0;g];m3_BR*[0;0;g]]);
d3_BR = GeoJ3_BR.'*M3_BR*dotGeoJ3_BR*dotgamma + GeoJ3_BR.'*[cross(w_33_I_BR, J33_BR*w_33_I_BR);T_I_3_BR*cross(w_33_I_BR,cross(w_33_I_BR,GAMMA33_BR))];


% Link 1 BL
I1_hat_BL = zeros(3,18);
I1_hat_BL(3,10) = 1; % z-axis, Theta1_BL
I1_tilde_BL = zeros(3,18);
[T_I_1_BL, w_11_I_BL, ~, GeoJ1_BL, dotGeoJ1_BL] = recursiveKin(dotgamma, T_I_B, T_B_1_BL, r_BB_1_BL, I1_hat_BL, I1_tilde_BL, GeoJB, dotGeoJB);
M1_BL = [J11_BL,skew(GAMMA11_BL)*T_I_1_BL.';T_I_1_BL*skew(GAMMA11_BL).',m1_BL*eye(3)];
H1_BL = GeoJ1_BL.'*M1_BL*GeoJ1_BL;
G1_BL = (GeoJ1_BL.'*[skew(GAMMA11_BL)*T_I_1_BL.'*[0;0;g];m1_BL*[0;0;g]]);
d1_BL = GeoJ1_BL.'*M1_BL*dotGeoJ1_BL*dotgamma + GeoJ1_BL.'*[cross(w_11_I_BL, J11_BL*w_11_I_BL);T_I_1_BL*cross(w_11_I_BL,cross(w_11_I_BL,GAMMA11_BL))];

% Link 2 BL
I2_hat_BL = zeros(3,18);
I2_hat_BL(1,14) = 1; % x-axis, Theta2_BL
I2_tilde_BL = zeros(3,18);
[T_I_2_BL, w_22_I_BL, ~, GeoJ2_BL, dotGeoJ2_BL] = recursiveKin(dotgamma, T_I_1_BL, T_1_2_BL, r_11_2_BL, I2_hat_BL, I2_tilde_BL, GeoJ1_BL, dotGeoJ1_BL);
M2_BL = [J22_BL,skew(GAMMA22_BL)*T_I_2_BL.';T_I_2_BL*skew(GAMMA22_BL).',m2_BL*eye(3)];
H2_BL = GeoJ2_BL.'*M2_BL*GeoJ2_BL;
G2_BL = (GeoJ2_BL.'*[skew(GAMMA22_BL)*T_I_2_BL.'*[0;0;g];m2_BL*[0;0;g]]);
d2_BL = GeoJ2_BL.'*M2_BL*dotGeoJ2_BL*dotgamma + GeoJ2_BL.'*[cross(w_22_I_BL, J22_BL*w_22_I_BL);T_I_2_BL*cross(w_22_I_BL,cross(w_22_I_BL,GAMMA22_BL))];

% Link 3 BL
I3_hat_BL = zeros(3,18);
I3_hat_BL(1,18) = 1; % x-axis, Theta3_BL
I3_tilde_BL = zeros(3,18);
[T_I_3_BL, w_33_I_BL, ~, GeoJ3_BL, dotGeoJ3_BL] = recursiveKin(dotgamma, T_I_2_BL, T_2_3_BL, r_22_3_BL, I3_hat_BL, I3_tilde_BL, GeoJ2_BL, dotGeoJ2_BL);
M3_BL = [J33_BL,skew(GAMMA33_BL)*T_I_3_BL.';T_I_3_BL*skew(GAMMA33_BL).',m3_BL*eye(3)];
H3_BL = GeoJ3_BL.'*M3_BL*GeoJ3_BL;
G3_BL = (GeoJ3_BL.'*[skew(GAMMA33_BL)*T_I_3_BL.'*[0;0;g];m3_BL*[0;0;g]]);
d3_BL = GeoJ3_BL.'*M3_BL*dotGeoJ3_BL*dotgamma + GeoJ3_BL.'*[cross(w_33_I_BL, J33_BL*w_33_I_BL);T_I_3_BL*cross(w_33_I_BL,cross(w_33_I_BL,GAMMA33_BL))];

H = HB+H1_FR+H1_FL+H1_BR+H1_BL+H2_FR+H2_FL+H2_BR+H2_BL+H3_FR+H3_FL+H3_BR+H3_BL;
G = (GB+G1_FR+G1_FL+G1_BR+G1_BL+G2_FR+G2_FL+G2_BR+G2_BL+G3_FR+G3_FL+G3_BR+G3_BL);
d = dB+d1_FR+d1_FL+d1_BR+d1_BL+d2_FR+d2_FL+d2_BR+d2_BL+d3_FR+d3_FL+d3_BR+d3_BL;
end