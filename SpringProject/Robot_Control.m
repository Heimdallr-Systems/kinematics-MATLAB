function [Theta1_d_out,Theta2_d_out,Theta3_d_out,phi_d_temp_out,r_II_B_d_temp_out,floor_toggle_out,legs_valid_out] = Robot_Control(r_II_B_d, Euler_d, gamma_m, init_toggle, legs_on_gnd) %#codegen
%Controls Robot's walking algorithm
%   input: r_II_B_d, Euler_d, gamma_m
%   output: Theta_d (1-3), phi_d_temp & r_II_b_d_temp (orientation for plotting),
%   floor_toggle & legs_valid (for computing forces)
%   NOTE: For the real system, phi_d_temp & r_II_b_d_temp,
%   floor_toggle & legs_valid can be thrown
%   away. For kinematic system, floor_toggle is not needed. For dynamic
%   system phi_d_temp % r_II_B_d_temp is not needed
%   r_II_B_d = [x_d;y_d;z_d];
%   Euler_d = [phi,theta,psi];
%   gamma_m = [Euler,r_II_B,Theta1,Theta2,Theta3];
%   Euler = [phi,theta,psi];
%   r_II_B = [x,y,z];
%   Theta1 = [Theta1FR,FL,BR,BL];
%   Theta2 = [Theta2FR,FL,BR,BL];
%   Theta3 = [Theta3FR,FL,BR,BL];

assert(all(size(r_II_B_d)==[3, 1]))
assert(all(size(gamma_m)==[36, 1]))
assert(all(size(Euler_d)==[1, 3]))
assert(isa(gamma_m, 'double'))
assert(isa(r_II_B_d, 'double'))
assert(isa(Euler_d, 'double'))
assert(isa(init_toggle, 'logical'))

% Variable Init
step_dist = 0.12; % leg step distance

%%%CHANGE IF NEEDED%%%
manual_step_breakout = 0;

% constants for mid-step resting position
% FR
Theta1_d_midpt_FR = pi/4;
Theta2_d_midpt_FR = -pi/4;
Theta3_d_midpt_FR = 3*pi/4;

% FL
Theta1_d_midpt_FL = -pi/4;
Theta2_d_midpt_FL = pi/4;
Theta3_d_midpt_FL = -3*pi/4;

% BR
Theta1_d_midpt_BR = -pi/4;
Theta2_d_midpt_BR = -pi/4;
Theta3_d_midpt_BR = 3*pi/4;

% BL
Theta1_d_midpt_BL = pi/4;
Theta2_d_midpt_BL = pi/4;
Theta3_d_midpt_BL = -3*pi/4;

% constants for hard stops
% FR
% Theta1_FR_Max = ;
% Theta1_FR_Min = ;
% 
% % FL
% Theta1_FL_Max = ;
% Theta1_FL_Min = ;
% 
% % BR
% Theta1_BR_Max = ;
% Theta1_BR_Min = ;
% 
% % BL
% Theta1_BL_Max = ;
% Theta1_BL_Min = ;

max_body_dist = 0.06; % initialize max bound for moving without steps

% persistent vars
persistent waypoint_toggle; % intialize toggle for determining direction of travel
persistent turn_toggle; % init toggle for determining direction of turning
persistent step_state; % intialize toggle for determining which phase of step leg is in
persistent turn_state; % init toggle for determining which phase of turn robot is in
persistent reached_centroid; % initialize toggle for determining which state the body is in
persistent reached_rest_centroid; % initialize toggle for determining which state the body is in when returning to a four-leg-defined polygon centroid
persistent step_needed; % initialize variable for determining which leg needs to be stepped next
persistent calc_manip; % initialize toggle for determining if manipulability needs to be recalculated
persistent legs_valid;
persistent floor_toggle;
persistent legs_stepped;
persistent leg_reset_needed;
persistent leg_index;
persistent startPoint;
persistent endPoint;
persistent mu;
persistent manip_vec;
persistent startPhi;
persistent endPhi;
persistent r_II_B_0;
persistent is_turning;
persistent turn_needed;
persistent phi_d_temp;
persistent r_II_B_d_temp;
persistent T_I_B_d_temp;
persistent r_II_c_FR_0 r_II_c_FL_0 r_II_c_BR_0 r_II_c_BL_0;
persistent Theta1_d_midpt Theta2_d_midpt Theta3_d_midpt;
persistent Theta1_d_reset Theta2_d_reset Theta3_d_reset;
persistent r_II_c_current;
persistent r_II_c_dstep;
persistent Theta1_d;
persistent Theta2_d;
persistent Theta3_d;

if isempty(waypoint_toggle)
    waypoint_toggle=false;
end

if isempty(turn_toggle)
    turn_toggle=false;
end

if isempty(step_state)
    step_state=uint8(0);
end

if isempty(turn_state)
    turn_state=false;
end

if isempty(reached_centroid)
    reached_centroid=uint8(0);
end

if isempty(reached_rest_centroid)
    reached_rest_centroid = uint8(1);
end

if isempty(step_needed)
    step_needed = uint8(1);
end

if isempty(calc_manip)
    calc_manip = true;
end

if isempty(legs_valid)
    legs_valid=uint8(ones(1,4));
end

if isempty(floor_toggle)
    floor_toggle=legs_valid;
end

if isempty(legs_stepped)
    legs_stepped = uint8(0);
end

if isempty(leg_reset_needed)
    leg_reset_needed = false;
end

if isempty(T_I_B_d_temp)
    T_I_B_d_temp = eye(3,3);
end

if isempty(phi_d_temp)
    phi_d_temp = 0;
end

if isempty(is_turning)
    is_turning = uint8(0);
end

%% Here begins the questionable initialization stuff. SHould be removed if possible

% leg_index=0 will fallthrough and cause function to not do anything
if isempty(leg_index)
    leg_index=uint8(0);
end

if isempty(Theta1_d_midpt)
    Theta1_d_midpt = 0;
end

if isempty(Theta2_d_midpt)
    Theta2_d_midpt = 0;
end

if isempty(Theta3_d_midpt)
    Theta3_d_midpt = 0;
end

if isempty(r_II_c_BL_0)
    r_II_c_BL_0 = zeros(3,1);
end

if isempty(r_II_c_BR_0)
    r_II_c_BR_0 = zeros(3,1);
end

if isempty(r_II_c_FL_0)
    r_II_c_FL_0 = zeros(3,1);
end

if isempty(r_II_c_FR_0)
    r_II_c_FR_0 = zeros(3,1);
end

if isempty(r_II_B_d_temp)
    r_II_B_d_temp = zeros(3,1);
end

if isempty(Theta1_d_reset)
    Theta1_d_reset=0;
end

if isempty(Theta2_d_reset)
    Theta2_d_reset=0;
end

if isempty(Theta3_d_reset)
    Theta3_d_reset=0;
end

if isempty(r_II_c_dstep)
    r_II_c_dstep = zeros(3,1);
end

if isempty(r_II_c_current)
    r_II_c_current = zeros(3,1);
end

if isempty(Theta1_d)
    Theta1_d = zeros(4,1);
end

if isempty(Theta2_d)
    Theta2_d = zeros(4,1);
end

if isempty(Theta3_d)
    Theta3_d = zeros(4,1);
end

if isempty(mu)
    mu = zeros(1,4);
end
if  isempty(manip_vec)
    manip_vec = zeros(1,4);
end

step_error = 0;



% TODO: Figure out how to remove these without breaking body pose controller
% Removing some vars causes errors in body pose controller
% Removing some vars causes a singular matrix
% Removing some vars casues legs to move too far forward
if init_toggle==true
    waypoint_toggle = false; % intialize toggle for determining direction of travel
    turn_toggle = false; % init toggle for determining direction of turning
    step_state = uint8(0); % intialize toggle for determining which phase of step leg is in
    turn_state = false; % init toggle for determining which phase of turn robot is in
    reached_centroid = uint8(0); % initialize toggle for determining which state the body is in
    reached_rest_centroid = uint8(1); % initialize toggle for determining which state the body is in when returning to a four-leg-defined polygon centroid
    step_needed = uint8(1); % initialize variable for determining which leg needs to be stepped next
    calc_manip = true; % initialize toggle for determining if manipulability needs to be recalculated
    legs_valid = uint8(ones(1,4));
    floor_toggle = legs_valid;
    legs_stepped = uint8(0);
    leg_reset_needed = false;
    T_I_B_d_temp = eye(3,3);
    phi_d_temp = 0;
    is_turning = uint8(0);
end

%% Initialize state be the correct size
state = zeros(36,1);

% current
phi = gamma_m(1);
theta = gamma_m(2);
psi = gamma_m(3);
r_II_B = [gamma_m(4);gamma_m(5);gamma_m(6)];
Theta1 = [gamma_m(7);gamma_m(8);gamma_m(9);gamma_m(10)];
Theta2 = [gamma_m(11);gamma_m(12);gamma_m(13);gamma_m(14)];
Theta3 = [gamma_m(15);gamma_m(16);gamma_m(17);gamma_m(18)];
T_I_B = rotz(phi)*roty(theta)*rotx(psi);
state(1:18) = gamma_m(1:18);
% Constants for "resetting" legs
% FR
r_BB_c_reset_FR = [.25;-.25;-r_II_B_d(3)];

% FL
r_BB_c_reset_FL = [.25;.25;-r_II_B_d(3)];

% BR
r_BB_c_reset_BR = [-.25;-.25;-r_II_B_d(3)];

% BL
r_BB_c_reset_BL = [-.25;.25;-r_II_B_d(3)];

% desired
phi_d = Euler_d(1);
theta_d = Euler_d(2);
psi_d = Euler_d(3);
T_I_B_d = rotz(phi_d)*roty(theta_d)*rotx(psi_d);


[r_II_c_FR, r_II_c_FL, r_II_c_BR, r_II_c_BL] = CPos_wrt_I(Theta1,Theta2,Theta3,T_I_B,r_II_B);
r_II_c = [r_II_c_FR,r_II_c_FL,r_II_c_BR,r_II_c_BL];

% Check if step is needed
goal_inside_pgon = inpolygon(r_II_B(1),r_II_B(2),[r_II_c_FL(1) r_II_c_FR(1) r_II_c_BR(1) r_II_c_BL(1)],[r_II_c_FL(2) r_II_c_FR(2) r_II_c_BR(2) r_II_c_BL(2)]);

%% manipulability calculations
% find the least manipulable leg, start stepping that leg. then
% step the next least manipulable leg until the last leg is stepped

%TODO: COnvert to seperate function
% TODO: Why not just run this every time?
if (step_state == 0) && (reached_rest_centroid == 1)
    if (calc_manip == true)
        [muFR, muFL, muBR, muBL] = manipulability(state);
        mu =  [muFR, muFL, muBR, muBL];
        [~, manip_vec] = sort(mu);
        calc_manip = false;
    end
    
    switch step_needed 
        case 1
            leg_index = uint8(manip_vec(1));
            step_needed = uint8(2);
        case 2
            leg_index = uint8(manip_vec(2));
            step_needed = uint8(3);
        case 3
            leg_index = uint8(manip_vec(3));
            step_needed = uint8(4);
        case 4
            leg_index = uint8(manip_vec(4));
            step_needed = uint8(1);
            calc_manip = true;
        otherwise
            if (~coder.target("MATLAB"))
                fprintf("Step_needed is set to an incorrect value")
            end
            error("Step_needed is set to an incorrect value")

    end
end

%% waypoint section
if (waypoint_toggle == false) || (isempty(endPoint))
    startPoint = r_II_B;
    endPoint = r_II_B_d;
    waypoint_toggle = true;
else
    if ~isequal(endPoint, r_II_B_d)
        waypoint_toggle = false;
    end
end

if (turn_toggle == false) || isempty(endPhi)
    startPhi = phi;
    endPhi = phi_d;
    turn_toggle = true;
    r_II_B_0 = r_II_B;
else
    if endPhi ~= phi_d
        turn_toggle = false;
    end
end

%% Turn Needed Algorithm
if (abs(endPhi - startPhi) > pi/15)
    turn_needed = uint8(1);
else
    turn_needed = uint8(0);
end

if turn_needed == 1
    if abs(phi_d - phi) < pi/10
        if (turn_state == false) && (leg_reset_needed == false)
            is_turning = uint8(1);
            phi_d_temp = phi_d;
            leg_reset_needed = false;
            turn_state = true;
        elseif turn_state == true
            phi_error = abs(phi_d - phi);
            if phi_error < 0.05
                turn_state = false;
                startPhi = phi;
                leg_reset_needed = true;   % reset legs
                is_turning = uint8(2);
            end
        end
    else
        % TODO: Replace turn dir calc with if statement to reduce number of
        % variables that are of type double
        turn_dir = sign(phi_d - phi);
        if (turn_state == false) && (leg_reset_needed == false)
            is_turning = uint8(1);
            phi_d_temp = phi + turn_dir*pi/15;
            T_I_B_d_temp = rotz(phi_d_temp)*roty(theta_d)*rotx(psi_d);
            turn_state = true ;
        elseif turn_state == true
            is_turning = uint8(1);
            phi_error = abs(phi_d_temp - phi);
            if phi_error < 0.05
                turn_state = false;
                startPhi = phi;
                leg_reset_needed = true;   % reset legs
            end
        end
    end
    %% Turn Stepping Section
    if leg_reset_needed == true
        % step to reset legs
        if leg_index == 1
            legs_valid(1) = 0;
            if reached_centroid == uint8(0)
                [x,y] = find_pgon_goal(r_II_c_FR,r_II_c_FL,r_II_c_BR,r_II_c_BL,r_II_B,leg_index);
                r_II_B_d_temp = [x;y;r_II_B_d(3)];
            end
            if step_state == 0 % hasn't started stepping yet
                Theta1_d_midpt = Theta1_d_midpt_FR;
                Theta2_d_midpt = Theta2_d_midpt_FR;
                Theta3_d_midpt = Theta3_d_midpt_FR;
                step_state = uint8(1);
                r_II_c_FR_0 = r_II_c_FR;
            elseif step_state == 1 % moving towards midpoint
                step_error = norm([Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)] - [Theta1_d_midpt,Theta2_d_midpt,Theta3_d_midpt]);
                if step_error < 0.2% reached midpoint
                    step_state = uint8(3);
                    [Theta1_d_reset,Theta2_d_reset,Theta3_d_reset] = Leg_Controller_B(r_BB_c_reset_FR, coder.ignoreConst(uint8(1)));
                end
            elseif step_state == 3 % stepping towards goal now
                [Theta1_d_reset,Theta2_d_reset,Theta3_d_reset] = Leg_Controller_B(r_BB_c_reset_FR, coder.ignoreConst(uint8(1)));
                step_error = norm([Theta1_d_reset,Theta2_d_reset,Theta3_d_reset] - [Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)]);
                if (r_II_c_FR(3) <= 0) || (legs_on_gnd(1) == 1) || (manual_step_breakout == 1) || (step_error < 0.005)
                    step_state = uint8(0);
                    legs_valid(1) = 1;
                    leg_index = uint8(0);
                    floor_toggle(1) = 1;
                    reached_centroid = uint8(0);
                    reached_rest_centroid = uint8(0);
                    legs_stepped = legs_stepped + 1;
                end
            end
        elseif leg_index == 2
            legs_valid(2) = 0;
            if reached_centroid == 0
                [x,y] = find_pgon_goal(r_II_c_FR,r_II_c_FL,r_II_c_BR,r_II_c_BL,r_II_B,leg_index);
                r_II_B_d_temp = [x;y;r_II_B_d(3)];
            end
            if step_state == 0 % hasn't started stepping yet
                Theta1_d_midpt = Theta1_d_midpt_FL;
                Theta2_d_midpt = Theta2_d_midpt_FL;
                Theta3_d_midpt = Theta3_d_midpt_FL;
                step_state = uint8(1);
                r_II_c_FL_0 = r_II_c_FL;
            elseif step_state == 1 % moving towards midpoint
                step_error = norm([Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)] - [Theta1_d_midpt,Theta2_d_midpt,Theta3_d_midpt]);
                if step_error < 0.2 % reached midpoint
                    step_state = uint8(3);
                    [Theta1_d_reset,Theta2_d_reset,Theta3_d_reset] = Leg_Controller_B(r_BB_c_reset_FL, coder.ignoreConst(uint8(2)));
                end
            elseif step_state == 3 % stepping towards goal now
                [Theta1_d_reset,Theta2_d_reset,Theta3_d_reset] = Leg_Controller_B(r_BB_c_reset_FL, coder.ignoreConst(uint8(2)));
                step_error = norm([Theta1_d_reset,Theta2_d_reset,Theta3_d_reset] - [Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)]);
                if (r_II_c_FL(3) <= 0)  || (legs_on_gnd(2) == 1) || (manual_step_breakout == 1) || (step_error < 0.005)
                    step_state = uint8(0);
                    legs_valid(2) = 1;
                    leg_index = uint8(0);
                    floor_toggle(2) = 1;
                    reached_centroid = uint8(0);
                    reached_rest_centroid = uint8(0);
                    %if error reached
                    legs_stepped = legs_stepped + 1;
                end
            end
        elseif leg_index == 3
            legs_valid(3) = 0;
            if reached_centroid == 0
                [x,y] = find_pgon_goal(r_II_c_FR,r_II_c_FL,r_II_c_BR,r_II_c_BL,r_II_B,leg_index);
                r_II_B_d_temp = [x;y;r_II_B_d(3)];
            end
            if step_state == 0 % hasn't started stepping yet
                Theta1_d_midpt = Theta1_d_midpt_BR;
                Theta2_d_midpt = Theta2_d_midpt_BR;
                Theta3_d_midpt = Theta3_d_midpt_BR;
                step_state = uint8(1);
                r_II_c_BR_0 = r_II_c_BR;
            elseif step_state == 1 % moving towards midpoint
                step_error = norm([Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)] - [Theta1_d_midpt,Theta2_d_midpt,Theta3_d_midpt]);
                if step_error < 0.2% reached midpoint
                    step_state = uint8(3);
                    [Theta1_d_reset,Theta2_d_reset,Theta3_d_reset] = Leg_Controller_B(r_BB_c_reset_BR, coder.ignoreConst(uint8(3)));
                end
            elseif step_state == 3 % stepping towards goal now
                [Theta1_d_reset,Theta2_d_reset,Theta3_d_reset] = Leg_Controller_B(r_BB_c_reset_BR, coder.ignoreConst(uint8(3)));
                step_error = norm([Theta1_d_reset,Theta2_d_reset,Theta3_d_reset] - [Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)]);
                if (r_II_c_BR(3) <= 0)  || (legs_on_gnd(3) == 1) || (manual_step_breakout == 1) || (step_error < 0.005)
                    legs_valid(3) = 1;
                    step_state = uint8(0);
                    leg_index = uint8(0);
                    floor_toggle(3) = 1;
                    reached_centroid = uint8(0);
                    reached_rest_centroid = uint8(0);
                    %if error reached
                    legs_stepped = legs_stepped + 1;
                end
            end
        elseif leg_index == 4
            legs_valid(4) = 0;
            if reached_centroid == 0
                [x,y] = find_pgon_goal(r_II_c_FR,r_II_c_FL,r_II_c_BR,r_II_c_BL,r_II_B,leg_index);
                r_II_B_d_temp = [x;y;r_II_B_d(3)];
            end
            if step_state == 0 % hasn't started stepping yet
                Theta1_d_midpt = Theta1_d_midpt_BL;
                Theta2_d_midpt = Theta2_d_midpt_BL;
                Theta3_d_midpt = Theta3_d_midpt_BL;
                step_state = uint8(1);
                r_II_c_BL_0 = r_II_c_BL;
            elseif step_state == 1 % moving towards midpoint
                step_error = norm([Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)] - [Theta1_d_midpt,Theta2_d_midpt,Theta3_d_midpt]);
                if step_error < 0.2% reached midpoint
                    step_state = uint8(3);
                    [Theta1_d_reset,Theta2_d_reset,Theta3_d_reset] = Leg_Controller_B(r_BB_c_reset_BL, coder.ignoreConst(uint8(4)));
                end
            elseif step_state == 3 % stepping towards goal now
                [Theta1_d_reset,Theta2_d_reset,Theta3_d_reset] = Leg_Controller_B(r_BB_c_reset_BL, coder.ignoreConst(uint8(4)));
                step_error = norm([Theta1_d_reset,Theta2_d_reset,Theta3_d_reset] - [Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)]);
                if (r_II_c_BL(3) <= 0) || (legs_on_gnd(4) == 1) || (manual_step_breakout == 1) || (step_error < 0.005)
                    step_state = uint8(0);
                    legs_valid(4) = 1;
                    leg_index = uint8(0);
                    floor_toggle(4) = 1;
                    reached_centroid = uint8(0);
                    reached_rest_centroid = uint8(0);
                    %if error reached
                    legs_stepped = legs_stepped + 1;
                end
            end
        end
        if legs_stepped == 4
            legs_stepped = uint8(0);
            leg_reset_needed = false;
            if is_turning == 2
                is_turning = uint8(0);
                turn_needed = uint8(0);
            end
        end
    end
    
    %% walking forward section
else
    if (norm(r_II_B_d - r_II_B) >= max_body_dist)  || (step_state ~= 0) || (~goal_inside_pgon)
        % determine direction of travel
        % the direction of travel is computed at the very beginning or when
        % the desired body position changes
        if waypoint_toggle == false
            startPoint = r_II_B;
            endPoint = r_II_B_d;
            waypoint_toggle = true;
        else
            if endPoint ~= r_II_B_d
                waypoint_toggle = false;
            end
        end
        %% leg stepping algorithm
        % FR leg needs to step
        if leg_index == 1
            legs_valid(1) = 0;
            if reached_centroid == 0
                [x,y] = find_pgon_goal(r_II_c_FR,r_II_c_FL,r_II_c_BR,r_II_c_BL,r_II_B,leg_index);
                r_II_B_d_temp = [x;y;r_II_B_d(3)];
            end
            if step_state == 0 % hasn't started stepping yet
                Theta1_d_midpt = Theta1_d_midpt_FR;
                Theta2_d_midpt = Theta2_d_midpt_FR;
                Theta3_d_midpt = Theta3_d_midpt_FR;
                step_state = uint8(1);
                r_II_c_FR_0 = r_II_c_FR;
            elseif step_state == 1 % moving towards midpoint
                step_error = norm([Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)] - [Theta1_d_midpt,Theta2_d_midpt,Theta3_d_midpt]);
                if step_error < 0.2% reached midpoint
                    step_state = uint8(2);
                    r_II_c_dstep = step_planner_intelligent(startPoint, endPoint, r_II_c_FR_0, step_dist);
                    r_II_c_current = r_II_c_FR_0;
                end
            elseif step_state == 2 % stepping towards goal now
                step_error = norm(r_II_c_dstep - r_II_c_FR);
                if (r_II_c_FR(3) <= 0)  || (legs_on_gnd(1) == 1) || (manual_step_breakout == 1) || (step_error < 0.005)
                    step_state = uint8(0);
                    legs_valid(1) = 1;
                    leg_index = uint8(0);
                    floor_toggle(1) = 1;
                    reached_centroid = uint8(0);
                    reached_rest_centroid = uint8(0);
                end
            end
        elseif leg_index == 2
            legs_valid(2) = 0;
            if reached_centroid == 0
                [x,y] = find_pgon_goal(r_II_c_FR,r_II_c_FL,r_II_c_BR,r_II_c_BL,r_II_B,leg_index);
                r_II_B_d_temp = [x;y;r_II_B_d(3)];
            end
            if step_state == 0 % hasn't started stepping yet
                Theta1_d_midpt = Theta1_d_midpt_FL;
                Theta2_d_midpt = Theta2_d_midpt_FL;
                Theta3_d_midpt = Theta3_d_midpt_FL;
                step_state = uint8(1);
                r_II_c_FL_0 = r_II_c_FL;
            elseif step_state == 1 % moving towards midpoint
                step_error = norm([Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)] - [Theta1_d_midpt,Theta2_d_midpt,Theta3_d_midpt]);
                if step_error < 0.2 % reached midpoint
                    step_state = uint8(2);
                    r_II_c_dstep = step_planner_intelligent(startPoint, endPoint, r_II_c_FL_0, step_dist);
                    r_II_c_current = r_II_c_FL_0;
                end
            elseif step_state == 2 % stepping towards goal now
                step_error = norm(r_II_c_dstep - r_II_c_FL);
                if (r_II_c_FL(3) <= 0)  || (legs_on_gnd(2) == 1) || (manual_step_breakout == 1) || (step_error < 0.005)
                    step_state = uint8(0);
                    legs_valid(2) = 1;
                    leg_index = uint8(0);
                    floor_toggle(2) = 1;
                    reached_centroid = uint8(0);
                    reached_rest_centroid = uint8(0);
                end
            end
        elseif leg_index == 3
            legs_valid(3) = 0;
            if reached_centroid == 0
                [x,y] = find_pgon_goal(r_II_c_FR,r_II_c_FL,r_II_c_BR,r_II_c_BL,r_II_B,leg_index);
                r_II_B_d_temp = [x;y;r_II_B_d(3)];
            end
            if step_state == 0 % hasn't started stepping yet
                Theta1_d_midpt = Theta1_d_midpt_BR;
                Theta2_d_midpt = Theta2_d_midpt_BR;
                Theta3_d_midpt = Theta3_d_midpt_BR;
                step_state = uint8(1);
                r_II_c_BR_0 = r_II_c_BR;
            elseif step_state == 1 % moving towards midpoint
                step_error = norm([Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)] - [Theta1_d_midpt,Theta2_d_midpt,Theta3_d_midpt]);
                if step_error < 0.2% reached midpoint
                    step_state = uint8(2);
                    r_II_c_dstep = step_planner_intelligent(startPoint, endPoint, r_II_c_BR_0, step_dist);
                    r_II_c_current = r_II_c_BR_0;
                end
            elseif step_state == 2 % stepping towards goal now
                step_error = norm(r_II_c_dstep - r_II_c_BR);
                if (r_II_c_BR(3) <= 0)  || (legs_on_gnd(3) == 1) || (manual_step_breakout == 1) || (step_error < 0.005)
                    legs_valid(3) = 1;
                    step_state = uint8(0);
                    leg_index = uint8(0);
                    floor_toggle(3) = 1;
                    reached_centroid = uint8(0);
                    reached_rest_centroid = uint8(0);
                end
            end
        elseif leg_index == 4
            legs_valid(4) = 0;
            if reached_centroid == 0
                [x,y] = find_pgon_goal(r_II_c_FR,r_II_c_FL,r_II_c_BR,r_II_c_BL,r_II_B,leg_index);
                r_II_B_d_temp = [x;y;r_II_B_d(3)];
            end
            if step_state == 0 % hasn't started stepping yet
                Theta1_d_midpt = Theta1_d_midpt_BL;
                Theta2_d_midpt = Theta2_d_midpt_BL;
                Theta3_d_midpt = Theta3_d_midpt_BL;
                step_state = uint8(1);
                r_II_c_BL_0 = r_II_c_BL;
            elseif step_state == 1 % moving towards midpoint
                step_error = norm([Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)] - [Theta1_d_midpt,Theta2_d_midpt,Theta3_d_midpt]);
                if step_error < 0.2% reached midpoint
                    step_state = uint8(2);
                    r_II_c_dstep = step_planner_intelligent(startPoint, endPoint, r_II_c_BL_0, step_dist);
                    r_II_c_current = r_II_c_BL_0;
                end
            elseif step_state == 2 % stepping towards goal now
                 step_error = norm(r_II_c_dstep - r_II_c_BL);
                if (r_II_c_BL(3) <= 0)  || (legs_on_gnd(4) == 1) || (manual_step_breakout == 1) || (step_error < 0.005)
                    step_state = uint8(0);
                    legs_valid(4) = 1;
                    leg_index = uint8(0);
                    floor_toggle(4) = 1;
                    reached_centroid = uint8(0);
                    reached_rest_centroid = uint8(0);
                end
            end
        end
    else
        % move with normal body pose controller
        [Theta1_d,Theta2_d,Theta3_d, r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d,r_II_B_d,r_II_B,floor_toggle);
    end
end

%% centroid moving/balance section, and commanding step (assigning thetad's)
if reached_rest_centroid == 0 % needs to move back to resting 4-legged position to find new leg to move
    [x,y] = centroid_codeGen([r_II_c_FR(1), r_II_c_FL(1), r_II_c_BL(1), r_II_c_BR(1)],[r_II_c_FR(2), r_II_c_FL(2), r_II_c_BL(2), r_II_c_BR(2)]);
    r_II_B_d_temp = [x;y;r_II_B_d(3)];
    [Theta1_d,Theta2_d,Theta3_d,r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d_temp,r_II_B_d_temp,r_II_B,uint8([1,1,1,1]));
    reached_rest_centroid = uint8(2);
elseif reached_rest_centroid == 2 % moving towards resting, or inbetween-step body pose
    body_error = norm(r_II_B - r_II_B_d_temp);
    [Theta1_d,Theta2_d,Theta3_d,r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d_temp,r_II_B_d_temp,r_II_B,floor_toggle);
    if body_error < 0.01
        reached_rest_centroid = uint8(1);
    end
elseif reached_rest_centroid == 1
    waypoint_toggle = false;
    % rockback before step
     if ~all(legs_valid)
        if reached_centroid == 0 % hasn't started moving towards centroid yet
            [Theta1_d,Theta2_d,Theta3_d,r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d_temp,r_II_B_d_temp,r_II_B,floor_toggle);
            reached_centroid = uint8(2);
        elseif reached_centroid == 2 % moving towards centroid
            body_error = norm(r_II_B - r_II_B_d_temp);
            if body_error < 0.01
                reached_centroid = uint8(1);
            end
        elseif reached_centroid == 1 % step
            % step leg
            [Theta1_d,Theta2_d,Theta3_d,r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d_temp,r_II_B_d_temp,r_II_B,floor_toggle);
            if (step_state == 1) && (reached_centroid == 1)
                Theta1_d(leg_index) = Theta1_d_midpt;
                Theta2_d(leg_index) = Theta2_d_midpt;
                Theta3_d(leg_index) = Theta3_d_midpt;
            elseif step_state == 2
                [Theta1_d(leg_index), Theta2_d(leg_index), Theta3_d(leg_index),r_II_c_dstep] = Leg_Controller(r_II_c_dstep, r_II_c_current, T_I_B, r_II_B, leg_index);
            elseif step_state == 3
                Theta1_d(leg_index) = Theta1_d_reset;
                Theta2_d(leg_index) = Theta2_d_reset;
                Theta3_d(leg_index) = Theta3_d_reset;
            end
%             if step_error <= 0.03
%                 reached_centroid = uint8(0);
%                 reached_rest_centroid = uint8(0);
%             end
        end
    else
        r_II_B_d_temp = [r_II_B(1),r_II_B(2),r_II_B_d(3)].';
        [Theta1_d,Theta2_d,Theta3_d,r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d_temp,r_II_B_d_temp,r_II_B,floor_toggle);
    end
end
floor_toggle_out = floor_toggle;
legs_valid_out = legs_valid;
%Theta_d = [Theta1_d,Theta2_d,Theta3_d]; % output
% Euler_d = [phi_d_temp;theta_d;psi_d];
phi_d_temp_out = phi_d_temp;
r_II_B_d_temp_out = r_II_B_d_temp;
Theta1_d_out = Theta1_d;

Theta2_d_out = Theta2_d;

Theta3_d_out = Theta3_d;
end

