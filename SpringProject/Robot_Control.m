function Theta_d = Robot_Control(r_II_B_d, Euler_d, gamma_m)
%Controls Robot's walking algorithm
%   input: r_II_B_d, Euler_d, gamma_m
%   output: Theta_d
%   r_II_B_d = [x_d;y_d;z_d];
%   Euler_d = [phi,theta,psi];
%   gamma_m = [Euler,r_II_B,Theta1,Theta2,Theta3];
%   Euler = [phi,theta,psi];
%   r_II_B = [x,y,z];
%   Theta1 = [Theta1FR,FL,BR,BL];
%   Theta2 = [Theta2FR,FL,BR,BL];
%   Theta3 = [Theta3FR,FL,BR,BL];

% Variable Init
step_dist = 0.12; % leg step distance

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

% Constants for "resetting" legs
% FR
Theta1_d_reset_FR = pi/4;
Theta2_d_reset_FR = pi/6;
Theta3_d_reset_FR = pi/3;

% FL
Theta1_d_reset_FL = -pi/4;
Theta2_d_reset_FL = -pi/6;
Theta3_d_reset_FL = -pi/3;

% BR
Theta1_d_reset_BR = -pi/4;
Theta2_d_reset_BR = pi/6;
Theta3_d_reset_BR = pi/3;

% BL
Theta1_d_reset_BL = pi/4;
Theta2_d_reset_BL = -pi/6;
Theta3_d_reset_BL = -pi/3;

max_body_dist = 0.1; % initialize max bound for moving without steps

% persistent vars
persistent hi;
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

if isempty(hi)
    hi = 1;
    waypoint_toggle = 0; % intialize toggle for determining direction of travel
    turn_toggle = 0; % init toggle for determining direction of turning
    step_state = 0; % intialize toggle for determining which phase of step leg is in
    turn_state = 0; % init toggle for determining which phase of turn robot is in
    reached_centroid = 0; % initialize toggle for determining which state the body is in
    reached_rest_centroid = 1; % initialize toggle for determining which state the body is in when returning to a four-leg-defined polygon centroid
    step_needed = 1; % initialize variable for determining which leg needs to be stepped next
    calc_manip = 1; % initialize toggle for determining if manipulability needs to be recalculated
    legs_valid = [1,1,1,1];
    floor_toggle = legs_valid;
    legs_stepped = 0;
    leg_reset_needed = 0;
end

% current
phi = gamma_m(1);
theta = gamma_m(2);
psi = gamma_m(3);
r_II_B = [gamma_m(4);gamma_mb(5);gamma_m(6)];
Theta1 = [gamma_m(7);gamma_m(8);gamma_m(9);gamma_m(10)];
Theta2 = [gamma_m(11);gamma_m(12);gamma_m(13);gamma_m(14)];
Theta3 = [gamma_m(15);gamma_m(16);gamma_m(17);gamma_m(18)];
T_I_B = rotz(phi)*roty(theta)*rotx(psi);
state = b(1:18);

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
if (step_state == 0) && (reached_rest_centroid == 1)
    if calc_manip == 1
        [muFR, muFL, muBR, muBL] = manipulability(state);
        mu =  [muFR, muFL, muBR, muBL];
        manip_vec = sort(mu);
        calc_manip = 0;
    end
    if step_needed == 1
        leg_index = find(manip_vec(1) == mu);
        step_needed = 2;
    elseif step_needed == 2
        leg_index = find(manip_vec(2) == mu);
        step_needed = 3;
    elseif step_needed == 3
        leg_index = find(manip_vec(3) == mu);
        step_needed = 4;
    elseif step_needed == 4
        leg_index = find(manip_vec(4) == mu);
        step_needed = 1;
        calc_manip = 1;
    end
end

%% waypoint section
if waypoint_toggle == 0
    startPoint = r_II_B;
    endPoint = r_II_B_d;
    waypoint_toggle = 1;
elseif waypoint_toggle == 1
    if endPoint ~= r_II_B_d
        waypoint_toggle = 0;
    end
end

if turn_toggle == 0
    startPhi = phi;
    endPhi = phi_d(ii);
    turn_toggle = 1;
    r_II_B_0 = r_II_B;
elseif turn_toggle == 1
    if endPhi ~= phi_d(ii)
        turn_toggle = 0;
    end
end

%% Turn Needed Algorithm
if (abs(endPhi - startPhi) > pi/10)
    turn_needed = 1;
elseif is_turning == 0
    turn_needed = 0;
end

if turn_needed == 1
    if abs(phi_d(ii) - phi) < pi/10
        if (turn_state == 0) && (leg_reset_needed == 0)
            is_turning = 1;
            phi_d_temp = phi_d(ii);
            leg_reset_needed = 0;
            turn_state = 1;
        elseif turn_state == 1
            phi_error = abs(phi_d(ii) - phi);
            if phi_error < 0.05
                turn_state = 0;
                startPhi = phi;
                leg_reset_needed = 1;   % reset legs
                is_turning = 2;
            end
        end
    else
        turn_dir = sign(phi_d(ii) - phi);
        if (turn_state == 0) && (leg_reset_needed == 0)
            is_turning = 1;
            phi_d_temp = phi + turn_dir*pi/10;
            T_I_B_d_temp = rotz(phi_d_temp)*roty(theta_d(ii))*rotx(psi_d(ii));
            turn_state = 1 ;
        elseif turn_state == 1
            is_turning = 1;
            phi_error = abs(phi_d_temp - phi);
            if phi_error < 0.05
                turn_state = 0;
                startPhi = phi;
                leg_reset_needed = 1;   % reset legs
            end
        end
    end
    %% Turn Stepping Section
    if leg_reset_needed == 1
        % step to reset legs
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
                step_state = 1;
                r_II_c_FR_0 = r_II_c_FR;
            elseif step_state == 1 % moving towards midpoint
                step_error = norm([Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)] - [Theta1_d_midpt,Theta2_d_midpt,Theta3_d_midpt]);
                if step_error < 0.2% reached midpoint
                    step_state = 3;
                    Theta1_d_reset = Theta1_d_reset_FR;
                    Theta2_d_reset = Theta2_d_reset_FR;
                    Theta3_d_reset = Theta3_d_reset_FR;
                end
            elseif step_state == 3 % stepping towards goal now
                Theta1_d_reset = Theta1_d_reset_FR;
                Theta2_d_reset = Theta2_d_reset_FR;
                Theta3_d_reset = Theta3_d_reset_FR;
                if r_II_c_FR(3) <= 0
                    step_state = 0;
                    legs_valid(1) = 1;
                    leg_index = 0;
                    floor_toggle(1) = 1;
                    reached_centroid = 0;
                    reached_rest_centroid = 0;
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
                step_state = 1;
                r_II_c_FL_0 = r_II_c_FL;
            elseif step_state == 1 % moving towards midpoint
                step_error = norm([Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)] - [Theta1_d_midpt,Theta2_d_midpt,Theta3_d_midpt]);
                if step_error < 0.2 % reached midpoint
                    step_state = 3;
                    Theta1_d_reset = Theta1_d_reset_FL;
                    Theta2_d_reset = Theta2_d_reset_FL;
                    Theta3_d_reset = Theta3_d_reset_FL;
                end
            elseif step_state == 3 % stepping towards goal now
                Theta1_d_reset = Theta1_d_reset_FL;
                Theta2_d_reset = Theta2_d_reset_FL;
                Theta3_d_reset = Theta3_d_reset_FL;
                if r_II_c_FL(3) <= 0
                    step_state = 0;
                    legs_valid(2) = 1;
                    leg_index = 0;
                    floor_toggle(2) = 1;
                    reached_centroid = 0;
                    reached_rest_centroid = 0;
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
                step_state = 1;
                r_II_c_BR_0 = r_II_c_BR;
            elseif step_state == 1 % moving towards midpoint
                step_error = norm([Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)] - [Theta1_d_midpt,Theta2_d_midpt,Theta3_d_midpt]);
                if step_error < 0.2% reached midpoint
                    step_state = 3;
                    Theta1_d_reset = Theta1_d_reset_BR;
                    Theta2_d_reset = Theta2_d_reset_BR;
                    Theta3_d_reset = Theta3_d_reset_BR;
                end
            elseif step_state == 3 % stepping towards goal now
                Theta1_d_reset = Theta1_d_reset_BR;
                Theta2_d_reset = Theta2_d_reset_BR;
                Theta3_d_reset = Theta3_d_reset_BR;
                if r_II_c_BR(3) <= 0
                    legs_valid(3) = 1;
                    step_state = 0;
                    leg_index = 0;
                    floor_toggle(3) = 1;
                    reached_centroid = 0;
                    reached_rest_centroid = 0;
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
                step_state = 1;
                r_II_c_BL_0 = r_II_c_BL;
            elseif step_state == 1 % moving towards midpoint
                step_error = norm([Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)] - [Theta1_d_midpt,Theta2_d_midpt,Theta3_d_midpt]);
                if step_error < 0.2% reached midpoint
                    step_state = 3;
                    Theta1_d_reset = Theta1_d_reset_BL;
                    Theta2_d_reset = Theta2_d_reset_BL;
                    Theta3_d_reset = Theta3_d_reset_BL;
                end
            elseif step_state == 3 % stepping towards goal now
                Theta1_d_reset = Theta1_d_reset_BL;
                Theta2_d_reset = Theta2_d_reset_BL;
                Theta3_d_reset = Theta3_d_reset_BL;
                if r_II_c_BL(3) <= 0
                    step_state = 0;
                    legs_valid(4) = 1;
                    leg_index = 0;
                    floor_toggle(4) = 1;
                    reached_centroid = 0;
                    reached_rest_centroid = 0;
                    %if error reached
                    legs_stepped = legs_stepped + 1;
                end
            end
        end
        if legs_stepped == 4
            legs_stepped = 0;
            leg_reset_needed = 0;
            if is_turning == 2
                is_turning = 0;
                turn_needed = 0;
            end
        end
    end
    
    %% walking forward section
elseif turn_needed == 0
    if (norm(r_II_B_d - r_II_B) >= max_body_dist)  || (step_state ~= 0) || (~goal_inside_pgon)
        % determine direction of travel
        % the direction of travel is computed at the very beginning or when
        % the desired body position changes
        if waypoint_toggle == 0
            startPoint = r_II_B;
            endPoint = r_II_B_d;
            waypoint_toggle = 1;
        elseif waypoint_toggle == 1
            if endPoint ~= r_II_B_d
                waypoint_toggle = 0;
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
                step_state = 1;
                r_II_c_FR_0 = r_II_c_FR;
            elseif step_state == 1 % moving towards midpoint
                step_error = norm([Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)] - [Theta1_d_midpt,Theta2_d_midpt,Theta3_d_midpt]);
                if step_error < 0.2% reached midpoint
                    step_state = 2;
                    r_II_c_dstep = step_planner_intelligent(startPoint, endPoint, r_II_c_FR_0, step_dist);
                    r_II_c_current = r_II_c_FR_0;
                end
            elseif step_state == 2 % stepping towards goal now
                %                 step_error = norm(r_II_c_FR - r_II_c_dstep);
                if r_II_c_FR(3) <= 0
                    step_state = 0;
                    legs_valid(1) = 1;
                    leg_index = 0;
                    floor_toggle(1) = 1;
                    reached_centroid = 0;
                    reached_rest_centroid = 0;
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
                step_state = 1;
                r_II_c_FL_0 = r_II_c_FL;
            elseif step_state == 1 % moving towards midpoint
                step_error = norm([Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)] - [Theta1_d_midpt,Theta2_d_midpt,Theta3_d_midpt]);
                if step_error < 0.2 % reached midpoint
                    step_state = 2;
                    r_II_c_dstep = step_planner_intelligent(startPoint, endPoint, r_II_c_FL_0, step_dist);
                    r_II_c_current = r_II_c_FL_0;
                end
            elseif step_state == 2 % stepping towards goal now
                if r_II_c_FL(3) <= 0
                    step_state = 0;
                    legs_valid(2) = 1;
                    leg_index = 0;
                    floor_toggle(2) = 1;
                    reached_centroid = 0;
                    reached_rest_centroid = 0;
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
                step_state = 1;
                r_II_c_BR_0 = r_II_c_BR;
            elseif step_state == 1 % moving towards midpoint
                step_error = norm([Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)] - [Theta1_d_midpt,Theta2_d_midpt,Theta3_d_midpt]);
                if step_error < 0.2% reached midpoint
                    step_state = 2;
                    r_II_c_dstep = step_planner_intelligent(startPoint, endPoint, r_II_c_BR_0, step_dist);
                    r_II_c_current = r_II_c_BR_0;
                end
            elseif step_state == 2 % stepping towards goal now
                if r_II_c_BR(3) <= 0
                    legs_valid(3) = 1;
                    step_state = 0;
                    leg_index = 0;
                    floor_toggle(3) = 1;
                    reached_centroid = 0;
                    reached_rest_centroid = 0;
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
                step_state = 1;
                r_II_c_BL_0 = r_II_c_BL;
            elseif step_state == 1 % moving towards midpoint
                step_error = norm([Theta1(leg_index),Theta2(leg_index),Theta3(leg_index)] - [Theta1_d_midpt,Theta2_d_midpt,Theta3_d_midpt]);
                if step_error < 0.2% reached midpoint
                    step_state = 2;
                    r_II_c_dstep = step_planner_intelligent(startPoint, endPoint, r_II_c_BL_0, step_dist);
                    r_II_c_current = r_II_c_BL_0;
                end
            elseif step_state == 2 % stepping towards goal now
                if r_II_c_BL(3) <= 0
                    step_state = 0;
                    legs_valid(4) = 1;
                    leg_index = 0;
                    floor_toggle(4) = 1;
                    reached_centroid = 0;
                    reached_rest_centroid = 0;
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
    pgon = polyshape([r_II_c_FR(1), r_II_c_FL(1), r_II_c_BL(1), r_II_c_BR(1)],[r_II_c_FR(2), r_II_c_FL(2), r_II_c_BL(2), r_II_c_BR(2)]);
    [x,y] = centroid(pgon);
    r_II_B_d_temp = [x;y;r_II_B_d(3)];
    [Theta1_d,Theta2_d,Theta3_d,r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d_temp,r_II_B_d_temp,r_II_B,[1,1,1,1]);
    reached_rest_centroid = 2;
elseif reached_rest_centroid == 2 % moving towards resting, or inbetween-step body pose
    body_error = norm(r_II_B - r_II_B_d_temp);
    [Theta1_d,Theta2_d,Theta3_d,r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d_temp,r_II_B_d_temp,r_II_B,floor_toggle);
    if body_error < 0.01
        reached_rest_centroid = 1;
    end
elseif reached_rest_centroid == 1
    waypoint_toggle = 0;
    % rockback before step
    if ~isempty(find(legs_valid == 0))
        if reached_centroid == 0 % hasn't started moving towards centroid yet
            [Theta1_d,Theta2_d,Theta3_d,r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d_temp,r_II_B_d_temp,r_II_B,floor_toggle);
            reached_centroid = 2;
        elseif reached_centroid == 2 % moving towards centroid
            body_error = norm(r_II_B - r_II_B_d_temp);
            if body_error < 0.01
                reached_centroid = 1;
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
            if step_error <= 0.03
                reached_centroid = 0;
                reached_rest_centroid = 0;
            end
        end
    else
        r_II_B_d_temp = r_II_B;
        [Theta1_d,Theta2_d,Theta3_d,r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d_temp,r_II_B_d_temp,r_II_B,floor_toggle);
    end
end


Theta_d = [Theta1_d,Theta2_d,Theta3_d]; % output
end

