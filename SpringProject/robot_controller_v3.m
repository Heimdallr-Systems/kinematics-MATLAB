clear
clc
close all

h = 0.0001; % time step
t = 0:h:10; % time vector % 6 sec %14

step_dist = 0.12; % leg step distance

legs_valid_array = zeros(length(t),4);

r_II_c_FR_d = 0;
r_II_c_FL_d = 0;
r_II_c_BR_d = 0;
r_II_c_BL_d = 0;

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

% Constants for "resetting legs"
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


%%% Initial Conditions %%%
b=zeros(36,length(t)); % state matrix
b(4,1) = 0;
b(5,1) = 0;
b(6,1) = 0.25; % body height % default 0.18 % 0.3
% first joint angles
b(7,1) = pi/4;
b(8,1) = -pi/4;
b(9,1) = -pi/4;
b(10,1) = pi/4;
% second joint angles
b(11,1) = pi/6+0.1; %pi/4 %0
b(12,1) = -pi/6;
b(13,1) = pi/6;
b(14,1) = -pi/6;
% third joint angles
b(15,1) = pi/3; %pi/8 %pi/2
b(16,1) = -pi/3;
b(17,1) = pi/3;
b(18,1) = -pi/3;

rcm = zeros(3,length(t)); % initialize rcm

Fgamma= zeros(18,length(t)); % initialize forces matrix (joint space)

legs_on_gnd = [1,1,1,1]; % initialize logical leg conditions

max_body_dist = 0.2; % initialize max bound for moving without steps

waypoint_toggle = 0; % intialize toggle for determining direction of travel


turn_toggle = 0; % init toggle for determining direction of turning

step_state = 0; % intialize toggle for determining which phase of step leg is in

turn_state = 0; % init toggle for determining which phase of turn robot is in

is_left_leg = -1; % intitialize toggle for controlling leg, if its a right or left one

reached_centroid = 0; % initialize toggle for determining which state the body is in

reached_rest_centroid = 1; % initialize toggle for determining which state the body is in when returning to a four-leg-defined polygon centroid

step_needed = 1; % initialize variable for determining which leg needs to be stepped next
calc_manip = 1; % initialize toggle for determining if manipulability needs to be recalculated

legs_valid = [1,1,1,1];

floor_toggle = legs_valid;

legs_stepped = 0;

leg_reset_needed = 0;

%%% Desired Body Pose Trajectory %%%
x_d = 1*ones(1,length(t));
y_d = 0.*ones(1,length(x_d));
z_d = 0.245*ones(1,length(x_d));
phi_d = pi/2.*ones(1,length(x_d));
theta_d =  zeros(1,length(x_d));
psi_d =  zeros(1,length(x_d));

% Control Constants
Kd=6;
Kp=30;

%%% Floor Definition %%%
Kp_floor = -5000; % floor spring constant
Kd_floor = -800; % floor damping coefficient
b_fric_floor = -2000; % floor coefficient of lateral, viscous friction

% Numerically Integrate for Position of Manipulator

for ii = 1:length(t)
   %% Named Vectors of State Values %%
    r_II_B(:,1) = [b(4,ii);b(5,ii);b(6,ii)];
    Theta1(:,1) = [b(7,ii);b(8,ii);b(9,ii);b(10,ii)];
    Theta2(:,1) = [b(11,ii);b(12,ii);b(13,ii);b(14,ii)];
    Theta3(:,1) = [b(15,ii);b(16,ii);b(17,ii);b(18,ii)];
    Theta = [Theta1;Theta2;Theta3];
    dotTheta1(:,1) = [b(25,ii);b(26,ii);b(27,ii);b(28,ii)];
    dotTheta2(:,1) = [b(29,ii);b(30,ii);b(31,ii);b(32,ii)];
    dotTheta3(:,1) = [b(33,ii);b(34,ii);b(35,ii);b(36,ii)];
    dotTheta(:,1) = [dotTheta1;dotTheta2;dotTheta3];
    
    phi = b(1,ii);
    theta = b(2,ii);
    psi = b(3,ii);
    T_I_B = rotz(b(1,ii))*roty(b(2,ii))*rotx(b(3,ii));
    T_I_B_d = rotz(phi_d(ii))*roty(theta_d(ii))*rotx(psi_d(ii));
    r_II_B_d(:,1) = [x_d(ii);y_d(ii);z_d(ii)];
    state = b(1:18,ii);
    
    [r_II_c_FR, r_II_c_FL, r_II_c_BR, r_II_c_BL] = CPos_wrt_I(Theta1,Theta2,Theta3,T_I_B,r_II_B);
    r_II_c = [r_II_c_FR,r_II_c_FL,r_II_c_BR,r_II_c_BL];
    
    % Check what legs are on gnd %
    legs_on_gnd = [r_II_c_FR(3) <= 0.0001; r_II_c_FL(3) <= 0.0001; r_II_c_BR(3) <= 0.0001; r_II_c_BL(3) <= 0.0001];
    
    % Check if step is needed
    % if the desired body position is more than a leg's length away from
    % any contact point
    d_B_dis_FR = norm(r_II_B_d - r_II_c_FR);
    d_B_dis_FL = norm(r_II_B_d - r_II_c_FL);
    d_B_dis_BR = norm(r_II_B_d - r_II_c_BR);
    d_B_dis_BL = norm(r_II_B_d - r_II_c_BL);
    
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
%                 % move with normal body pose controller
%                 [Theta1_d,Theta1_2_d,Theta2_1_d,Theta2_2_d,Theta2_3_d,Theta2_4_d,Theta3_1_d,Theta3_2_d,Theta3_3_d,Theta3_4_d,r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d,r_II_B_d,r_II_B,floor_toggle);
%                 % if theta1 wraps around into robot
%                 T1_cond(1) = (Theta1_d(1) <= -pi/2) || (Theta1_d(1) >= pi); % FR
%                 T1_cond(2) = (Theta1_d(2) <= -pi) || (Theta1_d(2) >= pi/2); %FL
%                 T1_cond(3) = (Theta1_d(3) <= -pi) || (Theta1_d(3) >= pi/2); % BR
%                 T1_cond(4) = (Theta1_d(4) <= -pi/2) || (Theta1_d(4) >= pi); % BL
%                 for hh = 1:1:4
%                     if T1_cond(hh)
%                         Theta1_d(hh,1) = Theta1_2_d(hh);
%                         Theta2_d(hh,1) = Theta2_4_d(hh);
%                         Theta3_d(hh,1) = Theta3_4_d(hh);
%                     else
%                         Theta2_d(hh,1) = Theta2_2_d(hh);
%                         Theta3_d(hh,1) = Theta3_2_d(hh);
%                     end
%                 end
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
                
%                 % move with normal body pose controller
%                 [Theta1_d,Theta1_2_d,Theta2_1_d,Theta2_2_d,Theta2_3_d,Theta2_4_d,Theta3_1_d,Theta3_2_d,Theta3_3_d,Theta3_4_d,r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d_temp,r_II_B_d,r_II_B,floor_toggle);
%                 % if theta1 wraps around into robot
%                 T1_cond(1) = (Theta1_d(1) <= -pi/2) || (Theta1_d(1) >= pi); % FR
%                 T1_cond(2) = (Theta1_d(2) <= -pi) || (Theta1_d(2) >= pi/2); %FL
%                 T1_cond(3) = (Theta1_d(3) <= -pi) || (Theta1_d(3) >= pi/2); % BR
%                 T1_cond(4) = (Theta1_d(4) <= -pi/2) || (Theta1_d(4) >= pi); % BL
%                 for hh = 1:1:4
%                     if T1_cond(hh)
%                         Theta1_d(hh,1) = Theta1_2_d(hh);
%                         Theta2_d(hh,1) = Theta2_4_d(hh);
%                         Theta3_d(hh,1) = Theta3_4_d(hh);
%                     else
%                         Theta2_d(hh,1) = Theta2_2_d(hh);
%                         Theta3_d(hh,1) = Theta3_2_d(hh);
%                     end
%                 end
                
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
                %if error reached
               
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
        %if ((d_B_dis_FR > max_body_dist) || (d_B_dis_FL > max_body_dist) || (d_B_dis_BR > max_body_dist) || (d_B_dis_BL > max_body_dist)) || (step_state ~= 0)
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
                    %                 step_error = norm(r_II_c_FL - r_II_c_dstep);
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
                    %                 step_error = norm(r_II_c_BR - r_II_c_dstep);
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
                    %                 step_error = norm(r_II_c_BL - r_II_c_dstep);
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
            [Theta1_d,Theta1_2_d,Theta2_1_d,Theta2_2_d,Theta2_3_d,Theta2_4_d,Theta3_1_d,Theta3_2_d,Theta3_3_d,Theta3_4_d,r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d,r_II_B_d,r_II_B,floor_toggle);
            % if theta1 wraps around into robot
            T1_cond(1) = (Theta1_d(1) <= -pi/2) || (Theta1_d(1) >= pi); % FR
            T1_cond(2) = (Theta1_d(2) <= -pi) || (Theta1_d(2) >= pi/2); %FL
            T1_cond(3) = (Theta1_d(3) <= -pi) || (Theta1_d(3) >= pi/2); % BR
            T1_cond(4) = (Theta1_d(4) <= -pi/2) || (Theta1_d(4) >= pi); % BL
            
            for hh = 1:1:4
                if T1_cond(hh)
                    Theta1_d(hh,1) = Theta1_2_d(hh);
                    Theta2_d(hh,1) = Theta2_4_d(hh);
                    Theta3_d(hh,1) = Theta3_4_d(hh);
                else
                    Theta2_d(hh,1) = Theta2_2_d(hh);
                    Theta3_d(hh,1) = Theta3_2_d(hh);
                end
            end
        end
    end
    
    %% centroid moving/balance section, and commanding step (assigning thetad's)
    if reached_rest_centroid == 0 % needs to move back to resting 4-legged position to find new leg to move
        pgon = polyshape([r_II_c_FR(1), r_II_c_FL(1), r_II_c_BL(1), r_II_c_BR(1)],[r_II_c_FR(2), r_II_c_FL(2), r_II_c_BL(2), r_II_c_BR(2)]);
        [x,y] = centroid(pgon);
        r_II_B_d_temp = [x;y;r_II_B_d(3)];
        [Theta1_d,Theta1_2_d,Theta2_1_d,Theta2_2_d,Theta2_3_d,Theta2_4_d,Theta3_1_d,Theta3_2_d,Theta3_3_d,Theta3_4_d,r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d_temp,r_II_B_d_temp,r_II_B,[1,1,1,1]);
        
        
        % if theta1 wraps around into robot
        T1_cond(1) = (Theta1_d(1) <= -pi/2) || (Theta1_d(1) >= pi); % FR
        T1_cond(2) = (Theta1_d(2) <= -pi) || (Theta1_d(2) >= pi/2); %FL
        T1_cond(3) = (Theta1_d(3) <= -pi) || (Theta1_d(3) >= pi/2); % BR
        T1_cond(4) = (Theta1_d(4) <= -pi/2) || (Theta1_d(4) >= pi); % BL
        
        for hh = 1:1:4
            if T1_cond(hh)
                Theta1_d(hh,1) = Theta1_2_d(hh);
                Theta2_d(hh,1) = Theta2_4_d(hh);
                Theta3_d(hh,1) = Theta3_4_d(hh);
            else
                Theta2_d(hh,1) = Theta2_2_d(hh);
                Theta3_d(hh,1) = Theta3_2_d(hh);
            end
        end
        
        
        reached_rest_centroid = 2;
    elseif reached_rest_centroid == 2 % moving towards resting, or inbetween-step body pose
        body_error = norm(r_II_B - r_II_B_d_temp);
        
        [Theta1_d,Theta1_2_d,Theta2_1_d,Theta2_2_d,Theta2_3_d,Theta2_4_d,Theta3_1_d,Theta3_2_d,Theta3_3_d,Theta3_4_d,r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d_temp,r_II_B_d_temp,r_II_B,floor_toggle);
                
                % if theta1 wraps around into robot
                T1_cond(1) = (Theta1_d(1) <= -pi/2) || (Theta1_d(1) >= pi); % FR
                T1_cond(2) = (Theta1_d(2) <= -pi) || (Theta1_d(2) >= pi/2); %FL
                T1_cond(3) = (Theta1_d(3) <= -pi) || (Theta1_d(3) >= pi/2); % BR
                T1_cond(4) = (Theta1_d(4) <= -pi/2) || (Theta1_d(4) >= pi); % BL
                
                for hh = 1:1:4
                    if T1_cond(hh)
                        Theta1_d(hh,1) = Theta1_2_d(hh);
                        Theta2_d(hh,1) = Theta2_4_d(hh);
                        Theta3_d(hh,1) = Theta3_4_d(hh);
                    else
                        Theta2_d(hh,1) = Theta2_2_d(hh);
                        Theta3_d(hh,1) = Theta3_2_d(hh);
                    end
                end
                
        if body_error < 0.01
            reached_rest_centroid = 1;
        end
    elseif reached_rest_centroid == 1
        waypoint_toggle = 0;
        % rockback before step
        if ~isempty(find(legs_valid == 0))
            if reached_centroid == 0 % hasn't started moving towards centroid yet
                [Theta1_d,Theta1_2_d,Theta2_1_d,Theta2_2_d,Theta2_3_d,Theta2_4_d,Theta3_1_d,Theta3_2_d,Theta3_3_d,Theta3_4_d,r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d_temp,r_II_B_d_temp,r_II_B,floor_toggle);
                
                
                % if theta1 wraps around into robot
                T1_cond(1) = (Theta1_d(1) <= -pi/2) || (Theta1_d(1) >= pi); % FR
                T1_cond(2) = (Theta1_d(2) <= -pi) || (Theta1_d(2) >= pi/2); %FL
                T1_cond(3) = (Theta1_d(3) <= -pi) || (Theta1_d(3) >= pi/2); % BR
                T1_cond(4) = (Theta1_d(4) <= -pi/2) || (Theta1_d(4) >= pi); % BL
                
                for hh = 1:1:4
                    if T1_cond(hh)
                        Theta1_d(hh,1) = Theta1_2_d(hh);
                        Theta2_d(hh,1) = Theta2_4_d(hh);
                        Theta3_d(hh,1) = Theta3_4_d(hh);
                    else
                        Theta2_d(hh,1) = Theta2_2_d(hh);
                        Theta3_d(hh,1) = Theta3_2_d(hh);
                    end
                end
                
                
                reached_centroid = 2;
            elseif reached_centroid == 2 % moving towards centroid
                body_error = norm(r_II_B - r_II_B_d_temp);
                if body_error < 0.01
                    reached_centroid = 1;
                end
            elseif reached_centroid == 1 % step
                % step leg
                [Theta1_d,Theta1_2_d,Theta2_1_d,Theta2_2_d,Theta2_3_d,Theta2_4_d,Theta3_1_d,Theta3_2_d,Theta3_3_d,Theta3_4_d,r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d_temp,r_II_B_d_temp,r_II_B,floor_toggle);
                
                
                % if theta1 wraps around into robot
                % if theta1 wraps around into robot
                T1_cond(1) = (Theta1_d(1) <= -pi/2) || (Theta1_d(1) >= pi); % FR
                T1_cond(2) = (Theta1_d(2) <= -pi) || (Theta1_d(2) >= pi/2); %FL
                T1_cond(3) = (Theta1_d(3) <= -pi) || (Theta1_d(3) >= pi/2); % BR
                T1_cond(4) = (Theta1_d(4) <= -pi/2) || (Theta1_d(4) >= pi); % BL
                
                for hh = 1:1:4
                    if T1_cond(hh)
                        Theta1_d(hh,1) = Theta1_2_d(hh);
                        Theta2_d(hh,1) = Theta2_4_d(hh);
                        Theta3_d(hh,1) = Theta3_4_d(hh);
                    else
                        Theta2_d(hh,1) = Theta2_2_d(hh);
                        Theta3_d(hh,1) = Theta3_2_d(hh);
                    end
                end
                
                
                if (step_state == 1) && (reached_centroid == 1)
                    floor_toggle(leg_index) = 0;
                    Theta1_d(leg_index) = Theta1_d_midpt;
                    Theta2_d(leg_index) = Theta2_d_midpt;
                    Theta3_d(leg_index) = Theta3_d_midpt;
                elseif step_state == 2
                    if (leg_index == 2) || (leg_index == 4)
                        is_left_leg = 1;
                    else
                        is_left_leg = 0;
                    end
                    [Theta1_d(leg_index), Theta1_2_d(leg_index), Theta2_1_d(leg_index), Theta2_2_d(leg_index), Theta2_3_d(leg_index), Theta2_4_d(leg_index), Theta3_1_d(leg_index), Theta3_2_d(leg_index),Theta3_3_d(leg_index),Theta3_4_d(leg_index),r_II_c_dstep] = Leg_Controller(r_II_c_dstep, r_II_c_current, T_I_B, r_II_B, leg_index);
                    
                    
                    % if theta1 wraps around into robot
                    T1_cond(1) = (Theta1_d(1) <= -pi/2) || (Theta1_d(1) >= pi); % FR
                    T1_cond(2) = (Theta1_d(2) <= -pi) || (Theta1_d(2) >= pi/2); %FL
                    T1_cond(3) = (Theta1_d(3) <= -pi) || (Theta1_d(3) >= pi/2); % BR
                    T1_cond(4) = (Theta1_d(4) <= -pi/2) || (Theta1_d(4) >= pi); % BL
                    
                    for hh = 1:1:4
                        if T1_cond(hh)
                            Theta1_d(hh,1) = Theta1_2_d(hh);
                            Theta2_d(hh,1) = Theta2_4_d(hh);
                            Theta3_d(hh,1) = Theta3_4_d(hh);
                        else
                            Theta2_d(hh,1) = Theta2_2_d(hh);
                            Theta3_d(hh,1) = Theta3_2_d(hh);
                        end
                    end
                    
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
            [Theta1_d,Theta1_2_d,Theta2_1_d,Theta2_2_d,Theta2_3_d,Theta2_4_d,Theta3_1_d,Theta3_2_d,Theta3_3_d,Theta3_4_d,r_II_B_d_temp] = Body_Pose_Controller(r_II_c, T_I_B_d_temp,r_II_B_d_temp,r_II_B,floor_toggle);
                
                    
                    % if theta1 wraps around into robot
                    T1_cond(1) = (Theta1_d(1) <= -pi/2) || (Theta1_d(1) >= pi); % FR
                    T1_cond(2) = (Theta1_d(2) <= -pi) || (Theta1_d(2) >= pi/2); %FL
                    T1_cond(3) = (Theta1_d(3) <= -pi) || (Theta1_d(3) >= pi/2); % BR
                    T1_cond(4) = (Theta1_d(4) <= -pi/2) || (Theta1_d(4) >= pi); % BL
                    
                    for hh = 1:1:4
                        if T1_cond(hh)
                            Theta1_d(hh,1) = Theta1_2_d(hh);
                            Theta2_d(hh,1) = Theta2_4_d(hh);
                            Theta3_d(hh,1) = Theta3_4_d(hh);
                        else
                            Theta2_d(hh,1) = Theta2_2_d(hh);
                            Theta3_d(hh,1) = Theta3_2_d(hh);
                        end
                    end
        end
    end
    
    % step not needed
    
    
    
    %% Control Law and Force Computation
    Theta_d = [Theta1_d;Theta2_d;Theta3_d];
    Theta_E = Theta_d - Theta;
    
    %%% Floor Constraint %%%
    [Jc_FR,Jc_FL,Jc_BR,Jc_BL] = contactJacobians(b(:,ii));
    
    c_vel_FR = Jc_FR*b(19:36,ii);
    c_vel_FL = Jc_FL*b(19:36,ii);
    c_vel_BR = Jc_BR*b(19:36,ii);
    c_vel_BL = Jc_BL*b(19:36,ii);
    
    dotr_II_c_FR = c_vel_FR(4:6,1);
    dotr_II_c_FL = c_vel_FL(4:6,1);
    dotr_II_c_BR = c_vel_BR(4:6,1);
    dotr_II_c_BL = c_vel_BL(4:6,1);
    
    Fc_FR = [0;0;0;b_fric_floor*dotr_II_c_FR(1);b_fric_floor*dotr_II_c_FR(2);Kp_floor*(r_II_c_FR(3)) + Kd_floor*(dotr_II_c_FR(3))]*heaviside(-r_II_c_FR(3))*floor_toggle(1);
    Fc_FL = [0;0;0;b_fric_floor*dotr_II_c_FL(1);b_fric_floor*dotr_II_c_FL(2);Kp_floor*(r_II_c_FL(3)) + Kd_floor*(dotr_II_c_FL(3))]*heaviside(-r_II_c_FL(3))*floor_toggle(2);
    Fc_BR = [0;0;0;b_fric_floor*dotr_II_c_BR(1);b_fric_floor*dotr_II_c_BR(2);Kp_floor*(r_II_c_BR(3)) + Kd_floor*(dotr_II_c_BR(3))]*heaviside(-r_II_c_BR(3))*floor_toggle(3);
    Fc_BL = [0;0;0;b_fric_floor*dotr_II_c_BL(1);b_fric_floor*dotr_II_c_BL(2);Kp_floor*(r_II_c_BL(3)) + Kd_floor*(dotr_II_c_BL(3))]*heaviside(-r_II_c_BL(3))*floor_toggle(4);
    
    Fgamma_FR = Jc_FR.'*Fc_FR;
    Fgamma_FL = Jc_FL.'*Fc_FL;
    Fgamma_BR = Jc_BR.'*Fc_BR;
    Fgamma_BL = Jc_BL.'*Fc_BL;
    
    % Compute necessary forces to achieve desired gamma
    Fgamma_control(1:6,1) = [0;0;0;0;0;0];
    % Condition that the legs' contacts will be in a fixed position (i.e. on the ground)
    Fgamma_control(7:18,1) = Kp*Theta_E - Kd*dotTheta;%+ G(7:18,1);
    
    % Saturate Motor Torque
    for jj = 7:1:18
        if Fgamma_control(jj,1) > 5
            Fgamma_control(jj,1) = 5;
        elseif Fgamma_control(jj,1) < -5
            Fgamma_control(jj,1) = -5;
        end
    end
    
    %%% Resultant Applied Force %%%
    Fgamma(:,ii) = (Fgamma_FR+Fgamma_FL+Fgamma_BR+Fgamma_BL)+Fgamma_control;
    
    %%% CM Location %%%
    rcm(:,ii) = compute_rcm(b(7:18,ii),b(4:6,ii),T_I_B);
    
    %%% Numerically Integrate %%%
    k1=robot_states(b(:,ii),Fgamma(:,ii));
    k2=robot_states(b(:,ii)+k1*h/2,Fgamma(:,ii));
    k3=robot_states(b(:,ii)+k2*h/2,Fgamma(:,ii));
    k4=robot_states(b(:,ii)+k3*h,Fgamma(:,ii));
    b(:,ii+1)=b(:,ii)+h*(k1/6+k2/3+k3/3+k4/6);
    
    legs_valid_array(ii,:) = legs_valid;
end

%% Draw animation
setgndplane = 1;

r_II_B = [b(4,:);b(5,:);b(6,:)];
phi = b(1,:);
theta = b(2,:);
psi = b(3,:);
% FR FL BR BL
Theta1 = [b(7,:);b(8,:);b(9,:);b(10,:)];
Theta2 = [b(11,:);b(12,:);b(13,:);b(14,:)];
Theta3 = [b(15,:);b(16,:);b(17,:);b(18,:)];

Ts = 1/45;
Theta = [Theta1;Theta2;Theta3];

writerObj = VideoWriter('SplitControlV13_Walk','MPEG-4');
writerObj.FrameRate = 45;
open(writerObj);

ax = gca;
ax.NextPlot = 'replaceChildren';
%Preallocate a 40-element array M to store the movie frames.

loops = 1:floor(Ts/h):ii;
M(loops) = struct('cdata',[],'colormap',[]);
for kk=1:floor(Ts/h):ii
    T_I_B = rotz(phi(kk))*roty(theta(kk))*rotx(psi(kk));
    r_II_B_a = r_II_B(:,kk);
    Theta_a = Theta(:,kk);
    r_I_sys_cm = compute_rcm(Theta_a,r_II_B_a,T_I_B);
    FK_Solver_Draw_CM(Theta1(:,kk),Theta2(:,kk),Theta3(:,kk),T_I_B,r_II_B(:,kk),r_I_sys_cm, legs_valid_array(kk,:), 'top', 'fixed')
    M(kk) = getframe(gcf);
    writeVideo(writerObj,M(kk));
end

close(writerObj);
