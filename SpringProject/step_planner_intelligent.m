function [r_II_c] = step_planner_intelligent(r_II_b_last, r_II_b,r_II_c_start, step_dist)
% This function returns r_II_c for all 4 joint positions, given a direction
% of travel, starting location and step distance.
% Rewritten to take into account the point cloud of the floor for placing
% the foot in a more safe location.
% INPUTS:
% r_II_b: Current location of the robot's body
% r_II_b_last: previous location of the robot's body
% r_II_c_start: current location of the robot's contact points in the I
% frame
% leg_index: index of the leg being moved
% step_dist: Linear distance a contact point will travel in a step
% floor_map: The point cloud of the robot's immediate surroundings
% step_tolerance: range at which a step location can deviate
% OUTPUTS:
% r_II_c: updated location of the contact points of the robot's legs.

% CONSTANTS
% point cloud resolution, points/mm
% c_pc_res=.1;


travel_dir_0=r_II_b-r_II_b_last;
% convert travel_dir to a unit vector if it is not already.

if isequal(travel_dir_0, [0;0;0])
    travel_dir_0 = r_II_b-r_II_c_start;
end

travel_dir=travel_dir_0/norm(travel_dir_0);


r_II_c = r_II_c_start+travel_dir.*step_dist;
r_II_c(3) = r_II_c(3)-0.005;


% r_BB_c_target = r_II_c(:,leg_index)-r_II_b;

% determine center coordinates
% ideal_leg_pos=round(r_BB_c_target*c_pc_res);
end