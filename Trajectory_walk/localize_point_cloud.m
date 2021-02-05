function [local_map,local_hp] = localize_point_cloud(global_map, global_indices, r_II_b, max_height, hp_psf)
% Function that calculates a local map, and its high pass transform. This
% is used to localize the floor immediately around the robot, and show the
% flattest location close to the robot for step planning
% INPUTS:
% global_map: The global point map of the floor, sampled at a constant
% distance from each other
% global_indices: A matrix of indicies of the global map. This allows for 
% the global map to grow as the robot moves in all directions
% Should be in the form of
% [-2,2], [-1,2], [0,2], [1,2], [2,2];
% [-2,1], [-1,1], [0,1], [1,1], [2,1];
% [-2,0], [-1,0], [0,0], [1,0], [2,0];
% [-2,-1], [-1,-1], [0,-1], [1,-1], [2,-1];
% [-2,-2], [-1,-2], [0,-2], [1,-2], [2,-2]
% r_II_b: vector from the inertial frame to the body of the robot
% max_height: The maximum height at which the robot cannot step over
% hp_psf: the point-spread function of the high-pass filter. Can be 3x3, or
% 5x5 with some modification.
% OUTPUTS:
% local_map: the localized floor map around the robot, within a .5 meter
% radius
% local_hp: the high-pass filter of the local map, with untraversible
% terrain marked as Inf

% CONSTANTS
% point cloud resolution, points/mm
c_pc_res=.1;

% Determine the index of the body
r_II_b_coord=round(r_II_b(1:2)*c_pc_res);
local_map_size=.5*1000*2*c_pc_res;
local_map=global_map
end