function [r_II_c]=step_planner(r_II_b_last, r_II_b,r_II_c_start,step_dist)
% This function returns r_II_c for all 4 joint positions, given a direction
% of travel, starting location and step distance.
% INPUTS: 
% r_II_b: Current location of the robot's body
% r_II_b_last: previous location of the robot's body
% r_II_c_start: current location of the robot's contact points in the I
% frame
% step_dist: Linear distance a contact point will travel in a step
% OUTPUTS:
% r_II_c: updated location of the contact points of the robot's legs. 

travel_dir=r_II_b-r_II_b_last;
% convert travel_dir to a unit vector if it is not already.
travel_dir=travel_dir/norm(travel_dir);

r_II_c=r_II_c_start+travel_dir.*step_dist;
end