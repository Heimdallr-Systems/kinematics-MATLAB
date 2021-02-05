function [floor_map]=floor_gen(floor_size,noise_sigma,offset)
% This function generates a normally distributed floor map of the robot's
% work space.
% Inputs: 
%  floor_size: the size (length x width) of the floor, in mm. Assumed to be
%    square
%  noise_sigma: standard deviation of noise on the floor
%  offset: height offset of the floor.
% Outputs:
%  floor_map: The map of the floor, in 3d space
ind=1;
floor_map=zeros(floor_size^2,3);
    for row=1:floor_size
        for col=1:floor_size
            floor_map(ind,1:3)=[row-floor(floor_size/2),col-floor(floor_size/2),randn()*2*noise_sigma-noise_sigma+offset];
            ind=ind+1;
        end
    end
end