function A = rotz(t)
% Rotation matrix around z in degrees
A = [cos(t) -sin(t) 0 
         sin(t) cos(t) 0 
         0 0 1];
end