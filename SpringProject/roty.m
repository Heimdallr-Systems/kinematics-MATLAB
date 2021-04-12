function A = roty(t) %#codegen
% Rotation matrix around y in radians
A = [cos(t) 0 sin(t) 
        0 1 0 
        -sin(t) 0 cos(t)];
end