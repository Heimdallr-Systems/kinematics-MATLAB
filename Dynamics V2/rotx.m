function A = rotx(t)
% Rotation matrix around x in degrees
A = [1 0 0
        0 cos(t) -sin(t)
        0 sin(t) cos(t)];
end