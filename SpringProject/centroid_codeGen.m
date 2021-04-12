function [Cx, Cy, A] = centroid_codeGen(x, y) %#codegen
%centroid_codeGen Calculate Centroid of a Polygon
% Unlike the centroid() function, this is compatible
% with MATLAB Coder

% Finish the loop of the polygon
x = [x, x(1)];
y = [y, y(1)];

% Common portion that will be used multiple times. 
commonBit = x.*circshift(y, -1) - circshift(x, -1).*y;
A = 0.5 .* sum(commonBit);
Cx = (1./(6.*A)) .* sum((x+circshift(x, -1)) .* commonBit);
Cy = (1./(6.*A)) .* sum((y+circshift(y, -1)) .* commonBit);

end