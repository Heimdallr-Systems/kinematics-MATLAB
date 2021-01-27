function J = geoJacobian(T, r, gamma)
% This function computes the geometric jacobian of a robot link given a
% transformation matrix, position vector, and vector of joint variables.
% by Nicholas Rossi
J = [T(:,3).'*jacobian(T(:,2),gamma)
        T(:,1).'*jacobian(T(:,3),gamma)
        T(:,2).'*jacobian(T(:,1),gamma)
        jacobian(r,gamma);];
J = simplify(expand(J),'criterion','preferreal','steps',200);
end