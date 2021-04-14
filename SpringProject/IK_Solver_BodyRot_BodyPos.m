function [T_I_B,r_II_B] = IK_Solver_BodyRot_BodyPos(r_BB_c, r_II_c, legs_on_gnd)
% This function is used to solve for the body rotation and the body
% position assuming we've kept track of our contact point positions with
% respect to the base sufficieintly. This solver only works if we know that
% at least 3 legs are touching the ground. If only 2 are touching, then
% there are infinite solutions.
% r_BB_c = [r_BB_c_FR_x, r_BB_c_FL_x, r_BB_c_BR_x, r_BB_c_BL_x
%           r_BB_c_FR_y, r_BB_c_FL_y, r_BB_c_BR_y, r_BB_c_BL_y
%           r_BB_c_FR_z, r_BB_c_FL_z, r_BB_c_BR_z, r_BB_c_BL_z]
% r_II_c = [r_II_c_FR_x, r_II_c_FL_x, r_II_c_BR_x, r_II_c_BL_x
%           r_II_c_FR_y, r_II_c_FL_y, r_II_c_BR_y, r_II_c_BL_y
%           r_II_c_FR_z, r_II_c_FL_z, r_II_c_BR_z, r_II_c_BL_z]
% legs_on_gnd = [1 or 0,1 or 0,1 or 0,1 or 0] either 3 or 4 legs are touching the ground
% leg notation is now in 1-4 since it may not always be the case that a
% specific set of three legs may be touching the ground, instead the three
% legs touching the ground may be a different combination. So 1-4 is used
% to be the most general.


numTrue = uint8(sum(legs_on_gnd==1));

legs = findTrue4Elem(legs_on_gnd);



r_BB_c_leg1 = r_BB_c(:,legs(1));
r_BB_c_leg2 = r_BB_c(:,legs(2));
r_BB_c_leg3 = r_BB_c(:,legs(3));
if numTrue == 4
    r_BB_c_leg4 = r_BB_c(:,legs(4));
end

r_II_c_leg1 = r_II_c(:,legs(1));
r_II_c_leg2 = r_II_c(:,legs(2));
r_II_c_leg3 = r_II_c(:,legs(3));
if numTrue == 4
    r_II_c_leg4 = r_II_c(:,legs(4));
end

% apply Markley's Solution to Wahbah's Problem
r_BB_i_1 = r_BB_c_leg1 - r_BB_c_leg2;
r_BB_i_2 = r_BB_c_leg1 - r_BB_c_leg3;
r_BB_i_3 = r_BB_c_leg2 - r_BB_c_leg3;

r_II_i_1 = r_II_c_leg1 - r_II_c_leg2;
r_II_i_2 = r_II_c_leg1 - r_II_c_leg3;
r_II_i_3 = r_II_c_leg2 - r_II_c_leg3;

product1 = r_II_i_1*(r_BB_i_1');
product2 = r_II_i_2*(r_BB_i_2');
product3 = r_II_i_3*(r_BB_i_3');
%product4 = r_II_i_4*(r_BB_i_4');

B = product1 + product2 + product3; %+ product4


[U,S,V] = svd(B);

M = diag([1, 1, det(U)*det(V)]);

T_I_B = U*M*V';

r_II_B = r_II_c_leg1 - T_I_B*(r_BB_c_leg1);
end