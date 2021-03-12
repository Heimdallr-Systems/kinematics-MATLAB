function [x,y] = find_pgon_goal(r_II_c_FR,r_II_c_FL,r_II_c_BR,r_II_c_BL,r_II_B,lifted_leg)

in_pgon = 0;

switch lifted_leg
    case 1
        pgonx = [r_II_c_FL(1), r_II_c_BR(1), r_II_c_BL(1)];
        pgony = [r_II_c_FL(2), r_II_c_BR(2), r_II_c_BL(2)];
        boundary_vec = (r_II_c_FL - r_II_c_BR)/norm((r_II_c_FL - r_II_c_BR));
    case 2
        pgonx = [r_II_c_FR(1), r_II_c_BR(1), r_II_c_BL(1)];
        pgony = [r_II_c_FR(2), r_II_c_BR(2), r_II_c_BL(2)];
        boundary_vec = (r_II_c_BL - r_II_c_FR)/norm(r_II_c_BL - r_II_c_FR);
    case 3
        pgonx = [r_II_c_FR(1), r_II_c_FL(1), r_II_c_BL(1)];
        pgony = [r_II_c_FR(2), r_II_c_FL(2), r_II_c_BL(2)];
        boundary_vec = (r_II_c_FR - r_II_c_BL)/norm(r_II_c_FR - r_II_c_BL);
    case 4
        pgonx = [r_II_c_FR(1), r_II_c_FL(1), r_II_c_BR(1)];
        pgony = [r_II_c_FR(2), r_II_c_FL(2), r_II_c_BR(2)];
        boundary_vec = (r_II_c_BR - r_II_c_FL)/norm((r_II_c_BR - r_II_c_FL));
    otherwise
        error("Lifted_Leg is not set to a valid value");
end

r_II_B_new = r_II_B;
dir_to_pgon = (cross([0;0;1],boundary_vec))/norm(r_II_B + cross([0;0;1],boundary_vec));
ii = 0;
while (in_pgon == 0) && (ii <= 100)
    ii = ii+1;
    if inpolygon(r_II_B_new(1),r_II_B_new(2),pgonx,pgony)
        r_II_B_new = r_II_B_new + dir_to_pgon.*0.035;
        in_pgon = 1;
    else
        r_II_B_new = r_II_B_new + dir_to_pgon.*0.001;
    end
end

x = r_II_B_new(1);
y = r_II_B_new(2);
end