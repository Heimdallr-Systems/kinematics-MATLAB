function [x,y] = find_pgon_goal(r_II_c_FR,r_II_c_FL,r_II_c_BR,r_II_c_BL,r_II_B,lifted_leg) %#codegen

in_pgon = false;

switch lifted_leg
    case 1
        pgonx = [r_II_c_FL(1), r_II_c_BR(1), r_II_c_BL(1)];
        pgony = [r_II_c_FL(2), r_II_c_BR(2), r_II_c_BL(2)];
        boundary_vec = (r_II_c_FL - r_II_c_BR);
    case 2
        pgonx = [r_II_c_FR(1), r_II_c_BR(1), r_II_c_BL(1)];
        pgony = [r_II_c_FR(2), r_II_c_BR(2), r_II_c_BL(2)];
        boundary_vec = (r_II_c_BL - r_II_c_FR);
    case 3
        pgonx = [r_II_c_FR(1), r_II_c_FL(1), r_II_c_BL(1)];
        pgony = [r_II_c_FR(2), r_II_c_FL(2), r_II_c_BL(2)];
        boundary_vec = (r_II_c_FR - r_II_c_BL);
    case 4
        pgonx = [r_II_c_FR(1), r_II_c_FL(1), r_II_c_BR(1)];
        pgony = [r_II_c_FR(2), r_II_c_FL(2), r_II_c_BR(2)];
        boundary_vec = (r_II_c_BR - r_II_c_FL);
    otherwise
        if (~coder.target("MATLAB"))
            fprintf("Lifted_Leg is not set to a valid value\n");
        end
        error("Lifted_Leg is not set to a valid value");
end

boundary_vec = boundary_vec / norm(boundary_vec);

% r_II_B_new = r_II_B;

dir_to_pgon = cross([0;0;1],boundary_vec);
dir_to_pgon = (dir_to_pgon)/norm(r_II_B + dir_to_pgon);

ii = uint16(0);
while (in_pgon == false) && (ii <= 100)
    ii = ii+1;
    if inpolygon(r_II_B(1),r_II_B(2),pgonx,pgony)
        r_II_B = r_II_B + dir_to_pgon.*0.035;
        in_pgon = true;
    else
        r_II_B = r_II_B + dir_to_pgon.*0.001;
    end
end

x = r_II_B(1);
y = r_II_B(2);
end