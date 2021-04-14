clear
T_I_B = eye(3);
r_II_B = [0;0;0.22];
stage = uint8(1);
Theta = zeros(12,1);
% FK_Solver_Draw(Theta(1:4),Theta(5:8),Theta(9:12),T_I_B,r_II_B);
while stage ~= 0 
    [Theta, stage] = getUp(Theta, stage);
    FK_Solver_Draw(Theta(1:4),Theta(5:8),Theta(9:12),T_I_B,r_II_B);
end
