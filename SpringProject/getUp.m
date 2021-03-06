function [Theta_d, stage] = getUp(Theta, stage) %#codegen
% This function "resets" the robot if it has fallen or a kinematic solution
% is unavailable. The legs all crunch up at once, then the robot lifts
% straight up.
% Start stage is 1
% Intermediate stage is 2
% End stage is 0
% USE THIS FUNCTION IN A WHILE LOOP LIKE SO
% while stage ~= 0 
%     [Theta, stage] = getUp(Theta, stage);
%     FK_Solver_Draw(Theta(1:4),Theta(5:8),Theta(9:12),T_I_B,r_II_B);
% end
% IF THE ERROR THRESHOLD FOR MOVING THE LEGS TAKES TOO LONG TO REACH, 
% MANUALLY CHANGE THE STAGE VALUE WITH A ITERATION COUNT CONDITION IN THE
% WHILE LOOP.

assert(all(size(Theta)==[12, 1]))
assert(all(size(stage)==[1, 1]))
assert(isa(Theta, 'double'))
assert(isa(stage, 'uint8'))


Theta_d = zeros(12, 1);
switch stage
    % constants for mid-step resting position
    case 1 % crunch up
        % FR
        Theta_d(1,1) = pi/4;
        Theta_d(5,1) = -pi/4;
        Theta_d(9,1) = 3*pi/4;
        
        % FL
        Theta_d(2,1) = -pi/4;
        Theta_d(6,1) = pi/4;
        Theta_d(10,1) = -3*pi/4;
        
        % BR
        Theta_d(3,1) = -pi/4;
        Theta_d(7,1) = -pi/4;
        Theta_d(11,1) = 3*pi/4;
        
        % BL
        Theta_d(4,1) = pi/4;
        Theta_d(8,1) = pi/4;
        Theta_d(12,1) = -3*pi/4;
        stage = uint8(1);
        
        % calc error
        error_theta = norm(Theta) - norm(Theta_d);
        if (error_theta < 0.3)
            stage = uint8(2);
            
        end
    case 2 % rise up
        Theta_d(1,1) = 0.7854;
        Theta_d(2,1) =  -0.7854;
        Theta_d(3,1) = -0.7854;
        Theta_d(4,1) = 0.7854;
        
        Theta_d(5,1) = 0.2542;
        Theta_d(6,1) = -0.2542;
        Theta_d(7,1) =  0.2542;
        Theta_d(8,1) = -0.2542;
        
        Theta_d(9,1) = 1.5095;
        Theta_d(10,1) = -1.5095;
        Theta_d(11,1) = 1.5095;
        Theta_d(12,1) = -1.5095;
        
        
        error_theta = norm(Theta) - norm(Theta_d);
        stage = uint8(2);
        if (error_theta < 0.3)
            stage = uint8(0);
        end
        
    otherwise
        if (~coder.target("MATLAB"))
            fprintf("Bad Index Input\n");
        end
        error('Bad Index Input');
        
end

end