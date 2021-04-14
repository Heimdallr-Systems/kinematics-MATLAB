function [r_II_B_dead,T_I_B_dead,prev_legs_valid,r_II_c_dead] = CallTheDead(Theta,r_II_B_dead,T_I_B_dead,firstCall,legs_valid,prev_legs_valid,r_II_c_dead)
  assert(all(size(Theta) == [12, 1]))
  assert(all(size(r_II_B_dead)==[3, 1]))
  assert(all(size(T_I_B_dead)==[3, 3]))
  assert(all(size(firstCall)==[1, 1]))
  assert(all(size(legs_valid)==[1,4]))
  assert(all(size(prev_legs_valid)==[1,4]))
  assert(all(size(r_II_c_dead)==[3,4]))
  assert(isa(Theta, 'double'))
  assert(isa(r_II_B_dead, 'double'))
  assert(isa(T_I_B_dead, 'double'))
  assert(isa(firstCall, 'logical'))
  assert(isa(legs_valid, 'logical'))
  assert(isa(prev_legs_valid, 'logical'))
  assert(isa(r_II_c_dead, 'double'))

  %%% DEAD RECKONING %%%
    %Theta = [Theta1;Theta2;Theta3];
    %r_II_c_dead = [r_II_c_FR_dead, r_II_c_FL_dead, r_II_c_BR_dead, r_II_c_BL_dead]
    Theta1 = Theta(1:4);
    Theta2 = Theta(5:8);
    Theta3 = Theta(9:12);
    [r_BB_c_FR_dead, r_BB_c_FL_dead, r_BB_c_BR_dead, r_BB_c_BL_dead] = CPos_wrt_B(Theta1,Theta2,Theta3);
    r_BB_c_dead = [r_BB_c_FR_dead, r_BB_c_FL_dead, r_BB_c_BR_dead, r_BB_c_BL_dead];
    r_II_c_FR_dead = r_II_c_dead(1:3,1);
    r_II_c_FL_dead = r_II_c_dead(1:3,2);
    r_II_c_BR_dead = r_II_c_dead(1:3,3);
    r_II_c_BL_dead = r_II_c_dead(1:3,4);
    if firstCall == 1 % must start robot in no-tilt orientation
        [r_II_c_FR_dead, r_II_c_FL_dead, r_II_c_BR_dead, r_II_c_BL_dead] = CPos_wrt_I(Theta1,Theta2,Theta3,T_I_B_dead,r_II_B_dead);
        r_II_c_dead = [r_II_c_FR_dead,r_II_c_FL_dead,r_II_c_BR_dead,r_II_c_BL_dead];
    end
    if isequal([1,1,1,1],legs_valid) && isequal([1,1,1,1], prev_legs_valid)
        [T_I_B_dead,r_II_B_dead] = IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, legs_valid);
    end
    if isequal([0,1,1,1],legs_valid) || isequal([0,1,1,1], prev_legs_valid)
        if isequal([0,1,1,1],legs_valid)
            [T_I_B_dead,r_II_B_dead] = IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, legs_valid);
        elseif isequal([0,1,1,1], prev_legs_valid)
            [T_I_B_dead,r_II_B_dead] = IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, prev_legs_valid);
        end
        r_II_c_FR_dead = r_II_B_dead + T_I_B_dead*r_BB_c_FR_dead;
    end
    if isequal([1,0,1,1],legs_valid) || isequal([1,0,1,1], prev_legs_valid) %falling edge (or rising) detection to finalize calculations
        if isequal([1,0,1,1],legs_valid)
            [T_I_B_dead,r_II_B_dead] = IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, legs_valid);
        elseif isequal([1,0,1,1], prev_legs_valid)
            [T_I_B_dead,r_II_B_dead] = IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, prev_legs_valid);
        r_II_c_FL_dead = r_II_B_dead + T_I_B_dead*r_BB_c_FL_dead;
        end
    end
    if isequal([1,1,0,1],legs_valid) || isequal([1,1,0,1], prev_legs_valid)
        if isequal([1,1,0,1],legs_valid)
            [T_I_B_dead,r_II_B_dead] = IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, legs_valid);
        elseif isequal([1,1,0,1], prev_legs_valid)
            [T_I_B_dead,r_II_B_dead] = IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, prev_legs_valid);
        end
        r_II_c_BR_dead = r_II_B_dead + T_I_B_dead*r_BB_c_BR_dead;
    end
    if isequal([1,1,1,0],legs_valid) || isequal([1,1,1,0], prev_legs_valid)
        if isequal([1,1,1,0],legs_valid)
            [T_I_B_dead,r_II_B_dead] = IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, legs_valid);
        elseif isequal([1,1,1,0], prev_legs_valid)
            [T_I_B_dead,r_II_B_dead] = IK_Solver_BodyRot_BodyPos(r_BB_c_dead, r_II_c_dead, prev_legs_valid);
        end
        r_II_c_BL_dead = r_II_B_dead + T_I_B_dead*r_BB_c_BL_dead;
    end
    r_II_c_dead = [r_II_c_FR_dead,r_II_c_FL_dead,r_II_c_BR_dead,r_II_c_BL_dead];
    prev_legs_valid = legs_valid;
    