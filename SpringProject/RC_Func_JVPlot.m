clear
clc
close all

h = 0.01; % time step
t = 0:h:10; % time vector

%%% Initial Conditions %%%
b=zeros(36,length(t)); % state matrix
b(4,1) = 0;
b(5,1) = 0;
b(6,1) = 0.25; % body height % default 0.18 % 0.3
% first joint angles
b(7,1) = pi/4;
b(8,1) = -pi/4;
b(9,1) = -pi/4;
b(10,1) = pi/4;
% second joint angles
b(11,1) = pi/6+0.1; %pi/4 %0
b(12,1) = -pi/6;
b(13,1) = pi/6;
b(14,1) = -pi/6;
% third joint angles
b(15,1) = pi/3; %pi/8 %pi/2
b(16,1) = -pi/3;
b(17,1) = pi/3;
b(18,1) = -pi/3;

%%% Desired Body Pose Trajectory %%%
% x_d = 1*ones(1,length(t));
% y_d = 0.*ones(1,length(x_d));
% z_d = 0.245*ones(1,length(x_d));
% phi_d = pi/2.*ones(1,length(x_d));
% theta_d =  zeros(1,length(x_d));
% psi_d =  zeros(1,length(x_d));
r_II_B_d = [1;0;0.2];
Euler_d = [pi/2,0,0];
phi_d = pi/2;
theta_d = 0;
psi_d = 0;

% Control Constants
Kd=6;
Kp=30;

%%% Floor Definition %%%
Kp_floor = -5000; % floor spring constant
Kd_floor = -800; % floor damping coefficient
b_fric_floor = -2000; % floor coefficient of lateral, viscous friction

% Numerically Integrate for Position of Manipulator
init_toggle = true;
plotPos.x = zeros(1,length(t));
plotPos.y = zeros(1,length(t));
plotPos.z = zeros(1,length(t));

plotPos.leg1.A = zeros(1,length(t));
plotPos.leg1.B = zeros(1,length(t));
plotPos.leg1.C = zeros(1,length(t));

plotPos.leg2.A = zeros(1,length(t));
plotPos.leg2.B = zeros(1,length(t));
plotPos.leg2.C = zeros(1,length(t));

plotPos.leg3.A = zeros(1,length(t));
plotPos.leg3.B = zeros(1,length(t));
plotPos.leg3.C = zeros(1,length(t));

plotPos.leg4.A = zeros(1,length(t));
plotPos.leg4.B = zeros(1,length(t));
plotPos.leg4.C = zeros(1,length(t));    
for ii = 1:length(t)
    
    [Theta1_d,Theta2_d,Theta3_d,phi_d_temp,r_II_B_d_temp,floor_toggle,legs_valid] = Robot_Control(r_II_B_d, Euler_d, b(:,ii), init_toggle);
    init_toggle = false;
    %%%DYNAMIC%%%
    % %% Control Law and Force Computation
    %     Theta_d = [Theta1_d;Theta2_d;Theta3_d];
    %     Theta_E = Theta_d - Theta;
    %
    %     %%% Floor Constraint %%%
    %     [Jc_FR,Jc_FL,Jc_BR,Jc_BL] = contactJacobians(b(:,ii));
    %
    %     c_vel_FR = Jc_FR*b(19:36,ii);
    %     c_vel_FL = Jc_FL*b(19:36,ii);
    %     c_vel_BR = Jc_BR*b(19:36,ii);
    %     c_vel_BL = Jc_BL*b(19:36,ii);
    %
    %     dotr_II_c_FR = c_vel_FR(4:6,1);
    %     dotr_II_c_FL = c_vel_FL(4:6,1);
    %     dotr_II_c_BR = c_vel_BR(4:6,1);
    %     dotr_II_c_BL = c_vel_BL(4:6,1);
    %
    %     Fc_FR = [0;0;0;b_fric_floor*dotr_II_c_FR(1);b_fric_floor*dotr_II_c_FR(2);Kp_floor*(r_II_c_FR(3)) + Kd_floor*(dotr_II_c_FR(3))]*heaviside(-r_II_c_FR(3))*floor_toggle(1);
    %     Fc_FL = [0;0;0;b_fric_floor*dotr_II_c_FL(1);b_fric_floor*dotr_II_c_FL(2);Kp_floor*(r_II_c_FL(3)) + Kd_floor*(dotr_II_c_FL(3))]*heaviside(-r_II_c_FL(3))*floor_toggle(2);
    %     Fc_BR = [0;0;0;b_fric_floor*dotr_II_c_BR(1);b_fric_floor*dotr_II_c_BR(2);Kp_floor*(r_II_c_BR(3)) + Kd_floor*(dotr_II_c_BR(3))]*heaviside(-r_II_c_BR(3))*floor_toggle(3);
    %     Fc_BL = [0;0;0;b_fric_floor*dotr_II_c_BL(1);b_fric_floor*dotr_II_c_BL(2);Kp_floor*(r_II_c_BL(3)) + Kd_floor*(dotr_II_c_BL(3))]*heaviside(-r_II_c_BL(3))*floor_toggle(4);
    %
    %     Fgamma_FR = Jc_FR.'*Fc_FR;
    %     Fgamma_FL = Jc_FL.'*Fc_FL;
    %     Fgamma_BR = Jc_BR.'*Fc_BR;
    %     Fgamma_BL = Jc_BL.'*Fc_BL;
    %
    %     % Compute necessary forces to achieve desired gamma
    %     Fgamma_control(1:6,1) = [0;0;0;0;0;0];
    %     % Condition that the legs' contacts will be in a fixed position (i.e. on the ground)
    %     Fgamma_control(7:18,1) = Kp*Theta_E - Kd*dotTheta;%+ G(7:18,1);
    %
    %     % Saturate Motor Torque
    %     for jj = 7:1:18
    %         if Fgamma_control(jj,1) > 5
    %             Fgamma_control(jj,1) = 5;
    %         elseif Fgamma_control(jj,1) < -5
    %             Fgamma_control(jj,1) = -5;
    %         end
    %     end
    %
    %     %%% Resultant Applied Force %%%
    %     Fgamma(:,ii) = (Fgamma_FR+Fgamma_FL+Fgamma_BR+Fgamma_BL)+Fgamma_control;
    %
    %     %%% CM Location %%%
    %     rcm(:,ii) = compute_rcm(b(7:18,ii),b(4:6,ii),T_I_B);
    %
    %     %%% Numerically Integrate %%%
    %     k1=robot_states(b(:,ii),Fgamma(:,ii));
    %     k2=robot_states(b(:,ii)+k1*h/2,Fgamma(:,ii));
    %     k3=robot_states(b(:,ii)+k2*h/2,Fgamma(:,ii));
    %     k4=robot_states(b(:,ii)+k3*h,Fgamma(:,ii));
    %     b(:,ii+1)=b(:,ii)+h*(k1/6+k2/3+k3/3+k4/6);
    %
    %     legs_valid_array(ii,:) = legs_valid;
    
    %%% Solve for Current State in order to Plot %%%
    phi = b(1,ii);
    theta = b(2,ii);
    psi = b(3,ii);
    T_I_B = rotz(phi)*roty(theta)*rotx(psi);
    r_II_B = [b(4,ii);b(5,ii);b(6,ii)];
    Theta1 = [b(7,ii);b(8,ii);b(9,ii);b(10,ii)];
    Theta2 = [b(11,ii);b(12,ii);b(13,ii);b(14,ii)];
    Theta3 = [b(15,ii);b(16,ii);b(17,ii);b(18,ii)];
    
    %%%KINEMATIC%%%
    b(1,ii+1) = phi_d_temp;
    b(2,ii+1) = theta_d;
    b(3,ii+1) = psi_d;
    
    %     if r_II_B_d_temp == r_II_B_d_temp_reachable
    b(4,ii+1) = r_II_B_d_temp(1);
    b(5,ii+1) = r_II_B_d_temp(2);
    b(6,ii+1) = r_II_B_d_temp(3);
    %     else
    %         b(4,ii+1) = r_II_B_d_temp_reachable(1);
    %         b(5,ii+1) = r_II_B_d_temp_reachable(2);
    %         b(6,ii+1) = r_II_B_d_temp_reachable(3);
    %     end
    b(7,ii+1) = Theta1_d(1);
    b(8,ii+1) = Theta1_d(2);
    b(9,ii+1) = Theta1_d(3);
    b(10,ii+1) = Theta1_d(4);
    b(11,ii+1) = Theta2_d(1);
    b(12,ii+1) = Theta2_d(2);
    b(13,ii+1) = Theta2_d(3);
    b(14,ii+1) = Theta2_d(4);
    b(15,ii+1) = Theta3_d(1);
    b(16,ii+1) = Theta3_d(2);
    b(17,ii+1) = Theta3_d(3);
    b(18,ii+1) = Theta3_d(4);
    
    
    
    %% Draw animation
    setgndplane = 1;
    
    r_II_B_plot = [b(4,ii);b(5,ii);b(6,ii)];
    phi_plot = b(1,ii);
    theta_plot = b(2,ii);
    psi_plot = b(3,ii);
    % FR FL BR BL
    Theta1_plot = [b(7,ii);b(8,ii);b(9,ii);b(10,ii)];
    Theta2_plot = [b(11,ii);b(12,ii);b(13,ii);b(14,ii)];
    Theta3_plot = [b(15,ii);b(16,ii);b(17,ii);b(18,ii)];
    
    Ts = 2;
    Theta_plot = [Theta1_plot;Theta2_plot;Theta3_plot];
    

    
    T_I_B_plot = rotz(phi_plot)*roty(theta_plot)*rotx(psi_plot);
    r_II_B_a_plot = r_II_B_plot;
    Theta_a_plot = Theta_plot;
    r_I_sys_cm = compute_rcm(Theta_a_plot,r_II_B_a_plot,T_I_B_plot);

    plotPos.x(ii) = b(4,ii);
    plotPos.y(ii) = b(5,ii);
    plotPos.z(ii) = b(6,ii);
    plotPos.leg1.A(ii) = b(7,ii);
    plotPos.leg2.A(ii) = b(8,ii);
    plotPos.leg3.A(ii) = b(9,ii);
    plotPos.leg4.A(ii) = b(10,ii);
    
    plotPos.leg1.B(ii) = b(11,ii);
    plotPos.leg2.B(ii) = b(12,ii);
    plotPos.leg3.B(ii) = b(13,ii);
    plotPos.leg4.B(ii) = b(14,ii);
    
    plotPos.leg1.C(ii) = b(15,ii);
    plotPos.leg2.C(ii) = b(16,ii);
    plotPos.leg3.C(ii) = b(17,ii);
    plotPos.leg4.C(ii) = b(18,ii);
end

figure("Name", "Position Plot")
plot3(plotPos.x, plotPos.y, plotPos.z);
title("Position Plot")

figure("Name", "Leg Joint Vars")
subplot(2,2,2)
plot(t, plotPos.leg1.A)
hold on
plot(t, plotPos.leg1.B);
plot(t, plotPos.leg1.C);
title("FR Joint Vars");
xlabel("t")
ylabel("Angle (rad)")
legend("Joint 1", "Joint 2", "Joint 3");
hold off

subplot(2,2,1)
plot(t, plotPos.leg2.A)
hold on
plot(t, plotPos.leg2.B);
plot(t, plotPos.leg2.C);
title("FL Joint Vars");
xlabel("t")
ylabel("Angle (rad)")
legend("Joint 1", "Joint 2", "Joint 3");
hold off


subplot(2,2,4)
plot(t, plotPos.leg3.A)
hold on
plot(t, plotPos.leg3.B);
plot(t, plotPos.leg3.C);
title("BR Joint Vars");
xlabel("t")
ylabel("Angle (rad)")
legend("Joint 1", "Joint 2", "Joint 3");
hold off

subplot(2,2,3)
plot(t, plotPos.leg4.A)
hold on
plot(t, plotPos.leg4.B);
plot(t, plotPos.leg4.C);
title("BL Joint Vars");
xlabel("t")
ylabel("Angle (rad)")
legend("Joint 1", "Joint 2", "Joint 3");
hold off

%%
