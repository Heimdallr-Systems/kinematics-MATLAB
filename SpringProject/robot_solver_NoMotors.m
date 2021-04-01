% Dynamical simulation of the heimdallr robot with PD controller


% gamma = [phi; theta; psi; x; y; z; ...
%                  Theta1_FR; Theta1_FL; Theta1_BR; Theta1_BL; ...
%                  Theta2_FR; Theta2_FL; Theta2_BR; Theta2_BL; ...
%                  Theta3_FR; Theta3_FL; Theta3_BR; Theta3_BL];

% dotgamma = [dotphi; dottheta; dotpsi; dotx; doty; dotz; ...
%                        dotTheta1_FR; dotTheta1_FL; dotTheta1_BR; dotTheta1_BL; ...
%                        dotTheta2_FR; dotTheta2_FL; dotTheta2_BR; dotTheta2_BL; ...
%                        dotTheta3_FR; dotTheta3_FR; dotTheta3_BR; dotTheta3_BL];

clear
clc
close all

h = 0.0001; % time step
t = 0:h:1; % time vector % 6 sec %14

%%% Initial Conditions %%%
b=zeros(36,length(t)); % state matrix
b(4,1) = 0;
b(5,1) = 0;
b(6,1) = 0.18; % body height % default 0.242 % 0.3
% first joint angles
b(7,1) = pi/4;
b(8,1) = -pi/4;
b(9,1) = -pi/4;
b(10,1) = pi/4;
% second joint angles
b(11,1) = 0; %pi/4 %0
b(12,1) = 0;
b(13,1) = 0;
b(14,1) = 0;
% third joint angles
b(15,1) = pi/2; %pi/8 %pi/2
b(16,1) = -pi/2;
b(17,1) = pi/2;
b(18,1) = -pi/2;

rcm = zeros(3,length(t)); % initialize rcm

Fgamma= zeros(18,length(t)); % initialize forces matrix (joint space)


%%% Desired Body Pose Trajectory %%%
% x_d_val = [0,0,0,0,-0.02,0.02,0,0,0,0,0,0,0,0,0,0,0];
% y_d_val = [0,0,0,0,0,0,-0.02,0.02,0,0,0,0,0,0,0,0,0];
% z_d_val = [0.22,0.2,0.22,0.19,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2];
% phi_d_val = [0,0,0,0,0,0,0,0,-pi/15,pi/15,0,0,0,0,0,0,0];
% theta_d_val =  [0,0,0,0,0,0,0,0,0,0,-pi/17,0,pi/17,0,0,0,0];
% psi_d_val =  [0,0,0,0,0,0,0,0,0,0,0,0,0,0,-pi/17,0,pi/17];
% x_d_val = [0,0,0,0,0,0,0,0,0,0];
% y_d_val = [0,0,0,0,0,0,0,0,0,0];
% z_d_val = [0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2];
% phi_d_val = [0,-pi/15,pi/15,0,0,0,0,0,0,0];
% theta_d_val =  [0,0,0,-pi/17,0,pi/17,0,0,0,0];
% psi_d_val =  [0,0,0,0,0,0,0,-pi/17,0,pi/17];
x_d_val = [-0.05];
y_d_val = [0.05];
z_d_val = [0.22];
phi_d_val = [0];
theta_d_val =  [0];
psi_d_val =  [0];

x_d = [];
y_d = [];
z_d = [];
phi_d = [];
theta_d = [];
psi_d = [];

cnt = length(x_d_val);

for ii = 1:1:cnt
    x_d = cat(2,x_d,x_d_val(ii).*ones(1,ceil(length(t)/cnt)));
    y_d = cat(2,y_d,y_d_val(ii).*ones(1,ceil(length(t)/cnt)));
    z_d = cat(2,z_d,z_d_val(ii).*ones(1,ceil(length(t)/cnt)));
    phi_d = cat(2,phi_d,phi_d_val(ii).*ones(1,ceil(length(t)/cnt)));
    theta_d = cat(2,theta_d,theta_d_val(ii).*ones(1,ceil(length(t)/cnt)));
    psi_d = cat(2,psi_d,psi_d_val(ii).*ones(1,ceil(length(t)/cnt)));
end

% Control Constants
Kd=4;
Kp=30;

%%% Floor Definition %%%
Kp_floor = -5000; % floor spring constant
Kd_floor = -800; % floor damping coefficient
b_fric_floor = -1000; % floor coefficient of lateral, viscous friction

% Numerically Integrate for Position of Manipulator
for ii = 1:length(t)
    
    %%% Named Vectors of State Values %%%
    r_II_B(:,1) = [b(4,ii);b(5,ii);b(6,ii)];
    Theta1(:,1) = [b(7,ii);b(8,ii);b(9,ii);b(10,ii)];
    Theta2(:,1) = [b(11,ii);b(12,ii);b(13,ii);b(14,ii)];
    Theta3(:,1) = [b(15,ii);b(16,ii);b(17,ii);b(18,ii)];
    Theta = [Theta1;Theta2;Theta3];
    dotTheta1(:,1) = [b(25,ii);b(26,ii);b(27,ii);b(28,ii)];
    dotTheta2(:,1) = [b(29,ii);b(30,ii);b(31,ii);b(32,ii)];
    dotTheta3(:,1) = [b(33,ii);b(34,ii);b(35,ii);b(36,ii)];
    dotTheta(:,1) = [dotTheta1;dotTheta2;dotTheta3];
    T_I_B = rotz(b(1,ii))*roty(b(2,ii))*rotx(b(3,ii));
    T_I_B_d = rotz(phi_d(ii))*roty(theta_d(ii))*rotx(psi_d(ii));
    r_II_B_d(:,1) = [x_d(ii);y_d(ii);z_d(ii)];
    
    %%% Floor Constraint %%%
    [r_II_c_FR, r_II_c_FL, r_II_c_BR, r_II_c_BL] = CPos_wrt_I(Theta1,Theta2,Theta3,T_I_B,r_II_B);
    r_II_c = [r_II_c_FR,r_II_c_FL,r_II_c_BR,r_II_c_BL];
    
    [Jc_FR,Jc_FL,Jc_BR,Jc_BL] = contactJacobians(b(:,ii));
    
    c_vel_FR = Jc_FR*b(19:36,ii);
    c_vel_FL = Jc_FL*b(19:36,ii);
    c_vel_BR = Jc_BR*b(19:36,ii);
    c_vel_BL = Jc_BL*b(19:36,ii);
    
    dotr_II_c_FR = c_vel_FR(4:6,1);
    dotr_II_c_FL = c_vel_FL(4:6,1);
    dotr_II_c_BR = c_vel_BR(4:6,1);
    dotr_II_c_BL = c_vel_BL(4:6,1);
    
    Fc_FR = [0;0;0;b_fric_floor*dotr_II_c_FR(1);b_fric_floor*dotr_II_c_FR(2);Kp_floor*(r_II_c_FR(3)) + Kd_floor*(dotr_II_c_FR(3))]*heaviside(-r_II_c_FR(3));
    Fc_FL = [0;0;0;b_fric_floor*dotr_II_c_FL(1);b_fric_floor*dotr_II_c_FL(2);Kp_floor*(r_II_c_FL(3)) + Kd_floor*(dotr_II_c_FL(3))]*heaviside(-r_II_c_FL(3));
    Fc_BR = [0;0;0;b_fric_floor*dotr_II_c_BR(1);b_fric_floor*dotr_II_c_BR(2);Kp_floor*(r_II_c_BR(3)) + Kd_floor*(dotr_II_c_BR(3))]*heaviside(-r_II_c_BR(3));
    Fc_BL = [0;0;0;b_fric_floor*dotr_II_c_BL(1);b_fric_floor*dotr_II_c_BL(2);Kp_floor*(r_II_c_BL(3)) + Kd_floor*(dotr_II_c_BL(3))]*heaviside(-r_II_c_BL(3));
    
    Fgamma_FR = Jc_FR.'*Fc_FR;
    Fgamma_FL = Jc_FL.'*Fc_FL;
    Fgamma_BR = Jc_BR.'*Fc_BR;
    Fgamma_BL = Jc_BL.'*Fc_BL;
    
        
    %%% Control Law %%%
    [Theta1_d,~,Theta2_d,~,Theta3_d] = IK_Solver_Legs_Inertial(r_II_c, T_I_B_d,r_II_B_d,[1,1,1,1]);
%     [Theta1_d,Theta2_d,Theta3_d] = Joint_Space_Solver(Theta1, Theta2, Theta3, r_II_B, r_II_B_d, T_I_B, T_I_B_d);
    Theta_d = [Theta1_d;Theta2_d;Theta3_d];
    Theta_E = Theta_d - Theta;
    
    % Compute necessary forces to achieve desired gamma
    Fgamma_control(1:6,1) = [0;0;0;0;0;0];
    % Condition that the legs' contacts will be in a fixed position (i.e. on the ground)
    Fgamma_control(7:18,1) = Kp*Theta_E - Kd*dotTheta;%+ G(7:18,1);
    
    for jj = 7:1:18
        if Fgamma_control(jj,1) > 1.47
            Fgamma_control(jj,1) = 1.47;
        elseif Fgamma_control(jj,1) < -1.47
            Fgamma_control(jj,1) = -1.47;
        end
    end
    
    %%% Resultant Applied Force %%%
    Fgamma(:,ii) = (Fgamma_FR+Fgamma_FL+Fgamma_BR+Fgamma_BL)+Fgamma_control;

%     Find desired joint angles given a desired body pose
%     note Theta_d vectors are rows not columns
%         for jj = 1:1:length(Theta_d)
%             if isnan(Theta_d(jj)) || (Theta_d(jj) > 3)
%                 ii
%                 r_II_B_d
%                 T_I_B_d
%                 error('Goal Not in Workspace.')
%             end
%         end

    %%% CM Location %%%
    rcm(:,ii) = compute_rcm(b(7:18,ii),b(4:6,ii));
    
    %%% Numerically Integrate %%%
    k1=robot_states(b(:,ii),Fgamma(:,ii));
    k2=robot_states(b(:,ii)+k1*h/2,Fgamma(:,ii));
    k3=robot_states(b(:,ii)+k2*h/2,Fgamma(:,ii));
    k4=robot_states(b(:,ii)+k3*h,Fgamma(:,ii));
    b(:,ii+1)=b(:,ii)+h*(k1/6+k2/3+k3/3+k4/6);
end

%% Draw animation
setgndplane = 1;

r_II_B = [b(4,:);b(5,:);b(6,:)];
phi = b(1,:);
theta = b(2,:);
psi = b(3,:);
% FR FL BR BL
Theta1 = [b(7,:);b(8,:);b(9,:);b(10,:)];
Theta2 = [b(11,:);b(12,:);b(13,:);b(14,:)];
Theta3 = [b(15,:);b(16,:);b(17,:);b(18,:)];

% figure(1)
% subplot(4,1,1)
% plot(t,Theta1(1,1:length(t)),'-r')
% grid on
% title('\theta_1 FR Leg')
% xlabel('Time (sec)')
% ylabel('\theta_1_,_F_R (rad)')
% subplot(4,1,2)
% plot(t,Theta1(2,1:length(t)),'-r')
% grid on
% title('\theta_1 FL Leg')
% xlabel('Time (sec)')
% ylabel('\theta_1_,_F_L (rad)')
% subplot(4,1,3)
% plot(t,Theta1(3,1:length(t)),'-r')
% grid on
% title('\theta_1 BR Leg')
% xlabel('Time (sec)')
% ylabel('\theta_1_,_B_R (rad)')
% subplot(4,1,4)
% plot(t,Theta1(4,1:length(t)),'-r')
% grid on
% title('\theta_1 BL Leg')
% xlabel('Time (sec)')
% ylabel('\theta_1_,_B_L (rad)')
%
%
% figure(2)
% subplot(4,1,1)
% plot(t,Theta2(1,1:length(t)),'-r')
% grid on
% title('\theta_2 FR Leg')
% xlabel('Time (sec)')
% ylabel('\theta_2_,_F_R (rad)')
% subplot(4,1,2)
% plot(t,Theta2(2,1:length(t)),'-r')
% grid on
% title('\theta_2 FL Leg')
% xlabel('Time (sec)')
% ylabel('\theta_2_,_F_L (rad)')
% subplot(4,1,3)
% plot(t,Theta2(3,1:length(t)),'-r')
% grid on
% title('\theta_2 BR Leg')
% xlabel('Time (sec)')
% ylabel('\theta_2_,_B_R (rad)')
% subplot(4,1,4)
% plot(t,Theta2(4,1:length(t)),'-r')
% grid on
% title('\theta_2 BL Leg')
% xlabel('Time (sec)')
% ylabel('\theta_2_,_B_L (rad)')
%
% figure(3)
% subplot(4,1,1)
% plot(t,Theta3(1,1:length(t)),'-r')
% grid on
% title('\theta_3 FR Leg')
% xlabel('Time (sec)')
% ylabel('\theta_3_,_F_R (rad)')
% subplot(4,1,2)
% plot(t,Theta3(2,1:length(t)),'-r')
% grid on
% title('\theta_3 FL Leg')
% xlabel('Time (sec)')
% ylabel('\theta_3_,_F_L (rad)')
% subplot(4,1,3)
% plot(t,Theta3(3,1:length(t)),'-r')
% grid on
% title('\theta_3 BR Leg')
% xlabel('Time (sec)')
% ylabel('\theta_3_,_B_R (rad)')
% subplot(4,1,4)
% plot(t,Theta3(4,1:length(t)),'-r')
% grid on
% title('\theta_3 BL Leg')
% xlabel('Time (sec)')
% ylabel('\theta_3_,_B_L (rad)')
%
% % body pos
% figure(4)
% subplot(3,1,1)
% plot(t,b(4,1:length(t)))
% grid on
% title('Body Position along Inertial x-axis')
% xlabel('Time (sec)')
% ylabel('x-position (m)')
% subplot(3,1,2)
% plot(t,b(5,1:length(t)))
% grid on
% title('Body Position along Inertial y-axis')
% xlabel('Time (sec)')
% ylabel('y-position (m)')
% subplot(3,1,3)
% plot(t,b(6,1:length(t)))
% grid on
% title('Body Position along Inertial z-axis')
% xlabel('Time (sec)')
% ylabel('z-position (m)')
%
% % body orientation
% figure(5)
% subplot(3,1,1)
% plot(t,b(1,1:length(t)))
% grid on
% title('Body Orientation around Inertial z-axis')
% xlabel('Time (sec)')
% ylabel('\phi (rad)')
% subplot(3,1,2)
% plot(t,b(2,1:length(t)))
% grid on
% title('Body Orientation around Inertial y-axis')
% xlabel('Time (sec)')
% ylabel('\theta (rad)')
% subplot(3,1,3)
% plot(t,b(3,1:length(t)))
% grid on
% title('Body Orientation around Inertial x-axis')
% xlabel('Time (sec)')
% ylabel('\psi (rad)')


%% Draw Simulation
Ts = 1/60;
% floor(Ts/h)

writerObj = VideoWriter('ControlsV5_NoMotors','MPEG-4');
writerObj.FrameRate = 60;
open(writerObj);

ax = gca;
ax.NextPlot = 'replaceChildren';
%Preallocate a 40-element array M to store the movie frames.

loops = 1:floor(Ts/h):length(t);
M(loops) = struct('cdata',[],'colormap',[]);
%For each iteration of j, capture each plot of function X as an individual frame. Store the frame in M.
for ii=1:floor(Ts/h):length(t)
    T_I_B = rotz(phi(ii))*roty(theta(ii))*rotx(psi(ii));
    FK_Solver_Draw(Theta1(:,ii),Theta2(:,ii),Theta3(:,ii),T_I_B,r_II_B(:,ii))
%     FK_Solver_Draw_CM(Theta1(:,ii),Theta2(:,ii),Theta3(:,ii),T_I_B,r_II_B(:,ii),1,rcm(:,ii))
    M(ii) = getframe(gcf);
    writeVideo(writerObj,M(ii));
end
%%
close(writerObj);