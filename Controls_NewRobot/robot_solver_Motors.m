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

h = 0.001; % time step
t = 0:h:12; % time vector

%%% Initial Conditions %%%
b=zeros(36,length(t)); % state matrix
b(6,1) = 0.242; % body height
% first joint angles
b(7,1) = pi/4;
b(8,1) = -pi/4;
b(9,1) = -pi/4;
b(10,1) = pi/4;
% second joint angles
b(11,1) = 0;
b(12,1) = 0;
b(13,1) = 0;
b(14,1) = 0;
% third joint angles
b(15,1) = pi/2;
b(16,1) = -pi/2;
b(17,1) = pi/2;
b(18,1) = -pi/2;

Fgamma= zeros(18,length(t)); % initialize forces matrix (joint space)

% phase = 0:2*pi/(length(t)/t(end)):2*pi;
% x_d = 0.02.*cos(phase);
% y_d = 0.02.*cos(phase);
% z_d = 0.02.*cos(phase)+0.242;
% phi_d = (pi/16)*sin(phase);
% theta_d = (pi/16)*sin(phase);
% psi_d = (pi/16)*cos(phase);
%%% Desired Body Pose Trajectory %%%
x_d_val = [0,0,0,0,-0.03,0.03,0,0,0,0,0,0,0,0,0,0];
y_d_val = [0,0,0,0,0,0,-0.03,0.03,0,0,0,0,0,0,0,0];
z_d_val = [0.242,0.2,0.31,0.22,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2,0.2];
phi_d_val = [0,0,0,0,0,0,0,0,-pi/14,pi/14,0,0,0,0,0,0];
theta_d_val =  [0,0,0,0,0,0,0,0,0,0,-pi/17,0,pi/17,0,0,0];
psi_d_val =  [0,0,0,0,0,0,0,0,0,0,0,0,0,-pi/17,0,pi/17];

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

%%% Floor Definition %%%
Kp_floor = -6000; % floor spring constant
Kd_floor = -700; % floor damping coefficient
b_fric_floor = -900; % floor coefficient of lateral, viscous friction

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
    
    %%% Resultant Applied Force %%%
    Fgamma(:,ii) = (Fgamma_FR+Fgamma_FL+Fgamma_BR+Fgamma_BL);
    
    %%% Control Law %%%
    % Find desired joint angles given a desired body pose
    % note Theta_d vectors are rows not columns
    [Theta1_d,Theta2_d,Theta3_d] = Joint_Space_Solver(Theta1, Theta2, Theta3, r_II_B, r_II_B_d, T_I_B, T_I_B_d);
    Theta_d = [Theta1_d.';Theta2_d.';Theta3_d.'];
    for jj = 1:1:length(Theta_d)
        if isnan(Theta_d(jj)) || (Theta_d(jj) > 3)
            ii
            r_II_B_d
            T_I_B_d
            error('Goal Not in Workspace.')
        end
    end
    
    %%% Numerically Integrate %%%
    k1=robot_states(b(:,ii),Fgamma(:,ii),Theta_d);
    k2=robot_states(b(:,ii)+k1*h/2,Fgamma(:,ii),Theta_d);
    k3=robot_states(b(:,ii)+k2*h/2,Fgamma(:,ii),Theta_d);
    k4=robot_states(b(:,ii)+k3*h,Fgamma(:,ii),Theta_d);
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
% plot(t,Theta1(1,:),'-r')
% grid on
% title('Theta1')
% xlabel('Time (sec)')
% ylabel('Theta1 FR (rad)')
% subplot(4,1,2)
% plot(t,Theta1(2,:),'-r')
% grid on
% xlabel('Time (sec)')
% ylabel('Theta1 FL (rad)')
% subplot(4,1,3)
% plot(t,Theta1(3,:),'-r')
% grid on
% xlabel('Time (sec)')
% ylabel('Theta1 BR (rad)')
% subplot(4,1,4)
% plot(t,Theta1(4,:),'-r')
% grid on
% xlabel('Time (sec)')
% ylabel('Theta1 BL (rad)')
%
%
% figure(2)
% subplot(4,1,1)
% plot(t,Theta2(1,:),'-r')
% grid on
% title('Theta2')
% xlabel('Time (sec)')
% ylabel('Theta2 FR (rad)')
% subplot(4,1,2)
% plot(t,Theta2(2,:),'-r')
% grid on
% xlabel('Time (sec)')
% ylabel('Theta2 FL (rad)')
% subplot(4,1,3)
% plot(t,Theta2(3,:),'-r')
% grid on
% xlabel('Time (sec)')
% ylabel('Theta2 BR (rad)')
% subplot(4,1,4)
% plot(t,Theta2(4,:),'-r')
% grid on
% xlabel('Time (sec)')
% ylabel('Theta2 BL (rad)')
%
% figure(3)
% subplot(4,1,1)
% plot(t,Theta3(1,:),'-r')
% grid on
% title('Theta3')
% xlabel('Time (sec)')
% ylabel('Theta3 FR (rad)')
% subplot(4,1,2)
% plot(t,Theta3(2,:),'-r')
% grid on
% xlabel('Time (sec)')
% ylabel('Theta3 FL (rad)')
% subplot(4,1,3)
% plot(t,Theta3(3,:),'-r')
% grid on
% xlabel('Time (sec)')
% ylabel('Theta3 BR (rad)')
% subplot(4,1,4)
% plot(t,Theta3(4,:),'-r')
% grid on
% xlabel('Time (sec)')
% ylabel('Theta3 BL (rad)')

Ts = 1/60;
%floor(Ts/h)

writerObj = VideoWriter('DynamicsDemoV2','MPEG-4');
writerObj.FrameRate = 60;
open(writerObj);

ax = gca;
ax.NextPlot = 'replaceChildren';
%Preallocate a 40-element array M to store the movie frames.

loops = 1:floor(Ts/h):length(t);
M(loops) = struct('cdata',[],'colormap',[]);
%For each iteration of j, capture each plot of function X as an individual frame. Store the frame in M.


%floor(Ts/h)
for ii=1:floor(Ts/h):length(t)
    T_I_B = rotz(phi(ii))*roty(theta(ii))*rotx(psi(ii));
    FK_Solver_Draw(Theta1(:,ii),Theta2(:,ii),Theta3(:,ii),T_I_B,r_II_B(:,ii),setgndplane)
    M(ii) = getframe(gcf);
    writeVideo(writerObj,M(ii));
    %     pause(Ts);
end

close(writerObj);