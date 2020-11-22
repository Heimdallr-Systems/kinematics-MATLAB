clear
clc
close all

% gamma = [phi; theta; psi; x; y; z; ...
%                  Theta1_FR; Theta1_FL; Theta1_BR; Theta1_BL; ...
%                  Theta2_FR; Theta2_FL; Theta2_BR; Theta2_BL; ...
%                  Theta3_FR; Theta3_FR; Theta3_BR; Theta3_BL];

% dotgamma = [dotphi; dottheta; dotpsi; dotx; doty; dotz; ...
%                        dotTheta1_FR; dotTheta1_FL; dotTheta1_BR; dotTheta1_BL; ...
%                        dotTheta2_FR; dotTheta2_FL; dotTheta2_BR; dotTheta2_BL; ...
%                        dotTheta3_FR; dotTheta3_FR; dotTheta3_BR; dotTheta3_BL];

h = 0.005; % time step
t = 0:h:10; % time vector

b=zeros(36,length(t)); % state matrix
b(6,1) = 0.4;
% b(1,1) = pi/2;
% b(11,1) = pi/2;
% b(12,1) =- pi/2;
% b(13,1) = pi/2;
% b(14,1) = -pi/2;
% b(15,1) = pi/6;
% b(16,1) =- pi/6;
% b(17,1) = pi/6;
% b(18,1) = -pi/6;

FEE = zeros(6,length(t)); % task space forces on end effector

Fgamma= zeros(18,length(t)); % forces matrix (joint space)

Kp = -2000; % floor spring constant
Kd = -200; % floor damping coefficient
b_fric = 0;%-200; % 200; viscous floor friction

% Numerically Integrate for Position of Manipulator
for ii = 1:length(t)-1
    
    r_II_B = [b(4,ii);b(5,ii);b(6,ii)];
    Theta1 = [b(7,ii);b(8,ii);b(9,ii);b(10,ii)];
    Theta2 = [b(11,ii);b(12,ii);b(13,ii);b(14,ii)];
    Theta3 = [b(15,ii);b(16,ii);b(17,ii);b(18,ii)];
    T_I_B = rotz(b(1,ii))*roty(b(2,ii))*rotx(b(3,ii));
    
%      if r_II_B(3) <= 0
%          Fgamma_B(6,1) = (Kp*b(6,ii) + Kd*b(24,ii));
%      else
%          Fgamma_B = zeros(18,1);
%      end
%     
    [r_II_c_FR, r_II_c_FL, r_II_c_BR, r_II_c_BL] = CPos_wrt_I(Theta1,Theta2,Theta3,T_I_B,r_II_B);
    
    %if r_II_c_FR <0
%     deltaZ_FR = -r_II_c_FR(3);
%     deltaZ_FL = -r_II_c_FL(3);
%     deltaZ_BR = -r_II_c_BR(3);
%     deltaZ_BL = -r_II_c_BL(3);
    
    [Jc_FR,Jc_FL,Jc_BR,Jc_BL] = contactJacobians(b(:,ii));
   
    c_vel_FR = Jc_FR*b(19:36,ii);
    c_vel_FL = Jc_FL*b(19:36,ii);
    c_vel_BR = Jc_BR*b(19:36,ii);
    c_vel_BL = Jc_BL*b(19:36,ii);
    
    dotr_II_c_FR = c_vel_FR(4:6,1);
    dotr_II_c_FL = c_vel_FL(4:6,1);
    dotr_II_c_BR = c_vel_BR(4:6,1);
    dotr_II_c_BL = c_vel_BL(4:6,1);
     
    Fc_FR = [0;0;0;b_fric*dotr_II_c_FR(1);b_fric*dotr_II_c_FR(2);Kp*(r_II_c_FR(3)) + Kd*(dotr_II_c_FR(3))]*heaviside(-r_II_c_FR(3));
    Fc_FL = [0;0;0;b_fric*dotr_II_c_FL(1);b_fric*dotr_II_c_FL(2);Kp*(r_II_c_FL(3)) + Kd*(dotr_II_c_FL(3))]*heaviside(-r_II_c_FL(3));
    Fc_BR = [0;0;0;b_fric*dotr_II_c_BR(1);b_fric*dotr_II_c_BR(2);Kp*(r_II_c_BR(3)) + Kd*(dotr_II_c_BR(3))]*heaviside(-r_II_c_BR(3));
    Fc_BL = [0;0;0;b_fric*dotr_II_c_BL(1);b_fric*dotr_II_c_BL(2);Kp*(r_II_c_BL(3)) + Kd*(dotr_II_c_BL(3))]*heaviside(-r_II_c_BL(3));
    
    Fgamma_FR = Jc_FR.'*Fc_FR;
    Fgamma_FL = Jc_FL.'*Fc_FL;
    Fgamma_BR = Jc_BR.'*Fc_BR;
    Fgamma_BL = Jc_BL.'*Fc_BL;
    
    Fgamma(:,ii) = (Fgamma_FR+Fgamma_FL+Fgamma_BR+Fgamma_BL);
    
  
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

figure(1)
subplot(4,1,1)
plot(t,Theta1(1,:),'-r')
grid on
title('Theta1')
xlabel('Time (sec)')
ylabel('Theta1 FR (rad)')
subplot(4,1,2)
plot(t,Theta1(2,:),'-r')
grid on
xlabel('Time (sec)')
ylabel('Theta1 FL (rad)')
subplot(4,1,3)
plot(t,Theta1(3,:),'-r')
grid on
xlabel('Time (sec)')
ylabel('Theta1 BR (rad)')
subplot(4,1,4)
plot(t,Theta1(4,:),'-r')
grid on
xlabel('Time (sec)')
ylabel('Theta1 BL (rad)')


figure(2)
subplot(4,1,1)
plot(t,Theta2(1,:),'-r')
grid on
title('Theta2')
xlabel('Time (sec)')
ylabel('Theta2 FR (rad)')
subplot(4,1,2)
plot(t,Theta2(2,:),'-r')
grid on
xlabel('Time (sec)')
ylabel('Theta2 FL (rad)')
subplot(4,1,3)
plot(t,Theta2(3,:),'-r')
grid on
xlabel('Time (sec)')
ylabel('Theta2 BR (rad)')
subplot(4,1,4)
plot(t,Theta2(4,:),'-r')
grid on
xlabel('Time (sec)')
ylabel('Theta2 BL (rad)')

figure(3)
subplot(4,1,1)
plot(t,Theta3(1,:),'-r')
grid on
title('Theta3')
xlabel('Time (sec)')
ylabel('Theta3 FR (rad)')
subplot(4,1,2)
plot(t,Theta3(2,:),'-r')
grid on
xlabel('Time (sec)')
ylabel('Theta3 FL (rad)')
subplot(4,1,3)
plot(t,Theta3(3,:),'-r')
grid on
xlabel('Time (sec)')
ylabel('Theta3 BR (rad)')
subplot(4,1,4)
plot(t,Theta3(4,:),'-r')
grid on
xlabel('Time (sec)')
ylabel('Theta3 BL (rad)')

Ts = 1/20;

for ii=1:floor(Ts/h):length(t)
    T_I_B = rotz(phi(ii))*roty(theta(ii))*rotx(psi(ii));
    FK_Solver_Draw(Theta1(:,ii),Theta2(:,ii),Theta3(:,ii),T_I_B,r_II_B(:,ii),setgndplane)
    drawnow
    pause(Ts);
end