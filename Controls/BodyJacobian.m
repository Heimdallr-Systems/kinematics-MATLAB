clear
clc

syms phi theta psi real
syms x y z real
syms Theta1_FR Theta1_FL Theta1_BR Theta1_BL real
syms Theta2_FR Theta2_FL Theta2_BR Theta2_BL real
syms Theta3_FR Theta3_FR Theta3_BR Theta3_BL real

T_I_B = rotz(phi)*roty(theta)*rotx(psi);
r_II_B = [x;y;z];
gamma = [phi; theta; psi; x; y; z; ...
    Theta1_FR; Theta1_FL; Theta1_BR; Theta1_BL; ...
    Theta2_FR; Theta2_FL; Theta2_BR; Theta2_BL; ...
    Theta3_FR; Theta3_FR; Theta3_BR; Theta3_BL];

J = geoJacobian(T_I_B,r_II_B,gamma)

syms phi_t(t) theta_t(t) psi_t(t)
syms dotphi dottheta dotpsi
dotJ = diff(subs(J,[phi theta psi],[phi_t theta_t psi_t]),t);
dotJ = subs(dotJ,[phi_t theta_t psi_t diff(phi_t) diff(theta_t) diff(psi_t)], [phi theta psi dotphi dottheta dotpsi])