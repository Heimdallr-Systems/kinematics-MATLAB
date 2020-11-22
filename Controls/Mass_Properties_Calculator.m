clear
clc

length = 0.479425;
width = 19.11/1000;
thickness = 4.67/1000;
mass = 114/1000;

volume = width*thickness*length;
rho = mass/volume;

syms x y z

r = [x;y;z];

GAMMA = vpa(int(int(int(rho*r,x,-length/2,length/2),y,-width/2,width/2),z,-thickness/2,thickness/2),7)
J = vpa(int(int(int(rho*skew(r)*skew(r).',x,-length/2,length/2),y,-width/2,width/2),z,-thickness/2,thickness/2),7)