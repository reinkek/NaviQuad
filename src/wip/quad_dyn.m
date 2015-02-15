function res = quad_dyn(t,v)
%{
y(1) = d_dphi
y(2) = 

%}
res = zeros(12,1);
%% Global variables
global Jr Ixx Iyy Izz l m g b d Om

%% Constants
a1 = (Iyy - Izz)/Ixx;
a2 = Jr/Ixx;
a3 = (Izz - Ixx)/Iyy;
a4 = Jr/Iyy;
a5 = (Ixx - Iyy)/Izz;

b1 = l/Ixx;
b2 = l/Iyy;
b3 = 1/Izz;

dphi = v(1);
phi = v(2);
dtheta = v(3);
theta = v(4);
dpsi = v(5);
psi = v(6);
%dz = v(7);
z = v(8);
%dx = v(9);
x = v(10);
%dy = v(11);
y = v(12);


U1 = b*(Om(1)^2 + Om(2)^2 + Om(3)^2 + Om(4)^2);
U2 = b*(-Om(2)^2 + Om(4)^2);
U3 = b*( Om(1)^2 - Om(3)^2);
U4 = d*(-Om(1)^2 + Om(2)^2 - Om(3)^2 + Om(4)^2);
OmR = U4;

ux = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
uy = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);


ddphi = dtheta*dpsi*a1 + dtheta*a2*OmR + b1*U2;
ddtheta = dphi*dpsi*a3 - dphi*a4*OmR + b2*U3;
ddpsi = dtheta*dphi*a5 + b3*U4;
ddz = g - cos(phi)*cos(theta)*U1/m;
ddx = ux*U1/m;
ddy = uy*U1/m;

res(1) = phi;
res(2) = ddphi;
res(3) = theta;
res(4) = ddtheta;
res(5) = psi;
res(6) = ddpsi;
res(7) = z;
res(8) = ddz;
res(9) = x;
res(10) = ddx;
res(11) = y;
res(12) = ddy;