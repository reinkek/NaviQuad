function [A,B,G] = linearize_quadcopter_dynamics(vtemp)

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

dphi = vtemp(1);
phi = vtemp(2);
dtheta = vtemp(3);
theta = vtemp(4);
dpsi = vtemp(5);
psi = vtemp(6);

% dphi = v(1);
% phi = v(2);
% dtheta = v(3);
% theta = v(4);
% dpsi = v(5);
% psi = v(6);
% 
% dz = v(7);
% z = v(8);
% dx = v(9);
% x = v(10);
% %dy = v(11);
% y = v(12);


% U1 = b*(Om(1)^2 + Om(2)^2 + Om(3)^2 + Om(4)^2);
% U2 = b*(-Om(2)^2 + Om(4)^2);
% U3 = b*( Om(1)^2 - Om(3)^2);
% U4 = d*(-Om(1)^2 + Om(2)^2 - Om(3)^2 + Om(4)^2);
% OmR = U4;


A1 = [0 1 0 0 0 0;...
    0 0 0 a1/2*dpsi 0 a1/2*dtheta;...
    0 0 0 1 0 0;...
    0 a3/2*dpsi 0 0 0 a3/2*dphi;...
    0 0 0 0 0 1;...
    0 a5/2*dtheta 0 a5/2*dphi 0 0];


%% x-y-z displacements
A2 = [0 1 0 0 0 0;...
      0 0 0 0 0 0;...
      0 0 0 1 0 0;...
      0 0 0 0 0 0;...
      0 0 0 0 0 1;...
      0 0 0 0 0 0];

ux = cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
uy = cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);

A = [A1 zeros(size(A1,1),size(A2,2));...
    zeros(size(A1,2),size(A2,1)) A2];

G = [zeros(size(A1,1),1); [0; g; 0; 0; 0; 0] ];
%% Finding large B matrix
zeroRow = zeros(1,4);
alternateOnes = [-1 1 -1 1];
onesRow = ones(1,4);
row2 = [(-d*a2*dtheta) (d*a2*dtheta-b1*b) (-d*a2*dtheta) (d*a2*dtheta+b1*b)];
row4 = [(d*a4*phi+b2*b) (-dphi*a4*d) (d*a4*dphi-b2*b) (-dphi*a4*d)];
row6 = b3*d*alternateOnes;
row8 = -b/m*cos(phi)*cos(theta)*onesRow;
row10 = b*ux/m*onesRow;
row12 = b*uy/m*onesRow;

B = [0 0 0 0;...
    row2;...
    0 0 0 0;...
    row4;...
    0 0 0 0;...
    row6;...
    zeroRow;...
    row8;...
    0 0 0 0;...
    row10;...
    zeroRow;...
    row12];

% U = [Om(1)^2; Om(2)^2; Om(3)^2; Om(4)^2];
% 
% res = A*v + B*U + G;