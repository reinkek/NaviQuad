function [A,B,G] = linearize_quad_dyn_xyz_controls(v)

% linearized quadcopter dyanmics

% linearized quadcopter dyanmics

%% Global variables
global Jr Ixx Iyy Izz l m g b d  vtemp dOmega

%% Constants
a1 = (Iyy - Izz)/Ixx;
a2 = Jr/Ixx;
a3 = (Izz - Ixx)/Iyy;
a4 = Jr/Iyy;
a5 = (Ixx - Iyy)/Izz;

b1 = l/Ixx;
b2 = l/Iyy;
b3 = 1/Izz;

% dphi = vtemp(1);
% phi = vtemp(2);
% dtheta = vtemp(3);
% theta = vtemp(4);
% dpsi = vtemp(5);
% psi = vtemp(6);

dphi = v(1);
phi = v(2);
dtheta = v(3);
theta = v(4);
dpsi = v(5);
psi = v(6);
% 
% dz = v(7);
% z = v(8);
% dx = v(9);
% x = v(10);
% %dy = v(11);
% y = v(12);
Om2 = v(13:16);


U1 = b*(Om2(1) + Om2(2) + Om2(3) + Om2(4));
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


A3 = zeros(size(A1,2),size(A2,1));

% Equations affecting ddz
A3(2,1) = 1/3*cos(theta)*sin(phi)*U1/m; %ddz(phi)
A3(2,3) = 1/3*cos(phi)*sin(theta)*U1/m; %ddz(theta)

% Equations affecting ddx
A3(4,1) = (-1/4*sin(theta)*cos(psi)*sin(phi) ...
    + 1/3*sin(psi)*cos(phi))*U1/m; %ddx(phi)
A3(4,3) = 1/4*cos(phi)*cos(theta)*cos(psi)*U1/m; %ddx(theta)
A3(4,5) = (-1/4*cos(phi)*sin(theta)*sin(psi)...
    + 1/3*sin(phi)*cos(psi))*U1/m; %ddx(psi)

% Equations affecting ddy
A3(6,1) = (-1/4*sin(phi)*sin(theta)*sin(psi)...
    -1/3*cos(phi)*cos(psi))*U1/m; %ddy(phi)
A3(6,3) = 1/4*cos(phi)*cos(theta)*sin(psi);%ddy(theta)
A3(6,5) = (1/4*cos(phi)*cos(theta)*cos(psi)...
    + 1/3*sin(phi)*sin(psi))*U1/m; 

A2 = [A1 zeros(size(A1,1),size(A2,2));...
    A3 A2];
% ddz_source = g;
ddz_source = g - 1/3*cos(phi)*(cos(theta) + sin(theta)*theta)*U1/m ...
    - 1/3*cos(theta)*(cos(phi) + sin(phi)*phi)*U1/m; % CHECKED

ddx_source = (1/4*( (cos(phi) + sin(phi)*phi)*sin(theta)*cos(phi)...
    + cos(phi)*(sin(theta) - cos(theta)*theta)*cos(phi)...
    + cos(phi)*sin(theta)*(cos(phi) + sin(phi)*phi))...
    + 1/3*(sin(psi)*(sin(phi) - cos(phi)*phi)...
    + sin(phi)*(sin(psi) - cos(psi)*psi)))*U1/m;

ddy_source = (1/4*( (cos(phi)+phi*sin(phi))*sin(theta)*sin(psi)...
    +cos(phi)*(sin(theta)-theta*cos(theta))*sin(psi)...
    +cos(phi)*sin(theta)*(sin(psi)-psi*cos(psi)) )...
    + 1/3*(-(sin(phi)-phi*cos(phi))*cos(psi)...
    -sin(phi)*(cos(psi)+psi*sin(psi)) ))*U1/m;


G2 = [zeros(size(A1,1),1);...
    [0;...
    ddz_source;...
    0; ddx_source; 0; ddy_source] ];
%% Finding large B matrix
zeroRow = zeros(1,4);
alternateOnes = [-1 1 -1 1];
onesRow = ones(1,4);
row2 = [(-d*a2*dtheta) (d*a2*dtheta-b1*b) (-d*a2*dtheta) (d*a2*dtheta+b1*b)];
row4 = [(d*a4*phi+b2*b) (-dphi*a4*d) (d*a4*dphi-b2*b) (-dphi*a4*d)];
row6 = b3*d*alternateOnes;
row8 = -1/3*b/m*cos(phi)*cos(theta)*onesRow;
row10 = b*(1/4*cos(phi)*sin(theta)*cos(psi) ...
    + 1/3*sin(phi)*sin(psi))/m*onesRow;
row12 = b*(1/4*cos(phi)*sin(theta)*sin(psi)...
    -1/3*sin(phi)*cos(psi))/m*onesRow;

B2 = [0 0 0 0;...
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

% U2 = [Om(1)^2; Om(2)^2; Om(3)^2; Om(4)^2];
ncontrols = size(B2,2);

A = [A2 B2;...
    zeros(ncontrols,size(A2,2)) zeros(ncontrols)];

B = [B2; eye(ncontrols)];

G = [G2; zeros(ncontrols,1)];

% res = A*v + B*dOmega + G;
% 
% A2
% B2
% G2
    
