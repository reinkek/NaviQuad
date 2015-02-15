% Script that allows quadcopter to follow a line

%% Global variables
global Jr Ixx Iyy Izz l m g b d Om

%% Quadcopter parameters

Ixx = 7.5*10^(-3);  % Quadrotor moment of inertia around X axis
Iyy = 7.5*10^(-3);  % Quadrotor moment of inertia around Y axis
Izz = 1.3*10^(-2);  % Quadrotor moment of inertia around Z axis
Jr = 6.5*10^(-5);  % Total rotational moment of inertia around the propeller axis
b = 3.13*10^(-5);  % Thrust factor
d = 7.5*10^(-7);  % Drag factor
l = 0.23;  % Distance to the center of the Quadrotor
m = 0.65;  % Mass of the Quadrotor in Kg
g = 9.81;   % Gravitational acceleration

%% Start simulation
p0 = [1; 1; -1]; % end point

p1 = [0; 0; 0]; % start point

v = 0.00*rand(16,1); % starting quadcopter state

rpm_bal = 225.6779;

Om = 1.02*rpm_bal*ones(4,1);
%  Om(1) = 1.01*rpm_bal;
%  Om(2) = 1.01*rpm_bal;
%  Om(3) = 0.98*rpm_bal;
%  Om(4) = 0.96*rpm_bal;

v(13:16) = Om.^2;

N = 40; % number of look-ahead steps

% Start control-dynamics loop

n = 1000; % number of simulation steps

dt = 0.01; % time step

Y = []; % variable size state array
T = []; % variable size time array

ti = 0;
tf = dt;

for i = 1:n
    
    % Linearize dynamics
    [A,B,G] = linearize_quad_dyn_xyz_controls(v);
    
    Anew = eye(size(A)) + A*dt;
%     Anew(end-3,end-3) = 1;
%     Anew(end-2,end-2) = 1;
%     Anew(end-1,end-1) = 1;
%     Anew(end-0,end-0) = 1
    Bnew = dt*B;
    Gnew = dt*G;
    
    % Call MPC
    [Yval, U] = call_mpc_2( v, p0, p1, Anew, Bnew, Gnew, N);
    
    % Get control inputs
    U(:,1)
%     Yval
    v(13:16) = Om.^2 + U(:,1);
    
    Om = v(13:16).^(1/2)
    
    % Call full dynamics for dt time
    [Ttemp,Ytemp] = ode45(@quad_dyn,[ti tf],v(1:12));
    T = [T; Ttemp];
    Y = [Y; Ytemp];
    
    % Update simulation parameters
    v = [Y(end,:)'; v(13:16)];
    ti = tf;
    tf = tf + dt;
    
    
    figure(3)
    hold off
    plot3(Y(:,10),Y(:,12),Y(:,8))
    drawnow
    hold on
    plot3(Yval(10,:) + p0(1),...
        Yval(12,:) + p0(2),...
        Yval(8,:) + p0(3),'r-o')
    axis equal
    xlabel('x')
ylabel('y')
zlabel('z')
    drawnow

% drawnow

end

%%
figure(2)
% vals = {'dphi','phi','dtheta',...
%     'theta','dpsi','psi','dz','z','dx','x',...
%     'dy','y'};
vals = {'\phi','\psi','\theta','z','x',...
    'y'};
for j = 1:6
%     i = 2*j-1;
%     subplot(6,2,i)
%     plot(T,Y(:,i))
%     legend('plant','plant',2)
%     title(vals{i})
    i = 2*j;
    subplot(3,2,j)
    plot(T,Y(:,i),'lineWidth',2)
    xlabel('t')
    grid minor
    title(vals{j})
end

%[10; 12; 8]
%%
figure(3)
plot3(Y(:,10),Y(:,12),Y(:,8))
xlabel('x')
ylabel('y')
zlabel('z')
%hold on
%plot3([p0(1) p1(1)],[p0(2) p1(2)],[p0(3) p1(3)])
    

