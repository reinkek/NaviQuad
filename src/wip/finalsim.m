% Script that allows quadcopter to follow a line

%% Global variables
global Jr Ixx Iyy Izz l m g b d Om 

%goal = [80 45 30];
%% Movie parameters

Nframes = 100; % number of frames
%wVals = linspace(0.5,5,Nframes); % values used

    figNum = 1;

% Prepare the movie
figure(figNum)
hold off
set(gcf,'Renderer','OpenGL') % create figure with options
    view([35 35])

set(gca,'NextPlot','replaceChildren');

 % the video creation functionality does not work in some versions
    % Generate a video object, like a file handle
    writerObj = VideoWriter('sim_MPC_1.avi');
    % Set the FPS of the movie
    set(writerObj, 'FrameRate',8);
    % Open the video object
    open(writerObj);







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
start = [10; 10; 10]; % start point

%goal = [0; 0; 0]; % end point

v = 0.001*ones(16,1); % starting quadcopter state

rpm_bal = 225.6779;

Om = 1.06*rpm_bal*ones(4,1);
Om(1) = 1.01*rpm_bal;
Om(2) = 1.01*rpm_bal;
Om(3) = 0.98*rpm_bal;
Om(4) = 0.96*rpm_bal;

v(13:16) = Om.^2;

N = 40; % number of look-ahead steps

% Start control-dynamics loop

n = 200; % number of simulation steps

dt = 0.01; % time step

Y = zeros(12,1)'; % variable size state array
T = []; % variable size time array

ti = 0;
tf = dt;
xyz_loc = [10; 12; 8];


%generate a randomized obstacle map
mapgen;
%draw the map to figure
map.draw(figNum)

quad_sensor.range = 10;
%% Constants for RRT
obstacleFilename = 'knownPlanes2.txt';%./custom/test.txt';%'./custom/output_planes.txt';
seedsPerAxis = 7;
treesMax = seedsPerAxis^3*3+2;

%% Create file
locc = [];

%%
keep_going = true;
goal_tol = 1;
%%
i = 0;
while(keep_going)
    
    locc = [locc; Y(end,xyz_loc)];
    
    quad_sensor.pos = Y(end, xyz_loc); %set sensor as current xyz position
    map.updateKnown(quad_sensor);   %update map with sensor
    
    % Check if quad is in building
    inBuildingFlag = map.checkCollision(Y(end, xyz_loc))
    %RRT here

    if map.hasChanged() && ~inBuildingFlag
        
        map.writeKnownPlanes('knownPlanes2.txt'); %overwrite file with new known map
        segment = 1;    %reset segment to first since we will get a new list now
        map.resetChanged(); %map file is now current
        
        rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
        rrt.drawingSkipsPerDrawing = 30;
        map.drawKnown(figNum);
        
        % make new rrt structure every time.  delete structures after
        % extracting seg_list, just to be sure theres no problems with residual
        % data left behind and getting recycled...
        rrt.SetGoal(Y(end, xyz_loc));
%         rrt.SetStart(goal); % needs to be hardcoded
        rrt.Run();
        seg_list = rrt.smoothedPath(end:-1:1, :); % get list backwards
        %plot3(rrt.path(:,1),rrt.path(:,2),rrt.path(:,3),'k*');
% plot3(rrt.smoothedPath(:,1),rrt.smoothedPath(:,2),rrt.smoothedPath(:,3),'ko');
        clear rrt;
    end

    [segment, p0, p1] = findSegment(Y(end, xyz_loc)',seg_list, segment);
    
    p0
    p1
    segment
    
    ddt = 0.2;
    vv = 10;
    
    Y(end, xyz_loc) = Y(end, xyz_loc) +...
        (p0-Y(end, xyz_loc)')'/norm(p0-Y(end, xyz_loc)',2)*vv*ddt;
    
    
    plot3(Y(end, xyz_loc(1)),Y(end, xyz_loc(2)),Y(end, xyz_loc(3)),'r*')
    plot3(seg_list(:,1),seg_list(:,2),seg_list(:,3),'ko-');
    %%
    % -60 0  to 
    
    view([-60+i 0+i])
    %%
    drawnow
    % Write the movie
    set(gcf,'Renderer','OpenGL')
    frame = getframe(gcf);
    % Directly write the frame to video
     writeVideo( writerObj, frame ); 
    %%{
    
    % Linearize dynamics
    [A,B,G] = linearize_quad_dyn_xyz_controls(v);
    Anew = eye(size(A)) + A*dt;
    Bnew = dt*B;
    Gnew = dt*G;
    
    % Call MPC
    [Yval, U]= call_mpc_2( v, p0, p1, Anew, Bnew, Gnew, N);
    plot3(Yval(10,:)+Y(end,xyz_loc(1)),...
        Yval(12,:)+Y(end,xyz_loc(2)),...
        Yval(8,:)+Y(end,xyz_loc(3)),'g-')
    % Get control inputs
    U(:,1)
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
    %}
    i = i+1;
    % TODO: check if we have reached the end
    if norm(Y(end, xyz_loc) - [60,35, 10]) < goal_tol
        keep_going = false;
    end
end
%%
% Close the video object
 close(writerObj); 

%%
%{
figure(2)
vals = {'dphi','phi','dtheta',...
    'theta','dpsi','psi','dz','z','dx','x',...
    'dy','y'};
for j = 1:6
    i = 2*j-1;
    subplot(6,2,i)
    plot(T,Y(:,i))
    %     legend('plant','plant',2)
    title(vals{i})
    i = 2*j;
    subplot(6,2,i)
    plot(T,Y(:,i))
    title(vals{i})
end
%}
%[10; 12; 8]
%%
figure(3)
plot3(Y(:,10),Y(:,12),Y(:,8))
xlabel('x')
ylabel('y')
zlabel('z')
%hold on
%plot3([p0(1) p1(1)],[p0(2) p1(2)],[p0(3) p1(3)])


