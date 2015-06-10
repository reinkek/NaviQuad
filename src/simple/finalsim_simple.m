% Script that allows quadcopter to follow a line
% This main script is a terrible, horrible mess, and needs to be given some organization and structure to be clear and maintainable!!!


% Movie parameters
iter = 106

Nframes = 100; % number of frames
%wVals = linspace(0.5,5,Nframes); % values used
iter
    figNum = 1;

% Prepare the movie
figure(figNum)
hold off
set(gcf,'Renderer','OpenGL') % create figure with options
    view([35 35])

set(gca,'NextPlot','replaceChildren');

% the video creation functionality does not work in some versions
% Generate a video object, like a file handle
vidname = ['sim_',num2str(iter),'.avi']
writerObj = VideoWriter(vidname);
% Set the FPS of the movie
set(writerObj, 'FrameRate',8);
% Open the video object
open(writerObj);





%% Start simulation
start = [10; 10; 10]; % start point

%goal = [0; 0; 0]; % end point

v = 0.001*ones(6,1); % starting quadcopter state

N = 20; % number of look-ahead steps

% Start control-dynamics loop

n = 200; % number of simulation steps

dt = 0.1; % time step

Y = zeros(6,1); % variable size state array
T = []; % variable size time array
U = zeros(3,1); %variable size input array
ti = 0;
tf = dt;

%generate randomized obstacle map
mapgen;
%draw the generated obstacle terrain
map.draw(figNum)

quad_sensor.range = 10;
%% Constants for RRT
obstacleFilename = 'knownPlanes_simp.txt';
seedsPerAxis = 7;
treesMax = seedsPerAxis^3*3+2;

%% Create file
locc = [];

%%
keep_going = true;
goal_tol = 2;
%%
m = 2.5;   %kg
A = [eye(3), dt*eye(3); zeros(3), eye(3)];
B = [zeros(3); dt/m * eye(3)];
G = dt*[0;0;0;0;0;-9.81];
D = -.06; %aero drag
wind = [0;-10;2].*randn(3,1);
i = 0;
while(keep_going)
    
    locc = [locc, Y(1:3,end)];
    
    quad_sensor.pos = Y(1:3,end)'; %set sensor as current xyz position
    map.updateKnown(quad_sensor);   %update map with sensor
    
    % Check if quad is in building
    inBuildingFlag = map.checkCollision(Y(1:3,end)');
    %RRT here

    if map.hasChanged() && ~inBuildingFlag
        
        map.writeKnownPlanes(obstacleFilename); %overwrite file with new known map
        segment = 1;    %reset segment to first since we will get a new list now
        map.resetChanged(); %map file is now current
        
        rrt = RrtPlanner(treesMax,seedsPerAxis,obstacleFilename);
        rrt.drawingSkipsPerDrawing = 30;
        map.drawKnown(figNum);
        
        % make new rrt structure every time.  delete structures after
        % extracting seg_list, just to be sure theres no problems with residual
        % data left behind and getting recycled...
        rrt.SetGoal(Y(1:3,end)');
        %rrt.SetStart(goal); % needs to be hardcoded into RrtPlanner.m instead due to some bug
        rrt.Run();
        seg_list = rrt.smoothedPath(end:-1:1, :); % get list backwards
        %plot3(rrt.path(:,1),rrt.path(:,2),rrt.path(:,3),'k*');
	%plot3(rrt.smoothedPath(:,1),rrt.smoothedPath(:,2),rrt.smoothedPath(:,3),'ko');
        clear rrt;
    end

    [segment, p0, p1] = findSegment(Y(1:3,end), seg_list, segment);
    %call MPC, add signal noise (GPS error)
    meas_state = Y(:,end) + [0.7*randn(3,1); zeros(3,1)];
    U = mpc_simple(meas_state, U(:,1), p0, p1, dt, N);
    
    %dynamics with added process noise (wind), and aero drag
    Y(:,end)
    wind = (1-0.08*abs(randn()))*wind + 15*dt*(randn(3,1) + [0; -1/3; 1/15])
    U(:,1)
    D*norm(Y(4:6,end))*Y(4:6,end)
    
    Y(:,end+1) = A*Y(:,end) + B*(U(:,1) + wind + D*norm(Y(4:6,end))*Y(4:6,end)) + G;
    
    
    plot3(Y(1,end),Y(2,end),Y(3,end),'r*')
    plot3(seg_list(:,1),seg_list(:,2),seg_list(:,3),'ko-');
    
    view([-60+i*1.35 30-25*cos(2*pi*i/150 )])
    %%
    drawnow
    % Write the movie
    set(gcf,'Renderer','OpenGL')
    frame = getframe(gcf);
    % Directly write the frame to video
     writeVideo( writerObj, frame ); 

    i = i+1;
    % check if we have reached the end
    if norm(Y(1:3,end)' - [60,35, 10]) < goal_tol
        keep_going = false;
    end
end

%% More movie angles! =)
for j = 1:200
    view([-60+(i+j)*1.35 30-25*cos(2*pi*(i+j)/150 )])
    %%
    drawnow
    % Write the movie
    set(gcf,'Renderer','OpenGL')
    frame = getframe(gcf);
    % Directly write the frame to video
     writeVideo( writerObj, frame ); 
end

% Close the video object
close(writerObj); 

figure(3)
plot3(Y(:,10),Y(:,12),Y(:,8))
xlabel('x')
ylabel('y')
zlabel('z')


