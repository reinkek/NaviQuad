classdef ObstacleMap < handle
    properties
        obstacles
        known
        changed
    end
    
    methods
        %constructor: accepts obstacle objects as inputs. no input is empty
        %map
        function map = ObstacleMap(varargin)
            if nargin == 0
                map.obstacles = [];
                map.known = [];
            elseif nargin == 1
                map.obstacles = varargin{1};
                map.known = false(size(varargin{1}));
            else
                error('this feature to be added. pass in single structarray')
            end
            map.changed = 1;
        end
        
        %add struct array of obstacles to the map
        function addObstacle(self, new_obstacles)
            self.obstacles = [self.obstacles, new_obstacles];
            self.known = [self.known, false(size(new_obstacles))];
        end

        function b = boundary(self, x)
            n = length(self.obstacles);
            b = NaN(1,n);
            for i = 1:n
                b(i) = self.obstacles(i).boundary(x);
            end
        end
        
        %returns true if point x is within collision space, false otherwise
        function c = checkCollision(self, x)
            [val, index] = min(self.boundary(x));
            if val <= 0
                c = index;
            else
                c = 0;
            end
        end
        
        function p = listPlanes(self)
            p = [];
            for i = 1:length(self.obstacles)
                p = [p; self.obstacles(i).planes()];
            end
        end
        
        function p = getKnownPlanes(self)
            k = find(self.known);
            p = [];
            for ind = k
                p = [p; self.obstacles(ind).planes()];
            end
        end
        
        function fid = writePlanes(self, file)
            if ischar(file)
                fid = fopen(file,'w');
            else
                fid = file;
                frewind(fid);
            end
            
            for i = 1:length(self.obstacles)
                 fprintf(fid,'%f %f %f \r\n',self.obstacles(i).planes()');
            end
        end
        
        function fid = writeKnownPlanes(self, file)
            if ischar(file)
                fid = fopen(file,'w');
            else
                fid = file;
            end
            
            k = find(self.known);
            for ind = k
                fprintf(fid,'%f %f %f \r\n',self.obstacles(ind).planes()');
            end
        end
        
        function updateKnown(self, sensor)
            unknown = find(self.known == 0);
            for ind = unknown
                 vis = self.obstacles(ind).checkVis(sensor, true);
                 if true == vis
                    self.known(ind) = true;
                    self.changed = true;
                 end
            end
        end
        
        function resetChanged(self)
            self.changed = false;
        end
        
        function tf = hasChanged(self)
            tf = self.changed;
        end
        
        %plot the obstacle shapes in 3d glory
        function draw(self,figNum)
            figure(figNum);
            hold all;
            for i = 1:length(self.obstacles)
                if self.known(i)
                    color = 'b';
                else
                    color = 'r';
                end
                
                obs = self.obstacles(i).planes();
                
                for j = 1:4:size(obs,1)
                    obsPlane = obs(j:j+3,:);
                	fill3([obsPlane(:,1)', obsPlane(1,1)] ...
                         ,[obsPlane(:,2)', obsPlane(1,2)] ...
                         ,[obsPlane(:,3)', obsPlane(1,3)] ...
                         ,color,'EdgeAlpha',0);
                    alpha(0.1);
                end
            end
            set(gca, 'DataAspectRatio',[1,1,1]);
            xlabel('x1');
            ylabel('x2');
            zlabel('x3');
        end
        
        % plot known
        function drawKnown(self,figNum)
            figure(figNum);
            hold all;
            k = find(self.known);
            for i = k
%                 if self.known(i)
                    color = 'b';
%                 else
%                     color = 'r';
%                 end
                
                obs = self.obstacles(i).planes();
                
                for j = 1:4:size(obs,1)
                    obsPlane = obs(j:j+3,:);
                	fill3([obsPlane(:,1)', obsPlane(1,1)] ...
                         ,[obsPlane(:,2)', obsPlane(1,2)] ...
                         ,[obsPlane(:,3)', obsPlane(1,3)] ...
                         ,color,'EdgeColor',[.91,.91,.91]);
                    alpha(0.05);
                end
            end
            set(gca, 'DataAspectRatio',[1,1,1]);
            xlabel('x1');
            ylabel('x2');
            zlabel('x3');
        end
    end
end
