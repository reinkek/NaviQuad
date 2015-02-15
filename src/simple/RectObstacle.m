classdef RectObstacle < Obstacle
    properties
        xc      %center as defined by relevant norm function
        scale   %defines how to scale norm to achieve desired dimensions
        vert    %vertices
    end
    methods 
        %constructor x is origin point, dx is change to oposing vertex
        %(sign matters here)
        function obst = RectObstacle(x,dx)
            x = reshape(x, 1,3);
            dx = reshape(dx, 1,3);
        %{
              1 - - 2
              |     |
              |     |
              3 - - 4
              |     |
              |     |
        3 - - 5 - - 6 - - 4
        |     |     |     |
        |     |     |     |
        1 - - 7 - - 8 - - 2
              |     |
              |     |
              1 - - 2
        %}

        % x, y, z values correspond to location of vertex
        % 5

        % Get locations of vertices
            obst.vert = [x+dx;...                      1
                        x(1),       x(2:3)+dx(2:3);...2
                        x(1),       x(2),       x(3)+dx(3);...3
                        x(1)+dx(1), x(2)        x(3)+dx(3);...4
                        x;...                           5
                        x(1)+dx(1), x(2),       x(3);...6
                        x(1)+dx(1), x(2)+dx(2), x(3);...7
                        x(1),       x(2)+dx(2), x(3)];% 8
        
            obst.xc = x+dx/2;
            obst.scale = abs(dx)/2;
        end
        
        %boundary function returns 0 if point x lies on boundary, 
        %negative if x lies inside boundary, 
        %positive if x lies outside boundary
        function b = boundary(self,x)
            b = max( abs(x - self.xc) - self.scale );
        end
            
        %output the vertices grouped into plane definitions
        function p = planes(self)
            p = [self.vert(1,:);self.vert(2,:);self.vert(3,:);self.vert(4,:);
                self.vert(1,:);self.vert(2,:);self.vert(8,:);self.vert(7,:);
                self.vert(2,:);self.vert(3,:);self.vert(5,:);self.vert(8,:);
                self.vert(5,:);self.vert(6,:);self.vert(4,:);self.vert(3,:);
                self.vert(7,:);self.vert(8,:);self.vert(5,:);self.vert(6,:);
                self.vert(7,:);self.vert(1,:);self.vert(4,:);self.vert(6,:)];
        end
        
        function feasibility = checkVis(self, sensor, shortcut)
            %check if any point of the obstacle is in the sensor's range
            %shortcut is bool flag to perform extra course check before rigorous check
            %(useful for large map-to-sensor-range ratio with many obstacles)
            %returns true if obstacle is within sensor range
            
            if shortcut
                if norm(self.scale) <= (self.xc - sensor.range)
                    feasibility = 0;
                    return
                end
            end
            % if invisibility can't be ruled out by shortcut: check cvx feasibility solution 
            cvx_begin quiet
                variable x(1,3)
                minimize 0
                subject to
                    norm( x - sensor.pos, 2) <= sensor.range
                    norm( (x - self.xc)./self.scale, inf) <= 1
            cvx_end
            %cvx_status
            feasibility =  ~(strcmpi(cvx_status, 'infeasible') || strcmpi(cvx_status, 'failed')) ;
        end
    end
end
