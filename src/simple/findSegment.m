function [new_segment, p0_new, p1_new] = findSegment( x, vertices, current_segment )
% Finds the current line segment for the MPC calculation to track
%   verteces listed from "start" to "goal"
    size(vertices)
    new_segment = current_segment;
    n_points = size(vertices,1);
    x = reshape(x, [numel(x),1]);
    D = inf(n_points-1,1);  % Distance Cost vector for each line segment
    
    for i = 1:(n_points-1)
        p1_new = vertices(i,:)';
        p0_new = vertices(i+1,:)';
        
	%unit vector in direction along segment being considered, towards goal
        v = (p1_new-p0_new)/norm(p1_new-p0_new,2);  

	% s0 negative if x is "past" plane thru goal-most point on segment, normal to v
        s0 = v'*(x-p0_new);
	% s1 negative if x is "before" plane thru start-most point on segment, normal to v
        s1 = -v'*(x-p1_new);

        if s0 <= 0
            D(i) = inf; % Dist Cost: inf (don't consider a segment we have "passed" already)
        elseif s1 < 0
            D(i) = norm(x-p1_new, 2); % Dist Cost: how far from start point we are
        elseif isnan(s0) || isnan(s1)
            D(i) = inf;
        else
		% x is between start-most and end-most planes
            D(i) = norm(cross(v,x), 2); % Dist Cost: how far we are (normal) from line
        end
        [minVal,new_segment] = min(D);
    end

	% skip short line segments
    while (new_segment < n_points-1) && (norm(vertices(new_segment,:) - vertices(new_segment+1,:), 2) <= .1) 
        new_segment = new_segment + 1;
        %'skipped'
    end
    new_segment
    p1_new = vertices(new_segment,:)';
    p0_new = vertices(new_segment+1,:)';
end

