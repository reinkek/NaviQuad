function [new_segment, p0_new, p1_new] = findSegment( x, vertices, current_segment )
% Finds the current line segment for the MPC calculation
%   verteces listed from "start" to "goal"
    size(vertices)
    new_segment = current_segment;
    n_points = size(vertices,1);
    x = reshape(x, [numel(x),1]);
    D = inf(n_points-1,1);
    
    for i = 1:(n_points-1)
        p1_new = vertices(i,:)';
        p0_new = vertices(i+1,:)';
        
        v = (p1_new-p0_new)/norm(p1_new-p0_new,2);

        s0 = v'*(x-p0_new);
        s1 = -v'*(x-p1_new);
        if s0 <= 0
            dp(i)=1;
            D(i) = inf;% norm(x-p0_new, 2); % distance from end point
        elseif s1 < 0
            dp(i)=2;
            D(i) = norm(x-p1_new, 2); % how far from start point we are
        elseif isnan(s0) || isnan(s1)
            D(i) = inf;
        else
            dp(i)=3;
            D(i) = norm(cross(v,x), 2); % check how far we are from line
        end
        [minVal,new_segment] = min(D);
    end

    while (new_segment < n_points-1) && (norm(vertices(new_segment,:) - vertices(new_segment+1,:), 2) <= .1) 
        new_segment = new_segment + 1;
        %'skipped'
    end
    new_segment
    p1_new = vertices(new_segment,:)';
    p0_new = vertices(new_segment+1,:)';
end

