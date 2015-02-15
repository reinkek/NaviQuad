function U = call_mpc( x0, p0, p1 , A, B, G, N)
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here

n_states = length(x0);

xyz_loc = [10; 12; 8];

%N = 10; % number of look-ahead steps
R = eye(4); % penalty of controls
y0 = x0;
y0(xyz_loc) = y0(xyz_loc) - p0; % make the end point the value of zero
v = (p1-p0)/norm(p1-p0,2); % unit direction vector pointing from goal to start
Rsqrt = sqrtm(R);
margin = 100; % constraint on how far away from trajectory we are
umax = 1000*ones(4,N); % maximum control input

Cp = 1;
Cg = 1;

theta = 4;
phi = 2;
psi = 5;

cvx_begin %quiet
    variables Y(n_states,N+1)  U(4,N)
    
    d = v'*Y(xyz_loc,:); % projection of current location 
                % onto unit vector pointing to start of segment
    S = repmat(v, 1,N+1)*diag(d); % 
     H = [Y(theta,1:(end-1)) * cos(y0(phi)) + Y(phi,1:(end-1)) * sin(y0(phi)); 
          Y(theta,1:(end-1)) * sin(y0(phi)) - Y(phi,1:(end-1)) * cos(y0(phi))];
     dY = (Y(xyz_loc(1:2),2:end) - Y(xyz_loc(1:2),1:(end-1)));
     for i = 1:N
         H(:,i) = H(:,i)/norm(H(:,i));
         dY(:,i) = dY(:,i)/norm(dY(:,i));
     end
    % penalize distance squared from line, distance away from "end" of
    % segment, attitude pointing away from direction of motion, and control "effort"
    minimize( Cp*norm(S-Y(xyz_loc,:),'fro') + Cg*sum(pos(d)) + Ch*norm((H - dY),'fro') + norm( Rsqrt*U, 'fro') ) %
    subject to
        %dynamics
        Y(:,1) == y0;
        Y(:,2:N+1) == A*Y(:,1:N)+B*U + repmat(G,1,N);
        % control limits
        U <= umax;
        U >= 0;
        % state limits
        %cross(v,Y(1:3,N+1)) == 0;  %final point must lie on line
 
        for i = 2:N+1
            % enforce max distance from line (consider changing cost to
            % barrier function instead)
            norm( S(:,i) - Y(1:3,i), 2 ) <= margin;
        end
        
cvx_end

end

