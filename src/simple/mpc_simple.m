function U = mpc_simple( x0,u0, p0, p1, dt, N )
%UNTITLED Summary of this function goes here
%   Detailed explanation goes here
%Tuning Parameters:
R = .3*eye(3); % control "effort"
Cp = 20;     % distance from path
Cg = 5;     % distance from path goal
Co = 1;

%constraints
%margin = 2;     %m
umax = 100*ones(1,N);   %Newton
alpha_max = pi/4;   %rad
w_max = 1.25*pi;   %rad/s

att_cone_param = abs(atan(alpha_max));
du_cone_param = abs(atan(w_max*dt));

%
n_states = length(x0);
y0 = x0-[p0; zeros(n_states - 3,1)];
v = (p1-p0)/norm(p1-p0,2);
Rhalf = sqrtm(R);

% dynamics parameters
m = 2.5;   %kg
A = [eye(3), dt*eye(3); zeros(3), eye(3)];
B = [zeros(3); dt/m * eye(3)];
G = repmat(dt*[0;0;0;0;0;-9.81], 1,N);

cvx_begin quiet
    variables Y(n_states,N+1)  U(3,N)
    
    d = v'*Y(1:3,:);
    S = repmat(v, 1,N+1)*diag(d);

    % penalize distance squared from line, distance away from "end" of
    % segment, and control "effort"
    minimize( Cp*norm(S-Y(1:3,:),'fro') + Cg*norm(Y(1:3,:),'fro') + norm( Rhalf*U, 'fro') )
    subject to
        %dynamics
        Y(:,1) == y0;
        Y(:,2:(N+1)) == A*Y(:,1:N) + B*U + G;
        % control limits
        for i = 1:N
            norm(U(:,i),2) <= umax;       % U restricted below max thrust available
            norm(U(1:2,i),2) <= att_cone_param*U(3,i);   %U lies in 2-norm cone describing max attitude angle from z-axis
        end

        norm(U(1:2,i)-u0(1:2),2) <= du_cone_param*u0(3);
        for i = 2:N
            norm(U(1:2,i)-U(1:2,i-1),2) <= du_cone_param*U(3,1:(i-1)); % du/dx,du/dy can only lie within 2-norm cone in plane of previous Uz 
        end 
        % state limits
        %cross(v,Y(1:3,N+1)) == 0;  %final point must lie on line
%         for i = 2:N+1
%             % enforce max distance from line (consider changing cost to
%             % barrier function instead)
%             norm( S(:,i) - Y(1:3,i), 2 ) <= margin;
%         end
        
cvx_end

end

