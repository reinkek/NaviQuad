% normal distribution parameters for three random obstacle clusters
% [x_1, y_1, z_1;
%  x_2, y_2, z_2;
%  x_3, y_3, z_3]
sigma_size = [ 2,3,15;
               2,3,15;
                3,6,6];
min_size = [5,6,12;
            5,6,12;
            2,11,2];
sigma_dist = [20,1,0;
                20,1,0;
                16,2,12];
mu_dist = [33,9,0;
            33,30,0;
            30,19,10];

% construct obstacle map
map = ObstacleMap(RectObstacle([0,0,-0.1],[75,50,-10])); % start with the ground.

for i = 1:6
    map.addObstacle([RectObstacle(abs(sigma_dist(1,:).*randn(1,3)+mu_dist(1,:))-[0,0,0.1], abs(sigma_size(1,:).*randn(1,3))+min_size(1,:)),...
        RectObstacle(abs(sigma_dist(2,:).*randn(1,3)+mu_dist(2,:))-[0,0,0.1], abs(sigma_size(2,:).*randn(1,3))+min_size(2,:)),...
        RectObstacle(abs(sigma_dist(3,:).*randn(1,3)+mu_dist(3,:))-[0,0,0.1], abs(sigma_size(3,:).*randn(1,3))+min_size(3,:))]);
end

