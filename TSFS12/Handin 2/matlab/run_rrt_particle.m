%% Run the RRT for a particle moving in a plane (2D world)

clear
close all
addpath Functions

%% Define world with obstacles

world = BoxWorld({[0, 10], [0, 10]});

world.add_box(0.5, 1, 2, 4)
world.add_box(0, 6, 6, 4)
world.add_box(3, 1, 6, 4)
world.add_box(6.1, 7, 3, 3)

figure(10)
clf()
world.draw()
axis([world.xmin, world.xmax, world.ymin, world.ymax])
xlabel('x');
ylabel('y');

start = [1; 0];     % Start state
goal = [6.1; 10];      % Goal state

% Define parameters and data structures

opts.beta = 0.05; % Probability of selecting goal state as target state 
                  % in the sample
opts.lambda = 0.1; % Step size
opts.eps = -0.01; % Threshold for stopping the search (negative for full search)
opts.K = 5000;    % Maximum number of iterations, if eps < 0

%% Solve problem

fprintf('Planning ...\n');
[goal_idx, nodes, parents, T] = rrt_particle(start, goal, world, opts);
fprintf('Finished in %.2f sek\n', T);

%% YOUR CODE HERE

% Hint on plotting: To plot the path corresponding to the found solution,
% the following code could be useful (utilizing backtracking from the goal 
% node:
%
hold on
idx = goal_idx;
for i = 1:size(nodes,2)
    ll = [nodes(:, i), nodes(:, parents(i))];
    plot(ll(1, :), ll(2, :), 'k', 'LineWidth', 1)
end

nr_nodes_goal = 1;
path_length = 0;
while idx > 1
    ll = [nodes(:, parents(idx)), nodes(:, idx)];
    plot(ll(1, :), ll(2, :), 'b', 'LineWidth', 2)
    idx = parents(idx); 
    nr_nodes_goal = nr_nodes_goal + 1;
    path_length = path_length + norm(ll(:,1)-ll(:,2));
end

