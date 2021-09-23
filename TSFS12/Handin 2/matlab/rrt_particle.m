%% RRT for a particle moving in a plane (2D world)

function [goal_idx, nodes, parents, Tplan] = rrt_particle(start, goal, world, opts)

    % Input arguments:
    % start - initial state
    % goal - desired goal state
    % world - description of the map of the world
    %         using an object from the class BoxWorld
    % opts - structure with options for the RRT
    
    % Output arguments:
    % goal_idx - index of the node closest to the desired goal state
    % nodes - 2 x N matrix with each column representing a state j
    %         in the tree
    % parents - 1 x N vector with the node number for the parent of node j 
    %           at element j in the vector (node number counted as column
    %           in the matrix nodes)
    % Tplan - the time taken for computing the plan
    
    % Sample a state x in the free state space
    function x = sample_free()
        if rand < opts.beta
            x = goal;
        else
            found_random = false;
            while ~found_random
                x = [rand*(world.xmax-world.xmin) + world.xmin;...
                    rand*(world.ymax-world.ymin) + world.ymin];
                if world.obstacle_free(x)
                    found_random = true;
                end
            end
        end
    end

    % Find index of state nearest to x in the matrix nodes
    function idx = nearest(x)
        [~, idx] = min(sum((nodes-x).^2, 1));
    end

    % Steer from x1 towards x2 with step size opts.lambda
    % 
    % If the distance to x2 is less than opts.lambda, return
    % state x2.
    function x_new = steer(x1, x2)
        if norm(x2 - x1) < opts.lambda
            x_new = x2;
        else
            step = opts.lambda;
            x_new = x1 + step*(x2 - x1)/norm(x2 - x1);
        end
    end

    % Start time measurement and define variables for nodes and parents
    tic;
    nodes = [start];
    parents = [1]; % Initial state has no parent

    % YOUR CODE HERE
    goal_found = false;
    j = 2; % j counts the newest node
    while ~goal_found
        % Create random node
        x_random = sample_free();
        
        % Find nearest node index to random node
        closest_node_index = nearest(x_random);
        
        % Create new node that is in reach of closest node
        x_new = steer(nodes(:,closest_node_index),x_random);
        
        euclidian_dist = norm(x_new-nodes(:,closest_node_index));
        new_node_step = euclidian_dist/10; % Tuning parameter
        
        obstacle_found = false; 
        no_new_node = false; % if this is true no new node is added to the RRT.
        step_node = nodes(:,closest_node_index); % used for stepping closer to x_new in steps
        for i = 1:10 % 10 is the number of steps between lambda and new_node_step
            step_node = step_node + new_node_step*(x_new - nodes(:,closest_node_index))/norm(x_new - nodes(:,closest_node_index));
            if ~world.obstacle_free(step_node)
                if i == 1 % If the first node is an obstacle
                    % Ignore new node and find new random node
                    obstacle_found = true;
                    no_new_node = true;
                    break;
                else
                    % Go back one step to find node closest to obstacle
                    new_node = step_node - new_node_step*(x_new - nodes(:,closest_node_index))/norm(x_new - nodes(:,closest_node_index));
                    obstacle_found = true;
                    break;
                end
            end
        end
        
        % If no obstacle found random node is ok
        if ~obstacle_found
            new_node = x_new;
        end
        
        % If a new node should be added to "nodes"
        if ~no_new_node
            nodes(:,j) = new_node;
            parents(j) = closest_node_index;        
            j = j+1;
        end
        
        % If goal node is found, end the search
        if norm(new_node-goal) < abs(opts.eps)
            goal_found = true;
            break;
        end
        
        % Check if max nr of iterations is reached
        if j > opts.K
            fprintf('Max number of iterations reached \n')
            break;
        end
            
    end

    
    Tplan = toc;
    [~, goal_idx] = min(sum((nodes - goal).^2, 1));
end
