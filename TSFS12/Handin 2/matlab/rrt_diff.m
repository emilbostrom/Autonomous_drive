%% RRT for a model with a differential constraint defined by the input sim

% Example usage:
%      [goal_idx, nodes, parents, state_trajectories, Tplan] = 
%            rrt_diff(start, goal, u_c, @sim_car, world, opts);

function [goal_idx, nodes, parents, state_trajectories, Tplan] = ...
          rrt_diff(start, goal, u_c, sim, world, opts)
      
    % Input arguments:
    % start - initial state
    % goal - desired goal state
    % u_c - vector with possible control actions (steering angles)
    % sim - function reference to the simulation model of the car motion
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
    % state_trajectories - a struct with the trajectory segment for 
    %                 reaching node j at element j (node number counted 
    %                 as column in the matrix nodes)
    % Tplan - the time taken for computing the plan
    
    % Sample a state x in the free state space
    function x = sample_free()
        if rand < opts.beta
            x = goal;
        else
            found_random = false;
            th = rand*2*pi - pi;
            while ~found_random
                p = [rand*(world.xmax - world.xmin) + world.xmin;...
                    rand*(world.ymax - world.ymin) + world.ymin];
                if world.obstacle_free(p)
                    found_random = true;
                    x = [p; th];
                end
            end
        end
    end

    % Find index of state nearest to x in nodes
    function idx = nearest(x)
        [~, idx] = min(distance_fcn(nodes, x));
    end

    % Compute all possible paths for different steering control signals u_c
    % to move from x_nearest towards x_rand, without collision
    %
    % If no valid paths are found, the returned variables are empty
    function [valid_new_paths, dist_to_x_rand] = steer_candidates(x_nearest, x_rand)
        valid_new_paths = {};
        dist_to_x_rand = [];
        
        for k=1:length(u_c)
            p = sim(x_nearest, u_c(k), opts.lambda);
            if world.obstacle_free(p)
                valid_new_paths{end+1} = p;
                dist_to_x_rand(end+1) = distance_fcn(p(:, end), x_rand);
            end
        end
    end

    % Function for computing the distance between states x1 and x2, 
    % where x1 can be a matrix with several state vectors, treating  
    % all states equally
    function dist = distance_fcn(x1, x2)
        c = 1;
        dist = sqrt(sum((x1(1:2,:) - x2(1:2,:)).^2, 1)) + c.*abs(x1(3,:)-x2(3,:)); % euclidian distance + angular distance
    end

    % Start time measurement and define variables for nodes, parents, and 
    % associated trajectories
    tic;
    nodes = [start];
    parents = [1];
    state_trajectories = {0}; % No trajectory segment needed to reach start state

    % YOUR CODE HERE    
    goal_noad_found = false;
    j = 2;
    while goal_noad_found == false
        % Slumpa fram en punkt
        random_point = sample_free();

        % Hitta närmsta nod till denna punkt med distance_fcn
        closest_node_idx = nearest(random_point);
        closest_node = nodes(:,closest_node_idx);

        % Kör steer_candidates för att få möjliga vägar
        [valid_new_paths, dist_to_x_rand] = steer_candidates(closest_node,random_point);

        % Kolla om det finns nåt i state_trajectories
        if ~isempty(valid_new_paths)
            % % Bedöm vilken av vägarna som är bäst mha dist_to_x_rand
            [~,shortest_idx] = min(dist_to_x_rand);

            % Spara vägen i state_trajectories
            state_trajectories{j} = valid_new_paths{shortest_idx};

            % Spara parent till noden i parents
            parents(j) = closest_node_idx;

            % Spara den nya noden i nodes
            nodes(:,j) = state_trajectories{j}(:,end);

            % Kolla om vi är i goal noad 
            if distance_fcn(nodes(:,j),goal) < abs(opts.eps)
                goal_noad_found = true;
                break;
            end

            % Öka while loopen med 1
            j = j+1;
        else
            % Gå vidare utan att spara
            continue;
        end
        
        if j > opts.K
            fprintf('Max nr of cycles reached \n')
            break;
        end
    end
    
    Tplan = toc;
    [~, goal_idx] = min(distance_fcn(nodes,goal));
end
