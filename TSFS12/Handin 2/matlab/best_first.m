function res = best_first(num_nodes, mission, f , dist, num_controls)
    if nargin < 5
        num_controls = 0;
    end
    previous = zeros(num_nodes, 1);
    control_to_come = zeros(num_nodes, num_controls);
    cost_to_come = zeros(num_nodes, 1);
    expanded_nodes = [];

    tic;
    q = PriorityQueue();
    q.insert(mission.start.id,0);

    foundPlan = false;
    while ~q.isempty()
        x = q.pop();
        expanded_nodes = [expanded_nodes x];
        if x == mission.goal.id
            foundPlan = true;
            break;
        end
        [neighbours, u, d] = f(x);
        for k=1:numel(neighbours)
            xi = neighbours(k);
            di = d(k);
            ui = u(k, :);
            if previous(xi) == 0
                previous(xi) = x;
                priority = dist(xi,mission.goal.id);
                q.insert(xi,priority)
                cost_to_come(xi) = cost_to_come(x) + di;
                if num_controls > 0
                    control_to_come(xi, :) = ui;
                end
            end
        end
    end

    % Collect plan by traversing visited nodes
    if ~foundPlan
        plan = [];
        control = [];
        length = 0;
    else 
        % collect the plan
        plan = [mission.goal.id];
        control = [];
        length = cost_to_come(mission.goal.id);
        while plan(1) ~= mission.start.id
            control = [control_to_come(plan(1),:);control];      
            plan = [previous(plan(1)) plan];
        end
    end
    res.time = toc;
    res.plan = plan;
    res.length = length;
    res.num_expanded_nodes = numel(expanded_nodes);
    res.expanded_nodes = expanded_nodes;
    res.name = 'Best First';
    res.control = control;
end