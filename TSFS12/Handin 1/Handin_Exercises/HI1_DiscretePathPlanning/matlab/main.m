%% TSFS12 Hand-in exercise 1: Discrete planning in structured road networks
clear
close all

addpath Functions

%% Read map information
mapfile = 'linkoping.osm'; 
figurefile = 'linkoping.png';
osm_map = load_osm_map(mapfile, figurefile);

num_nodes = osm_map.nodes.size(1); % Number of nodes in the map
%% Display basic information about the map
osm_map.info()

% Plot the map
figure(10)
osm_map.plotmap()
xlabel('Longitude (^o)')
ylabel('Latitude (^o)')
title('Linköping')

% Which nodes are neighbors to node number 111?
osm_map.neighbors(111)

% What are the distances (in m) between nodes 111-3401 and 111-3402?
full(osm_map.distancematrix(111, 3401))

% What is the position in latitude and longitude of node 3401
pos = osm_map.nodeposition(3401);
fprintf('Longitude=%.2f, Latitude=%.2f\n', pos(1), pos(2));

% Plot the distance matrix and illustrate sparseness
figure(20)
spy(osm_map.distancematrix>0)
density = full(sum(sum(osm_map.distancematrix>0)))/(num_nodes^2);
title(sprintf('Density %.2f%%', density*100));
xlabel('Node index');
ylabel('Node index');

%% Some pre-defined missions to experiment with. 
% Example, the first mission can be used in a planner with
%   planner(num_nodes, pre_mission{1}, f_next, cost_to_go)

pre_mission = {...
    struct('start', struct('id', 10907), 'goal', struct('id', 1025)), ...
    struct('start', struct('id', 3988), 'goal', struct('id', 4725)), ...
    struct('start', struct('id', 424), 'goal', struct('id', 365))};

mission = pre_mission{3};

%% Define planning mission
% figure(30)
% osm_map.plotmap()
% title('Linköping - click in map to define mission')
% hold on
% mission.start = osm_map.getmapposition();
% plot(mission.start.pos(1), mission.start.pos(2), 'bd', 'MarkerSize', 10);
% 
% mission.goal = osm_map.getmapposition();
% plot(mission.goal.pos(1), mission.goal.pos(2), 'bd', 'MarkerSize', 10);
% hold off
% 
% % Inspect the defined mission
% mission.start
% mission.goal


%% Define state update function. Here, the function computes neighbouring nodes
% and the corresponding distance
f_next = @(x) map_state_update(x, osm_map.distancematrix);

%% Plan using the pre-defined depth first planner
df_plan = depth_first(num_nodes, mission, f_next);
fprintf('Plan: %.1f m, %d visited nodes, planning time %.1f msek\n', ...
    df_plan.length, df_plan.num_expanded_nodes, df_plan.time*1000);

% Plot the resulting plan
figure(40)
osm_map.plotmap()
hold on
osm_map.plotplan(df_plan.plan, 'b', 'linewidth', 2);
hold off
legend(sprintf('%s (%.1f m)', df_plan.name, df_plan.length));

% Plot visited nodes during search
figure(41)
osm_map.plotmap()
hold on
osm_map.plotplan(df_plan.expanded_nodes, 'b.');
hold off
legend(sprintf('%s (%.1f m)', df_plan.name, df_plan.length));

%% Define heuristic for astar and best_first
% Hint: Function latlong_d is useful
heuristic = @(x, xg) latlong_distance(osm_map.nodeposition(x), osm_map.nodeposition(xg)); % WRITE_YOUR_CODE_HERE


%% Implement all planners and investigate

% Breadth first algorithm
breadth_first_plan = breadth_first(num_nodes, mission, f_next);
fprintf('Plan: %.1f m, %d visited nodes, planning time %.1f msek\n', ...
    breadth_first_plan.length, breadth_first_plan.num_expanded_nodes, breadth_first_plan.time*1000);
plot_result(breadth_first_plan,osm_map)

% Best first algorithm
best_first_plan = best_first(num_nodes, mission, f_next, heuristic);
fprintf('Plan: %.1f m, %d visited nodes, planning time %.1f msek\n', ...
    best_first_plan.length, best_first_plan.num_expanded_nodes, best_first_plan.time*1000);
plot_result(best_first_plan,osm_map)

% Dijkstras algirithm
dijkstras_plan = dijkstras(num_nodes, mission, f_next);
fprintf('Plan: %.1f m, %d visited nodes, planning time %.1f msek\n', ...
    dijkstras_plan.length, dijkstras_plan.num_expanded_nodes, dijkstras_plan.time*1000);
plot_result(dijkstras_plan,osm_map)

% A* algirithm
a_star_plan = a_star(num_nodes, mission, f_next, heuristic);
fprintf('Plan: %.1f m, %d visited nodes, planning time %.1f msek\n', ...
    a_star_plan.length, a_star_plan.num_expanded_nodes, a_star_plan.time*1000);
plot_result(a_star_plan,osm_map)

% Time vs length plot
figure()
hold on
plot(breadth_first_plan.time,breadth_first_plan.length,'*')
plot(best_first_plan.time,best_first_plan.length,'*')
plot(dijkstras_plan.time,dijkstras_plan.length,'*')
plot(a_star_plan.time,a_star_plan.length,'*')
title('Time vs Length')
xlabel('Time [s]')
ylabel('Length [m]')
legend('Breadth first','Best first','Dijsktras','A*')

% Time vs nr of nodes plot
figure()
hold on
plot(breadth_first_plan.num_expanded_nodes,breadth_first_plan.time,'*')
plot(best_first_plan.num_expanded_nodes,best_first_plan.time,'*')
plot(dijkstras_plan.num_expanded_nodes,dijkstras_plan.time,'*')
plot(a_star_plan.num_expanded_nodes,a_star_plan.time,'*')
title('Nr of nodes vs Time')
xlabel('Nr of nodes [-]')
ylabel('Time [s]')
legend('Breadth first','Best first','Dijsktras','A*')


%%
function plot_result(planner,map)
    % Plot the resulting plan
    figure()
    map.plotmap()
    hold on
    map.plotplan(planner.plan, 'b', 'linewidth', 2);
    hold off
    legend(sprintf('%s (%.1f m)', planner.name, planner.length));

    % Plot visited nodes during search
    figure()
    map.plotmap()
    hold on
    map.plotplan(planner.expanded_nodes, 'b.');
    hold off
    legend(sprintf('%s (%.1f m)', planner.name, planner.length));
end



