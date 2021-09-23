classdef PurePursuitController < ControllerBase
    properties
        l;
        L;
        plan;
        goal_tol;
        s;
    end
    methods
        function obj=PurePursuitController(l, L, path, goal_tol)
            obj = obj@ControllerBase();
            if nargin < 4
                goal_tol = 0.25;
            end
            obj.l = l;
            obj.L = L;
            obj.plan = path;
            obj.goal_tol = goal_tol;
            obj.s = 0;
        end

        function p_purepursuit = pursuit_point(obj, p_car)
            % Input: 
            %   p_car - position of vehicle in globval coordinates
            %
            % Output:
            %   p_purepursuit - position of pursuit point in global
            %                   coordinates.

            s = obj.s; % Last stored path parameter
            path_points = obj.plan.path;  % Points 
            l = obj.l;  % Pure-pursuit look-ahead
            % Your code here
%             new_points = [];
%             circle = linspace(0,2*pi,100);
%             for th = circle
%                 new_points = [new_points; (p_car + l*(cos(th)+sin(th)))];
%             end
            
            dist = pdist2(path_points,p_car);
            path_idx = find(abs(dist-l) < 1);
            new_point_idx = max(path_idx);
            
            % Hint: It is typically not important to find a point at _exactly_ distance l, 
            %       for example search pure-pursuit point among the points in path_points
            %       but don't forget to take into account the approximate pursuit-horizon when
            %       computing the steering angle.
            
            if isempty(new_point_idx)
                if s == 0
                    [~, s] = min(dist);
                end
                new_point_idx = s;
            end

            p_purepursuit = path_points(new_point_idx,:); 
            obj.s = new_point_idx;
        end
     
        function delta = pure_pursuit_control(obj, dp, theta)
            % Compute pure-pursuit steer angle.
            %
            %  Input:
            %    dp - Vector from position of car to pursuit point
            %    theta - heading of vehicle
            %
            % Output:
            %   delta - steer angle

            % Your code here to compute new steering angle
            x_angle = theta-pi/2;
            x_local = [cos(x_angle); sin(x_angle)]./norm([cos(x_angle); sin(x_angle)]);
            x_prim = dot(dp,x_local)/(norm(x_local)^2);
            
            delta = atan(-obj.L*2*x_prim/(obj.l^2));
        end

        function c = u(obj, t, w)
            % Compute control action
            %
            %  Input:
            %    t - current time
            %    w - current state w = (x, y, theta, v)
            %  
            %  Output:
            %    return [delta, acc] where delta is steer angle and acc acceleration

            p_car = w(1:2);
            theta = w(3);      

            % Your code here to compute steering angle, use the functions
            % obj.pursuit_point() and obj.pure_pursuit_control() you 
            % have written above.
            point_to_pursuit = obj.pursuit_point(p_car);
            dp = (point_to_pursuit-p_car)';
            delta = obj.pure_pursuit_control(dp,theta);
            acc = 0;
            c = [delta, acc];
        end
    
        function r = run(obj, t, w)
            % Function that returns true until goal is reached
            p_car = w(1:2);
            p_goal = obj.plan.path(end, :);
            r = norm(p_car - p_goal) > obj.goal_tol;
        end
    end
end
