classdef StateFeedbackController < ControllerBase
    properties
        K;
        L;
        plan;
        goal_tol;
        state;
        s0;
    end
    methods
        function obj=StateFeedbackController(K, L, path, goal_tol)
            if nargin < 4
                goal_tol = 1;
            end
            obj = obj@ControllerBase();
            obj.K = K;  % Feedback gain
            obj.L = L;  % Vehicle wheel base
            obj.plan = path;  % Path to follow
            obj.goal_tol = goal_tol;  % Goal tolerance
            obj.s0 = 0.0;  % Path position state
        end

        function theta_e = heading_error(obj, theta, s)
            % theta, heading of vehicle
            % s - position (length) on path
            % Compute heading error. The SplinePath method heading is useful
            
            % YOUR CODE HERE
            [h_s, ~] = obj.plan.heading(s);
            h_s = h_s';
            
            h = [cos(theta);sin(theta)];
            
            cos_theta_e = dot(h_s,h)/(norm(h_s)*norm(h));
            sin_theta_e = cross([h_s;0],[h;0])/(norm(h_s)*norm(h));
            
            theta_e = atan2(sin_theta_e(3),cos_theta_e);
            
        end

        function c = u(obj, t, w)
            theta = w(3);
            p_car = w(1:2);
            
            % Compute d and theta_e errors. Use the SplinePath method project
            % and the obj.heading_error() function you've written above

            % YOUR CODE HERE
            %d = obj.path_error(p_car);
            [obj.s0, d] = obj.plan.project(p_car,obj.s0);
            
            theta_e = obj.heading_error(theta,obj.s0);

            % Compute control signal delta
            acc = 0;  % Constant speed
            
            look_ahead = 1;
            s_curvature = obj.plan.c(obj.s0+look_ahead);
            u0 = s_curvature;
            u = u0-obj.K*[d;theta_e];
            delta = atan(obj.L*u); % Steering angle

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
