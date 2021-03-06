clear
addpath Functions

%% Define a simple vehicle controller, simulate, and create a reference path
car = SingleTrackModel();  % Create car object
car.controller = MiniController();  % Create controller object and assign to car
w0 = [0, 0, 0, 2];  % Initial state (x, y, theta, v)
[t, w, u] = car.simulate(w0, 40, 0.1);  % Simulate closed-loop system

z_simple = {t, w, u};  % Save results
%fprintf('Total time in controller: %.2f msek\n', car.controller.u_time*1000);

M = 10;  % Path downsample factor
p = w(1:M:end, 1:2);
ref_path = SplinePath(p); %% Create path from MiniController output

s = linspace(0, ref_path.length, 200);

% Plot resulting paths and control signals
figure(10)
plot(ref_path.x(s), ref_path.y(s), 'b')
hold on
plot(p(:, 1), p(:,2), 'rx');
hold off
title('Path from simple controller');
xlabel('x [m]');
ylabel('y [m]');

figure(11)
subplot(1, 2, 1)
hold on
plot(t, u(:,1)*180/pi)
xlabel('t [s]')
ylabel('steer [deg]');
title('Steer');
box off

subplot(1, 2, 2)
plot(t, u(:,2))
xlabel('t [s]')
ylabel('Acceleration [m/s^2]')
title('Acceleration');
box off

%pp_error = 

%% Pure pursuit controller
% Create vehicle and controller objects
car = SingleTrackModel();
pp_controller = PurePursuitController(2, car.L, ref_path);
car.controller = pp_controller;

w0 = [0, 1, pi/2*0.9, 2];  % Sample starting state

% YOUR CODE HERE
[t, w, u] = car.simulate(w0,42,0.1);
fprintf('Total time in controller for PP: %.2f msek\n', car.controller.u_time*1000);

M = 10;  % Path downsample factor
p = w(1:M:end, 1:2);
purePursuitPath = SplinePath(p); %% Create path from MiniController output

PP_error = purePursuitPath.path_error(w(:,1:2));

s = linspace(0, purePursuitPath.length, 200);

% Plot resulting paths and control signals
figure(10)
hold on
plot(purePursuitPath.x(s), purePursuitPath.y(s), 'k')
hold off
title('Path from simple controller');
xlabel('x [m]');
ylabel('y [m]');

figure(11)
hold on
subplot(1, 2, 1)
plot(t, u(:,1)*180/pi)
xlabel('t [s]')
ylabel('steer [deg]');
title('Steer');
box off

%% LQR Path tracker with linear feedback

% YOUR CODE HERE
% Create vehicle and controller objects
car = SingleTrackModel();

v = 2;
A = v*[0 1;
     0 0];
B = v*[0; 1];
Ts = 0.1;
sys_cont = ss(A,B,[],[]);
sys_discrete = c2d(sys_cont,Ts);
A_disc = sys_discrete.A;
B_disc = sys_discrete.B;
Q = [1 0;
     0 1];
R = 1;
[X,K,L] = idare(A_disc,B_disc,Q,R);

sf_controller = StateFeedbackController(K, car.L, ref_path);
car.controller = sf_controller;

w0 = [0, 1, pi/2*0.9, 2];  % Sample starting state
%w0 = [-5, 10, pi/2*0.9, 2];

% YOUR CODE HERE
[t, w, u] = car.simulate(w0,42,0.1);
fprintf('Total time in controller for SF: %.2f msek\n', car.controller.u_time*1000);

M = 10;  % Path downsample factor
p = w(1:M:end, 1:2);
StateFeedbackPath = SplinePath(p); %% Create path from MiniController output

Lin_error = StateFeedbackPath.path_error(w(:,1:2));

s = linspace(0, StateFeedbackPath.length, 200);

% Plot resulting paths and control signals
figure(10)
hold on
plot(StateFeedbackPath.x(s), StateFeedbackPath.y(s), 'r')
hold off
title('Path from simple controller');
xlabel('x [m]');
ylabel('y [m]');

figure(11)
hold on
subplot(1, 2, 1)
plot(t, u(:,1)*180/pi)
xlabel('t [s]')
ylabel('steer [deg]');
title('Steer');
box off


%% LQR Path tracker with non-linear feedback

% YOUR CODE HERE
car = SingleTrackModel();

Q = [1 0;
     0 1];
R = 1;
[X,K,L] = idare(A_disc,B_disc,Q,R);

sf_non_lin_controller = NonLinearStateFeedbackController(K, car.L, ref_path);
car.controller = sf_non_lin_controller;

%w0 = [0, 1, pi/2*0.9, 2];  % Sample starting state
w0 = [-50, 10, pi/2*0.9, 2];

% YOUR CODE HERE
[t, w, u] = car.simulate(w0,42,0.1);
fprintf('Total time in controller for SF: %.2f msek\n', car.controller.u_time*1000);

M = 10;  % Path downsample factor
p = w(1:M:end, 1:2);
StateFeedbackPathNL = SplinePath(p); %% Create path from MiniController output

Non_Lin_error = StateFeedbackPathNL.path_error(w(:,1:2));

s = linspace(0, StateFeedbackPathNL.length, 200);

% Plot resulting paths and control signals
figure(10)
hold on
plot(StateFeedbackPathNL.x(s), StateFeedbackPathNL.y(s), 'm')
hold off
title('Path from simple controller');
xlabel('x [m]');
ylabel('y [m]');

figure(11)
hold on
subplot(1, 2, 1)
plot(t, u(:,1)*180/pi)
xlabel('t [s]')
ylabel('steer [deg]');
title('Steer');
box off

%%

figure(12)
hold on
plot(PP_error, 'k')
plot(Lin_error, 'b')
plot(Non_Lin_error, 'r')
hold off
title('Path error for the three controllers');
xlabel('Point');
ylabel('Error');
legend('PP','Lin','Non-Lin')
