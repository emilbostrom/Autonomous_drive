close all
clear all

addpath functions


% THE AGENTS IN THE MULTI-AGENT SYSTEM

% Define the structure 'agent'. that contains information about the four
% agents in this exampel. The functions f1, h1, and g1, used below, can be
% founnd in the directory 'functions'.

% Specify which dynamic model that is used
agent(1).f = @f2;
agent(2).f = @f2;
agent(3).f = @f2;
agent(4).f = @f2;
agent(5).f = @f2;
agent(6).f = @f2;
agent(7).f = @f2;
agent(8).f = @f2;

% The dynamic model f1 is as single integrator model. You control the
% velocity of the agent. The acceleration of (or force on) the agent will
% be controlled in later assignments.

% The model parameter is the mass, which is not used in this example
modelparam.m = 1;
modelparam.w = 0.5;

% Specify which set of parameters that is used in the dynamic model.
agent(1).mdlpar = modelparam;
agent(2).mdlpar = modelparam;
agent(3).mdlpar = modelparam;
agent(4).mdlpar = modelparam;
agent(5).mdlpar = modelparam;
agent(6).mdlpar = modelparam;
agent(7).mdlpar = modelparam;
agent(8).mdlpar = modelparam;

% Specify which controller model that is used
agent(1).g = @g3a;
agent(2).g = @g3b;
agent(3).g = @g3b;
agent(4).g = @g3b;
agent(5).g = @g3b;
agent(6).g = @g3b;
agent(7).g = @g3b;
agent(8).g = @g3b;

% The controllerT g1 is a P-controller, which gives a control signal
% proportional to the vector between the current position and the reference
% position. A PD-controller will be used later.


% Two different proportionality constants are used by the controller in
% this example.
controlparam1.kp = 2;
controlparam1.kv = 2;

% Specify which set of parameters that is used by the controller
agent(1).ctrlpar = controlparam1;
agent(2).ctrlpar = controlparam1;
agent(3).ctrlpar = controlparam1;
agent(4).ctrlpar = controlparam1;
agent(5).ctrlpar = controlparam1;
agent(6).ctrlpar = controlparam1;
agent(7).ctrlpar = controlparam1;
agent(8).ctrlpar = controlparam1;

% Specify which measure model that is used
agent(1).h = @h3a;
agent(2).h = @h3b;
agent(3).h = @h3b;
agent(4).h = @h3b;
agent(5).h = @h3b;
agent(6).h = @h3b;
agent(7).h = @h3b;
agent(8).h = @h3b;
% The measurment h1 gives the absolute position in a global coordinate
% system

% Specify index in the state vector of the measured signals
% agent(1).measpar.meas_idx = 1:4;
% agent(2).measpar.meas_idx = 5:8;
% agent(3).measpar.meas_idx = 9:12;
% agent(4).measpar.meas_idx = 13:16;

% Reference position for the agents
% agent(1).xref = @(t) [cos(2*t) sin(2*t) -2*sin(2*t) 2*cos(2*t)]'; % Moving counter-clockwise on a circle 1
% agent(2).xref = @(t) [2*cos(-t) 2*sin(-t) -2*sin(t) -2*cos(t)]'; % Moving clockwaise on a circle with radius 2
% agent(3).xref = @(t) [3 2 0 0]'; % Fixed position
% agent(4).xref = @(t) [5 4 0 0]'; % Fixed position

formation_ex5_2

% SIMULATION

n=4; %number of states for each agent

x01 = [1 0 0 0];
x02 = [2 0 0 0];
x03 = [3 0 0 0];
x04 = [4 0 0 0];   
x05 = [5 0 0 0];
x06 = [6 0 0 0];
x07 = [7 0 0 0];
x08 = [8 0 0 0];  

x0 = [x01 x02 x03 x04 x05 x06 x07 x08]'; % Initial values of the state vector.
tspan = 0:0.05:10; % Time vector for the simulation

% Simulate the system
[t,x] = ode45(@(t,x) multi_agent_ode(t,x,agent,n), tspan, x0);

% Plot the result of the simulation
for i=1:length(t)
    plot(x(i,1),x(i,2),'x',x(i,5),x(i,6),'o',x(i,9),x(i,10),'s',x(i,13),x(i,14),'d',x(i,17),x(i,18),'+',x(i,21),x(i,22),'h',x(i,25),x(i,26),'p',x(i,29),x(i,30),'<')
    axis([-2 6 -2 6])
    pause(0.03)
end


