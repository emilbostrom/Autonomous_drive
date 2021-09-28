
function dxdt = f2(t, x, u, mdlpar)
% Dynamic function for single integrator model
%     
% Input arguments are:
%   t - time 
%   x - state vector of the agent (position) 
%   u - the control input u, 
%   mdlpar - structure with parameters of the model 
%     
% Output:
%   dxdt - state-derivative

A = [0 0 1 0;
     0 0 0 1;
     0 0 0 0;
     0 0 0 0];
 
B = [0 0;
     0 0;
     1 0;
     0 1];

W = [1;
     0;
     0;
     0];

dxdt = A*x + B*u + W*mdlpar.w;
end
