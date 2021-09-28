
function y = h3a(x, measpar)
% Measurement function
%
% The measurement function gives the measurements available to the agent.
%
% Input arguments:
%   x - the complete state vector of all agents in the multi-agent system
%   measpar - structure containing parameters used in the function. In
%             this case the indices in the state vector x of the states
%             measured by the agent stored in meas_idx.
%
% Output:
%   y - The measurement vector

%if size(x_previous) > 15
%    y = [x(measpar.meas_idx(1:2)) x(measpar.meas_idx(3:4))-x_previous(measpar.meas_idx(3:4))/0.05];
%else
    y = x(measpar.idx);
%end

end