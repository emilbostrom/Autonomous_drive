
function y = h3b(x, measpar)
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

y1 = [x(measpar.idx(2,:,1))-x(measpar.idx(1,:,1))];

% if size(measpar.idx,3) > 1 
%     y2 = [x(measpar.idx(2,:,2))-x(measpar.idx(1,:,2))];
%     y = [y1,y2];
% else
    y = y1;
% end

end