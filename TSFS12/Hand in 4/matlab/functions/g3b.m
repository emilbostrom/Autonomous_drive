
function u = g3b(y, xref, ctrlpar)
% Control function
%     
% Compute the control signal, in this case it is a P-controller which gives
% a control signal proportional to the vector between the current position
% and the reference position. 
%  
% Input arguments:
%   y - measurement y, 
%   xref - the reference vector xref (in this case the desired position of
%          the agent)
%   ctrlpar - dictionary which contains parameters used by the controller 
%             (in this case the proportional gain k).
% 
% Output argument:
%   Control signal 

if size(y,2) > 1
    u = sum(ctrlpar.kp*(y(1:2,1:2)-xref(1:2,1:2)),2) + sum(ctrlpar.kv*(y(3:4,1:2)-xref(3:4,1:2)),2);
else
    u = sum(ctrlpar.kp*(y(1:2,1)-xref(1:2,1)),2) + sum(ctrlpar.kv*(y(3:4,1)-xref(3:4,1)),2);
end

