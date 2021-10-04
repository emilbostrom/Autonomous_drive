
function u = g4(y, xref, ctrlpar)
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

var = [];
xref_vec = [];
for i= 1:size(y,2)
    var = [var, norm(y(1:2,i))];
    xref_vec = [xref_vec, norm(xref(1:2,i))];
end

gamma_prim = 4*(var.^2-xref_vec.^2).*var;

u = ctrlpar.kp*sum(gamma_prim.*y(1:2,:)./var,2);

end

