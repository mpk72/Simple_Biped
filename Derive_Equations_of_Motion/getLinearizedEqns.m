function [A, B] = getLinearizedEqns(States,Actuators,Dynamics)

%FUNCTION: 
%   This function takes a list of states and actuators, and then
%   symbolically linearizes the equations of motion
%
%INPUTS:
%   States: [n x 2] cell array with state names in the first column and
%       comments in the second column
%   Actuators: [n x 2] cell array with actuator names in the first column 
%       and comments in the second column
%   Dynamics: A struct with a field for each symbolic acceleration
%
%OUTPUTS:
%   A = jacobian(dynamics, states);
%   B = jacobian(dynamics, actuators);
%

nState = size(States,1);
X = sym('X',[nState,1]);
for i=1:nState
   X(i) = sym(States{i,1});
end

nAcc = nState/2;
dX = sym('dX',[nState,1]);
for i=1:nAcc
   dX(i) = sym(['d' States{i,1}]); 
end
for i=(1+nAcc):(2*nAcc)
    dX(i) = Dynamics.(['d' States{i,1}]);
end

nActuator = size(Actuators,1);
U = sym('U',[nActuator,1]);
for i=1:nActuator
   U(i) = sym(Actuators{i,1}); 
end

A = simplify(jacobian(dX,X));
B = simplify(jacobian(dX,U));

end