function output = Continuous_Single(input)

States = input.phase(1).state;
Actuators = input.phase(1).control;
Parameters = input.auxdata.dynamics;

[dStates, contactForces] = dynamics_single(States, Actuators, Parameters);
[Position, ~, Power] = kinematics_single(States, Actuators, Parameters);

output(1).dynamics = dStates;

output(1).path = zeros(size(dStates,1),2);
H1 = contactForces(:,1);
V1 = contactForces(:,2);
output(1).path(:,1) = atan2(H1,V1);
output(1).path(:,2) = Position.footTwo.y;

output(1).integrand = ...
    Power.ankleOne.^2 +...
    Power.hip.^2 + ...
    Power.legOne.^2 + ...
    Power.legTwo.^2;

end