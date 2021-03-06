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

switch input.auxdata.cost.method
    case 'Work'
        alpha = input.auxdata.cost.smoothing.power;
        output(1).integrand = ...
            SmoothAbs(Power.ankleOne,alpha) +...
            SmoothAbs(Power.hip,alpha) + ...
            SmoothAbs(Power.legOne,alpha) + ...
            SmoothAbs(Power.legTwo,alpha);
    case 'Squared'
        Scale = input.auxdata.cost.Scale;
        output(1).integrand = ...
            (Actuators(:,1)/Scale.leg).^2 + ...
            (Actuators(:,2)/Scale.leg).^2 + ...
            (Actuators(:,3)/Scale.ankle).^2 + ...
            (Actuators(:,4)/Scale.hip).^2;
   
    otherwise
        error('Unsupported cost function')
end

end