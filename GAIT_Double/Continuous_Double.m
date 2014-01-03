function output = Continuous_Double(input)

States = input.phase(1).state;
Actuators = input.phase(1).control;
Parameters = input.auxdata.dynamics;

[dStates, contactForces] = dynamics_double(States, Actuators, Parameters);
[~, Power, ~] = kinematics_double(States, Actuators, Parameters);

output(1).dynamics = dStates;

output(1).path = zeros(size(dStates,1),2);
H1 = contactForces(:,1);
V1 = contactForces(:,2);
H2 = contactForces(:,3);
V2 = contactForces(:,4);
output(1).path(:,1) = atan2(H1,V1);
output(1).path(:,2) = atan2(H2,V2);

switch input.auxdata.cost.method
    case 'Work'
        alpha = input.auxdata.cost.smoothing.power;
        output(1).integrand = ...
            SmoothAbs(Power.legOne,alpha) + ...
            SmoothAbs(Power.legTwo,alpha);
    case 'Squared'
        output(1).integrand = ...
            Actuators(:,1).^2 + ...
            Actuators(:,2).^2;
    
    otherwise
        error('Unsupported cost function')
end

end