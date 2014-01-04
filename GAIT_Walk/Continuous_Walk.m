function output = Continuous_Walk(input)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Phase 1  --  Double Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 1;

States = input.phase(iphase).state;
Actuators = input.phase(iphase).control;
Parameters = input.auxdata.dynamics;

[dStates, contactForces] = dynamics_double(States, Actuators, Parameters);

output(iphase).dynamics = dStates;

output(iphase).path = zeros(size(dStates,1),2);
H1 = contactForces(:,1);
V1 = contactForces(:,2);
H2 = contactForces(:,3);
V2 = contactForces(:,4);
output(iphase).path(:,1) = atan2(H1,V1);
output(iphase).path(:,2) = atan2(H2,V2);

switch input.auxdata.cost.method
    case 'Work'
        Power = getPower_double(States, Actuators, Parameters);
        alpha = input.auxdata.cost.smoothing.power;
        output(iphase).integrand = ...
            SmoothAbs(Power.legOne,alpha) + ...
            SmoothAbs(Power.legTwo,alpha);
    case 'Squared'
        Scale = input.auxdata.cost.Scale;
        output(iphase).integrand = ...
            (Actuators(:,1)/Scale.leg).^2 + ...
            (Actuators(:,2)/Scale.leg).^2;
    otherwise
        error('Unsupported cost function')
end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Phase 2  --  Single Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 2;

States = input.phase(iphase).state;
Actuators = input.phase(iphase).control;
Parameters = input.auxdata.dynamics;

[dStates, contactForces] = dynamics_single(States, Actuators, Parameters);
Position = getPosVel_single(States);

output(iphase).dynamics = dStates;

output(iphase).path = zeros(size(dStates,1),2);
H1 = contactForces(:,1);
V1 = contactForces(:,2);
output(iphase).path(:,1) = atan2(H1,V1);
output(iphase).path(:,2) = Position.footTwo.y;

switch input.auxdata.cost.method
    case 'Work'
        Power = getPower_single(States, Actuators);
        alpha = input.auxdata.cost.smoothing.power;
        output(iphase).integrand = ...
            SmoothAbs(Power.ankleOne,alpha) +...
            SmoothAbs(Power.hip,alpha) + ...
            SmoothAbs(Power.legOne,alpha) + ...
            SmoothAbs(Power.legTwo,alpha);
    case 'Squared'
        Scale = input.auxdata.cost.Scale;
        output(iphase).integrand = ...
            (Actuators(:,1)/Scale.leg).^2 + ...
            (Actuators(:,2)/Scale.leg).^2 + ...
            (Actuators(:,3)/Scale.ankle).^2 + ...
            (Actuators(:,4)/Scale.hip).^2;
        
    otherwise
        error('Unsupported cost function')
end

end