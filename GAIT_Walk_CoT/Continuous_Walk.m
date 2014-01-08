function output = Continuous_Walk(input)

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Phase 1  --  Double Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 1;

IdxState = 1:4;
IdxAbsPos = 5:6;
IdxAbsNeg = 7:8;

IdxControl = 1:2;
IdxAbsRatePos = 3:4;
IdxAbsRateNeg = 5:6;

States = input.phase(iphase).state(:,IdxState);
Actuators = input.phase(iphase).control(:,IdxControl);
Parameters = input.auxdata.dynamics;

AbsPos = input.phase(iphase).state(:,IdxAbsPos);
AbsNeg = input.phase(iphase).state(:,IdxAbsNeg);
AbsRatePos = input.phase(iphase).control(:,IdxAbsRatePos);
AbsRateNeg = input.phase(iphase).control(:,IdxAbsRateNeg);

[dStates, contactForces] = dynamics_double(States, Actuators, Parameters);

output(iphase).dynamics = [dStates, AbsRatePos, AbsRateNeg];

H1 = contactForces(:,1);
V1 = contactForces(:,2);
H2 = contactForces(:,3);
V2 = contactForces(:,4);
output(iphase).path = zeros(size(dStates,1),4);
output(iphase).path(:,1) = atan2(H1,V1);
output(iphase).path(:,2) = atan2(H2,V2);

%Path constraint on integrand:  (THIS IS AN ABSOLUTE VALUE FUNCTION)
Power = getPower_double(States, Actuators, Parameters);
output(iphase).path(:,3) = Power.legOne - (AbsPos(:,1) - AbsNeg(:,1));
output(iphase).path(:,4) = Power.legTwo - (AbsPos(:,2) - AbsNeg(:,2));

switch input.auxdata.cost.method
    case 'Work'
        alpha = input.auxdata.cost.ratePenalty;
        output(iphase).integrand = ...
            AbsPos(:,1) + AbsNeg(:,1) +...
            AbsPos(:,2) + AbsNeg(:,2) +...
            alpha*( AbsRatePos(:,1).^2 + AbsRateNeg(:,1).^2 +...
                    AbsRatePos(:,2).^2 + AbsRateNeg(:,2).^2 );
    otherwise
        error('Invalid cost function');
end

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Phase 2  --  Single Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 2;

IdxState = 1:8;
IdxAbsPos = 9:12;
IdxAbsNeg = 13:16;

IdxControl = 1:4;
IdxAbsRatePos = 5:8;
IdxAbsRateNeg = 9:12;

States = input.phase(iphase).state(:,IdxState);
Actuators = input.phase(iphase).control(:,IdxControl);
Parameters = input.auxdata.dynamics;

AbsPos = input.phase(iphase).state(:,IdxAbsPos);
AbsNeg = input.phase(iphase).state(:,IdxAbsNeg);
AbsRatePos = input.phase(iphase).control(:,IdxAbsRatePos);
AbsRateNeg = input.phase(iphase).control(:,IdxAbsRateNeg);

[dStates, contactForces] = dynamics_single(States, Actuators, Parameters);
Position = getPosVel_single(States);

output(iphase).dynamics = [dStates, AbsRatePos, AbsRateNeg];

output(iphase).path = zeros(size(dStates,1),6);
H1 = contactForces(:,1);
V1 = contactForces(:,2);
output(iphase).path(:,1) = atan2(H1,V1);
output(iphase).path(:,2) = ... %Foot height above ground
    Position.footTwo.y - feval(input.auxdata.ground.func,Position.footTwo.x);

%Path constraint on integrand:  (THIS IS AN ABSOLUTE VALUE FUNCTION)
Power = getPower_single(States, Actuators);
output(iphase).path(:,3) = Power.legOne - (AbsPos(:,1) - AbsNeg(:,1));
output(iphase).path(:,4) = Power.legTwo - (AbsPos(:,2) - AbsNeg(:,2));
output(iphase).path(:,5) = Power.hip - (AbsPos(:,3) - AbsNeg(:,3));
output(iphase).path(:,6) = Power.ankleOne - (AbsPos(:,4) - AbsNeg(:,4));

switch input.auxdata.cost.method
    case 'Work'
        alpha = input.auxdata.cost.ratePenalty;
        output(iphase).integrand = ...
            AbsPos(:,1) + AbsNeg(:,1) +...
            AbsPos(:,2) + AbsNeg(:,2) +...
            AbsPos(:,3) + AbsNeg(:,3) +...
            AbsPos(:,4) + AbsNeg(:,4) +...
            alpha*( AbsRatePos(:,1).^2 + AbsRateNeg(:,1).^2 +...
                    AbsRatePos(:,2).^2 + AbsRateNeg(:,2).^2 +...
                    AbsRatePos(:,3).^2 + AbsRateNeg(:,3).^2 +...
                    AbsRatePos(:,4).^2 + AbsRateNeg(:,4).^2 );
    otherwise
        error('Invalid cost function');
end

end