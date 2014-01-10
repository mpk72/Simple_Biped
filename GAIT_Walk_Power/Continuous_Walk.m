function output = Continuous_Walk(input)

Cost = input.auxdata.cost;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Phase 1  --  Double Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 1;

IdxState = 1:4;
IdxAct = 5:6;
IdxActRate = 1:2;

IdxAbsPos = 3:4;
IdxAbsNeg = 5:6;

States = input.phase(iphase).state(:,IdxState);
Actuators = input.phase(iphase).state(:,IdxAct);
Actuator_Rate = input.phase(iphase).control(:,IdxActRate);
Parameters = input.auxdata.dynamics;

AbsPos = input.phase(iphase).control(:,IdxAbsPos);
AbsNeg = input.phase(iphase).control(:,IdxAbsNeg);

[dStates, contactForces] = dynamics_double(States, Actuators, Parameters);

output(iphase).dynamics = [dStates, Actuator_Rate];

output(iphase).path = zeros(size(dStates,1),2);
H1 = contactForces(:,1);
V1 = contactForces(:,2);
H2 = contactForces(:,3);
V2 = contactForces(:,4);
output(iphase).path(:,1) = atan2(H1,V1);
output(iphase).path(:,2) = atan2(H2,V2);

Power = getPower_double(States, Actuators, Parameters);
PowMat = [Power.legOne, Power.legTwo];
output(iphase).path(:,3:4) = PowMat - (AbsPos-AbsNeg);

switch Cost.method
    case 'Work'
        alpha = Cost.weight.actuator;
        beta = Cost.weight.actuator_rate;
        ScaleA = 1/Cost.scale.leg;
        ScaleB = Cost.scale.timeConst*ScaleA;
        output(iphase).integrand = ...
            sum(AbsPos+AbsNeg,2) + ...  %abs(power)
            alpha*sum((Actuators.*ScaleA).^2,2) +... %torque.^2
            beta*sum((Actuator_Rate.*ScaleB).^2,2);   %dTorque.^2
    otherwise
        error('Unsupported cost function')
end


%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Phase 2  --  Single Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 2;

IdxState = 1:8;
IdxAct = 9:12;
IdxActRate = 1:4;

IdxAbsPos = 5:8;
IdxAbsNeg = 9:12;

States = input.phase(iphase).state(:,IdxState);
Actuators = input.phase(iphase).state(:,IdxAct);
Actuator_Rate = input.phase(iphase).control(:,IdxActRate);

AbsPos = input.phase(iphase).control(:,IdxAbsPos);
AbsNeg = input.phase(iphase).control(:,IdxAbsNeg);

[dStates, contactForces] = dynamics_single(States, Actuators, Parameters);

output(iphase).dynamics = [dStates, Actuator_Rate];
Position = getPosVel_single(States);

output(iphase).path = zeros(size(dStates,1),2);
H1 = contactForces(:,1);
V1 = contactForces(:,2);
output(iphase).path(:,1) = atan2(H1,V1);
output(iphase).path(:,2) = ... %Foot height above ground
    Position.footTwo.y - feval(input.auxdata.ground.func,Position.footTwo.x);

Power = getPower_single(States, Actuators);
PowMat = [Power.legOne, Power.legTwo, Power.ankleOne, Power.hip];
output(iphase).path(:,3:6) = PowMat - (AbsPos-AbsNeg);
switch Cost.method
    case 'Work'
        Nt = size(States,1);
        alpha = Cost.weight.actuator;
        beta = Cost.weight.actuator_rate;
        ScaleA = ones(Nt,1)*(1./[Cost.scale.leg,Cost.scale.leg,Cost.scale.ank,Cost.scale.hip]);
        ScaleB = Cost.scale.timeConst*ScaleA;
        output(iphase).integrand = ...
            sum(AbsPos+AbsNeg,2) + ...  %abs(power)
            alpha*sum((Actuators.*ScaleA).^2,2) +... %torque.^2
            beta*sum((Actuator_Rate.*ScaleB).^2,2);   %dTorque.^2
    otherwise
        error('Unsupported cost function')
end


end