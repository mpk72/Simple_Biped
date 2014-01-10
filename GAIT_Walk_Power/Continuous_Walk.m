function output = Continuous_Walk(input)

%Check MAIN_Walk for index definitions throughout this file

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Phase 1  --  Double Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 1;

%Check MAIN_Walk for index definitions
IdxState = 1:4;
IdxAct = 5:6;
IdxActRate = 1:2;

States = input.phase(iphase).state(:,IdxState);
Actuators = input.phase(iphase).state(:,IdxAct);
Actuator_Rate = input.phase(iphase).control(:,IdxActRate);
Parameters = input.auxdata.dynamics;

[dStates, contactForces] = dynamics_double(States, Actuators, Parameters);
output(iphase).dynamics = [dStates, Actuator_Rate];

output(iphase).path = zeros(size(dStates,1),2);
H1 = contactForces(:,1);
V1 = contactForces(:,2);
H2 = contactForces(:,3);
V2 = contactForces(:,4);
output(iphase).path(:,1) = atan2(H1,V1);
output(iphase).path(:,2) = atan2(H2,V2);

IdxAbsPos = 3:4;
IdxAbsNeg = 5:6;
AbsPos = input.phase(iphase).control(:,IdxAbsPos);
AbsNeg = input.phase(iphase).control(:,IdxAbsNeg);

[cost, path] = costFunc(States, Actuators, Actuator_Rate,...
    input.auxdata, AbsPos, AbsNeg, 'D');
output(iphase).path(:,3:4) = path;
output(iphase).integrand = cost;

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Phase 2  --  Single Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 2;

IdxState = 1:8;
IdxAct = 9:12;
IdxActRate = 1:4;

States = input.phase(iphase).state(:,IdxState);
Actuators = input.phase(iphase).state(:,IdxAct);
Actuator_Rate = input.phase(iphase).control(:,IdxActRate);

[dStates, contactForces] = dynamics_single(States, Actuators, Parameters);
output(iphase).dynamics = [dStates, Actuator_Rate];

output(iphase).path = zeros(size(dStates,1),2);
H1 = contactForces(:,1);
V1 = contactForces(:,2);
output(iphase).path(:,1) = atan2(H1,V1);

Position = getPosVel_single(States);
output(iphase).path(:,2) = ... %Foot height above ground
    Position.footTwo.y - feval(input.auxdata.ground.func,Position.footTwo.x);

IdxAbsPos = 5:8;
IdxAbsNeg = 9:12;
AbsPos = input.phase(iphase).control(:,IdxAbsPos);
AbsNeg = input.phase(iphase).control(:,IdxAbsNeg);

[cost, path] = costFunc(States, Actuators, Actuator_Rate,...
    input.auxdata, AbsPos, AbsNeg, 'S1');
output(iphase).path(:,3:6) = path;
output(iphase).integrand = cost;

end