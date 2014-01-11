function output = Continuous_Walk(input)

%Check MAIN_Walk for index definitions throughout this file

groundFunc = input.auxdata.ground.func;
[~, n1] = feval(groundFunc,0,[]);   %Ground normal for stance foot

%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
%                     Phase 1  --  Double Stance                          %
%~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~%
iphase = 1;

IdxState = 1:4;
IdxAct = 5:6;
IdxActRate = 1:2;

States = input.phase(iphase).state(:,IdxState);
Actuators = input.phase(iphase).state(:,IdxAct);
Actuator_Rate = input.phase(iphase).control(:,IdxActRate);
P_Dyn = input.auxdata.dynamics;

STEP_DIST = input.phase(iphase).parameter;
P_Dyn.x2 = -STEP_DIST;
[P_Dyn.y2, n2] = feval(groundFunc,-STEP_DIST,[]);

[dStates, contactForces] = dynamics_double(States, Actuators, P_Dyn);
output(iphase).dynamics = [dStates, Actuator_Rate];

output(iphase).path = zeros(size(dStates,1),2);
H1 = contactForces(:,1);
V1 = contactForces(:,2);
H2 = contactForces(:,3);
V2 = contactForces(:,4);
output(iphase).path(:,1) = atan2(H1,V1) + n1;
output(iphase).path(:,2) = atan2(H2,V2) + n2;

IdxAbsPos = 3:4;
IdxAbsNeg = 5:6;
AbsPos = input.phase(iphase).control(:,IdxAbsPos);
AbsNeg = input.phase(iphase).control(:,IdxAbsNeg);

P_Cost = input.auxdata.cost;
[cost, path] = costFunc(States, Actuators, Actuator_Rate,...
    P_Dyn, P_Cost, AbsPos, AbsNeg, 'D', STEP_DIST);
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

[dStates, contactForces] = dynamics_single(States, Actuators, P_Dyn);
output(iphase).dynamics = [dStates, Actuator_Rate];

output(iphase).path = zeros(size(dStates,1),2);
H1 = contactForces(:,1);
V1 = contactForces(:,2);
output(iphase).path(:,1) = atan2(H1,V1) + n1;

Position = getPosVel_single(States);
STEP_DIST = input.phase(iphase).parameter;
[~,~,min_height] = feval(groundFunc,Position.footTwo.x,STEP_DIST);
output(iphase).path(:,2) = ... %Foot height above ground
    Position.footTwo.y - min_height;

IdxAbsPos = 5:8;
IdxAbsNeg = 9:12;
AbsPos = input.phase(iphase).control(:,IdxAbsPos);
AbsNeg = input.phase(iphase).control(:,IdxAbsNeg);

[cost, path] = costFunc(States, Actuators, Actuator_Rate,...
    P_Dyn, P_Cost, AbsPos, AbsNeg, 'S1', STEP_DIST);
output(iphase).path(:,3:6) = path;
output(iphase).integrand = cost;

end